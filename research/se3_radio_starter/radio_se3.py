"""Bluetooth/USB joystick -> position target -> upstream SE3 -> MuJoCo.

This program NEVER sends anything to a real aircraft. Bluetooth pairing is done
by the operating system; pygame reads the resulting joystick device.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import sys
import time

import numpy as np

from core import InputGate, MotionSettings, PositionReference, RadioProfile, demo_target

ROOT = Path(__file__).resolve().parent


class Telemetry:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["sim_time_s", "x_m", "y_m", "z_m", "target_x_m", "target_y_m", "target_z_m",
                              "target_yaw_rad", "position_error_m", "motor1", "motor2", "motor3", "motor4"])
        self.next_time = 0.0
        self.previous_time = -1.0

    def write(self, simulation, target: np.ndarray, yaw: float) -> None:
        t = float(simulation.data.time)
        if t < self.previous_time:
            self.next_time = t
        self.previous_time = t
        if t + 1e-9 < self.next_time:
            return
        position = simulation.position
        self.writer.writerow([t, *position, *target, yaw, float(np.linalg.norm(position - target)),
                              *simulation.last_control])
        self.next_time = t + 0.02

    def close(self) -> None:
        self.file.close()


def run_headless(args, simulation, log: Telemetry) -> int:
    from upstream_adapter import source_fingerprints
    seconds = 24.0 if args.seconds is None else args.seconds
    heading = np.array([0., 1., 0.])
    phase_errors = []
    targets = []
    next_checkpoint = 6.0
    max_error = 0.0
    steps = math.ceil(seconds / simulation.DT)
    for _ in range(steps):
        target = demo_target(float(simulation.data.time))
        simulation.step(target, heading)
        error = float(np.linalg.norm(simulation.position - target))
        max_error = max(max_error, error)
        log.write(simulation, target, 0.0)
        if simulation.data.time + 1e-8 >= next_checkpoint and next_checkpoint <= 24:
            phase_errors.append(error)
            targets.append(target.tolist())
            print(f"t={next_checkpoint:.0f}s, position error={error:.6f}m, position={simulation.position}")
            next_checkpoint += 6
    final_error = float(np.linalg.norm(simulation.position - target))
    complete = len(phase_errors) == 4
    passed = bool(complete and max(phase_errors) < 0.10)
    report = {
        "test": "upstream_headless_four_point_tracking", "simulated_seconds": float(simulation.data.time),
        "checkpoint_targets_m": targets, "checkpoint_position_errors_m": phase_errors,
        "maximum_position_error_including_steps_m": max_error, "final_position_error_m": final_error,
        "complete_24s_test": complete, "checkpoint_tolerance_m": 0.10,
        "passed": passed if complete else None,
        "mujoco_version": simulation.mj.__version__, "numpy_version": np.__version__,
        "upstream_sha256": source_fingerprints(simulation.repo),
        "scope": "Simulation only. Does not test pygame, rendering, Bluetooth, or physical hardware.",
    }
    report_path = log.path.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"Report: {report_path}")
    if complete:
        print("PASS" if passed else "FAIL: inspect the CSV and source/model versions before adding the radio.")
    else:
        print("Diagnostic run only: use --seconds 24 or longer for all four checkpoints.")
    return 0 if (passed or not complete) else 1


def run_graphical(args, simulation, log: Telemetry) -> int:
    from pygame_io import Panel, Radio
    import mujoco.viewer

    settings = MotionSettings.load(args.settings)
    reference = PositionReference(settings)
    gate = InputGate()
    panel = Panel("SE3 Radio - Input / Target Monitor")
    key_queue: queue.SimpleQueue[str] = queue.SimpleQueue()

    def viewer_key(keycode: int) -> None:
        if keycode == 256:
            key_queue.put("QUIT")
        elif keycode in (71, 72, 80, 82):
            key_queue.put(chr(keycode))

    try:
        radio = Radio(panel, args.device) if args.mode == "fly" else None
        profile = RadioProfile.load(args.profile) if radio else None
        if radio:
            profile.validate_device(radio.guid, radio.axis_count)
        status = "HOLD: center every axis (throttle too), then press G." if radio else "DEMO: automatic point tracking."
        paused = False
        running = True
        accumulator = 0.0
        last_time = time.perf_counter()
        last_draw = 0.0
        raw = np.zeros(radio.axis_count) if radio else np.zeros(0)
        command = np.zeros(4)
        with mujoco.viewer.launch_passive(simulation.model, simulation.data,
                                          key_callback=viewer_key) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = [0, 0, 0.3]
                viewer.cam.distance = 2.0
                viewer.cam.azimuth = 110
                viewer.cam.elevation = -25
            while running and viewer.is_running():
                loop_start = time.perf_counter()
                elapsed = loop_start - last_time
                last_time = loop_start
                events = panel.events()
                actions = panel.commands(events)
                while not key_queue.empty():
                    actions.append(key_queue.get())
                if radio:
                    readings = radio.read(events)
                    if readings is None:
                        if not gate.disconnected:
                            status = "DISCONNECTED: target frozen. Reconnect and RESTART to use radio."
                            print(status)
                        gate.stop(disconnected=True)
                        reference.hold()
                        command[:] = 0
                    else:
                        raw = readings
                        command = profile.normalize(raw)
                for action in actions:
                    if action == "QUIT":
                        running = False
                    elif action == "G" and radio and not paused:
                        status = gate.toggle(command)
                        if not gate.enabled:
                            reference.hold()
                    elif action == "H" and radio:
                        gate.stop()
                        reference.hold()
                        status = "HOLD: current target frozen. Center axes and G to enable again."
                    elif action == "P":
                        paused = not paused
                        gate.stop()
                        reference.hold()
                        accumulator = 0.0
                        status = "PAUSED: P resumes." if paused else "Resumed. Radio movement still disabled; press G."
                        if not radio and not paused:
                            status = "DEMO resumed."
                    elif action == "R":
                        with viewer.lock():
                            simulation.reset()
                        reference.reset()
                        gate.stop()  # a disconnected session stays latched after reset
                        accumulator = 0.0
                        status = "RESET: original state restored; auto-hover target. Radio disabled."
                if not running:
                    break
                # Avoid a giant target jump after debugger/OS stalls. No stale input catch-up.
                if elapsed > 0.25:
                    accumulator = 0.0
                    gate.stop()
                    reference.hold()
                    status = "Timing stall: input disabled. Center axes and G to continue."
                elif not paused:
                    accumulator += min(elapsed, 0.05)
                with viewer.lock():
                    while not paused and accumulator + 1e-10 >= simulation.DT:
                        if radio:
                            reference.update(command, simulation.DT, gate.enabled)
                        else:
                            reference.position = demo_target(float(simulation.data.time))
                            reference.yaw = 0.0
                        simulation.step(reference.position, reference.heading)
                        log.write(simulation, reference.position, reference.yaw)
                        accumulator -= simulation.DT
                    if loop_start - last_draw >= 1 / 60:
                        simulation.add_target_marker(viewer, reference.position, reference.heading)
                        position = simulation.position
                if args.seconds is not None and simulation.data.time >= args.seconds:
                    break
                if loop_start - last_draw >= 1 / 60:
                    viewer.sync()
                    lines = [status, "", "G=enable/disable radio  H=hold target  R=reset  P=pause  ESC=exit",
                             "Keys work in either window. G requires all four calibrated inputs near zero.", "",
                             f"Simulation t: {simulation.data.time:8.3f}s   {'PAUSED' if paused else 'RUNNING'}",
                             f"Aircraft xyz: {position[0]:+.3f}  {position[1]:+.3f}  {position[2]:+.3f} m",
                             f"Target   xyz: {reference.position[0]:+.3f}  {reference.position[1]:+.3f}  {reference.position[2]:+.3f} m",
                             f"Position error: {np.linalg.norm(position - reference.position):.3f} m",
                             f"Target yaw (from world +Y): {math.degrees(reference.yaw):+.1f} deg CCW", "",
                             "Target marker: large red sphere; heading marker: small yellow sphere.",
                             "World frame: +X right, +Y forward, +Z up. Camera rotation changes the view.", "",
                             "Normalized input: [right, forward, up, yaw_right]",
                             np.array2string(command, precision=3, suppress_small=True),
                             "Motor normalized inputs: " + np.array2string(simulation.last_control, precision=3), "",
                             "No spring on throttle? Manually put it at MIDPOINT to stop height changes.",
                             "HOLD freezes the target, not the aircraft. SE3 keeps tracking that target.",
                             "Target bounds are NOT guaranteed aircraft bounds; no obstacle avoidance.",
                             "Simulator ground truth. No real aircraft connection."]
                    panel.draw("SE3 Position-Target Control", lines)
                    last_draw = loop_start
                time.sleep(max(0.0, 0.01 - (time.perf_counter() - loop_start)))
    finally:
        panel.close()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    for name in ("probe", "calibrate"):
        command = sub.add_parser(name, help="Inspect joystick axes" if name == "probe" else "Calibrate four joystick axes")
        command.add_argument("--device", type=int, default=0)
        command.add_argument("--profile", type=Path, default=ROOT / "radio.json")
    for name in ("demo", "fly"):
        command = sub.add_parser(name, help="No-radio controller check" if name == "demo" else "Radio controls target position")
        command.add_argument("--repo", type=Path, default=ROOT / "upstream")
        command.add_argument("--settings", type=Path, default=ROOT / "settings.json")
        command.add_argument("--seconds", type=float, default=None, help="Stop after this many simulation seconds")
        command.add_argument("--log", type=Path, default=None)
        command.add_argument("--device", type=int, default=0)
        command.add_argument("--profile", type=Path, default=ROOT / "radio.json")
        if name == "demo":
            command.add_argument("--headless", action="store_true", help="No windows or joystick required; defaults to 24s")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if getattr(args, "seconds", None) is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    try:
        if args.mode in ("probe", "calibrate"):
            from pygame_io import probe, calibrate
            (probe if args.mode == "probe" else calibrate)(args.device, args.profile)
            return 0
        from upstream_adapter import UpstreamSimulation, source_fingerprints
        simulation = UpstreamSimulation(args.repo)
        path = args.log or ROOT / "logs" / f"{args.mode}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        log = Telemetry(path)
        provenance = {"mujoco_version": simulation.mj.__version__, "numpy_version": np.__version__,
                      "upstream_sha256": source_fingerprints(simulation.repo), "mode": args.mode}
        path.with_suffix(".source.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
        try:
            return run_headless(args, simulation, log) if getattr(args, "headless", False) else run_graphical(args, simulation, log)
        finally:
            log.close()
            print(f"Telemetry: {path}")
    except KeyboardInterrupt:
        print("Stopped by user.")
        return 130
    except (OSError, ValueError, RuntimeError, ImportError, KeyError, AttributeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        print("See README_zh-TW.md for installation and troubleshooting.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
