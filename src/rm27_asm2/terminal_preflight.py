"""Verify complete ASM2 terminal resets and actual collider events in Isaac."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .contact_task import load_contact_spec
from .flight import allocate_ctbr, load_profile
from .terminal_isaac import TerminalSim

CONFIG = ROOT / "configs/rm27_asm2_terminal_pilot.json"
RUN = ROOT / ".agents/runs/asm2-terminal/phase4-pilot"


def load_experiment():
    config = read_json(CONFIG)
    authorization = dict(training_allowed_after_preflight=True, candidates=1, max_transitions=3_000_000,
                         max_learning_s=1800, max_preflight_s=1800)
    if config["schema"] != "rm27_asm2_terminal_pilot_v1" or config["authorization"] != authorization:
        raise ValueError("Unexpected ASM2 pilot authorization")
    pins = {}
    for name, pin in config["pins"].items():
        path = ROOT / pin["path"]
        if digest(path) != pin["sha256"]:
            raise ValueError(f"Pilot source changed: {name}")
        pins[name] = path
    return config, pins


def load_development(manifest):
    split = read_json(manifest)["splits"]["development"]
    path = manifest.parent / split["path"]
    if digest(path) != split["sha256"]:
        raise ValueError("Development reset archive changed")
    with np.load(path, allow_pickle=False) as rows:
        mask = rows["phase"] == "terminal"
        data = {key: rows[key][mask] for key in ("states", "motor_thrust_N", "motor_target_N", "source_frame", "case_ids")}
    if len(data["states"]) != 80:
        raise ValueError("Expected all 80 development terminal resets")
    return data


def cases(config, pins, native, spec):
    dev = load_development(pins["resets"])
    cases = []
    for name, frame in config["preflight"]["positive_source_frames"].items():
        at, = np.flatnonzero((dev["case_ids"] == name) & (dev["source_frame"] == frame))
        with np.load(pins[name], allow_pickle=False) as tape:
            state, thrust, target = [dev[k][at] for k in ("states", "motor_thrust_N", "motor_target_N")]
            for key, value in (("states", state), ("motor_thrust_N", thrust), ("motor_target_N", target)):
                np.testing.assert_array_equal(tape[key][frame, 0], value)
            command = tape["ctbr_N_rad_s"][frame:, 0]
            times = tape["sample_times_s"]
            source_window = float(times[-1]-times[frame])
        cases.append(dict(name=name, state=state, thrust=thrust, target=target, tape=command,
                          source_window_s=source_window, expected="DESIGNATED_LOADED"))
    fixture = native["negative"]
    cases.append(dict(name="field_inverted_body", state=np.asarray(fixture["initial_state"]),
                      thrust=fixture["initial_motor_thrust_N"], target=fixture["initial_held_target_N"],
                      tape=[fixture["ctbr_N_rad_s"]], source_window_s=.020, expected="FORBIDDEN_AIRFRAME_FIRST_TOUCH"))
    clear = cases[-1]["state"].copy()
    clear[:3] += .25*spec.target_normal
    cases.append(dict(cases[-1], name="clear_no_touch", state=clear, expected="NO_SIGNED_TOUCH"))
    return cases


def batch(sim, cases, assignment, extension_s):
    import torch

    rows = [cases[i] for i in assignment]
    state, thrust, targets = [sim.tensor(np.asarray([r[key] for r in rows])) for key in ("state", "thrust", "target")]
    ids = torch.arange(sim.worlds, device=sim.device)
    sim.events.clear()
    sim.details.clear()
    sim.restore(ids, state, thrust, targets)
    pose, R = sim.observe()
    reset_error = float((pose-state).abs().max())
    motor_readback = bool(torch.equal(sim.thrust, thrust) and torch.equal(sim.targets, targets))
    source_steps = [math.ceil(r["source_window_s"]/sim.dt - 1e-9) for r in rows]
    start = sim.tick
    saturation = torch.zeros(sim.worlds, dtype=torch.int64, device=sim.device)
    free_momentum = torch.zeros(sim.worlds, device=sim.device)
    valid_pre_event = torch.ones(sim.worlds, dtype=torch.bool, device=sim.device)
    first_target_error = first_lag_error = first_interval_lag_error = None
    max_steps = max(source_steps)+round(extension_s/sim.dt)
    for index in range(max_steps):
        if index % sim.substeps == 0:
            u = sim.tensor(np.asarray([r["tape"][min(index//sim.substeps, len(r["tape"])-1)] for r in rows]))
            clipped = sim.set_commands(u, pose[:, 10:13])
            saturation += clipped & valid_pre_event
            if index == 0:
                expected, _ = allocate_ctbr(sim.profile, u.cpu().numpy(), pose[:, 10:13].cpu().numpy())
                first_target_error = float(np.max(np.abs(sim.targets.cpu().numpy()-expected)))
                first_target = sim.targets.clone()
        pose, R = sim.physics_step(pose, R)
        if index in (0, sim.substeps-1):
            alpha = -math.expm1(-(index+1)*sim.dt/sim.profile.motor_time_constant_s)
            error = float((sim.thrust-(thrust+alpha*(first_target-thrust))).abs().max())
            if index == 0:
                first_lag_error = error
            else:
                first_interval_lag_error = error
        if sim.signed:
            valid_pre_event[list(sim.signed)] = False
        free_momentum = torch.maximum(free_momentum, torch.where(valid_pre_event, sim.latest_normal_dp.abs(), 0.))
    momentum, saturation = free_momentum.tolist(), saturation.tolist()
    results = []
    for index, row in enumerate(rows):
        detail = sim.details.get(index)
        signed_tick = None if not detail else detail["tick"]-start
        reason = sim.events.get(index, "NO_SIGNED_TOUCH")
        results.append(dict(name=row["name"], world=index, reason=reason, expected=row["expected"],
                            signed_tick=signed_tick, original_bracket_steps=source_steps[index],
                            signed_in_original_bracket=signed_tick is not None and signed_tick <= source_steps[index],
                            contact=detail, saturated_control_steps=saturation[index],
                            max_pre_signed_abs_normal_momentum_Ns=momentum[index]))
    return dict(reset_max_abs_error=reset_error, exact_motor_readback=motor_readback,
                first_target_error_N=first_target_error, first_lag_error_N=first_lag_error,
                first_control_lag_error_N=first_interval_lag_error, wall_clock_physics_steps=max_steps,
                finite=bool(torch.isfinite(pose).all()), rows=results)


def run(layout, output, trim_development=False):
    config, pins = load_experiment()
    settings = config["preflight"]
    native_path = RUN / "native-preflight.json"
    native = read_json(native_path)
    profile, spec = load_profile(), load_contact_spec()
    controls = cases(config, pins, native, spec)
    sim = TerminalSim(settings["worlds"], layout, profile, spec, diagnostic=True)
    if trim_development:
        # Matched follow-up for unexpected body-first events during the pilot.
        # These are the same 24 rows selected by the native development suite.
        dev = load_development(pins["resets"])
        rows = []
        for name in sorted(set(dev["case_ids"])):
            for at in np.flatnonzero(dev["case_ids"] == name)[[0, 4, 9]]:
                rows.append(dict(name=f"{name}:{dev['source_frame'][at]}", state=dev["states"][at],
                                 thrust=dev["motor_thrust_N"][at], target=dev["motor_target_N"][at],
                                 tape=[[profile.hover_thrust_N, 0., 0., 0.]],
                                 source_window_s=config["horizon_s"], expected="DESIGNATED_LOADED"))
        batches = [batch(sim, rows, np.arange(i, i+sim.worlds) % len(rows), 0.)
                   for i in range(0, len(rows), sim.worlds)]
        write_result(output, dict(config_sha256=digest(CONFIG), layout=layout, physics_dt_s=sim.dt,
                                  diagnostic="frozen development trim replay", batches=batches, training_transitions=0))
        return 0
    assignment = np.arange(sim.worlds) % len(controls)
    started = time.perf_counter()
    batches = [batch(sim, controls, mapping, settings["extension_ms"]*.001)
               for mapping in (assignment if layout == 0 else assignment[::-1],
                               assignment[::-1] if layout == 0 else assignment)]
    valid = all(
        b["finite"] and b["exact_motor_readback"] and b["reset_max_abs_error"] < 1e-6
        and max(b["first_target_error_N"], b["first_lag_error_N"], b["first_control_lag_error_N"]) < 1e-6
        and all(r["reason"] == r["expected"] for r in b["rows"])
        for b in batches
    )
    write_result(output, dict(config_sha256=digest(CONFIG), native_sha256=digest(native_path),
                             physics_dt_s=sim.dt, control_dt_s=profile.policy_dt_s, world_count=sim.worlds,
                             layout=layout, tensor_view_device=str(sim.view_device), asset_sha256=sim.asset_sha256,
                             batches=batches, passed=valid, time_s=time.perf_counter()-started, training_transitions=0))
    for b in batches:
        print("ASM2_PREFLIGHT", layout, sorted({(r["name"], r["reason"], r["signed_tick"]) for r in b["rows"]}),
              flush=True)
    return 0 if valid else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=int, choices=(0, 1), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trim-development", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose an unused preflight evidence path")
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    try:
        code = run(args.layout, args.output, args.trim_development)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
