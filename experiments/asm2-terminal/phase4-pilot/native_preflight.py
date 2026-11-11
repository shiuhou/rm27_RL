"""Bounded Phase 4 native replay checks. Reads only Phase 3 development states."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

import mujoco
import numpy as np

from rm27_asm2.asset import ROOT, digest, read_json, write_result
from rm27_asm2.contact_native import NativeContact
from rm27_asm2.contact_task import ContactScorer, load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.handover import support_points
from rm27_asm2.handover_curriculum import native_verifier
from rm27_asm2.handover_isaac import load_tape, terminal_tape
from rm27_asm2.native import records
from rm27_asm2.native_resolution import _before


RUN = ROOT / ".agents/runs/asm2-handover/phase3-v1"
CASES = ("dev_center", "dev_near_slow")
LOOKBACK_MS = 45.0
EXTENSION_S = 0.025


def development_resets(plant):
    manifest_path = RUN / "reset-dataset/manifest.json"
    manifest = read_json(manifest_path)
    split = manifest["splits"]["development"]
    path = manifest_path.parent / split["path"]
    if (digest(path) != split["sha256"] or manifest["training_allowed"] is not False
            or manifest["profile_sha256"] != plant.profile.profile_sha256
            or manifest["contact_contract_sha256"] != plant.spec.contract_sha256):
        raise ValueError("The development reset archive does not match the native task")
    with np.load(path, allow_pickle=False) as data:
        terminal = data["phase"] == "terminal"
        states = data["states"][terminal]
        thrusts = data["motor_thrust_N"][terminal]
        targets = data["motor_target_N"][terminal]
        cases = data["case_ids"][terminal]
        frames = data["source_frame"][terminal]
    if len(states) != 80:
        raise ValueError("Expected the frozen 80 development terminal resets")
    verify = native_verifier(plant)
    snapshots = {}
    for case, frame, state, thrust, target in zip(cases, frames, states, thrusts, targets):
        status = verify(state, thrust, target)
        if not status["passed"]:
            raise ValueError(f"Invalid native terminal reset {case} at frame {frame}: {status}")
        snapshots[(str(case), int(frame))] = state, thrust, target
    if len(snapshots) != len(states):
        raise ValueError("Duplicate development terminal resets")
    return snapshots, dict(archive_sha256=digest(path), manifest_sha256=digest(manifest_path),
                           verified_exact_and_contact_free=len(snapshots))


def negative_control(plant):
    spec = plant.spec
    body, _ = support_points(plant)
    rotation = spec.target_rotation @ np.diag([1., -1., -1.])
    normal = spec.target_normal
    clearance = float(spec.config["field_fixture"]["clearance_shift_m"])
    position = spec.target_center + (clearance - np.min(body @ (rotation.T @ normal))) * normal
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, rotation.ravel())
    velocity = -spec.config["field_fixture"]["normal_approach_speed_m_s"] * normal
    state = np.r_[position, quaternion, velocity, np.zeros(3)]
    zeros = np.zeros(4)
    fixture = dict(name="field_inverted_body", fixture="registered_field",
                   position=position.tolist(), rotation=rotation.tolist(),
                   velocity=velocity.tolist(), omega=[0., 0., 0.], steps=20,
                   normal_reference_world=normal.tolist(), expected_groups=["body"])
    return (state, zeros, zeros), fixture


def replay(plant, snapshot, tape, source_end_s, extension_s):
    state, thrust, targets = snapshot
    data = plant.data
    mujoco.mj_resetData(plant.model, data)
    data.qpos[:], data.qvel[:], data.act[:], data.ctrl[:] = state[:7], state[7:], thrust, targets
    mujoco.mj_forward(plant.model, data)
    np.testing.assert_array_equal(plant.state(), state)
    np.testing.assert_array_equal(data.act, thrust)
    np.testing.assert_array_equal(data.ctrl, targets)
    plant.tick, plant.pre_state = 0, plant.state()
    scorer = ContactScorer(plant.spec, dt=plant.dt)
    initial_rows = records(plant.model, data, plant.spec.entries, 0, _before(data))
    if any(row["distance_m"] <= 0 or row["normal_force_N"] > 0 for row in initial_rows):
        raise ValueError("Replay initial state contains active contact")
    result = scorer.observe(initial_rows, plant.pre_state, 0)
    steps = math.ceil((source_end_s + extension_s) / plant.dt - 1e-9)
    control_ticks = round(plant.profile.policy_dt_s / plant.dt)
    saturation, action_updates = 0, 0
    alpha = -math.expm1(-plant.dt / plant.profile.motor_time_constant_s)
    max_motor_residual = 0.
    first_applied_target = None
    for tick in range(steps):
        if result is not None:
            break
        if tick % control_ticks == 0:
            # A reset lies exactly on a policy boundary. Restore the previous
            # held targets, then write the new command before the first solve.
            command = tape[min(tick // control_ticks, len(tape)-1)]
            saturation += plant.set_ctbr(command)
            action_updates += 1
            if tick == 0:
                first_applied_target = data.ctrl.copy().tolist()
        expected_motor = data.act + alpha * (data.ctrl - data.act)
        rows = plant.step_physics()
        max_motor_residual = max(max_motor_residual, float(np.max(np.abs(data.act - expected_motor))))
        result = scorer.observe(rows, plant.pre_state, plant.tick)
    if max_motor_residual > 2e-12:
        raise ValueError("Native motor update timing disagrees with filterexact")
    result = result or scorer.finish()
    for key in ("first_touch", "first_loaded"):
        event = result[key]
        if event:
            points = event.pop("records")
            speeds = [point["normal_approach_speed_m_s"] for point in points]
            event["normal_speed_range_m_s"] = [min(speeds), max(speeds)]
    return dict(dt_s=plant.dt, source_end_s=source_end_s, extra_window_s=extension_s,
                final_interval_end_s=plant.tick*plant.dt, requested_interval_end_s=steps*plant.dt,
                policy_command_updates=action_updates, first_applied_target_N=first_applied_target,
                initial_contact_reports=len(initial_rows), max_filterexact_residual_N=max_motor_residual,
                mixer_saturated_controls=saturation, result=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose an unused evidence path")
    start = time.monotonic()
    profile, spec = load_profile(), load_contact_spec()
    plant = NativeContact(profile, spec)
    snapshots, reset_status = development_resets(plant)
    negative, negative_fixture = negative_control(plant)
    print("NATIVE_NEGATIVE_FIXTURE", negative_fixture, flush=True)
    selected = {}
    for case in CASES:
        path = RUN / f"{case}.npz"
        tape, metadata = terminal_tape(path, load_tape(path, profile), LOOKBACK_MS)
        frame = metadata["source_frame"]
        snapshot = snapshots[(case, frame)]
        np.testing.assert_array_equal(snapshot[0], tape[0][0])
        np.testing.assert_array_equal(snapshot[1], tape[2][0])
        with np.load(path, allow_pickle=False) as trace:
            np.testing.assert_array_equal(snapshot[2], trace["motor_target_N"][frame, 0])
        selected[case] = dict(trace_sha256=digest(path), reset=metadata,
                              initial_state=snapshot[0].tolist(), initial_motor_thrust_N=snapshot[1].tolist(),
                              initial_held_target_N=snapshot[2].tolist(), measurements=[])
        for dt in (spec.dt, spec.config["comparison_dt_s"]):
            plant.dt = plant.model.opt.timestep = dt
            for extension in (0., EXTENSION_S):
                measurement = replay(plant, snapshot, tape[1][:, 0], float(tape[3][-1]), extension)
                selected[case]["measurements"].append(measurement)
                print("NATIVE_POSITIVE", case, dt, extension, measurement["result"]["reason"],
                      measurement["final_interval_end_s"], flush=True)
    measurements = []
    for dt in (spec.dt, spec.config["comparison_dt_s"]):
        plant.dt = plant.model.opt.timestep = dt
        measurement = replay(plant, negative, np.zeros((1, 4)), 0.020, 0.)
        measurements.append(measurement)
        print("NATIVE_NEGATIVE", dt, measurement["result"]["reason"],
              measurement["final_interval_end_s"], flush=True)
    write_result(args.output, dict(
        scope="Phase 4 diagnostic only. Extended window repeats last recorded CTBR with live rate feedback.",
        reset_status=reset_status, model=plant.metadata, positives=selected,
        negative=dict(fixture=negative_fixture, initial_state=negative[0].tolist(),
                      initial_motor_thrust_N=negative[1].tolist(), initial_held_target_N=negative[2].tolist(),
                      ctbr_N_rad_s=[0., 0., 0., 0.], measurements=measurements),
        diagnostic_seconds=time.monotonic()-start, training_transitions=0))


if __name__ == "__main__":
    main()
