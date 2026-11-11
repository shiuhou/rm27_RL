"""Evaluate portable ASM2 terminal actors on the frozen native contact task."""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from contextlib import nullcontext
from itertools import count
import math
import multiprocessing
from pathlib import Path
import time

import mujoco
import numpy as np

from simpleflight_lab.stage1_native_eval import NativeActor
from .asset import ROOT, digest, read_json, write_result
from .contact_native import NativeContact
from .contact_task import ContactScorer, load_contact_spec
from .contact_witness import target_mesh
from .flight import load_profile
from .flight_suite import flight_settings, rotation_matrix
from .handover import CONFIG as HANDOVER_CONFIG, MovingReference, flight_limits
from .native import records


CONFIG = ROOT / "configs/rm27_asm2_terminal_pilot.json"
TRACK_WEIGHTS = ROOT / ".agents/runs/asm2-flight/ver2-weights.npz"


def load_evaluation_inputs(experiment=CONFIG):
    config = read_json(experiment)
    if config["schema"] not in ("rm27_asm2_terminal_pilot_v1", "rm27_asm2_terminal_recovery_v1"):
        raise ValueError("Unknown ASM2 terminal experiment schema")
    profile, spec = load_profile(), load_contact_spec()
    manifest_pin = config["pins"]["resets"]
    manifest_path = ROOT / manifest_pin["path"]
    if digest(manifest_path) != manifest_pin["sha256"]:
        raise ValueError("Frozen Phase 3 reset manifest changed")
    manifest = read_json(manifest_path)
    if (manifest["training_allowed"] is not False or manifest["restoration"]["rejected"]
            or manifest["profile_sha256"] != profile.profile_sha256
            or manifest["contact_contract_sha256"] != spec.contract_sha256
            or manifest["physics_dt_s"] != spec.dt
            or manifest["control_dt_s"] != profile.policy_dt_s):
        raise ValueError("Reset bank does not match the frozen native flight and contact task")
    horizons = ["horizon_s", "handover_horizon_s"]
    if config["schema"] == "rm27_asm2_terminal_recovery_v1":
        horizons.append("braking_horizon_s")
    for key in horizons:
        steps = config[key] / profile.policy_dt_s
        if not math.isfinite(steps) or steps <= 0. or not math.isclose(steps, round(steps)):
            raise ValueError(f"Expected a positive whole number of policy intervals for {key}")
    goal = spec.target_center + (-spec.patch_face_z_m-config["goal_inset_m"])*spec.target_normal
    return config, profile, spec, manifest_path, manifest, goal


def load_starts(directory, pin, *, expected_count, profile):
    path = directory / pin["path"]
    if digest(path) != pin["sha256"]:
        raise ValueError(f"Reset archive changed: {path}")
    with np.load(path, allow_pickle=False) as archive:
        data = dict(archive)
    for key, width in (("states", 13), ("motor_thrust_N", 4), ("motor_target_N", 4)):
        if data[key].shape != (expected_count, width) or not np.isfinite(data[key]).all():
            raise ValueError(f"Invalid reset array {key} in {path}")
    if (float(data["dt"]) != profile.policy_dt_s
            or any(data[key].shape != (expected_count,) for key in (
                "phase", "case_ids", "source_frame", "sample_times_s"))
            or not np.allclose(np.linalg.norm(data["states"][:, 3:7], axis=1), 1., atol=1e-8, rtol=0.)
            or any(np.any((data[key] < 0.) | (data[key] > profile.max_rotor_thrust_N))
                   for key in ("motor_thrust_N", "motor_target_N"))):
        raise ValueError(f"Invalid reset pose, actuator state or timing in {path}")
    return data


def load_controller(weights_path, expected_checkpoint_sha256, profile, goal):
    with np.load(weights_path, allow_pickle=False) as archive:
        if archive["checkpoint_sha256"].item() != expected_checkpoint_sha256:
            raise ValueError(f"Portable actor is not from the expected checkpoint: {weights_path}")
        layers = NativeActor.layers(archive, 45)

    def command(state):
        features = np.r_[np.tile(goal-state[:3], 10), state[7:10],
                         rotation_matrix(state[3:7]).T.ravel(), state[10:13]].astype(np.float32)
        action = np.tanh(NativeActor.mean(features[None, :], layers))[0]
        return np.r_[profile.hover_thrust_N*(1.+.5*action[3]), 2.*action[:3]]

    return command, dict(checkpoint_sha256=expected_checkpoint_sha256, weights_sha256=digest(weights_path))


def restore(plant, state, motor_thrust, motor_target):
    data = plant.data
    mujoco.mj_resetData(plant.model, data)
    data.qpos[:], data.qvel[:] = state[:7], state[7:]
    data.act[:], data.ctrl[:] = motor_thrust, motor_target
    mujoco.mj_forward(plant.model, data)
    plant.tick, plant.pre_state = 0, plant.state()
    if (not np.array_equal(plant.pre_state, state) or not np.array_equal(data.act, motor_thrust)
            or not np.array_equal(data.ctrl, motor_target)
            or not np.isfinite(data.qacc).all() or any(w.number for w in data.warning)):
        raise RuntimeError("Exact native state and actuator restore failed")
    if not data.ncon:
        return []
    before = state[:3], state[7:10], rotation_matrix(state[3:7]) @ state[10:13]
    return records(plant.model, data, plant.spec.entries, 0, before)


def rollout(plant, state, motors, targets, command, horizon, limits):
    scorer = ContactScorer(plant.spec)
    initial = restore(plant, state, motors, targets)
    result = scorer.observe(initial, plant.pre_state, 0)
    states, times, commands = [state], [0.], []
    saturated = 0
    safety = flight_limits(state, plant.spec, limits)
    substeps = round(plant.profile.policy_dt_s / plant.dt)
    for _ in range(round(horizon / plant.profile.policy_dt_s)):
        if result is not None or safety:
            break
        ctbr = command(states[-1])
        saturated += int(plant.set_ctbr(ctbr))
        commands.append(ctbr)
        for _ in range(substeps):
            rows = plant.step_physics()
            result = scorer.observe(rows, plant.pre_state, plant.tick)
            if result is not None:
                break
        states.append(plant.state())
        times.append(plant.tick*plant.dt)
        safety = flight_limits(states[-1], plant.spec, limits)
    result = result or scorer.finish()
    return dict(qualified=bool(result["qualified"] and safety is None),
                reason=safety or result["reason"], safety_failure=safety,
                saturation_controls=saturated, controls=len(commands), end_s=times[-1],
                contact=result), (np.asarray(states), np.asarray(times), np.asarray(commands))


def save_replay(path, name, trace, result, goal, mesh, dt):
    states, times, commands = trace
    contact = result["contact"]["first_touch"]
    point = np.asarray(contact["records"][0]["position_world_m"]) if contact else np.empty(0)
    with path.open("xb") as stream:
        np.savez_compressed(stream, names=np.array([name]), states=states[:, None, :], dt=dt,
                            sample_times_s=times, reference=np.broadcast_to(goal, (len(times), 1, 3)),
                            ctbr_N_rad_s=commands[:, None, :], contact_point_m=point,
                            target_vertices_m=mesh[0], target_faces=mesh[1],
                            contact_outcome=result["reason"])


def development_indices(data):
    terminal = np.flatnonzero(data["phase"] == "terminal")
    cases = data["case_ids"][terminal]
    if len(terminal) != 80 or len(set(cases)) != 8:
        raise ValueError("Expected the frozen eight-witness, 80-reset development bank")
    # Fixed before selection. Earliest, middle and latest snapshot per witness.
    indices = []
    for name in sorted(set(cases)):
        rows = terminal[cases == name]
        if len(rows) != 10:
            raise ValueError("Expected ten terminal snapshots per development witness")
        indices.extend(rows[[0, 4, 9]])
    return indices


def recovery_indices(data, selection_cases):
    """Select the prospectively fixed development braking cases in full."""
    if len(set(selection_cases)) != 3:
        raise ValueError("Recovery requires three distinct development selection cases")
    indices = np.flatnonzero((data["phase"] == "braking") & np.isin(data["case_ids"], selection_cases))
    if len(indices) != 36 or any(sum(data["case_ids"][indices] == case) != 12 for case in selection_cases):
        raise ValueError("Expected all 12 braking resets for each selected case")
    return indices


def load_deterministic(directory, manifest, populations, profile, spec):
    """Reconstruct the Phase 3 feedback controller, with no command-tape input."""
    handover_config = read_json(HANDOVER_CONFIG)
    normal_speed = handover_config["terminal_normal_speed_m_s"]
    names = sorted({str(data["case_ids"][i]) for _, data, indices in populations for i in indices})
    references, pins = {}, {}
    for name in names:
        source = manifest["cases"][name]
        trace, report = (directory / source[key]["path"] for key in ("trace", "report"))
        if any(digest(path) != source[key]["sha256"] for key, path in (("trace", trace), ("report", report))):
            raise ValueError(f"Phase 3 witness source changed: {name}")
        report_data = read_json(report)
        gate, selected = report_data["handover_frame"], report_data["selected_preview"]
        if selected is None or gate != source["handover_frame"]:
            raise ValueError(f"No valid deterministic reference at the source handover: {name}")
        with np.load(trace, allow_pickle=False) as archive:
            start = archive["states"][gate, 0]
            thrust = archive["motor_thrust_N"][gate, 0]
            target = archive["motor_target_N"][gate, 0]
        reference = MovingReference(profile, spec, start, thrust, target,
                                    selected["duration_s"], selected["terminal_depth_m"], normal_speed)
        references[name] = reference, gate
        pins[name] = {key: source[key]["sha256"] for key in ("trace", "report")}
    return references, dict(kind="closed_loop_phase3_reference", handover_config_sha256=digest(HANDOVER_CONFIG),
                            sources=pins)


def summary(rows):
    point_speeds = [max(p["normal_approach_speed_m_s"] for p in row["contact"]["first_touch"]["records"])
                    for row in rows if row["qualified"]]
    return dict(episodes=len(rows), qualified=sum(row["qualified"] for row in rows),
                safety_failures=sum(row["safety_failure"] is not None for row in rows),
                saturation_controls=sum(row["saturation_controls"] for row in rows),
                controls=sum(row["controls"] for row in rows),
                mean_worst_point_normal_speed_m_s=float(np.mean(point_speeds)) if point_speeds else None,
                max_worst_point_normal_speed_m_s=max(point_speeds, default=None),
                mean_abs_worst_point_speed_error_to_0p060_m_s=(
                    float(np.mean(np.abs(np.asarray(point_speeds)-.060))) if point_speeds else None),
                reasons=dict(Counter(row["reason"] for row in rows)))


def run_episode(plant, controllers, horizons, limits, need_trace, task):
    mode, population, data, idx, source_touch = task
    phase, case, frame = str(data["phase"][idx]), str(data["case_ids"][idx]), int(data["source_frame"][idx])
    lead_s = source_touch - float(data["sample_times_s"][idx]) if source_touch is not None else None
    horizon = horizons.get(phase, horizons["handover"])
    command = controllers[mode][0]
    if mode == "deterministic":
        reference, gate = command[case]
        if frame < gate:
            raise ValueError(f"Reset predates the deterministic reference: {case}:{frame}")
        reference_tick = count(frame-gate)
        command = lambda state: reference.command(state, next(reference_tick))
    result, trace = rollout(plant, data["states"][idx], data["motor_thrust_N"][idx],
                            data["motor_target_N"][idx], command, horizon, limits)
    result.update(population=population, phase=phase, case=case,
                  source_frame=frame, source_contact_lead_s=lead_s, horizon_s=horizon)
    if mode == "deterministic":
        commands = trace[2].reshape(-1, 4)
        thrust_ratio = commands[:, 0] / plant.profile.hover_thrust_N
        result["outside_actor_action_map_controls"] = int(np.count_nonzero(
            (thrust_ratio < .5) | (thrust_ratio > 1.5) | np.any(np.abs(commands[:, 1:]) > 2., axis=1)))
    return result, trace if need_trace and (phase in ("terminal", "braking") or population == "reached_handover") else None


def init_worker(plant, *args):
    # The frozen MjModel is shared copy-on-write by fork. Every process owns its MjData.
    global _WORKER_ARGS
    plant.data = mujoco.MjData(plant.model)
    _WORKER_ARGS = plant, *args


def worker_episode(task):
    return run_episode(*_WORKER_ARGS, task)


def evaluation(args):
    started = time.monotonic()
    config, profile, spec, manifest_path, manifest, goal = load_evaluation_inputs(args.experiment)
    if args.suite == "recovery-development" and config["schema"] != "rm27_asm2_terminal_recovery_v1":
        raise ValueError("Recovery development requires the recovery experiment config")
    controllers = {}
    if args.actor:
        controllers["candidate"] = load_controller(args.actor, digest(args.checkpoint), profile, goal)
    if args.controls:
        track_pin = flight_settings()["ver2_track_checkpoint"]
        if digest(ROOT / track_pin["path"]) != track_pin["sha256"]:
            raise ValueError("Frozen Track checkpoint changed")
        controllers["track"] = load_controller(TRACK_WEIGHTS, track_pin["sha256"], profile, goal)
        trim = np.array([profile.hover_thrust_N, 0., 0., 0.])
        controllers["trim"] = (lambda _: trim, {"ctbr_N_rad_s": trim.tolist()})

    directory = manifest_path.parent
    split = "development" if args.suite == "recovery-development" else args.suite
    bank = load_starts(directory, manifest["splits"][split],
                       expected_count=280 if split == "development" else 105, profile=profile)
    if args.suite == "development":
        populations = [("development", bank, development_indices(bank))]
    elif args.suite == "recovery-development":
        populations = [(args.suite, bank, recovery_indices(bank, config["selection_cases"]))]
    else:
        handovers = load_starts(directory, manifest["heldout_handover"], expected_count=4, profile=profile)
        populations = [("heldout", bank, range(105)), ("reached_handover", handovers, range(4))]
    if args.deterministic:
        controllers["deterministic"] = load_deterministic(directory, manifest, populations, profile, spec)

    plant = NativeContact(profile, spec)
    limits = read_json(HANDOVER_CONFIG)["flight_envelope"]
    mesh = None
    if args.replay_dir:
        args.replay_dir.mkdir(parents=True, exist_ok=False)
        mesh = target_mesh(plant, spec.legal_targets)

    results = {}
    horizons = dict(terminal=config["horizon_s"], handover=config["handover_horizon_s"],
                    braking=config.get("braking_horizon_s", config["handover_horizon_s"]))
    worker_args = plant, controllers, horizons, limits, mesh is not None
    pool = nullcontext() if args.workers == 1 else ProcessPoolExecutor(
        max_workers=args.workers, mp_context=multiprocessing.get_context("fork"),
        initializer=init_worker, initargs=worker_args)
    with pool as executor:
        for mode, (_, provenance) in controllers.items():
            rows, replays = [], {}
            tasks = ((mode, population, data, idx,
                      manifest["cases"][str(data["case_ids"][idx])].get("first_contact_sample_s"))
                     for population, data, indices in populations for idx in indices)
            outcomes = (executor.map(worker_episode, tasks) if executor else
                        (run_episode(*worker_args, task) for task in tasks))
            for result, trace in outcomes:
                rows.append(result)
                label = result["phase"] if result["phase"] in ("terminal", "braking") else result["population"]
                status = "qualified" if result["qualified"] else "failed"
                key = f"{label}-{status}"
                if trace is not None and len(trace[0]) >= 2 and key not in replays:
                    path = args.replay_dir / f"{mode}-{key}.npz"
                    name = f"{mode}-{result['case']}-{result['source_frame']}"
                    save_replay(path, name, trace, result, goal, mesh, profile.policy_dt_s)
                    replays[key] = {"path": str(path), "sha256": digest(path)}

            buckets = {}
            for row in rows:
                label = (row["phase"] if row["population"] != "reached_handover" else "reached_handover")
                buckets.setdefault(label, []).append(row)
            lead_bins = ((0., .005), (.005, .025), (.025, math.inf))
            cohorts = {label: summary(group) for label, group in buckets.items()}
            results[mode] = dict(actor=provenance, cohorts=cohorts, replays=replays, episodes=rows)
            if "terminal" in buckets:
                results[mode]["terminal_source_lead"] = {
                    label: summary([row for row in buckets["terminal"] if lo <= row["source_contact_lead_s"] < hi])
                    for label, (lo, hi) in zip(("under_5_ms", "5_to_25_ms", "25_ms_or_more"), lead_bins)}
            if "braking" in buckets:
                results[mode]["braking_source_lead"] = {
                    label: summary([row for row in buckets["braking"] if lo <= row["source_contact_lead_s"] < hi])
                    for label, (lo, hi) in zip(("under_100_ms", "100_to_250_ms", "250_ms_or_more"),
                                              ((0., .1), (.1, .25), (.25, math.inf)))}
            print("ASM2_TERMINAL_NATIVE", args.suite, mode,
                  {label: f"{row['qualified']}/{row['episodes']}" for label, row in cohorts.items()},
                  flush=True)
    write_result(args.output, dict(schema="rm27_asm2_terminal_native_eval_v1", suite=args.suite,
                                   config_sha256=digest(args.experiment), reset_manifest_sha256=digest(manifest_path),
                                   evaluator_sha256=digest(__file__), model=plant.metadata,
                                   goal_com_m=goal.tolist(), wall_s=time.monotonic()-started,
                                   workers=args.workers, results=results, training_transitions=0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("development", "recovery-development", "heldout"), required=True)
    parser.add_argument("--experiment", type=Path, default=CONFIG, help="Pinned experiment config")
    parser.add_argument("--actor", type=Path, help="Portable float32 actor weights from a saved PPO checkpoint")
    parser.add_argument("--checkpoint", type=Path, help="Source checkpoint for the portable weights")
    parser.add_argument("--controls", action="store_true", help="Include matched frozen Track and hover trim")
    parser.add_argument("--deterministic", action="store_true", help="Include the Phase 3 closed-loop witness")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, help="Native processes, default 1 for development and 8 for held-out")
    parser.add_argument("--replay-dir", type=Path, help="Unused directory for representative 3D replay files")
    args = parser.parse_args()
    args.workers = args.workers if args.workers is not None else (8 if args.suite == "heldout" else 1)
    if not 1 <= args.workers <= 16:
        parser.error("Native worker count must be between 1 and 16")
    if bool(args.actor) != bool(args.checkpoint) or not (args.actor or args.controls or args.deterministic):
        parser.error("Supply both actor and checkpoint, controls, or both")
    if args.suite == "heldout" and not (args.actor and args.controls):
        parser.error("Final evaluation requires the selected actor and both matched controls")
    if args.output.exists() or (args.replay_dir and args.replay_dir.exists()):
        parser.error("Choose fresh output paths")
    evaluation(args)


if __name__ == "__main__":
    main()
