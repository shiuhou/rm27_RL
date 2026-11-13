"""Matched native Phase R evaluation on development witness resets."""
from __future__ import annotations

import argparse
from collections import Counter
from itertools import count
import json
import multiprocessing as mp
from pathlib import Path
import time

import numpy as np

from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json, write_result
from .handover_curriculum import native_verifier
from .terminal_workflow_final import legacy_adapters

CONFIG = ROOT / "configs/rm27_asm2_frontier.json"
MODES = ("phase4", "fixed_a", "deterministic")
OUT = ROOT / ".agents/runs/asm2-frontier/phase-r"


def select_anchors(bank, cases, anchors):
    selected = []
    for case in cases:
        for label, source_phase, ordinal in anchors:
            indices = np.flatnonzero((bank["case_ids"] == case) & (bank["phase"] == source_phase))
            indices = indices[np.argsort(bank["source_frame"][indices])]
            if not -len(indices) <= ordinal < len(indices):
                raise ValueError(f"No {source_phase} reset at ordinal {ordinal} for {case}")
            selected.append((case, label, int(indices[ordinal])))
    keys = [(case, int(bank["source_frame"][index])) for case, _, index in selected]
    if len(set(keys)) != len(keys):
        raise ValueError("Duplicate witness case/frame in frontier anchors")
    return selected


def perturbed_state(state, tangent, sign, position_m, velocity_m_s):
    state = np.asarray(state).copy()
    if sign:
        state[:3] += sign * position_m * tangent
        state[7:10] += sign * velocity_m_s * tangent
    return state


def make_protocol(config_path):
    cfg = read_json(config_path)
    if cfg["schema"] != "rm27_asm2_policy_frontier_v1":
        raise ValueError("Unexpected frontier config")
    if not (0 < cfg["max_rollouts"] <= 600 and 0 < cfg["max_seconds"] <= 1800):
        raise ValueError("Phase R cap exceeded")
    experiment = ROOT / cfg["experiment"]
    task, profile, spec, manifest_path, manifest, goal = native.load_evaluation_inputs(experiment)
    split = manifest["splits"]["development"]
    cases = sorted(split["reset_cases"])
    if len(cases) != 8 or any(manifest["cases"][name]["split"] != "development"
                              or not manifest["cases"][name]["qualified"] for name in cases):
        raise ValueError("Expected eight qualified development witnesses")
    source = manifest_path.parent
    bank = native.load_starts(source, split, expected_count=split["resets"], profile=profile)
    indices = select_anchors(bank, cases, cfg["anchors"])
    prior = ROOT / cfg["comparison"]
    selection_path = prior / "final-selection.json"
    selections = read_json(selection_path)["selections"]
    controllers, provenance = {}, {}
    for mode, selected in (("phase4", selections["phase4"]), ("fixed_a", selections["A"])):
        checkpoint = ROOT / selected["checkpoint"]
        weights = checkpoint.with_suffix(".npz")
        command, pin = native.load_controller(weights, digest(checkpoint), profile, goal)
        if any(pin[k] != selected[k] for k in pin):
            raise ValueError(f"Frozen policy pin mismatch for {mode}")
        controllers[mode] = command
        provenance[mode] = dict(checkpoint=selected["checkpoint"], **pin)
    old, _ = legacy_adapters((ROOT / selections["phase4"]["checkpoint"]).parents[1])
    controllers["deterministic"], provenance["deterministic"] = old["load_action_limited_deterministic"](
        source, manifest, [("phase-r", bank, [i for _, _, i in indices])], profile, spec)

    # Read A's actual sampling stage and counts at the evaluated checkpoint.
    stage_frame = selections["A"]["selected_at_transitions"]
    log_path = prior / "A/learning.jsonl"
    with log_path.open() as stream:
        log = next(row for line in stream if (row := json.loads(line))["transitions"] == stage_frame)
    training_path = prior / "training.npz"
    with np.load(training_path, allow_pickle=False) as training:
        coverage = {name: training["source_frame"][(training["case_ids"] == name)
                                                   & np.isin(training["band"], [b for b, c in log["region_counts"].items()
                                                                                if c["resets"] > 0])]
                    for name in cases}
    plant = native.NativeContact(profile, spec)
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    verify = native_verifier(plant)
    perturb = cfg["perturbation"]
    tangent = spec.target_rotation[:, perturb["axis_target_frame"]]
    samples = []
    for case, phase, index in indices:
        witness = manifest["cases"][case]
        frame = int(bank["source_frame"][index])
        lead = witness["first_contact_sample_s"] - float(bank["sample_times_s"][index])
        if frame < witness["handover_frame"] or lead <= 0:
            raise ValueError(f"Expected a pre-contact reset at or after handover: {case}:{frame}")
        for variant, sign in perturb["variants"].items():
            state = perturbed_state(bank["states"][index], tangent, sign,
                                    perturb["position_m"], perturb["velocity_m_s"])
            motors, targets = bank["motor_thrust_N"][index], bank["motor_target_N"][index]
            checked = verify(state, motors, targets)
            if not checked["passed"] or native.flight_limits(state, spec, limits):
                raise ValueError(f"Invalid restored start: {case}:{frame}:{variant}: {checked}")
            samples.append(dict(case=case, frame=frame, phase=phase, source_phase=str(bank["phase"][index]),
                                witness_kind=witness["kind"], lead_s=lead, variant=variant,
                                state=state.tolist(), motor_thrust_N=motors.tolist(), motor_target_N=targets.tolist(),
                                fixed_a_nearest_sampled_reset_s=(
                                    float(np.min(np.abs(coverage[case] - frame)) * profile.policy_dt_s)
                                    if len(coverage[case]) else None)))
    pilot = [i for i, s in enumerate(samples)
             if s["case"] == cfg["pilot"]["case"] and s["phase"] in cfg["pilot"]["labels"]]
    planned = len(MODES) * (len(samples) + len(pilot))
    if planned > cfg["max_rollouts"] or len(pilot) != 6:
        raise ValueError(f"Invalid pilot or aggregate rollout count: {planned}")
    pins = (config_path, experiment, manifest_path, source / split["path"], selection_path, training_path,
            log_path, native.HANDOVER_CONFIG, Path(native.__file__), Path(__file__),
            ROOT / "src/rm27_asm2/contact_task.py", ROOT / "src/rm27_asm2/contact_native.py")
    protocol = dict(schema=cfg["schema"], configuration=cfg,
                    pins={str(p.resolve().relative_to(ROOT)): digest(p) for p in pins},
                    controllers=provenance, model=plant.metadata, horizon_s=task["handover_horizon_s"],
                    control_dt_s=profile.policy_dt_s, goal=goal.tolist(), variants=list(perturb["variants"]),
                    planned_rollouts=planned, pilot_indices=pilot, samples=samples,
                    fixed_a_selected_stage=log["sampler_stage"], fixed_a_selected_reset_counts=log["region_counts"],
                    robustness={"mastered": "3/3", "partial": "1/3 or 2/3", "failing": "0/3"},
                    training_transitions=0)
    return protocol, plant, limits, controllers


def summarize_episode(result, trace, plant):
    states, times, commands = trace
    spec, profile = plant.spec, plant.profile
    xyz = (states[:, :3] - spec.target_center) @ spec.target_rotation
    velocity = states[:, 7:10] @ spec.target_rotation
    selection = spec.field_geometry["selection"]
    margin = np.minimum(xyz - selection["safety_lower_target_frame_m"],
                        selection["safety_upper_target_frame_m"] - xyz)
    first = result["contact"]["first_touch"]
    contact = None
    if first:
        points = first["records"]
        contact = {k: first[k] for k in ("sample_time_s", "groups", "aircraft_colliders",
                    "target_colliders", "signed_points", "body_rate_rad_s", "max_penetration_m",
                    "compressive_normal_impulse_N_s")}
        for key in ("normal_approach_speed_m_s", "tangential_speed_m_s", "body_to_target_alignment_deg"):
            contact[key] = [min(p[key] for p in points), max(p[key] for p in points)]
    near_cap = (np.abs(commands[:, 1:]) >= 1.9).any(axis=1) if len(commands) else []
    lift = commands[:, 0] / profile.hover_thrust_N if len(commands) else np.empty(0)
    initial_rotation = native.rotation_matrix(states[0, 3:7])
    sign = np.sign(velocity[0, 0])
    reversal = np.flatnonzero(sign * velocity[:, 0] <= -.05) if abs(velocity[0, 0]) >= .05 else []
    # Keep early response on a common clock. A terminated rollout has no later state.
    early = {}
    for t in (.10, .25, .50):
        if times[-1] >= t:
            i = int(np.searchsorted(times, t))
            early[str(t)] = dict(position_target_m=xyz[i].tolist(), velocity_target_m_s=velocity[i].tolist(),
                                 ctbr=commands[i - 1].tolist())
    return dict(qualified=result["qualified"], reason=result["reason"], safety_failure=result["safety_failure"],
                contact_reason=result["contact"]["reason"], body_first=bool(first and "body" in first["groups"]),
                timeout=result["reason"] == "NO_SIGNED_TOUCH" and not first,
                end_s=result["end_s"], controls=result["controls"], saturation_controls=result["saturation_controls"],
                near_rate_cap_controls=int(np.count_nonzero(near_cap)),
                near_collective_cap_controls=int(np.count_nonzero((lift <= .525) | (lift >= 1.475))),
                initial_position_target_m=xyz[0].tolist(), initial_velocity_target_m_s=velocity[0].tolist(),
                initial_alignment_deg=float(np.rad2deg(np.arccos(np.clip(initial_rotation[:, 2] @ spec.target_normal, -1, 1)))),
                initial_volume_margin_m=float(margin[0].min()), min_volume_margin_m=float(margin.min()),
                peak_speed_m_s=float(np.linalg.norm(states[:, 7:10], axis=1).max()),
                target_x_reversal_s=float(times[reversal[0]]) if len(reversal) else None,
                initial_command=commands[0].tolist() if len(commands) else None,
                initial_command_rate_headroom_rad_s=float(2. - np.max(np.abs(commands[0, 1:]))) if len(commands) else None,
                final_position_target_m=xyz[-1].tolist(), early=early, first_touch=contact)


def evaluate_reset(plant, limits, controllers, horizon, sample):
    state, motors, targets = (np.array(sample[k]) for k in ("state", "motor_thrust_N", "motor_target_N"))
    metadata = {k: v for k, v in sample.items() if k not in ("state", "motor_thrust_N", "motor_target_N")}
    available = plant.profile.max_rotor_thrust_N
    metadata.update(initial_body_rates_rad_s=state[10:13].tolist(),
                    initial_speed_m_s=float(np.linalg.norm(state[7:10])),
                    rotor_activation_rail_margin_N=float(np.minimum(motors, available - motors).min()),
                    rotor_target_rail_margin_N=float(np.minimum(targets, available - targets).min()))
    out = []
    for mode in MODES:
        command = controllers[mode]
        if mode == "deterministic":
            ref, gate = command[sample["case"]]
            ticks = count(sample["frame"] - gate)
            command = lambda live: ref.command(live, next(ticks))
        result, trace = native.rollout(plant, state, motors, targets, command, horizon, limits)
        row = dict(metadata, controller=mode, **summarize_episode(result, trace, plant))
        out.append((row, result, trace))
    return out


def worker(sample):
    return evaluate_reset(*native._WORKER_ARGS[:-1], sample)


def run_batch(pool, samples, deadline):
    result = []
    pending = pool.imap(worker, samples, chunksize=1)
    for _ in samples:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            result.extend(pending.next(timeout=remaining))
        except mp.TimeoutError:
            break
    return result


def rows_summary(rows):
    anchors = {}
    for row in rows:
        key = (row["controller"], row["case"], row["frame"])
        anchors.setdefault(key, []).append(row)
    grouped = {}
    for mode in MODES:
        mode_rows = [r for r in rows if r["controller"] == mode]
        phases = {}
        for phase in dict.fromkeys(r["phase"] for r in mode_rows):
            phase_rows = [r for r in mode_rows if r["phase"] == phase]
            phase_anchors = [a for a in anchors.values() if a[0]["controller"] == mode and a[0]["phase"] == phase]
            robust = [sum(r["qualified"] for r in a) for a in phase_anchors if len(a) == 3]
            phases[phase] = dict(qualified=sum(r["qualified"] for r in phase_rows), episodes=len(phase_rows),
                                 mastered=robust.count(3), partial=sum(q in (1, 2) for q in robust),
                                 failing=robust.count(0), body_first=sum(r["body_first"] for r in phase_rows),
                                 reasons=dict(Counter(r["reason"] for r in phase_rows)))
        grouped[mode] = phases
    return grouped


def export_replays(directory, outcomes, plant, goal):
    nominal = {}
    for item in outcomes:
        row = item[0]
        if row["variant"] == "nominal":
            nominal.setdefault((row["case"], row["frame"]), {})[row["controller"]] = item
    triplets = [g for g in nominal.values() if len(g) == len(MODES)]
    failures = [g for g in triplets if g["deterministic"][0]["qualified"]
                and all(g[m][0]["safety_failure"] for m in MODES[:2])]
    gains = [g for g in triplets if g["fixed_a"][0]["qualified"] and not g["phase4"][0]["qualified"]]
    chosen = {}
    if failures:
        chosen["paired_exit"] = max(failures, key=lambda g: g["phase4"][0]["lead_s"])
    if gains:
        chosen["paired_gain"] = max(gains, key=lambda g: g["fixed_a"][0]["lead_s"])
    elif triplets:
        successes = [g for g in triplets if g["fixed_a"][0]["qualified"]]
        if successes:
            chosen["paired_contact"] = max(successes, key=lambda g: g["fixed_a"][0]["lead_s"])
    mesh = native.target_mesh(plant, plant.spec.legal_targets)
    replays = {}
    for label, group in chosen.items():
        for mode, (row, result, trace) in group.items():
            path = directory / "replays" / f"{label}-{mode}.npz"
            path.parent.mkdir(exist_ok=True)
            native.save_replay(path, f"{label}-{mode}-{row['case']}:{row['frame']}", trace,
                               result, np.array(goal), mesh, plant.profile.policy_dt_s)
            replays[f"{label}/{mode}"] = dict(path=str(path.relative_to(directory)), sha256=digest(path),
                                               case=row["case"], frame=row["frame"], reason=row["reason"])
    return replays


def run(config_path, directory, expected_manifest=None):
    started = time.monotonic()
    protocol, plant, limits, controllers = make_protocol(config_path)
    cfg = protocol["configuration"]
    deadline = started + cfg["max_seconds"]
    if expected_manifest and protocol != read_json(expected_manifest):
        raise ValueError("Reconstructed protocol differs from the frozen manifest")
    directory.mkdir(parents=True, exist_ok=False)
    samples = protocol["samples"]
    initargs = (plant, limits, controllers, protocol["horizon_s"], None)
    with mp.get_context("fork").Pool(cfg["workers"], initializer=native.init_worker, initargs=initargs) as pool:
        pilot = [samples[i] for i in protocol["pilot_indices"]]
        preview = [item[0] for item in run_batch(pool, pilot, deadline)]
        passed = (len(preview) == len(pilot) * len(MODES)
                  and all(r["controls"] > 0 for r in preview)
                  and any(r["qualified"] and r["controller"] == "deterministic" for r in preview))
        write_result(directory / "pilot.json", dict(passed=passed, completed_rollouts=len(preview),
                                                       elapsed_s=time.monotonic() - started, rows=preview))
        if not passed:
            raise RuntimeError("Phase R validation pilot did not pass")
        # Publish before main results are inspected.
        manifest = directory / "manifest.json"
        write_result(manifest, protocol)
        print(f"Pilot passed ({len(preview)}). Frozen {manifest} {digest(manifest)}", flush=True)
        outcomes = run_batch(pool, samples, deadline)
    rows = [item[0] for item in outcomes]
    status = "complete" if len(rows) == len(samples) * len(MODES) else "time_limit"
    # Verify the repeated pilot matches the corresponding independently restored main rollouts.
    index = {(r["case"], r["frame"], r["variant"], r["controller"]): r for r in rows}
    for r in preview:
        key = (r["case"], r["frame"], r["variant"], r["controller"])
        if key in index and r != index[key]:
            raise RuntimeError(f"Pilot and main rollout differ for {key}")
    replays = export_replays(directory, outcomes, plant, protocol["goal"]) if outcomes else {}
    write_result(directory / "results.json", dict(
        status=status, manifest_sha256=digest(manifest), completed_rollouts=len(rows),
        pilot_rollouts=len(preview), elapsed_s=time.monotonic() - started, training_transitions=0,
        regions=rows_summary(rows), replays=replays, rows=rows))
    print(f"Phase R {status}: {len(rows)} main rollouts, {time.monotonic() - started:.1f} s", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output", type=Path, default=OUT)
    parser.add_argument("--manifest", type=Path, help="Verify reconstruction against a prior frozen manifest")
    args = parser.parse_args()
    run(args.config.resolve(), args.output.resolve(), args.manifest)


if __name__ == "__main__":
    main()
