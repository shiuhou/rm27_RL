"""Freeze development-only resets for the matched Phase R2 continuation."""

from __future__ import annotations

from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from pathlib import Path
import time

import numpy as np

from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json, write_result
from .handover_curriculum import _source_trace, native_verifier
from .policy_frontier import perturbed_state
from .terminal_workflow_final import legacy_adapters

BRIDGE = "rotation_bridge"
VALIDATION = "bridge-validation.npz"
RECEIPT = "bridge-bank.json"
OUTWARD_POS_M = .05
OUTWARD_SPEED_M_S = .05
OUTWARD_WINDOW_S = .25
RECENTER_WINDOW_S = .18
TRAIN_CANDIDATES = 12
VALIDATION_CANDIDATES = 8
VALIDATION_ANCHORS = 12

RESET_KEYS = ("states", "motor_thrust_N", "motor_target_N", "case_ids",
              "source_frame", "sample_times_s", "phase")


def spread(frames, n):
    """Use time-ordered quantiles, retaining both edges when there is room."""
    return frames[np.rint(np.linspace(0, len(frames) - 1, min(n, len(frames)))).astype(int)]


def robust_frontier(result, variants, cases):
    outcomes = defaultdict(dict)
    for row in result["rows"]:
        if row["controller"] != "fixed_a":
            continue
        key = (row["case"], row["frame"])
        if row["variant"] in outcomes[key]:
            raise ValueError(f"Duplicate Phase R variant at {key}")
        outcomes[key][row["variant"]] = row["qualified"]
    if any(set(v) != set(variants) for v in outcomes.values()):
        raise ValueError("Phase R Fixed A outcome is missing a reset variant")
    return {case: min(frame for (name, frame), values in outcomes.items()
                      if case == name and all(values.values())) for case in cases}


def _case_candidates(states, times, spec, frames, frontier, last_rotation, excluded, validation):
    tangent = spec.target_rotation[:, 0]
    x = (states[:, :3] - spec.target_center) @ tangent
    vx = states[:, 7:10] @ tangent
    early = frames[(frames < frontier) & (frames <= last_rotation)]
    outward = (np.abs(x) >= OUTWARD_POS_M) & (np.abs(vx) >= OUTWARD_SPEED_M_S) & (x * vx > 0)
    usable_outward = early[outward[early]]
    candidates = {}
    if len(usable_outward):
        window = usable_outward[times[usable_outward] >= times[usable_outward[-1]] - OUTWARD_WINDOW_S]
        window = window[~np.isin(window, list(excluded))]
        n = VALIDATION_CANDIDATES if validation else TRAIN_CANDIDATES
        candidates.update({int(frame): "outward" for frame in spread(window, n)})
    if validation:
        # Retain reserved-source coverage even when the capped reference
        # cannot complete the outward segment on that case. These starts remain
        # before braking and before Fixed A's earliest sampled robust anchor.
        inward = early[(x[early] * vx[early] < 0) & (np.abs(x[early]) >= OUTWARD_POS_M)]
        if len(inward):
            window = inward[times[inward] >= times[inward[-1]] - RECENTER_WINDOW_S]
            window = window[~np.isin(window, list(excluded))]
            candidates.update({int(frame): "recentering" for frame in spread(window, VALIDATION_CANDIDATES)})
    return candidates, x, vx


def freeze_bridge(cfg, directory):
    """Evaluate the capped reference, then freeze training and reserved validation states."""
    started = time.monotonic()
    directory = Path(directory)
    source_run = ROOT / cfg["bridge"]["source_run"]
    original_path = ROOT / cfg["bridge"]["training_source"]
    manifest_path = source_run / "manifest.json"
    result_path = source_run / "results.json"
    frontier_protocol, frontier_result = map(read_json, (manifest_path, result_path))
    if (frontier_protocol["schema"] != "rm27_asm2_policy_frontier_v1"
            or frontier_result["status"] != "complete"
            or frontier_result["manifest_sha256"] != digest(manifest_path)
            or frontier_result["completed_rollouts"] != 576
            or frontier_protocol["controllers"]["fixed_a"]["checkpoint_sha256"] != digest(ROOT / cfg["parent"])):
        raise ValueError("Phase R and the Phase R2 initializer must match the frozen Fixed A evidence")
    for name, pin in frontier_protocol["pins"].items():
        if digest(ROOT / name) != pin:
            raise ValueError(f"Phase R pinned source changed: {name}")

    _, profile, spec, witness_path, witness, _ = native.load_evaluation_inputs(ROOT / cfg["task_config"])
    source_dir = witness_path.parent
    development = witness["splits"]["development"]
    bank = native.load_starts(source_dir, development, expected_count=development["resets"], profile=profile)
    with np.load(original_path, allow_pickle=False) as f:
        original = dict(f)
    if set(original) != {*RESET_KEYS, "band", "dt"} or original["dt"] != profile.policy_dt_s:
        raise ValueError("Control reset bank has a different schema or control interval")
    cases = sorted(development["reset_cases"])
    training_cases = set(original["case_ids"].tolist())
    reserved = set(cases) - training_cases
    if reserved != {"dev_far", "dev_lateral_positive", "stationary_1"}:
        raise ValueError("Expected the prior disjoint training and selection sources")
    variants = frontier_protocol["configuration"]["perturbation"]
    if variants["variants"] != {"nominal": 0, "minus": -1, "plus": 1}:
        raise ValueError("Unexpected Phase R perturbation variants")
    earliest = robust_frontier(frontier_result, variants["variants"], cases)
    excluded = defaultdict(set)
    for data in (original, bank):
        for case, frame in zip(data["case_ids"], data["source_frame"], strict=True):
            excluded[str(case)].add(int(frame))
    for sample in frontier_protocol["samples"]:
        excluded[sample["case"]].add(sample["frame"])

    old, _ = legacy_adapters((ROOT / frontier_protocol["controllers"]["phase4"]["checkpoint"]).parents[1])
    reference = old["load_action_limited_deterministic"](
        source_dir, witness, [("phase-r2", bank, range(len(bank["states"])))], profile, spec)
    if reference[1] != frontier_protocol["controllers"]["deterministic"]:
        raise ValueError("Capped feasibility controller differs from Phase R")
    controllers = {"deterministic": reference}
    plant = native.NativeContact(profile, spec)
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    verify = native_verifier(plant)
    tangent = spec.target_rotation[:, variants["axis_target_frame"]]
    candidates, rejected, columns = {}, [], defaultdict(list)
    for case in cases:
        entry = witness["cases"][case]
        if entry["split"] != "development" or not entry["qualified"]:
            raise ValueError(f"Witness is outside the qualified development bank: {case}")
        gate, touch = entry["handover_frame"], entry["first_contact_sample_s"]
        trace = (source_dir / entry["trace"]["path"]).read_bytes()
        states, motors, targets, times = _source_trace(trace, gate, touch, profile)
        frames = np.flatnonzero((np.arange(len(times)) >= gate) & (times < touch - 1e-12))
        last_rotation = int(bank["source_frame"][(bank["case_ids"] == case) & (bank["phase"] == "rotation")].max())
        chosen, x, vx = _case_candidates(states, times, spec, frames, earliest[case], last_rotation,
                                         excluded[case], case in reserved)
        signs = variants["variants"] if case in reserved else {"nominal": 0}
        for frame, kind in sorted(chosen.items()):
            meta = dict(case=case, source_frame=frame, geometry=kind, target_x_m=float(x[frame]),
                        target_vx_m_s=float(vx[frame]), witness_lead_s=float(touch - times[frame]))
            perturbed = {name: perturbed_state(states[frame], tangent, sign,
                                             variants["position_m"], variants["velocity_m_s"])
                         for name, sign in signs.items()}
            invalid = [name for name, state in perturbed.items()
                       if not verify(state, motors[frame], targets[frame])["passed"]
                       or native.flight_limits(state, spec, limits) is not None]
            if invalid:
                rejected.append(dict(meta, invalid_variants=invalid))
                continue
            candidates[case, frame] = dict(meta, trial_indices=[])
            for name, state in perturbed.items():
                candidates[case, frame]["trial_indices"].append(len(columns["states"]))
                for key, value in zip(RESET_KEYS + ("variant",),
                                      (state, motors[frame], targets[frame], case, frame, times[frame],
                                       "rotation", name), strict=True):
                    columns[key].append(value)

    data = {k: np.asarray(v) for k, v in columns.items()}
    data["dt"] = np.asarray(profile.policy_dt_s)
    horizon = frontier_protocol["horizon_s"]
    jobs = [("deterministic", "phase-r2-freeze", data, i,
             witness["cases"][str(case)]["first_contact_sample_s"]) for i, case in enumerate(data["case_ids"])]
    with ProcessPoolExecutor(max_workers=cfg["workers"], mp_context=multiprocessing.get_context("fork"),
                             initializer=native.init_worker,
                             initargs=(plant, controllers, {"handover": horizon, "rotation": horizon},
                                       limits, False)) as pool:
        outcomes = [row for row, _ in pool.map(native.worker_episode, jobs)]
    for meta in candidates.values():
        meta["feasibility"] = [dict(variant=str(data["variant"][i]),
                                    **{k: outcomes[i][k] for k in ("qualified", "reason", "safety_failure",
                                                                    "saturation_controls", "controls",
                                                                    "outside_actor_action_map_controls")})
                               for i in meta["trial_indices"]]
        meta["capped_feasible"] = all(row["qualified"] and row["outside_actor_action_map_controls"] == 0
                                      for row in meta["feasibility"])

    feasible = {case: [m for (name, _), m in candidates.items() if name == case and m["capped_feasible"]]
                for case in cases}
    training_rows = [m for case in cases if case in training_cases for m in feasible[case]]
    if len({m["case"] for m in training_rows}) < 2:
        raise RuntimeError("The bridge requires capped-feasible outward starts on at least two training sources")

    # Round-robin allocations preserve all available reserved sources without
    # requiring equal quotas when capped feasibility differs across witnesses.
    reserved_cases = sorted(reserved)
    capacity = {case: len(feasible[case]) for case in reserved_cases}
    if sum(capacity.values()) < VALIDATION_ANCHORS:
        raise RuntimeError(f"Expected at least {VALIDATION_ANCHORS} robust reserved anchors, got {capacity}")
    quotas = Counter()
    remaining = VALIDATION_ANCHORS
    while remaining:
        for case in reserved_cases:
            if quotas[case] < capacity[case] and remaining:
                quotas[case] += 1
                remaining -= 1
    validation_rows = []
    for case in reserved_cases:
        outgoing = [m for m in feasible[case] if m["geometry"] == "outward"]
        incoming = [m for m in feasible[case] if m["geometry"] == "recentering"]
        kept = spread(np.array(outgoing, dtype=object), quotas[case]).tolist()
        kept.extend(spread(np.array(incoming, dtype=object), quotas[case] - len(kept)).tolist())
        validation_rows.extend(kept)
    take_train = [i for row in training_rows for i in row["trial_indices"]]
    take_validation = [i for row in validation_rows for i in row["trial_indices"]]
    if set(data["case_ids"][take_train]) & set(data["case_ids"][take_validation]):
        raise ValueError("Training and validation witness sources overlap")
    expanded = {k: np.concatenate((original[k], data[k][take_train].astype(original[k].dtype)))
                for k in RESET_KEYS}
    expanded["band"] = np.r_[original["band"], np.full(len(take_train), BRIDGE, dtype=original["band"].dtype)]
    expanded["dt"] = original["dt"]
    n = len(original["states"])
    if any(original[k].tobytes() != expanded[k][:n].tobytes() for k in (*RESET_KEYS, "band")):
        raise AssertionError("The control reset rows changed")
    validation = {k: data[k][take_validation] for k in (*RESET_KEYS, "variant")}
    validation["dt"] = data["dt"]

    directory.mkdir(parents=True, exist_ok=True)
    for name, archive in (("training.npz", expanded), (VALIDATION, validation)):
        with (directory / name).open("xb") as file:
            np.savez_compressed(file, **archive)
    archive_hashes = {name: digest(directory / name) for name in ("training.npz", VALIDATION)}
    inputs = (manifest_path, result_path, original_path, witness_path, source_dir / development["path"],
              Path(__file__))
    report = dict(
        schema="rm27_asm2_rotation_bridge_bank_v1", archives=archive_hashes.copy(),
        source_pins={str(path.relative_to(ROOT)): digest(path) for path in inputs},
        deterministic=reference[1], fixed_a_earliest_3_of_3=earliest,
        design=dict(train_outward_position_min_m=OUTWARD_POS_M, train_outward_speed_min_m_s=OUTWARD_SPEED_M_S,
                    latest_outward_window_s=OUTWARD_WINDOW_S, latest_recentering_window_s=RECENTER_WINDOW_S,
                    training_nominal_candidates_per_case=TRAIN_CANDIDATES,
                    validation_candidates_per_case_per_geometry=VALIDATION_CANDIDATES,
                    validation_selection="Twelve total via round-robin quotas up to each capped-3/3 capacity. Spread across outward starts first, then pre-braking recentering starts.",
                    validation_capacity=capacity, validation_quotas=dict(quotas),
                    validation_cases=sorted(reserved), perturbation=variants, horizon_s=horizon),
        original_rows=n, training_count=len(take_train), validation_anchors=len(validation_rows),
        validation_rollouts=len(take_validation), feasibility_rollouts=len(outcomes),
        candidate_count=len(candidates), invalid_restore=rejected,
        feasibility_reasons=dict(Counter(row["reason"] for row in outcomes)),
        training_selected=[m for m in training_rows], validation_selected=[m for m in validation_rows],
        candidates=list(candidates.values()), wall_s=time.monotonic() - started)
    write_result(directory / RECEIPT, report)
    archive_hashes[RECEIPT] = digest(directory / RECEIPT)
    return dict(archives=archive_hashes, bridge_training_count=len(take_train),
                bridge_validation_anchors=len(validation_rows), bridge_validation_rollouts=len(take_validation))
