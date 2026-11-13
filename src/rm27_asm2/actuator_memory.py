"""Phase R1.5-M. Compare naturally recorded aircraft states in native MuJoCo."""
from __future__ import annotations

import argparse
from collections import Counter
from itertools import combinations
from pathlib import Path
import signal
import time

import numpy as np

from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json, write_result
from .flight import allocate_ctbr
from .handover_curriculum import _source_trace, native_verifier
from .policy_frontier import summarize_episode

CONFIG = ROOT / "configs/rm27_asm2_actuator_memory.json"


def actor_features(state, goal):
    """Mirror the frozen portable actor's float32 input order."""
    return np.r_[np.tile(goal - state[:3], 10), state[7:10],
                 native.rotation_matrix(state[3:7]).T.ravel(), state[10:13]].astype(np.float32)


def visible_errors(state, other):
    """Physical residuals with quaternion sign invariance and explicit units."""
    other = np.asarray(other)
    dot = np.abs(other[..., 3:7] @ state[3:7])
    angle = np.degrees(2. * np.arccos(np.clip(dot, 0., 1.)))
    return np.stack((np.linalg.norm(other[..., :3] - state[:3], axis=-1),
                     np.linalg.norm(other[..., 7:10] - state[7:10], axis=-1),
                     angle, np.linalg.norm(other[..., 10:13] - state[10:13], axis=-1)), axis=-1)


VISIBLE_KEYS = ("position_m", "velocity_m_s", "attitude_deg", "body_rates_rad_s")
MEMORY_KEYS = ("state", "motor_thrust_N", "motor_target_N")


def natural_sample(case, frame, trace, path, state, thrust, target):
    return dict(case=case, frame=int(frame), trace=trace, path=str(path.relative_to(ROOT)),
                state=np.asarray(state), motor_thrust_N=np.asarray(thrust), motor_target_N=np.asarray(target))


def load_corpus(cfg, phase_r, witness, witness_path, plant):
    source = witness_path.parent
    samples, pins = [], {}
    for case in sorted(witness["splits"]["development"]["reset_cases"]):
        entry = witness["cases"][case]
        if entry["split"] != "development" or not entry["qualified"]:
            raise ValueError(f"Invalid development source {case}")
        pin = entry["trace"]
        path = (source / pin["path"]).resolve()
        if digest(path) != pin["sha256"]:
            raise ValueError(f"Witness trace changed: {path}")
        pins[str(path.relative_to(ROOT))] = pin["sha256"]
        touch = entry["first_contact_sample_s"]
        state, thrust, target, times = _source_trace(path.read_bytes(), entry["handover_frame"],
                                                     touch, plant.profile)
        for frame in np.flatnonzero((np.arange(len(times)) >= entry["handover_frame"]) & (times < touch - 1e-12)):
            samples.append(natural_sample(case, frame, "phase3", path, state[frame], thrust[frame], target[frame]))

    path = ROOT / cfg["branch_trace"]
    branch = read_json(path)
    case, origin = branch["case"], branch["source_frame"]
    if (case not in witness["splits"]["development"]["reset_cases"]
            or branch["model"] != plant.metadata or branch["reset_manifest_sha256"] != digest(witness_path)
            or branch["control_dt_s"] != plant.profile.policy_dt_s or branch["goal_world_m"] != phase_r["goal"]):
        raise ValueError("Archived diagnostic does not match the development plant")
    # Verify the shared branch root against an original natural record.
    roots = [s for s in samples if s["case"] == case and s["frame"] == origin]
    root, = roots
    for a, b in zip(MEMORY_KEYS, ("initial_state", "initial_activation_N", "initial_held_targets_N"), strict=True):
        np.testing.assert_array_equal(root[a], branch[b])
    for label, run in sorted(branch["results"].items()):
        for i, row in enumerate(run["ticks"]):
            if abs(row["time_s"] - i * plant.profile.policy_dt_s) > 1e-12:
                raise ValueError("Archived actuator records are not at the control boundary")
            np.testing.assert_array_equal(row["state"], run["full_state"][i])
            if i == 0:
                for a, b in zip(MEMORY_KEYS, ("state", "activation_N", "held_targets_N"), strict=True):
                    np.testing.assert_array_equal(root[a], row[b])
            samples.append(natural_sample(case, origin + i, f"phase5-{origin}-{label}", path,
                                          row["state"], row["activation_N"], row["held_targets_N"]))
    pins[str(path.relative_to(ROOT))] = digest(path)
    states, thrust, target = (np.array([s[k] for s in samples]) for k in MEMORY_KEYS)
    if (states.shape != (len(samples), 13) or not np.isfinite(states).all()
            or not np.allclose(np.linalg.norm(states[:, 3:7], axis=1), 1., rtol=0, atol=1e-9)
            or any(x.shape != (len(samples), 4) or not np.isfinite(x).all()
                   or np.any((x < 0) | (x > plant.profile.max_rotor_thrust_N + 1e-12)) for x in (thrust, target))):
        raise ValueError("Invalid natural-state corpus")
    return samples, pins


def search_pairs(samples, anchors, cfg, command, hover, dt):
    """Search phase-anchor windows and retain the best natural activation contrasts."""
    states, thrust, targets = (np.array([s[k] for s in samples]) for k in MEMORY_KEYS)
    cases = np.array([s["case"] for s in samples])
    trace_ids = np.array([s["case"] + "/" + s["trace"] for s in samples])
    frames = np.array([s["frame"] for s in samples])
    tolerance = np.array([cfg["visible_limits"][k] for k in VISIBLE_KEYS])
    radius = round(cfg["anchor_window_s"] / dt)
    window_anchors = {}
    for anchor in anchors:
        eligible = np.flatnonzero((cases == anchor["case"]) & (np.abs(frames - anchor["frame"]) <= radius))
        for i in eligible:
            if i not in window_anchors or abs(frames[i] - anchor["frame"]) < abs(frames[i] - window_anchors[i]["frame"]):
                window_anchors[i] = anchor
    closest = None
    closest_cross_witness = None
    visible = {}
    searched = 0
    for i, anchor in window_anchors.items():
        other = np.flatnonzero(trace_ids != trace_ids[i])
        errors = visible_errors(states[i], states[other])
        scaled = (errors / tolerance).max(axis=1)
        searched += len(other)
        for cross in (False, True):
            candidates = np.flatnonzero(cases[other] != cases[i]) if cross else np.arange(len(other))
            if not len(candidates):
                continue
            best = candidates[scaled[candidates].argmin()]
            candidate = (float(scaled[best]), int(i), int(other[best]), anchor)
            if cross:
                if closest_cross_witness is None or candidate[0] < closest_cross_witness[0]:
                    closest_cross_witness = candidate
            elif closest is None or candidate[0] < closest[0]:
                closest = candidate
        for j, distance in zip(other[scaled <= 1.], scaled[scaled <= 1.], strict=True):
            key = tuple(sorted((int(i), int(j))))
            if key not in visible:
                visible[key] = (float(distance), anchor)

    feasible, counts = [], Counter()
    for (i, j), (distance, anchor) in visible.items():
        act = float(np.max(np.abs(thrust[i] - thrust[j])))
        ctrl = float(np.max(np.abs(targets[i] - targets[j])))
        contrast = cfg["min_memory_delta_N"]
        if act < contrast and ctrl < contrast:
            counts["insufficient_memory_contrast"] += 1
            continue
        delta = np.abs(command(states[i]) - command(states[j]))
        if (delta[0] / hover > cfg["actor_limits"]["collective_fraction_hover"]
                or delta[1:].max() > cfg["actor_limits"]["rate_rad_s"]):
            counts["actor_command_mismatch"] += 1
            continue
        kind = "activation" if act >= contrast else "target_dominant"
        counts[kind] += 1
        feasible.append(dict(i=i, j=j, anchor=anchor, distance=distance,
                             activation_delta_N=act, held_target_delta_N=ctrl, kind=kind))
    feasible.sort(key=lambda p: (p["kind"] != "activation", -p["activation_delta_N"], p["distance"], p["i"], p["j"]))

    def nearest(candidate):
        if candidate is None:
            return None
        distance, i, j, anchor = candidate
        return dict(i=i, j=j, anchor=anchor, normalized_visible_residual=distance,
                    activation_delta_N=float(np.max(np.abs(thrust[i] - thrust[j]))),
                    held_target_delta_N=float(np.max(np.abs(targets[i] - targets[j]))))

    return feasible, dict(window_records=len(window_anchors), directed_comparisons=searched,
                          visible_pairs=len(visible), cross_witness_visible_pairs=sum(cases[i] != cases[j] for i, j in visible),
                          classifications=dict(counts), closest=nearest(closest),
                          closest_cross_witness=nearest(closest_cross_witness))


def pulse_panel(center, hover, pulse):
    """Common absolute CTBR commands with one symmetric change per axis."""
    panel = {"center": np.array(center, dtype=float)}
    step = [hover * pulse["collective_fraction_hover"]] + [pulse["rate_rad_s"]] * 3
    lower, upper = np.array([.5 * hover, -2., -2., -2.]), np.array([1.5 * hover, 2., 2., 2.])
    for axis, name in enumerate(("thrust", "roll", "pitch", "yaw")):
        for sign, suffix in ((-1, "minus"), (1, "plus")):
            candidate = panel["center"].copy()
            candidate[axis] += sign * step[axis]
            if np.all((candidate >= lower) & (candidate <= upper)):
                panel[f"{name}_{suffix}"] = candidate
    return panel


def region_features(states, spec):
    """Target-frame position, outward/inward motion and facing geometry."""
    xyz = (states[:, :3] - spec.target_center) @ spec.target_rotation
    velocity = states[:, 7:10] @ spec.target_rotation
    rotation = native.rotation_matrix(states[:, 3:7])
    alignment = np.degrees(np.arccos(np.clip(rotation[:, :, 2] @ spec.target_normal, -1., 1.)))
    return np.column_stack((xyz, -velocity[:, 2], alignment)), xyz[:, 0] * velocity[:, 0] < 0.


def classify_local(result, entry_s, window_s):
    if result["end_s"] <= window_s + 1e-9:
        if result["qualified"]:
            return 3
        if result["safety_failure"] or result["contact"]["first_touch"] is not None:
            return 0
    return 2 if entry_s is not None and entry_s <= window_s else 1


def compare_panel(left, right):
    """Ordinal decision changes. Tied preferences do not count as reversals."""
    if left.keys() != right.keys():
        raise ValueError("The same actions must be tested on both variants")
    viable = [sorted(k for k, score in scores.items() if score > 0) for scores in (left, right)]
    reversals = [(a, b) for a, b in combinations(left, 2)
                 if (left[a] - left[b]) * (right[a] - right[b]) < 0]
    return dict(viable_actions=viable, changed_viable_actions=sorted(set(viable[0]) ^ set(viable[1])),
                strict_preference_reversals=reversals)


def serial_sample(sample):
    return {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in sample.items()}


def restore_check(sample, plant, limits, verify, center):
    state, act, target = (sample[k] for k in MEMORY_KEYS)
    receipt = verify(state, act, target)
    if not receipt["passed"] or native.flight_limits(state, plant.spec, limits):
        return None
    native.restore(plant, state, act, target)
    expected, _ = allocate_ctbr(plant.profile, np.array(center)[None], state[None, 10:13])
    saturated = plant.set_ctbr(np.array(center))
    np.testing.assert_array_equal(plant.data.act, act)
    np.testing.assert_array_equal(plant.data.ctrl, expected[0])
    return dict(restore=receipt, target_before_command_N=target.tolist(),
                target_after_command_N=plant.data.ctrl.tolist(), act_unchanged=True,
                time_s=float(plant.data.time), saturated=saturated)


def pair_measure(left, right, command, goal, profile, layers):
    initial = [actor_features(s["state"], goal) for s in (left, right)]
    norm = [native.NativeActor.mean(obs[None], layers[:1])[0] for obs in initial]
    act = right["motor_thrust_N"] - left["motor_thrust_N"]
    target = right["motor_target_N"] - left["motor_target_N"]
    body_wrench = profile.mixer @ act
    return dict(
        visible_residual=dict(zip(VISIBLE_KEYS, visible_errors(left["state"], right["state"]).tolist(), strict=True)),
        raw_observation=[x.tolist() for x in initial],
        raw_observation_delta=(initial[1] - initial[0]).tolist(),
        raw_observation_max_abs=float(np.max(np.abs(initial[1] - initial[0]))),
        raw_observation_exact=bool(np.array_equal(*initial)),
        first_layernorm_delta=(norm[1] - norm[0]).tolist(),
        ctbr=[command(s["state"]).tolist() for s in (left, right)],
        activation_delta_N=act.tolist(), previous_target_delta_N=target.tolist(),
        activation_wrench_delta_N_Nm=body_wrench.tolist(),
        common_target_memory_velocity_scale_m_s=float(np.linalg.norm(act @ profile.axes)
                                                       * profile.motor_time_constant_s / profile.mass_kg),
        common_target_memory_rate_scale_rad_s=(profile.motor_time_constant_s *
                                               np.linalg.solve(profile.inertia_kg_m2, body_wrench[1:])).tolist())


def command_with_pulse(command, pulse, control_ticks):
    """Use a shared command for exactly N policy callbacks, then frozen feedback."""
    remaining = control_ticks

    def apply(state):
        nonlocal remaining
        if pulse is not None and remaining:
            remaining -= 1
            return pulse
        return command(state)

    return apply


def rollout_sample(sample, command, pulse, pulse_ticks, plant, horizon, limits, region, cfg):
    control = command_with_pulse(command, pulse, pulse_ticks)
    outcome, trace = native.rollout(plant, *(np.array(sample[k]) for k in MEMORY_KEYS),
                                    control, horizon, limits)
    states, times, commands = trace
    applied = min(pulse_ticks, len(commands)) if pulse is not None else 0
    if applied:
        np.testing.assert_array_equal(commands[:applied], np.broadcast_to(pulse, (applied, 4)))
    features, inward = region_features(states, plant.spec)
    inside = inward & np.all((features >= region["min"]) & (features <= region["max"]), axis=1)
    entry = float(times[np.flatnonzero(inside)[0]]) if inside.any() else None
    early = {}
    for t in cfg["sample_times_s"]:
        found = np.flatnonzero(np.isclose(times, t, atol=1e-10, rtol=0))
        early[str(t)] = states[found[0]].tolist() if len(found) else None
    row = summarize_episode(outcome, trace, plant)
    row.update(local_rank=classify_local(outcome, entry, cfg["local_horizon_s"]),
               first_region_entry_s=entry, early_states=early, pulse_controls=applied,
               first_applied_ctbr=commands[0].tolist() if len(commands) else None)
    return row, outcome, trace


def run(config_path, output, timing_pilot=None):
    cfg = read_json(config_path)
    if cfg["schema"] != "rm27_asm2_actuator_memory_v1" or not (0 < cfg["max_rollouts"] <= 300
                                                              and 0 < cfg["max_seconds"] <= 600):
        raise ValueError("Unrecognized protocol or Phase R1.5-M cap exceeded")
    started = time.monotonic()
    prior_rollouts, prior_seconds, prior_usage = 0, 0., None
    if timing_pilot is not None:
        previous = read_json(timing_pilot / "results.json")
        previous_manifest = timing_pilot / "manifest.json"
        if previous["manifest_sha256"] != digest(previous_manifest) or read_json(previous_manifest)["configuration"] != cfg:
            raise ValueError("Pilot and diagnostic configurations differ")
        prior_rollouts, prior_seconds = previous["attempted_rollouts"], previous["wall_s"]
        prior_usage = dict(rollouts=prior_rollouts, wall_s=prior_seconds,
                           result_sha256=digest(timing_pilot / "results.json"),
                           exclusion="Pilot used one control tick due to a physics/policy counter mismatch.")
    max_rollouts = cfg["max_rollouts"] - prior_rollouts
    deadline = started + cfg["max_seconds"] - prior_seconds
    phase_r_dir = ROOT / cfg["frontier_run"]
    phase_r_path, results_path = (phase_r_dir / name for name in ("manifest.json", "results.json"))
    phase_r, prior = map(read_json, (phase_r_path, results_path))
    if prior["status"] != "complete" or prior["manifest_sha256"] != digest(phase_r_path):
        raise ValueError("Phase R protocol is incomplete")
    for source, pin in phase_r["pins"].items():
        if digest(ROOT / source) != pin:
            raise ValueError(f"Phase R source changed: {source}")
    _, profile, spec, witness_path, witness, goal = native.load_evaluation_inputs(ROOT / cfg["experiment"])
    plant = native.NativeContact(profile, spec)
    if plant.metadata != phase_r["model"] or goal.tolist() != phase_r["goal"]:
        raise ValueError("Phase R plant or actor goal changed")
    actor = phase_r["controllers"]["fixed_a"]
    checkpoint = ROOT / actor["checkpoint"]
    weights = checkpoint.with_suffix(".npz")
    command, pin = native.load_controller(weights, digest(checkpoint), profile, goal)
    if any(actor[k] != v for k, v in pin.items()):
        raise ValueError("Fixed A checkpoint or portable actor changed")
    with np.load(weights, allow_pickle=False) as data:
        layers = native.NativeActor.layers(data, 45)
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    verify = native_verifier(plant)
    samples, source_pins = load_corpus(cfg, phase_r, witness, witness_path, plant)
    nominal = [s for s in phase_r["samples"] if s["variant"] == "nominal"]
    anchors = [{k: s[k] for k in ("case", "frame", "phase")} for s in nominal if s["phase"] in cfg["anchor_phases"]]
    feasible, audit = search_pairs(samples, anchors, cfg, command, profile.hover_thrust_N, profile.policy_dt_s)
    for key in ("closest", "closest_cross_witness"):
        item = audit[key]
        if item is not None:
            left, right = (samples[item.pop(k)] for k in ("i", "j"))
            item["states"] = [serial_sample(s) for s in (left, right)]
            item["measurements"] = pair_measure(left, right, command, goal, profile, layers)
    region_samples = np.array([s["state"] for s in nominal if s["phase"] in ("transition_pre", "transition_post", "braking_mid")])
    features, _ = region_features(region_samples, spec)
    region = dict(min=features.min(axis=0).tolist(), max=features.max(axis=0).tolist(),
                  columns=["target_x_m", "target_y_m", "target_normal_m", "inward_normal_speed_m_s", "alignment_deg"],
                  condition="inside Phase R nominal transition/braking bounding box and moving inward on target x")
    chosen, occupied = [], set()
    restore_rejections = []
    planned = len(cfg["calibration"])
    for pair in feasible:
        left, right = (samples[pair[k]] for k in ("i", "j"))
        families = {left["case"], right["case"]}
        if families & occupied:
            continue
        center = (command(left["state"]) + command(right["state"])) / 2.
        panel = pulse_panel(center, profile.hover_thrust_N, cfg["pulse"])
        if planned + 2 * (1 + len(panel)) > max_rollouts:
            break
        checks = [restore_check(s, plant, limits, verify, center) for s in (left, right)]
        if any(c is None for c in checks):
            restore_rejections.append(dict(i=pair["i"], j=pair["j"]))
            continue
        chosen.append(dict(anchor=pair["anchor"], kind=pair["kind"], samples=[serial_sample(left), serial_sample(right)],
                           measurements=pair_measure(left, right, command, goal, profile, layers),
                           restore_checks=checks, panel={name: u.tolist() for name, u in panel.items()}))
        planned += 2 * (1 + len(panel))
        occupied.update(families)

    calibration = []
    for key in cfg["calibration"]:
        anchor, = [s for s in nominal if all(s[k] == v for k, v in key.items())]
        sample, = [s for s in samples if s["case"] == anchor["case"] and s["frame"] == anchor["frame"]
                   and s["trace"] == "phase3"]
        expected, = [r for r in prior["rows"] if r["case"] == anchor["case"] and r["frame"] == anchor["frame"]
                     and r["variant"] == "nominal" and r["controller"] == "fixed_a"]
        for k in MEMORY_KEYS:
            np.testing.assert_array_equal(sample[k], anchor[k])
        check = restore_check(sample, plant, limits, verify, command(sample["state"]))
        if check is None:
            raise ValueError("Calibration reset is invalid")
        calibration.append(dict(sample=serial_sample(sample), restore_check=check,
                                expected={k: expected[k] for k in ("qualified", "reason", "end_s")}))
    if time.monotonic() >= deadline or planned > max_rollouts:
        raise TimeoutError("Phase R1.5-M budget exhausted before evaluation")
    for path in (config_path, Path(__file__), phase_r_path, results_path, witness_path, checkpoint, weights,
                 Path(native.__file__), native.HANDOVER_CONFIG):
        source_pins[str(path.resolve().relative_to(ROOT))] = digest(path)
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(configuration=cfg, source_pins=source_pins, actor=actor, model=plant.metadata,
                    motor_time_constant_s=profile.motor_time_constant_s, horizon_s=phase_r["horizon_s"],
                    goal=goal.tolist(), anchors=anchors, natural_records=len(samples), matching=audit,
                    restore_rejections=restore_rejections, pairs=chosen, calibration=calibration,
                    planned_rollouts=planned, region=region,
                    local_ranks=["envelope exit or rejected touch", "safe without region entry",
                                 "safe with region entry", "qualified contact"],
                    training_transitions=0)
    ticks = round(cfg["pulse"]["duration_s"] / profile.policy_dt_s)
    if ticks <= 0 or not np.isclose(ticks * profile.policy_dt_s, cfg["pulse"]["duration_s"], rtol=0, atol=1e-12):
        raise ValueError("Pulse duration must be a whole number of control decisions")
    manifest.update(pulse_control_ticks=ticks, timing_pilot=prior_usage)
    write_result(output / "manifest.json", manifest)
    print(f"Frozen {len(chosen)} independent pairs and {planned} planned rollouts", flush=True)

    attempted, rows, comparisons, replays = 0, [], [], {}
    mesh = native.target_mesh(plant, spec.legal_targets)
    horizon = manifest["horizon_s"]

    def execute(label, sample, pulse=None, replay=False):
        nonlocal attempted
        if attempted >= max_rollouts or time.monotonic() >= deadline:
            raise TimeoutError("Phase R1.5-M evaluation budget reached")
        attempted += 1
        row, outcome, trace = rollout_sample(sample, command, pulse, ticks, plant, horizon, limits, region, cfg)
        row.update(label=label, source={k: sample[k] for k in ("case", "trace", "frame")})
        rows.append(row)
        if replay:
            path = output / f"{label}.npz"
            native.save_replay(path, label, trace, outcome, goal, mesh, profile.policy_dt_s)
            replays[label] = dict(path=path.name, reason=outcome["reason"], sha256=digest(path))
        return row

    def timeout(*_):
        raise TimeoutError("Phase R1.5-M evaluation wall cap reached")

    status = "complete"
    handler = signal.signal(signal.SIGALRM, timeout)
    signal.setitimer(signal.ITIMER_REAL, max(.001, deadline - time.monotonic()))
    try:
        for i, control in enumerate(calibration):
            row = execute(f"calibration-{i}", control["sample"], replay=True)
            if any(row[k] != v for k, v in control["expected"].items()):
                raise RuntimeError("Frozen Phase R calibration outcome changed")
        for i, pair in enumerate(chosen):
            sides = []
            for j, sample in enumerate(pair["samples"]):
                label = f"pair-{i}-side-{j}"
                auto = execute(f"{label}-autonomous", sample, replay=True)
                panel = {k: execute(f"{label}-{k}", sample, np.array(u)) for k, u in pair["panel"].items()}
                sides.append((auto, panel))
            left, right = sides
            comparison = compare_panel(*({k: r["local_rank"] for k, r in side[1].items()} for side in sides))
            divergence = {}
            for k in pair["panel"].keys() | {"autonomous"}:
                l = left[0] if k == "autonomous" else left[1][k]
                r = right[0] if k == "autonomous" else right[1][k]
                divergence[k] = {t: (dict(zip(VISIBLE_KEYS, visible_errors(np.array(state), np.array(r["early_states"][t])).tolist(), strict=True))
                                         if state is not None and r["early_states"][t] is not None else None)
                                 for t, state in l["early_states"].items()}
            comparisons.append(dict(pair=i, kind=pair["kind"],
                autonomous_outcomes=[left[0]["reason"], right[0]["reason"]],
                autonomous_qualification_changed=left[0]["qualified"] != right[0]["qualified"],
                early_divergence=divergence, **comparison))
    except TimeoutError:
        status = "time_limit"
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        signal.signal(signal.SIGALRM, handler)

    informative = [c for c in comparisons if c["kind"] == "activation"]
    relevant = sum(bool(c["changed_viable_actions"] or c["strict_preference_reversals"]
                        or c["autonomous_qualification_changed"]) for c in informative)
    minimum = cfg["minimum_independent_anchors"]
    decision = "inconclusive"
    if status == "complete" and len(informative) >= minimum:
        if relevant >= minimum:
            decision = "aliasing_supported"
        elif relevant == 0:
            decision = "aliasing_not_supported"
    elapsed = time.monotonic() - started
    write_result(output / "results.json", dict(manifest_sha256=digest(output/"manifest.json"), status=status,
        decision=decision, independent_activation_pairs=len(informative), decision_relevant_pairs=relevant,
        attempted_rollouts=attempted, completed_rollouts=len(rows), wall_s=elapsed,
        cumulative_rollouts=prior_rollouts + attempted, cumulative_wall_s=prior_seconds + elapsed,
        reasons=dict(Counter(row["reason"] for row in rows)), rows=rows, comparisons=comparisons,
        replays=replays, training_transitions=0))
    print(f"{decision}: {len(informative)} independent activation pairs, {len(rows)} rollouts", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timing-pilot", type=Path, help="Charge the preserved timing pilot to this budget")
    args = parser.parse_args()
    run(args.config.resolve(), args.output.resolve(), args.timing_pilot)


if __name__ == "__main__":
    main()
