"""Phase R1.5-M2. Probe one fresh, continuously integrated actuator-memory pair."""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import signal
import time

import mujoco
import numpy as np

from . import actuator_memory as memory
from . import terminal_native_eval as native
from .asset import ROOT, digest, read_json, write_result
from .flight import allocate_ctbr
from .handover_curriculum import native_verifier


ANCHOR = ("dev_center", 691, "transition_post")
PRIOR = ROOT / ".agents/runs/asm2-frontier/actuator-memory/main"
MAX_ROLLOUTS = 100
MAX_SECONDS = 300
BRANCH_OFFSET = .14


def branch_commands(center, hover):
    """Change only collective for one decision at the common branch root."""
    commands = {}
    for label, sign in (("minus", -1), ("plus", 1)):
        command = np.array(center, dtype=float)
        command[0] += sign * BRANCH_OFFSET * hover
        if not (.5 * hover <= command[0] <= 1.5 * hover and np.all(np.abs(command[1:]) <= 2.)):
            raise ValueError("Branch command exceeds the frozen CTBR bounds")
        commands[label] = command
    return commands


def matched_pair(measurement, cfg, hover):
    delta = np.abs(np.diff(measurement["ctbr"], axis=0)[0])
    return bool(
        all(measurement["visible_residual"][key] <= cfg["visible_limits"][key] for key in memory.VISIBLE_KEYS)
        and delta[0] <= cfg["actor_limits"]["collective_fraction_hover"] * hover
        and delta[1:].max() <= cfg["actor_limits"]["rate_rad_s"]
        and np.max(np.abs(measurement["activation_delta_N"])) >= cfg["min_memory_delta_N"]
    )


def snapshot(plant, state, goal, command, label, frame, case):
    integration_mask = mujoco.mjtState.mjSTATE_INTEGRATION
    integration = np.empty(mujoco.mj_stateSize(plant.model, integration_mask))
    mujoco.mj_getState(plant.model, plant.data, integration, integration_mask)
    return dict(case=case, trace=label, frame=frame, state=state.copy(),
                motor_thrust_N=plant.data.act.copy(), motor_target_N=plant.data.ctrl.copy(),
                branch_time_s=plant.tick * plant.dt, actor_input=memory.actor_features(state, goal),
                actor_ctbr=command(state), mj_integration_state=integration)


def contact_margins(outcome, spec):
    """Signed distances to selected frozen first-touch criteria, for attribution only."""
    touch = outcome["contact"]["first_touch"]
    if not touch:
        return None
    limits = spec.config["thresholds"]
    points = touch["records"]
    xyz = np.abs([p["target_position_m"] for p in points])
    speeds = [p["normal_approach_speed_m_s"] for p in points]
    return dict(
        footprint_xy_m=(spec.region_halfsize_m - xyz[:, :2].max(axis=0)).tolist(),
        plane_m=float(limits["target_plane_error_max_m"] - xyz[:, 2].max()),
        normal_speed_m_s=min(min(speeds) - limits["normal_speed_min_m_s"],
                             limits["normal_speed_max_m_s"] - max(speeds)),
        tangent_speed_m_s=limits["tangent_speed_max_m_s"] - max(p["tangential_speed_m_s"] for p in points),
        alignment_deg=limits["alignment_max_deg"] - max(
            max(p["body_to_target_alignment_deg"], p["surface_normal_error_deg"], p["patch_normal_error_deg"])
            for p in points),
    )


def run(output):
    started = time.monotonic()
    old_manifest, old_result = (PRIOR / n for n in ("manifest.json", "results.json"))
    old, old_results = map(read_json, (old_manifest, old_result))
    if old_results["status"] != "complete" or old_results["manifest_sha256"] != digest(old_manifest):
        raise ValueError("Incomplete or changed Phase R1.5-M evidence")
    cfg = old["configuration"]
    if read_json(memory.CONFIG) != cfg:
        raise ValueError("Phase R1.5-M configuration changed")
    frontier = ROOT / cfg["frontier_run"]
    phase_path, prior_path = (frontier / n for n in ("manifest.json", "results.json"))
    phase, prior = map(read_json, (phase_path, prior_path))
    if prior["status"] != "complete" or prior["manifest_sha256"] != digest(phase_path):
        raise ValueError("Incomplete or changed Phase R evidence")
    case, frame, _ = ANCHOR
    if any(s["case"] == case for p in old["pairs"] for s in p["samples"]):
        raise ValueError("Follow-up anchor must come from an independent witness")
    anchor, = [s for s in phase["samples"] if
               (s["case"], s["frame"], s["phase"], s["variant"]) == (*ANCHOR, "nominal")]
    prior_trials = [{k: r[k] for k in ("variant", "controller", "reason", "end_s", "qualified")}
                    for r in prior["rows"] if (r["case"], r["frame"], r["phase"]) == ANCHOR]
    _, profile, spec, _, _, goal = native.load_evaluation_inputs(ROOT / cfg["experiment"])
    plant = native.NativeContact(profile, spec)
    actor = old["actor"]
    if plant.metadata != phase["model"] or goal.tolist() != phase["goal"] or actor != phase["controllers"]["fixed_a"]:
        raise ValueError("Frozen model, actor or goal changed")
    checkpoint = ROOT / actor["checkpoint"]
    weights = checkpoint.with_suffix(".npz")
    command, pin = native.load_controller(weights, digest(checkpoint), profile, goal)
    if any(actor[k] != v for k, v in pin.items()):
        raise ValueError("Fixed A checkpoint or portable actor changed")
    with np.load(weights, allow_pickle=False) as data:
        layers = native.NativeActor.layers(data, 45)
    limits = read_json(native.HANDOVER_CONFIG)["flight_envelope"]
    verify = native_verifier(plant)
    hover, dt = profile.hover_thrust_N, profile.policy_dt_s
    root = tuple(np.asarray(anchor[k]) for k in memory.MEMORY_KEYS)
    center = command(root[0])
    if memory.restore_check(anchor | dict(zip(memory.MEMORY_KEYS, root)), plant, limits, verify, center) is None:
        raise ValueError("Invalid common branch root")
    commands = branch_commands(center, hover)
    targets, saturated = allocate_ctbr(profile, np.asarray(list(commands.values())),
                                       np.broadcast_to(root[0][10:], (2, 3)))
    if saturated.any():
        raise ValueError("Branch allocation saturates")
    predicted = (targets[1] - targets[0]) * (-np.expm1(-dt / profile.motor_time_constant_s))
    ticks = old["pulse_control_ticks"]
    if ticks != 4 or not np.isclose(ticks * dt, cfg["pulse"]["duration_s"], rtol=0., atol=1e-12):
        raise ValueError("Corrected Phase R1.5-M pulse timing changed")
    pins = dict(phase["pins"])
    for path in (old_manifest, old_result, phase_path, prior_path, Path(__file__), Path(memory.__file__),
                 memory.CONFIG, checkpoint, weights):
        pins[str(path.relative_to(ROOT))] = digest(path)
    for source, expected in pins.items():
        if digest(ROOT / source) != expected:
            raise ValueError(f"Frozen input changed: {source}")
    if old["source_pins"][str(Path(memory.__file__).relative_to(ROOT))] != digest(memory.__file__):
        raise ValueError("Corrected pulse implementation changed")
    output.mkdir(parents=True, exist_ok=False)
    protocol_path = output / "protocol.json"
    manifest_path = output / "manifest.json"
    write_result(protocol_path, dict(
        schema="rm27_asm2_actuator_memory_followup_v1", source_pins=pins, actor=actor, model=plant.metadata,
        anchor=anchor, archived_phase_r_trials=prior_trials,
        selection="Independent witness family, first braking boundary, Fixed A 1/3 and capped controller 3/3",
        branch_commands={k: v.tolist() for k, v in commands.items()}, branch_duration_s=dt,
        predicted_activation_delta_N=predicted.tolist(),
        history="Actor is feedforward. Record both decision-boundary input vectors and integration states.",
        matching={k: cfg[k] for k in ("visible_limits", "actor_limits", "min_memory_delta_N")},
        region=old["region"], pulse=cfg["pulse"], pulse_control_ticks=ticks, local_horizon_s=cfg["local_horizon_s"],
        sample_times_s=cfg["sample_times_s"], horizon_s=phase["horizon_s"], limits=limits, goal=goal.tolist(),
        interpretation="Require repeatable robust changes in viable actions or ordinal preferences. "
                       "Score timing or footprint boundary flips alone leave attribution unresolved.",
        max_rollouts=MAX_ROLLOUTS, max_seconds=MAX_SECONDS, training_transitions=0))

    attempted = 0
    histories, branch_rows, rows, replays = {}, {}, {}, {}
    comparison, receipt = None, None
    status = "complete"
    error = None
    mesh = native.target_mesh(plant, spec.legal_targets)

    def deadline(*_):
        raise TimeoutError("Phase R1.5-M2 evaluation budget reached")

    def charge():
        nonlocal attempted
        if attempted >= MAX_ROLLOUTS or time.monotonic() - started >= MAX_SECONDS:
            deadline()
        attempted += 1

    def save(label, trace, outcome):
        path = output / f"{label}.npz"
        native.save_replay(path, label, trace, outcome, goal, mesh, dt)
        replays[label] = dict(path=path.name, sha256=digest(path), reason=outcome["reason"])

    handler = signal.signal(signal.SIGALRM, deadline)
    signal.setitimer(signal.ITIMER_REAL, max(.001, MAX_SECONDS - (time.monotonic() - started)))
    try:
        for label, pulse in commands.items():
            history = []

            def branch(state):
                history.append(snapshot(plant, state, goal, command, label, frame, case))
                return pulse

            charge()
            outcome, trace = native.rollout(plant, *root, branch, dt, limits)
            branch_rows[label] = outcome
            if outcome["reason"] == "NO_SIGNED_TOUCH" and outcome["contact"]["positive_separation_compressive_records"] == 0:
                np.testing.assert_array_equal(trace[2], pulse[None])
                if outcome["saturation_controls"] or not np.isclose(outcome["end_s"], dt, rtol=0., atol=1e-12):
                    raise RuntimeError("Invalid one-decision branch")
                history.append(snapshot(plant, trace[0][-1], goal, command, label, frame + 1, case))
            histories[label] = history
        panel, measurement, checks = {}, None, []
        if all(len(h) == 2 for h in histories.values()):
            left_history, right_history = histories.values()
            for key in (*memory.MEMORY_KEYS, "actor_input", "actor_ctbr", "mj_integration_state"):
                np.testing.assert_array_equal(left_history[0][key], right_history[0][key])
            pair = [h[-1] for h in histories.values()]
            measurement = memory.pair_measure(*pair, command, goal, profile, layers)
            np.testing.assert_allclose(measurement["activation_delta_N"], predicted, rtol=0., atol=1e-12)
            shared = np.mean(measurement["ctbr"], axis=0)
            checks = [memory.restore_check(s, plant, limits, verify, shared) for s in pair]
            if matched_pair(measurement, cfg, hover) and all(c is not None for c in checks):
                panel = memory.pulse_panel(shared, hover, cfg["pulse"])
                if len(panel) != 9:
                    raise ValueError("The unchanged nine-action panel is outside CTBR limits at this pair")
        write_result(manifest_path, dict(protocol_sha256=digest(protocol_path),
            branch_histories={k: [memory.serial_sample(s) for s in h] for k, h in histories.items()},
            branch_results=branch_rows, measurements=measurement, restore_checks=checks,
            valid_pair=bool(panel), panel={k: v.tolist() for k, v in panel.items()},
            planned_rollouts=2 + (2 * (1 + len(panel)) if panel else 0)))
        if panel:
            selection = spec.field_geometry["selection"]
            local_window = cfg["local_horizon_s"]
            for sample in pair:
                side = sample["trace"]
                rows[side] = {}
                for name, pulse in [("autonomous", None), *panel.items()]:
                    charge()
                    row, outcome, trace = memory.rollout_sample(
                        sample, command, pulse, ticks, plant, phase["horizon_s"], limits, old["region"], cfg)
                    if pulse is not None and row["pulse_controls"] != ticks:
                        raise RuntimeError("Truncated local pulse")
                    states, times, _ = trace
                    local = states[times <= local_window + 1e-9]
                    xyz = (local[:, :3] - spec.target_center) @ spec.target_rotation
                    row["local_volume_margin_m"] = float(np.minimum(
                        xyz - selection["safety_lower_target_frame_m"],
                        selection["safety_upper_target_frame_m"] - xyz).min())
                    row["local_speed_margin_m_s"] = float(
                        limits["max_com_speed_m_s"] - np.linalg.norm(local[:, 7:10], axis=1).max())
                    row["region_occupied_at_reset"] = row["first_region_entry_s"] == 0.
                    row["first_touch_margins"] = contact_margins(outcome, spec)
                    row["native_contact"] = outcome["contact"]
                    rows[side][name] = row
                    if name == "autonomous" or row["qualified"] and not any(
                            r["reason"] == "QUALIFIED" for r in replays.values()):
                        save(f"{side}-{name}", trace, outcome)
            left, right = rows.values()
            comparison = memory.compare_panel(*({k: side[k]["local_rank"] for k in panel} for side in (left, right)))
            comparison["matched"] = {
                name: dict(
                    outcomes=[left[name]["reason"], right[name]["reason"]],
                    ranks=[left[name]["local_rank"], right[name]["local_rank"]],
                    early_residuals={
                        t: dict(zip(memory.VISIBLE_KEYS, memory.visible_errors(np.asarray(s),
                                    np.asarray(right[name]["early_states"][t])).tolist()))
                        if s is not None and right[name]["early_states"][t] is not None else None
                        for t, s in left[name]["early_states"].items()})
                for name in left}
            receipt = dict(
                visible_initial_velocity_residual_m_s=measurement["visible_residual"]["velocity_m_s"],
                common_target_motor_velocity_scale_m_s=measurement["common_target_memory_velocity_scale_m_s"],
                first_replacement_target_residual_max_N=float(np.max(np.abs(np.diff(
                    [c["target_after_command_N"] for c in checks], axis=0)))))
    except TimeoutError:
        status = "time_limit"
    except Exception as exc:
        status, error = "error", str(exc)
        raise
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        signal.signal(signal.SIGALRM, handler)
        write_result(output / "results.json", dict(
            status=status, error=error, protocol_sha256=digest(protocol_path),
            manifest_sha256=digest(manifest_path) if manifest_path.is_file() else None,
            attempted_rollouts=attempted, completed_branches=len(branch_rows),
            completed_probes=sum(map(len, rows.values())), wall_s=time.monotonic() - started,
            training_transitions=0, comparison=comparison, initial_residual_scales=receipt,
            outcomes={side: dict(Counter(row["reason"] for row in probes.values())) for side, probes in rows.items()},
            rows=rows, replays=replays))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    run(parser.parse_args().output.resolve())


if __name__ == "__main__":
    main()
