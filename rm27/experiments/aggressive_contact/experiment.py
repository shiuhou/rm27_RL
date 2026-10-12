"""Whole-episode runner using the shared completed-interval evaluator."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from .control import Skill
from .evaluation import ContactEvaluator
from .model import TargetState, build_model, scenario, truth_estimate, unit


def run_episode(cfg, parameters=None, seed=None, scale=1., output=None, physical=None, record_sources=False,
                plant_bundle=None, initial_skill=None, observe=None, save_model=True, stop_on_recovery=False,
                stop_after_contact_s=None):
    parameters = dict(cfg["nominal_skill"] if parameters is None else parameters)
    physical = scenario(cfg, seed, scale) if physical is None else physical
    if output is not None:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=False)
        (output/"physical.json").write_text(json.dumps(physical, indent=2))
        (output/"config.json").write_text(json.dumps(cfg, indent=2))
        (output/"parameters.json").write_text(json.dumps(parameters, indent=2))
        if record_sources:
            from .evidence import snapshot
            snapshot(output, cfg)
    model, data, info, xml = build_model(physical, cfg) if plant_bundle is None else plant_bundle
    episode_start = float(data.time)
    target = TargetState(np.array(cfg["target"]["center_m"]), unit(cfg["target"]["normal"]),
                         np.eye(3)*cfg["uncertainty"]["target_position_std_m"]**2,
                         episode_start-cfg["uncertainty"]["target_latency_s"])
    state = truth_estimate(model, data)
    skill = Skill(cfg, parameters, state, target, info) if initial_skill is None else initial_skill
    evaluator = ContactEvaluator(cfg, physical, episode_start)
    evaluator.initialize(model, data, state)
    dt, control_steps = cfg["physics_dt_s"], round(cfg["control_dt_s"]/cfg["physics_dt_s"])
    trace, observer_trace, phases = [], [], []
    last_phase = None
    saturated = False
    tracking_squared = tracking_samples = 0
    mid_approach_min_speed = np.inf

    def record():
        if output is not None and (not trace or data.time > trace[-1][0] + 1e-12):
            trace.append(np.r_[data.time, data.qpos.copy(), data.qvel.copy(), data.act.copy(),
                               data.ctrl.copy(), skill.last_reference, evaluator.touching_count, saturated,
                               skill.last_reference_state])
            observer_trace.append(np.r_[skill.motor_estimate, skill.specific_thrust_ratio])
            phases.append(skill.phase)

    def add_teacher_diagnostics():
        # Controller fields remain runner-owned, outside shared task scoring.
        if evaluator.first is not None and "estimated_specific_thrust_ratio" not in evaluator.first:
            ratio = float(skill.specific_thrust_ratio)
            evaluator.first["estimated_specific_thrust_ratio"] = ratio
            for event in evaluator.events:
                if event["event"] == "FIRST_CONTACT":
                    event["estimated_specific_thrust_ratio"] = ratio
                    break

    def update_control():
        nonlocal saturated, last_phase
        data.ctrl[:], saturated = skill.command(state, evaluator.consume_contact())
        if skill.phase != last_phase:
            evaluator.events.append(dict(time_s=state.timestamp, event="PHASE", phase=skill.phase))
            last_phase = skill.phase
        record()
        if observe is not None and observe(model, data, skill.phase) is False:
            evaluator.reasons.add("VIEWER_CLOSED")
            return False
        return True

    add_teacher_diagnostics()
    for step in range(round(cfg["horizon_s"]/dt)):
        if "NUMERICAL_FAILURE" in evaluator.reasons or "GROUND_CONTACT" in evaluator.reasons:
            break
        if evaluator.first is None and state.timestamp-episode_start <= parameters["duration_s"]:
            error = state.position-skill.trajectory.sample(state.timestamp-episode_start)[0]
            tracking_squared += float(error @ error)
            tracking_samples += 1
            if .2*parameters["duration_s"] < state.timestamp-episode_start < .85*parameters["duration_s"]:
                mid_approach_min_speed = min(mid_approach_min_speed, float(np.linalg.norm(state.velocity)))
        if step % control_steps == 0:
            if not update_control():
                break
        state = evaluator.advance(model, data, state, saturated)
        add_teacher_diagnostics()
        if ((stop_on_recovery and evaluator.recovered)
                or (stop_after_contact_s is not None and evaluator.first is not None
                    and state.timestamp-evaluator.first["time_s"] >= stop_after_contact_s-1e-12)):
            # Preserve the old early-stop controller diagnostics at a control
            # boundary, without integrating a nonexistent additional interval.
            if (step + 1) % control_steps == 0:
                update_control()
            record()
            break
    if skill.invalid_target:
        evaluator.reasons.add("ESTIMATOR_FAILURE")
    # A full-horizon trace ends at its last control tick. Mission consumers
    # record their next phase at data.time; adding a final row here would
    # duplicate that handoff timestamp. Endpoint scoring is already consumed.
    result = evaluator.finalize(model, data)
    result.update(seed=seed, uncertainty_scale=scale, parameters=parameters,
                  precontact_position_rmse_m=float(np.sqrt(tracking_squared/max(1, tracking_samples))),
                  mid_approach_min_speed_m_s=None if np.isinf(mid_approach_min_speed) else mid_approach_min_speed,
                  estimated_specific_thrust_ratio=float(skill.specific_thrust_ratio),
                  mode="HEADLESS_MUJOCO_CLOSED_LOOP_MOTOR_CONTROL_TRUTH_TEACHER",
                  prefix_only=stop_after_contact_s is not None)
    if output is not None:
        (output/"scene.xml").write_text(xml)
        # mj_saveModel includes runtime modifications of mass/inertia/gears/lag, unlike XML alone.
        if save_model:
            mujoco.mj_saveModel(model, str(output/"model.mjb"))
        np.savez_compressed(output/"trajectory.npz", trace=np.array(trace),
                            controller_estimate=np.array(observer_trace),phases=np.array(phases),
                            columns=np.array(["t", "x", "y", "z", "qw", "qx", "qy", "qz",
                                              "vx", "vy", "vz", "wx", "wy", "wz", "m0", "m1", "m2", "m3",
                                              "u0", "u1", "u2", "u3", "ref_x", "ref_y", "ref_z", "contacts", "saturated",
                                              "ref_vx", "ref_vy", "ref_vz", "ref_ax", "ref_ay", "ref_az",
                                              "ref_qw", "ref_qx", "ref_qy", "ref_qz", "ref_wx", "ref_wy", "ref_wz",
                                              "force_x", "force_y", "force_z"]))
        (output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def objective(result, recovery_aware):
    """Matched search space/control; only post-contact terms differ."""
    c = result["first_contact"]
    if c is None:
        return 100 + 100*result["closest_bumper_center_m"]
    loss = 30*(not result["valid_contact"]) + 80*c["position_error_m"]
    loss += .04*c["attitude_error_deg"] + .15*c["body_rate_rad_s"]
    loss += 2*c["tangent_speed_m_s"] + .3*result["parameters"]["duration_s"]
    if recovery_aware:
        loss += 35*(not result["passed"]) + 12*result["secondary_contact"]
        loss += 4*(result["recovery_time_s"] if result["recovery_time_s"] is not None else 4)
        loss += .4*result["max_post_body_rate_rad_s"] + 12*(result["altitude_loss_m"] or 0)
    return float(loss)
