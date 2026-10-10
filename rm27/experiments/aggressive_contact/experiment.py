"""Event evaluator independent of policy, full-episode success, raw replay state."""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from rm27.experiments.contact_metrics import armor_contacts, point_approach_speed
from .control import Skill
from .model import TargetState, build_model, frame, scenario, truth_estimate, unit, target_frame


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
    armor = model.geom("armor").id
    bumper = model.geom("bumper").id
    target = TargetState(np.array(cfg["target"]["center_m"]), unit(cfg["target"]["normal"]),
                         np.eye(3)*cfg["uncertainty"]["target_position_std_m"]**2,
                         episode_start-cfg["uncertainty"]["target_latency_s"])
    skill = Skill(cfg, parameters, truth_estimate(model, data), target, info) if initial_skill is None else initial_skill
    g = cfg["gates"]
    normal = unit(physical["target"]["normal"])
    target_R = target_frame(physical["target"])
    center = np.array(physical["target"]["center_m"])
    half = np.array(physical["target"]["dimensions_m"][:2])/2
    dt, control_steps = cfg["physics_dt_s"], round(cfg["control_dt_s"]/cfg["physics_dt_s"])
    first, reasons, events, trace = None, set(), [], []
    observer_trace, phases = [], []
    last_touch, secondary = None, False
    contact_latch = False
    separated, recovery_time, hold_start = False, None, None
    contact_height = None
    post_min, post_max = np.inf, -np.inf
    max_rate = max_speed = peak_force = impulse = sat_run = max_sat_run = 0.
    sat_steps = steps = 0
    saturated = False
    min_distance = np.inf
    last_phase = None
    max_tilt = max_flight_speed = 0.
    min_altitude = np.inf
    recovery_distance = None
    tracking_squared, tracking_samples = 0., 0
    mid_approach_min_speed = np.inf
    pre = truth_estimate(model, data)
    for step in range(round(cfg["horizon_s"]/dt)):
        # mj_forward after mj_step makes contact/state timestamps consistent.
        state = truth_estimate(model, data)
        max_tilt = max(max_tilt, float(np.rad2deg(np.arccos(np.clip(state.rotation[2, 2], -1, 1)))))
        max_flight_speed = max(max_flight_speed, float(np.linalg.norm(state.velocity)))
        min_altitude = min(min_altitude, float(state.position[2]))
        if first is None and state.timestamp-episode_start <= parameters["duration_s"]:
            error = state.position-skill.trajectory.sample(state.timestamp-episode_start)[0]
            tracking_squared += float(error @ error)
            tracking_samples += 1
            if .2*parameters["duration_s"] < state.timestamp-episode_start < .85*parameters["duration_s"]:
                mid_approach_min_speed = min(mid_approach_min_speed, float(np.linalg.norm(state.velocity)))
        touching, target_names = [], []
        for i in range(data.ncon):
            c = data.contact[i]
            if c.dist > 0 or c.efc_address < 0:
                continue
            names = [model.geom(int(v)).name for v in c.geom]
            if (model.body("uav").id in model.geom_bodyid[c.geom] and "armor" not in names
                    and not any(n.startswith("rim_") for n in names) and "ground" not in names):
                reasons.add("ENVIRONMENT_CONTACT")
            if "ground" in names:
                reasons.add("GROUND_CONTACT")
            if "armor" in names or any(n.startswith("rim_") for n in names):
                touching.append((i, c, names))
                target_names.extend(names)
        contact_latch = contact_latch or bool(touching)
        pc = state.position + state.rotation @ physical["contact"]["offset_body_m"]
        min_distance = min(min_distance, float(np.linalg.norm(pc-center)))
        if touching:
            if last_touch is not None and state.timestamp-last_touch > g["secondary_contact_gap_s"]:
                secondary = True
            last_touch = state.timestamp
            if first is None:
                # ALL simultaneous first pairs matter: a simultaneous guard hit invalidates a bumper hit.
                valid_part = all(set(names) == {"armor", "bumper"} for _, _, names in touching)
                c = touching[0][1]
                local = target_R.T @ (c.pos-center)
                r_world = np.asarray(c.pos) - pre.position
                point_velocity = pre.velocity + np.cross(pre.rotation @ pre.omega_body, r_world)
                vn = point_approach_speed(pre.velocity, pre.rotation @ pre.omega_body, r_world, np.zeros(3), normal)
                vt = float(np.linalg.norm(point_velocity + vn*normal))
                angle = float(np.rad2deg(np.arccos(np.clip(pre.rotation[:, 2] @ normal, -1, 1))))
                edge = bool(np.any(np.abs(local[:2]) > half-g["edge_margin_m"]))
                outward = c.frame.reshape(3, 3)[0] * (1 if c.geom[0] == armor else -1)
                front_face = bool(float(outward @ normal) > .9)
                first = dict(time_s=state.timestamp, pairs=[names for _, _, names in touching],
                             valid_part=valid_part, edge=edge, front_face=front_face, position_world_m=c.pos.tolist(),
                             position_error_m=float(np.linalg.norm(local[:2])),
                             normal_speed_m_s=vn, tangent_speed_m_s=vt,
                             attitude_error_deg=angle, body_rate_rad_s=float(np.linalg.norm(pre.omega_body)),
                             region_margin_m=float(np.min(half-np.abs(local[:2]))),
                             local_coordinates_m=local.tolist(), motor_state_N=data.act.tolist(),
                             estimated_specific_thrust_ratio=skill.specific_thrust_ratio,
                             pre_velocity_world_m_s=pre.velocity.tolist(), pre_omega_body_rad_s=pre.omega_body.tolist())
                first.update(pre_position_world_m=pre.position.tolist(), pre_rotation_body_to_world=pre.rotation.tolist(),
                             pre_state_timestamp_s=pre.timestamp)
                if not valid_part:
                    reasons.add("WRONG_FIRST_CONTACT")
                if edge:
                    reasons.add("EDGE_CONTACT")
                if not front_face:
                    reasons.add("WRONG_TARGET_FACE")
                if vt > g["max_contact_tangent_speed_m_s"]:
                    reasons.add("EXCESSIVE_TANGENTIAL_SLIP")
                if not g["contact_normal_speed_range_m_s"][0] <= vn <= g["contact_normal_speed_range_m_s"][1]:
                    reasons.add("CONTACT_SPEED_OUT_OF_RANGE")
                if angle > g["max_contact_attitude_error_deg"]:
                    reasons.add("CONTACT_ATTITUDE_ERROR")
                contact_height = state.position[2]
                events.append(dict(time_s=state.timestamp, event="FIRST_CONTACT", **{k: v for k, v in first.items() if k != "time_s"}))
            # Later wrong-part contacts remain failures even if within the same contact episode.
            if any(set(names) != {"armor", "bumper"} for _, _, names in touching):
                reasons.add("UNDESIGNATED_CONTACT")
            rows = armor_contacts(model, data, armor, normal)
            force = sum(r["compressive_normal_force_N"] for r in rows)
            impulse += force*dt
            peak_force = max(peak_force, force)
        if first is not None:
            post_min, post_max = min(post_min, state.position[2]), max(post_max, state.position[2])
            max_rate = max(max_rate, float(np.linalg.norm(state.omega_body)))
            max_speed = max(max_speed, float(np.linalg.norm(state.velocity)))
            clearance = float((pc-center) @ normal - physical["contact"]["radius_m"])
            if clearance > g["separation_m"] and not touching:
                separated = True
            tilt = np.rad2deg(np.arccos(np.clip(state.rotation[2, 2], -1, 1)))
            stable = (clearance > g["separation_m"] and not touching and tilt <= g["max_recovery_tilt_deg"]
                      and np.linalg.norm(state.omega_body) <= g["max_recovery_rate_rad_s"]
                      and np.linalg.norm(state.velocity) <= g["max_recovery_speed_m_s"])
            hold_start = (state.timestamp if hold_start is None else hold_start) if stable else None
            if hold_start is not None and state.timestamp-hold_start >= g["recovery_hold_s"] and recovery_time is None:
                recovery_time = hold_start-first["time_s"]
                recovery_distance = float(np.linalg.norm(state.position-first["position_world_m"]))
                events.append(dict(time_s=state.timestamp, event="RECOVERED", stable_since_s=hold_start))
            if (np.any(state.position < g["recovery_volume_min_m"]) or
                    np.any(state.position > g["recovery_volume_max_m"])):
                reasons.add("RECOVERY_VOLUME_EXIT")
        if step % control_steps == 0:
            data.ctrl[:], saturated = skill.command(state, contact_latch)
            contact_latch = False
            if skill.phase != last_phase:
                events.append(dict(time_s=state.timestamp, event="PHASE", phase=skill.phase))
                last_phase = skill.phase
            if output is not None:
                trace.append(np.r_[state.timestamp, data.qpos.copy(), data.qvel.copy(), data.act.copy(),
                                   data.ctrl.copy(), skill.last_reference, len(touching), saturated,
                                   skill.last_reference_state])
                observer_trace.append(np.r_[skill.motor_estimate,skill.specific_thrust_ratio])
                phases.append(skill.phase)
            if observe is not None:
                if observe(model, data, skill.phase) is False:
                    reasons.add("VIEWER_CLOSED")
                    break
        sat_steps += int(saturated)
        sat_run = sat_run+dt if saturated else 0.
        max_sat_run = max(max_sat_run, sat_run)
        steps += 1
        if not np.isfinite(np.r_[data.qpos, data.qvel]).all():
            reasons.add("NUMERICAL_FAILURE")
            break
        if "GROUND_CONTACT" in reasons:
            break
        if ((stop_on_recovery and recovery_time is not None)
                or (stop_after_contact_s is not None and first is not None and state.timestamp-first['time_s'] >= stop_after_contact_s)):
            if output is not None and step % control_steps:
                trace.append(np.r_[state.timestamp, data.qpos.copy(), data.qvel.copy(), data.act.copy(),
                                   data.ctrl.copy(), skill.last_reference, len(touching), saturated,
                                   skill.last_reference_state])
                observer_trace.append(np.r_[skill.motor_estimate,skill.specific_thrust_ratio])
                phases.append(skill.phase)
            break
        pre = state
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
    if skill.invalid_target:
        reasons.add("ESTIMATOR_FAILURE")
    if first is None:
        reasons.add("TARGET_MISS")
    else:
        if not separated:
            reasons.add("NO_SEPARATION")
        if secondary:
            reasons.add("SECONDARY_CONTACT")
        if max_rate > g["max_body_rate_rad_s"]:
            reasons.add("EXCESSIVE_BODY_RATE")
        if recovery_time is None or recovery_time > g["recovery_deadline_s"]:
            reasons.add("RECOVERY_TIMEOUT")
        if contact_height-post_min > g["max_altitude_loss_m"]:
            reasons.add("ALTITUDE_LOSS")
    if max_sat_run > g["max_saturation_run_s"]:
        reasons.add("ACTUATOR_SATURATION")
    if any(data.warning[i].number for i in range(len(data.warning))):
        reasons.add("SIMULATOR_WARNING")
    valid_contact = bool(first and first["valid_part"] and first["front_face"] and not first["edge"])
    result = dict(passed=not reasons, seed=seed, uncertainty_scale=scale, parameters=parameters,
                  first_contact=first, valid_contact=valid_contact, separated=separated,
                  secondary_contact=secondary, recovery_time_s=recovery_time,
                  recovery_distance_m=recovery_distance, max_tilt_deg=max_tilt,
                  max_flight_speed_m_s=max_flight_speed, min_altitude_m=min_altitude,
                  precontact_position_rmse_m=float(np.sqrt(tracking_squared/max(1, tracking_samples))),
                  mid_approach_min_speed_m_s=None if np.isinf(mid_approach_min_speed) else mid_approach_min_speed,
                  max_post_body_rate_rad_s=max_rate, max_post_speed_m_s=max_speed,
                  min_post_altitude_m=None if first is None else float(post_min),
                  max_post_altitude_m=None if first is None else float(post_max),
                  altitude_loss_m=None if first is None else float(max(0., contact_height-post_min)),
                  peak_normal_force_N=peak_force, normal_impulse_Ns=impulse,
                  saturation_fraction=sat_steps/max(steps, 1), max_saturation_run_s=max_sat_run,
                  closest_bumper_center_m=min_distance, failure_modes=sorted(reasons),
                  duration_s=float(data.time), referee_scoring_success=None,
                  episode_elapsed_s=float(data.time-episode_start),
                  estimated_specific_thrust_ratio=skill.specific_thrust_ratio,
                  mode="HEADLESS_MUJOCO_CLOSED_LOOP_MOTOR_CONTROL_TRUTH_TEACHER", events=events)
    result['prefix_only'] = stop_after_contact_s is not None
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
