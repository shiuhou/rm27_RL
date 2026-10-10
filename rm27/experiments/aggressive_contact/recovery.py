"""Empirical free-air recovery map on the same rotor-level plant/controller.

Initial states are initialized once; all subsequent motion comes from motors.
This finite grid estimates a basin for this controller, not a viability kernel.
"""
import itertools
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .control import Skill
from .evidence import save_failure, snapshot, write_json
from .model import TargetState, build_model, scenario, truth_estimate, unit


def recover(cfg, initial, output=None):
    model, data, info, xml = build_model(scenario(cfg), cfg)
    # Recovery-only experiment has ground collision, but no target/rim collision.
    for i in range(model.ngeom):
        name = model.geom(i).name
        if name == "armor" or name.startswith("rim_"):
            model.geom_contype[i] = model.geom_conaffinity[i] = 0
    data.qpos[:3] = [.5, 0, initial["altitude_m"]]
    q = Rotation.from_euler("xyz", [initial["roll_deg"], initial["pitch_deg"], 0], degrees=True).as_quat()
    data.qpos[3:7] = q[[3, 0, 1, 2]]
    data.qvel[:] = np.r_[initial["velocity_m_s"], initial["omega_rad_s"]]
    data.act[:] *= initial["motor_hover_fraction"]
    mujoco.mj_forward(model, data)
    target = TargetState(np.array(cfg["target"]["center_m"]), unit(cfg["target"]["normal"]), np.zeros((3, 3)), 0.)
    skill = Skill(cfg, cfg["nominal_skill"], truth_estimate(model, data), target, info)
    skill.contact_time = 0.
    skill.recovery_anchor = data.qpos[:3].copy()
    g, dt = cfg["gates"], cfg["physics_dt_s"]
    steps_per_control = round(cfg["control_dt_s"]/dt)
    stable_start = recovery_time = None
    min_height, max_rate, max_tilt = initial["altitude_m"], 0., 0.
    sat_run = max_sat_run = 0.
    saturated = False
    reasons, trace = set(), []
    for i in range(round(4/dt)):
        s = truth_estimate(model, data)
        tilt = float(np.rad2deg(np.arccos(np.clip(s.rotation[2, 2], -1, 1))))
        rate, speed = float(np.linalg.norm(s.omega_body)), float(np.linalg.norm(s.velocity))
        max_rate, max_tilt, min_height = max(max_rate, rate), max(max_tilt, tilt), min(min_height, s.position[2])
        stable = tilt < g["max_recovery_tilt_deg"] and rate < g["max_recovery_rate_rad_s"] and speed < g["max_recovery_speed_m_s"]
        stable_start = (s.timestamp if stable_start is None else stable_start) if stable else None
        if stable_start is not None and s.timestamp-stable_start >= g["recovery_hold_s"] and recovery_time is None:
            recovery_time = stable_start
        for c in data.contact:
            if c.dist <= 0 and model.geom("ground").id in c.geom:
                reasons.add("GROUND_CONTACT")
        if i % steps_per_control == 0:
            data.ctrl[:], saturated = skill.command(s, False)
            trace.append(np.r_[s.timestamp, data.qpos.copy(), data.qvel.copy(), data.act.copy(), data.ctrl.copy()])
        sat_run = sat_run + dt if saturated else 0.
        max_sat_run = max(max_sat_run, sat_run)
        if "GROUND_CONTACT" in reasons:
            break
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
    if recovery_time is None or recovery_time > g["recovery_deadline_s"]:
        reasons.add("RECOVERY_TIMEOUT")
    if initial["altitude_m"]-min_height > g["max_altitude_loss_m"]:
        reasons.add("ALTITUDE_LOSS")
    if max_sat_run > g["max_saturation_run_s"]:
        reasons.add("ACTUATOR_SATURATION")
    if max_rate > g["max_body_rate_rad_s"]:
        reasons.add("EXCESSIVE_BODY_RATE")
    result = dict(passed=not reasons, initial=initial, recovery_time_s=recovery_time,
                  min_altitude_m=float(min_height), altitude_loss_m=float(initial["altitude_m"]-min_height),
                  max_body_rate_rad_s=max_rate, max_tilt_deg=max_tilt, max_saturation_run_s=max_sat_run,
                  failure_modes=sorted(reasons), mode="HEADLESS_RECOVERY_ONLY_TRUTH_TEACHER")
    if output is not None:
        output = Path(output)
        output.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(output/"trajectory.npz", trace=np.array(trace))
        mujoco.mj_saveModel(model, str(output/"model.mjb"))
        write_json(output/"summary.json", result)
    return result


def recovery_map(cfg, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    snapshot(output, cfg)
    rows = []
    # 75 fixed grid trials and 24 reproducible multiaxis draws; all declared before execution.
    states = [dict(roll_deg=0., pitch_deg=p, altitude_m=h, velocity_m_s=[.4, 0, -.3],
                   omega_rad_s=[0, w, 0], motor_hover_fraction=1.)
              for h, p, w in itertools.product([.6, 1.2, 1.8], [-60, -30, 0, 30, 60], [-12, -6, 0, 6, 12])]
    rng = np.random.default_rng(92311)
    for _ in range(24):
        states.append(dict(roll_deg=float(rng.uniform(-50, 50)), pitch_deg=float(rng.uniform(-60, 60)),
                           altitude_m=float(rng.uniform(.6, 1.8)), velocity_m_s=rng.uniform(-.6, .6, 3).tolist(),
                           omega_rad_s=rng.uniform(-8, 8, 3).tolist(), motor_hover_fraction=float(rng.uniform(.6, 1.2))))
    write_json(output/"initial_states.json", states)
    for i, state in enumerate(states):
        try:
            row = recover(cfg, state, output/f"trial-{i:03d}")
        except Exception as exc:
            row = save_failure(output/f"trial-{i:03d}", exc)
            row["initial"] = state
        rows.append(row)
        with (output/"results.jsonl").open("a") as f:
            import json
            f.write(json.dumps(row)+"\n")
    result = dict(passed=not any("RUNNER_ERROR" in r["failure_modes"] for r in rows),
                  passed_scope="map execution completed; individual recoverability is in each row",
                  successes=sum(r["passed"] for r in rows), trials=len(rows), grid_trials=75,
                  random_trials=24, rows=rows, scope="empirical controller basin, not formal viability or contact robustness")
    write_json(output/"summary.json", result)
    return result
