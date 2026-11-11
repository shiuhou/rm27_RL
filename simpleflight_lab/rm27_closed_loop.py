"""Reload a port checkpoint in native MuJoCo for fixed RM27 free-flight cases."""

import argparse
from dataclasses import asdict
from hashlib import sha256
import io
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import torch
from isaaclab.utils.math import matrix_from_quat

from .ppo import Actor, UPSTREAM_REVISION
from .rm27 import RM27Actuator, RM27_SIM_REVISION, load_rm27_profile
from .rm27_compare import NativeVer2, read_sources
from .rm27_legacy import RM27_RL_REVISION, load_legacy_flight
from .stage1_experiment import STAGE1B_CONFIG, STAGE1C_CONFIG
from .task import FlightTask, track_hold_observation


def safety_reason(state, data, legacy):
    if not np.isfinite(state).all() or not np.isfinite(data.act).all():
        return "NUMERICAL_FAILURE"
    if any(w.number for w in data.warning):
        return "MUJOCO_WARNING"
    if any(contact.dist <= 0 and contact.efc_address >= 0 for contact in data.contact):
        return "UNEXPECTED_CONTACT"
    local = (state[:3] - [0.0, 0.0, 1.0]) @ legacy.target_rotation
    local[2] += legacy.normal_standoff_m
    if np.any(local < legacy.safety_low) or np.any(local > legacy.safety_high):
        return "FLIGHT_VOLUME_EXIT"
    if np.linalg.norm(state[7:10]) > legacy.safety_max_speed_m_s:
        return "EXCESSIVE_SPEED"
    if np.linalg.norm(state[10:13]) > legacy.safety_max_body_rate_rad_s:
        return "EXCESSIVE_BODY_RATE"
    return None


@torch.inference_mode()
def rollout(actor, task, legacy, actuator, plant, *, hold_with_track_policy=False, bumper_offset_body_m=None):
    n = len(legacy.names)
    n_controls = task.horizon_ticks
    horizon = n_controls * plant.substeps
    physics = [mujoco.MjData(plant.model) for _ in legacy.names]
    pos, quat, vel = task.reset(torch.arange(n))
    current = np.column_stack((pos.numpy(), quat.numpy(), vel.numpy())).astype(np.float64)
    states = np.full((horizon + 1, n, 13), np.nan)
    physical_ctbr = np.full((n_controls, n, 4), np.nan, dtype=np.float32)
    mixer_saturated = np.zeros((n_controls, n), dtype=bool)
    reasons = [None] * n
    done_tick = np.full(n, horizon)
    for i, data in enumerate(physics):
        plant.reset(data, current[i])
        reasons[i] = safety_reason(current[i], data, legacy)
        if reasons[i] is not None:
            done_tick[i] = 0
    states[0] = current
    policy_steps = torch.zeros(n, dtype=torch.long)
    if bumper_offset_body_m is not None:
        offset = torch.as_tensor(bumper_offset_body_m, dtype=torch.float32)
        virtual_goal = task.reference(policy_steps, lookahead=False)[:, 0] + offset
        normal = torch.as_tensor(legacy.target_rotation[:, 2], dtype=torch.float32).expand(n, 3)
    for control in range(n_controls):
        obs_state = torch.as_tensor(current, dtype=torch.float32)
        if bumper_offset_body_m is None:
            obs = task.observe(obs_state[:, :3], obs_state[:, 3:7], obs_state[:, 7:10],
                               policy_steps.fill_(control), obs_state[:, 10:13])["policy"]
            if hold_with_track_policy:
                obs = track_hold_observation(obs)
        else:
            rotation = matrix_from_quat(obs_state[:, 3:7])
            bumper = obs_state[:, :3] + rotation @ offset
            obs = torch.cat((virtual_goal - bumper, obs_state[:, 7:10], rotation.transpose(-2, -1).flatten(1),
                             obs_state[:, 10:13], normal), dim=-1)
        commands = actuator.decode_command(actor(obs).tanh()).numpy()
        for i, data in enumerate(physics):
            if reasons[i] is not None:
                continue
            if not np.isfinite(commands[i]).all():
                reasons[i] = "NUMERICAL_COMMAND"
                done_tick[i] = control * plant.substeps
                continue
            physical_ctbr[control, i] = commands[i]
            mixer_saturated[control, i] = plant.apply_ctbr(data, commands[i])
            for substep in range(plant.substeps):
                tick = control * plant.substeps + substep + 1
                mujoco.mj_step(plant.model, data)
                state = states[tick, i]
                state[:7] = data.qpos
                state[7:] = data.qvel
                reasons[i] = safety_reason(state, data, legacy)
                if reasons[i] is not None:
                    done_tick[i] = tick
                    break
                current[i] = state
    for i, data in enumerate(physics):
        if abs(data.time - done_tick[i] * plant.dt) > 1e-10:
            raise RuntimeError(f"MuJoCo endpoint clock mismatch in {legacy.names[i]}")
    return states, physical_ctbr, mixer_saturated, reasons, done_tick


def score(legacy, states, reference, reasons, done_tick, saturated, dt, substeps):
    stable_first_tick = round((legacy.horizon_s - legacy.gate.window_s) / dt)
    gate = legacy.gate
    case_results = {}
    for i, name in enumerate(legacy.names):
        done = int(done_tick[i])
        reason = reasons[i]
        tail = states[stable_first_tick:done + 1, i]
        settled = None
        if reason is None:
            distance = np.linalg.norm(tail[:, :3] - reference[i], axis=-1)
            settled = {
                "position_rmse_m": float(np.sqrt(np.mean(distance**2))),
                "position_peak_m": float(distance.max()),
                "speed_peak_m_s": float(np.linalg.norm(tail[:, 7:10], axis=-1).max()),
                "tilt_peak_deg": float(np.rad2deg(np.arccos(
                    np.clip(1 - 2 * np.sum(tail[:, 4:6] ** 2, axis=-1), -1, 1))).max()),
                "body_rate_peak_rad_s": float(np.linalg.norm(tail[:, 10:13], axis=-1).max()),
                "final_error_xyz_m": (tail[-1, :3] - reference[i]).tolist(),
            }
            if not (
                settled["position_peak_m"] <= gate.position_m
                and settled["speed_peak_m_s"] <= gate.speed_m_s
                and settled["tilt_peak_deg"] <= gate.tilt_deg
                and settled["body_rate_peak_rad_s"] <= gate.body_rate_rad_s
            ):
                reason = "NOT_STABLE_AT_HORIZON"
        n_applied = (done + substeps - 1) // substeps
        case_results[name] = {
            "pass": reason is None, "reason": reason, "settled": settled,
            "sim_time_s": done * dt,
            "mixer_saturated_controls": int(saturated[:n_applied, i].sum()),
        }
    return case_results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Fresh output prefix for the JSON and NPZ")
    parser.add_argument("--hold-with-track-policy", action="store_true",
                        help="Use a Track checkpoint for the stationary Hover starts")
    parser.add_argument("--stage1b-experiment", type=Path, default=STAGE1B_CONFIG,
                        help="Pinned config for the flight path of a Stage 1-B checkpoint")
    parser.add_argument("--stage1c-flight", choices=("hover", "shift"),
                        help="Run the Stage 1-C actor on a frozen flight suite using a virtual bumper goal")
    parser.add_argument("--stage1c-experiment", type=Path, default=STAGE1C_CONFIG,
                        help="Pinned Pilot config for a Stage 1-C checkpoint")
    parser.add_argument("--bridge-config", type=Path, help="Separate pinned config for a braking bridge checkpoint")
    args = parser.parse_args()
    if args.output.with_suffix(".json").exists() or args.output.with_suffix(".npz").exists():
        parser.error(f"Choose a fresh output prefix: {args.output}")
    checkpoint_bytes = args.checkpoint.read_bytes()
    saved = torch.load(io.BytesIO(checkpoint_bytes), map_location="cpu", weights_only=True)
    profile = load_rm27_profile()
    expected = {
        "format": 1, "airframe": "rm27", "upstream_revision": UPSTREAM_REVISION,
        "rm27_sim_revision": RM27_SIM_REVISION, "policy_dt": profile.policy_dt_s,
    }
    residual = saved["task"] == "rm27_stage1b"
    pilot = saved["task"] == "rm27_stage1c"
    if pilot != bool(args.stage1c_flight) or (pilot and args.hold_with_track_policy):
        parser.error("Use --stage1c-flight hover|shift with a Stage 1-C checkpoint only")
    if args.bridge_config is not None and not pilot:
        parser.error("A bridge config requires a Stage 1-C compatible checkpoint")
    policy_task = "track" if residual else saved["task"]
    bumper_offset = None
    if residual:
        from .stage1_native_eval import load_residual_policy

        policy, _ = load_residual_policy(saved, args.stage1b_experiment)
        actor = policy.flight
    elif pilot:
        from .stage1_native_eval import load_stage1c_policy

        actor, geometry = load_stage1c_policy(saved, args.stage1c_experiment, args.bridge_config)
        bumper_offset = next(g["position_m"] for g in geometry["aircraft"] if g["name"] == "drone_1_stage1_contact")
        np.testing.assert_array_equal(bumper_offset, profile.bumper_offset_body_m)
    else:
        if any(saved.get(key) != value for key, value in expected.items()):
            raise ValueError("Checkpoint does not match the pinned RM27 actor and dynamics contract")
        actor = Actor(saved["actor_obs"])
        actor.load_state_dict(saved["actor"])
        actor.eval()
    if args.hold_with_track_policy and policy_task != "track":
        parser.error("--hold-with-track-policy requires a Track checkpoint")
    if pilot:
        mode = "hover" if args.stage1c_flight == "hover" else "track"
    else:
        mode = "hover" if args.hold_with_track_policy else policy_task
    legacy = load_legacy_flight(mode)
    if pilot:
        np.testing.assert_allclose(legacy.target_rotation[:, 2], geometry["target"]["normal"], atol=1e-12, rtol=0)
    n = len(legacy.names)
    task = FlightTask(mode, n, "cpu", profile.policy_dt_s, rm27=True,
                      evaluation=True, eval_profile="legacy", legacy_flight=legacy)
    plant = NativeVer2(*read_sources())
    actuator = RM27Actuator(profile, n, "cpu")
    states, commands, saturated, reasons, done_tick = rollout(
        actor, task, legacy, actuator, plant, hold_with_track_policy=args.hold_with_track_policy,
        bumper_offset_body_m=bumper_offset,
    )
    reference = task.reference(torch.zeros(n, dtype=torch.long), lookahead=False)[:, 0].numpy()
    cases = score(legacy, states, reference, reasons, done_tick, saturated, plant.dt, plant.substeps)
    count = sum(case["pass"] for case in cases.values())
    summary = {
        "pass": count >= legacy.gate.min_pass_count,
        "success_count": count,
        "episodes": n,
        "finish_count": sum(reason is None for reason in reasons),
        "scope": "Isolated nominal Ver2 free flight with translated starts and goals. Safety and terminal windows checked at 1 ms.",
        "checkpoint": str(args.checkpoint),
        "checkpoint_sha256": sha256(checkpoint_bytes).hexdigest(),
        "task": mode, "policy_task": policy_task, "training_update": saved["update"],
        "training_frames": saved["frames"],
        "upstream_revision": UPSTREAM_REVISION, "rm27_sim_revision": RM27_SIM_REVISION,
        "rm27_rl_revision": RM27_RL_REVISION, "profile_sha256": profile.profile_sha256,
        "physics_dt_s": plant.dt, "policy_dt_s": profile.policy_dt_s,
        "gate": asdict(legacy.gate), "cases": cases, **plant.metadata,
    }
    if residual:
        summary.update(checkpoint_task=saved["task"], flight_dispatch="immutable Track without contact residual",
                       track_source_checkpoint_sha256=saved["source_checkpoint_sha256"])
    if pilot:
        summary.update(
            flight_suite=args.stage1c_flight,
            flight_dispatch="Standalone Stage 1-C actor with a virtual bumper goal",
            input_adapter={"goal": "Frozen COM reference plus upright body fixture offset",
                           "actual_bumper": "COM plus current body-to-world rotation of fixture offset",
                           "bumper_offset_body_m": bumper_offset,
                           "surface_normal_world": legacy.target_rotation[:, 2].tolist()},
        )
        summary["scope"] += " This off-task flight diagnostic adapts inputs for the contact actor."
    if args.bridge_config is not None:
        summary["bridge_config_sha256"] = saved["bridge_config_sha256"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(args.output.with_suffix(".npz"), states=states, reference=reference, ctbr_N_rad_s=commands,
                        mixer_saturated=saturated, dt=plant.dt, done_tick=done_tick, names=legacy.names)
    from .monitoring import publish_evaluation
    publish_evaluation(summary, args.output, source="mujoco_independent", controller="track_control" if residual else "ppo")
    print(f"RM27_MUJOCO_CLOSED_LOOP {mode} {count}/{n} success, {summary['finish_count']}/{n} full flights")
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
