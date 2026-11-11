"""Headless Isaac Lab entry point for SimpleFlight smoke, PPO and evaluation."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

from isaaclab.app import AppLauncher

CURVE_FIELDS = ("update", "frames", "rollout_s", "update_s", "samples_per_s", "step_reward",
                "position_rmse_m", "done_count", "episode_return", "episode_ticks", "actor_loss",
                "value_loss", "actor_grad", "critic_grad", "approx_kl", "gpu_used_mib", "torch_peak_mib")


def args_for_run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("smoke", "bench", "train", "eval"))
    parser.add_argument("--airframe", choices=("crazyflie", "rm27"), default="crazyflie")
    parser.add_argument("--task", choices=("hover", "track"), default="hover")
    parser.add_argument("--num-envs", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--updates", type=int, help="Total PPO updates, including any resumed updates")
    parser.add_argument("--max-seconds", type=float, help="Stop training at the next checkpoint after this many seconds")
    parser.add_argument("--output", type=Path, help="Training directory or evaluation file prefix")
    parser.add_argument("--checkpoint", type=Path, help="Required for eval; optional resume checkpoint for train")
    parser.add_argument("--hold-with-track-policy", action="store_true",
                        help="Evaluate a goal-conditioned RM27 Track checkpoint on stationary Hover references")
    parser.add_argument("--profile", choices=("standard", "stress", "position", "normal", "legacy", "precision"),
                        default="standard",
                        help="Evaluation profile, Hover stress training, or RM27 Track precision continuation")
    parser.add_argument("--initial-std", type=float, default=1.0, help="Raw Gaussian initial standard deviation")
    parser.add_argument("--hover-thrust-bias", action="store_true", help="Initialize the collective mean at level hover thrust")
    args = parser.parse_args()
    if args.num_envs is None:
        args.num_envs = {"smoke": 4, "bench": 512, "train": 2048, "eval": 64}[args.command]
        if args.profile == "legacy":
            args.num_envs = 19 if args.task == "hover" else 4
    if args.seed is None:
        args.seed = 127 if args.command == "eval" else 0
    if args.updates is None:
        args.updates = {"smoke": 1, "bench": 4, "train": 120, "eval": 1}[args.command]
    if args.num_envs < (4 if args.command == "smoke" else 1) or args.updates < 1 or args.initial_std <= 0:
        parser.error("positive environment count, update count and initial std required (smoke needs at least 4 worlds)")
    if args.command == "eval" and args.checkpoint is None:
        parser.error("eval requires --checkpoint")
    if args.hold_with_track_policy and (args.command, args.airframe, args.task) != ("eval", "rm27", "hover"):
        parser.error("--hold-with-track-policy requires eval --airframe rm27 --task hover")
    if args.command in ("train", "eval") and args.output is None:
        parser.error("train and eval require --output")
    hover_stress_train = args.command == "train" and args.task == "hover" and args.profile == "stress"
    if args.profile == "precision" and (
        (args.command, args.airframe, args.task) != ("train", "rm27", "track") or args.checkpoint is None
    ):
        parser.error("precision requires RM27 Track training with --checkpoint")
    if args.command != "eval" and args.profile not in ("standard", "precision") and not hover_stress_train:
        parser.error("nonstandard profiles are for eval, Hover stress training or RM27 precision")
    if args.profile == "legacy" and args.airframe != "rm27":
        parser.error("the legacy flight evaluation is specific to RM27")
    if args.command == "bench" and args.updates < 2:
        parser.error("bench needs a warm-up and at least one measured update")
    return args


def check_flight_axes(env):
    import torch

    env.reset()
    ids = torch.arange(env.num_envs, device=env.device)
    pose = torch.zeros((env.num_envs, 7), device=env.device)
    pose[:, :3] = env.scene.env_origins
    pose[:, 2] += 1.0
    pose[:, 3] = 1.0
    robot = env.scene.rigid_objects[env.cfg.airframe]
    robot.write_root_pose_to_sim(pose, ids)
    robot.write_root_velocity_to_sim(torch.zeros_like(pose[:, :6]), ids)
    env.sim.forward()
    expected_mass, expected_inertia = env.mass_kg, env.inertia_kg_m2
    actual_mass = robot.root_physx_view.get_masses().flatten()
    if not torch.allclose(actual_mass, torch.full_like(actual_mass, expected_mass), atol=1e-6):
        raise RuntimeError(f"PhysX mass differs from actuator mass {expected_mass}: {actual_mass[:4]}")
    inertia = robot.root_physx_view.get_inertias().reshape(-1, 3, 3).diagonal(dim1=-2, dim2=-1)
    if not torch.allclose(inertia, inertia.new_tensor(expected_inertia).expand_as(inertia), atol=1e-8):
        raise RuntimeError(f"PhysX inertia differs from the aggregate Crazyflie model: {inertia[:4]}")
    action = torch.zeros((env.num_envs, 4), device=env.device)
    action[:, 3] = env.hover_logit
    action[1:4, :3] = torch.eye(3, device=env.device) * 0.25
    for _ in range(12):
        env.step(action)
    pos, _, velocity, body_rate = env.state()
    signed = body_rate[1:4].diag()
    if not bool((signed > 0.01).all() and abs(pos[0, 2] - 1) < 0.002 and abs(velocity[0, 2]) < 0.02):
        raise RuntimeError(f"Direct CTBR hover/axis check failed: z={pos[0, 2]}, vz={velocity[0, 2]}, rates={signed}")
    print(f"CTBR_AXES_PASS z={pos[0, 2]:.5f} m, p/q/r response={signed.tolist()} rad/s")
    if env.cfg.task == "hover" and env.cfg.airframe == "crazyflie":
        env.reset()
        robot.write_root_pose_to_sim(pose, ids)
        robot.write_root_velocity_to_sim(torch.zeros_like(pose[:, :6]), ids)
        env.sim.forward()
        action.zero_()
        action[:, 3] = -4.0
        for _ in range(85):
            env.step(action)
        grounded = env.state()[0][:, 2]
        if not bool(((grounded > 0) & (grounded < 0.04)).all()):
            raise RuntimeError(f"Ground drop failed: z={grounded.tolist()} m")
        print(f"CTBR_GROUND_PASS landed z={grounded.tolist()} m")


def read_checkpoint(path, args):
    import torch

    from .ppo import UPSTREAM_REVISION

    data = torch.load(path, weights_only=True, map_location="cuda")
    policy_task = "track" if args.hold_with_track_policy else args.task
    if data["task"] != policy_task or data["upstream_revision"] != UPSTREAM_REVISION or data["format"] != 1:
        raise ValueError(f"Checkpoint {path} does not match policy task {policy_task}, port format or upstream revision")
    if data.get("airframe", "crazyflie") != args.airframe:
        raise ValueError(f"Checkpoint {path} does not match airframe {args.airframe}")
    if args.airframe == "rm27":
        from .rm27 import RM27_SIM_REVISION

        if data.get("rm27_sim_revision") != RM27_SIM_REVISION:
            raise ValueError(f"Checkpoint {path} does not match the RM27 simulator revision")
    return data


def training(env, args):
    import torch

    from .ppo import GAE_LAMBDA, GAMMA, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION

    if args.command == "smoke":
        check_flight_axes(env)
    obs, _ = env.reset(seed=args.seed)
    if not all(x.is_cuda and bool(x.isfinite().all()) for x in obs.values()):
        raise RuntimeError("Simulator did not produce finite CUDA observations")
    precision = args.profile == "precision"
    if precision:
        # Preserve the SimpleFlight 100 Hz time constants at RM27's 200 Hz.
        ppo = PPO(obs, args.initial_std, learning_rate=2e-4, gamma=math.sqrt(GAMMA), gae_lambda=math.sqrt(GAE_LAMBDA))
    else:
        ppo = PPO(obs, args.initial_std)
    if args.hover_thrust_bias:
        with torch.no_grad():
            ppo.actor.mean[-1].bias[3] = env.hover_logit
    initial_weight = ppo.actor.mean[-1].weight.detach().clone() if args.command == "smoke" else None
    start_update, initial_frames = 1, 0
    if args.checkpoint is not None:
        saved = read_checkpoint(args.checkpoint, args)
        ppo.actor.load_state_dict(saved["actor"])
        if precision and saved.get("training_profile") != "precision":
            # The new reward and reset mix change the value targets. Preserve
            # the policy mean while refreshing the critic and optimizer state.
            with torch.no_grad():
                ppo.actor.log_std.clamp_(max=math.log(0.12))
        else:
            ppo.critic.load_state_dict(saved["critic"])
            ppo.normalizer.load_state_dict(saved["value_normalizer"])
            ppo.actor_opt.load_state_dict(saved["actor_optimizer"])
            ppo.critic_opt.load_state_dict(saved["critic_optimizer"])
        start_update, initial_frames = saved["update"] + 1, saved["frames"]
        if start_update > args.updates:
            raise ValueError("--updates must exceed the checkpoint's update number")

    def checkpoint(update, frames, filename="checkpoint.pt"):
        path = args.output / filename
        temp = path.with_suffix(".tmp")
        provenance = {"airframe": args.airframe}
        if args.airframe == "rm27":
            from .rm27 import RM27_SIM_REVISION

            provenance["rm27_sim_revision"] = RM27_SIM_REVISION
        torch.save({"format": 1, "task": args.task, "upstream_revision": UPSTREAM_REVISION, "update": update,
                    **provenance,
                    "frames": frames, "seed": args.seed, "training_profile": args.profile,
                    "num_envs": args.num_envs, "policy_dt": env.step_dt,
                    "actor_obs": obs["policy"].shape[-1], "critic_obs": obs["critic"].shape[-1],
                    "actor": ppo.actor.state_dict(), "critic": ppo.critic.state_dict(),
                    "value_normalizer": ppo.normalizer.state_dict(), "actor_optimizer": ppo.actor_opt.state_dict(),
                    "critic_optimizer": ppo.critic_opt.state_dict()}, temp)
        temp.replace(path)

    curve = None
    writer = None
    if args.output is not None:
        args.output.mkdir(parents=True, exist_ok=True)
        curve_path = args.output / "training.csv"
        append = args.checkpoint is not None and curve_path.exists()
        curve = curve_path.open("a" if append else "x", newline="")
        writer = csv.writer(curve)
        if not append:
            writer.writerow(CURVE_FIELDS)
            curve.flush()
    samples = env.num_envs * ROLLOUT_STEPS
    rows = []
    selection_update = start_update - 1 + math.ceil(0.75 * (args.updates - start_update + 1))
    from .monitoring import Monitor, training_metrics
    monitor = Monitor((args.output or Path(".agents/runs/simpleflight")) / "monitoring",
                      (args.output.name if args.output else args.command),
                      metadata={"kind": "isaac-training-proxy", "airframe": args.airframe,
                                "task": args.task, "profile": args.profile})
    start = time.perf_counter()
    try:
        for i in range(start_update, args.updates + 1):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            obs, rollout = ppo.collect(env, obs)
            torch.cuda.synchronize()
            t1 = time.perf_counter()
            update = ppo.update()
            torch.cuda.synchronize()
            t2 = time.perf_counter()
            free, total = torch.cuda.mem_get_info()
            frames = initial_frames + (i - start_update + 1) * samples
            training_stats = torch.cat((rollout, update)).tolist()
            if not all(math.isfinite(x) for x in training_stats):
                raise RuntimeError(f"Nonfinite PPO update {i}: {training_stats}")
            row = (i, frames, t1 - t0, t2 - t1, samples / (t2 - t0), *training_stats,
                   (total - free) / 2**20, torch.cuda.max_memory_allocated() / 2**20)
            rows.append(dict(zip(CURVE_FIELDS, row, strict=True)))
            if monitor.process is not None:
                monitor.log(training_metrics(rows[-1], optimization=training_stats[5:10],
                                             log_std=ppo.actor.log_std.detach().cpu().tolist()), step=frames)
            if writer:
                writer.writerow(row)
                curve.flush()
            elapsed = t2 - start
            final = i == args.updates or (args.max_seconds is not None and elapsed >= args.max_seconds)
            if args.command in ("smoke", "bench") or i == start_update or i % 10 == 0 or final:
                print(f"{args.task} update {i}/{args.updates} frames={frames:,} "
                      f"reward={row[5]:.3f} RMSE={row[6]:.3f} m ep_len={row[9]:.0f} "
                      f"rollout={row[2]:.2f}s ppo={row[3]:.2f}s {row[4]:,.0f} samples/s "
                      f"GPU used={row[-2]:.0f} MiB torch_peak={row[-1]:.0f} MiB elapsed={elapsed:.0f}s")
            if args.command == "train" and (i % 10 == 0 or final):
                checkpoint(i, frames)
            if precision and i == selection_update and not final:
                checkpoint(i, frames, "selection.pt")
            if final:
                break
    finally:
        if curve:
            curve.close()
        monitor.finish()
    if args.command == "smoke":
        if torch.equal(initial_weight, ppo.actor.mean[-1].weight):
            raise RuntimeError("PPO optimizer failed to update the actor")
        print("PPO_INTEGRATION_PASS")
    elif args.command == "bench":
        from statistics import median
        print("BENCH " + json.dumps({"task": args.task, "envs": env.num_envs, "timed_updates": len(rows) - 1,
              "median_samples_per_s": median(r["samples_per_s"] for r in rows[1:]),
              "sampled_max_gpu_mib": max(r["gpu_used_mib"] for r in rows),
              "torch_peak_mib": max(r["torch_peak_mib"] for r in rows)}))


def evaluate(env, args):
    import numpy as np
    import torch

    from .ppo import Actor, UPSTREAM_REVISION
    from .task import track_hold_observation

    saved = read_checkpoint(args.checkpoint, args)
    actor = Actor(saved["actor_obs"]).to(env.device)
    actor.load_state_dict(saved["actor"])
    actor.eval()
    obs, _ = env.reset(seed=args.seed)
    horizon = env.max_episode_length
    alive = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    finished = torch.zeros_like(alive)
    first_done = torch.full((env.num_envs,), horizon, dtype=torch.long, device=env.device)
    trace = {}
    with torch.inference_mode():
        for t in range(horizon):
            policy_obs = track_hold_observation(obs["policy"]) if args.hold_with_track_policy else obs["policy"]
            obs, _, terminated, truncated, info = env.step(actor(policy_obs))
            for key, values in info["flight"].items():
                if t == 0:
                    trace[key] = torch.empty((horizon, *values.shape), device=env.device)
                trace[key][t] = values.masked_fill(~alive[:, None], torch.nan)
            finished |= alive & truncated & ~terminated & (t == horizon - 1)
            done = terminated | truncated
            first_done.masked_fill_(alive & done, t + 1)
            alive &= ~done
    legacy = env.legacy_flight
    settle_s = (legacy.horizon_s - legacy.gate.window_s if legacy else 3.0 if args.task == "hover" else 2.0)
    # First recorded sample is the end of the first tick (t = dt).
    after_settle = max(0, round(settle_s / env.step_dt) - 1)
    full_error = (trace["position"] - trace["reference"]).norm(dim=-1)
    error = full_error[after_settle:]
    rmse = error.square().mean(dim=0).sqrt()
    peak = error.amax(dim=0)
    valid = torch.stack([v.isfinite().all(dim=(0, 2)) for v in trace.values()]).all(dim=0)
    pass_rule = {"settle_s": settle_s, "min_success_fraction": 0.9}
    if args.task == "hover" or legacy:
        velocity = trace["linear_velocity"][after_settle:]
        speed_rms = velocity.square().sum(dim=-1).mean(dim=0).sqrt()
        quat = trace["quat"][after_settle:]
        tilt = torch.rad2deg((1 - 2 * quat[..., 1:3].square().sum(dim=-1)).clamp(-1, 1).acos())
        tilt_peak = tilt.amax(dim=0)
        if legacy:
            g = legacy.gate
            speed_peak = velocity.norm(dim=-1).amax(dim=0)
            rate_peak = trace["angular_velocity"][after_settle:].norm(dim=-1).amax(dim=0)
            pass_rule = {"settle_s": settle_s, "min_pass_count": g.min_pass_count,
                         "max_error_m": g.position_m, "speed_max_m_s": g.speed_m_s,
                         "tilt_max_deg": g.tilt_deg, "body_rate_max_rad_s": g.body_rate_rad_s}
            settled = (speed_peak <= g.speed_m_s) & (tilt_peak <= g.tilt_deg) & (rate_peak <= g.body_rate_rad_s)
        else:
            pass_rule.update(rmse_m=0.15, max_error_m=0.25, speed_rms_m_s=0.25, tilt_max_deg=20.0)
            settled = (speed_rms <= pass_rule["speed_rms_m_s"]) & (tilt_peak <= pass_rule["tilt_max_deg"])
    else:
        pass_rule.update(rmse_m=0.25, max_error_m=0.50)
        settled = torch.ones_like(finished)
    success = finished & valid & settled & (peak <= pass_rule["max_error_m"])
    if not legacy:
        success &= rmse <= pass_rule["rmse_m"]
    if args.airframe == "rm27":
        position = trace["position"]
        success &= (position[..., 2] > 0.1).all(dim=0)
        pass_rule["min_altitude_m"] = 0.1
        if legacy:
            frame = position.new_tensor(legacy.target_rotation)
            target_frame = ((position - position.new_tensor([0.0, 0.0, 1.0])) @ frame
                            + position.new_tensor([0.0, 0.0, legacy.normal_standoff_m]))
            success &= ((target_frame >= position.new_tensor(legacy.safety_low))
                        & (target_frame <= position.new_tensor(legacy.safety_high))).all(dim=(0, 2))
            success &= (trace["linear_velocity"].norm(dim=-1) <= legacy.safety_max_speed_m_s).all(dim=0)
            success &= (trace["angular_velocity"].norm(dim=-1) <= legacy.safety_max_body_rate_rad_s).all(dim=0)

    def distribution(tensor):
        finite = tensor[tensor.isfinite()]
        if not finite.numel():
            return {"median": None, "p90": None, "max": None}
        return {"median": finite.quantile(0.5).item(), "p90": finite.quantile(0.9).item(), "max": finite.max().item()}

    reference_span = (trace["reference"].nan_to_num(nan=-torch.inf).amax(dim=0)
                      - trace["reference"].nan_to_num(nan=torch.inf).amin(dim=0)).norm(dim=-1)
    if args.profile in ("position", "legacy") and bool((reference_span > 1e-6).any()):
        raise RuntimeError("Stationary target changed during its recorded first episode")

    minimum_successes = legacy.gate.min_pass_count if legacy else math.ceil(env.num_envs * pass_rule["min_success_fraction"])
    summary = {"task": args.task, "policy_task": saved["task"], "airframe": args.airframe, "profile": args.profile,
               "checkpoint": str(args.checkpoint),
               "upstream_revision": UPSTREAM_REVISION, "training_seed": saved["seed"], "training_update": saved["update"],
               "training_frames": saved["frames"], "training_profile": saved.get("training_profile", "standard"),
               "seed": args.seed, "episodes": env.num_envs, "horizon_ticks": horizon,
               "dt_s": env.step_dt, "finish_count": int(finished.sum()), "success_count": int(success.sum()),
               "pass": int(success.sum()) >= minimum_successes, "gate": pass_rule,
               "position_rmse_m_completed": distribution(rmse), "position_peak_m_completed": distribution(peak),
               "initial_error_m": distribution(full_error[0]), "full_episode_rmse_m_completed": distribution(full_error.square().mean(dim=0).sqrt()),
               "reference_span_m": distribution(reference_span)}
    if args.task == "hover":
        summary.update(speed_rms_m_s_completed=distribution(speed_rms), max_tilt_deg_completed=distribution(tilt_peak),
                       final_2cm_count=int((finished & (full_error[-1] <= 0.02)).sum()))
    if args.airframe == "rm27":
        summary["rm27_sim_revision"] = saved["rm27_sim_revision"]
        saturation = trace["mixer_saturated"][:, :, 0]
        summary["motor_saturation_fraction"] = distribution(saturation.nanmean(dim=0))
        if legacy:
            summary.update(case_success=dict(zip(legacy.names, success.tolist(), strict=True)),
                           terminal_speed_peak_m_s=distribution(speed_peak),
                           terminal_body_rate_peak_rad_s=distribution(rate_peak),
                           terminal_tilt_peak_deg=distribution(tilt_peak))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    np.savez_compressed(args.output.with_suffix(".npz"), **{k: v.cpu().numpy() for k, v in trace.items()},
                        seed=args.seed, dt=env.step_dt, task=args.task, profile=args.profile,
                        finished=finished.cpu().numpy(), success=success.cpu().numpy(),
                        first_done_tick=first_done.cpu().numpy(), rmse_m=rmse.cpu().numpy())
    print("EVAL " + json.dumps(summary, allow_nan=False))


def main():
    args = args_for_run()
    launcher = AppLauncher(headless=True, device="cuda:0")
    app = launcher.app
    env = None
    # Kit's fast shutdown may terminate the process directly; preserve Python failures.
    try:
        import torch

        from .env import FlightEnv, FlightEnvCfg

        torch.set_float32_matmul_precision("high")
        cfg = FlightEnvCfg()
        cfg.airframe = args.airframe
        cfg.task = args.task
        cfg.seed = args.seed
        cfg.scene.num_envs = args.num_envs
        cfg.sim.device = "cuda:0"
        cfg.evaluation = args.command == "eval"
        cfg.eval_profile = args.profile
        env = FlightEnv(cfg)
        print(f"RUN {args.command} airframe={args.airframe} task={args.task} envs={env.num_envs} "
              f"physics_dt={env.physics_dt} policy_dt={env.step_dt} seed={args.seed}")
        if cfg.evaluation:
            evaluate(env, args)
        else:
            training(env, args)
        env.close()
    except Exception:
        import os
        import sys
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    print("SIMPLEFLIGHT_RUN_COMPLETE", flush=True)
    app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
