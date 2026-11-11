"""Train the single authorized RM27 Stage 1-B Direct-CTBR residual."""

from collections import Counter
import csv
import fcntl
import math
import os
import signal
import sys
import time
import traceback

from .stage1_experiment import RUN, STAGE1B_CONFIG, STAGE1B_EXPERIMENT, digest, load_stage1b_experiment, read_json
from .stage1_training import LearningBudget, LearningLimit, make_environment, write_json


OUTPUT = RUN.parent / "ppo-approach/candidate-1"


def promote_from_result(pending, config, task):
    checkpoint, band = pending
    result = read_json(checkpoint.with_suffix(".json"))
    cases = result["cases"]
    actual_qualified = sum(case["passed"] for case in cases.values())
    if (result["frontier_index"] != band or band != task.frontier_index
            or result["actor_metadata"]["checkpoint_sha256"] != digest(checkpoint)
            or result["scorer_sha256"] != config["assets"]["scorer"]["sha256"]
            or result["frozen_private_scene_sha256"] != config["assets"]["native_scene"]["sha256"]
            or len(cases) != 19 or result["episodes"] != 19 or result["success_count"] != actual_qualified):
        raise ValueError("Stationary frontier evaluation does not match its frozen checkpoint and scene")
    passed = actual_qualified >= config["promotion"]["native_min_qualified"]
    if passed:
        task.set_frontier(band + 1)
    return {"checkpoint": checkpoint.name, "frontier": band, "qualified": actual_qualified,
            "episodes": len(cases), "advanced": passed}


def train(config, assets, frozen, geometry, budget):
    import torch

    from .ppo import Actor, GAE_LAMBDA, GAMMA, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from .rm27 import RM27_SIM_REVISION
    from .stage1_residual import RAW_RESIDUAL_BOUND, Stage1ResidualPolicy
    from .stage1_sim import Stage1Sim
    from .stage1_task import Stage1ResidualTask
    from .task import track_hold_observation

    settings = config["candidate"]
    if tuple(config["raw_bounds"]) != RAW_RESIDUAL_BOUND:
        raise ValueError("Stage 1-B checkpoint bounds must match the contact action path")
    torch.manual_seed(settings["seed"])
    sim = Stage1Sim(settings["num_envs"], assets["isaac_scene"], device="cuda:0")
    if (sim.physics_dt_s, sim.control_dt_s) != (frozen["physics_dt_s"], frozen["policy_dt_s"]):
        raise RuntimeError("Stage 1-B timing differs from the frozen task")
    task = Stage1ResidualTask(sim.num_envs, sim.device, geometry, frozen, training=True,
                              goal_inset_m=config["goal_inset_m"], frontier_clearance_m=config["frontier_clearance_m"])
    source = torch.load(assets["checkpoint"], map_location=sim.device, weights_only=True)
    if (source["format"] != 1 or source["task"] != "track" or source["actor_obs"] != 45
            or source["airframe"] != "rm27" or source["upstream_revision"] != UPSTREAM_REVISION
            or source["rm27_sim_revision"] != RM27_SIM_REVISION):
        raise ValueError("Stage 1-B requires the original selected Track actor")
    track = Actor(45).to(sim.device)
    track.load_state_dict(source["actor"])
    policy = Stage1ResidualPolicy(track, settings["initial_std"]).to(sim.device)
    if any(parameter.requires_grad for parameter in track.parameters()):
        raise RuntimeError("Track parameters must remain frozen")
    env = make_environment(sim, task, frozen, geometry, budget, residual_policy=policy)
    env.reset(env.ids)
    obs = env.observe()
    if not all(value.is_cuda and bool(value.isfinite().all()) for value in obs.values()):
        raise RuntimeError("Stage 1-B did not initialize finite CUDA observations")
    with torch.no_grad():
        flight_probe = track_hold_observation(obs["policy"][:, :18])
        expected_flight = track(flight_probe)
        torch.testing.assert_close(policy.flight(flight_probe), expected_flight, atol=0, rtol=0)
        torch.testing.assert_close(policy.contact_mean(obs["policy"]), expected_flight, atol=0, rtol=0)
    ppo = PPO(obs, settings["initial_std"], actor=policy.residual, learning_rate=settings["learning_rate"],
              gamma=math.sqrt(GAMMA), gae_lambda=math.sqrt(GAE_LAMBDA))
    if {id(p) for p in track.parameters()} & {id(p) for group in ppo.actor_opt.param_groups for p in group["params"]}:
        raise RuntimeError("Track parameters entered the optimizer")
    config_sha = digest(STAGE1B_CONFIG)
    updates, frames = 0, 0
    totals, window = Counter(), Counter()
    validation = []
    latest_saved = 0
    midpoint_saved = False
    best_frontier_score = -1.0
    last_request = -config["promotion"]["retry_transitions"]
    pending = None
    status = "ERROR"
    update_in_progress = False

    def save(name="checkpoint.pt"):
        path = OUTPUT / name
        temporary = path.with_suffix(".tmp")
        torch.save({"format": 1, "task": "rm27_stage1b", "experiment": STAGE1B_EXPERIMENT,
                    "config_sha256": config_sha, "source_checkpoint_sha256": config["assets"]["checkpoint"]["sha256"],
                    "upstream_revision": UPSTREAM_REVISION, "rm27_sim_revision": RM27_SIM_REVISION,
                    "policy_dt": sim.control_dt_s, "actor_obs": obs["policy"].shape[-1], "seed": settings["seed"],
                    "raw_bounds": config["raw_bounds"], "goal_inset_m": config["goal_inset_m"],
                    "frontier_index": task.frontier_index, "update": updates, "frames": frames,
                    "actor": policy.residual.state_dict(), "critic": ppo.critic.state_dict(),
                    "value_normalizer": ppo.normalizer.state_dict()}, temporary)
        temporary.replace(path)
        return path

    fields = ("update", "frames", "learning_s", "samples_per_s", "frontier", "reward", "position_rmse_m", "kl",
              "episodes", "qualified", "unsafe_contacts", "physical_contacts", "unsafe_flight", "timeout",
              "near_episodes", "near_qualified", "intermediate_episodes", "intermediate_qualified", "full_episodes",
              "full_qualified", "frontier_episodes", "frontier_qualified", "frontier_rejected_contact",
              "frontier_timeout", "mean_episode_min_error_m", "mean_saturation_fraction", "gpu_used_mib")

    def stop_at_deadline(_signum, _frame):
        budget.stop_reason = "TIME_LIMIT"

    OUTPUT.mkdir(parents=True)
    save("initial.pt")
    rollout_size = sim.num_envs * ROLLOUT_STEPS
    signal.signal(signal.SIGALRM, stop_at_deadline)
    torch.cuda.synchronize()
    budget.started = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, budget.max_seconds - 1.0)
    try:
        with (OUTPUT / "training.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            while budget.frames + rollout_size <= budget.max_frames:
                budget.check()
                start = time.perf_counter()
                obs, rollout = ppo.collect(env, obs)
                if not bool(torch.isfinite(rollout).all() & torch.isfinite(ppo.rollout["advantage"]).all()):
                    raise RuntimeError(f"Nonfinite Stage 1-B rollout after {budget.frames:,} charged transitions")
                update_in_progress = True
                optimization = ppo.update(before_minibatch=budget.check)
                values = torch.cat((rollout, optimization)).tolist()
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError(f"Nonfinite Stage 1-B update after {budget.frames:,} charged transitions")
                update_in_progress = False
                updates += 1
                frames = budget.frames
                stats = env.interval_stats()
                totals.update({key: value for key, value in stats.items() if isinstance(value, int)})
                window.update({key: stats.get(key, 0) for key in ("frontier_episodes", "frontier_qualified")})
                free, total = torch.cuda.mem_get_info()
                row = dict(update=updates, frames=frames, learning_s=budget.elapsed,
                           samples_per_s=rollout_size / (time.perf_counter() - start), frontier=task.frontier_index,
                           reward=values[0], position_rmse_m=values[1], kl=values[-1], gpu_used_mib=(total-free)/2**20)
                row.update({key: stats.get(key, 0) for key in fields if key not in row})
                writer.writerow(row)
                stream.flush()
                if pending and pending[0].with_suffix(".json").exists():
                    outcome = promote_from_result(pending, config, task)
                    validation.append(outcome)
                    print(f"STAGE1B_FRONTIER_RESULT {outcome}", flush=True)
                    pending = None
                    window.clear()
                    if outcome["advanced"]:
                        best_frontier_score = -1.0
                if window["frontier_episodes"] >= config["promotion"]["window_episodes"]:
                    ratio = window["frontier_qualified"] / window["frontier_episodes"]
                    if ratio > 0.0 and ratio > best_frontier_score:
                        best_frontier_score = ratio
                        save("best-frontier.pt")
                    if (pending is None and ratio >= config["promotion"]["min_fraction"]
                            and task.frontier_index + 1 < task.num_frontiers
                            and frames - last_request >= config["promotion"]["retry_transitions"]):
                        band = task.frontier_index
                        pending = (save(f"promotion-{band}-u{updates}.pt"), band)
                        last_request = frames
                        print(f"STAGE1B_FRONTIER_REVIEW {pending[0].name} training={ratio:.3f}", flush=True)
                    window.clear()
                if not midpoint_saved and frames >= budget.max_frames // 2:
                    save("midpoint.pt")
                    midpoint_saved = True
                if updates == 1 or updates % 10 == 0:
                    save()
                    latest_saved = updates
                    print(f"STAGE1B_PPO u{updates} frames={frames:,} t={budget.elapsed:.0f}s "
                          f"frontier={task.frontier_index} qualified={totals['frontier_qualified']}/"
                          f"{totals['frontier_episodes']} full={totals['full_qualified']}/{totals['full_episodes']} "
                          f"{row['samples_per_s']:,.0f}/s GPU={row['gpu_used_mib']:.0f}MiB", flush=True)
            status = "TRANSITION_LIMIT"
    except LearningLimit as stop:
        status = str(stop)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        elapsed = budget.elapsed
        if status != "ERROR" and updates and not update_in_progress and updates != latest_saved:
            save()
            latest_saved = updates
        with torch.no_grad():
            flight_preserved = torch.equal(policy.flight(flight_probe), expected_flight)
        track_hash_preserved = digest(assets["checkpoint"]) == config["assets"]["checkpoint"]["sha256"]
        write_json(OUTPUT / "learning.json", dict(status=status, frames=budget.frames, completed_frames=frames,
                   learning_s=elapsed, updates=updates, checkpoint_update=latest_saved, frontier_index=task.frontier_index,
                   config_sha256=config_sha, training_counts=dict(totals), failure_modes=dict(env.failure_modes),
                   frontier_reviews=validation, pending_review=None if pending is None else pending[0].name,
                   reset_overlap_rejections=env.reset_rejections, representative_contacts=env.examples,
                   frozen_track_actions_equal=flight_preserved, frozen_track_hash_equal=track_hash_preserved))
        if not flight_preserved or not track_hash_preserved:
            raise RuntimeError("Frozen Track failed the post-learning preservation check")
    print(f"STAGE1B_COMPLETE {status} frames={budget.frames:,} learning_s={elapsed:.3f}", flush=True)


def main():
    config, assets, frozen, geometry = load_stage1b_experiment(STAGE1B_CONFIG)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with (OUTPUT.parent / "budget.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if OUTPUT.exists():
            raise FileExistsError("The one authorized Stage 1-B candidate already has a run directory")
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(headless=True, device="cuda:0")
        budget = LearningBudget(config["budget"]["transitions"], config["budget"]["learning_s"])
        try:
            train(config, assets, frozen, geometry, budget)
        except Exception:
            traceback.print_exc()
            sys.stderr.flush()
            os._exit(2)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
