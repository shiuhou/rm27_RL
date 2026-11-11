"""Train the explicitly authorized, bounded Stage 1-C handover Pilot."""

import argparse
from collections import Counter
import csv
import fcntl
import math
import os
import signal
import sys
import time
import traceback

from .stage1_experiment import (STAGE1C_CONFIG, STAGE1C_EXPERIMENT, STAGE1C_RUN,
                                digest, load_stage1c_experiment, read_json)
from .stage1_training import LearningBudget, LearningLimit, make_environment, write_json


SEGMENTS = ("informed_near", "informed_rotation", "informed_approach", "stationary")
OUTCOMES = ("episodes", "qualified", "rejected_contact", "unsafe_flight", "timeout")


def train(candidate, settings, config, assets, frozen, geometry, budget, output):
    import torch

    from .ppo import GAE_LAMBDA, GAMMA, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from .rm27 import RM27_SIM_REVISION
    from .stage1_sim import Stage1Sim
    from .stage1_task import Stage1Actor
    from .stage1c_resets import Stage1CStarts
    from .stage1c_task import Stage1CTask

    torch.manual_seed(settings["seed"])
    gamma = settings.get("gamma", math.sqrt(GAMMA))
    starts = Stage1CStarts.from_witnesses(assets["witness_1p0"].parent)
    sim = Stage1Sim(settings["num_envs"], assets["isaac_scene"], device="cuda:0")
    if (sim.physics_dt_s, sim.control_dt_s) != (frozen["physics_dt_s"], frozen["policy_dt_s"]):
        raise RuntimeError("Stage 1-C simulator timing does not match the frozen task")
    task = Stage1CTask(sim.num_envs, sim.device, geometry, frozen, starts=starts, gamma=gamma,
                      goal_inset_m=config["goal_inset_m"], nominal_standoff_m=config["nominal_standoff_m"],
                      reward_config=settings.get("reward"))
    env = make_environment(sim, task, frozen, geometry, budget, informed_resets=True)
    env.reset(env.ids)
    obs = env.observe()
    if obs["policy"].shape != (sim.num_envs, 21) or not all(
            value.is_cuda and bool(value.isfinite().all()) for value in obs.values()):
        raise RuntimeError("Stage 1-C did not initialize finite CUDA observations")
    actor = Stage1Actor(settings["initial_std"]).to(sim.device)
    source = torch.load(assets["checkpoint"], map_location=sim.device, weights_only=True)
    if (source["format"] != 1 or source["task"] != "track" or source["actor_obs"] != 45
            or source["airframe"] != "rm27" or source["upstream_revision"] != UPSTREAM_REVISION
            or source["rm27_sim_revision"] != RM27_SIM_REVISION):
        raise ValueError("Stage 1-C requires the original validated Track initialization")
    actor.load_track_state_dict(source["actor"], initial_std=settings["initial_std"])
    ppo = PPO(obs, settings["initial_std"], actor=actor, learning_rate=settings["learning_rate"],
              gamma=gamma, gae_lambda=math.sqrt(GAE_LAMBDA))

    config_sha = digest(STAGE1C_CONFIG)
    updates = frames = saved_update = 0
    totals, window = Counter(), Counter()
    consecutive = 0
    promotions = []
    milestones = iter((1_000_000, 3_000_000, 6_000_000))
    milestone = next(milestones)
    status = "ERROR"
    update_in_progress = False
    rollout_size = sim.num_envs * ROLLOUT_STEPS

    def save(name="checkpoint.pt"):
        path = output / name
        temporary = path.with_suffix(".tmp")
        torch.save({"format": 1, "task": "rm27_stage1c", "experiment": STAGE1C_EXPERIMENT,
                    "config_sha256": config_sha, "source_checkpoint_sha256": config["assets"]["checkpoint"]["sha256"],
                    "upstream_revision": UPSTREAM_REVISION, "rm27_sim_revision": RM27_SIM_REVISION,
                    "policy_dt": sim.control_dt_s, "actor_obs": 21, "seed": settings["seed"],
                    "candidate": candidate, "goal_inset_m": config["goal_inset_m"],
                    "frontier_index": task.frontier_index, "update": updates, "frames": frames,
                    "actor": actor.state_dict(), "critic": ppo.critic.state_dict(),
                    "value_normalizer": ppo.normalizer.state_dict(), "reward_config": task.pilot_reward,
                    "actor_optimizer": ppo.actor_opt.state_dict(), "critic_optimizer": ppo.critic_opt.state_dict()},
                   temporary)
        temporary.replace(path)

    fields = ("update", "frames", "learning_s", "samples_per_s", "frontier", "reward", "position_rmse_m", "kl",
              "physical_contacts", "unsafe_contacts", *OUTCOMES,
              *(f"{segment}_{key}" for segment in (*SEGMENTS, "frontier") for key in OUTCOMES),
              "mean_episode_min_error_m", "mean_saturation_fraction", "gpu_used_mib")

    def stop_at_deadline(_signum, _frame):
        budget.stop_reason = "TIME_LIMIT"

    output.mkdir(parents=True)
    write_json(output / "settings.json", settings | {"effective_transition_cap": budget.max_frames,
                                                      "effective_learning_s_cap": budget.max_seconds,
                                                      "reward": task.pilot_reward, "gamma": gamma})
    save("initial.pt")
    print(f"STAGE1C_PPO_START candidate={candidate} worlds={sim.num_envs} "
          f"transition_cap={budget.max_frames:,} learning_s_cap={budget.max_seconds:.1f}", flush=True)
    signal.signal(signal.SIGALRM, stop_at_deadline)
    torch.cuda.synchronize()
    budget.started = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, budget.max_seconds - 1.0)
    try:
        with (output / "training.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            while budget.frames + rollout_size <= budget.max_frames:
                budget.check()
                start = time.perf_counter()
                obs, rollout = ppo.collect(env, obs)
                if not bool(torch.isfinite(rollout).all() & torch.isfinite(ppo.rollout["advantage"]).all()):
                    raise RuntimeError(f"Nonfinite Stage 1-C rollout after {budget.frames:,} charged transitions")
                update_in_progress = True
                optimization = ppo.update(before_minibatch=budget.check)
                values = torch.cat((rollout, optimization)).tolist()
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError(f"Nonfinite Stage 1-C update after {budget.frames:,} charged transitions")
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

                promotion = config["promotion"]
                if task.frontier_index + 1 < task.num_frontiers and window["frontier_episodes"] >= promotion["window_episodes"]:
                    ratio = window["frontier_qualified"] / window["frontier_episodes"]
                    consecutive = consecutive + 1 if ratio >= promotion["min_fraction"] else 0
                    if consecutive >= promotion["consecutive_windows"]:
                        band = task.frontier_index
                        task.set_frontier(band + 1)
                        promotions.append(dict(update=updates, frames=frames, from_frontier=band,
                                               to_frontier=task.frontier_index, final_window=dict(window)))
                        save(f"frontier-{task.frontier_index}.pt")
                        print(f"STAGE1C_PROMOTION {promotions[-1]}", flush=True)
                        consecutive = 0
                    window.clear()

                if milestone is not None and frames >= milestone:
                    save(f"development-{milestone}.pt")
                    milestone = next(milestones, None)
                if updates == 1 or updates % 20 == 0:
                    save()
                    saved_update = updates
                    print(f"STAGE1C_PPO u{updates} frames={frames:,} t={budget.elapsed:.0f}s "
                          f"frontier={task.frontier_index} qualified={totals['qualified']}/{totals['episodes']} "
                          f"stationary={totals['stationary_qualified']}/{totals['stationary_episodes']} "
                          f"{row['samples_per_s']:,.0f}/s GPU={row['gpu_used_mib']:.0f}MiB", flush=True)
            status = "TRANSITION_LIMIT"
    except LearningLimit as stop:
        status = str(stop)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        elapsed = budget.elapsed
        if status != "ERROR" and updates and not update_in_progress and updates != saved_update:
            save()
            saved_update = updates
        write_json(output / "learning.json", dict(status=status, frames=budget.frames, completed_frames=frames,
                   learning_s=elapsed, updates=updates, checkpoint_update=saved_update, frontier_index=task.frontier_index,
                   config_sha256=config_sha, training_counts=dict(totals), failure_modes=dict(env.failure_modes),
                   promotions=promotions, reset_overlap_rejections=env.reset_rejections,
                   representative_contacts=env.examples))
    print(f"STAGE1C_COMPLETE candidate={candidate} {status} frames={budget.frames:,} learning_s={elapsed:.3f}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=int, choices=(1, 2), required=True)
    args = parser.parse_args()
    config, assets, frozen, geometry = load_stage1c_experiment()
    cap = config["budget"]
    STAGE1C_RUN.mkdir(parents=True, exist_ok=True)
    output = STAGE1C_RUN / f"candidate-{args.candidate}"
    with (STAGE1C_RUN / "budget.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if output.exists() or (STAGE1C_RUN / "selection.json").exists():
            raise FileExistsError("Candidate slot or completed selection is already recorded")
        previous = []
        if args.candidate == 1:
            settings = config["candidate_1"]
        else:
            previous.append(read_json(STAGE1C_RUN / "candidate-1/learning.json"))
            settings = read_json(STAGE1C_RUN / "candidate-2-settings.json")
            if not settings["rationale"].strip():
                raise ValueError("A second candidate requires justification based on the first")
        budget = LearningBudget(min(settings["transitions"], cap["transitions"] - sum(run["frames"] for run in previous)),
                                min(settings["learning_s"], cap["learning_s"] - sum(run["learning_s"] for run in previous)))
        if budget.max_frames <= 0 or budget.max_seconds <= 1.0:
            raise ValueError("Stage 1-C Pilot budget is exhausted")
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(headless=True, device="cuda:0")
        exit_code = 0
        try:
            train(args.candidate, settings, config, assets, frozen, geometry, budget, output)
        except Exception:
            traceback.print_exc()
            exit_code = 2
        finally:
            receipt_path = output / "learning.json"
            if receipt_path.exists():
                runs = previous + [read_json(receipt_path)]
                write_json(STAGE1C_RUN / "budget.json", dict(config_sha256=digest(STAGE1C_CONFIG),
                           authorized=cap, candidates=len(runs), frames=sum(run["frames"] for run in runs),
                           learning_s=sum(run["learning_s"] for run in runs)))
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)


if __name__ == "__main__":
    main()
