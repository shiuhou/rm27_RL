"""Bounded GPU PPO for the authorized RM27 Stage 1 contact experiment."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

from .stage1_experiment import CONFIG, EXPERIMENT, RUN, digest, load_experiment, read_json


class LearningLimit(Exception):
    pass


class LearningBudget:
    def __init__(self, frames, seconds):
        self.frames = 0
        self.max_frames = frames
        self.max_seconds = seconds
        self.started = None
        self.stop_reason = None

    @property
    def elapsed(self):
        return 0.0 if self.started is None else time.perf_counter() - self.started

    def check(self):
        if self.stop_reason is not None or self.elapsed >= self.max_seconds:
            raise LearningLimit(self.stop_reason or "TIME_LIMIT")

    def before_step(self, num_envs):
        self.check()
        if self.frames + num_envs > self.max_frames:
            raise LearningLimit("TRANSITION_LIMIT")
        self.frames += num_envs


def write_json(path: Path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def make_environment(sim, task, frozen, geometry, budget, *, residual_policy=None, informed_resets=False):
    import numpy as np
    import torch
    from isaaclab.utils.math import matrix_from_quat

    from .stage1_physx import classify_contacts, safety_reasons

    class ContactEnvironment:
        def __init__(self):
            n, device = sim.num_envs, sim.device
            self.ids = torch.arange(n, device=device)
            self.policy_ticks = torch.zeros(n, dtype=torch.long, device=device)
            self.physics_ticks = torch.zeros_like(self.policy_ticks)
            self.episode_start_ticks = torch.zeros_like(self.policy_ticks)
            self.horizon = round(frozen["horizon_s"] / sim.physics_dt_s)
            self.substeps = round(sim.control_dt_s / sim.physics_dt_s)
            self.previous_command = torch.zeros(n, 4, device=device)
            self.step_error_m = torch.zeros(n, device=device)
            self.minimum_error = torch.zeros(n, device=device)
            self.saturated_ticks = torch.zeros(n, device=device)
            self.progress = 0.0
            self.center = np.asarray(geometry["target"]["center_m"])
            self.frame = np.asarray(geometry["target"]["target_rotation"])
            self.halfsize = np.asarray(geometry["target"]["halfsize_m"])
            self.counts = Counter()
            self.failure_modes = Counter()
            self.examples = {}
            self.completed_error = 0.0
            self.completed_saturated = 0.0
            self.reset_rejections = 0

        def apply_reset(self, ids, pose):
            if ids.numel() == 0:
                return
            if informed_resets:
                sim.reset(ids, *pose[:6])
                self.policy_ticks[ids] = pose.policy_ticks
                self.physics_ticks[ids] = pose.physics_ticks
                self.previous_command[ids] = pose.previous_command
            else:
                sim.reset(ids, *pose)
                self.policy_ticks[ids] = 0
                self.physics_ticks[ids] = 0
                self.previous_command[ids] = self.previous_command.new_tensor([sim.actuator.hover_force_N, 0, 0, 0])
            self.episode_start_ticks[ids] = self.physics_ticks[ids]

        def reset(self, ids):
            if ids.numel() == 0:
                return
            pending = ids
            for _ in range(4):
                pose = task.reset(pending, progress=self.progress)
                overlaps = sim.initial_overlaps(*pose[:2])
                if not overlaps:
                    self.apply_reset(pending, pose)
                    break
                self.reset_rejections += len(overlaps)
                clear = torch.ones(len(pending), dtype=torch.bool, device=sim.device)
                clear[list(overlaps)] = False
                clear_pose = tuple(value[clear] for value in pose)
                self.apply_reset(pending[clear], type(pose)(*clear_pose) if informed_resets else clear_pose)
                pending = pending[~clear]
            else:
                raise RuntimeError(f"Unable to sample clear Stage 1 training starts: {overlaps}")
            self.minimum_error[ids] = float("inf")
            self.saturated_ticks[ids] = 0

        def observe(self):
            obs = task.observe(sim.state(), self.policy_ticks)
            if residual_policy is not None:
                self.base_raw = residual_policy.contact_base(obs["policy"])
            return obs

        def score_reports(self, tick, active):
            records = {}
            if not tick.contacts:
                return records
            rows = [(i, [c for c in contacts if c["distance_m"] <= 0.0]) for i, contacts in tick.contacts.items()]
            ids = [i for i, contacts in rows if contacts]
            if not ids:
                return records
            selected = torch.tensor(ids, device=sim.device, dtype=torch.long)
            before, after = tick.before, tick.after

            def pack(state):
                rotation = matrix_from_quat(state.quaternion[selected]).reshape(-1, 9)
                return torch.cat((state.position[selected], state.velocity[selected], rotation, state.omega_body[selected]), dim=1)

            values = torch.cat((pack(before), pack(after), active[selected, None],
                                self.physics_ticks[selected, None], self.episode_start_ticks[selected, None]),
                               dim=1).cpu().numpy()

            def state_record(row):
                return {"position_world_m": row[:3].tolist(), "velocity_world_m_s": row[3:6].tolist(),
                        "rotation_body_to_world": row[6:15].reshape(3, 3).tolist(), "omega_body_rad_s": row[15:18].tolist()}

            solids = {i: contacts for i, contacts in rows if contacts}
            for env_id, row in zip(ids, values, strict=True):
                if not row[36]:
                    continue
                before_row, after_row = state_record(row[:18]), state_record(row[18:36])
                contacts = solids[env_id]
                reasons = safety_reasons(before_row, self.center, self.frame, frozen["safety"])
                reasons |= safety_reasons(after_row, self.center, self.frame, frozen["safety"])
                reasons |= classify_contacts(contacts, before_row, self.center, self.frame, self.halfsize,
                                             frozen["contact_acceptance"])
                if row[37] - row[38] == 1:
                    # Training starts have clearance. A first-step overlap is
                    # conservatively invalid even if the reset was at tangency.
                    reasons.add("INITIAL_OVERLAP")
                records[env_id] = {"accepted": not reasons, "failure_modes": sorted(reasons),
                                   "time_s": float(row[37]) * sim.physics_dt_s, "contacts": contacts,
                                   "pre_state": before_row, "post_state": after_row}
            return records

        def step(self, raw_action):
            budget.before_step(len(self.ids))
            if residual_policy is not None:
                raw_action = residual_policy.compose(self.base_raw, raw_action)
            command = sim.actuator.decode_command(raw_action.tanh())
            sim.set_ctbr(command[:, 0], command[:, 1:])
            saturated = sim.actuator.mixer_saturated.clone()
            clipped = torch.zeros_like(saturated)
            self.policy_ticks.add_(1)
            self.saturated_ticks.add_(saturated)
            done = torch.zeros_like(saturated)
            good = torch.zeros_like(done)
            unsafe = torch.zeros_like(done)
            reward = torch.zeros_like(self.step_error_m)
            events = {}
            before = task.metrics(sim.state())
            initial_metrics = before
            self.minimum_error = torch.minimum(self.minimum_error, before.distance_m)
            for _ in range(self.substeps):
                active = ~done
                tick = sim.step_physics()
                self.physics_ticks.add_(active)
                after = task.metrics(tick.after)
                new_events = self.score_reports(tick, active)
                hit = torch.zeros_like(done)
                accepted = torch.zeros_like(done)
                if new_events:
                    selected = list(new_events)
                    hit[selected] = True
                    accepted[selected] = torch.tensor([new_events[i]["accepted"] for i in selected], device=sim.device)
                    events.update(new_events)
                safety_failure = active & (before.unsafe | after.unsafe)
                ended = active & (hit | safety_failure | (self.physics_ticks >= self.horizon))
                success = active & accepted & ~safety_failure
                interval = task.interval_reward(before, after, command, self.previous_command, saturated,
                                                clipped, ended, success)
                reward.add_(torch.where(active, interval, 0.0))
                self.step_error_m.copy_(torch.where(active, after.distance_m, self.step_error_m))
                self.minimum_error = torch.where(active, torch.minimum(self.minimum_error, after.distance_m),
                                                 self.minimum_error)
                good |= success
                done |= ended
                unsafe |= safety_failure
                before = after
            if informed_resets:
                reward = task.policy_reward(initial_metrics, after, reward, done, good)
            self.previous_command.copy_(command)
            reset_ids = done.nonzero(as_tuple=True)[0]
            if reset_ids.numel():
                self.record_completed(reset_ids, good, unsafe, events)
                self.reset(reset_ids)
            return self.observe(), reward, done, torch.zeros_like(done), {}

        def record_completed(self, ids, success, unsafe, events):
            elapsed = self.policy_ticks[ids] - self.episode_start_ticks[ids] // self.substeps
            columns = (ids, task.start_bucket[ids], success[ids], unsafe[ids], self.minimum_error[ids],
                       self.saturated_ticks[ids], elapsed)
            frontier = (task.start_frontier_index[ids] == task.frontier_index if residual_policy is not None or informed_resets
                        else torch.zeros_like(success[ids]))
            segment = task.start_segment[ids] if informed_resets else torch.full_like(ids, -1)
            bands = getattr(task, "start_band", None)
            band_ids = bands[ids] if bands is not None else torch.full_like(ids, -1)
            values = torch.stack((*columns, frontier, segment, band_ids), dim=-1).cpu().tolist()
            for index, bucket, passed, unsafe_flag, error, sat, ticks, at_frontier, segment, band in values:
                index = int(index)
                name = ("near", "intermediate", "full")[int(bucket)]
                self.counts["episodes"] += 1
                self.counts[f"{name}_episodes"] += 1
                self.completed_error += error
                self.completed_saturated += sat / ticks
                if index in events:
                    self.counts["physical_contacts"] += 1
                    kind = "qualified" if passed else "rejected_contact"
                    if not passed:
                        self.counts["unsafe_contacts"] += 1
                    self.failure_modes.update(events[index]["failure_modes"])
                    key = f"{name}_{kind}"
                    self.examples.setdefault(key, {"bucket": name, "minimum_error_m": error, **events[index]})
                elif unsafe_flag:
                    kind = "unsafe_flight"
                    self.failure_modes["SAFETY_BOUNDARY"] += 1
                else:
                    kind = "timeout"
                    self.failure_modes["CONTACT_TIMEOUT"] += 1
                self.counts[kind] += 1
                self.counts[f"{name}_{kind}"] += 1
                if segment >= 0:
                    label = ("informed_near", "informed_rotation", "informed_approach", "stationary")[int(segment)]
                    self.counts[f"{label}_episodes"] += 1
                    self.counts[f"{label}_{kind}"] += 1
                if band >= 0:
                    self.counts[f"band_{int(band)}_episodes"] += 1
                    self.counts[f"band_{int(band)}_{kind}"] += 1
                if at_frontier:
                    self.counts["frontier_episodes"] += 1
                    self.counts[f"frontier_{kind}"] += 1

        def interval_stats(self):
            counts = dict(self.counts)
            n = max(counts.get("episodes", 0), 1)
            counts.update(mean_episode_min_error_m=self.completed_error / n,
                          mean_saturation_fraction=self.completed_saturated / n)
            self.counts.clear()
            self.completed_error = self.completed_saturated = 0.0
            return counts

    return ContactEnvironment()


def train(args, config, assets, frozen, geometry, output, budget, config_sha256, bootstrap):
    import torch

    from .ppo import GAE_LAMBDA, GAMMA, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from .rm27 import RM27_SIM_REVISION
    from .stage1_sim import Stage1Sim
    from .stage1_task import Stage1Actor, Stage1Task

    torch.manual_seed(args.seed)
    sim = Stage1Sim(args.num_envs, assets["isaac_scene"], device="cuda:0")
    curriculum = config["curriculum"] | (bootstrap["curriculum"] if bootstrap else {})
    task = Stage1Task(args.num_envs, sim.device, geometry, frozen, training=True,
                      curriculum=curriculum, capture_weight=args.capture_weight,
                      goal_inset_m=args.goal_inset_m)
    if sim.physics_dt_s != frozen["physics_dt_s"] or sim.control_dt_s != frozen["policy_dt_s"]:
        raise RuntimeError("Batched Stage 1 timing differs from the frozen task")
    env = make_environment(sim, task, frozen, geometry, budget)
    env.reset(env.ids)
    obs = env.observe()
    if not all(value.is_cuda and bool(value.isfinite().all()) for value in obs.values()):
        raise RuntimeError("Stage 1 did not initialize finite CUDA observations")
    actor = Stage1Actor(args.initial_std).to(sim.device)
    source = torch.load(assets["checkpoint"], map_location=sim.device, weights_only=True)
    if (source["format"] != 1 or source["task"] != "track" or source["airframe"] != "rm27"
            or source["actor_obs"] != 45 or source["upstream_revision"] != UPSTREAM_REVISION
            or source["rm27_sim_revision"] != RM27_SIM_REVISION):
        raise ValueError("The authorized starting checkpoint is not the validated RM27 Track policy")
    actor.load_track_state_dict(source["actor"], initial_std=args.initial_std)
    if args.resume is not None:
        previous = torch.load(args.resume, map_location=sim.device, weights_only=True)
        if (args.candidate != 2 or previous["candidate"] != 1 or previous["task"] != "rm27_stage1"
                or previous["config_sha256"] != config_sha256 or previous["frames"] <= 0):
            raise ValueError("Candidate continuation requires a trained checkpoint from this experiment")
        actor.load_state_dict(previous["actor"])
        with torch.no_grad():
            actor.log_std.fill_(math.log(args.initial_std))
    ppo = PPO(obs, args.initial_std, actor=actor, learning_rate=args.learning_rate,
              gamma=math.sqrt(GAMMA), gae_lambda=math.sqrt(GAE_LAMBDA))

    updates = 0
    completed_frames = 0
    totals = Counter()
    learning_status = "ERROR"
    best_full = -1.0
    full_window = Counter()
    near_window = Counter()
    expand_gate = bootstrap["expand_gate"] if bootstrap else None
    unlocked_at = None if expand_gate else 0.0
    last_checkpoint = -1
    wrote_midpoint = False
    latest_path = output / "checkpoint.pt"

    def save(filename="checkpoint.pt"):
        path = output / filename
        temporary = path.with_suffix(".tmp")
        torch.save({"format": 1, "task": "rm27_stage1", "experiment": EXPERIMENT, "candidate": args.candidate,
                    "config_sha256": config_sha256, "source_checkpoint_sha256": config["assets"]["checkpoint"]["sha256"],
                    "upstream_revision": UPSTREAM_REVISION, "rm27_sim_revision": RM27_SIM_REVISION,
                    "actor_obs": obs["policy"].shape[-1], "critic_obs": obs["critic"].shape[-1],
                    "seed": args.seed, "update": updates, "frames": completed_frames, "policy_dt": sim.control_dt_s,
                    "goal_inset_m": args.goal_inset_m, "curriculum": curriculum,
                    "bootstrap_sha256": digest(args.bootstrap) if args.bootstrap else None,
                    "actor": actor.state_dict(), "critic": ppo.critic.state_dict(),
                    "value_normalizer": ppo.normalizer.state_dict(), "actor_optimizer": ppo.actor_opt.state_dict(),
                    "critic_optimizer": ppo.critic_opt.state_dict()}, temporary)
        temporary.replace(path)

    if args.candidate == 1:
        save("initial.pt")
    fields = ("update", "frames", "learning_s", "samples_per_s", "progress", "reward", "position_rmse_m", "kl",
              "episodes", "qualified", "unsafe_contacts", "physical_contacts", "unsafe_flight", "timeout",
              "near_episodes", "near_qualified", "intermediate_episodes", "intermediate_qualified", "full_episodes",
              "full_qualified", "mean_episode_min_error_m", "mean_saturation_fraction", "gpu_used_mib")

    def request_stop(signum, _frame):
        budget.stop_reason = "TIME_LIMIT" if signum == signal.SIGALRM else "EARLY_STOP"

    # Time includes simulation, actor/critic updates and periodic checkpointing.
    # A 1 s guard leaves the clock ceiling ahead of an in-flight GPU call.
    signal.signal(signal.SIGALRM, request_stop)
    signal.signal(signal.SIGUSR1, request_stop)
    torch.cuda.synchronize()
    budget.started = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, max(budget.max_seconds - 1.0, 0.01))
    try:
        with (output / "training.csv").open("x", newline="") as curve:
            writer = csv.DictWriter(curve, fields)
            writer.writeheader()
            while budget.frames + args.num_envs * ROLLOUT_STEPS <= budget.max_frames:
                budget.check()
                start = time.perf_counter()
                env.progress = (0.0 if unlocked_at is None else
                                min(1.0, (budget.elapsed - unlocked_at) / (budget.max_seconds - unlocked_at)))
                obs, rollout = ppo.collect(env, obs)
                budget.check()
                optimization = ppo.update()
                values = torch.cat((rollout, optimization)).tolist()
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError(f"Nonfinite PPO rollout/update after {budget.frames:,} transitions")
                updates += 1
                completed_frames = budget.frames
                stats = env.interval_stats()
                totals.update({key: value for key, value in stats.items() if isinstance(value, int)})
                if unlocked_at is None:
                    near_window.update({key: stats.get(key, 0) for key in ("near_episodes", "near_qualified")})
                    if near_window["near_episodes"] >= expand_gate["minimum_near_episodes"]:
                        if near_window["near_qualified"] / near_window["near_episodes"] >= expand_gate["minimum_qualified_fraction"]:
                            unlocked_at = budget.elapsed
                            print(f"STAGE1_CURRICULUM unlocked after {unlocked_at:.1f}s with "
                                  f"{near_window['near_qualified']}/{near_window['near_episodes']} near qualifications",
                                  flush=True)
                        near_window.clear()
                full_window.update({key: stats.get(key, 0) for key in ("full_episodes", "full_qualified")})
                free, total = torch.cuda.mem_get_info()
                row = dict(update=updates, frames=completed_frames, learning_s=budget.elapsed,
                           samples_per_s=args.num_envs * ROLLOUT_STEPS / (time.perf_counter() - start),
                           progress=env.progress, reward=values[0], position_rmse_m=values[1], kl=values[-1],
                           gpu_used_mib=(total - free) / (1 << 20))
                row.update({key: stats.get(key, 0) for key in fields if key not in row})
                writer.writerow(row)
                curve.flush()
                if full_window["full_episodes"] >= 64:
                    ratio = full_window["full_qualified"] / full_window["full_episodes"]
                    if ratio > 0.0 and ratio > best_full:
                        best_full = ratio
                        save("best-full.pt")
                    full_window.clear()
                if not wrote_midpoint and env.progress >= 0.50:
                    save("midpoint.pt")
                    wrote_midpoint = True
                if updates == 1 or updates % 10 == 0:
                    save()
                    last_checkpoint = updates
                if updates == 1 or updates % 10 == 0:
                    print(f"STAGE1_PPO c{args.candidate} u{updates} frames={budget.frames:,} "
                          f"{row['samples_per_s']:,.0f}/s t={budget.elapsed:.0f}s "
                          f"qualified={totals['qualified']}/{totals['episodes']} full={totals['full_qualified']}/"
                          f"{totals['full_episodes']} min_error={row['mean_episode_min_error_m']:.3f}m "
                          f"GPU={row['gpu_used_mib']:.0f}MiB", flush=True)
            learning_status = "TRANSITION_LIMIT"
    except LearningLimit as limit:
        learning_status = str(limit)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        elapsed = budget.elapsed
        if learning_status != "ERROR" and updates and last_checkpoint != updates:
            # An interrupt during optimization can leave partially updated weights.
            # Do not save those as a completed update.
            if budget.frames != completed_frames:
                print(f"Retaining last completed periodic checkpoint at update {last_checkpoint}", flush=True)
            else:
                save()
        write_json(output / "learning.json", {"status": learning_status, "candidate": args.candidate,
                   "learning_s": elapsed, "frames": budget.frames, "completed_frames": completed_frames,
                   "updates": updates, "num_envs": args.num_envs, "settings": vars(args) | {
                       "resume": str(args.resume) if args.resume else None,
                       "bootstrap": str(args.bootstrap) if args.bootstrap else None},
                   "curriculum_unlocked_s": unlocked_at, "reset_overlap_rejections": env.reset_rejections,
                   "config_sha256": config_sha256, "counts": dict(totals),
                   "failure_modes": dict(env.failure_modes), "representative_contact_events": env.examples,
                   "checkpoint": str(latest_path.name) if latest_path.exists() else None})
    print(f"STAGE1_PPO_COMPLETE c{args.candidate} {learning_status} frames={budget.frames:,} learning_s={elapsed:.2f}",
          flush=True)


def main():
    config, assets, frozen, geometry = load_experiment()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", type=int, choices=(1, 2), required=True)
    parser.add_argument("--num-envs", type=int, default=128)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--bootstrap", type=Path, help="Candidate 2 training-only curriculum overlay")
    parser.set_defaults(goal_inset_m=config["goal_inset_m"])
    for key, default in config["candidate_1"].items():
        parser.add_argument("--" + key.replace("_", "-"), type=type(default), default=default)
    args = parser.parse_args()
    if (args.num_envs <= 0 or args.learning_s <= 1 or args.initial_std <= 0 or args.learning_rate <= 0
            or args.capture_weight < 0 or not 0 < args.goal_inset_m < 0.04
            or (args.candidate == 1 and (args.resume is not None or args.bootstrap is not None))):
        parser.error("Invalid Stage 1 candidate parameters")
    bootstrap = read_json(args.bootstrap) if args.bootstrap else None
    if bootstrap is not None and (bootstrap["experiment"] != EXPERIMENT or bootstrap["candidate"] != args.candidate):
        parser.error("The bootstrap config must match this experiment and candidate")
    RUN.mkdir(parents=True, exist_ok=True)
    with (RUN / "budget.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        prior = []
        for i in range(1, args.candidate):
            prior.append(read_json(RUN / f"candidate-{i}" / "learning.json"))
        frames = config["budget"]["transitions"] - sum(row["frames"] for row in prior)
        seconds = config["budget"]["learning_s"] - sum(row["learning_s"] for row in prior)
        budget = LearningBudget(frames, min(args.learning_s, seconds))
        if budget.max_seconds <= 1 or budget.max_frames < 64 * args.num_envs:
            raise RuntimeError("No authorized Stage 1 rollout remains")
        output = RUN / f"candidate-{args.candidate}"
        if output.exists():
            raise FileExistsError(output)
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(headless=True, device="cuda:0")
        output.mkdir()
        try:
            train(args, config, assets, frozen, geometry, output, budget, digest(CONFIG), bootstrap)
        except Exception:
            if budget.started is None and not any(output.iterdir()):
                output.rmdir()
            traceback.print_exc()
            sys.stderr.flush()
            os._exit(2)
        sys.stdout.flush()
        sys.stderr.flush()
        # This one-shot process uses the same explicit exit as the contact preflight.
        os._exit(0)


if __name__ == "__main__":
    main()
