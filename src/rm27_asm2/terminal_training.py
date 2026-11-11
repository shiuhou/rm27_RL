"""Run the single authorized ASM2 Direct-CTBR terminal PPO candidate."""

from __future__ import annotations

from collections import Counter
import io
import json
import os
import signal
import sys
import time
import traceback

import numpy as np

from .asset import ROOT, digest, read_json, write_result
from .contact_task import load_contact_spec
from .flight import load_profile
from .flight_suite import flight_settings
from .terminal_isaac import TerminalSim
from .terminal_preflight import CONFIG, RUN, load_development, load_experiment

# Supplemental quality reward after physical contact. Inside the existing
# native envelope and close to the measured Phase 3 witness speeds.
GENTLE_SPEED_M_S = .060


def verify_preflight(config):
    native = RUN / "native-preflight.json"
    settings = config["preflight"]
    hashes, timings = {}, {}
    for layout in settings["layouts"]:
        path = RUN / f"isaac-layout{layout}.json"
        report = read_json(path)
        if (report["config_sha256"] != digest(CONFIG) or report["native_sha256"] != digest(native)
                or report["layout"] != layout or report["world_count"] != settings["worlds"]
                or report["passed"] is not True or len(report["batches"]) != 2):
            raise ValueError("Both mixed-layout contact preflights must pass before learning")
        for batch in report["batches"]:
            if len(batch["rows"]) != settings["worlds"]:
                raise ValueError("Incomplete mixed contact preflight")
            for row in batch["rows"]:
                if row["reason"] != row["expected"]:
                    raise ValueError("Unreliable first-contact feedback")
                if row["signed_tick"] is not None:
                    timings.setdefault(row["name"], []).append(row["signed_tick"])
        hashes[path.name] = digest(path)
    if set(timings) != {*settings["positive_source_frames"], "field_inverted_body"} or any(
            max(ticks)-min(ticks) > settings["max_tick_spread"] for ticks in timings.values()):
        raise ValueError("First-contact timing differs across world layouts")
    return {"native-preflight.json": digest(native), **hashes}


class TerminalEnvironment:
    def __init__(self, sim, data, goal, horizon, budget):
        import torch

        self.torch, self.sim, self.budget = torch, sim, budget
        self.data = tuple(sim.tensor(data[name]) for name in ("states", "motor_thrust_N", "motor_target_N"))
        self.goal = sim.tensor(goal)
        self.normal = sim.tensor(sim.spec.target_normal)
        self.frame = sim.tensor(sim.spec.target_rotation)
        self.center = sim.tensor(sim.spec.target_center)
        selection = sim.spec.field_geometry["selection"]
        self.lower = sim.tensor(selection["safety_lower_target_frame_m"])
        self.upper = sim.tensor(selection["safety_upper_target_frame_m"])
        self.limits = read_json(ROOT / "configs/rm27_asm2_handover.json")["flight_envelope"]
        self.tilt_cos = np.cos(np.deg2rad(self.limits["max_tilt_deg"]))
        self.horizon = round(horizon/sim.profile.policy_dt_s)
        self.ids = torch.arange(sim.worlds, device=sim.device)
        self.episode_ticks = torch.zeros(sim.worlds, dtype=torch.int32, device=sim.device)
        self.reset_counts = torch.zeros(len(self.data[0]), dtype=torch.int64, device=sim.device)
        self.saturations = torch.zeros((), dtype=torch.int64, device=sim.device)
        self.outcomes = Counter()
        self.reset(self.ids, self.ids)

    def reset(self, ids, rows=None):
        if rows is None:
            rows = self.torch.randint(len(self.data[0]), (len(ids),), device=self.sim.device)
        self.reset_counts.index_add_(0, rows, self.torch.ones_like(rows))
        self.sim.restore(ids, *(item[rows] for item in self.data))
        self.episode_ticks[ids] = 0
        self.state, self.R = self.sim.observe()

    def observation(self):
        rel = (self.goal-self.state[:, :3]).repeat(1, 10)
        axes = self.R.transpose(1, 2).reshape(-1, 9)
        obs = self.torch.cat((rel, self.state[:, 7:10], axes, self.state[:, 10:13]), dim=1)
        return {"policy": obs, "critic": obs}

    def step(self, action):
        torch, sim = self.torch, self.sim
        self.budget.before_step(sim.worlds)
        action = action.tanh()
        ctbr = torch.cat((sim.profile.hover_thrust_N*(1.+.5*action[:, 3:]), 2.*action[:, :3]), dim=1)
        old_distance = (self.state[:, :3]-self.goal).norm(dim=1)
        old_alignment = self.R[:, :, 2] @ self.normal
        self.saturations += sim.set_commands(ctbr, self.state[:, 10:13]).sum()
        sim.events.clear()
        for _ in range(sim.substeps):
            self.state, self.R = sim.physics_step(self.state, self.R)
        self.episode_ticks += 1
        self.step_error_m = (self.state[:, :3]-self.goal).norm(dim=1)
        alignment = self.R[:, :, 2] @ self.normal
        reward = 8.*(old_distance-self.step_error_m) + .15*(alignment-old_alignment)
        if not bool(torch.isfinite(self.state).all()):
            raise RuntimeError("Nonfinite terminal PhysX state")
        local = (self.state[:, :3]-self.center) @ self.frame
        unsafe = (((local < self.lower) | (local > self.upper)).any(dim=1)
                  | (self.state[:, 7:10].norm(dim=1) > self.limits["max_com_speed_m_s"])
                  | (self.state[:, 10:13].norm(dim=1) > self.limits["max_body_rate_rad_s"])
                  | (self.R[:, 2, 2] < self.tilt_cos))
        unsafe_ids = torch.nonzero(unsafe).flatten().tolist()
        for world in unsafe_ids:
            sim.events[world] = "FLIGHT_ENVELOPE_EXIT"
        terminated = torch.zeros_like(self.episode_ticks, dtype=torch.bool)
        if sim.events:
            world_ids, values = [], []
            for world, reason in sim.events.items():
                self.outcomes[reason] += 1
                world_ids.append(world)
                if reason == "DESIGNATED_LOADED":
                    worst_speed = sim.event_speed[world]
                    values.append(1.0 + .2*max(0., 1.-abs(worst_speed-GENTLE_SPEED_M_S)/.1))
                elif reason.startswith("FORBIDDEN_") or reason == "FLIGHT_ENVELOPE_EXIT":
                    values.append(-1.25)
                else:
                    values.append(-.8)
            terminated[world_ids] = True
            reward[world_ids] += sim.tensor(values)
        truncated = (self.episode_ticks >= self.horizon) & ~terminated
        reward -= .6*truncated
        ids = torch.nonzero(terminated | truncated).flatten()
        self.outcomes["TIMEOUT"] += len(ids)-len(sim.events)
        if len(ids):
            self.reset(ids)
        return self.observation(), reward, terminated, truncated, {}


def train():
    import torch

    from simpleflight_lab.ppo import Actor, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from simpleflight_lab.rm27 import RM27_SIM_REVISION
    from simpleflight_lab.stage1_training import LearningBudget, LearningLimit

    config, pins = load_experiment()
    evidence = verify_preflight(config)
    settings = config["learning"]
    if settings["worlds"] != config["preflight"]["worlds"]:
        raise ValueError("Learning batch size differs from preflight")
    directory = RUN / "learning"
    if directory.exists():
        raise ValueError("The single pilot has already been claimed")
    profile, spec = load_profile(), load_contact_spec()
    data = load_development(pins["resets"])
    sim = TerminalSim(settings["worlds"], 0, profile, spec)
    auth = config["authorization"]
    budget = LearningBudget(auth["max_transitions"], auth["max_learning_s"])
    goal = spec.target_center + (-spec.patch_face_z_m-config["goal_inset_m"])*spec.target_normal
    torch.manual_seed(settings["seed"])
    env = TerminalEnvironment(sim, data, goal, config["horizon_s"], budget)
    obs = env.observation()
    track = flight_settings()["ver2_track_checkpoint"]
    if digest(ROOT / track["path"]) != track["sha256"]:
        raise ValueError("Frozen Track checkpoint differs from its pin")
    source = torch.load(io.BytesIO((ROOT / track["path"]).read_bytes()), map_location=sim.device, weights_only=True)
    expected = dict(format=1, task="track", airframe="rm27", actor_obs=45, upstream_revision=UPSTREAM_REVISION,
                    rm27_sim_revision=RM27_SIM_REVISION, policy_dt=profile.policy_dt_s)
    if any(source.get(k) != v for k, v in expected.items()):
        raise ValueError("Incompatible frozen Track checkpoint")
    actor = Actor(45).to(sim.device)
    actor.load_state_dict(source["actor"])
    with torch.no_grad():
        actor.log_std.fill_(np.log(settings["initial_std"]))
    ppo = PPO(obs, settings["initial_std"], actor=actor, learning_rate=settings["lr"])
    directory.mkdir(exist_ok=False)
    write_result(directory / "authorization.json", dict(config_sha256=digest(CONFIG), preflight_hashes=evidence,
                                                        track_sha256=track["sha256"], budgets=auth,
                                                        trainer_sha256=digest(__file__)))
    saved = []
    complete_frames = updates = 0

    def save(tag):
        path = directory / f"{tag}.pt"
        weights = {name: value.detach().cpu() for name, value in ppo.actor.state_dict().items()}
        temp = path.with_suffix(".tmp")
        torch.save(dict(format=1, task="asm2_terminal", airframe="asm2", actor_obs=45, actor=weights,
                        critic=ppo.critic.state_dict(), value_normalizer=ppo.normalizer.state_dict(),
                        actor_optimizer=ppo.actor_opt.state_dict(), critic_optimizer=ppo.critic_opt.state_dict(),
                        frames=complete_frames, update=updates, learning_s=budget.elapsed,
                        config_sha256=digest(CONFIG), track_sha256=track["sha256"]), temp)
        temp.replace(path)
        # Export only from the reloaded checkpoint. Native verifies the hash.
        reloaded = torch.load(path, map_location="cpu", weights_only=True)
        weight_path = path.with_suffix(".npz")
        with weight_path.with_suffix(".tmp").open("wb") as stream:
            np.savez_compressed(stream, **{name: value.numpy() for name, value in reloaded["actor"].items()},
                                checkpoint_sha256=digest(path))
        weight_path.with_suffix(".tmp").replace(weight_path)
        saved.append(dict(path=path.name, frames=complete_frames, sha256=digest(path)))
        print("ASM2_PILOT_SAVED", path.name, complete_frames, flush=True)

    def alarm(signum, _frame):
        raise LearningLimit("TIME_LIMIT" if signum == signal.SIGALRM else "INTERRUPTED")

    signal.signal(signal.SIGALRM, alarm)
    signal.signal(signal.SIGTERM, alarm)
    reason = "ERROR"
    rollout_size = ROLLOUT_STEPS*sim.worlds
    snapshot_times = iter((60., 240., 540., 900., 1260., 1560.))
    next_snapshot = next(snapshot_times, np.inf)
    updating = False
    budget.started = time.perf_counter()
    signal.setitimer(signal.ITIMER_REAL, budget.max_seconds-.5)
    try:
        with (directory / "learning.jsonl").open("x", buffering=1) as log:
            while budget.frames + rollout_size <= budget.max_frames:
                budget.check()
                obs, stats = ppo.collect(env, obs)
                budget.check()
                updating = True
                losses = ppo.update(before_minibatch=budget.check)
                updating = False
                if not bool(torch.isfinite(losses).all() and torch.isfinite(stats).all()):
                    raise RuntimeError("Nonfinite PPO statistics")
                updates += 1
                complete_frames = budget.frames
                if updates % 10 == 0 or updates == 1:
                    row = dict(updates=updates, transitions=complete_frames, learning_s=budget.elapsed,
                               outcomes=dict(env.outcomes), saturations=int(env.saturations),
                               mean_reward=float(stats[0]), error_m=float(stats[1]), loss=losses.tolist())
                    log.write(json.dumps(row, allow_nan=False)+"\n")
                    print("ASM2_PILOT", updates, complete_frames, round(budget.elapsed, 1), dict(env.outcomes), flush=True)
                if budget.elapsed >= next_snapshot:
                    save(f"update-{updates:05d}")
                    next_snapshot = next(snapshot_times, np.inf)
            reason = "TRANSITION_LIMIT"
    except LearningLimit as stop:
        reason = str(stop)
    finally:
        learning_s = budget.elapsed
        signal.setitimer(signal.ITIMER_REAL, 0)
        if updates and not updating and reason != "ERROR" and (not saved or saved[-1]["frames"] != complete_frames):
            save("last")
        write_result(directory / "result.json", dict(
            stop_reason=reason, transitions=budget.frames, completed_transitions=complete_frames, updates=updates,
            learning_s=learning_s, samples_per_s=budget.frames/max(1e-9, learning_s), outcomes=dict(env.outcomes),
            saturations=int(env.saturations), reset_counts=env.reset_counts.tolist(), checkpoints=saved,
            partial_update_discarded=updating, config_sha256=digest(CONFIG), preflight_hashes=evidence))
    return 0


def main():
    from isaaclab.app import AppLauncher

    launcher = AppLauncher(headless=True)
    try:
        code = train()
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
