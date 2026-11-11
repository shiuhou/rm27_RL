"""Resume the frozen Phase 4R.2 learner for one measured braking trial."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback

import numpy as np

from rm27_asm2.asset import ROOT, digest, read_json, write_result
from rm27_asm2.contact_task import load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.terminal_isaac import TerminalSim
from simpleflight_lab.stage1_training import LearningBudget, LearningLimit, write_json

RUN = Path(__file__).resolve().parent
PARENT = RUN.parent / "phase4r2"
sys.path.insert(0, str(PARENT))
import braking_pilot as previous
from matched_contact import corrected_asset

PARENT_CHECKPOINT = PARENT / "learning/development.pt"
PARENT_SHA = "d95af494aaeca59494cacb86438d9e5d93fea8956f6495e28eca0be92f5bceda"
MAX_TRANSITIONS = 300_000
MAX_LEARNING_S = 90*60
RESUME_SEED = 42705
MIN_TRANSITIONS = 100_000
PATIENCE = 3
FIRST_EVALUATIONS = (24_576, 49_152, 73_728, 100_352)
EVALUATION_INTERVAL = 12_288
EVALUATION_WAIT_S = 180


def inputs():
    receipt = previous.verify_receipt()
    cfg, bank, starts, n_braking = previous.inputs(receipt)
    fresh_path = RUN / "fresh_manifest.json"
    fresh = read_json(fresh_path)
    fresh_archive = RUN / fresh["archive"]["path"]
    if (fresh["schema"] != "asm2_phase4_meaningful_fresh_braking_v1"
            or not fresh.get("frozen_utc") or digest(fresh_archive) != fresh["archive"]["sha256"]):
        raise ValueError("Fresh reset suite must be frozen before learning")
    if digest(PARENT_CHECKPOINT) != PARENT_SHA:
        raise ValueError("Prior Phase 4R.2 checkpoint changed")
    parent_eval = PARENT / "learning/development-result.json"
    parent = read_json(parent_eval)["results"]["candidate"]
    if (parent["actor"]["checkpoint_sha256"] != PARENT_SHA
            or {(r["case"], r["source_frame"]) for r in parent["episodes"]} != starts):
        raise ValueError("Initial checkpoint requires its frozen development result")
    baseline = parent["cohorts"]["braking"]
    if (baseline["qualified"], baseline["episodes"], baseline["safety_failures"]) != (11, 36, 0):
        raise ValueError("Unexpected initial development score")
    return receipt, cfg, bank, starts, n_braking, fresh_path, parent_eval


def save_checkpoint(directory, ppo, frames, updates, elapsed, track_sha, profile):
    import torch

    path = directory / f"checkpoint-{frames:06d}.pt"
    temp = path.with_suffix(".tmp")
    actor = {name: value.detach().cpu() for name, value in ppo.actor.state_dict().items()}
    torch.save(dict(format=1, task="asm2_braking_continuation", airframe="asm2", actor_obs=45,
                    actor=actor, critic=ppo.critic.state_dict(),
                    value_normalizer=ppo.normalizer.state_dict(), actor_optimizer=ppo.actor_opt.state_dict(),
                    critic_optimizer=ppo.critic_opt.state_dict(), parent_sha256=PARENT_SHA,
                    frames=12_288+frames, additional_transitions=frames, update=12+updates,
                    learning_s=elapsed, policy_dt_s=profile.policy_dt_s,
                    action_rate_rad_s=[2., 2., 2.], action_collective_hover=[.5, 1.5],
                    track_sha256=track_sha, config_sha256=digest(previous.CONFIG),
                    backend_receipt_sha256=digest(previous.RECEIPT)), temp)
    temp.replace(path)
    reloaded = torch.load(path, map_location="cpu", weights_only=True)
    weights = path.with_suffix(".npz")
    with weights.with_suffix(".tmp").open("wb") as stream:
        np.savez_compressed(stream, **{name: value.numpy() for name, value in reloaded["actor"].items()},
                            checkpoint_sha256=digest(path))
    weights.with_suffix(".tmp").replace(weights)
    return path, weights


def evaluate_development(directory, checkpoint, weights, frames, starts, receipt, budget):
    result = directory / f"eval-{frames:06d}.json"
    write_json(directory / f"request-{frames:06d}.json", {
        "additional_transitions": frames,
        "suite": "recovery-development",
        "experiment": str(previous.CONFIG.relative_to(ROOT)),
        "checkpoint": str(checkpoint.relative_to(ROOT)),
        "actor": str(weights.relative_to(ROOT)),
        "output": str(result.relative_to(ROOT)),
        "checkpoint_sha256": digest(checkpoint),
        "weights_sha256": digest(weights),
    })
    deadline = time.monotonic()+EVALUATION_WAIT_S
    while not result.exists():
        if budget.stop_reason not in (None, "TIME_LIMIT"):
            raise LearningLimit(budget.stop_reason)
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Development evaluation unavailable at {frames:,} transitions")
        time.sleep(.25)
    report = read_json(result)
    candidate = report["results"]["candidate"]
    episodes = candidate["episodes"]
    if (report["schema"] != "rm27_asm2_terminal_native_eval_v1"
            or report["suite"] != "recovery-development" or report["config_sha256"] != digest(previous.CONFIG)
            or report["reset_manifest_sha256"] != receipt["reset_manifest_sha256"]
            or report["model"]["contract_sha256"] != receipt["contact_contract_sha256"]
            or report["evaluator_sha256"] != digest(ROOT / "src/rm27_asm2/terminal_native_eval.py")
            or candidate["actor"]["checkpoint_sha256"] != digest(checkpoint)
            or candidate["actor"]["weights_sha256"] != digest(weights)
            or len(episodes) != 36
            or {(r["case"], r["source_frame"]) for r in episodes if r["phase"] == "braking"} != starts):
        raise ValueError("Development report differs from the fixed suite or checkpoint")
    qualified = sum(r["qualified"] for r in episodes)
    exits = sum(r["safety_failure"] is not None for r in episodes)
    summary = dict(transitions=frames, qualified=qualified, exits=exits, promotion=qualified >= 15 and exits == 0,
                   checkpoint=checkpoint.name, checkpoint_sha256=digest(checkpoint), actor=weights.name,
                   result=result.name, result_sha256=digest(result))
    print(f"ASM2_BRAKING_DEV {frames} {qualified}/36 exits={exits} promotion={summary['promotion']}", flush=True)
    return summary


def run(prepared):
    import torch
    from simpleflight_lab.ppo import Actor, PPO, ROLLOUT_STEPS
    from simpleflight_lab.ppo import GAMMA, GAE_LAMBDA
    from rm27_asm2.flight_suite import flight_settings

    receipt, cfg, bank, starts, n_braking, fresh_path, parent_eval = prepared
    settings = cfg["learning"]
    profile, spec = load_profile(), load_contact_spec()
    corrected = corrected_asset(spec)
    track = flight_settings()["ver2_track_checkpoint"]
    if digest(corrected[0]) != receipt["asset_sha256"] or settings["worlds"] != receipt["worlds"]:
        raise ValueError("Instantiated collision asset or world count differs from the matched gate")
    source = torch.load(PARENT_CHECKPOINT, map_location="cpu", weights_only=True)
    expected = dict(format=1, task="asm2_braking_phase4r2", airframe="asm2", actor_obs=45, frames=12_288, update=12,
                    action_rate_rad_s=[2., 2., 2.], action_collective_hover=[.5, 1.5],
                    policy_dt_s=profile.policy_dt_s, config_sha256=digest(previous.CONFIG),
                    backend_receipt_sha256=digest(previous.RECEIPT), track_sha256=track["sha256"])
    if any(source.get(k) != value for k, value in expected.items()):
        raise ValueError("Checkpoint does not match the frozen braking pilot")
    directory = RUN / "learning"
    directory.mkdir(exist_ok=False)
    write_result(directory / "authorization.json", dict(
        parent_checkpoint=str(PARENT_CHECKPOINT.relative_to(ROOT)), parent_sha256=PARENT_SHA,
        parent_evaluation_sha256=digest(parent_eval), fresh_manifest_sha256=digest(fresh_path),
        trainer_sha256=digest(__file__), parent_trainer_sha256=digest(previous.__file__),
        ppo_sha256=digest(ROOT / "simpleflight_lab/ppo.py"), config_sha256=digest(previous.CONFIG),
        backend_receipt_sha256=digest(previous.RECEIPT), learning_caps=dict(transitions=MAX_TRANSITIONS, seconds=MAX_LEARNING_S),
        evaluation_schedule=dict(first=list(FIRST_EVALUATIONS), interval=EVALUATION_INTERVAL, patience=PATIENCE,
                                 min_transitions=MIN_TRANSITIONS, promotion=dict(qualified=15, exits=0)),
        resume_seed=RESUME_SEED, initial_score=11, optimizer_state_restored=True,
        fresh_simulation_reset=True, gamma=GAMMA, gae_lambda=GAE_LAMBDA))
    budget = LearningBudget(MAX_TRANSITIONS, MAX_LEARNING_S)
    sim = env = ppo = None
    evaluations = []
    reason, frames, updates, updating = "ERROR", 0, 0, False
    elapsed = evaluation_pause_s = 0.
    patience, best = 0, 11

    def stop(signum, _frame):
        budget.stop_reason = "TIME_LIMIT" if signum == signal.SIGALRM else "INTERRUPTED"

    def checkpoint_and_evaluate():
        nonlocal evaluation_pause_s
        path, weights = save_checkpoint(directory, ppo, frames, updates, budget.elapsed, track["sha256"], profile)
        print(f"ASM2_BRAKING_CHECKPOINT {frames} {budget.elapsed:.1f}s", flush=True)
        signal.setitimer(signal.ITIMER_REAL, 0.)
        started = time.perf_counter()
        try:
            result = evaluate_development(directory, path, weights, frames, starts, receipt, budget)
        finally:
            pause = time.perf_counter()-started
            evaluation_pause_s += pause
            budget.started += pause
        remaining = MAX_LEARNING_S-budget.elapsed
        if budget.stop_reason is None and remaining > .5:
            signal.setitimer(signal.ITIMER_REAL, remaining-.5)
        evaluations.append(result)
        return result

    try:
        sim = TerminalSim(settings["worlds"], 0, profile, spec, diagnostic=True,
                          physics_dt=receipt["physics_dt_s"], contact_offset_m=receipt["contact_offset_m"],
                          asset_override=corrected)
        if sim.asset_sha256 != receipt["asset_sha256"]:
            raise ValueError("Live overlay does not match the validated asset")
        torch.manual_seed(RESUME_SEED)
        goal = spec.target_center + (-spec.patch_face_z_m-cfg["goal_inset_m"])*spec.target_normal
        env = previous.BrakingEnvironment(sim, bank, goal, cfg["braking_horizon_s"], budget, settings, n_braking)
        obs = env.observation()
        actor = Actor(45).to(sim.device)
        actor.load_state_dict(source["actor"])
        ppo = PPO(obs, settings["initial_std"], actor=actor, learning_rate=settings["lr"])
        ppo.critic.load_state_dict(source["critic"])
        ppo.normalizer.load_state_dict(source["value_normalizer"])
        ppo.actor_opt.load_state_dict(source["actor_optimizer"])
        ppo.critic_opt.load_state_dict(source["critic_optimizer"])
        probe = obs["policy"].clone()
        with torch.no_grad():
            initial_actions = actor(probe).tanh()
            initial_parameters = torch.nn.utils.parameters_to_vector(actor.mean.parameters()).clone()
        del source
        signal.signal(signal.SIGALRM, stop)
        signal.signal(signal.SIGTERM, stop)
        rollout = ROLLOUT_STEPS*sim.worlds
        if any(step % rollout for step in (*FIRST_EVALUATIONS, EVALUATION_INTERVAL)):
            raise ValueError("Development evaluations must use complete PPO updates")
        final_frames = MAX_TRANSITIONS//rollout*rollout
        next_eval = FIRST_EVALUATIONS[0]
        budget.started = time.perf_counter()
        signal.setitimer(signal.ITIMER_REAL, MAX_LEARNING_S-.5)
        with (directory / "learning.jsonl").open("x", buffering=1) as log:
            while budget.frames+rollout <= MAX_TRANSITIONS:
                budget.check()
                obs, stats = ppo.collect(env, obs)
                budget.check()
                updating = True
                losses = ppo.update(before_minibatch=budget.check)
                updating = False
                if not bool(torch.isfinite(losses).all() and torch.isfinite(stats).all()):
                    raise RuntimeError("Nonfinite PPO update")
                frames, updates = budget.frames, updates+1
                if updates == 1 or updates % 8 == 0 or frames in (next_eval, final_frames):
                    with torch.no_grad():
                        actions = actor(probe).tanh()
                        exploration = actor.log_std.exp()
                        parameters = torch.nn.utils.parameters_to_vector(actor.mean.parameters())
                        delta = (actions-initial_actions).square().mean(dim=0).sqrt()
                        param_delta = (parameters-initial_parameters).norm()/initial_parameters.norm()
                        if not bool(torch.isfinite(actions).all() and torch.isfinite(exploration).all()):
                            raise RuntimeError("Nonfinite actor actions or exploration")
                    row = dict(updates=updates, transitions=frames, learning_s=budget.elapsed,
                               outcomes=dict(env.outcomes), saturations=int(env.saturations),
                               mean_reward=float(stats[0]), error_m=float(stats[1]), loss=losses.tolist(),
                               action_std=exploration.tolist(), action_rms_delta=delta.tolist(),
                               mean_parameter_relative_l2=float(param_delta))
                    log.write(json.dumps(row, allow_nan=False)+"\n")
                    print(f"ASM2_BRAKING_LEARNING {frames} {budget.elapsed:.1f}s {dict(env.outcomes)}", flush=True)
                if frames in (next_eval, final_frames):
                    result = checkpoint_and_evaluate()
                    improves = result["exits"] == 0 and result["qualified"] > best
                    if improves:
                        best = result["qualified"]
                    if frames >= MIN_TRANSITIONS:
                        patience = 0 if improves else patience+1
                    if frames == final_frames:
                        break
                    if patience >= PATIENCE:
                        raise LearningLimit("DEVELOPMENT_PATIENCE")
                    next_eval = next((step for step in FIRST_EVALUATIONS if step > frames),
                                     frames+EVALUATION_INTERVAL)
            reason = "TRANSITION_LIMIT"
    except LearningLimit as limit:
        reason = str(limit)
        if (reason == "TIME_LIMIT" and frames and not updating
                and (not evaluations or evaluations[-1]["transitions"] != frames)):
            checkpoint_and_evaluate()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.)
        elapsed = budget.elapsed
        # Choose among continuation checkpoints by safe development performance.
        selected = min(evaluations, default=None, key=lambda r: (
            r["exits"] != 0, r["exits"], -r["qualified"], r["transitions"]))
        write_result(directory / "result.json", dict(
            stop_reason=reason, transitions=budget.frames, completed_transitions=frames, updates=updates,
            learning_s=elapsed, evaluation_pause_s=evaluation_pause_s, evaluations=evaluations,
            selected=selected, outcomes=dict(env.outcomes) if env else {},
            saturations=int(env.saturations) if env else 0, reset_counts=env.reset_counts.tolist() if env else [],
            partial_update_discarded=updating, gpu=torch.cuda.get_device_name() if sim else None,
            torch_cuda_peak_allocated_bytes=torch.cuda.max_memory_allocated() if sim else None,
            torch_cuda_peak_reserved_bytes=torch.cuda.max_memory_reserved() if sim else None,
            backend_receipt_sha256=digest(previous.RECEIPT), config_sha256=digest(previous.CONFIG),
            parent_sha256=PARENT_SHA, fresh_manifest_sha256=digest(fresh_path)))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Validate all frozen inputs without launching Isaac")
    args = parser.parse_args()
    try:
        prepared = inputs()
        if args.check:
            print("Frozen backend, resume checkpoint, development split and fresh suite verified.")
            return
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(headless=True)
        code = run(prepared)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
