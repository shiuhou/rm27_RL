"""Run the one gated Phase 4R.2 braking pilot. No held-out inputs are loaded."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback

import numpy as np

from rm27_asm2.asset import ROOT, digest, obj_mesh, read_json, write_result
from rm27_asm2.contact_task import PATCH_GROUPS, load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.flight_suite import flight_settings
from rm27_asm2.terminal_isaac import TerminalSim
from rm27_asm2.terminal_training import GENTLE_SPEED_M_S, TerminalEnvironment
from simpleflight_lab.stage1_training import LearningBudget, LearningLimit, write_json

RUN = Path(__file__).resolve().parent
CONFIG = ROOT / "configs/rm27_asm2_terminal_recovery.json"
RECEIPT = RUN / "backend-receipt.json"
NATIVE_GATE = RUN.parent / "phase4r" / "native-gate-braking.json"
MAX_TRANSITIONS, MAX_LEARNING_S = 65_536, 900
FIRST_TRANSITIONS, FIRST_LEARNING_S = 12_288, 360
MIN_QUALIFIED = 15


def verify_receipt():
    receipt = read_json(RECEIPT)
    settings = dict(status="PASS", physics_dt_s=.0001, contact_offset_m=1e-7, worlds=16)
    if any(receipt.get(name) != value for name, value in settings.items()):
        raise ValueError("Phase 4R.2 requires an exact passing two-layout backend receipt")
    if receipt.get("asset_kind", "corrected") != "corrected":
        raise ValueError("Only the matched corrected asset is authorized")
    pins = receipt["run_pins"]
    required = {str((RUN / name).relative_to(ROOT)) for name in (
        "isaac-corrected-layout0.json", "isaac-corrected-layout1.json",
        "action-authority.json", "proxy-with-reference-body.usda", "matched_contact.py")}
    if not required.issubset(pins):
        raise ValueError("Receipt omits required contact or action-authority evidence")
    for name, expected in pins.items():
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or digest(path) != expected:
            raise ValueError(f"Backend receipt input changed: {name}")
    native = read_json(NATIVE_GATE)
    native_cases = {row["name"]: row for row in native["results"]}
    if len(native_cases) != 10 or native["contact_config_sha256"] != receipt["contact_contract_sha256"]:
        raise ValueError("Frozen native contact reference changed")
    for layout in (0, 1):
        report = read_json(RUN / f"isaac-corrected-layout{layout}.json")
        if (report["passed"] is not True or report["layout"] != layout
                or report["source_gate_sha256"] != digest(NATIVE_GATE)
                or report["asset_sha256"] != receipt["asset_sha256"]
                or report["contact_config_sha256"] != receipt["contact_contract_sha256"]
                or report["physics_dt_s"] != receipt["physics_dt_s"]
                or report["contact_offset_m"] != receipt["contact_offset_m"]
                or report["control_dt_s"] != .005 or len(report["batches"]) != 2):
            raise ValueError(f"Corrected layout {layout} does not match its gate")
        timings = {name: [] for name in native_cases}
        for batch in report["batches"]:
            rows = batch["rows"]
            if (len(rows) != receipt["worlds"] or {row["name"] for row in rows} != set(native_cases)
                    or not batch["finite"] or not batch["exact_motor_readback"]
                    or max(batch["reset_max_abs_error"], batch["first_target_error_N"],
                           batch["first_lag_error_N"], batch["first_control_lag_error_N"]) >= 1e-6):
                raise ValueError(f"Incorrect reset, motor lag, or case coverage in layout {layout}")
            for row in rows:
                ref = native_cases[row["name"]]
                if row["reason"] != ref["label"] or row["expected"] != ref["label"]:
                    raise ValueError(f"Label disagreement in layout {layout}: {row['name']}")
                tick = row["signed_tick"]
                first = ref["first_touch"]
                if (first is None) != (tick is None) or (first is not None and abs(
                        (tick-1)*receipt["physics_dt_s"] - first["sample_time_s"]) > .002):
                    raise ValueError(f"Signed contact timing disagreement: {row['name']}")
                if tick is not None:
                    timings[row["name"]].append(tick)
        if any(ticks and max(ticks)-min(ticks) > 1 for ticks in timings.values()):
            raise ValueError(f"Signed first-contact time depends on layout {layout} placement")
    authority = read_json(RUN / "action-authority.json")
    current = authority["results"]["current"]["summary"]
    if (authority["scorer_sha256"] != receipt["contact_contract_sha256"]
            or authority["reset_manifest_sha256"] != receipt["reset_manifest_sha256"]
            or authority["rate_caps_rad_s"]["current"] != [2., 2., 2.]
            or authority["bounded_collective_hover_range"] != [.5, 1.5]
            or (current["episodes"], current["qualified"], current["safety_failures"],
                current["saturation_controls"]) != (36, 36, 0, 0)):
        raise ValueError("The original PPO action map must qualify all 36 deterministic starts")
    return receipt


def inputs(receipt):
    cfg = read_json(CONFIG)
    if cfg["schema"] != "rm27_asm2_terminal_recovery_v1" or cfg["braking_horizon_s"] != .8:
        raise ValueError("The frozen braking experiment configuration changed")
    for pin in cfg["pins"].values():
        if digest(ROOT / pin["path"]) != pin["sha256"]:
            raise ValueError(f"Frozen input changed: {pin['path']}")
    manifest_path = ROOT / cfg["pins"]["resets"]["path"]
    if digest(manifest_path) != receipt["reset_manifest_sha256"]:
        raise ValueError("Receipt and frozen reset manifest differ")
    split = read_json(manifest_path)["splits"]["development"]
    archive = manifest_path.parent / split["path"]
    if digest(archive) != split["sha256"]:
        raise ValueError("Development archive changed")
    with np.load(archive, allow_pickle=False) as data:
        selected = np.isin(data["case_ids"], cfg["selection_cases"])
        train_braking = np.flatnonzero(~selected & (data["phase"] == "braking"))
        train_terminal = np.flatnonzero(~selected & (data["phase"] == "terminal"))
        eval_rows = np.flatnonzero(selected & (data["phase"] == "braking"))
        if (len(train_braking), len(train_terminal), len(eval_rows)) != (60, 50, 36):
            raise ValueError("Development case-level split changed")
        indices = np.r_[train_braking, train_terminal]
        bank = {key: data[key][indices] for key in ("states", "motor_thrust_N", "motor_target_N")}
        expected_eval = {(str(data["case_ids"][i]), int(data["source_frame"][i])) for i in eval_rows}
    return cfg, bank, expected_eval, len(train_braking)


class BrakingEnvironment(TerminalEnvironment):
    def __init__(self, sim, data, goal, horizon, budget, settings, n_braking):
        self.n_braking = n_braking
        self.mix = settings["terminal_mix"]
        self.deceleration = settings["braking_decel_m_s2"]
        self.shaping_gap = settings["braking_gap_m"]
        patch = np.concatenate([obj_mesh(sim.spec.source / "mujoco" / entry["mesh"])[0]
                                for entry in sim.spec.entries.values() if entry["group"] in PATCH_GROUPS])
        self.patch = sim.tensor(np.unique(patch, axis=0))
        super().__init__(sim, data, goal, horizon, budget)

    def reset(self, ids, rows=None):
        if rows is None:
            torch = self.torch
            sample = torch.rand(len(ids), device=self.sim.device)
            braking = torch.randint(self.n_braking, (len(ids),), device=self.sim.device)
            terminal = torch.randint(self.n_braking, len(self.data[0]), (len(ids),), device=self.sim.device)
            rows = torch.where(sample < self.mix, terminal, braking)
        super().reset(ids, rows)

    def step(self, action):
        torch = self.torch
        local_normal = self.R.transpose(1, 2) @ self.normal
        support = local_normal @ self.patch.T
        offset, index = support.min(dim=1)
        gap = (self.state[:, :3]-self.center) @ self.normal + offset
        point_rate = torch.linalg.cross(self.state[:, 10:13], self.patch[index], dim=1)
        inward = -(self.state[:, 7:10] @ self.normal + (local_normal*point_rate).sum(dim=1))
        speed_limit = (GENTLE_SPEED_M_S**2+2.*self.deceleration*gap.clamp_min(0.)).sqrt()
        overspeed = ((inward-speed_limit).clamp_min(0.)/.25).square().clamp_max(1.)
        penalty = .02*overspeed*((gap > 0.) & (gap <= self.shaping_gap))
        obs, reward, terminated, truncated, extras = super().step(action)
        return obs, reward-penalty, terminated, truncated, extras


def save(directory, tag, ppo, frames, updates, elapsed, track_sha, profile):
    import torch

    path = directory / f"{tag}.pt"
    temp = path.with_suffix(".tmp")
    weights = {name: value.detach().cpu() for name, value in ppo.actor.state_dict().items()}
    torch.save(dict(format=1, task="asm2_braking_phase4r2", airframe="asm2", actor_obs=45,
                    actor=weights, critic=ppo.critic.state_dict(),
                    value_normalizer=ppo.normalizer.state_dict(),
                    actor_optimizer=ppo.actor_opt.state_dict(), critic_optimizer=ppo.critic_opt.state_dict(),
                    frames=frames, update=updates, learning_s=elapsed, policy_dt_s=profile.policy_dt_s,
                    action_rate_rad_s=[2., 2., 2.], action_collective_hover=[.5, 1.5],
                    track_sha256=track_sha, config_sha256=digest(CONFIG), backend_receipt_sha256=digest(RECEIPT)), temp)
    temp.replace(path)
    reloaded = torch.load(path, map_location="cpu", weights_only=True)
    weight_path = path.with_suffix(".npz")
    temp = weight_path.with_suffix(".tmp")
    with temp.open("wb") as stream:
        np.savez_compressed(stream, **{name: value.numpy() for name, value in reloaded["actor"].items()},
                            checkpoint_sha256=digest(path))
    temp.replace(weight_path)
    return path, weight_path


def first_development_gate(directory, checkpoint, weights, expected_eval, receipt, wait_s, budget):
    result_path = directory / "development-result.json"
    write_json(directory / "development-request.json", {
        "suite": "recovery-development",
        "experiment": str(CONFIG.relative_to(ROOT)),
        "checkpoint": str(checkpoint.relative_to(ROOT)),
        "actor": str(weights.relative_to(ROOT)),
        "output": str(result_path.relative_to(ROOT)),
        "checkpoint_sha256": digest(checkpoint),
        "weights_sha256": digest(weights),
    })
    deadline = time.monotonic()+wait_s
    while time.monotonic() < deadline:
        if budget.stop_reason is not None:
            raise LearningLimit(budget.stop_reason)
        if result_path.exists():
            try:
                report = read_json(result_path)
            except json.JSONDecodeError:
                time.sleep(.25)
                continue
            break
        time.sleep(.25)
    else:
        raise LearningLimit("DEVELOPMENT_RESULT_UNAVAILABLE")
    candidate = report["results"]["candidate"]
    episodes = candidate["episodes"]
    actual = {(row["case"], row["source_frame"]) for row in episodes if row["phase"] == "braking"}
    if (report["schema"] != "rm27_asm2_terminal_native_eval_v1" or report["suite"] != "recovery-development"
            or report["config_sha256"] != digest(CONFIG)
            or report["reset_manifest_sha256"] != receipt["reset_manifest_sha256"]
            or report["evaluator_sha256"] != digest(ROOT / "src/rm27_asm2/terminal_native_eval.py")
            or report["model"]["contract_sha256"] != receipt["contact_contract_sha256"]
            or candidate["actor"]["checkpoint_sha256"] != digest(checkpoint)
            or candidate["actor"]["weights_sha256"] != digest(weights)
            or len(episodes) != 36 or actual != expected_eval):
        raise ValueError("Independent result does not match the checkpoint or frozen braking suite")
    qualified = sum(row["qualified"] for row in episodes)
    exits = sum(row["safety_failure"] is not None for row in episodes)
    passed = qualified >= MIN_QUALIFIED and exits == 0
    write_json(directory / "development-decision.json", {
        "passed": passed, "qualified": qualified, "episodes": 36, "safety_exits": exits,
        "checkpoint_sha256": digest(checkpoint), "result_sha256": digest(result_path),
    })
    print(f"ASM2_4R2_NATIVE_GATE {'PASS' if passed else 'FAIL'} {qualified}/36 exits={exits}", flush=True)
    return passed


def train(receipt):
    import torch
    from simpleflight_lab.ppo import Actor, PPO, ROLLOUT_STEPS, UPSTREAM_REVISION
    from simpleflight_lab.rm27 import RM27_SIM_REVISION
    from matched_contact import corrected_asset

    cfg, data, expected_eval, n_braking = inputs(receipt)
    settings = cfg["learning"]
    profile, spec = load_profile(), load_contact_spec()
    track = flight_settings()["ver2_track_checkpoint"]
    if spec.contract_sha256 != receipt["contact_contract_sha256"] or digest(ROOT / track["path"]) != track["sha256"]:
        raise ValueError("Frozen native scorer or Track checkpoint changed")
    corrected = corrected_asset(spec)
    if digest(corrected[0]) != receipt["asset_sha256"] or settings["worlds"] != receipt["worlds"]:
        raise ValueError("Live collision asset or world count differs from the signed receipt")
    directory = RUN / "learning"
    directory.mkdir(exist_ok=False)
    budget = LearningBudget(MAX_TRANSITIONS, MAX_LEARNING_S)
    settings_receipt = dict(config_sha256=digest(CONFIG), trainer_sha256=digest(__file__),
                            track_sha256=track["sha256"], backend_receipt_sha256=digest(RECEIPT),
                            max_transitions=MAX_TRANSITIONS, max_learning_s=MAX_LEARNING_S,
                            first_gate_transitions=FIRST_TRANSITIONS, first_gate_learning_s=FIRST_LEARNING_S,
                            shaping_gain=.02, shaping_speed_scale_m_s=.25)
    write_result(directory / "authorization.json", settings_receipt)
    sim = env = None
    frames = updates = 0
    evaluation_pause_s = 0.
    saved = {}
    updating = False
    reason = "ERROR"
    gate_passed = False

    def stop(signum, _frame):
        budget.stop_reason = "TIME_LIMIT" if signum == signal.SIGALRM else "INTERRUPTED"

    def save_candidate(tag):
        checkpoint, weights = save(directory, tag, ppo, frames, updates, budget.elapsed, track["sha256"], profile)
        saved[tag] = dict(frames=frames, checkpoint=checkpoint.name, actor=weights.name,
                          checkpoint_sha256=digest(checkpoint), actor_sha256=digest(weights))
        print(f"ASM2_4R2_CHECKPOINT {tag} {frames} {budget.elapsed:.1f}s", flush=True)
        return checkpoint, weights

    try:
        sim = TerminalSim(settings["worlds"], 0, profile, spec, diagnostic=True,
                          physics_dt=receipt["physics_dt_s"], contact_offset_m=receipt["contact_offset_m"],
                          asset_override=corrected)
        if sim.asset_sha256 != receipt["asset_sha256"]:
            raise ValueError("Instantiated collision asset differs from the matched gate")
        goal = spec.target_center + (-spec.patch_face_z_m-cfg["goal_inset_m"])*spec.target_normal
        torch.manual_seed(settings["seed"])
        env = BrakingEnvironment(sim, data, goal, cfg["braking_horizon_s"], budget, settings, n_braking)
        obs = env.observation()
        checkpoint = torch.load(ROOT / track["path"], map_location=sim.device, weights_only=True)
        expected = dict(format=1, task="track", airframe="rm27", actor_obs=45, upstream_revision=UPSTREAM_REVISION,
                        rm27_sim_revision=RM27_SIM_REVISION, policy_dt=profile.policy_dt_s)
        if any(checkpoint.get(key) != value for key, value in expected.items()):
            raise ValueError("Incompatible frozen Track checkpoint")
        actor = Actor(45).to(sim.device)
        actor.load_state_dict(checkpoint["actor"])
        with torch.no_grad():
            actor.log_std.fill_(np.log(settings["initial_std"]))
        ppo = PPO(obs, settings["initial_std"], actor=actor, learning_rate=settings["lr"])
        signal.signal(signal.SIGALRM, stop)
        signal.signal(signal.SIGTERM, stop)
        rollout_size = ROLLOUT_STEPS*sim.worlds
        budget.started = time.perf_counter()
        signal.setitimer(signal.ITIMER_REAL, MAX_LEARNING_S-.5)
        with (directory / "learning.jsonl").open("x", buffering=1) as log:
            while budget.frames+rollout_size <= budget.max_frames:
                budget.check()
                obs, stats = ppo.collect(env, obs)
                budget.check()
                updating = True
                losses = ppo.update(before_minibatch=budget.check)
                updating = False
                if not bool(torch.isfinite(losses).all() and torch.isfinite(stats).all()):
                    raise RuntimeError("Nonfinite PPO update")
                updates += 1
                frames = budget.frames
                if updates == 1 or updates % 4 == 0:
                    row = dict(updates=updates, transitions=frames, learning_s=budget.elapsed,
                               outcomes=dict(env.outcomes), saturations=int(env.saturations),
                               mean_reward=float(stats[0]), error_m=float(stats[1]), loss=losses.tolist())
                    log.write(json.dumps(row, allow_nan=False)+"\n")
                    print("ASM2_4R2_PPO", frames, f"{budget.elapsed:.1f}s", dict(env.outcomes), flush=True)
                if not gate_passed and (frames >= FIRST_TRANSITIONS or budget.elapsed >= FIRST_LEARNING_S):
                    path, weights = save_candidate("development")
                    started = time.perf_counter()
                    signal.setitimer(signal.ITIMER_REAL, 0)
                    try:
                        gate_passed = first_development_gate(directory, path, weights, expected_eval, receipt,
                                                             cfg["development_gate"]["wait_s"], budget)
                    finally:
                        evaluation_pause_s += time.perf_counter()-started
                        budget.started += evaluation_pause_s
                    if not gate_passed:
                        raise LearningLimit("NO_VALIDATED_BRAKING_IMPROVEMENT")
                    budget.check()
                    signal.setitimer(signal.ITIMER_REAL, max(.001, MAX_LEARNING_S-budget.elapsed-.5))
                elif gate_passed and updates % 4 == 0:
                    save_candidate("latest")
            reason = "TRANSITION_LIMIT"
    except LearningLimit as limit:
        reason = str(limit)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        learning_s = budget.elapsed
        try:
            if gate_passed and not updating and frames > max(item["frames"] for item in saved.values()):
                save_candidate("latest")
        finally:
            write_result(directory / "result.json", dict(stop_reason=reason, transitions=budget.frames,
                         completed_transitions=frames, updates=updates, learning_s=learning_s,
                         evaluation_pause_s=evaluation_pause_s, gate_passed=gate_passed, checkpoints=saved,
                         outcomes=dict(env.outcomes) if env else {}, saturations=int(env.saturations) if env else 0,
                         reset_counts=env.reset_counts.tolist() if env else [], partial_update_discarded=updating,
                         backend_receipt_sha256=digest(RECEIPT), config_sha256=digest(CONFIG)))
    return 0


def main():
    try:
        receipt = verify_receipt()
        from isaaclab.app import AppLauncher
        launcher = AppLauncher(headless=True)
        code = train(receipt)
    except Exception:
        traceback.print_exc()
        code = 2
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
