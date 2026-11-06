"""Stage 1 evaluation and bounded engineering checks using the existing PPO.

No trajectory teacher, new optimizer, learned dynamics or recovery controller.
All success statistics here use explicitly provisional simulation contact gates.
"""
from __future__ import annotations

from collections import Counter
from contextlib import contextmanager
import copy
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import gymnasium as gym

from .paths import ROOT


def validate_train_config(config):
    """Task-specific limits without relaxing the historical I3/I4 protocols."""
    protocol = config.get("protocol")
    if protocol not in ("stage1_foundation", "stage1_state_ppo") or config.get("algorithm", "PPO") != "PPO":
        raise ValueError("Stage 1 requires its own PPO protocol")
    if any(config.get(k) is not None for k in ("informed_reset", "tolerance_curriculum", "dynamics_randomization", "observation_history")):
        raise ValueError("Stage 1 foundation does not inherit I4 curricula or randomization")
    ppo = config["ppo"]
    values = dict(seed=config["seed"], n_envs=config["n_envs"], torch_threads=config["torch_threads"],
                  **{k: ppo[k] for k in ("n_steps", "batch_size", "n_epochs")})
    if any(isinstance(v, bool) or not isinstance(v, int) for v in values.values()):
        raise ValueError("Stage 1 resources must be integer values")
    rollout = config["n_envs"] * ppo["n_steps"]
    if (config["seed"] != 0 or config["torch_threads"] != 1 or not 1 <= config["n_envs"] <= 2
            or not 2 <= ppo["n_steps"] <= 512 or not 1 <= ppo["n_epochs"] <= 5
            or not 2 <= ppo["batch_size"] <= rollout or rollout % ppo["batch_size"]):
        raise ValueError("Stage 1 resources exceed the local CPU envelope")
    budget = config["budget"]
    transitions, wall = budget["transitions"], budget["learning_wall_s"]
    upper, wall_upper = (64, 60) if protocol == "stage1_foundation" else (131072, 1800)
    if (isinstance(transitions, bool) or not isinstance(transitions, int)
            or not rollout <= transitions <= upper or transitions % rollout
            or isinstance(wall, bool) or not np.isfinite(wall) or not 0 < wall <= wall_upper):
        raise ValueError("Stage 1 protocol budget exceeded")
    if protocol == "stage1_foundation" and (ppo["n_epochs"] != 1 or config["n_envs"] != 1
                                            or ppo["n_steps"] != 64 or transitions != 64):
        raise ValueError("Foundation permits one 64-transition CPU engineering update only")
    if protocol == "stage1_state_ppo":
        evaluation = config.get("evaluation_budget", {})
        if (config["n_envs"] != 1 or ppo["n_steps"] != 512 or ppo["batch_size"] != 128 or ppo["n_epochs"] != 4
                or set(evaluation) != {"controls", "physics_substeps", "wall_s"}
                or any(isinstance(evaluation.get(key), bool) or not isinstance(evaluation.get(key), int)
                       or not 0 < evaluation[key] <= limit
                       for key, limit in (("controls", 30400), ("physics_substeps", 152000), ("wall_s", 600)))):
            raise ValueError("Stage 1-A requires its approved single-environment PPO and independent evaluation budget")
    for name in ("learning_rate", "gamma", "gae_lambda"):
        value = ppo[name]
        if isinstance(value, bool) or not np.isfinite(value) or not 0 < value <= 1:
            raise ValueError(f"Invalid Stage 1 PPO {name}")
    log_std = ppo.get("policy_kwargs", {}).get("log_std_init", 0.)
    if isinstance(log_std, bool) or not np.isfinite(log_std) or not -3 <= log_std <= 0:
        raise ValueError("Invalid initial policy log standard deviation")
    if config.get("action_parameterization") not in ("hover_centered_v1", "bounds_affine_v1"):
        raise ValueError("Stage 1 must declare the existing CTBR parameterization")
    beta = config.get("rate_scale_beta", 1.)
    if (isinstance(beta, bool) or not isinstance(beta, (int, float))
            or not np.isfinite(beta) or not 0 < beta <= 1):
        raise ValueError("Stage 1 rate_scale_beta must be in (0, 1]")
    if config.get("initial_checkpoint"):
        if (protocol != "stage1_state_ppo" or not isinstance(config["initial_checkpoint"], str)
                or not (ROOT / config["initial_checkpoint"] / "checkpoint.json").is_file()):
            raise ValueError("Stage 1 comparison requires an existing initial checkpoint")
    evidence = config.get("evidence", {})
    if (not isinstance(evidence, dict)
            or set(evidence) - {"all_training_episodes", "all_update_checkpoints"}
            or any(not isinstance(value, bool) for value in evidence.values())):
        raise ValueError("Invalid Stage 1 evidence retention options")
    if not (ROOT / config["env_config"]).is_file():
        raise ValueError("Stage 1 environment configuration is missing")
    return config


def initialize_comparison(model, vec, config):
    """Restore original zero-update parameters/statistics without reseeding/resetting.

    The fresh seed0 construction and first reset follow the original trainer.
    Load tensors into that model instead of constructing another random policy.
    Checkpoint comparison is restricted to the same task with a rate-map change;
    retention options affect evidence I/O, not the rollout or learning algorithm.
    """
    import pickle
    import torch
    from .ppo import _model_state, _state_hash, _task_contract, _policy_observation
    from .provenance import source_hash

    path = (ROOT / config["initial_checkpoint"]).resolve()
    meta = json.loads((path / "checkpoint.json").read_text())
    if (meta["state"]["num_timesteps"] != 0 or meta["state"]["sb3_epoch_updates"] != 0
            or meta["completed_rollout_updates"] != 0):
        raise ValueError("Comparison must start from a zero-update initial checkpoint, never final")
    actual = {k: v for k, v in config.items() if k not in ("initial_checkpoint", "rate_scale_beta", "evidence")}
    expected = {k: v for k, v in meta["config"].items() if k not in ("initial_checkpoint", "rate_scale_beta", "evidence")}
    if actual != expected or meta["task"] != _task_contract(config):
        raise ValueError("Comparison may change only rate scale, source reference and evidence retention")
    for name, digest in meta["files"].items():
        if source_hash(path / name)["sha256"] != digest:
            raise ValueError(f"Initial checkpoint content mismatch: {name}")
    env = vec.venv.envs[0].unwrapped
    for name, source in env.sources.items():
        if source_hash(source)["sha256"] != meta["provenance"]["source_hashes"][name]["sha256"]:
            raise ValueError(f"Initial checkpoint environment changed: {name}")
    np.testing.assert_array_equal(_policy_observation(vec.venv.envs[0]),
                                  np.asarray(meta["probe_observation"], np.float32))
    with (path / "normalize.pkl").open("rb") as stream:
        saved = pickle.load(stream)
    for name in ("norm_obs", "norm_reward", "clip_obs", "clip_reward", "gamma", "epsilon", "norm_obs_keys", "training"):
        if getattr(vec, name) != getattr(saved, name):
            raise ValueError(f"Initial normalization setting changed: {name}")
    before = _model_state(model, vec)
    rng_before = _state_hash((torch.get_rng_state(), np.random.get_state()))
    model.set_parameters(path / "model.zip", exact_match=True, device="cpu")
    for name in ("obs_rms", "ret_rms", "old_obs", "old_reward"):
        setattr(vec, name, copy.deepcopy(getattr(saved, name)))
    restored = _model_state(model, vec)
    if restored != meta["state"]:
        raise ValueError("Initial model/optimizer/normalizer/counters did not restore exactly")
    if _state_hash((torch.get_rng_state(), np.random.get_state())) != rng_before:
        raise ValueError("Loading the initial comparison changed the seeded RNG state")
    return dict(source_checkpoint=str(path), files=meta["files"], restored_state=restored,
                fresh_seeded_state_before_restore=before, exact_original_initial_state=True,
                seeded_rng_preserved=True, first_reset_observation_identical=True,
                changed_algorithm_field="rate_scale_beta", beta=config.get("rate_scale_beta", 1.))


def check_training_scope(config):
    """Provisional gates are usable for engineering, not a formal learning run."""
    if config["protocol"] == "stage1_foundation":
        return
    from .stage1_env import load_stage1_config
    task = load_stage1_config(ROOT / config["env_config"])
    if task["parameter_status"] != "CONFIRMED" or not task["training_allowed"]:
        raise ValueError("Confirm the proposed Stage 1 physical/task parameters before formal PPO training")


def evaluation_initials(env):
    """Nineteen predeclared starts; always retain every result, including failures."""
    from .stage1_env import fixed_evaluation_initials
    names = ["nominal"]
    for group, start in (("position", 0), ("velocity", 3), ("attitude", 6)):
        for axis in range(3):
            for sign in (-1, 1):
                names.append(f"{group}_{axis}_{sign:+d}")
    return list(zip(names, fixed_evaluation_initials(env.rl_config)))


class SmokeBudget:
    def __init__(self, *, controls=20000, substeps=100000, wall_s=900):
        self.max_controls, self.max_substeps = controls, substeps
        self.controls = self.substeps = 0
        self.started = time.monotonic()
        self.deadline = self.started + wall_s
        self.wall_limit_s = wall_s
        self.paused_at = None

    def before_step(self):
        if self.paused_at is not None:
            raise RuntimeError("Cannot consume a paused Stage 1 budget")
        if (self.controls >= self.max_controls or self.substeps + 5 > self.max_substeps
                or time.monotonic() >= self.deadline):
            raise TimeoutError("Stage 1 engineering smoke budget exhausted")

    def record(self, info):
        self.controls += 1
        self.substeps += int(info["physics_substeps"])

    def summary(self):
        return dict(reference_controls=self.controls, physics_substeps=self.substeps,
                    elapsed_wall_s=(time.monotonic() if self.paused_at is None else self.paused_at) - self.started,
                    limits=dict(controls=self.max_controls, physics_substeps=self.max_substeps, wall_s=self.wall_limit_s))

    def pause(self):
        if self.paused_at is not None:
            raise RuntimeError("Budget is already paused")
        self.paused_at = time.monotonic()

    def resume(self):
        if self.paused_at is None:
            raise RuntimeError("Budget is already running")
        idle = time.monotonic() - self.paused_at
        self.started += idle
        self.deadline += idle
        self.paused_at = None

    @contextmanager
    def activity(self):
        """Charge evaluation phases together without charging intervening learning."""
        self.resume()
        try:
            yield self
        finally:
            self.pause()


class BudgetedTrainingEnv(gym.Wrapper):
    """Count every consumed training transition, including a discarded partial rollout."""
    def __init__(self, env, budget, *, output=None, record_all=False):
        super().__init__(env)
        self.budget = budget
        self.output = None if output is None else Path(output)
        self.saved_kinds = set()
        self.rows = []
        self.episode_index = 0
        self.record_all = record_all

    def reset(self, **kwargs):
        result = self.env.reset(**kwargs)
        raw = self.unwrapped
        self.initial_state = raw.plant.snapshot()
        self.rows = [_episode_sample(raw, np.zeros(4), raw.previous_action_ctbr, 0., result[1])]
        return result

    def step(self, action):
        from .ppo import _WallLimit
        try:
            self.budget.before_step()
        except TimeoutError as error:
            raise _WallLimit(str(error)) from error
        from .ppo import NormalizedCTBR
        wrapper = self.env
        while not isinstance(wrapper, NormalizedCTBR):
            wrapper = wrapper.env
        requested = wrapper.action(action)
        result = self.env.step(action)
        self.budget.record(result[4])
        self.rows.append(_episode_sample(self.unwrapped, action, requested, result[1], result[4]))
        if result[2] or result[3]:
            terminal = result[4]["result"]
            kind = "success" if terminal["passed"] else "+".join(terminal["failure_modes"])
            if self.output is not None:
                self.output.mkdir(parents=True, exist_ok=True)
                with (self.output / "episodes.jsonl").open("a") as stream:
                    stream.write(json.dumps(dict(episode_index=self.episode_index,
                        training_transition=self.budget.controls, **terminal), allow_nan=False)+"\n")
                if self.record_all or (kind not in self.saved_kinds and (len(self.saved_kinds) < 12 or kind == "success")):
                    summary = dict(terminal, case=kind, initial_perturbation=self.unwrapped.initial_perturbation.tolist(),
                        total_reward=float(sum(row["reward"] for row in self.rows)), fixture_state=None,
                        policy="STOCHASTIC_TRAINING_EPISODE; updates may occur across rollout boundaries",
                        execution_status="COMPLETE", training_transition=self.budget.controls)
                    _write_episode(self.output / f"episode_{self.episode_index:05d}", self.unwrapped,
                                   self.rows, self.initial_state, summary)
                    self.saved_kinds.add(kind)
            self.episode_index += 1
        failures = result[4].get("result", {}).get("failure_modes", [])
        if any(reason in failures for reason in ("NUMERICAL_FAILURE", "MUJOCO_WARNING")):
            raise FloatingPointError(f"Stage 1 physics execution failed: {failures}")
        return result


    def flush_partial(self):
        """Retain the unfinished final training episode without inventing an event."""
        if not self.record_all or self.output is None or len(self.rows) < 2 or not self.unwrapped._active:
            return
        env = self.unwrapped
        summary = dict(env.evaluator.summary(), case="UNFINISHED_AT_TRAINING_END",
                       total_reward=float(sum(row["reward"] for row in self.rows)),
                       initial_perturbation=env.initial_perturbation.tolist(), fixture_state=None,
                       policy="STOCHASTIC_TRAINING_EPISODE; updates may occur across rollout boundaries",
                       execution_status="INCOMPLETE_TRAINING_END", training_transition=self.budget.controls)
        _write_episode(self.output / f"episode_{self.episode_index:05d}_partial", env,
                       self.rows, self.initial_state, summary)


def _episode_sample(env, action, requested, reward, detail, actor_mean=None):
    plant, state = env.plant, env.plant.state
    return dict(time_s=float(plant.data.time), qpos=plant.data.qpos.copy(), qvel=plant.data.qvel.copy(),
                motor_state_N=plant.data.act.copy(), motor_commands_N=plant.data.ctrl.copy(),
                position=state.position, velocity=state.velocity, rotation=state.rotation, omega_body=state.omega_body,
                action_normalized=np.asarray(action).copy(), action_ctbr=env.previous_action_ctbr.copy(),
                action_requested_ctbr=np.asarray(requested).copy(), observation=env._observation(), reward=float(reward),
                actor_mean_normalized=np.asarray(action if actor_mean is None else actor_mean).copy(),
                action_clipped=bool(detail.get("action_clipped", False)),
                proxy_saturated=bool(detail.get("proxy_saturated", False)),
                physics_substeps=int(detail["physics_substeps"]))


def _write_episode(output, env, rows, initial_state, result):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    arrays = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    np.savez_compressed(output / "trajectory.npz", initial_integration_state=initial_state, **arrays)
    config = dict(rl=env.rl_config, task_config_path=str(env.sources["rl_config"]),
                  private_model_path=str(env.plant.sources["private_scene_binary"]))
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    write_plot(output, arrays, result)


def record_episode(episode, output, perturbation, *, name="nominal", model=None, vec=None,
                   seed=0, budget=None, controller=None, fixture_state=None):
    """Run a complete physical episode; manual stepping avoids VecEnv auto-reset."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    raw, info = episode.reset(seed=seed, options={"initial_perturbation": np.asarray(perturbation).tolist()})
    env = episode.unwrapped
    if fixture_state is not None:
        _initialize_fixture(env, fixture_state)
        raw = env._observation()
        info = env._info(0, env.episode_start, False, False, dict.fromkeys(env.reward_component_totals, 0.))
    initial_state = env.plant.snapshot()
    rows = []
    requested_ctbr = env.previous_action_ctbr.copy()
    actor_mean = np.zeros(4)
    wrapper = episode
    from .ppo import NormalizedCTBR
    while not isinstance(wrapper, NormalizedCTBR):
        wrapper = wrapper.env

    def sample(action, reward, detail):
        rows.append(_episode_sample(env, action, requested_ctbr, reward, detail, actor_mean))

    sample(np.zeros(4), 0., info)
    total = 0.
    done = not env._active
    budget_error = None
    while not done:
        if budget is not None:
            try:
                budget.before_step()
            except TimeoutError as error:
                budget_error = str(error)
                break
        if model is not None:
            import torch
            normalized = vec.normalize_obs(raw)
            action, _ = model.predict(normalized, deterministic=True)
            with torch.no_grad():
                actor_mean = model.policy.get_distribution(torch.as_tensor(
                    normalized.reshape(1, -1), device=model.device)).distribution.mean.cpu().numpy().ravel()
        elif controller is not None:
            action = np.asarray(controller(env), float)
        else:
            action = np.zeros(4)  # hover-centered wrapper, explicitly no learned policy
        requested_ctbr = wrapper.action(action)
        raw, reward, done, truncated, info = episode.step(action)
        if truncated:
            raise RuntimeError("Stage 1 task deadlines are terminations, not rollout truncations")
        if budget is not None:
            budget.record(info)
        total += float(reward)
        sample(action, reward, info)
    result = copy.deepcopy(info["result"] if done else env.evaluator.summary())
    result.update(case=name, initial_perturbation=np.asarray(perturbation).tolist(), total_reward=total,
                  observation_schema=env.observation_schema, task_id=env.task_id,
                  policy="DETERMINISTIC_CHECKPOINT" if model is not None else "ENGINEERING_CONTROL_FIXTURE",
                  fixture_state=fixture_state,
                  task_parameters_confirmed=env.rl_config["parameter_status"] == "CONFIRMED",
                  confirmation_scope=env.rl_config.get("confirmation_scope", "PROVISIONAL_ENGINEERING_ONLY"),
                  execution_status="COMPLETE" if done else "INCOMPLETE_BUDGET",
                  budget_error=budget_error)
    _write_episode(output, env, rows, initial_state, result)
    return result


def write_plot(output, arrays, result):
    """Standalone telemetry plots with no plotting dependency or external service."""
    from html import escape
    time_values = arrays["time_s"]
    span = max(float(time_values[-1] - time_values[0]), .001)
    colors = ["#2563eb", "#ea580c", "#16a34a", "#9333ea"]
    panels = []
    from scipy.spatial.transform import Rotation
    attitude = np.degrees(Rotation.from_matrix(arrays["rotation"]).as_rotvec())
    for title, values in (("Position [m]", arrays["position"]), ("Velocity [m/s]", arrays["velocity"]),
                          ("Attitude rotation vector [deg]", attitude),
                          ("CTBR [N, rad/s]", arrays["action_ctbr"])):
        low, high = float(values.min()), float(values.max())
        scale = max(high - low, .01)
        lines = []
        for index in range(values.shape[1]):
            points = " ".join(f"{50 + (t-time_values[0])/span*680:.2f},{190-(v-low)/scale*150:.2f}"
                              for t, v in zip(time_values, values[:, index]))
            lines.append(f'<polyline fill="none" stroke="{colors[index]}" stroke-width="1.5" points="{points}"/>')
        panels.append(f'<h2>{title}</h2><svg viewBox="0 0 780 230"><path d="M50 30 V190 H735" fill="none" stroke="#aaa"/>'
                      f'<text x="5" y="40">{high:.3g}</text><text x="5" y="190">{low:.3g}</text>'
                      + "".join(lines) + f'<text x="50" y="220">0 s</text><text x="670" y="220">{span:.3f} s</text></svg>')
    (Path(output) / "telemetry.html").write_text('<!doctype html><meta charset="utf-8"><title>RM27 Stage 1 episode</title>'
        '<style>body{font:16px system-ui;max-width:1000px;margin:32px auto}svg{width:100%;border:1px solid #ddd}h2{font-size:18px}</style>'
        '<h1>RM27 Stage 1 recorded episode</h1><p>Simulation engineering evidence; provisional contact criteria; no trained capability claim.</p>'
        f'<pre>{escape(json.dumps({k: result.get(k) for k in ("case", "passed", "failure_modes", "total_reward")}, indent=2))}</pre>'
        '<p>Axes: blue=x/collective, orange=y/roll rate, green=z/pitch rate, purple=yaw rate.</p>' + "".join(panels))


def evaluate_stage1(checkpoint, output, *, budget=None):
    from .ppo import _load_checkpoint, _normalization_state, _state_hash, STAGE1_TASK
    from .provenance import provenance
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    initialization_started = time.monotonic()
    model, vec, meta = _load_checkpoint(checkpoint, training=False, n_envs=1, expected_task=STAGE1_TASK)
    initialization_wall_s = time.monotonic() - initialization_started
    try:
        stats = _state_hash(_normalization_state(vec))
        wrapped = vec.venv.envs[0]
        env = wrapped.unwrapped
        episodes = [record_episode(wrapped, output / f"episode_{index:02d}", perturbation,
                                   name=name, model=model, vec=vec, seed=0, budget=budget)
                    for index, (name, perturbation) in enumerate(evaluation_initials(env))]
        if stats != _state_hash(_normalization_state(vec)):
            raise RuntimeError("Stage 1 evaluation changed normalization statistics")
        failures = Counter(reason for row in episodes for reason in row["failure_modes"])
        success = sum(bool(row["passed"]) for row in episodes)
        complete = all(row["execution_status"] == "COMPLETE" for row in episodes)
        summary = dict(task_id=env.task_id, observation_schema=env.observation_schema,
                       passed=complete and success == len(episodes), complete=complete,
                       success_count=success, episode_count=len(episodes),
                       success_rate=success / len(episodes), failure_counts=dict(failures),
                       mean_total_reward=float(np.mean([r["total_reward"] for r in episodes])),
                       task_result="INCOMPLETE_BUDGET" if not complete else "PASS" if success == len(episodes) else "FAIL",
                       parameters_confirmed=env.rl_config["parameter_status"] == "CONFIRMED",
                       confirmation_scope=env.rl_config.get("confirmation_scope", "PROVISIONAL_ENGINEERING_ONLY"),
                       initialization_wall_s=initialization_wall_s,
                       scope="Independent physical-contact assessment under provisional simulation gates",
                       normalization_frozen=True, normalization_sha256=stats,
                       checkpoint_state=meta["state"], provenance=provenance(env, algorithm="PPO_CPU", seed=0),
                       episodes=episodes)
        (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        return summary
    finally:
        vec.close()


def record_checkpoint_behavior(checkpoint, output, budget):
    """Same nominal hover, deterministic policy and frozen checkpoint normalization."""
    from .ppo import _load_checkpoint, _normalization_state, _state_hash, STAGE1_TASK
    model, vec, meta = _load_checkpoint(checkpoint, training=False, n_envs=1, expected_task=STAGE1_TASK)
    try:
        state = _state_hash(_normalization_state(vec))
        result = record_episode(vec.venv.envs[0], output, np.zeros(12), model=model, vec=vec,
                                budget=budget, name="nominal")
        if state != _state_hash(_normalization_state(vec)):
            raise RuntimeError("Behavior comparison changed normalization statistics")
        return dict(passed=result["passed"], failure_modes=result["failure_modes"],
                    execution_status=result["execution_status"], checkpoint_state=meta["state"],
                    normalization_frozen=True)
    finally:
        vec.close()


def verify_episode_replay(episode_dir, *, budget=None):
    """Re-execute recorded CTBR through MuJoCo from the complete initial state."""
    from .stage1_env import Stage1ContactEnv
    from .stage1_plant import Stage1Plant
    metadata = json.loads((Path(episode_dir) / "config.json").read_text())
    plant = Stage1Plant(metadata["rl"], model_path=metadata["private_model_path"])
    if json.loads(Path(metadata["task_config_path"]).read_text()) != metadata["rl"]:
        raise ValueError("Recorded task configuration changed before replay")
    env = Stage1ContactEnv(metadata["task_config_path"], plant=plant)
    archive = np.load(Path(episode_dir) / "trajectory.npz")
    result = json.loads((Path(episode_dir) / "summary.json").read_text())
    if result.get("execution_status", "COMPLETE") != "COMPLETE":
        raise ValueError("Incomplete budget-limited trajectory is not a complete episode replay")
    env.reset(seed=0, options={"initial_perturbation": result["initial_perturbation"]})
    if result.get("fixture_state") is not None:
        _initialize_fixture(env, result["fixture_state"])
    # Normal reset sets evaluator history; restore adds exact integrator warm-start state.
    plant.restore(archive["initial_integration_state"])
    errors = {name: 0. for name in ("qpos", "qvel", "motor_state_N", "motor_commands_N", "observation", "reward")}
    info = env._info(0, env.episode_start, False, False, dict.fromkeys(env.reward_component_totals, 0.))
    try:
        # Frame zero is evidence too, especially an initially terminated
        # overlap episode with no step loop to check it later.
        initial = dict(qpos=plant.data.qpos, qvel=plant.data.qvel, motor_state_N=plant.data.act,
                       motor_commands_N=plant.data.ctrl, observation=env._observation(), reward=0.)
        for key, value in initial.items():
            errors[key] = float(np.max(np.abs(np.asarray(value)-archive[key][0])))
        if (abs(plant.data.time-archive["time_s"][0]) > 1e-12
                or bool(env.evaluator.done) != (len(archive["time_s"]) == 1)):
            raise RuntimeError("Replay initial time/termination differs from the recorded episode")
        for index in range(1, len(archive["time_s"])):
            if budget is not None:
                budget.before_step()
            observation, reward, done, truncated, info = env.step(archive["action_requested_ctbr"][index])
            if budget is not None:
                budget.record(info)
            actual = dict(qpos=plant.data.qpos, qvel=plant.data.qvel, motor_state_N=plant.data.act,
                          motor_commands_N=plant.data.ctrl, observation=observation, reward=reward)
            for key, value in actual.items():
                errors[key] = max(errors[key], float(np.max(np.abs(np.asarray(value) - archive[key][index]))))
            if (info["physics_substeps"] != archive["physics_substeps"][index]
                    or abs(plant.data.time - archive["time_s"][index]) > 1e-12
                    or done != (index == len(archive["time_s"]) - 1) or truncated):
                raise RuntimeError("Replay termination/time differs from recorded physical episode")
        matching_result = info["result"]["passed"] == result["passed"] and info["result"]["failure_modes"] == result["failure_modes"]
        passed = max(errors.values()) <= 1e-12 and matching_result
        return dict(passed=passed, max_absolute_errors=errors, matching_terminal_result=matching_result,
                    physics_intervals=int(archive["physics_substeps"].sum()),
                    scope="Complete-episode recorded-control MuJoCo re-simulation; not a new policy trial")
    finally:
        env.close()


def _initialize_fixture(env, state):
    """Engineering-only near-contact fixture, never part of training reset."""
    from .stage1_evaluation import Stage1Evaluator
    env.plant.reset(**{key: np.asarray(value, float) for key, value in state.items()})
    env.episode_start = float(env.plant.data.time)
    env.previous_action_ctbr = np.r_[env.plant.hover_force, np.zeros(3)]
    env.evaluator = Stage1Evaluator(env.rl_config, env.plant)
    env.evaluator.initialize()
    env._active = not env.evaluator.done
    env._terminal_result = env.evaluator.summary() if env.evaluator.done else None


def run_smoke(config_path, output, *, previous_attempt=None):
    """One 64-transition update, fixed evaluation and real contact/replay checks.

    Fixtures test event plumbing only. They are neither demonstrations nor
    training starts; the independent policy evaluation always uses hover starts.
    """
    import hashlib
    import tarfile
    import traceback
    import gymnasium as gym
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.logger import configure
    from .ppo import (load_train_config, make_vec_env, NormalizedCTBR, _algorithm_kwargs,
                      _learn_rollout, _save_checkpoint, _load_checkpoint, _model_state)
    from .provenance import provenance
    from .stage1_env import Stage1ContactEnv

    config_path, output = Path(config_path).resolve(), Path(output).resolve()
    config = load_train_config(config_path)
    if config["protocol"] != "stage1_foundation":
        raise ValueError("stage1-smoke accepts only the fixed engineering protocol")
    output.mkdir(parents=True, exist_ok=False)
    budget = SmokeBudget()
    previous_wall_s = 0.
    summary = dict(passed=False, scope="BOUNDED_TASK_FOUNDATION_ENGINEERING",
                   policy_capability="NOT_TRAINED_ONE_UPDATE_ENGINEERING_ONLY",
                   budget=dict(controls=20000, physics_substeps=100000, wall_s=900,
                               learning_transitions=64, learning_wall_s=60),
                   fixtures={}, replays={})
    if previous_attempt is not None:
        previous_attempt = Path(previous_attempt).resolve()
        previous = json.loads((previous_attempt / "summary.json").read_text())
        if (previous.get("scope") != summary["scope"] or previous.get("update")
                or previous.get("passed") or previous.get("stop_reason") != "ENGINEERING_ERROR"):
            raise ValueError("Only a recorded pre-update engineering failure can continue under the same budget")
        budget.controls = previous["cost"]["reference_controls"]
        budget.substeps = previous["cost"]["physics_substeps"]
        previous_wall_s = previous["cost"].get("cumulative_elapsed_wall_s", previous["cost"]["elapsed_wall_s"]
            + previous.get("previous_attempt", {}).get("cost", {}).get("elapsed_wall_s", 0.))
        budget.deadline -= previous_wall_s
        summary["previous_attempt"] = dict(path=str(previous_attempt), error=previous["error"],
                                           cost=previous["cost"], source_snapshot_preserved=True)
        summary["fixtures"] = copy.deepcopy(previous["fixtures"])
        for name, row in summary["fixtures"].items():
            row.setdefault("evidence_directory", str(previous_attempt / "fixtures" / name))
    vec = wrapped = None
    digest = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    try:
        task_path = ROOT / config["env_config"]
        wrapped = NormalizedCTBR(Stage1ContactEnv(task_path), config["action_parameterization"])
        env = wrapped.unwrapped
        env.reset(seed=0, options={"initial_perturbation": np.zeros(12)})
        source = provenance(env, algorithm="PPO_CPU", seed=0,
                            extra_sources={"training_config": config_path})
        (output / "provenance.json").write_text(json.dumps(source, indent=2) + "\n")
        frozen = {str((ROOT / row["path"]).resolve()): row["sha256"]
                  for row in [*source["source_hashes"].values(), *source["code_hashes"]]}
        # Snapshot actual dirty source bytes as well as their identities.
        with tarfile.open(output / "sources.tar.gz", "w:gz") as archive:
            for index, path in enumerate(sorted(frozen)):
                if Path(path).suffix != ".mjb":
                    archive.add(path, arcname=f"{index:03d}/{Path(path).name}")
        (output / "sources.json").write_text(json.dumps(frozen, indent=2) + "\n")
        g = env.geometry
        rotation = g["target_rotation"]

        def fixture(gap, *, across=0., velocity=None, omega=None):
            position = (g["center"] + across*rotation[:, 1]
                        + (g["radius"]+gap)*g["normal"]-rotation@g["offset_body"])
            return dict(position=position.tolist(), rotation=rotation.tolist(),
                        velocity=(-.2*g["normal"] if velocity is None else velocity).tolist(),
                        omega_body=(np.zeros(3) if omega is None else omega).tolist())

        specs = [("hover_timeout", None, None, "CONTACT_TIMEOUT"),
                 ("designated_contact", fixture(.001), None, "CONTROLLED_CONTACT"),
                 ("initial_overlap", fixture(-.001), None, "INITIAL_OVERLAP"),
                 ("outside_region", fixture(.001, across=.06), None, "CONTACT_OUTSIDE_REGION")]
        boundary = dict(position=(g["center"]+rotation@np.array([.799, 0., 1.])).tolist(),
                        velocity=(.5*rotation[:, 0]).tolist(), rotation=np.eye(3).tolist(),
                        omega_body=np.zeros(3).tolist())
        specs.append(("partial_control_exit", boundary, None, "FLIGHT_VOLUME_EXIT"))
        specs.append(("clipped_saturated", None, lambda _: np.array([-2., 2., -2., 2.]), None))
        for name, state, controller, expected in specs:
            if name in summary["fixtures"]:
                continue  # Preserve completed evidence; do not repeat physical trials.
            result = record_episode(wrapped, output / "fixtures" / name, np.zeros(12),
                                    name=name, budget=budget, fixture_state=state, controller=controller)
            actual = "CONTROLLED_CONTACT" if result["passed"] else result["failure_modes"]
            check = expected is None or (actual == expected if isinstance(actual, str) else expected in actual)
            summary["fixtures"][name] = dict(result=result, expected=expected, engineering_passed=check,
                                             evidence_directory=str(output / "fixtures" / name))
        # An unsuccessful fixture remains unsuccessful. Interface checks still
        # proceed so task/asset blockers do not hide update or evaluation bugs.
        summary["fixture_expectation_mismatches"] = [name for name, row in summary["fixtures"].items()
                                                     if not row["engineering_passed"]]
        summary["controlled_contact_fixture"] = ("PROVISIONAL_PASS" if
            summary["fixtures"]["designated_contact"]["result"]["passed"] else "FAIL")
        wrapped.close()
        wrapped = None

        torch.set_num_threads(1)
        vec = make_vec_env(config)

        class CountIntervals(gym.Wrapper):
            def step(self, action):
                budget.before_step()
                result = self.env.step(action)
                budget.record(result[4])
                return result

        vec.venv.envs[0] = CountIntervals(vec.venv.envs[0])
        model = PPO("MlpPolicy", vec, seed=0, device="cpu", verbose=0, **_algorithm_kwargs(config))
        model.set_logger(configure(format_strings=[]))
        vec.reset()
        env = vec.venv.envs[0].unwrapped
        source = provenance(env, algorithm="PPO_CPU", seed=0,
                            extra_sources={"training_config": config_path})
        before = {name: parameter.detach().cpu().clone() for name, parameter in model.policy.named_parameters()}
        learning_start = time.monotonic()
        update = _learn_rollout(model, vec, min(budget.deadline, learning_start+60), 64)
        update["learning_wall_s"] = time.monotonic()-learning_start
        update["parameter_delta_l2"] = {
            group: float(sum((parameter.detach().cpu()-before[name]).square().sum().item()
                             for name, parameter in model.policy.named_parameters()
                             if any(key in name for key in keys))**.5)
            for group, keys in (("actor", ("policy_net", "action_net", "log_std")),
                                ("critic", ("value_net",))) }
        if not all(value > 0 and np.isfinite(value) for value in update["parameter_delta_l2"].values()):
            raise RuntimeError("Both actor and critic must receive finite nonzero engineering updates")
        saved = _save_checkpoint(output / "checkpoint", model, vec, config, source, 1)
        (output / "update.json").write_text(json.dumps(update, indent=2, allow_nan=False)+"\n")
        summary["update"] = update
        vec.close()
        vec = None
        restored, restored_vec, meta = _load_checkpoint(output / "checkpoint", training=False, n_envs=1,
                                                       expected_task=env.task_id)
        try:
            summary["checkpoint"] = dict(passed=_model_state(restored, restored_vec) == saved["state"],
                                         exact_policy_optimizer_statistics_counters=True,
                                         exact_deterministic_probe_action=True, state=meta["state"])
        finally:
            restored_vec.close()
        evaluation = evaluate_stage1(output / "checkpoint", output / "evaluation", budget=budget)
        summary["evaluation"] = {key: evaluation[key] for key in
                                 ("success_count", "episode_count", "success_rate", "failure_counts",
                                  "normalization_frozen", "parameters_confirmed")}
        for name, path in (("hover_timeout", summary["fixtures"]["hover_timeout"]["evidence_directory"]),
                           ("designated_contact", summary["fixtures"]["designated_contact"]["evidence_directory"]),
                           ("clipped_saturated", summary["fixtures"]["clipped_saturated"]["evidence_directory"]),
                           ("policy_nominal", output / "evaluation/episode_00")):
            replay = verify_episode_replay(path, budget=budget)
            summary["replays"][name] = replay
            if not replay["passed"]:
                raise RuntimeError(f"Full physical replay differed for {name}")
        changed = [path for path, expected in frozen.items() if digest(path) != expected]
        protection = output.parent / "inputs.json"
        protected = json.loads(protection.read_text())["protected_sha256"] if protection.is_file() else {}
        changed_old = [path for path, expected in protected.items() if digest(path) != expected]
        summary["source_protection"] = dict(frozen_files=len(frozen), protected_existing_files=len(protected),
                                             changed_frozen=changed, changed_existing=changed_old)
        if changed or changed_old:
            raise RuntimeError("Source or protected historical evidence changed during smoke")
        summary["passed"] = True
        summary["passed_meaning"] = "INTERFACES_UPDATE_CHECKPOINT_EVALUATION_REPLAY; NOT_TASK_SUCCESS"
        summary.setdefault("controlled_contact_fixture", "PROVISIONAL_PASS")
        summary["formal_training_ready"] = False  # all physical/task gates remain proposals
        summary["stop_reason"] = "DECLARED_ENGINEERING_CHECKS_COMPLETED"
    except Exception as exc:
        summary.update(error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc(),
                       stop_reason="BUDGET_EXHAUSTED" if isinstance(exc, TimeoutError) else "ENGINEERING_ERROR")
    finally:
        if wrapped is not None:
            wrapped.close()
        if vec is not None:
            vec.close()
        summary["cost"] = budget.summary()
        summary["cost"]["cumulative_elapsed_wall_s"] = previous_wall_s+summary["cost"]["elapsed_wall_s"]
        (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    return summary
