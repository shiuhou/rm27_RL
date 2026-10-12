"""SB3 CPU PPO integration; physics, reward and scoring remain in the I1 env.

One run includes update/state checks. No independent optimization test budget,
curriculum, reward shaping or privileged policy input is introduced here.
"""
from __future__ import annotations

import hashlib
import inspect
import json
from pathlib import Path
import shutil
import time
import tempfile
import traceback

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from .env import ContactRecoveryEnv
from .provenance import provenance, source_hash
from .rm27_adapter import ROOT

DEFAULT_TRAIN_CONFIG = ROOT / "configs/contact_rl/ppo_minimal.json"


def load_train_config(path=DEFAULT_TRAIN_CONFIG):
    config = json.loads(Path(path).read_text())
    if config.get("schema_version") != 1:
        raise ValueError("PPO config requires schema_version 1")
    rollout = config["n_envs"] * config["ppo"]["n_steps"]
    budget = config["budget"]
    if (config["seed"] != 0 or config["n_envs"] != 4 or config["torch_threads"] != 1
            or rollout != 2048 or config["ppo"]["n_steps"] != 512
            or not 6144 <= budget["transitions"] <= 65536
            or budget["transitions"] % rollout
            or not 0 < budget["learning_wall_s"] <= 1800):
        raise ValueError("I3 requires seed 0, four CPU envs and the approved bounded rollout budget")
    if not (ROOT / config["env_config"]).is_file():
        raise ValueError("PPO env_config must locate existing checkout assets")
    return config


class NormalizedCTBR(gym.ActionWrapper):
    """Affine [-1,1] action to the existing physical CTBR input boundary."""

    def __init__(self, env):
        super().__init__(env)
        # Use the environment's double-precision physical limits, not rounded
        # Box metadata; preserve the original linear map and proxy chain.
        self.low = env.unwrapped._action_low.copy()
        self.high = env.unwrapped._action_high.copy()
        self.action_space = gym.spaces.Box(-1., 1., shape=(4,), dtype=np.float32)
        hover = np.r_[env.unwrapped.plant.hover_force, np.zeros(3)]
        self.hover_action = 2 * (hover - self.low) / (self.high - self.low) - 1

    def action(self, action):
        requested = np.asarray(action, dtype=float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("Normalized CTBR requires four finite values")
        return self.low + (np.clip(requested, -1., 1.) + 1) * (self.high - self.low) / 2


def _make_dummy_vec(config, n_envs=None):
    path = ROOT / config["env_config"]
    count = config["n_envs"] if n_envs is None else n_envs
    # Every factory creates its own model/data/evaluator/proxy/action memory.
    vec = DummyVecEnv([lambda: Monitor(NormalizedCTBR(ContactRecoveryEnv(path)))
                       for _ in range(count)])
    vec.seed(config["seed"])
    return vec


def make_vec_env(config, n_envs=None, training=True):
    return VecNormalize(_make_dummy_vec(config, n_envs), training=training,
                        norm_obs=True, norm_reward=False)


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _state_hash(value):
    """Content comparison of tensors/statistics, independent of pickle bytes."""
    digest = hashlib.sha256()
    def visit(item):
        if isinstance(item, torch.Tensor):
            item = item.detach().cpu().numpy()
        if isinstance(item, np.ndarray):
            digest.update(str((item.dtype.str, item.shape)).encode())
            digest.update(np.ascontiguousarray(item).tobytes())
        elif isinstance(item, dict):
            for key in sorted(item, key=str):
                digest.update(str(key).encode())
                visit(item[key])
        elif isinstance(item, (tuple, list)):
            for child in item:
                visit(child)
        else:
            digest.update(repr(item).encode())
    visit(value)
    return digest.hexdigest()


def _normalization_state(vec):
    return {name: {key: np.asarray(getattr(getattr(vec, name), key)).copy()
                   for key in ("mean", "var", "count")}
            for name in ("obs_rms", "ret_rms")}


def _assert_finite(value):
    if isinstance(value, dict):
        for item in value.values():
            _assert_finite(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _assert_finite(item)
    elif isinstance(value, torch.Tensor):
        if not torch.isfinite(value).all():
            raise FloatingPointError("Nonfinite PPO tensor")
    elif isinstance(value, (np.ndarray, float)) and not np.isfinite(value).all():
        raise FloatingPointError("Nonfinite PPO value")


def _model_state(model, vec):
    return dict(policy_sha256=_state_hash(model.policy.state_dict()),
                optimizer_sha256=_state_hash(model.policy.optimizer.state_dict()),
                normalization_sha256=_state_hash(_normalization_state(vec)),
                num_timesteps=int(model.num_timesteps), sb3_epoch_updates=int(model._n_updates))


def _save_checkpoint(path, model, vec, config, source, completed_updates):
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    previous = destination.with_name(f".{destination.name}.previous")
    if previous.exists():
        raise RuntimeError("Unexpected unfinished checkpoint replacement; preserve it for inspection")
    with tempfile.TemporaryDirectory(prefix=f".{destination.name}-", dir=destination.parent) as staging:
        path = Path(staging)
        model.save(path / "model.zip")
        vec.save(path / "normalize.pkl")
        raw_probe = vec.venv.envs[0].unwrapped._observation()
        normalized = vec.normalize_obs(raw_probe)
        action, _ = model.predict(normalized, deterministic=True)
        meta = dict(config=config, provenance=source, completed_rollout_updates=completed_updates,
                    state=_model_state(model, vec), probe_observation=raw_probe.tolist(),
                    probe_action=action.tolist(),
                    files={name: source_hash(path / name)["sha256"]
                           for name in ("model.zip", "normalize.pkl")},
                    resume_scope="optimizer/counters/normalization; no episode, rollout or RNG trajectory restoration")
        # Metadata written last also lets loading detect an interrupted file pair.
        _write_json(path / "checkpoint.json", meta)
        if destination.exists():
            destination.rename(previous)
        try:
            path.rename(destination)
        except Exception:
            if previous.exists():
                previous.rename(destination)
            raise
        if previous.exists():
            shutil.rmtree(previous)
    return meta


def _load_checkpoint(path, *, training, n_envs=None):
    path = Path(path).resolve()
    meta = json.loads((path / "checkpoint.json").read_text())
    for name, expected in meta["files"].items():
        if source_hash(path / name)["sha256"] != expected:
            raise ValueError(f"Checkpoint content mismatch: {name}")
    config = meta["config"]
    torch.set_num_threads(config["torch_threads"])
    fresh = _make_dummy_vec(config, n_envs=n_envs)
    try:
        vec = VecNormalize.load(path / "normalize.pkl", fresh)
    except Exception:
        fresh.close()
        raise
    vec.training = training
    vec.norm_reward = False
    try:
        # force_reset deliberately starts new episodes, retaining model counts.
        model = PPO.load(path / "model.zip", env=vec, device="cpu", force_reset=True)
        if _model_state(model, vec) != meta["state"]:
            raise ValueError("Policy/optimizer/statistics/counters did not restore exactly")
        action, _ = model.predict(vec.normalize_obs(np.array(meta["probe_observation"], np.float32)),
                                  deterministic=True)
        if not np.array_equal(action, np.array(meta["probe_action"], dtype=action.dtype)):
            raise ValueError("Deterministic checkpoint action changed after loading")
        model.set_logger(configure(format_strings=[]))
        return model, vec, meta
    except Exception:
        vec.close()
        raise


def evaluate(checkpoint, output):
    """Exactly one nominal episode; terminal scoring comes from the I1 env."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, vec, meta = _load_checkpoint(checkpoint, training=False, n_envs=1)
    try:
        before_stats = _state_hash(_normalization_state(vec))
        # Drive the same fresh wrapped plant directly for one episode. This
        # retains exact raw terminal observations; reversing VecNormalize's
        # clipped terminal sample would fabricate the exported terminal state.
        episode = vec.venv.envs[0]
        raw, _ = episode.reset(seed=meta["config"]["seed"])
        obs = vec.normalize_obs(raw)
        rows = [dict(time_s=0., observation=raw.copy(),
                     action_normalized=np.zeros(4), action_applied=False, reward=0.)]
        total_reward = 0.
        # Bound by the original task horizon, including a terminal short step.
        env = vec.venv.envs[0].unwrapped
        max_steps = int(np.ceil(env.horizon_s / env.policy_dt_s)) + 1
        for _ in range(max_steps):
            action, _ = model.predict(obs, deterministic=True)
            raw, reward, terminated, truncated, info = episode.step(action)
            obs = vec.normalize_obs(raw)
            total_reward += float(reward)
            rows.append(dict(time_s=info["time_s"], observation=raw.copy(),
                             action_normalized=action.copy(), action_applied=True, reward=float(reward)))
            if terminated or truncated:
                if truncated:
                    raise RuntimeError("Original task deadline was changed into truncation")
                result = info["result"]
                break
        else:
            raise RuntimeError("Policy evaluation did not terminate within the original task")
        if before_stats != _state_hash(_normalization_state(vec)):
            raise RuntimeError("Evaluation mutated normalization statistics")
        result.update(total_event_reward=total_reward, deterministic=True,
                      normalization_frozen=True, normalization_sha256=before_stats,
                      checkpoint_state=meta["state"], baseline_privileged=False,
                      stable_learning_reliability_generalization="NOT_VERIFIED",
                      provenance=provenance(env, algorithm="PPO_CPU", seed=meta["config"]["seed"]),
                      checkpoint_provenance=meta["provenance"])
        _write_json(output / "config.json", dict(training=meta["config"],
                    rl=env.rl_config, experiment=env.plant.cfg, physical=env.plant.physical,
                    vehicle=env.plant.vehicle_config, profile=env.plant.profile))
        np.savez_compressed(output / "trajectory.npz",
                            **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})
        _write_json(output / "summary.json", result)
        return result
    finally:
        vec.close()


class _WallLimit(Exception):
    pass


class _BudgetCallback(BaseCallback):
    def __init__(self, deadline):
        super().__init__()
        self.deadline = deadline
        self.expired = False

    def _on_step(self):
        _assert_finite(self.locals["new_obs"])
        _assert_finite(self.locals["rewards"])
        _assert_finite(self.locals["actions"])
        self.expired = time.monotonic() >= self.deadline
        return not self.expired


def _learn_rollout(model, vec, deadline):
    """Delegate all optimization to SB3; check one actual rollout/update."""
    if time.monotonic() >= deadline:
        raise _WallLimit("Learning wall budget exhausted before rollout")
    start_state = _model_state(model, vec)
    gradient_checks = 0
    def gradient_check(gradient):
        nonlocal gradient_checks
        if time.monotonic() >= deadline:
            raise _WallLimit("Learning wall budget exhausted during optimizer update")
        _assert_finite(gradient)
        gradient_checks += 1
        return gradient
    handles = [parameter.register_hook(gradient_check) for parameter in model.policy.parameters()]
    callback = _BudgetCallback(deadline)
    try:
        model.learn(total_timesteps=model.n_steps * vec.num_envs, callback=callback,
                    reset_num_timesteps=False, log_interval=1)
    finally:
        for handle in handles:
            handle.remove()
    if callback.expired:
        raise _WallLimit("Learning wall budget exhausted; incomplete rollout discarded")
    metrics = {key: float(value) for key, value in model.logger.name_to_value.items()
               if key.startswith("train/")}
    for key, value in metrics.items():
        # Constant returns legitimately have undefined explained variance.
        if key != "train/explained_variance" or not np.isnan(value):
            _assert_finite(value)
    _assert_finite(model.policy.state_dict())
    _assert_finite(model.policy.optimizer.state_dict())
    end = _model_state(model, vec)
    if (end["num_timesteps"] != start_state["num_timesteps"] + model.n_steps * vec.num_envs
            or end["sb3_epoch_updates"] != start_state["sb3_epoch_updates"] + model.n_epochs
            or end["policy_sha256"] == start_state["policy_sha256"] or not gradient_checks):
        raise RuntimeError("PPO did not complete the expected finite parameter update")
    return dict(state=end, gradient_checks=gradient_checks,
                parameters_changed=True,
                metrics={key: value if np.isfinite(value) else None for key, value in metrics.items()})


def train(config_path, output):
    """One bounded engineering run, including two evaluations and resume."""
    config = load_train_config(config_path)
    torch.set_num_threads(config["torch_threads"])
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    vec = make_vec_env(config)
    model = PPO("MlpPolicy", vec, seed=config["seed"], device="cpu", verbose=0, **config["ppo"])
    model.set_logger(configure(format_strings=[]))
    vec.reset()
    env = vec.venv.envs[0].unwrapped
    source = provenance(env, algorithm="PPO_CPU", seed=config["seed"],
                        extra_sources={"training_config": Path(config_path).resolve()})
    wrapper = vec.venv.envs[0].env
    action_mapping = dict(low_physical_ctbr=wrapper.low.tolist(), high_physical_ctbr=wrapper.high.tolist(),
                          hover_physical_ctbr=[env.plant.hover_force, 0., 0., 0.],
                          hover_normalized=wrapper.hover_action.tolist(),
                          zero_normalized_physical_ctbr=wrapper.action(np.zeros(4)).tolist())
    defaults = {name: parameter.default for name, parameter in inspect.signature(PPO.__init__).parameters.items()
                if parameter.default is not inspect.Parameter.empty
                and name not in ("_init_setup_model", "policy_kwargs")}
    resolved = dict(defaults, **config["ppo"], policy="MlpPolicy", device="cpu", seed=config["seed"],
                    observation_normalization=True, reward_normalization=False,
                    observation_clip=vec.clip_obs, normalization_epsilon=vec.epsilon,
                    policy_class=type(model.policy).__name__,
                    activation_function=model.policy.activation_fn.__name__,
                    optimizer_class=type(model.policy.optimizer).__name__,
                    optimizer_defaults=model.policy.optimizer.defaults,
                    learn_schedule="one complete rollout per learn call; fixed constant LR/clip defaults")
    _write_json(output / "config.json", dict(training=config, resolved_ppo=resolved,
                action_mapping=action_mapping, rl=env.rl_config, experiment=env.plant.cfg,
                physical=env.plant.physical, vehicle=env.plant.vehicle_config, profile=env.plant.profile))
    _write_json(output / "provenance.json", source)
    checkpoints = output / "checkpoints"
    completed = 0
    resume = dict(status="NOT_COMPLETED")
    started = None
    before = after = None
    stop_reason = "TRANSITION_BUDGET"
    error = None
    error_traceback = None
    engineering_passed = False
    try:
        _save_checkpoint(checkpoints / "initial", model, vec, config, source, 0)
        _save_checkpoint(checkpoints / "latest", model, vec, config, source, 0)
        before = evaluate(checkpoints / "initial", output / "evaluation_before")
        # The initial evaluation is outside the learning budget. One shared
        # clock covers all learning, saves, reloads and stage transitions.
        started = time.monotonic()
        deadline = started + config["budget"]["learning_wall_s"]
        rollout_size = config["n_envs"] * config["ppo"]["n_steps"]
        for update in range(config["budget"]["transitions"] // rollout_size):
            evidence = _learn_rollout(model, vec, deadline)
            completed += 1
            evidence.update(completed_rollout_updates=completed, learning_wall_s=time.monotonic() - started)
            with (output / "updates.jsonl").open("a") as stream:
                stream.write(json.dumps(evidence, allow_nan=False) + "\n")
            _save_checkpoint(checkpoints / "latest", model, vec, config, source, completed)
            if completed == 2:
                saved = _save_checkpoint(checkpoints / "pre_resume", model, vec, config, source, completed)
                if saved["state"]["num_timesteps"] != 4096:
                    raise RuntimeError("Unexpected pre-resume timestep count")
                vec.close()
                model, vec, restored = _load_checkpoint(checkpoints / "pre_resume", training=True)
                resume.update(restored_state=restored["state"], exact_state_restore=True,
                              deterministic_action_match=True, fresh_independent_environments=True)
            if completed == 3:
                if model.num_timesteps != 6144:
                    raise RuntimeError("Resume did not extend counters to 6144")
                resume.update(status="PASS", resumed_state=evidence["state"],
                              parameters_changed=evidence["parameters_changed"], resumed_transitions=2048)
                _write_json(output / "resume.json", resume)
            # At most one concise output per eight completed rollouts.
            if completed in (2, 3) or completed % 8 == 0:
                print(json.dumps(dict(completed_updates=completed, transitions=model.num_timesteps,
                                      learning_wall_s=round(time.monotonic() - started, 3))), flush=True)
        engineering_passed = resume["status"] == "PASS"
    except _WallLimit as exc:
        stop_reason = "LEARNING_WALL_BUDGET"
        error = str(exc)
        engineering_passed = resume["status"] == "PASS"
    except Exception as exc:
        stop_reason = "ENGINEERING_ERROR"
        error = f"{type(exc).__name__}: {exc}"
        error_traceback = traceback.format_exc()
    finally:
        collected = int(model.num_timesteps)
        vec.close()
        # On interruption retain the last wholly validated update, never a
        # partially applied optimizer state. Count discarded samples separately.
        final_state = None
        if (checkpoints / "latest" / "checkpoint.json").is_file():
            shutil.copytree(checkpoints / "latest", checkpoints / "final")
            final_state = json.loads((checkpoints / "final" / "checkpoint.json").read_text())["state"]
        learning_wall = 0. if started is None else time.monotonic() - started
        if (checkpoints / "final").is_dir():
            if stop_reason != "ENGINEERING_ERROR":
                try:
                    after = evaluate(checkpoints / "final", output / "evaluation_after")
                except Exception as exc:
                    engineering_passed = False
                    stop_reason = "ENGINEERING_ERROR"
                    error = f"{type(exc).__name__}: {exc}"
                    error_traceback = traceback.format_exc()
        summary = dict(passed=engineering_passed, engineering_loop="PASS" if engineering_passed else "INCOMPLETE",
                       stop_reason=stop_reason, error=error, error_traceback=error_traceback,
                       training_transitions=collected,
                       final_checkpoint_state=final_state,
                       discarded_transitions=collected - (0 if final_state is None else final_state["num_timesteps"]),
                       completed_rollout_updates=completed, learning_wall_s=learning_wall,
                       budget=config["budget"], resume=resume, provenance=source,
                       before=None if before is None else {key: before[key] for key in
                           ("passed", "total_event_reward", "episode_elapsed_s", "failure_modes", "events")},
                       after=None if after is None else {key: after[key] for key in
                           ("passed", "total_event_reward", "episode_elapsed_s", "failure_modes", "events")},
                       comparison="whole training pipeline; checkpoint-specific normalization, no policy-only attribution",
                       nominal_contact_recovery="NOT_RUN" if after is None else "PASS" if after["passed"] else "FAIL",
                       stable_learning_reliability_generalization="NOT_VERIFIED")
        _write_json(output / "summary.json", summary)
        print(json.dumps({key: summary[key] for key in
                          ("engineering_loop", "stop_reason", "training_transitions", "learning_wall_s",
                           "nominal_contact_recovery")}), flush=True)
    return 1 if stop_reason == "ENGINEERING_ERROR" else 0 if engineering_passed else 2
