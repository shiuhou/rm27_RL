"""One bounded CPU PPO trainer for Contact and agile terminal arrival.

Task physics, scoring and rewards stay in their environments. I3 retains its
resume acceptance protocol; I4 uses independent fixed-set evaluation.
"""
from __future__ import annotations

import hashlib
import inspect
import itertools
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
from .paths import ROOT

DEFAULT_TRAIN_CONFIG = ROOT / "configs/contact_rl/ppo_minimal.json"
CONTACT_TASK = "contact_recovery_v1"
CONTACT_SCHEMA = "contact_world_v1"
ARRIVAL_TASK = "agile_terminal_arrival_v1"
ARRIVAL_SCHEMA = "arrival_world_v1"
# Normalized-action maps: the original affine map onto [low, high], and an
# Arrival-only map whose zero is synthetic hover (collective m*g, zero rates).
BOUNDS_AFFINE = "bounds_affine_v1"
HOVER_CENTERED = "hover_centered_v1"
# Largest approved single Arrival run on the local CPU (user 2026-10-19): one seed,
# at most 1024 complete rollout/updates of 2048 transitions or one hour of learning.
ARRIVAL_MAX_TRANSITIONS = 2_097_152
ARRIVAL_MAX_LEARNING_WALL_S = 3600
FLATNESS_REFERENCE = "flatness_feedforward_v1"
# Reference-flight snapshot spacing in policy steps. Performance only: a restore
# continues with the same flight loop, so the reset state is identical for any spacing.
SNAPSHOT_INTERVAL_STEPS = 10
# Curriculum advance statistics: every informed episode in [min_lead, frontier]
# (third run and batch A/B), or only those whose lead lies in the newest band
# [frontier - lead_step, frontier].
ALL_INFORMED = "all_informed_v1"
FRONTIER_BAND = "frontier_band_v1"


def _task_contract(config):
    task = config.get("task_id", CONTACT_TASK)
    schema = config.get("observation_schema", CONTACT_SCHEMA)
    pairs = {CONTACT_TASK: CONTACT_SCHEMA, ARRIVAL_TASK: ARRIVAL_SCHEMA}
    if task not in pairs or schema != pairs[task]:
        raise ValueError("Unknown or mismatched task/observation schema")
    return {"task_id": task, "observation_schema": schema}


def load_train_config(path=DEFAULT_TRAIN_CONFIG):
    config = json.loads(Path(path).read_text())
    if config.get("schema_version") != 1:
        raise ValueError("PPO config requires schema_version 1")
    contract = _task_contract(config)
    arrival = contract["task_id"] == ARRIVAL_TASK
    protocol = config.get("protocol", "i3_engineering")
    if protocol != ("i4_arrival" if arrival else "i3_engineering"):
        raise ValueError("Training protocol does not match its task")
    for name, value in (("seed", config["seed"]), ("n_envs", config["n_envs"]),
                        ("n_steps", config["ppo"]["n_steps"]), ("batch_size", config["ppo"]["batch_size"]),
                        ("n_epochs", config["ppo"]["n_epochs"]), ("torch_threads", config["torch_threads"])):
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"{name} must be an integer")
    if (not 0 <= config["seed"] < 2**32 or not 1 <= config["n_envs"] <= 4
            or not 2 <= config["ppo"]["n_steps"] <= 512 or config["torch_threads"] != 1
            or not 1 <= config["ppo"]["n_epochs"] <= 5):
        raise ValueError("Training resources exceed the CPU experiment envelope")
    rollout = config["n_envs"] * config["ppo"]["n_steps"]
    if not 2 <= config["ppo"]["batch_size"] <= rollout or rollout % config["ppo"]["batch_size"]:
        raise ValueError("Batch size must divide the complete rollout")
    budget = config["budget"]
    lower = rollout if arrival else 6144
    upper = ARRIVAL_MAX_TRANSITIONS if arrival else 65536
    wall = ARRIVAL_MAX_LEARNING_WALL_S if arrival else 1800
    if ((not arrival and (config["seed"] != 0 or config["n_envs"] != 4 or rollout != 2048
                          or config["ppo"]["n_steps"] != 512))
            or not isinstance(budget["transitions"], int) or isinstance(budget["transitions"], bool)
            or not lower <= budget["transitions"] <= upper
            or budget["transitions"] % rollout
            or not np.isfinite(budget["learning_wall_s"])
            or not 0 < budget["learning_wall_s"] <= wall):
        raise ValueError("Task requires its approved bounded rollout budget and protocol")
    for name in ("learning_rate", "gamma", "gae_lambda"):
        if not np.isfinite(config["ppo"][name]) or config["ppo"][name] <= 0:
            raise ValueError(f"Invalid PPO {name}")
    if config["ppo"]["gamma"] > 1 or config["ppo"]["gae_lambda"] > 1:
        raise ValueError("PPO discount factors must not exceed one")
    parameterization = config.get("action_parameterization", BOUNDS_AFFINE)
    if parameterization not in (BOUNDS_AFFINE, HOVER_CENTERED) or (
            parameterization == HOVER_CENTERED and not arrival):
        raise ValueError("Unknown action parameterization, or one not declared for this task")
    log_std_init = config["ppo"].get("policy_kwargs", {}).get("log_std_init", 0.)
    if (isinstance(log_std_init, bool) or not isinstance(log_std_init, (int, float))
            or not -3. <= log_std_init <= 0. or (not arrival and log_std_init != 0.)):
        raise ValueError("Initial policy log standard deviation outside the declared envelope")
    randomization = config.get("dynamics_randomization")
    if randomization is not None:
        ranges = [randomization.get(key) for key in ("mass_scale", "thrust_scale")]
        if (not arrival or set(randomization) != {"mass_scale", "thrust_scale"}
                or any(not isinstance(r, list) or len(r) != 2
                       or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in r)
                       or not .5 <= r[0] <= r[1] <= 1.5 for r in ranges)):
            raise ValueError("Dynamics randomization must declare Arrival mass/thrust scale ranges in [0.5, 1.5]")
    informed = config.get("informed_reset")
    if informed is not None:
        keys = {"reference", "reference_duration_s", "probability", "initial_lead_s", "min_lead_s",
                "lead_step_s", "advance_success_rate", "advance_window_episodes"}
        numbers = [informed.get(key) for key in sorted(keys - {"reference", "advance_window_episodes"})]
        window = informed.get("advance_window_episodes")
        reuse = informed.get("reference_reuse_episodes", 1)
        if (not arrival or not keys <= set(informed) <= keys | {"reference_reuse_episodes", "advance_statistic"}
                or informed.get("advance_statistic", ALL_INFORMED) not in (ALL_INFORMED, FRONTIER_BAND)
                or isinstance(reuse, bool) or not isinstance(reuse, int) or not 1 <= reuse <= 64
                or informed["reference"] != FLATNESS_REFERENCE
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) for v in numbers)
                or isinstance(window, bool) or not isinstance(window, int) or window < 1
                or not 0 < informed["probability"] <= 1 or not 0 < informed["advance_success_rate"] <= 1
                or not 0 < informed["min_lead_s"] <= informed["initial_lead_s"] <= informed["reference_duration_s"]
                or informed["lead_step_s"] <= 0):
            raise ValueError("Informed reset must be a declared Arrival training curriculum")
    if not (ROOT / config["env_config"]).is_file():
        raise ValueError("PPO env_config must locate existing checkout assets")
    return config


class NormalizedCTBR(gym.ActionWrapper):
    """Affine [-1,1] action to physical CTBR, then the existing env input clip.

    ``bounds_affine_v1`` maps [-1,1] onto [low, high]. ``hover_centered_v1``
    maps 0 to synthetic hover with per-axis half-range max(high-hover,
    hover-low); a request below the physical collective lower bound is left
    to the environment's reported input clip, not hidden here.
    """

    def __init__(self, env, parameterization=BOUNDS_AFFINE):
        super().__init__(env)
        # Use the environment's double-precision physical limits, not rounded
        # Box metadata; preserve the proxy chain.
        self.low = env.unwrapped._action_low.copy()
        self.high = env.unwrapped._action_high.copy()
        self.parameterization = parameterization
        self.action_space = gym.spaces.Box(-1., 1., shape=(4,), dtype=np.float32)
        hover = np.r_[env.unwrapped.plant.hover_force, np.zeros(3)]
        if parameterization == BOUNDS_AFFINE:
            self.center = (self.high + self.low) / 2
            self.half_range = (self.high - self.low) / 2
            self.hover_action = 2 * (hover - self.low) / (self.high - self.low) - 1
        elif parameterization == HOVER_CENTERED:
            self.center = hover
            self.half_range = np.maximum(self.high - hover, hover - self.low)
            self.hover_action = np.zeros(4)
        else:
            raise ValueError("Unknown action parameterization")

    def action(self, action):
        requested = np.asarray(action, dtype=float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("Normalized CTBR requires four finite values")
        clipped = np.clip(requested, -1., 1.)
        if self.parameterization == BOUNDS_AFFINE:
            # Keep the original expression so earlier runs replay bit-for-bit.
            return self.low + (clipped + 1) * (self.high - self.low) / 2
        return self.center + clipped * self.half_range


class InformedResetCurriculum:
    """Reverse-curriculum frontier shared by one training vector's environments.

    Informed episodes start at reference time ``duration - lead`` with lead drawn
    from [min_lead, frontier]. The frontier grows by one step after a full window
    of counted informed episodes reaches the declared success rate; it is checked
    once per complete rollout so the schedule follows completed optimization
    updates. ``frontier_band_v1`` counts only episodes whose lead lies in the
    newest band, so short, already mastered leads cannot carry the frontier.
    """

    def __init__(self, spec):
        self.spec = dict(spec)
        self.lead_s = float(spec["initial_lead_s"])
        self.window = []
        self.prefix_failures = 0
        self.reference_flights = 0

    def record(self, success, lead_s):
        if (self.spec.get("advance_statistic", ALL_INFORMED) == FRONTIER_BAND
                and lead_s < self.lead_s - self.spec["lead_step_s"] - 1e-12):
            return
        self.window = (self.window + [bool(success)])[-self.spec["advance_window_episodes"]:]

    def update(self):
        full = len(self.window) == self.spec["advance_window_episodes"]
        rate = float(np.mean(self.window)) if self.window else None
        advanced = (full and rate >= self.spec["advance_success_rate"]
                    and self.lead_s < self.spec["reference_duration_s"])
        if advanced:
            # Round so repeated steps land exactly on the declared grid (0.9 + 0.1 must reach 1.0).
            self.lead_s = min(round(self.lead_s + self.spec["lead_step_s"], 9), self.spec["reference_duration_s"])
            self.window = []
        return dict(lead_s=self.lead_s, window_episodes=len(self.window) if not advanced else 0,
                    window_success_rate=rate, advanced=advanced, prefix_failures=self.prefix_failures,
                    reference_flights=self.reference_flights)


class InformedReset(gym.Wrapper):
    """Training-only Arrival reset from a dynamically consistent reference prefix.

    The prefix is flown through the real ``step`` (motor states, previous action,
    evaluator history and clock stay consistent) but its transitions are not
    returned to the learner. Explicit-option resets, as used by evaluation, are
    untouched. The reference never supplies actions, observations or rewards.

    With ``reference_reuse_episodes`` K > 1, one reference flight (to the longest
    prefix) serves K informed episodes of this environment: the latest episode
    snapshot before the requested start is restored and the same flight loop
    finishes the prefix, giving the state a fresh prefix from those coordinates
    reaches. Coordinates are drawn as before but repeat within the K episodes.

    Under dynamics randomization the prefix is flown on the nominal plant (it only
    generates a start state on the feasible reference) and the episode's drawn
    scales are applied from the scored start onward; an open-loop prefix flown on
    a mismatched plant would drift away from the reference.
    """

    def __init__(self, env, curriculum):
        super().__init__(env)
        self.curriculum = curriculum
        self.informed = False
        self._flight = None

    def reset(self, *, seed=None, options=None):
        observation, info = self.env.reset(seed=seed, options=options)
        base = self.env.unwrapped
        spec = self.curriculum.spec
        self.informed = False
        if options is not None or base.np_random.random() >= spec["probability"]:
            return observation, info
        lead = base.np_random.uniform(spec["min_lead_s"], self.curriculum.lead_s)
        self._lead_s = lead
        prefix = spec["reference_duration_s"] - lead
        if prefix < base.policy_dt_s:
            return observation, info
        from .baselines import FlatnessReference
        episode_scales = dict(base.dynamics_scales)
        base.set_dynamics_scales()
        reuse = spec.get("reference_reuse_episodes", 1)
        if reuse == 1:
            reference = FlatnessReference(base, spec["reference_duration_s"])
            self.curriculum.reference_flights += 1
        else:
            if self._flight is None or self._flight["uses"] >= reuse:
                self._flight = self._fly_reference(base, spec)
            self._flight["uses"] += 1
            reference = self._flight["reference"]
            start = [snap for elapsed, snap in self._flight["snapshots"] if elapsed < prefix - 1e-9][-1]
            base.restore_episode(start)
        while base.plant.data.time - base.episode_start < prefix - 1e-9:
            observation, _, terminated, _, info = base.step(reference.action(base))
            if terminated:
                self.curriculum.prefix_failures += 1
                return self.env.reset(options={"initial_coordinates": base.initial_coordinates})
        base.set_dynamics_scales(**episode_scales)
        observation = base._observation()
        base.start_scored_segment()
        self.informed = True
        return observation, dict(info, informed_prefix_s=base.informed_prefix_s)

    def _fly_reference(self, base, spec):
        """Fly one reference to the longest prefix, keeping spaced episode snapshots."""
        from .baselines import FlatnessReference
        reference = FlatnessReference(base, spec["reference_duration_s"])
        self.curriculum.reference_flights += 1
        longest = spec["reference_duration_s"] - spec["min_lead_s"]
        snapshots, steps = [], 0
        while base.plant.data.time - base.episode_start < longest - 1e-9:
            if steps % SNAPSHOT_INTERVAL_STEPS == 0:
                snapshots.append((base.plant.data.time - base.episode_start, base.episode_snapshot()))
            if base.step(reference.action(base))[2]:
                break
            steps += 1
        return dict(reference=reference, snapshots=snapshots, uses=0)

    def step(self, action):
        transition = self.env.step(action)
        if transition[2] and self.informed:
            self.curriculum.record(transition[-1]["result"]["passed"], self._lead_s)
        return transition


class DynamicsRandomization(gym.Wrapper):
    """Training-only per-episode plant mass and thrust-gain scales, drawn uniformly.

    The first reset seeds the environment and draws the initial coordinates; the
    scales come from the same RNG and the episode is then reset with them applied.
    Evaluation vectors never carry this wrapper, so they use the nominal plant.
    """

    def __init__(self, env, spec):
        super().__init__(env)
        self.ranges = {key: tuple(value) for key, value in spec.items()}

    def reset(self, *, seed=None, options=None):
        self.env.reset(seed=seed, options=options)
        base = self.env.unwrapped
        base.set_dynamics_scales(mass=base.np_random.uniform(*self.ranges["mass_scale"]),
                                 thrust=base.np_random.uniform(*self.ranges["thrust_scale"]))
        return self.env.reset(options={"initial_coordinates": base.initial_coordinates})


def _make_dummy_vec(config, n_envs=None, curriculum=None, randomization=None):
    path = ROOT / config["env_config"]
    count = config["n_envs"] if n_envs is None else n_envs
    # Every factory creates its own model/data/evaluator/proxy/action memory.
    if _task_contract(config)["task_id"] == ARRIVAL_TASK:
        from .arrival_env import AgileTerminalArrivalEnv
        env_class = AgileTerminalArrivalEnv
    else:
        env_class = ContactRecoveryEnv
    parameterization = config.get("action_parameterization", BOUNDS_AFFINE)
    def build():
        env = env_class(path)
        if randomization is not None:
            env = DynamicsRandomization(env, randomization)
        if curriculum is not None:
            env = InformedReset(env, curriculum)
        return Monitor(NormalizedCTBR(env, parameterization))
    vec = DummyVecEnv([build for _ in range(count)])
    vec.seed(config["seed"])
    return vec


def make_vec_env(config, n_envs=None, training=True):
    """Training vectors carry the declared curriculum and dynamics randomization; evaluation never does."""
    curriculum = (InformedResetCurriculum(config["informed_reset"])
                  if training and config.get("informed_reset") else None)
    randomization = config.get("dynamics_randomization") if training else None
    vec = VecNormalize(_make_dummy_vec(config, n_envs, curriculum, randomization), training=training,
                       norm_obs=True, norm_reward=False)
    vec.informed_curriculum = curriculum
    return vec


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
        meta = dict(config=config, task=_task_contract(config), provenance=source,
                    completed_rollout_updates=completed_updates,
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


def _load_checkpoint(path, *, training, n_envs=None, expected_task=None):
    path = Path(path).resolve()
    meta = json.loads((path / "checkpoint.json").read_text())
    for name, expected in meta["files"].items():
        if source_hash(path / name)["sha256"] != expected:
            raise ValueError(f"Checkpoint content mismatch: {name}")
    config = meta["config"]
    contract = _task_contract(config)
    recorded = meta.get("task")
    if recorded is None:
        if contract["task_id"] != CONTACT_TASK:
            raise ValueError("Legacy checkpoint without task metadata is Contact-only")
    elif recorded != contract:
        raise ValueError("Checkpoint task/schema differs from its configuration")
    if expected_task is not None and expected_task != contract["task_id"]:
        raise ValueError("Checkpoint cannot be loaded for a different task")
    torch.set_num_threads(config["torch_threads"])
    fresh = _make_dummy_vec(config, n_envs=n_envs)
    try:
        # Check logical assets by content, not the old checkout's absolute path.
        sources = fresh.envs[0].unwrapped.sources
        actual_env = fresh.envs[0].unwrapped
        if (getattr(actual_env, "task_id", CONTACT_TASK) != contract["task_id"]
                or getattr(actual_env, "observation_schema", CONTACT_SCHEMA) != contract["observation_schema"]):
            raise ValueError("Environment task/schema does not match the checkpoint")
        expected = meta["provenance"]["source_hashes"]
        for name, source in sources.items():
            if source_hash(source)["sha256"] != expected[name]["sha256"]:
                raise ValueError(f"Checkpoint environment config changed: {name}")
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


def _evaluate_contact(checkpoint, output):
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


def _arrival_initials(env):
    """The declared center and eight corners, without sampling extra cases."""
    initial = env.rl_config["initial_state"]
    ranges = [initial[key] for key in ("L_m", "eta1_m", "eta2_m")]
    values = [[sum(bounds) / 2 for bounds in ranges]]
    values.extend(list(point) for point in itertools.product(*ranges))
    if len({tuple(value) for value in values}) != 9:
        raise ValueError("Arrival evaluation requires nine distinct declared initial states")
    return values


def _arrival_episode(episode, output, coordinates, *, model=None, vec=None, controller_class=None, seed=0):
    """Use the real env step and retain one bounded deterministic trajectory."""
    output.mkdir(parents=True, exist_ok=False)
    raw, _ = episode.reset(seed=seed, options={"initial_coordinates": coordinates})
    env = episode.unwrapped
    controller = controller_class(env) if controller_class is not None else None
    rows = []
    totals = {"progress": 0., "cost": 0., "event": 0.}
    total_reward = 0.
    def record(raw, reward, info, action, applied):
        data = env.plant.data
        rows.append(dict(time_s=float(data.time), observation=raw.copy(),
                         qpos=data.qpos.copy(), qvel=data.qvel.copy(),
                         motor_state_N=data.act.copy(), motor_commands_N=data.ctrl.copy(),
                         previous_action_ctbr=env.previous_action_ctbr.copy(),
                         action_normalized=np.asarray(action).copy(), action_applied=applied,
                         reward=float(reward), proxy_saturated=bool(info.get("proxy_saturated", False)),
                         action_clipped=bool(info.get("action_clipped", False)),
                         physics_substeps=int(info.get("physics_substeps", 0))))
    record(raw, 0., {}, np.zeros(4), False)
    max_steps = int(np.ceil(env.horizon_s / env.policy_dt_s)) + 1
    for _ in range(max_steps):
        if controller is None:
            action, _ = model.predict(vec.normalize_obs(raw), deterministic=True)
            raw, reward, terminated, truncated, info = episode.step(action)
        else:
            raw, reward, terminated, truncated, info = controller.step(env)
            action = (2 * (env.previous_action_ctbr - env._action_low)
                      / (env._action_high - env._action_low) - 1)
        total_reward += float(reward)
        for key, value in info["reward_components"].items():
            totals[key] = totals.get(key, 0.) + float(value)
        record(raw, reward, info, action, True)
        if terminated or truncated:
            if truncated:
                raise RuntimeError("Arrival task deadline must remain a task termination")
            result = dict(info["result"])
            break
    else:
        raise RuntimeError("Arrival evaluation exceeded its original task deadline")
    result.update(initial_coordinates=list(coordinates), total_reward=total_reward,
                  total_event_reward=totals["event"], reward_component_totals=totals,
                  deterministic=True, baseline_privileged=False,
                  traditional_reference=controller is not None,
                  contact_recovery_task_evaluation="NOT_EVALUATED",
                  stable_learning_reliability_generalization="NOT_VERIFIED")
    np.savez_compressed(output / "trajectory.npz",
                        **{key: np.asarray([row[key] for row in rows]) for key in rows[0]})
    _write_json(output / "summary.json", result)
    return result


def _arrival_summary(episodes):
    failures = {}
    for episode in episodes:
        for reason in episode["failure_modes"]:
            failures[reason] = failures.get(reason, 0) + 1
    successes = sum(bool(episode["passed"]) for episode in episodes)
    return dict(task_id=ARRIVAL_TASK, observation_schema=ARRIVAL_SCHEMA,
                passed=successes == len(episodes), success_count=successes,
                episode_count=len(episodes), success_rate=successes / len(episodes),
                episodes=episodes, failure_counts=failures,
                mean_total_reward=float(np.mean([e["total_reward"] for e in episodes])),
                mean_event_reward=float(np.mean([e["total_event_reward"] for e in episodes])),
                terminal_arrival="PASS" if successes == len(episodes) else "FAIL",
                candidate_agility="PASS" if successes == len(episodes)
                    and all(e["episode_elapsed_s"] <= 1.5 for e in episodes) else "FAIL",
                local_fixed_set_only=True, contact_recovery_success="NOT_EVALUATED",
                stable_learning_reliability_generalization="NOT_VERIFIED")


def evaluate(checkpoint, output, *, expected_task=None):
    """Contact keeps its single nominal episode; Arrival uses its nine cases."""
    metadata = json.loads((Path(checkpoint) / "checkpoint.json").read_text())
    contract = _task_contract(metadata["config"])
    if expected_task is not None and expected_task != contract["task_id"]:
        raise ValueError("Checkpoint cannot be evaluated for a different task")
    if contract["task_id"] == CONTACT_TASK:
        return _evaluate_contact(checkpoint, output)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, vec, meta = _load_checkpoint(checkpoint, training=False, n_envs=1,
                                         expected_task=ARRIVAL_TASK)
    try:
        before_stats = _state_hash(_normalization_state(vec))
        env = vec.venv.envs[0].unwrapped
        episodes = [_arrival_episode(vec.venv.envs[0], output / f"episode_{index:02d}", coordinates,
                                    model=model, vec=vec, seed=meta["config"]["seed"])
                    for index, coordinates in enumerate(_arrival_initials(env))]
        if before_stats != _state_hash(_normalization_state(vec)):
            raise RuntimeError("Evaluation mutated normalization statistics")
        result = _arrival_summary(episodes)
        result.update(normalization_frozen=True, normalization_sha256=before_stats,
                      checkpoint_state=meta["state"], checkpoint_provenance=meta["provenance"],
                      provenance=provenance(env, algorithm="PPO_CPU", seed=meta["config"]["seed"]))
        _write_json(output / "config.json", dict(training=meta["config"], rl=env.rl_config,
                    experiment=env.plant.cfg, physical=env.plant.physical,
                    vehicle=env.plant.vehicle_config, profile=env.plant.profile))
        _write_json(output / "summary.json", result)
        return result
    finally:
        vec.close()


def evaluate_arrival_baseline(config_path, output):
    """Nine same-task traditional comparisons; no teacher actions enter PPO."""
    from .arrival_env import AgileTerminalArrivalEnv
    from .baselines import ArrivalCTBRBaseline
    env = AgileTerminalArrivalEnv(config_path)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    try:
        result = _arrival_summary([
            _arrival_episode(env, output / f"episode_{index:02d}", coordinates,
                             controller_class=ArrivalCTBRBaseline)
            for index, coordinates in enumerate(_arrival_initials(env))])
        result["provenance"] = provenance(env, algorithm="TRADITIONAL_ARRIVAL_SHARED_CTBR")
        result["baseline_privileged"] = False
        result["traditional_reference"] = True
        _write_json(output / "config.json", dict(rl=env.rl_config, experiment=env.plant.cfg,
                    physical=env.plant.physical, vehicle=env.plant.vehicle_config, profile=env.plant.profile))
        _write_json(output / "summary.json", result)
        return result
    finally:
        env.close()


class _WallLimit(Exception):
    pass


class _BudgetCallback(BaseCallback):
    def __init__(self, deadline):
        super().__init__()
        self.deadline = deadline
        self.expired = False
        self.episodes = []
        self.reward_components = {}
        self.action_clipped_steps = 0
        self.proxy_saturated_steps = 0
        self.normalized_action_clipped_steps = 0
        self.normalized_action_clipped_steps_per_axis = np.zeros(4, dtype=int)
        self.policy_mean = None
        self.curriculum = None

    def _on_step(self):
        _assert_finite(self.locals["new_obs"])
        _assert_finite(self.locals["rewards"])
        _assert_finite(self.locals["actions"])
        outside = np.abs(self.locals["actions"]) > 1.
        self.normalized_action_clipped_steps += int(np.count_nonzero(np.any(outside, axis=-1)))
        self.normalized_action_clipped_steps_per_axis += outside.sum(axis=0)
        for info in self.locals["infos"]:
            self.action_clipped_steps += int(info.get("action_clipped", False))
            self.proxy_saturated_steps += int(info.get("proxy_saturated", False))
            for name, value in info.get("reward_components", {}).items():
                self.reward_components[name] = self.reward_components.get(name, 0.) + float(value)
            if "episode" in info:
                result = info.get("result", {})
                self.episodes.append(dict(passed=bool(result.get("passed", False)),
                    failure_modes=result.get("failure_modes", []),
                    informed=result.get("informed_prefix_s", 0.) > 0,
                    episode_elapsed_s=result.get("episode_elapsed_s"),
                    reward=float(info["episode"]["r"]), steps=int(info["episode"]["l"])))
        self.expired = time.monotonic() >= self.deadline
        return not self.expired

    def _on_rollout_end(self):
        # Behavior-policy Gaussian mean on the stored (normalized) rollout
        # observations, before this rollout's optimization changes it.
        observations = self.model.rollout_buffer.observations.reshape(-1, self.model.observation_space.shape[0])
        with torch.no_grad():
            mean = self.model.policy.get_distribution(
                torch.as_tensor(observations, device=self.model.device)).distribution.mean.cpu().numpy()
        _assert_finite(mean)
        self.policy_mean = dict(mean=mean.mean(axis=0).tolist(),
                                outside_bounds_fraction=(np.abs(mean) > 1.).mean(axis=0).tolist())
        curriculum = getattr(self.training_env, "informed_curriculum", None)
        self.curriculum = None if curriculum is None else curriculum.update()


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
    failure_counts = {}
    for episode in callback.episodes:
        for reason in episode["failure_modes"]:
            failure_counts[reason] = failure_counts.get(reason, 0) + 1
    return dict(state=end, gradient_checks=gradient_checks,
                parameters_changed=True,
                episodes=dict(completed=len(callback.episodes),
                              successful=sum(e["passed"] for e in callback.episodes),
                              failure_counts=failure_counts,
                              mean_return=None if not callback.episodes else
                                  float(np.mean([e["reward"] for e in callback.episodes]))),
                reward_components=callback.reward_components,
                action_clipped_steps=callback.action_clipped_steps,
                proxy_saturated_steps=callback.proxy_saturated_steps,
                normalized_action_clipped_steps=callback.normalized_action_clipped_steps,
                normalized_action_clipped_steps_per_axis=callback.normalized_action_clipped_steps_per_axis.tolist(),
                policy_mean_on_rollout=callback.policy_mean,
                informed_reset=None if callback.curriculum is None else dict(
                    callback.curriculum,
                    informed_completed=sum(e["informed"] for e in callback.episodes),
                    informed_successful=sum(e["informed"] and e["passed"] for e in callback.episodes),
                    full_start_completed=sum(not e["informed"] for e in callback.episodes),
                    full_start_successful=sum(not e["informed"] and e["passed"] for e in callback.episodes)),
                metrics={key: value if np.isfinite(value) else None for key, value in metrics.items()})


def train(config_path, output):
    """One approved budget; task-specific evaluation and optional I3 resume check."""
    config = load_train_config(config_path)
    arrival = _task_contract(config)["task_id"] == ARRIVAL_TASK
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
                          zero_normalized_physical_ctbr=wrapper.action(np.zeros(4)).tolist(),
                          parameterization=wrapper.parameterization,
                          half_range_physical_ctbr=wrapper.half_range.tolist())
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
    resume = dict(status="NOT_REQUIRED" if arrival else "NOT_COMPLETED")
    started = None
    before = after = traditional = None
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
            curriculum_state = evidence.get("informed_reset")
            if curriculum_state and curriculum_state["advanced"]:
                # Policy that has just passed the advance gate, for later offline evaluation.
                _save_checkpoint(checkpoints / f"frontier_{curriculum_state['lead_s']:.2f}s",
                                 model, vec, config, source, completed)
            if not arrival and completed == 2:
                saved = _save_checkpoint(checkpoints / "pre_resume", model, vec, config, source, completed)
                if saved["state"]["num_timesteps"] != 4096:
                    raise RuntimeError("Unexpected pre-resume timestep count")
                vec.close()
                model, vec, restored = _load_checkpoint(checkpoints / "pre_resume", training=True)
                resume.update(restored_state=restored["state"], exact_state_restore=True,
                              deterministic_action_match=True, fresh_independent_environments=True)
            if not arrival and completed == 3:
                if model.num_timesteps != 6144:
                    raise RuntimeError("Resume did not extend counters to 6144")
                resume.update(status="PASS", resumed_state=evidence["state"],
                              parameters_changed=evidence["parameters_changed"], resumed_transitions=2048)
                _write_json(output / "resume.json", resume)
            # At most one concise output per eight completed rollouts.
            if completed in (2, 3) or completed % 8 == 0:
                print(json.dumps(dict(completed_updates=completed, transitions=model.num_timesteps,
                                      learning_wall_s=round(time.monotonic() - started, 3))), flush=True)
        engineering_passed = completed > 0 if arrival else resume["status"] == "PASS"
    except _WallLimit as exc:
        stop_reason = "LEARNING_WALL_BUDGET"
        error = str(exc)
        engineering_passed = completed > 0 if arrival else resume["status"] == "PASS"
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
                    if arrival:
                        traditional = evaluate_arrival_baseline(ROOT / config["env_config"],
                                                               output / "evaluation_baseline")
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
                       task=_task_contract(config),
                       before=_evaluation_brief(before, arrival),
                       after=_evaluation_brief(after, arrival),
                       traditional=_evaluation_brief(traditional, arrival),
                       comparison="whole training pipeline; checkpoint-specific normalization, no policy-only attribution",
                       nominal_contact_recovery="NOT_EVALUATED" if arrival else
                           "NOT_RUN" if after is None else "PASS" if after["passed"] else "FAIL",
                       terminal_arrival="NOT_EVALUATED" if not arrival else
                           "NOT_RUN" if after is None else after["terminal_arrival"],
                       stable_learning_reliability_generalization="NOT_VERIFIED")
        _write_json(output / "summary.json", summary)
        print(json.dumps({key: summary[key] for key in
                          ("engineering_loop", "stop_reason", "training_transitions", "learning_wall_s",
                           "nominal_contact_recovery")}), flush=True)
    return 1 if stop_reason == "ENGINEERING_ERROR" else 0 if engineering_passed else 2


def _evaluation_brief(result, arrival):
    if result is None:
        return None
    keys = ("passed", "success_count", "episode_count", "success_rate", "mean_total_reward",
            "mean_event_reward", "failure_counts", "terminal_arrival", "candidate_agility") if arrival else (
                "passed", "total_event_reward", "episode_elapsed_s", "failure_modes", "events")
    return {key: result[key] for key in keys}
