"""Bounded E1 policy integration on the audited, nominal I4 dynamics.

The actor and critic consume the original 26 observation fields. Motor
activations and evaluator history belong to the rollout, never to the policy.
The supplied environment is a dedicated authoritative shadow: its normal
``step`` owns every 1 ms event and is intentionally advanced by this function.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import torch
from torch import Tensor, nn

from .diff_dynamics import DynamicsParameters, step_control


ACTION_PARAMETERIZATION = "hover_centered_v1"
CHECKPOINT_FORMAT = "rm27_diff_policy_e1_a_v1"


def _tensor(value, reference: Tensor) -> Tensor:
    return torch.as_tensor(np.array(value, copy=True), dtype=reference.dtype, device=reference.device)


def _rotation(quaternion: Tensor) -> Tensor:
    w, x, y, z = quaternion.unbind(-1)
    return torch.stack((1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y),
                        2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x),
                        2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)), -1).reshape(
                            *quaternion.shape[:-1], 3, 3)


def arrival_observation(state: Tensor, previous_action: Tensor, elapsed_s: float, env) -> Tensor:
    """Original ``arrival_world_v1`` fields, including its float32 conversion.

    A dtype conversion retains autograd's floating-point derivative. It does
    not imply a derivative of exact, real-number quantization boundaries.
    """
    if state.shape != (17,) or previous_action.shape != (4,):
        raise ValueError("Expected one internal state17 and previous physical CTBR4")
    if not np.isfinite(elapsed_s):
        raise ValueError("Elapsed time must be finite")
    remaining = state.new_tensor(max(0., env.horizon_s - float(elapsed_s))).reshape(1)
    observation = torch.cat((_tensor(env.target_position, state) - state[:3],
                             _tensor(env.target_normal, state), state[7:10],
                             _rotation(state[3:7]).reshape(-1), state[10:13],
                             previous_action, remaining))
    return observation.to(torch.float32).to(state.dtype)


def physical_action(normalized: Tensor, env, parameterization=ACTION_PARAMETERIZATION) -> Tensor:
    """Exact existing NormalizedCTBR mapping, before the environment input clip."""
    if normalized.shape[-1:] != (4,) or not bool(torch.isfinite(normalized).all()):
        raise ValueError("Expected four finite normalized CTBR values")
    low, high = _tensor(env._action_low, normalized), _tensor(env._action_high, normalized)
    clipped = normalized.clamp(-1., 1.)
    if parameterization == "bounds_affine_v1":
        return low + (clipped + 1) * (high - low) / 2
    if parameterization != ACTION_PARAMETERIZATION:
        raise ValueError("Unknown action parameterization")
    center = _tensor(np.r_[env.plant.hover_force, np.zeros(3)], normalized)
    return center + clipped * torch.maximum(high - center, center - low)


class ActorCritic(nn.Module):
    """Separate deterministic actor and observation-only bootstrap critic."""

    def __init__(self, env, hidden_size=64):
        super().__init__()
        if not isinstance(hidden_size, int) or hidden_size <= 0:
            raise ValueError("A positive hidden size is required")
        self.hidden_size = hidden_size
        self.action_parameterization = ACTION_PARAMETERIZATION
        # Explicit physical scales, fixed for a checkpoint; no running statistics
        # or privileged state are fitted by this engineering smoke test.
        safety = env.rl_config["safety"]
        scales = np.r_[np.ones(6), np.full(3, safety["max_speed_m_s"]), np.ones(9),
                       np.full(3, safety["max_body_rate_rad_s"]),
                       np.maximum(np.abs(env._action_low), np.abs(env._action_high)), env.horizon_s]
        if scales.shape != (26,) or not np.isfinite(scales).all() or np.any(scales <= 0):
            raise ValueError("Invalid original-observation physical scales")
        self.register_buffer("observation_scale", torch.tensor(scales, dtype=torch.float64))
        def network(outputs, squash=False):
            layers = [nn.Linear(26, hidden_size), nn.Tanh(), nn.Linear(hidden_size, hidden_size),
                      nn.Tanh(), nn.Linear(hidden_size, outputs)]
            return nn.Sequential(*layers, *([nn.Tanh()] if squash else []))
        self.actor, self.critic = network(4, True), network(1)
        self.to(device="cpu", dtype=torch.float64)

    def _input(self, observation):
        if observation.shape[-1:] != (26,) or not bool(torch.isfinite(observation).all()):
            raise ValueError("Policy requires the original 26 finite observation fields")
        return observation.to(self.observation_scale) / self.observation_scale

    def act(self, observation: Tensor) -> Tensor:
        return self.actor(self._input(observation))

    def value(self, observation: Tensor) -> Tensor:
        return self.critic(self._input(observation)).squeeze(-1)

    def predict(self, observation, deterministic=True):
        """Existing MuJoCo evaluation interface; this actor is deterministic."""
        with torch.no_grad():
            value = self.act(torch.as_tensor(np.asarray(observation), dtype=torch.float64))
        return value.cpu().numpy(), None


@contextmanager
def _frozen_parameters(module):
    settings = [(parameter, parameter.requires_grad) for parameter in module.parameters()]
    try:
        for parameter, _ in settings:
            parameter.requires_grad_(False)
        yield
    finally:
        for parameter, enabled in settings:
            parameter.requires_grad_(enabled)


def _metrics(state: Tensor, env):
    """Continuous quantities in ArrivalEvaluator.metrics, in the same units."""
    geometry, terminal = env.geometry, env.rl_config["terminal"]
    normal, center = _tensor(geometry["normal"], state), _tensor(geometry["center"], state)
    rotation = _rotation(state[3:7])
    offset = rotation @ _tensor(geometry["offset_body"], state)
    omega_world = rotation @ state[10:13]
    bumper = state[:3] + offset
    facing = bumper - geometry["radius"] * normal
    material = state[7:10] + torch.linalg.cross(omega_world, offset - geometry["radius"] * normal)
    gap = normal @ (facing - center)
    com_normal, material_normal = -normal @ state[7:10], -normal @ material
    relative = _tensor(geometry["target_rotation"], state).T @ rotation
    cosine = ((torch.trace(relative) - 1) / 2).clamp(-1., 1.)
    sine = torch.linalg.vector_norm(torch.stack((relative[2, 1] - relative[1, 2],
                                               relative[0, 2] - relative[2, 0],
                                               relative[1, 0] - relative[0, 1]))) / 2
    # atan2 avoids acos's infinite derivative at identity/pi. At the pi cut
    # locus the norm's selected subgradient is finite, not a unique derivative.
    attitude = torch.atan2(sine, cosine)
    norm = torch.linalg.vector_norm
    return dict(plane_gap_error_m=(gap - terminal["gap_m"]).abs(),
                tangent_position_error_m=norm(facing - center - gap * normal),
                com_normal_speed_m_s=com_normal, material_normal_speed_m_s=material_normal,
                com_tangent_speed_m_s=norm(state[7:10] + com_normal * normal),
                material_tangent_speed_m_s=norm(material + material_normal * normal),
                bumper_position_error_m=norm(bumper - _tensor(geometry["bumper_target"], state)),
                velocity_error_m_s=torch.maximum(norm(state[7:10] - _tensor(geometry["velocity_target"], state)),
                                                 norm(material - _tensor(geometry["velocity_target"], state))),
                attitude_error_rad=attitude, attitude_error_deg=torch.rad2deg(attitude),
                angular_velocity_error_rad_s=norm(state[10:13]))


def _normalized_errors(metrics, env):
    terminal = env.rl_config["terminal"]
    speed = terminal["normal_speed_m_s"]
    return torch.stack((metrics["plane_gap_error_m"] / terminal["gap_tolerance_m"],
                        metrics["tangent_position_error_m"] / terminal["tangent_position_tolerance_m"],
                        (metrics["com_normal_speed_m_s"] - speed).abs() / terminal["normal_speed_tolerance_m_s"],
                        (metrics["material_normal_speed_m_s"] - speed).abs() / terminal["normal_speed_tolerance_m_s"],
                        metrics["com_tangent_speed_m_s"] / terminal["tangent_speed_tolerance_m_s"],
                        metrics["material_tangent_speed_m_s"] / terminal["tangent_speed_tolerance_m_s"],
                        metrics["attitude_error_deg"] / terminal["attitude_tolerance_deg"],
                        metrics["angular_velocity_error_rad_s"] / terminal["body_rate_tolerance_rad_s"]))


def _running_reward(before, after, action, previous, dt, saturated, env):
    reward = env.rl_config["reward"]
    phi = lambda distance: 1 / (1 + (distance / reward["distance_scale_m"]).square())
    capped = lambda error, scale: (error / scale).square().clamp(max=1.)
    distance = after["bumper_position_error_m"]
    gate = torch.exp(-(distance / reward["near_scale_m"]).square())
    cost = (reward["time_cost_per_s"] + gate * (
            reward["velocity_cost_per_s"] * capped(after["velocity_error_m_s"], reward["velocity_scale_m_s"])
            + reward["attitude_cost_per_s"] * capped(after["attitude_error_rad"], np.deg2rad(reward["attitude_scale_deg"])))
            + reward["rate_cost_per_s"] * capped(after["angular_velocity_error_rad_s"], reward["rate_scale_rad_s"])
            + reward["saturation_cost_per_s"] * bool(saturated))
    scale = _tensor((env._action_high - env._action_low) / 2, action)
    hover = _tensor(np.r_[env.plant.hover_force, np.zeros(3)], action)
    action_cost = (reward.get("action_smoothness_cost_per_s", 0.) * ((action - previous) / scale).square().sum()
                   + reward.get("action_magnitude_cost_per_s", 0.) * ((action - hover) / scale).square().sum())
    return reward["progress_gain"] * (phi(distance) - phi(before["bumper_position_error_m"])) - dt * (cost + action_cost)


def _event_reward(metrics, best_ratio, elapsed, env):
    reward, evaluator = env.rl_config["reward"], env.evaluator
    if evaluator.arrived:
        accuracy = (1 - _normalized_errors(metrics, env).square().mean()).clamp(0., 1.)
        event = reward["event_success"] * (1 + reward.get("success_accuracy_bonus", 0.) * accuracy)
    else:
        remaining = env.horizon_s if reward.get("failure_penalty_time_independent", 0) else max(0., env.horizon_s - elapsed)
        event = best_ratio.new_tensor(reward["event_failure"] - reward["remaining_failure_cost_per_s"] * remaining)
    crossed = evaluator.min_plane_gap <= env.rl_config["terminal"]["gap_m"] + env.rl_config["terminal"]["gap_tolerance_m"]
    eligible = crossed or not reward.get("terminal_precision_requires_crossing", 0)
    timeout = "ARRIVAL_TIMEOUT" in evaluator.reasons and not evaluator.arrived
    eligible = eligible and not (timeout and reward.get("terminal_precision_excludes_timeout", 0))
    if eligible and bool(torch.isfinite(best_ratio)):
        # Clamping before reciprocal avoids inf/NaN gradients in an inactive
        # torch.where reciprocal branch at a zero violation ratio.
        event = event + reward.get("terminal_precision_bonus", 0.) / best_ratio.clamp_min(1.)
    return event


@dataclass
class WindowRollout:
    objective: Tensor
    critic_target: Tensor
    initial_observation: Tensor
    final_observation: Tensor
    final_state: Tensor
    terminated: bool
    substeps: list[int]
    rewards: list[Tensor]
    bootstrap: Tensor
    diagnostics: dict
    observations: list[Tensor]
    actions: list[Tensor]


def _nominal_parameters(env):
    if (env.task_id != "agile_terminal_arrival_v1" or env.observation_schema != "arrival_world_v1"
            or env.terminal_tolerance_scale != 1. or env.dynamics_scales != dict(mass=1., thrust=1.)
            or not np.allclose(env.plant.model.actuator_gainprm[env._motors, 0], env._nominal_thrust_gain,
                               rtol=0, atol=0)):
        raise ValueError("E1-A supports only nominal I4 dynamics and original terminal tolerances")
    return DynamicsParameters.from_env(env)


def rollout_window(env, policy: ActorCritic, *, steps=16, gamma=.995, budget=None) -> WindowRollout:
    """Truncated BPTT with detached decisions from the real ArrivalEvaluator.

    Initial state/history are constants at the window boundary. The shadow
    environment continues normally across calls, including all motor and scorer
    memory. The returned critic target is detached; bootstrap weights are
    frozen while its observation derivative remains in the actor objective.
    """
    if not isinstance(steps, int) or not 1 <= steps <= 16 or not np.isfinite(gamma) or not 0 < gamma <= 1:
        raise ValueError("E1-A requires 1..16 controls (at most 80 ms) and 0<gamma<=1")
    parameters = _nominal_parameters(env)
    if env.evaluator is None or env._closed:
        raise RuntimeError("Rollout requires a reset, open dedicated shadow environment")
    if (policy.observation_scale.dtype != torch.float64 or policy.observation_scale.device.type != "cpu"
            or policy.action_parameterization != ACTION_PARAMETERIZATION):
        raise ValueError("E1-A policy requires CPU float64 and hover_centered_v1 actions")
    state = torch.tensor(np.r_[env.plant.data.qpos, env.plant.data.qvel, env.plant.data.act], dtype=torch.float64)
    previous = torch.tensor(env.previous_action_ctbr.copy(), dtype=torch.float64)
    elapsed = float(env.plant.data.time - env.episode_start)
    observation = arrival_observation(state, previous, elapsed, env)
    initial_observation = observation
    zero = state.new_zeros(())
    rewards, substeps, observations, actions = [], [], [], []
    best_ratio = state.new_tensor(env.evaluator.best_violation_ratio)
    diagnostic = dict(state_max_abs_error=0., observation_max_abs_error=0., reward_max_abs_error=0.,
                      physical_action_clipped_controls=0, motor_saturated_controls=0,
                      motor_at_bound_controls=0, authoritative_rewards=[],
                      terminal_reason=None, initial_terminal=bool(env.evaluator.done),
                      actor_observation_schema=env.observation_schema, actor_observation_size=26,
                      critic_observation_size=26, event_decisions="detached_authoritative_1ms",
                      initial_motor_state=state[13:].tolist())
    for _ in range(steps):
        if env.evaluator.done or not env._active:
            break
        observations.append(observation)
        normalized = policy.act(observation)
        action = physical_action(normalized, env, policy.action_parameterization)
        control = step_control(state, action, parameters)
        actions.append(normalized)
        captured = []
        evaluator = env.evaluator
        original_advance = evaluator.advance
        had_override = "advance" in evaluator.__dict__
        previous_override = evaluator.__dict__.get("advance")
        def advance(model, data, before, saturated=False):
            was_arrived = evaluator.arrived
            endpoint = original_advance(model, data, before, saturated=saturated)
            # Timeout is added after snapshot's best-ratio update. All other
            # failure reasons prevent that update; safety wins over arrival.
            valid_precision = not was_arrived and not (set(evaluator.reasons) - {"ARRIVAL_TIMEOUT"})
            captured.append((float(endpoint.timestamp - before.timestamp), valid_precision))
            return endpoint
        if budget is not None:
            # Conservative charge before mutation, even if an event uses only
            # one of these five reserved physical intervals.
            budget.tick(controls=1, substeps=parameters.substeps)
        try:
            evaluator.advance = advance
            actual_obs, actual_reward, terminated, truncated, info = env.step(action.detach().numpy())
        finally:
            if had_override:
                evaluator.advance = previous_override
            else:
                evaluator.__dict__.pop("advance", None)
        count = int(info["physics_substeps"])
        if truncated or count != len(captured) or not 1 <= count <= parameters.substeps:
            raise RuntimeError("Unexpected authoritative interval or termination semantics")
        if np.any(env.plant.data.warning.number):
            raise RuntimeError("MuJoCo warning during E1-A rollout")
        step_reward = zero
        before_metrics = _metrics(state, env)
        for index, (dt, valid_precision) in enumerate(captured):
            endpoint = control.endpoints[index]
            after_metrics = _metrics(endpoint, env)
            step_reward = step_reward + _running_reward(before_metrics, after_metrics,
                                                        control.clipped_action, previous, dt,
                                                        info["proxy_saturated"], env)
            if valid_precision:
                best_ratio = torch.minimum(best_ratio, _normalized_errors(after_metrics, env).max())
            before_metrics = after_metrics
        state = control.endpoints[count - 1]
        previous = control.clipped_action
        elapsed = float(info["episode_elapsed_s"])
        if terminated:
            step_reward = step_reward + _event_reward(before_metrics, best_ratio, elapsed, env)
            diagnostic["terminal_reason"] = info["termination_reason"]
        observation = arrival_observation(state, previous, elapsed, env)
        actual_state = np.r_[env.plant.data.qpos, env.plant.data.qvel, env.plant.data.act]
        state_error = float(np.max(np.abs(state.detach().numpy() - actual_state)))
        obs_error = float(np.max(np.abs(observation.detach().numpy() - actual_obs)))
        reward_error = abs(float(step_reward.detach()) - float(actual_reward))
        diagnostic["state_max_abs_error"] = max(diagnostic["state_max_abs_error"], state_error)
        diagnostic["observation_max_abs_error"] = max(diagnostic["observation_max_abs_error"], obs_error)
        diagnostic["reward_max_abs_error"] = max(diagnostic["reward_max_abs_error"], reward_error)
        diagnostic["physical_action_clipped_controls"] += int(info["action_clipped"])
        diagnostic["motor_saturated_controls"] += int(info["proxy_saturated"])
        diagnostic["motor_at_bound_controls"] += int(np.any(info["motor_commands_N"] <= 0)
                                                       or np.any(info["motor_commands_N"] >= env.plant.maximum))
        diagnostic["authoritative_rewards"].append(float(actual_reward))
        if (not np.isfinite([state_error, obs_error, reward_error]).all()
                or state_error > 1e-9 or obs_error > 2e-6
                or reward_error > 1e-7 + 1e-8 * abs(float(actual_reward))):
            raise RuntimeError("Differentiable/authoritative rollout mismatch: " + str(
                dict(state=state_error, observation=obs_error, reward=reward_error)))
        rewards.append(step_reward)
        substeps.append(count)
    terminated = bool(env.evaluator.done or not env._active)
    bootstrap = zero
    if not terminated:
        with _frozen_parameters(policy.critic):
            bootstrap = policy.value(observation)
    objective = sum((gamma ** index * reward for index, reward in enumerate(rewards)), zero)
    objective = objective + gamma ** len(rewards) * bootstrap
    if not bool(torch.isfinite(objective)):
        raise RuntimeError("Nonfinite E1-A objective")
    diagnostic.update(controls=len(substeps), physics_substeps=sum(substeps),
                      reserved_physics_substeps=parameters.substeps * len(substeps),
                      discount_steps=len(rewards), bootstrap_mask=0 if terminated else 1,
                      final_motor_state=state[13:].detach().tolist(),
                      bootstrap_value=float(bootstrap.detach()), gamma=gamma)
    return WindowRollout(objective, objective.detach().clone(), initial_observation,
                         observation, state, terminated, substeps, rewards, bootstrap,
                         diagnostic, observations, actions)


def _json_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _contract(env, policy):
    parameters = _nominal_parameters(env)
    physical = {name: value.detach().cpu().tolist() if isinstance(value, Tensor) else value
                for name, value in asdict(parameters).items()}
    from rm27_contact_physics import arrival, model, plant
    from . import arrival_env, diff_dynamics
    modules = (arrival, model, plant, arrival_env, diff_dynamics)
    sources = {module.__name__: hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
               for module in modules}
    sources[__name__] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    sources["rm27_contact_rl.ppo"] = hashlib.sha256(Path(__file__).with_name("ppo.py").read_bytes()).hexdigest()
    sources.update({"config:" + name: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                    for name, path in env.sources.items()})
    compiled = env.plant.model
    model_fields = ("body_mass", "body_inertia", "body_pos", "body_quat", "body_ipos", "body_iquat",
                    "geom_type", "geom_bodyid", "geom_pos", "geom_quat", "geom_size", "geom_contype",
                    "geom_conaffinity", "geom_friction", "site_bodyid", "site_pos", "site_quat",
                    "actuator_gear", "actuator_gainprm", "actuator_dynprm", "actuator_ctrlrange")
    compiled_values = {name: np.asarray(getattr(compiled, name)).tolist() for name in model_fields}
    return dict(task_id=env.task_id, observation_schema=env.observation_schema,
                observation_size=26, action_parameterization=policy.action_parameterization,
                config=env.rl_config, physical_config=_json_digest(env.plant.physical),
                physics_parameters=_json_digest(physical), compiled_model=_json_digest(compiled_values),
                sources=sources, mujoco_version=mujoco.__version__, torch_version=str(torch.__version__),
                observation_scale=policy.observation_scale.detach().cpu().tolist(), dtype="float64", device="cpu")


def save_checkpoint(path, policy: ActorCritic, env, *, metadata=None):
    """Save a new own-format checkpoint; historical SB3 files are never reused."""
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    metadata = {} if metadata is None else metadata
    _json_digest(metadata)  # Restrict metadata to portable, finite JSON values.
    payload = dict(format=CHECKPOINT_FORMAT, hidden_size=policy.hidden_size,
                   contract=_contract(env, policy), state_dict=policy.state_dict(), metadata=metadata)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation also protects against another worker creating this path.
    with path.open("xb") as handle:
        torch.save(payload, handle)


def load_checkpoint(path, env) -> ActorCritic:
    payload = torch.load(Path(path), map_location="cpu", weights_only=True)
    if not isinstance(payload, dict) or payload.get("format") != CHECKPOINT_FORMAT:
        raise ValueError("Not an E1-A differentiable-policy checkpoint")
    policy = ActorCritic(env, hidden_size=payload["hidden_size"])
    if payload.get("contract") != _contract(env, policy):
        raise ValueError("Checkpoint task/config/Physics/source/normalizer/action contract mismatch")
    policy.load_state_dict(payload["state_dict"], strict=True)
    if (not all(bool(torch.isfinite(value).all()) for value in policy.state_dict().values())
            or policy.observation_scale.tolist() != payload["contract"]["observation_scale"]):
        raise ValueError("Checkpoint contains nonfinite weights or a changed normalizer")
    policy.eval()
    return policy
