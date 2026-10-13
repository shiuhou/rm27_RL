"""Gymnasium interaction with the existing native-MuJoCo contact task.

CTBR commands are physical N / FLU rad/s. The event reward is a minimal
interface fixture; it has not been validated as a PPO reward.
"""
from __future__ import annotations

import copy

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from rm27_contact_physics.evaluation import ContactEvaluator
from rm27_contact_physics.model import unit
from .rm27_adapter import DEFAULT_CONFIG, RM27Plant, load_rl_config


class ContactRecoveryEnv(gym.Env):
    """Fixed nominal plant; all dynamic state is private to this instance."""

    metadata = {"render_modes": []}

    def __init__(self, config=DEFAULT_CONFIG):
        super().__init__()
        self.rl_config, cfg, self.sources = load_rl_config(config)
        self.plant = RM27Plant(cfg, self.sources)
        self.physics_dt_s = float(cfg["physics_dt_s"])
        self.policy_dt_s = float(self.rl_config["policy_dt_s"])
        self.physics_substeps = round(self.policy_dt_s / self.physics_dt_s)
        if not np.isclose(self.physics_substeps * self.physics_dt_s, self.policy_dt_s,
                          rtol=0, atol=1e-12):
            raise ValueError("Policy period must contain an integer number of physical intervals")
        self.horizon_s = float(cfg["horizon_s"])
        rate_limit = float(self.rl_config["action"]["body_rate_limit_rad_s"])
        self._action_low = np.array([0, -rate_limit, -rate_limit, -rate_limit], dtype=float)
        self._action_high = np.array([4 * self.plant.maximum, rate_limit, rate_limit, rate_limit], dtype=float)
        self.action_space = spaces.Box(self._action_low.astype(np.float32),
                                       self._action_high.astype(np.float32), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(26,), dtype=np.float32)
        self.target_position = np.asarray(self.plant.physical["target"]["center_m"], dtype=float)
        self.target_normal = unit(self.plant.physical["target"]["normal"])
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self.last_step_contact = False
        self.evaluator = None
        self._active = False
        self._closed = False
        self._terminal_result = None

    def _observation(self):
        state = self.plant.state
        remaining = max(0., self.horizon_s - (state.timestamp - self.episode_start))
        return np.r_[self.target_position - state.position, self.target_normal,
                     state.velocity, state.rotation.reshape(-1), state.omega_body,
                     self.previous_action_ctbr, remaining].astype(np.float32)

    def _info(self, substeps, step_start, motors, saturated, clipped, events):
        now = float(self.plant.data.time)
        info = dict(time_s=now, episode_elapsed_s=now - self.episode_start,
                    step_elapsed_s=now - step_start, physics_substeps=substeps,
                    previous_action_ctbr=self.previous_action_ctbr.copy(),
                    motor_commands_N=motors.copy(), proxy_saturated=bool(saturated),
                    action_clipped=bool(clipped), contact_seen=bool(self.last_step_contact),
                    events=copy.deepcopy(events),
                    previous_action_initialization="synthetic_hover_at_reset" if substeps == 0 else None,
                    observation_source="MUJOCO_TRUTH_STATE_WITHOUT_CONTACT_OR_EVALUATOR_PHASE")
        if self._terminal_result is not None:
            info["result"] = copy.deepcopy(self._terminal_result)
            info["termination_reason"] = ("SUCCESS" if self._terminal_result["passed"]
                                          else list(self._terminal_result["failure_modes"]))
        return info

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError("Cannot reset a closed ContactRecoveryEnv")
        if options:
            raise ValueError("I1 reset supports the fixed nominal scene; no reset options")
        super().reset(seed=seed)
        state = self.plant.reset()
        self.episode_start = state.timestamp
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self.last_step_contact = False
        self._terminal_result = None
        self.evaluator = ContactEvaluator(self.plant.cfg, self.plant.physical, self.episode_start)
        self.evaluator.initialize(self.plant.model, self.plant.data, state)
        self._active = True
        return self._observation(), self._info(
            0, self.episode_start, self.plant.data.ctrl.copy(), False, False, self.evaluator.events)

    def step(self, action):
        if not self._active or self._closed:
            raise RuntimeError("Call reset before step; terminated episodes are not automatically reset")
        requested = np.asarray(action, dtype=float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("CTBR action must be four finite physical values")
        # This remembered command precedes proxy allocation and motor clipping.
        command = np.clip(requested, self._action_low, self._action_high)
        clipped = not np.array_equal(command, requested)
        motors, saturated = self.plant.apply_ctbr(command)
        self.previous_action_ctbr = command.copy()
        step_start = float(self.plant.data.time)
        first_before = self.evaluator.first is not None
        separated_before = self.evaluator.separated
        event_start = len(self.evaluator.events)
        self.last_step_contact = False
        substeps = 0
        terminated = False
        for _ in range(self.physics_substeps):
            before = self.plant.state
            self.evaluator.advance(self.plant.model, self.plant.data, before, saturated=saturated)
            substeps += 1
            irreversible = self.evaluator.irreversible_reasons(self.plant.model, self.plant.data)
            recovered = (self.evaluator.recovered
                         and self.evaluator.recovery_time <= self.plant.cfg["gates"]["recovery_deadline_s"])
            deadline = self.plant.data.time - self.episode_start >= self.horizon_s - 1e-12
            if irreversible or recovered or deadline:
                self._terminal_result = copy.deepcopy(
                    self.evaluator.finalize(self.plant.model, self.plant.data))
                self._terminal_result.update(
                    mode="HEADLESS_MUJOCO_CTBR_CONTACT_RECOVERY_ENV",
                    stopping_protocol="FIRST_IRREVERSIBLE_FAILURE_OR_VALID_RECOVERY_OR_TASK_DEADLINE",
                    profile_status=self.plant.profile["status"],
                    reward_status="MINIMAL_EVENT_FIXTURE_NOT_PPO_VALIDATED")
                self._active = False
                terminated = True
                break
        self.last_step_contact = self.evaluator.consume_contact()
        rewards = self.rl_config["reward"]
        if terminated and not self._terminal_result["passed"]:
            reward = float(rewards["failure"])
        else:
            reward = 0.
            if not first_before and self.evaluator.first is not None:
                first = self.evaluator.first
                if first["valid_part"] and first["front_face"] and not first["edge"]:
                    reward += float(rewards["valid_contact"])
            if not separated_before and self.evaluator.separated:
                reward += float(rewards["separation"])
            if terminated and self._terminal_result["passed"]:
                reward += float(rewards["success"])
        info = self._info(substeps, step_start, motors, saturated, clipped,
                          self.evaluator.events[event_start:])
        return self._observation(), reward, terminated, False, info

    def close(self):
        self._active = False
        self._closed = True
        self.evaluator = None
        self.plant = None
