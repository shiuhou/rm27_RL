"""State-based no-contact terminal arrival through the existing CTBR plant."""
from __future__ import annotations

import copy
import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from rm27_contact_physics.arrival import ArrivalEvaluator, terminal_geometry
from .paths import ROOT
from .rm27_adapter import RM27Plant, load_rl_config

DEFAULT_ARRIVAL_CONFIG = ROOT / "configs/agile_flight/terminal_arrival.json"
# Every per-episode attribute that step/reset mutate besides the MuJoCo data.
_EPISODE_FIELDS = ("initial_coordinates", "episode_start", "previous_action_ctbr", "_terminal_result",
                   "reward_component_totals", "informed_prefix_s", "action_clipped_time_s",
                   "motor_command_saturation_time_s", "evaluator", "_active", "dynamics_scales")


class AgileTerminalArrivalEnv(gym.Env):
    metadata = {"render_modes": []}
    task_id = "agile_terminal_arrival_v1"
    observation_schema = "arrival_world_v1"

    def __init__(self, config=DEFAULT_ARRIVAL_CONFIG):
        super().__init__()
        self.rl_config, cfg, self.sources = load_rl_config(config, task_id=self.task_id)
        if self.rl_config.get("observation_schema") != self.observation_schema:
            raise ValueError("Arrival requires its own observation schema")
        self.plant = RM27Plant(cfg, self.sources)
        model = self.plant.model
        self._uav_body = model.body("uav").id
        self._motors = [model.actuator(f"uav_motor_{i}").id for i in range(4)]
        self._nominal_mass = float(model.body_mass[self._uav_body])
        self._nominal_thrust_gain = model.actuator_gainprm[self._motors, 0].copy()
        self.dynamics_scales = dict(mass=1., thrust=1.)
        self.geometry = terminal_geometry(self.rl_config, self.plant.physical)
        self.target_position = self.geometry["position_target"].copy()
        self.target_normal = self.geometry["normal"].copy()
        self.physics_dt_s = float(cfg["physics_dt_s"])
        self.policy_dt_s = float(self.rl_config["policy_dt_s"])
        self.physics_substeps = round(self.policy_dt_s / self.physics_dt_s)
        if not np.isclose(self.physics_substeps * self.physics_dt_s, self.policy_dt_s, rtol=0, atol=1e-12):
            raise ValueError("Policy period must contain integer physical intervals")
        self.horizon_s = float(self.rl_config["horizon_s"])
        if not np.isfinite(self.horizon_s) or self.horizon_s <= 0:
            raise ValueError("Positive finite arrival horizon required")
        self.initial_ranges = np.asarray([self.rl_config["initial_state"][k]
                                         for k in ("L_m", "eta1_m", "eta2_m")], float)
        if (self.initial_ranges.shape != (3, 2) or not np.isfinite(self.initial_ranges).all()
                or np.any(self.initial_ranges[:, 1] < self.initial_ranges[:, 0])):
            raise ValueError("Finite ordered initial position ranges required")
        rewards = self.rl_config["reward"]
        required = {"progress_gain", "distance_scale_m", "near_scale_m", "time_cost_per_s",
                    "velocity_cost_per_s", "attitude_cost_per_s", "rate_cost_per_s",
                    "saturation_cost_per_s", "velocity_scale_m_s", "attitude_scale_deg",
                    "rate_scale_rad_s", "event_success", "event_failure", "remaining_failure_cost_per_s"}
        # Optional shaping terms; absent means the original I4 reward.
        optional = {"terminal_precision_bonus", "terminal_precision_requires_crossing", "action_smoothness_cost_per_s",
                    "action_magnitude_cost_per_s", "success_accuracy_bonus"}
        if (not required <= set(rewards) <= required | optional
                or not np.isfinite(list(rewards.values())).all()):
            raise ValueError("Expected declared finite arrival reward terms")
        if any(rewards[k] <= 0 for k in ("distance_scale_m", "near_scale_m", "velocity_scale_m_s",
                                        "attitude_scale_deg", "rate_scale_rad_s")):
            raise ValueError("Reward scales must be positive")
        if rewards["event_success"] <= 0 or rewards["event_failure"] >= 0:
            raise ValueError("Success and failure event signs are invalid")
        if any(v < 0 for k, v in rewards.items() if k != "event_failure"):
            raise ValueError("Reward gains and costs must be nonnegative")
        rate = float(self.rl_config["action"]["body_rate_limit_rad_s"])
        self._action_low = np.array([0., -rate, -rate, -rate])
        self._action_high = np.array([4 * self.plant.maximum, rate, rate, rate])
        self.action_space = spaces.Box(self._action_low.astype(np.float32), self._action_high.astype(np.float32))
        self.observation_space = spaces.Box(-np.inf, np.inf, (26,), dtype=np.float32)
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self._active = False
        self._closed = False
        self.evaluator = None
        self._terminal_result = None

    def _observation(self):
        state = self.plant.state
        remaining = max(0., self.horizon_s - (state.timestamp - self.episode_start))
        return np.r_[self.target_position - state.position, self.target_normal,
                     state.velocity, state.rotation.reshape(-1), state.omega_body,
                     self.previous_action_ctbr, remaining].astype(np.float32)

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError("Cannot reset a closed Arrival environment")
        if options and set(options) != {"initial_coordinates"}:
            raise ValueError("Arrival reset only accepts initial_coordinates")
        super().reset(seed=seed)
        if options:
            initial = np.asarray(options["initial_coordinates"], float)
            if (initial.shape != (3,) or not np.isfinite(initial).all()
                    or np.any(initial < self.initial_ranges[:, 0] - 1e-12)
                    or np.any(initial > self.initial_ranges[:, 1] + 1e-12)):
                raise ValueError("Evaluation initial coordinates outside declared initial region")
        else:
            initial = self.np_random.uniform(self.initial_ranges[:, 0], self.initial_ranges[:, 1])
        self.initial_coordinates = initial.copy()
        axes = self.geometry["target_rotation"]
        position = self.target_position + initial[0] * self.target_normal + axes[:, :2] @ initial[1:]
        state = self.plant.reset(position=position, velocity=np.zeros(3), rotation=np.eye(3),
                                 omega_body=np.zeros(3))
        self.episode_start = state.timestamp
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self._terminal_result = None
        self.reward_component_totals = dict(progress=0., cost=0., event=0.)
        self.informed_prefix_s = 0.
        self.action_clipped_time_s = 0.
        self.motor_command_saturation_time_s = 0.
        self.evaluator = ArrivalEvaluator(self.rl_config, self.plant.physical, self.episode_start)
        self.evaluator.initialize(self.plant.model, self.plant.data, state)
        self._active = True
        return self._observation(), self._info(0, self.episode_start, False, False,
                                              dict(progress=0., cost=0., event=0.))

    def set_dynamics_scales(self, *, mass=1., thrust=1.):
        """Scale the simulated plant's mass and motor thrust gain from their compiled values.

        Training-only domain randomization: the rate proxy, mixer, hover command and
        reference keep the nominal profile, as a real controller would. The new values
        take effect from the next reset or step.
        """
        if not all(np.isfinite([mass, thrust])) or mass <= 0 or thrust <= 0:
            raise ValueError("Dynamics scales must be positive and finite")
        model = self.plant.model
        model.body_mass[self._uav_body] = self._nominal_mass * mass
        model.actuator_gainprm[self._motors, 0] = self._nominal_thrust_gain * thrust
        self.dynamics_scales = dict(mass=float(mass), thrust=float(thrust))

    def episode_snapshot(self):
        """Complete mutable episode state: a full MjData copy plus env/evaluator accounting."""
        if self._closed or self.evaluator is None:
            raise RuntimeError("Snapshot requires a reset, open Arrival environment")
        data = mujoco.MjData(self.plant.model)
        mujoco.mj_copyData(data, self.plant.model, self.plant.data)
        return dict(data=data, fields=copy.deepcopy({name: getattr(self, name) for name in _EPISODE_FIELDS}))

    def restore_episode(self, snapshot):
        """Return this instance to a snapshot of itself; the RNG stream is not part of it."""
        if self._closed:
            raise RuntimeError("Cannot restore a closed Arrival environment")
        mujoco.mj_copyData(self.plant.data, self.plant.model, snapshot["data"])
        for name, value in copy.deepcopy(snapshot["fields"]).items():
            setattr(self, name, value)
        self.set_dynamics_scales(**self.dynamics_scales)

    def start_scored_segment(self):
        """Training-only informed reset: later reward totals exclude the prefix.

        The prefix was flown through this same ``step``; time, evaluator history,
        safety latches and the remaining-time terms continue unchanged.
        """
        if not self._active:
            raise RuntimeError("Scored segment requires an active episode")
        self.informed_prefix_s = float(self.plant.data.time - self.episode_start)
        self.reward_component_totals = dict(progress=0., cost=0., event=0.)

    def _process_reward(self, before_metrics, after_metrics, dt, saturated):
        r = self.rl_config["reward"]
        before_d = before_metrics["bumper_position_error_m"]
        d = after_metrics["bumper_position_error_m"]
        phi = lambda distance: 1. / (1. + (distance / r["distance_scale_m"]) ** 2)
        capped = lambda error, scale: min((error / scale) ** 2, 1.)
        gate = np.exp(-(d / r["near_scale_m"]) ** 2)
        cost = (r["time_cost_per_s"] + gate * (
                r["velocity_cost_per_s"] * capped(after_metrics["velocity_error_m_s"], r["velocity_scale_m_s"])
                + r["attitude_cost_per_s"] * capped(after_metrics["attitude_error_rad"],
                                                    np.deg2rad(r["attitude_scale_deg"])))
                + r["rate_cost_per_s"] * capped(after_metrics["angular_velocity_error_rad_s"], r["rate_scale_rad_s"])
                + r["saturation_cost_per_s"] * bool(saturated))
        return dict(progress=float(r["progress_gain"] * (phi(d) - phi(before_d))),
                    cost=-float(dt * cost), event=0.)

    def _info(self, substeps, step_start, saturated, clipped, components):
        now = float(self.plant.data.time)
        info = dict(time_s=now, episode_elapsed_s=now - self.episode_start,
                    step_elapsed_s=now - step_start, physics_substeps=substeps,
                    previous_action_ctbr=self.previous_action_ctbr.copy(),
                    motor_commands_N=self.plant.data.ctrl.copy(), proxy_saturated=bool(saturated),
                    action_clipped=bool(clipped), task_id=self.task_id, observation_schema=self.observation_schema,
                    initial_coordinates=self.initial_coordinates.copy(),
                    observation_source="MUJOCO_TRUTH_STATE_WITHOUT_CONTACT_OR_EVALUATOR_PHASE",
                    reward_components=components.copy(), metrics=copy.deepcopy(self.evaluator.last_metrics),
                    events=copy.deepcopy(self.evaluator.events),
                    previous_action_initialization="synthetic_hover_at_reset" if substeps == 0 else None)
        if self._terminal_result is not None:
            info["result"] = copy.deepcopy(self._terminal_result)
            info["termination_reason"] = ("ARRIVAL_SUCCESS" if self._terminal_result["passed"]
                                          else list(self._terminal_result["failure_modes"]))
        return info

    def step(self, action):
        if not self._active or self._closed:
            raise RuntimeError("Call reset before step; terminated episodes are not automatically reset")
        requested = np.asarray(action, float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("CTBR action must be four finite physical values")
        command = np.clip(requested, self._action_low, self._action_high)
        clipped = not np.array_equal(command, requested)
        motors, saturated = self.plant.apply_ctbr(command)
        # Optional action regularizers, normalized by the physical half-range per axis.
        r = self.rl_config["reward"]
        scale = (self._action_high - self._action_low) / 2
        action_cost = (float(r.get("action_smoothness_cost_per_s", 0.)) * float(np.sum(((command - self.previous_action_ctbr) / scale) ** 2))
                       + float(r.get("action_magnitude_cost_per_s", 0.)) * float(np.sum(
                           ((command - np.r_[self.plant.hover_force, 0., 0., 0.]) / scale) ** 2)))
        self.previous_action_ctbr = command.copy()
        step_start = float(self.plant.data.time)
        components = dict(progress=0., cost=0., event=0.)
        substeps = 0
        for _ in range(self.physics_substeps):
            before = self.plant.state
            before_metrics = self.evaluator.metrics(before)
            state = self.evaluator.advance(self.plant.model, self.plant.data, before, saturated=saturated)
            dt = state.timestamp - before.timestamp
            terms = self._process_reward(before_metrics, self.evaluator.last_metrics, dt, saturated)
            terms["cost"] -= dt * action_cost
            self.action_clipped_time_s += dt * clipped
            self.motor_command_saturation_time_s += dt * bool(
                np.any(motors <= 0.) or np.any(motors >= self.plant.maximum))
            substeps += 1
            if self.evaluator.done:
                r = self.rl_config["reward"]
                accuracy = 0.
                if self.evaluator.arrived:
                    errors = self.evaluator.terminal_normalized_errors(self.evaluator.last_metrics)
                    accuracy = float(np.clip(1. - np.mean(errors ** 2), 0., 1.))
                terms["event"] = (float(r["event_success"]) * (1. + float(r.get("success_accuracy_bonus", 0.)) * accuracy)
                                  if self.evaluator.arrived else
                                  float(r["event_failure"] - r["remaining_failure_cost_per_s"] *
                                        max(0., self.horizon_s - (state.timestamp - self.episode_start))))
                ratio = self.evaluator.best_violation_ratio
                # Optionally pay the closeness bonus only to episodes whose facing point reached the
                # gap window, so hovering in front of the target cannot harvest it until timeout.
                crossed = self.evaluator.min_plane_gap <= (self.rl_config["terminal"]["gap_m"]
                                                           + self.rl_config["terminal"]["gap_tolerance_m"])
                eligible = crossed or not r.get("terminal_precision_requires_crossing", 0)
                precision = (0. if not np.isfinite(ratio) or not eligible else
                             float(r.get("terminal_precision_bonus", 0.)) * (1. if ratio <= 1. else 1. / ratio))
                terms["event"] += precision
                self._terminal_result = copy.deepcopy(self.evaluator.finalize(self.plant.model, self.plant.data))
                self._active = False
            for key, value in terms.items():
                components[key] += value
                self.reward_component_totals[key] += value
            if not self._active:
                self._terminal_result.update(
                    task_id=self.task_id, observation_schema=self.observation_schema,
                    reward_component_totals=self.reward_component_totals.copy(),
                    total_reward=sum(self.reward_component_totals.values()),
                    informed_prefix_s=self.informed_prefix_s,
                    dynamics_scales=dict(self.dynamics_scales),
                    action_clipped_time_s=self.action_clipped_time_s,
                    motor_command_saturation_time_s=self.motor_command_saturation_time_s,
                    terminal_precision_bonus=precision,
                    profile_status=self.plant.profile["status"],
                    reward_status=("I4_TERMINAL_PRECISION_SHAPING_V1_NOT_VALIDATED_LEARNING"
                                   if "terminal_precision_bonus" in self.rl_config["reward"]
                                   else "I4_PERCH_STYLE_SHAPING_V1_NOT_VALIDATED_LEARNING"
                                   if "success_accuracy_bonus" in self.rl_config["reward"]
                                   else "FROZEN_I4_FIRST_EXPERIMENT_NOT_VALIDATED_LEARNING"))
                break
        return self._observation(), sum(components.values()), not self._active, False, self._info(
            substeps, step_start, saturated, clipped, components)

    def close(self):
        self._active = False
        self._closed = True
        self.evaluator = None
        self.plant = None
