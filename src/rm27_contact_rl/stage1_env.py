"""Stable-hover takeover to first controlled physical contact, with state PPO inputs."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import gymnasium as gym
import numpy as np
from scipy.spatial.transform import Rotation

from .env import contact_observation
from .paths import ROOT
from .stage1_evaluation import Stage1Evaluator, stage1_geometry


DEFAULT_STAGE1_CONFIG = ROOT / "configs/stage1/task_contact_geometry.json"
TASK_ID = "fixed_surface_contact_v1"
OBSERVATION_SCHEMA = "fixed_surface_state_v1"


def load_stage1_config(path=DEFAULT_STAGE1_CONFIG):
    config = json.loads(Path(path).read_text())
    if (config.get("schema_version") != 1 or config.get("task_id") != TASK_ID
            or config.get("observation_schema") != OBSERVATION_SCHEMA):
        raise ValueError("Stage 1 requires its independent task and observation schema")
    if (config.get("parameter_status") not in {"PROPOSED_UNCONFIRMED", "CONFIRMED"}
            or not isinstance(config.get("training_allowed"), bool)
            or (config["parameter_status"] == "PROPOSED_UNCONFIRMED" and config["training_allowed"])):
        raise ValueError("Unconfirmed task parameters cannot enable policy training")
    for section in ("reward", "contact_acceptance", "safety", "initial_state"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Stage 1 requires the {section} contract")
        values = np.concatenate([np.asarray(value, float).ravel()
                                 for value in config[section].values()])
        if not np.isfinite(values).all():
            raise ValueError(f"Stage 1 {section} values must be finite")
    if (not np.isfinite([config["horizon_s"], config["policy_dt_s"], config["physics_dt_s"],
                         config["action"]["body_rate_limit_rad_s"]]).all()
            or config["horizon_s"] <= 0
            or not np.isclose(config["policy_dt_s"], .005, rtol=0, atol=1e-12)
            or not np.isclose(config["physics_dt_s"], .001, rtol=0, atol=1e-12)
            or config["action"]["body_rate_limit_rad_s"] <= 0):
        raise ValueError("Stage 1 requires positive limits and the original 5/1 ms periods")
    acceptance, safety, reward = config["contact_acceptance"], config["safety"], config["reward"]
    speed = np.asarray(acceptance["normal_speed_range_m_s"])
    if (speed.shape != (2,) or not 0 <= speed[0] <= speed[1]
            or any(acceptance[key] <= 0 for key in ("max_tangent_speed_m_s", "max_body_rate_rad_s",
                                                  "max_penetration_m", "max_plane_error_m"))
            or any(not 0 < acceptance[key] <= 180 for key in
                   ("max_bottom_normal_error_deg", "max_surface_normal_error_deg"))):
        raise ValueError("Invalid provisional contact gates")
    low, high = np.asarray(safety["position_min_target_frame_m"]), np.asarray(safety["position_max_target_frame_m"])
    if (low.shape != (3,) or high.shape != (3,) or np.any(low >= high)
            or safety["max_speed_m_s"] <= 0 or safety["max_body_rate_rad_s"] <= 0):
        raise ValueError("Invalid Stage 1 flight volume or speed/rate limits")
    if (any(reward[key] <= 0 for key in ("guide_distance_scale_m", "alignment_distance_scale_m", "event_success"))
            or reward["event_failure"] >= 0
            or any(value < 0 for key, value in reward.items() if key != "event_failure")):
        raise ValueError("Reward scales and signs must be valid")
    if reward["time_cost_per_s"] <= reward["near_alignment_gain_per_s"]:
        raise ValueError("The bounded near-alignment term must not reward indefinite hovering")
    initial_ranges(config)
    if config["initial_state"]["normal_standoff_m"] <= 0:
        raise ValueError("A positive hover standoff is required")
    return config


def initial_ranges(config):
    spec = config["initial_state"]
    keys = ("position_perturbation_halfwidth_target_frame_m", "velocity_perturbation_halfwidth_world_m_s",
            "rotation_vector_halfwidth_body_deg", "body_rate_perturbation_halfwidth_rad_s")
    if any(np.shape(spec[key]) != (3,) for key in keys):
        raise ValueError("Each hover perturbation block must have three axes")
    half = np.r_[spec["position_perturbation_halfwidth_target_frame_m"],
                 spec["velocity_perturbation_halfwidth_world_m_s"],
                 np.deg2rad(spec["rotation_vector_halfwidth_body_deg"]),
                 spec["body_rate_perturbation_halfwidth_rad_s"]]
    if half.shape != (12,) or np.any(half < 0):
        raise ValueError("Stage 1 requires twelve nonnegative initial perturbation bounds")
    return np.column_stack((-half, half))


def fixed_evaluation_initials(config=DEFAULT_STAGE1_CONFIG):
    """Nineteen fixed starts: nominal and signed position/velocity/attitude axes."""
    config = load_stage1_config(config) if not isinstance(config, dict) else config
    ranges = initial_ranges(config)
    result = [np.zeros(12)]
    for axis in range(9):
        for sign in (-1, 1):
            value = np.zeros(12)
            value[axis] = sign*ranges[axis, 1]
            result.append(value)
    return result


class Stage1ContactEnv(gym.Env):
    metadata = {"render_modes": []}
    task_id, observation_schema = TASK_ID, OBSERVATION_SCHEMA

    def __init__(self, config=DEFAULT_STAGE1_CONFIG, *, plant=None):
        super().__init__()
        self.rl_config = load_stage1_config(config)
        if plant is None:
            from .stage1_plant import Stage1Plant
            plant = Stage1Plant(self.rl_config)
        self.plant = plant
        self.sources = dict(plant.sources, rl_config=Path(config).resolve())
        self.geometry = stage1_geometry(plant)
        self.target_position, self.target_normal = self.geometry["center"], self.geometry["normal"]
        self.initial_ranges = initial_ranges(self.rl_config)
        self.physics_dt_s, self.policy_dt_s = .001, .005
        self.physics_substeps = 5
        if not np.isclose(plant.model.opt.timestep, self.physics_dt_s, rtol=0, atol=1e-12):
            raise ValueError("Stage 1 plant physics period differs from the task contract")
        self.horizon_s = float(self.rl_config["horizon_s"])
        rate = float(self.rl_config["action"]["body_rate_limit_rad_s"])
        self._action_low = np.array([0., -rate, -rate, -rate])
        self._action_high = np.array([4*plant.maximum, rate, rate, rate])
        self.action_space = gym.spaces.Box(self._action_low.astype(np.float32),
                                           self._action_high.astype(np.float32))
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (26,), dtype=np.float32)
        self.previous_action_ctbr = np.r_[plant.hover_force, np.zeros(3)]
        self.evaluator = None
        self._active = self._closed = False
        self._terminal_result = None

    def _observation(self):
        # Only externally estimable state is provided. Contact truth, motor
        # activations and the task scorer remain in diagnostics, never here.
        state = self.plant.state
        remaining = max(0., self.horizon_s-(state.timestamp-self.episode_start))
        observation = contact_observation(state, self.target_position, self.target_normal,
                                          self.previous_action_ctbr, remaining)
        self._observation_valid = bool(np.isfinite(observation).all())
        if self._observation_valid:
            self._last_valid_observation = observation.copy()
            return observation
        if not self._active and hasattr(self, "_last_valid_observation"):
            # No further action is accepted after a numerical failure. Preserve
            # a finite terminal tensor while diagnostics explicitly mark that
            # the physical endpoint observation was invalid.
            return self._last_valid_observation.copy()
        raise RuntimeError("Nonfinite Stage 1 observation outside a terminal numerical failure")

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError("Cannot reset a closed Stage 1 environment")
        super().reset(seed=seed)
        if options is not None and set(options) != {"initial_perturbation"}:
            raise ValueError("Stage 1 reset only accepts initial_perturbation")
        if options is None:
            initial = self.np_random.uniform(self.initial_ranges[:, 0], self.initial_ranges[:, 1])
        else:
            initial = np.asarray(options["initial_perturbation"], float)
            if (initial.shape != (12,) or not np.isfinite(initial).all()
                    or np.any(initial < self.initial_ranges[:, 0]-1e-12)
                    or np.any(initial > self.initial_ranges[:, 1]+1e-12)):
                raise ValueError("Initial perturbation is outside the declared hover region")
        self.initial_perturbation = initial.copy()
        position = (self.geometry["center"]
                    + self.rl_config["initial_state"]["normal_standoff_m"]*self.geometry["normal"]
                    + self.geometry["target_rotation"] @ initial[:3])
        state = self.plant.reset(position=position, velocity=initial[3:6],
                                 rotation=Rotation.from_rotvec(initial[6:9]).as_matrix(),
                                 omega_body=initial[9:12])
        self.episode_start = float(state.timestamp)
        self.previous_action_ctbr = np.r_[self.plant.hover_force, np.zeros(3)]
        self.reward_component_totals = dict(guide=0., alignment=0., action=0., constraint=0., time=0., event=0.)
        self.evaluator = Stage1Evaluator(self.rl_config, self.plant)
        self.evaluator.initialize()
        self._terminal_result = self.evaluator.summary() if self.evaluator.done else None
        self._active = not self.evaluator.done
        return self._observation(), self._info(0, self.episode_start, False, False,
                                              dict.fromkeys(self.reward_component_totals, 0.))

    def _running_reward(self, before, after, command, previous, dt, *, saturated=False, clipped=False):
        """Bounded task shaping; does not participate in independent success scoring."""
        r = self.rl_config["reward"]
        potential = lambda distance: 1./(1.+(distance/r["guide_distance_scale_m"])**2)
        distance = after["position_error_m"]
        near = np.exp(-(distance/r["alignment_distance_scale_m"])**2)
        alignment = (1.+after["contact_alignment_cosine"])/2.
        scale = (self._action_high-self._action_low)/2.
        hover = np.r_[self.plant.hover_force, np.zeros(3)]
        magnitude = np.minimum(((command-hover)/scale)**2, 1.).sum()
        smoothness = np.minimum(((command-previous)/scale)**2, 4.).sum()
        speed = min((after["speed_m_s"]/self.rl_config["safety"]["max_speed_m_s"])**2, 1.)
        rate = min((after["body_rate_rad_s"]/self.rl_config["safety"]["max_body_rate_rad_s"])**2, 1.)
        return dict(guide=float(r["guide_gain"]*(potential(distance)-potential(before["position_error_m"]))),
                    alignment=float(dt*r["near_alignment_gain_per_s"]*near*alignment),
                    action=-float(dt*(r["action_magnitude_cost_per_s"]*magnitude
                                      + r["action_smoothness_cost_per_s"]*smoothness)),
                    constraint=-float(dt*(r["speed_cost_per_s"]*speed+r["body_rate_cost_per_s"]*rate
                                          + r["saturation_cost_per_s"]*bool(saturated)
                                          + r["input_clip_cost_per_s"]*bool(clipped))),
                    time=-float(dt*r["time_cost_per_s"]), event=0.)

    def _info(self, substeps, step_start, saturated, clipped, components):
        info = dict(task_id=self.task_id, observation_schema=self.observation_schema,
                    time_s=float(self.plant.data.time), episode_elapsed_s=self.evaluator.elapsed_s,
                    step_elapsed_s=float(self.plant.data.time)-step_start, physics_substeps=substeps,
                    previous_action_ctbr=self.previous_action_ctbr.copy(),
                    initial_perturbation=self.initial_perturbation.copy(),
                    action_clipped=bool(clipped), proxy_saturated=bool(saturated),
                    observation_source="MUJOCO_STATE_WITHOUT_CONTACT_MOTOR_OR_EVALUATOR_PRIVILEGE",
                    observation_valid=self._observation_valid,
                    reward_components=components.copy(), metrics=copy.deepcopy(self.evaluator.last_metrics),
                    events=copy.deepcopy(self.evaluator.events),
                    last_interval=copy.deepcopy(self.evaluator.last_interval),
                    parameter_status=self.rl_config["parameter_status"],
                    training_allowed=self.rl_config["training_allowed"])
        if self._terminal_result is not None:
            info["result"] = copy.deepcopy(self._terminal_result)
            info["termination_reason"] = ("CONTROLLED_CONTACT" if self._terminal_result["passed"]
                                          else self._terminal_result["failure_modes"])
        return info

    def step(self, action):
        if not self._active or self._closed:
            raise RuntimeError("Call reset before step; finished episodes never auto-reset")
        requested = np.asarray(action, float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("CTBR requires four finite physical values")
        command = np.clip(requested, self._action_low, self._action_high)
        clipped = not np.array_equal(command, requested)
        _, saturated = self.plant.apply_ctbr(command)
        previous = self.previous_action_ctbr.copy()
        self.previous_action_ctbr = command.copy()
        start = float(self.plant.data.time)
        components = dict.fromkeys(self.reward_component_totals, 0.)
        substeps = 0
        for _ in range(self.physics_substeps):
            before = self.plant.state
            before_metrics = self.evaluator.last_metrics
            count_before = self.evaluator.completed_intervals
            self.evaluator.advance(before, saturated=saturated)
            count = self.evaluator.completed_intervals-count_before
            substeps += count
            terms = (self._running_reward(before_metrics, self.evaluator.last_metrics,
                                          command, previous, self.physics_dt_s,
                                          saturated=saturated, clipped=clipped)
                     if count else dict.fromkeys(components, 0.))
            if self.evaluator.done:
                self._terminal_result = self.evaluator.summary()
                terms["event"] = float(self.rl_config["reward"][
                    "event_success" if self._terminal_result["passed"] else "event_failure"])
                self._active = False
            for key, value in terms.items():
                components[key] += value
                self.reward_component_totals[key] += value
            if not self._active:
                self._terminal_result.update(reward_component_totals=self.reward_component_totals.copy(),
                                             total_reward=sum(self.reward_component_totals.values()),
                                             initial_perturbation=self.initial_perturbation.tolist())
                break
        return (self._observation(), float(sum(components.values())), not self._active, False,
                self._info(substeps, start, saturated, clipped, components))

    def close(self):
        self._active, self._closed = False, True
