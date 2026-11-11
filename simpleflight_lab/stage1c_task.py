"""Stage 1-C reset curriculum and training reward for frozen contact geometry."""

import math
from typing import NamedTuple

import torch
from isaaclab.utils.math import quat_apply

from .stage1c_resets import BANDS as INFORMED_SEGMENTS
from .stage1_task import Stage1Task, _rotvec_quaternion


STAGE1C_REWARD = {
    "event_success": 20.0,
    "event_failure": -5.0,
    "time_cost_per_s": .03,
    "body_rate_cost_per_s": .015,
    "command_smoothness_cost_per_s": .02,
    "saturation_cost_per_s": .035,
    "progress_weight": .8,
    "progress_scale_m": .8,
    "capture_weight": .6,
    "capture_gap_scale_m": .16,
}


class Stage1CReset(NamedTuple):
    position: torch.Tensor
    quaternion: torch.Tensor
    velocity: torch.Tensor
    omega_body: torch.Tensor
    motor_thrust_N: torch.Tensor
    motor_command_N: torch.Tensor
    previous_command: torch.Tensor
    physics_ticks: torch.Tensor

    @property
    def policy_ticks(self) -> torch.Tensor:
        return self.physics_ticks // 5


class Stage1CMetrics(NamedTuple):
    distance_m: torch.Tensor
    body_rate_rad_s: torch.Tensor
    unsafe: torch.Tensor
    potential: torch.Tensor


class Stage1CTask(Stage1Task):
    """Train from verified snapshots and a continuous stationary start mixture."""

    num_frontiers = len(INFORMED_SEGMENTS) + 1

    def __init__(self, num_envs, device, geometry, frozen_config, *, starts, gamma,
                 reward_config=None, training=True, goal_inset_m=.012, nominal_standoff_m=(1., 1.25)):
        super().__init__(num_envs, device, geometry, frozen_config, training=training, goal_inset_m=goal_inset_m)
        if self.physics_dt_s != .001 or self.policy_dt_s != .005 or not 0 < gamma <= 1:
            raise ValueError("Stage 1-C requires 1/5 ms clocks and a valid policy-step discount")
        if training and starts is None:
            raise ValueError("Stage 1-C training requires verified informed snapshots")
        if tuple(nominal_standoff_m) != (1., 1.25):
            raise ValueError("Stage 1-C stationary resets require the selected 1.0–1.25 m nominal range")
        self.starts = starts
        self.gamma = gamma
        self.nominal_standoff_m = tuple(nominal_standoff_m)
        self.frontier_index = 0
        self.start_frontier_index = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.start_segment = torch.full_like(self.start_frontier_index, self.num_frontiers - 1)
        self.is_frontier = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self._halfsize = self.center.new_tensor(geometry["target"]["halfsize_m"])
        gates = frozen_config["contact_acceptance"]
        self._inward_low, self._inward_high = gates["normal_speed_range_m_s"]
        self._max_tangent = gates["max_tangent_speed_m_s"]
        self._contact_rate = gates["max_body_rate_rad_s"]
        self._command_scale = self.center.new_tensor([float(self.hover_ctbr[0]) * .5, 2., 2., 2.])

        if reward_config is not None and reward_config.keys() - STAGE1C_REWARD.keys():
            raise ValueError("Unknown Stage 1-C reward setting")
        self.pilot_reward = STAGE1C_REWARD | (reward_config or {})
        r = self.pilot_reward
        if (not all(math.isfinite(value) for value in r.values()) or r["event_failure"] >= 0
                or any(value < 0 for key, value in r.items() if key != "event_failure")
                or min(r["progress_scale_m"], r["capture_gap_scale_m"]) <= 0):
            raise ValueError("Invalid Stage 1-C reward settings")
        max_cost = sum(value for key, value in r.items() if key.endswith("_cost_per_s"))
        max_potential = r["progress_weight"] + r["capture_weight"]
        if self.gamma ** (self.horizon_ticks - 1) * r["event_success"] <= self.horizon_s * max_cost + max_potential:
            raise ValueError("Discounted contact success does not dominate bounded running costs and shaping")

    def set_frontier(self, index: int) -> None:
        if not isinstance(index, int) or not 0 <= index < self.num_frontiers:
            raise ValueError(f"Invalid Stage 1-C frontier index {index}")
        self.frontier_index = index

    def reset(self, ids, progress=0., *, perturbations=None) -> Stage1CReset:
        n = len(ids)
        final = self.num_frontiers - 1
        if self.training:
            if perturbations is None:
                perturbations = (2. * torch.rand((n, 12), device=self.device) - 1.) * self.initial_halfwidth
            draw = torch.rand((n, 2), device=self.device)
            lower, upper = self.nominal_standoff_m
            standoff = lower + (upper - lower) * draw[:, 1]
            position = self.center + standoff[:, None] * self.normal + perturbations[:, :3] @ self.frame.T
            yaw = torch.zeros_like(position)
            yaw[:, 2] = perturbations[:, 8]
            quaternion = _rotvec_quaternion(yaw)
            velocity = torch.zeros_like(position)
            omega = torch.zeros_like(position)
            if self.frontier_index == final:
                # Retain approach starts while full starts are the frontier.
                segment = torch.where(draw[:, 0] < .10, final - 1, final)
            elif self.frontier_index == 0:
                segment = torch.where(draw[:, 0] < .35, final, 0)
            else:
                segment = torch.where(draw[:, 0] < .25, final, self.frontier_index)
                support = (draw[:, 0] >= .25) & (draw[:, 0] < .35)
                segment[support] = self.frontier_index - 1
        else:
            position, quaternion, velocity, omega = super().reset(ids, perturbations=perturbations)
            segment = torch.full_like(ids, final)

        motors = self.hover_ctbr[0].expand(n, 4).clone() * .25
        motor_command = motors.clone()
        previous_command = self.hover_ctbr.expand(n, 4).clone()
        ticks = torch.zeros_like(ids)
        reset = Stage1CReset(position, quaternion, velocity, omega, motors, motor_command, previous_command, ticks)
        for band, name in enumerate(INFORMED_SEGMENTS):
            if not self.training:
                break
            selected = torch.nonzero(segment == band, as_tuple=True)[0]
            if not len(selected):
                continue
            sample = self.starts.sample(len(selected), name, self.device)
            if torch.any(sample.control_phase != 0):
                raise ValueError("Informed reset must be at a complete native CTBR interval")
            fields = (*sample.state, sample.motor_activation_N, sample.motor_command_N,
                      sample.previous_ctbr, sample.elapsed_physics_ticks)
            for destination, values in zip(reset, fields, strict=True):
                destination[selected] = values

        self.start_segment[ids] = segment
        self.start_standoff_m[ids] = (position - self.center) @ self.normal
        self.start_bucket[ids] = torch.where(segment == final, 2, (segment > 0).long())
        is_frontier = segment == self.frontier_index
        self.is_frontier[ids] = is_frontier
        self.start_frontier_index[ids] = torch.where(is_frontier, self.frontier_index, -1)
        return reset

    def metrics(self, state) -> Stage1CMetrics:
        base = super().metrics(state)
        position, quaternion, velocity, omega_body = state
        point_arm = self.center + base.bumper_delta_m - position
        point_velocity = velocity + torch.linalg.cross(quat_apply(quaternion, omega_body), point_arm, dim=-1)
        inward = -(point_velocity @ self.normal)
        tangent = torch.linalg.vector_norm(point_velocity + inward[:, None] * self.normal, dim=-1)
        speed_error = (self._inward_low - inward).clamp_min(0.) + (inward - self._inward_high).clamp_min(0.)
        speed_error = speed_error.square() + (tangent / self._max_tangent - 1.).clamp_min(0.).square()
        footprint = (base.bumper_delta_m @ self.frame[:, :2]).abs() / self._halfsize
        edge = (footprint.amax(dim=-1) - 1.).clamp_min(0.).square()
        tilt = ((1. - base.alignment) / self._capture_angle_scale - 1.).clamp_min(0.)
        r = self.pilot_reward
        coarse = r["progress_weight"] / (1. + (base.distance_m / r["progress_scale_m"]).square())
        local = r["capture_weight"] * torch.exp(-base.gap_m.clamp_min(0.) / r["capture_gap_scale_m"]
                                                - edge - tilt - speed_error)
        return Stage1CMetrics(base.distance_m, base.body_rate_rad_s, base.unsafe, coarse + local)

    def interval_reward(self, before, after, command, previous_command, saturated, clipped, done, success):
        r = self.pilot_reward
        rate = (after.body_rate_rad_s / self._contact_rate).square().clamp_max(1.)
        change = ((command - previous_command) / self._command_scale).square().mean(dim=-1).clamp_max(1.)
        return -self.physics_dt_s * (r["time_cost_per_s"] + r["body_rate_cost_per_s"] * rate
                                     + r["command_smoothness_cost_per_s"] * change
                                     + r["saturation_cost_per_s"] * (saturated | clipped))

    def policy_reward(self, before, after, running_reward, done, success):
        """Discount the bounded potential on the same 5 ms clock as PPO."""
        r = self.pilot_reward
        next_potential = after.potential.masked_fill(done, 0.)
        event = torch.where(success, r["event_success"], r["event_failure"])
        return running_reward + event * done + self.gamma * next_potential - before.potential
