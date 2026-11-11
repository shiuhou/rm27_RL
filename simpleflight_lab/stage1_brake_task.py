"""Experimental braking guidance using verified Stage 1 handover snapshots."""

import math
from typing import NamedTuple

import numpy as np
import torch
from isaaclab.utils.math import quat_apply
from scipy.spatial.transform import Rotation

from .stage1c_resets import Stage1CStarts, WITNESS_TAPES
from .stage1c_task import Stage1CReset, Stage1CTask
from .stage1_task import _rotvec_quaternion


FRONTIER_BANDS_M = ((.008, .065), (.045, .150), (.115, .220),
                    (.180, .280), (.240, .340), (.300, .420))
SOURCE_STANDOFFS_M = tuple(standoff for standoff, _ in WITNESS_TAPES.values())
BRAKING_REWARD = {"terminal_speed_m_s": .85, "decel_m_s2": 4.5,
                  "overspeed_scale_m_s": .25, "cost_per_s": .40}
_STATIONARY_FRACTION = .25
_RETENTION_FRACTION = .25


class Stage1BrakeStarts(Stage1CStarts):
    """Group the existing verified pre-contact bank by band and witness."""

    @classmethod
    def from_witnesses(cls, handover_dir, geometry):
        verified = super().from_witnesses(handover_dir)
        states, ticks = zip(*verified._rows.values(), strict=True)
        states = np.concatenate(states)
        ticks = np.concatenate(ticks)
        target = geometry["target"]
        normal = np.asarray(target["normal"])
        center = np.asarray(target["center_m"])
        sphere = next(g for g in geometry["aircraft"] if g["name"] == "drone_1_stage1_contact")
        rotation = Rotation.from_quat(states[:, [4, 5, 6, 3]])
        arm = rotation.apply(sphere["position_m"])
        gap = (states[:, :3] + arm - center) @ normal - sphere["size"][0]
        # The spherical surface point's additional -radius * normal arm
        # contributes only tangential velocity, not normal approach speed.
        inward = -(states[:, 7:10] + np.cross(rotation.apply(states[:, 10:13]), arm)) @ normal
        grouped = {}
        for band, (low, high) in enumerate(FRONTIER_BANDS_M):
            for source, standoff in enumerate(SOURCE_STANDOFFS_M):
                selected = (gap >= low) & (gap <= high) & (inward > 0.) & (states[:, 25] == standoff)
                if not selected.any():
                    raise ValueError(f"No verified inward resets for band {band} and {standoff:g} m witness")
                grouped[(band, source)] = (states[selected], ticks[selected])
        return cls(grouped, verified._substeps)


class Stage1BrakeMetrics(NamedTuple):
    distance_m: torch.Tensor
    body_rate_rad_s: torch.Tensor
    unsafe: torch.Tensor
    potential: torch.Tensor
    brake_excess_fraction: torch.Tensor


class Stage1BrakeTask(Stage1CTask):
    """Retain the state actor and scorer while testing earlier braking."""

    num_frontiers = len(FRONTIER_BANDS_M)

    def __init__(self, num_envs, device, geometry, frozen_config, *, braking_config=None, **kwargs):
        super().__init__(num_envs, device, geometry, frozen_config, **kwargs)
        self.start_band = torch.full_like(self.start_frontier_index, -1)
        if braking_config is not None and braking_config.keys() - BRAKING_REWARD.keys():
            raise ValueError("Unknown Stage 1 braking reward setting")
        self.braking_reward = BRAKING_REWARD | (braking_config or {})
        r = self.braking_reward
        if (not all(math.isfinite(value) for value in r.values())
                or not 0 < r["terminal_speed_m_s"] <= self._inward_high
                or min(r["decel_m_s2"], r["overspeed_scale_m_s"]) <= 0
                or r["cost_per_s"] < 0):
            raise ValueError("Invalid Stage 1 braking reward settings")
        discount_duration = self.horizon_s if self.gamma == 1 else (
            self.policy_dt_s * (1. - self.gamma**self.horizon_ticks) / (1. - self.gamma))
        max_cost = sum(value for key, value in self.pilot_reward.items() if key.endswith("_cost_per_s"))
        max_potential = self.pilot_reward["progress_weight"] + self.pilot_reward["capture_weight"]
        if (self.gamma ** (self.horizon_ticks - 1) * self.pilot_reward["event_success"]
                <= discount_duration * (max_cost + r["cost_per_s"]) + max_potential):
            raise ValueError("Braking cost and discount no longer guarantee positive qualified return")

    def reset(self, ids, progress=0., *, perturbations=None) -> Stage1CReset:
        n = len(ids)
        if not self.training:
            reset = super().reset(ids, progress, perturbations=perturbations)
            self.start_band[ids] = -1
            self.start_segment[ids] = 3
            self.is_frontier[ids] = False
            self.start_frontier_index[ids] = -1
            return reset
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
        motors = self.hover_ctbr[0].expand(n, 4).clone() * .25
        motor_command = motors.clone()
        previous_command = self.hover_ctbr.expand(n, 4).clone()
        ticks = torch.zeros_like(ids)
        reset = Stage1CReset(position, quaternion, velocity, omega, motors, motor_command, previous_command, ticks)

        frontier = self.frontier_index
        cohort = draw[:, 0]
        stationary = cohort < _STATIONARY_FRACTION
        bands = torch.where(stationary, -1, frontier)
        if frontier:
            retain = (~stationary) & (cohort < _STATIONARY_FRACTION + _RETENTION_FRACTION)
            bands = torch.where(retain, torch.randint(frontier, (n,), device=self.device), bands)
        # Select the witness independently of the band and environment ID.
        # Within a witness, uniform 5 ms rows cover the coupled speed/tilt
        # range without synthesizing poses or velocity labels.
        sources = torch.randint(len(SOURCE_STANDOFFS_M), (n,), device=self.device)
        groups = bands * len(SOURCE_STANDOFFS_M) + sources
        for group in torch.unique(groups[~stationary]).tolist():
            band, source = divmod(group, len(SOURCE_STANDOFFS_M))
            selected = torch.nonzero(groups == group, as_tuple=True)[0]
            sample = self.starts.sample(len(selected), (band, source), self.device)
            if torch.any(sample.control_phase != 0):
                raise ValueError("Braking reset must preserve the native 5 ms control phase")
            fields = (*sample.state, sample.motor_activation_N, sample.motor_command_N,
                      sample.previous_ctbr, sample.elapsed_physics_ticks)
            for destination, values in zip(reset, fields, strict=True):
                destination[selected] = values

        gap, _ = self._bumper_motion(reset[:4])
        segment = torch.where(stationary, 3, (gap > .055).long() + (gap > .35).long())
        fresh = gap > FRONTIER_BANDS_M[frontier - 1][1] if frontier else ~stationary
        current = (bands == frontier) & fresh
        self.start_band[ids] = bands
        self.start_segment[ids] = segment
        self.start_standoff_m[ids] = (position - self.center) @ self.normal
        self.start_bucket[ids] = torch.where(stationary, 2, (segment > 0).long())
        self.is_frontier[ids] = current
        self.start_frontier_index[ids] = torch.where(current, frontier, -1)
        return reset

    def _bumper_motion(self, state):
        position, quaternion, velocity, omega_body = state
        arm = quat_apply(quaternion, self.fixture_offset_b.expand_as(position))
        gap = (position + arm - self.center) @ self.normal - self.fixture_radius_m
        point_velocity = velocity + torch.linalg.cross(quat_apply(quaternion, omega_body), arm, dim=-1)
        return gap, -(point_velocity @ self.normal)

    def metrics(self, state) -> Stage1BrakeMetrics:
        base = super().metrics(state)
        gap, inward = self._bumper_motion(state)
        r = self.braking_reward
        allowed = (r["terminal_speed_m_s"]**2 + 2. * r["decel_m_s2"] * gap.clamp_min(0.)).sqrt()
        excess = ((inward - allowed) / r["overspeed_scale_m_s"]).clamp(0., 1.)
        return Stage1BrakeMetrics(*base, excess)

    def interval_reward(self, before, after, command, previous_command, saturated, clipped, done, success):
        base = super().interval_reward(before, after, command, previous_command, saturated, clipped, done, success)
        # Use the pre-interval state as in the contact gate. The terminal
        # solver impulse must not erase an already excessive approach speed.
        return base - self.physics_dt_s * self.braking_reward["cost_per_s"] * before.brake_excess_fraction
