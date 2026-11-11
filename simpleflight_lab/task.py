# SPDX-License-Identifier: MIT
# Adapted from upstream/SimpleFlight. See its LICENSE for the MIT terms
# and copyright notice for Botian Xu, Tsinghua University (2023).

"""Hover and Track task contracts adapted from the pinned SimpleFlight checkout.

See upstream/SimpleFlight/LICENSE and omni_drones/envs/single/{hover,track}.py
in that checkout. Positions are relative to each environment origin but use
world axes. Velocities are in m/s and rad/s, and quaternions are wxyz.
"""

import math

import torch
from isaaclab.utils.math import matrix_from_quat, quat_from_euler_xyz


_LOOKAHEAD_POINTS = 10
_LOOKAHEAD_SPACING_S = 0.05
_TIME_FEATURES = 4
_EPISODE_DURATION_S = {"hover": 5.0, "track": 10.0}
_EVAL_PROFILES = {"hover": ("standard", "stress"), "track": ("standard", "position", "normal")}
_TRACK_POLY_DURATION_S = (1.5, 4.0)
_TRACK_ZIGZAG_DURATION_S = (1.0, 1.5)


def observation_dim(mode: str) -> int:
    """Width of the actor input. The critic appends four time features."""
    return {"hover": 15, "track": 12 + 3 * _LOOKAHEAD_POINTS}[mode]


def episode_duration_s(mode: str) -> float:
    return _EPISODE_DURATION_S[mode]


def track_hold_observation(hover_observation: torch.Tensor) -> torch.Tensor:
    """Supply the same stationary displacement at every Track lookahead time."""
    offsets = hover_observation[:, None, :3].expand(-1, _LOOKAHEAD_POINTS, -1)
    return torch.cat((offsets.flatten(1), hover_observation[:, 3:]), dim=-1)


class FlightTask:
    def __init__(
        self,
        mode: str,
        num_envs: int,
        device: torch.device | str,
        step_dt: float,
        *,
        evaluation: bool = False,
        eval_profile: str = "standard",
        rm27: bool = False,
        legacy_flight=None,
    ):
        legacy = evaluation and eval_profile == "legacy" and legacy_flight is not None
        self.precision = rm27 and mode == "track" and not evaluation and eval_profile == "precision"
        if mode not in _EVAL_PROFILES or (eval_profile not in _EVAL_PROFILES[mode] and not (legacy or self.precision)):
            raise ValueError(f"Unsupported task/profile: {mode!r}/{eval_profile!r}")
        self.mode = mode
        self.device = device
        self.step_dt = step_dt
        self.evaluation = evaluation
        self.eval_profile = eval_profile
        horizon_s = legacy_flight.horizon_s if legacy else episode_duration_s(mode)
        self.horizon_ticks = round(horizon_s / step_dt)
        self.origin = torch.tensor([0.0, 0.0, 1.0], device=device)
        self.heading = torch.tensor([1.0, 0.0, 0.0], device=device)
        self.working_hover = None
        if rm27 and mode == "hover" and not evaluation:
            from .rm27_legacy import load_working_hover

            self.working_hover = load_working_hover()
        if legacy:
            self.legacy_position = self.origin + torch.as_tensor(legacy_flight.position, dtype=torch.float32, device=device)
            self.legacy_quat = torch.as_tensor(legacy_flight.quat, dtype=torch.float32, device=device)
            self.legacy_velocity = torch.as_tensor(legacy_flight.velocity, dtype=torch.float32, device=device)
            self.position_targets = self.origin + torch.as_tensor(legacy_flight.target, dtype=torch.float32, device=device)

        if mode == "hover":
            if eval_profile == "stress":
                pos_low, pos_high = [-1.0, -1.0, 0.05], [1.0, 1.0, 2.0]
                rpy_low = [-0.2 * math.pi, -0.2 * math.pi, 0.0]
                rpy_high = [0.2 * math.pi, 0.2 * math.pi, 0.5 * math.pi]
            else:
                # The bounded port uses recoverable initial errors. Independent
                # standard evaluation draws fresh samples from this envelope.
                pos_low, pos_high = [-0.25, -0.25, 0.75], [0.25, 0.25, 1.25]
                rpy_low, rpy_high = [-0.12, -0.12, -0.35], [0.12, 0.12, 0.35]
            self.pos_low = torch.tensor(pos_low, device=device)
            self.pos_span = torch.tensor(pos_high, device=device) - self.pos_low
        else:
            if rm27:
                rpy_low, rpy_high = [-0.12, -0.12, -0.35], [0.12, 0.12, 0.35]
            else:
                rpy_low = [-0.2 * math.pi, -0.2 * math.pi, 0.0]
                rpy_high = [0.2 * math.pi, 0.2 * math.pi, 2.0 * math.pi]
            lookahead_stride = round(_LOOKAHEAD_SPACING_S / step_dt)
            self.lookahead = torch.arange(_LOOKAHEAD_POINTS, device=device) * lookahead_stride
            if not evaluation:
                horizon = horizon_s + (_LOOKAHEAD_POINTS - 1) * _LOOKAHEAD_SPACING_S
                # Upstream generates 100 polynomial and 50 zigzag segments
                # and resamples the whole batch even for a partial reset.
                poly_segments = math.ceil(horizon / _TRACK_POLY_DURATION_S[0])
                zigzag_segments = math.ceil(horizon / _TRACK_ZIGZAG_DURATION_S[0])
                self.poly_times = torch.zeros(num_envs, 2, poly_segments + 1, device=device)
                self.poly_coeffs = torch.zeros(num_envs, 2, poly_segments, 6, device=device)
                self.zigzag_times = torch.zeros(num_envs, 2, zigzag_segments + 1, device=device)
                self.zigzag_points = torch.zeros_like(self.zigzag_times)
                self.zigzag_style = torch.zeros(num_envs, 1, 1, device=device, dtype=torch.bool)
                self.zigzag_sign = 1 - 2 * (torch.arange(zigzag_segments, device=device) % 2)
                if self.precision:
                    self.static_track = torch.zeros(num_envs, 1, 1, device=device, dtype=torch.bool)
                    self.position_targets = self.origin.expand(num_envs, 3).clone()
            elif eval_profile == "position":
                self.position_targets = self.origin.expand(num_envs, 3).clone()

        self.rpy_low = torch.tensor(rpy_low, device=device)
        self.rpy_span = torch.tensor(rpy_high, device=device) - self.rpy_low

    def reset(self, env_ids: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Reset references and return batched pose and zero world velocity."""
        if self.eval_profile == "legacy":
            return self.legacy_position[env_ids], self.legacy_quat[env_ids], self.legacy_velocity[env_ids]
        count = env_ids.numel()
        if self.mode == "hover":
            pos = self.pos_low + torch.rand(count, 3, device=self.device) * self.pos_span
        else:
            pos = self.origin.expand(count, 3)
            if not self.evaluation:
                self._reset_training_references(env_ids)
                if self.precision:
                    static = torch.rand(count, 1, device=self.device) < 0.5
                    # Sample continuous shifts, including both small steps and
                    # the larger standard position suite's flight envelope.
                    offsets = torch.rand(count, 3, device=self.device) - 0.5
                    offsets[:, 2] *= 0.4
                    self.position_targets[env_ids] = self.origin + offsets
                    self.static_track[env_ids] = static[:, None, :]
                    near = static & (torch.rand(count, 1, device=self.device) < 0.5)
                    near_pos = self.position_targets[env_ids] + 0.08 * (torch.rand_like(offsets) - 0.5)
                    pos = torch.where(near, near_pos, pos)
            elif self.eval_profile == "position":
                # Force a distinct horizontal setpoint in every trial.
                sample = torch.rand(count, 3, device=self.device)
                offset_xy = sample[:, :2] - 0.5
                offset_xy = (0.2 + 0.6 * offset_xy.abs()) * torch.where(offset_xy < 0.0, -1.0, 1.0)
                offsets = torch.cat((offset_xy, sample[:, 2:] * 0.4 - 0.2), dim=1)
                self.position_targets[env_ids] = pos + offsets

        if self.mode == "track" and self.evaluation:
            if self.eval_profile == "standard":
                # Keep the upstream moving reference, but evaluate recovery from
                # independent initial position and attitude errors around it.
                pos = pos + 0.24 * (torch.rand(count, 3, device=self.device) - 0.5)
                rpy = 0.24 * (torch.rand(count, 3, device=self.device) - 0.5)
            else:
                rpy = torch.zeros(count, 3, device=self.device)
        else:
            rpy = self.rpy_low + torch.rand(count, 3, device=self.device) * self.rpy_span
            if self.precision:
                rpy = torch.where(near, 0.25 * rpy, rpy)
        quat = quat_from_euler_xyz(*rpy.unbind(dim=-1))
        return pos, quat, torch.zeros(count, 6, device=self.device)

    def _reset_training_references(self, env_ids: torch.Tensor) -> None:
        """Sample upstream ChainedPolynomial and RandomZigzag for reset rows."""
        count = env_ids.numel()
        poly_segments = self.poly_coeffs.shape[-2]
        min_dt, max_dt = _TRACK_POLY_DURATION_S
        dt = min_dt + (max_dt - min_dt) * torch.rand(count, 2, poly_segments, device=self.device)
        self.poly_times[env_ids, :, 1:] = dt.cumsum(dim=-1)
        # ChainedPolynomial constrains position to zero at all knots and
        # samples independent velocity and acceleration in [-2.5, 2.5].
        derivatives = torch.rand(count, 2, poly_segments + 1, 2, device=self.device) * 5.0 - 2.5
        velocity, acceleration = derivatives.unbind(dim=-1)
        # Quintic Hermite interpolation at u=t/dt. Closed form of upstream's
        # endpoint derivative solve, vectorized over axes and segments.
        v0, v1 = velocity[..., :-1] * dt, velocity[..., 1:] * dt
        a0, a1 = acceleration[..., :-1] * dt.square(), acceleration[..., 1:] * dt.square()
        c3 = -6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1
        c4 = 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1
        c5 = -3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1
        self.poly_coeffs[env_ids] = torch.stack(
            (c5, c4, c3, 0.5 * a0, v0, torch.zeros_like(v0)), dim=-1
        )

        zigzag_segments = self.zigzag_sign.numel()
        min_dt, max_dt = _TRACK_ZIGZAG_DURATION_S
        shape = (count, 2, zigzag_segments)
        dt = min_dt + (max_dt - min_dt) * torch.rand(shape, device=self.device)
        self.zigzag_times[env_ids, :, 1:] = dt.cumsum(dim=-1)
        self.zigzag_points[env_ids, :, 1:] = torch.rand(shape, device=self.device) * self.zigzag_sign
        self.zigzag_style[env_ids] = torch.rand(count, 1, 1, device=self.device) < 0.5

    def reference(self, steps: torch.Tensor, *, lookahead: bool = True) -> torch.Tensor:
        """Return local world-axis positions at current and optional future ticks."""
        if self.mode == "hover":
            return self.origin.expand(steps.shape[0], 1, 3)

        num_points = _LOOKAHEAD_POINTS if lookahead else 1
        if self.evaluation and self.eval_profile in ("position", "legacy"):
            return self.position_targets[:, None, :].expand(-1, num_points, -1)

        t = (steps[:, None] + self.lookahead[:num_points]) * self.step_dt
        if self.evaluation:
            # Upstream Lemniscate(T) with a T/4 phase offset.
            period_s = 5.5 if self.eval_profile == "normal" else 15.0
            phase = (2.0 * math.pi) * (t + period_s / 4.0) / period_s
            x = phase.cos()
            return torch.stack((x, phase.sin() * x, torch.ones_like(x)), dim=-1)

        t = t[:, None, :].expand(-1, 2, -1).contiguous()
        poly_segment, u = self._segment_time(self.poly_times, t)
        coeffs = self.poly_coeffs.gather(2, poly_segment[..., None].expand(-1, -1, -1, 6))
        polynomial = coeffs[..., 0]
        for c in coeffs.unbind(dim=-1)[1:]:
            polynomial = polynomial * u + c

        zigzag_segment, u = self._segment_time(self.zigzag_times, t)
        start = self.zigzag_points.gather(2, zigzag_segment)
        end = self.zigzag_points.gather(2, zigzag_segment + 1)
        xy = torch.where(self.zigzag_style, torch.lerp(start, end, u), polynomial).transpose(1, 2)
        moving = torch.cat((xy, torch.ones_like(xy[..., :1])), dim=-1)
        if self.precision:
            return torch.where(self.static_track, self.position_targets[:, None, :], moving)
        return moving

    @staticmethod
    def _segment_time(knots: torch.Tensor, t: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        segment = (torch.searchsorted(knots, t, right=True) - 1).clamp(0, knots.shape[-1] - 2)
        left = knots.gather(2, segment)
        right = knots.gather(2, segment + 1)
        return segment, (t - left) / (right - left)

    def observe(
        self,
        pos_local: torch.Tensor,
        quat_wxyz: torch.Tensor,
        lin_vel_w: torch.Tensor,
        steps: torch.Tensor,
        body_rate: torch.Tensor | None = None,
    ) -> dict[str, torch.Tensor]:
        offsets = self.reference(steps) - pos_local[:, None, :]
        # Columns are heading, lateral and up. Concatenate each whole axis.
        axes_w = matrix_from_quat(quat_wxyz).transpose(-2, -1).flatten(1)
        policy = torch.cat((offsets.flatten(1), lin_vel_w, axes_w), dim=-1)
        if body_rate is not None:
            policy = torch.cat((policy, body_rate), dim=-1)
        time = (steps / self.horizon_ticks)[:, None].expand(-1, _TIME_FEATURES)
        return {"policy": policy, "critic": torch.cat((policy, time), dim=-1)}

    def outcome(
        self,
        pos_local: torch.Tensor,
        quat_wxyz: torch.Tensor,
        lin_vel_w: torch.Tensor,
        ang_vel_b: torch.Tensor,
        steps: torch.Tensor,
        action_delta: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return reward, failure, and position error in meters before reset."""
        reference = self.reference(steps, lookahead=False)[:, 0]
        delta = reference - pos_local
        distance = torch.linalg.vector_norm(delta, dim=-1)
        rotation = matrix_from_quat(quat_wxyz)
        up_z = rotation[:, 2, 2]
        failure = (
            ~torch.isfinite(distance)
            | ~torch.isfinite(quat_wxyz).all(dim=-1)
            | ~torch.isfinite(lin_vel_w).all(dim=-1)
            | ~torch.isfinite(ang_vel_b).all(dim=-1)
        )

        if self.working_hover is not None:
            # The historical RM27 running reward bounds escaped-flight gradients.
            # A failed flight loses future positive reward and pays the source cost.
            r, safety = self.working_hover
            speed = lin_vel_w.norm(dim=-1)
            rate = ang_vel_b.norm(dim=-1)
            tilt = up_z.clamp(-1.0, 1.0).acos()
            proximity = 0.5 / (1.0 + (distance / r["position_scale_m"]).square())
            proximity += 0.5 * (-(distance / r["fine_position_scale_m"]).square()).exp()
            stability = ((-(speed / r["speed_scale_m_s"]).square()).exp()
                         + (-(tilt / r["tilt_scale_rad"]).square()).exp()
                         + (-(rate / r["rate_scale_rad_s"]).square()).exp()) / 3.0
            reward = self.step_dt * r["gain_per_s"] * (0.6 * proximity + 0.3 * stability + 0.1)
            # Use the local free-flight boundary shared with Track and the native
            # working task's speed and rate limits. The port has no arena target.
            failure |= ((pos_local[:, 2] <= 0.1) | (distance > 1.0)
                        | (speed > safety["max_speed_m_s"]) | (rate > safety["max_body_rate_rad_s"]))
            reward = torch.where(failure, reward.nan_to_num() + r["event_failure"], reward)
        elif self.mode == "hover":
            near_target = distance <= 0.02
            heading_error = torch.linalg.vector_norm(self.heading - rotation[:, :, 0], dim=-1)
            heading_bonus = near_target & (heading_error <= 0.02)
            reward = (
                -10.0 * distance
                + 10.0 * near_target
                - heading_error * near_target
                + 10.0 * heading_bonus
                + ((up_z + 1.0) * 0.5).square()
            )
        else:
            position_reward = 5.0 * (-distance).exp()
            upright_reward = 0.5 / (1.0 + (1.0 - up_z).square())
            spin_reward = 0.5 / (1.0 + ang_vel_b[:, 2].square().square())
            smoothness_reward = 2.0 * (-action_delta).exp() * (steps > 1)
            reward = position_reward * (1.0 + upright_reward + spin_reward) + smoothness_reward
            failure = failure | (pos_local[:, 2] < 0.1) | (distance > 1.0)
        if self.precision:
            future = self.reference(steps + self.lookahead[1], lookahead=False)[:, 0]
            velocity_error = lin_vel_w - (future - reference) / _LOOKAHEAD_SPACING_S
            broad = (-distance / 0.20).exp()
            fine = (-distance / 0.05).exp()
            brake = (-velocity_error.norm(dim=-1) / 0.15).exp()
            precision_reward = 0.35 * broad + 0.50 * fine + 0.15 * broad * brake
            # Keep the original reward and full failure event at the boundary.
            reward = torch.where(failure, reward, 0.25 * reward + 0.75 * 12.0 * precision_reward)
        return reward, failure, distance
