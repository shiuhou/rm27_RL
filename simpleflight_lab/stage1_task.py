"""Batched Stage 1 state, resets, and PPO shaping for the verified RM27 scene.

Aircraft state is world NWU position and velocity, a wxyz FLU-to-world
quaternion, and measured FLU angular velocity. Physics and contact judgement
are owned by the backend. The actor sees only vehicle state and a static target.
"""

import math
from typing import NamedTuple

import torch
from isaaclab.utils.math import axis_angle_from_quat, matrix_from_quat, quat_from_matrix, quat_mul
from torch import nn

from .ppo import Actor
from .rm27 import load_rm27_profile
from .task import track_hold_observation


POLICY_OBS_DIM = 21
CRITIC_OBS_DIM = POLICY_OBS_DIM + 4
START_BUCKETS = ("near", "intermediate", "full")
_ALIGN_THROUGH_M = 0.10
_NEAR_BUCKET_M = 0.12
_FULL_BUCKET_FRACTION = 0.8
_SEED_FRACTION_AFTER_EXPANSION = 0.25
_SEED_POSITION_HALFWIDTH_M = (0.005, 0.005, 0.002)
_FRONTIER_CLEARANCE_M = ((.010, .022), (.022, .060), (.060, .120), (.120, .250), (.250, .500), (.500, .800))
_FRONTIER_FULL_FRACTION = .15
_FRONTIER_SUPPORT_FRACTION = .20
_FRONTIER_ASSIST_SPEED_M_S = .11
_FRONTIER_POTENTIAL_GAP_SCALE_M = .04


class Stage1Actor(Actor):
    """Lift the 45D Track actor without changing its input normalization."""

    def __init__(self, initial_std: float = 1.0):
        super().__init__(45, initial_std)
        self.normal_weights = nn.Linear(3, self.mean[1].out_features, bias=False)
        nn.init.zeros_(self.normal_weights.weight)

    def load_track_state_dict(self, track_state: dict, *, initial_std: float | None = None) -> None:
        """Copy the selected Track actor and initialize only the new normal path."""
        with torch.no_grad():
            self.normal_weights.weight.zero_()
        weights = dict(track_state, **{"normal_weights.weight": self.normal_weights.weight.detach().clone()})
        if initial_std is not None:
            if not math.isfinite(initial_std) or initial_std <= 0:
                raise ValueError("Initial exploration standard deviation must be positive and finite")
            weights["log_std"] = torch.full_like(self.log_std, math.log(initial_std))
        self.load_state_dict(weights, strict=True)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        hold = track_hold_observation(obs[:, :18])
        hidden = self.mean[1](self.mean[0](hold)) + self.normal_weights(obs[:, 18:21])
        return self.mean[2:](hidden)


class Stage1Metrics(NamedTuple):
    distance_m: torch.Tensor
    gap_m: torch.Tensor
    alignment: torch.Tensor
    speed_m_s: torch.Tensor
    body_rate_rad_s: torch.Tensor
    unsafe: torch.Tensor
    bumper_delta_m: torch.Tensor


def _rotvec_quaternion(rotvec: torch.Tensor) -> torch.Tensor:
    angle = torch.linalg.vector_norm(rotvec, dim=-1, keepdim=True)
    return torch.cat(((angle * 0.5).cos(), 0.5 * torch.sinc(angle / (2 * math.pi)) * rotvec), dim=-1)


class Stage1Task:
    """Native task shaping and a bounded training-only start curriculum."""

    def __init__(
        self,
        num_envs: int,
        device: str | torch.device,
        geometry: dict,
        frozen_config: dict,
        *,
        training: bool,
        curriculum: dict | None = None,
        capture_weight: float = 0.0,
        goal_inset_m: float = 0.012,
    ):
        self.device = device
        self.training = training
        self.physics_dt_s = frozen_config["physics_dt_s"]
        self.policy_dt_s = frozen_config["policy_dt_s"]
        self.horizon_s = frozen_config["horizon_s"]
        self.horizon_ticks = round(self.horizon_s / self.policy_dt_s)
        self.reward = frozen_config["reward"]
        self.capture_weight = capture_weight
        if not (math.isfinite(capture_weight) and capture_weight >= 0):
            raise ValueError("Capture weight must be finite and nonnegative")
        if not (math.isfinite(goal_inset_m) and goal_inset_m >= 0):
            raise ValueError("Goal inset must be finite and nonnegative")

        def tensor(value):
            return torch.tensor(value, dtype=torch.float32, device=device)

        target = geometry["target"]
        self.center = tensor(target["center_m"])
        self.normal = tensor(target["normal"])
        self.frame = tensor(target["target_rotation"])
        fixture = next(g for g in geometry["aircraft"] if g["name"] == "drone_1_stage1_contact")
        if fixture["type"] != "sphere" or fixture["size"][0] <= 0:
            raise ValueError("Stage 1 requires its named spherical underside fixture")
        self.fixture_offset_b = tensor(fixture["position_m"])
        self.fixture_radius_m = fixture["size"][0]
        self.bump_goal = self.center + (self.fixture_radius_m - goal_inset_m) * self.normal
        self._aligned_rotvec = axis_angle_from_quat(quat_from_matrix(self.frame[None]))[0]

        init = frozen_config["initial_state"]
        self.full_standoff_m = init["normal_standoff_m"]
        self.initial_halfwidth = tensor(
            [
                *init["position_perturbation_halfwidth_target_frame_m"],
                *init["velocity_perturbation_halfwidth_world_m_s"],
                *(math.radians(x) for x in init["rotation_vector_halfwidth_body_deg"]),
                *init["body_rate_perturbation_halfwidth_rad_s"],
            ]
        )
        safety = frozen_config["safety"]
        self.safety_low = tensor(safety["position_min_target_frame_m"])
        self.safety_high = tensor(safety["position_max_target_frame_m"])
        self.max_speed_m_s = safety["max_speed_m_s"]
        self.max_rate_rad_s = safety["max_body_rate_rad_s"]
        self._capture_angle_scale = 1.0 - math.cos(math.radians(
            frozen_config["contact_acceptance"]["max_bottom_normal_error_deg"]
        ))
        profile = load_rm27_profile()
        self.hover_ctbr = tensor([profile.mass_kg * 9.81, 0.0, 0.0, 0.0])
        self.action_scale = tensor([2 * profile.max_rotor_thrust_N, *([frozen_config["action"]["body_rate_limit_rad_s"]] * 3)])

        self.curriculum = {
            "near_min_m": 0.03,
            "initial_max_m": 0.25,
            "final_max_m": self.full_standoff_m,
            "expand_fraction": 0.65,
            "hover_fraction": 0.15,
            "full_fraction_after_expansion": 0.40,
            "near_inward_speed_m_s": 0.30,
            "seed_fraction": 0.0,
            "seed_clearance_min_m": 0.008,
            "seed_clearance_max_m": 0.025,
            "seed_inward_speed_m_s": 0.22,
        }
        if curriculum is not None:
            unknown = curriculum.keys() - self.curriculum.keys()
            if unknown:
                raise ValueError(f"Unknown Stage 1 curriculum settings: {sorted(unknown)}")
            self.curriculum.update(curriculum)
        c = self.curriculum
        if not (
            0 < c["near_min_m"] <= c["initial_max_m"] <= c["final_max_m"] <= self.full_standoff_m
            and 0 < c["expand_fraction"] <= 1
            and 0 <= c["hover_fraction"] <= c["full_fraction_after_expansion"] < 1
            and 0 <= c["near_inward_speed_m_s"] <= frozen_config["contact_acceptance"]["normal_speed_range_m_s"][1]
            and (c["near_inward_speed_m_s"] == 0 or c["near_min_m"] < _NEAR_BUCKET_M)
            and 0 <= c["seed_fraction"] <= 1
            and _SEED_POSITION_HALFWIDTH_M[2] < c["seed_clearance_min_m"]
            <= c["seed_clearance_max_m"] <= _ALIGN_THROUGH_M
            and 0 < c["seed_inward_speed_m_s"] <= frozen_config["contact_acceptance"]["normal_speed_range_m_s"][1]
        ):
            raise ValueError("Invalid Stage 1 curriculum bounds")
        # Seed offsets use the same uniform draw as the original starts, with
        # a narrower position envelope centered on the audited contact face.
        self._seed_position_scale = tensor(_SEED_POSITION_HALFWIDTH_M) / self.initial_halfwidth[:3]
        self.start_standoff_m = torch.zeros(num_envs, device=device)
        self.start_bucket = torch.zeros(num_envs, dtype=torch.long, device=device)

    def reset(
        self, ids: torch.Tensor, progress: float = 0.0, *, perturbations: torch.Tensor | None = None
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return COM position, attitude, world velocity, and body rate.

        Independent perturbations use the exact archived upright 1 m reset.
        Training short starts blend continuously to it as standoff increases.
        """
        n = ids.numel()
        if perturbations is None:
            perturbations = (2.0 * torch.rand((n, 12), device=self.device) - 1.0) * self.initial_halfwidth
        if not self.training:
            nominal = self.center + self.full_standoff_m * self.normal
            pos = nominal + perturbations[:, :3] @ self.frame.T
            quat = _rotvec_quaternion(perturbations[:, 6:9])
            self.start_standoff_m[ids] = (pos - self.center) @ self.normal
            self.start_bucket[ids] = 2
            return pos, quat, perturbations[:, 3:6], perturbations[:, 9:12]

        c = self.curriculum
        expansion = min(max(progress / c["expand_fraction"], 0.0), 1.0)
        largest = c["initial_max_m"] + expansion * (c["final_max_m"] - c["initial_max_m"])
        full_fraction = c["hover_fraction"] + expansion * (c["full_fraction_after_expansion"] - c["hover_fraction"])
        draw = torch.rand((n, 2), device=self.device)
        full = draw[:, 0] < full_fraction
        standoff = c["near_min_m"] + draw[:, 1] * (largest - c["near_min_m"])
        standoff = torch.where(full, self.full_standoff_m, standoff)
        seed = None
        if c["seed_fraction"]:
            # Fade the extra contact-feasible starts to at most 25% of
            # non-full resets as the ordinary curriculum expands.
            final_seed_fraction = min(c["seed_fraction"], _SEED_FRACTION_AFTER_EXPANSION)
            seed_fraction = c["seed_fraction"] + expansion * (final_seed_fraction - c["seed_fraction"])
            seed = ~full & (torch.rand(n, device=self.device) < seed_fraction)
            seed_clearance = c["seed_clearance_min_m"] + draw[:, 1] * (
                c["seed_clearance_max_m"] - c["seed_clearance_min_m"]
            )
            standoff = torch.where(seed, seed_clearance, standoff)
            seed_position_jitter = perturbations[:, :3] * self._seed_position_scale
        tilt = ((self.full_standoff_m - standoff) / (self.full_standoff_m - _ALIGN_THROUGH_M)).clamp(0.0, 1.0)
        # Use the original jitter at the original start, and reduce near-wall
        # jitter without altering any acceptance tolerance.
        perturbations = perturbations * (1.0 - 0.75 * tilt[:, None])
        nominal_quat = _rotvec_quaternion(tilt[:, None] * self._aligned_rotvec)
        quat = quat_mul(nominal_quat, _rotvec_quaternion(perturbations[:, 6:9]))
        rotation = matrix_from_quat(quat)
        # Near starts specify actual bumper clearance. At 1 m this additional
        # offset vanishes, recovering the archived COM standoff exactly.
        bumper_offset = rotation @ self.fixture_offset_b
        near_com_offset = self.fixture_radius_m * self.normal - bumper_offset
        position_jitter = perturbations[:, :3]
        if seed is not None:
            position_jitter = torch.where(seed[:, None], seed_position_jitter, position_jitter)
        pos = (self.center + standoff[:, None] * self.normal + tilt[:, None] * near_com_offset
               + position_jitter @ self.frame.T)
        self.start_standoff_m[ids] = (pos - self.center) @ self.normal
        self.start_bucket[ids] = (
            (standoff > _NEAR_BUCKET_M).long() + (standoff >= self.full_standoff_m * _FULL_BUCKET_FRACTION).long()
        )
        velocity = perturbations[:, 3:6]
        inward_speed = torch.zeros_like(standoff)
        if c["near_inward_speed_m_s"]:
            # Short aligned starts already have a small inward velocity.
            # It vanishes at 12 cm, leaving the full hover distribution intact.
            fraction = ((_NEAR_BUCKET_M - standoff) / (_NEAR_BUCKET_M - c["near_min_m"])).clamp(0.0, 1.0)
            inward_speed = fraction * c["near_inward_speed_m_s"]
        if seed is not None:
            inward_speed = torch.where(seed, c["seed_inward_speed_m_s"], inward_speed)
        velocity = velocity - inward_speed[:, None] * self.normal
        return pos, quat, velocity, perturbations[:, 9:12]

    def observe(self, state, ticks: torch.Tensor) -> dict[str, torch.Tensor]:
        pos, quat, vel, omega_b = state
        rotation = matrix_from_quat(quat)
        bumper_center = pos + rotation @ self.fixture_offset_b
        axes_world = rotation.transpose(-2, -1).flatten(1)
        policy = torch.cat((self.bump_goal - bumper_center, vel, axes_world, omega_b,
                            self.normal.expand(len(pos), -1)), dim=-1)
        time = (ticks / self.horizon_ticks)[:, None].expand(-1, 4)
        return {"policy": policy, "critic": torch.cat((policy, time), dim=-1)}

    def metrics(self, state) -> Stage1Metrics:
        pos, quat, velocity, omega_b = state
        rotation = matrix_from_quat(quat)
        bump_point = pos + rotation @ self.fixture_offset_b - self.fixture_radius_m * self.normal
        delta = bump_point - self.center
        volume = (pos - self.center) @ self.frame
        distance = torch.linalg.vector_norm(delta, dim=-1)
        gap = delta @ self.normal
        alignment = (rotation[:, :, 2] @ self.normal).clamp(-1.0, 1.0)
        speed = torch.linalg.vector_norm(velocity, dim=-1)
        body_rate = torch.linalg.vector_norm(omega_b, dim=-1)
        unsafe = (
            ~torch.isfinite(distance) | ~torch.isfinite(alignment) | ~torch.isfinite(speed)
            | ~torch.isfinite(body_rate) | ((volume < self.safety_low) | (volume > self.safety_high)).any(dim=-1)
            | (speed > self.max_speed_m_s) | (body_rate > self.max_rate_rad_s)
        )
        return Stage1Metrics(distance, gap, alignment, speed, body_rate, unsafe, delta)

    def terminal_reward(self, done: torch.Tensor, success: torch.Tensor) -> torch.Tensor:
        r = self.reward
        return torch.where(success, r["event_success"], r["event_failure"]) * done

    def interval_reward(
        self,
        before: Stage1Metrics,
        after: Stage1Metrics,
        command: torch.Tensor,
        previous_command: torch.Tensor,
        saturated: torch.Tensor,
        clipped: torch.Tensor,
        done: torch.Tensor,
        success: torch.Tensor,
    ) -> torch.Tensor:
        """Frozen native 1 ms reward plus optional local capture potential."""
        r = self.reward
        dist = after.distance_m
        guide_before = (1.0 + (before.distance_m / r["guide_distance_scale_m"]).square()).reciprocal()
        guide_after = (1.0 + (dist / r["guide_distance_scale_m"]).square()).reciprocal()
        guide = r["guide_gain"] * (guide_after - guide_before)
        near = (-(dist / r["alignment_distance_scale_m"]).square()).exp()
        align = r["near_alignment_gain_per_s"] * near * ((1.0 + after.alignment) * 0.5)
        magnitude = ((command - self.hover_ctbr) / self.action_scale).square().clamp_max(1.0).sum(dim=-1)
        smoothness = ((command - previous_command) / self.action_scale).square().clamp_max(4.0).sum(dim=-1)
        speed = (after.speed_m_s / self.max_speed_m_s).square().clamp_max(1.0)
        rate = (after.body_rate_rad_s / self.max_rate_rad_s).square().clamp_max(1.0)
        cost = (
            r["action_magnitude_cost_per_s"] * magnitude + r["action_smoothness_cost_per_s"] * smoothness
            + r["speed_cost_per_s"] * speed + r["body_rate_cost_per_s"] * rate
            + r["saturation_cost_per_s"] * saturated + r["input_clip_cost_per_s"] * clipped + r["time_cost_per_s"]
        )
        if self.capture_weight:
            # A difference of potentials rewards getting both close and
            # aligned. Holding or retreating gains no sustained bonus.
            potential_before = (-before.distance_m / _ALIGN_THROUGH_M
                                - (1.0 - before.alignment) / self._capture_angle_scale).exp()
            potential_after = (-dist / _ALIGN_THROUGH_M
                               - (1.0 - after.alignment) / self._capture_angle_scale).exp()
            guide = guide + self.capture_weight * (potential_after - potential_before)
        return guide + self.physics_dt_s * (align - cost) + self.terminal_reward(done, success)


class Stage1ResidualTask(Stage1Task):
    """Start from stationary bumper-clearance bands and retain the original full starts."""

    def __init__(self, num_envs, device, geometry, frozen_config, *,
                 frontier_clearance_m=_FRONTIER_CLEARANCE_M, **kwargs):
        super().__init__(num_envs, device, geometry, frozen_config, **kwargs)
        bands = tuple(tuple(band) for band in frontier_clearance_m)
        if (not bands
                or any(not 0 < low < high < self.full_standoff_m for low, high in bands)
                or any(left[1] != right[0] for left, right in zip(bands, bands[1:]))):
            raise ValueError("Stage 1-B requires contiguous, increasing bumper-clearance bands below the full reset")
        self._clearance_bands = self.center.new_tensor(bands)
        self._near_jitter_limit = self.center.new_tensor(_SEED_POSITION_HALFWIDTH_M)
        self._halfsize = self.center.new_tensor(geometry["target"]["halfsize_m"])
        self.num_frontiers = len(bands) + 1
        self.frontier_index = 0
        self.is_frontier = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.start_frontier_index = torch.full((num_envs,), -1, dtype=torch.long, device=device)

    def set_frontier(self, index: int) -> None:
        if not isinstance(index, int) or not 0 <= index < self.num_frontiers:
            raise ValueError(f"Invalid Stage 1-B frontier index: {index}")
        self.frontier_index = index

    def reset(self, ids, progress=0.0, *, perturbations=None):
        """Sample the current promoted band without using time-based progress."""
        if not self.training:
            return super().reset(ids, progress=progress, perturbations=perturbations)
        n = ids.numel()
        if perturbations is None:
            perturbations = (2.0 * torch.rand((n, 12), device=self.device) - 1.0) * self.initial_halfwidth
        draws = torch.rand((n, 3), device=self.device)
        cohort = draws[:, 0]
        full = cohort < _FRONTIER_FULL_FRACTION
        support = (~full) & (cohort < _FRONTIER_FULL_FRACTION + _FRONTIER_SUPPORT_FRACTION)
        full_frontier = self.frontier_index == self.num_frontiers - 1
        if full_frontier:
            full = ~support
        frontier = ~support & (full if full_frontier else ~full)
        self.is_frontier[ids] = frontier
        self.start_frontier_index[ids] = torch.where(frontier, self.frontier_index, -1)

        current = min(self.frontier_index, self.num_frontiers - 2)
        previous = max(0, self.frontier_index - 1)
        band = self._clearance_bands[torch.where(support, previous, current)]
        gap = torch.lerp(band[:, 0], band[:, 1], draws[:, 1])
        # Align short starts to the surface and relax tilt toward upright as
        # clearance grows. The separate full cohort uses the exact archived COM reset.
        tilt = ((self.full_standoff_m - gap) / (self.full_standoff_m - _ALIGN_THROUGH_M)).clamp(0.0, 1.0)
        jitter = perturbations * (1.0 - 0.75 * tilt[:, None])
        limits = torch.lerp(self.initial_halfwidth[:3], self._near_jitter_limit, tilt[:, None])
        position_jitter = jitter[:, :3].clamp(-limits, limits)
        nominal_quat = _rotvec_quaternion(tilt[:, None] * self._aligned_rotvec)
        quat = quat_mul(nominal_quat, _rotvec_quaternion(jitter[:, 6:9]))
        offset_w = matrix_from_quat(quat) @ self.fixture_offset_b
        pos = (self.center + (self.fixture_radius_m + gap[:, None]) * self.normal
               - offset_w + position_jitter @ self.frame.T)
        velocity = jitter[:, 3:6]
        if self.frontier_index == 0:
            # Assistance is optional support; set its normal component within
            # [0, 0.11] m/s inward while preserving tangential reset jitter.
            normal_correction = -_FRONTIER_ASSIST_SPEED_M_S * draws[:, 2] - velocity @ self.normal
            velocity = velocity + (support * normal_correction)[:, None] * self.normal

        nominal_full = self.center + self.full_standoff_m * self.normal
        pos = torch.where(full[:, None], nominal_full + perturbations[:, :3] @ self.frame.T, pos)
        quat = torch.where(full[:, None], _rotvec_quaternion(perturbations[:, 6:9]), quat)
        velocity = torch.where(full[:, None], perturbations[:, 3:6], velocity)
        omega = torch.where(full[:, None], perturbations[:, 9:12], jitter[:, 9:12])
        self.start_standoff_m[ids] = (pos - self.center) @ self.normal
        self.start_bucket[ids] = torch.where(full, 2, (gap > _NEAR_BUCKET_M).long())
        return pos, quat, velocity, omega

    def _capture_potential(self, metrics: Stage1Metrics) -> torch.Tensor:
        tangential = metrics.bumper_delta_m @ self.frame[:, :2]
        edge = (tangential.abs().div(self._halfsize).amax(dim=-1) - 1.0).clamp_min(0.0)
        tilt = ((1.0 - metrics.alignment) / self._capture_angle_scale - 1.0).clamp_min(0.0)
        gap = metrics.gap_m.clamp_min(0.0) / _FRONTIER_POTENTIAL_GAP_SCALE_M
        return torch.exp(-gap - edge.square() - tilt)

    def interval_reward(self, before, after, *args, **kwargs):
        potential_difference = self._capture_potential(after) - self._capture_potential(before)
        return (super().interval_reward(before, after, *args, **kwargs)
                + potential_difference)
