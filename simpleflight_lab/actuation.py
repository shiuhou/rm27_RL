# SPDX-License-Identifier: MIT
# Rate loop, PWM mixer, and rotor model adapted from upstream/SimpleFlight
# (Copyright (c) 2023 Botian Xu, Tsinghua University; see its LICENSE).

"""Crazyflie Direct-CTBR action, rate loop, and rotor wrench."""

import math
from pathlib import Path

import torch
import yaml


POLICY_DT = 0.01  # cfg/base/sim_base.yaml: one physics step per policy step
_PWM_FULL_SCALE = 2**16
_UPSTREAM_ASSETS = (
    Path(__file__).resolve().parents[1] / "upstream" / "SimpleFlight"
    / "omni_drones" / "robots" / "assets" / "usd"
)

# From cf2x_pybullet.usd, in the order of rotor_0 through rotor_3.
# The YAML arm length is not used by upstream's PIDrate mixer.
_HUB_XY_M = ((0.028, 0.028), (-0.028, 0.028), (-0.028, -0.028), (0.028, -0.028))
_ROTOR_LINK_MASS_KG = 0.0001


def load_crazyflie_params() -> dict:
    with (_UPSTREAM_ASSETS / "crazyflie.yaml").open() as stream:
        return yaml.safe_load(stream)


def rigid_body_properties(params: dict) -> tuple[float, tuple[float, float, float]]:
    """Collapse the upstream base and rotor links about their common center of mass."""
    mass = params["mass"] + len(_HUB_XY_M) * _ROTOR_LINK_MASS_KG
    inertia = params["inertia"]
    # Parallel-axis contributions of the four symmetric rotor link masses.
    i_x = inertia["xx"] + _ROTOR_LINK_MASS_KG * sum(y * y for _, y in _HUB_XY_M)
    i_y = inertia["yy"] + _ROTOR_LINK_MASS_KG * sum(x * x for x, _ in _HUB_XY_M)
    i_z = inertia["zz"] + _ROTOR_LINK_MASS_KG * sum(x * x + y * y for x, y in _HUB_XY_M)
    return mass, (i_x, i_y, i_z)


class CTBRActuator:
    """Update the upstream motor model once per step, from measured body rates.

    The policy supplies Gaussian logits in [roll, pitch, yaw, thrust] order.
    The transform maps the first three to deg/s and the fourth to collective PWM.
    Outputs are a local-frame force (N) and torque (N m) about the body COM.
    """

    def __init__(self, params: dict, num_envs: int, device: str, gravity_m_s2: float):
        def tensor(values):
            return torch.tensor(values, device=device, dtype=torch.float32)

        rotor = params["rotor_configuration"]
        omega_sq = tensor(rotor["max_rotation_velocities"]).square()
        self._kf = tensor(rotor["force_constants"]) * omega_sq
        km = tensor(rotor["moment_constants"]) * omega_sq
        hub_x, hub_y = tensor(_HUB_XY_M).unbind(dim=-1)
        self._moment_map = torch.stack((hub_y * self._kf, -hub_x * self._kf, -km * tensor(rotor["directions"])), dim=-1)
        self._motor_alpha = POLICY_DT / rotor["time_constant"]
        self._max_rate_deg_s = 180.0 * params["target_clip"]
        self._min_thrust_ratio = params["min_thrust_ratio"]
        self._max_thrust_ratio = params["max_thrust_ratio"]
        self._hover_ratio = rigid_body_properties(params)[0] * gravity_m_s2 / self._kf.sum().item()
        self.hover_logit = math.atanh(2.0 * self._hover_ratio - 1.0)

        # PIDRateController in omni_drones/controllers/lee_position_controller.py.
        self._kp = tensor((250.0, 250.0, 120.0))
        self._ki = tensor((500.0, 500.0, 16.7))
        self._kd_over_dt = tensor((2.5, 2.5, 0.0)) / POLICY_DT
        self._integral_limit = tensor((33.3, 33.3, 166.7))
        self._pwm_limit = 2**15 - 1.0
        self._mixer = tensor((
            (0.5, 0.5, -0.5, -0.5),
            (-0.5, 0.5, 0.5, -0.5),
            (1.0, -1.0, 1.0, -1.0),
        ))
        self._integral = torch.zeros((num_envs, 3), device=device)
        self._prev_rate_deg_s = torch.zeros_like(self._integral)
        self._throttle = torch.full((num_envs, 4), math.sqrt(self._hover_ratio), device=device)
        self.force_b = torch.zeros_like(self._integral)

    def reset(self, env_ids: torch.Tensor):
        self._integral[env_ids] = 0.0
        self._prev_rate_deg_s[env_ids] = 0.0
        self._throttle[env_ids] = math.sqrt(self._hover_ratio)

    def update(self, command: torch.Tensor, body_rate_rad_s: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Convert a tanh-squashed CTBR command to the current body wrench."""
        body_rate_deg_s = torch.rad2deg(body_rate_rad_s)
        rate_error = command[:, :3] * self._max_rate_deg_s - body_rate_deg_s
        self._integral.add_(rate_error, alpha=POLICY_DT).clamp_(-self._integral_limit, self._integral_limit)
        derivative = (self._prev_rate_deg_s - body_rate_deg_s) * self._kd_over_dt
        output = rate_error * self._kp + self._integral * self._ki + derivative
        output = torch.nan_to_num(output, nan=0.0).clamp_(-self._pwm_limit, self._pwm_limit)
        self._prev_rate_deg_s = body_rate_deg_s

        collective_ratio = ((command[:, 3:] + 1.0) * 0.5).clamp_(self._min_thrust_ratio, self._max_thrust_ratio)
        motor_ratio = (collective_ratio + (output @ self._mixer) / _PWM_FULL_SCALE).clamp_(0.0, 1.0)
        self._throttle.lerp_(motor_ratio.sqrt_(), self._motor_alpha)
        thrust_fraction = self._throttle.square().clamp_(0.0, 1.0)

        self.force_b[:, 2] = thrust_fraction @ self._kf
        return self.force_b, thrust_fraction @ self._moment_map
