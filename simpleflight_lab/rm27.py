"""RM27 Ver2 free-flight dynamics and CTBR allocation for the Isaac Lab port."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import torch


RM27_SIM_REVISION = "6ec552b4a836c601295b7657c78c4bd3af977600"
RM27_RL_REVISION = "ed4ebc550800baf032348f67af5699b44e7953f2"
_UPSTREAM = Path(__file__).resolve().parents[1] / "upstream"
_SIM_SOURCE = _UPSTREAM / "rm27_sim"
_RL_SOURCE = _UPSTREAM / "rm27_RL"

# Port-specific exploration envelope. The historical controller used this
# collective fraction and rate scale for its Direct-CTBR candidate.
_TRAIN_RATE_RANGE_RAD_S = 2.0
_TRAIN_COLLECTIVE_FRACTION = 0.5


@dataclass(frozen=True)
class RM27Profile:
    mass_kg: float
    inertia_kg_m2: tuple[float, float, float]
    max_rotor_thrust_N: float
    motor_time_constant_s: float
    yaw_torque_per_thrust_m: float
    rotor_arm_m: float
    rate_gain_per_s: float
    native_rate_limit_rad_s: float
    physics_dt_s: float
    policy_dt_s: float
    outer_width_m: float
    outer_height_m: float
    bumper_offset_body_m: tuple[float, float, float]
    bumper_radius_m: float
    profile_status: str
    profile_sha256: str


def load_rm27_profile() -> RM27Profile:
    """Read the Ver2 sources used by the historical Stage 1 free-flight task."""
    task = json.loads((_RL_SOURCE / "configs/stage1/task_stage1a_seed0.json").read_text())
    # The RM27 repositories were renamed. Resolve the Stage 1 paths under this checkout.
    def sim_source(key: str) -> Path:
        return _SIM_SOURCE / Path(task[key]).relative_to("../rm_uav_mujoco")

    profile_bytes = sim_source("dynamics_profile").read_bytes()
    profile = json.loads(profile_bytes)
    values = profile["values"]
    vehicle = json.loads(sim_source("vehicle_config").read_text())
    fixture = json.loads(sim_source("contact_fixture_config").read_text())["contact"]
    if profile["frame"] != "body axes; principal inertia; COM at model origin":
        raise ValueError("RM27 port requires principal inertias and COM at the origin")
    if values["mass_kg"] != vehicle["mass_kg"]:
        raise ValueError("RM27 Ver2 vehicle and profile disagree on mass")
    return RM27Profile(
        mass_kg=values["mass_kg"],
        inertia_kg_m2=tuple(values["inertia_kg_m2"]),
        max_rotor_thrust_N=values["max_rotor_thrust_N"],
        motor_time_constant_s=values["motor_time_constant_s"],
        yaw_torque_per_thrust_m=values["yaw_torque_per_thrust_m"],
        rotor_arm_m=vehicle["rotor_center_offset_m"],
        rate_gain_per_s=task["rate_gain"],
        native_rate_limit_rad_s=task["action"]["body_rate_limit_rad_s"],
        physics_dt_s=task["physics_dt_s"],
        policy_dt_s=task["policy_dt_s"],
        outer_width_m=vehicle["outer_width_m"],
        outer_height_m=vehicle["outer_height_m"],
        bumper_offset_body_m=tuple(fixture["offset_body_m"]),
        bumper_radius_m=fixture["radius_m"],
        profile_status=profile["status"],
        profile_sha256=hashlib.sha256(profile_bytes).hexdigest(),
    )


class RM27Actuator:
    """Stage 1 CTBR proxy, held motor commands, and per-tick lagged body wrench.

    Physical CTBR order is [collective N, p, q, r rad/s] in FLU body axes.
    The body-to-world attitude is separate. MuJoCo applies the current filtered
    thrust at each tick, then advances its actuator state with filterexact.
    """

    hover_logit = 0.0

    def __init__(self, profile: RM27Profile, num_envs: int, device: str, gravity_m_s2: float = 9.81):
        self.hover_force_N = profile.mass_kg * gravity_m_s2
        self._max_rotor_N = profile.max_rotor_thrust_N
        self._native_rate_limit = profile.native_rate_limit_rad_s
        self._inertia = torch.tensor(profile.inertia_kg_m2, dtype=torch.float32, device=device)
        self._rate_gain = profile.rate_gain_per_s
        self._alpha = -math.expm1(-profile.physics_dt_s / profile.motor_time_constant_s)
        a, k = profile.rotor_arm_m, profile.yaw_torque_per_thrust_m
        # RM27 motor_mixer(): hubs (+,+), (+,-), (-,-), (-,+), unlike Crazyflie.
        mixer = torch.tensor(
            [[1, 1, 1, 1], [a, -a, -a, a], [-a, -a, a, a], [k, -k, k, -k]],
            dtype=torch.float64,
        )
        self._mixer_t = mixer.T.contiguous().to(device=device, dtype=torch.float32)
        self._inverse_mixer_t = torch.linalg.inv(mixer).T.contiguous().to(device=device, dtype=torch.float32)
        self.motor_thrust_N = torch.full((num_envs, 4), self.hover_force_N / 4.0, device=device)
        self.motor_command_N = self.motor_thrust_N.clone()
        self.mixer_saturated = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.force_b = torch.zeros((num_envs, 3), device=device)

    def decode_command(self, squashed_action: torch.Tensor) -> torch.Tensor:
        """Convert SimpleFlight actor order [p,q,r,T] to native RM27 physical units."""
        thrust = self.hover_force_N * (1.0 + _TRAIN_COLLECTIVE_FRACTION * squashed_action[:, 3:])
        rates = squashed_action[:, :3] * _TRAIN_RATE_RANGE_RAD_S
        return torch.cat((thrust, rates), dim=-1)

    def reset(self, env_ids: torch.Tensor) -> None:
        self.motor_thrust_N[env_ids] = self.hover_force_N / 4.0
        self.motor_command_N[env_ids] = self.hover_force_N / 4.0
        self.mixer_saturated[env_ids] = False

    def set_ctbr(self, command: torch.Tensor, body_rates_rad_s: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Allocate a physical CTBR command once per 5 ms policy tick."""
        collective = command[:, :1].clamp(0.0, 4.0 * self._max_rotor_N)
        rate_setpoint = command[:, 1:].clamp(-self._native_rate_limit, self._native_rate_limit)
        torque = self._inertia * self._rate_gain * (rate_setpoint - body_rates_rad_s)
        torque = torque + torch.linalg.cross(body_rates_rad_s, self._inertia * body_rates_rad_s, dim=-1)
        requested = torch.cat((collective, torque), dim=-1) @ self._inverse_mixer_t
        self.mixer_saturated.copy_(((requested < 0.0) | (requested > self._max_rotor_N)).any(dim=-1))
        self.motor_command_N.copy_(requested.clamp(0.0, self._max_rotor_N))
        return self.motor_command_N, self.mixer_saturated

    def step_wrench(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Return the wrench for this 1 ms interval and advance the actuator lag."""
        wrench = self.motor_thrust_N @ self._mixer_t
        self.motor_thrust_N.lerp_(self.motor_command_N, self._alpha)
        self.force_b[:, 2] = wrench[:, 0]
        return self.force_b, wrench[:, 1:].contiguous()
