"""Provisional ASM2 rigid-body flight parameters and Direct-CTBR actuation."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
import math

import numpy as np

from .asset import ROOT, digest, read_json


CONFIG = ROOT / "configs/rm27_asm2_flight.json"


@dataclass(frozen=True)
class FlightProfile:
    mass_kg: float
    inertia_kg_m2: np.ndarray
    hubs_m: np.ndarray
    axes: np.ndarray
    yaw_torque_per_thrust_m: np.ndarray
    max_rotor_thrust_N: float
    motor_time_constant_s: float
    rate_gain_per_s: float
    native_rate_limit_rad_s: float
    physics_dt_s: float
    policy_dt_s: float
    gravity_m_s2: float
    mixer: np.ndarray
    inverse_mixer: np.ndarray
    hover_motor_thrust_N: np.ndarray
    profile_sha256: str
    source_pins: dict[str, str]
    provenance: dict[str, str]

    @property
    def hover_thrust_N(self) -> float:
        return self.mass_kg * self.gravity_m_s2


def load_profile() -> FlightProfile:
    """Read hash-pinned CAD properties and explicitly provisional legacy settings."""
    config_bytes = CONFIG.read_bytes()
    config = json.loads(config_bytes)
    if config["schema"] != "rm27_asm2_free_flight_v1":
        raise ValueError("Unknown ASM2 flight profile schema")
    pins = config["source_pins"]
    for field in ("mass_properties", "legacy_dynamics", "legacy_timing"):
        if config[field] not in pins:
            raise ValueError(f"Missing source pin for {field}")
    for source, expected in pins.items():
        if digest(ROOT / source) != expected:
            raise ValueError(f"ASM2 flight source changed: {source}")

    mass = read_json(ROOT / config["mass_properties"])
    dynamics = read_json(ROOT / config["legacy_dynamics"])["values"]
    timing = read_json(ROOT / config["legacy_timing"])
    mass_kg = float(config["mass_kg"])
    inertia = np.asarray(mass["inertia_body_kg_m2"], dtype=np.float64)
    if mass_kg != mass["total_mass_kg"]:
        raise ValueError("ASM2 configured mass differs from the CAD package")
    if (inertia.shape != (3, 3) or not np.isfinite(inertia).all()
            or not np.allclose(inertia, inertia.T, atol=1e-12, rtol=0)
            or np.any(np.linalg.eigvalsh(inertia) <= 0)):
        raise ValueError("Expected a finite, symmetric positive definite ASM2 inertia tensor")

    # The source COM already includes the battery shift. Rotor force passes
    # through each dmotor center in XY. Its application height along +Z has
    # no effect on the arm torque.
    rotor_parts = [mass["parts"][i] for i in config["rotor_part_indices"]]
    axis = np.asarray(config["rotor_thrust_axis_body"], dtype=np.float64)
    if (len(rotor_parts) != 4 or any(part["name"] != "dmotor" for part in rotor_parts)
            or not np.array_equal(axis, [0.0, 0.0, 1.0])):
        raise ValueError("Expected four CAD motor assemblies with upward body-Z thrust")
    hubs = np.array([part["center_body_m"] for part in rotor_parts], dtype=np.float64)
    if (hubs.shape != (4, 3) or not np.isfinite(hubs).all()
            or not np.array_equal(np.sign(hubs[:, :2]), [[1, 1], [1, -1], [-1, -1], [-1, 1]])):
        raise ValueError("Unexpected ASM2 motor center or rotor order")
    cad_to_body = np.asarray(mass["cad_to_body"], dtype=np.float64)
    for part in rotor_parts:
        part_inertia = cad_to_body @ np.asarray(part["inertia_kg_m2"]) @ cad_to_body.T
        _, eigenvectors = np.linalg.eigh(part_inertia)
        if abs(float(eigenvectors[:, -1] @ axis)) < 1.0 - 1e-8:
            raise ValueError("Rotor thrust axis disagrees with the CAD motor principal axis")
    axes = np.broadcast_to(axis, (4, 3)).copy()

    maximum = float(dynamics["max_rotor_thrust_N"])
    tau = float(dynamics["motor_time_constant_s"])
    yaw_factor = float(dynamics["yaw_torque_per_thrust_m"])
    rate_gain = float(timing["rate_gain"])
    rate_limit = float(timing["action"]["body_rate_limit_rad_s"])
    dt = float(timing["physics_dt_s"])
    policy_dt = float(timing["policy_dt_s"])
    gravity = float(config["gravity_m_s2"])
    if not all(math.isfinite(x) and x > 0 for x in
               (mass_kg, maximum, tau, yaw_factor, rate_gain, rate_limit, dt, policy_dt, gravity)):
        raise ValueError("ASM2 flight scalars must be positive and finite")
    if policy_dt < dt or not math.isclose(policy_dt / dt, round(policy_dt / dt), abs_tol=1e-12):
        raise ValueError("The CTBR interval must contain an integer number of physics steps")
    yaw_signs = np.asarray(config["yaw_reaction_signs"], dtype=np.float64)
    if not np.array_equal(yaw_signs, [1, -1, 1, -1]):
        raise ValueError("Expected the historical alternating rotor reaction torques")
    yaw = yaw_signs * yaw_factor
    torque_per_N = np.cross(hubs, axes) + yaw[:, None] * axes
    mixer = np.vstack((axes[:, 2], torque_per_N.T))
    inverse = np.linalg.inv(mixer)
    hover = inverse @ np.array([mass_kg * gravity, 0.0, 0.0, 0.0])
    if np.any((hover <= 0) | (hover >= maximum)):
        raise ValueError("The configured rotor geometry and thrust limits do not allow hover trim")
    for array in (inertia, hubs, axes, yaw, mixer, inverse, hover):
        array.setflags(write=False)
    return FlightProfile(
        mass_kg=mass_kg,
        inertia_kg_m2=inertia,
        hubs_m=hubs,
        axes=axes,
        yaw_torque_per_thrust_m=yaw,
        max_rotor_thrust_N=maximum,
        motor_time_constant_s=tau,
        rate_gain_per_s=rate_gain,
        native_rate_limit_rad_s=rate_limit,
        physics_dt_s=dt,
        policy_dt_s=policy_dt,
        gravity_m_s2=gravity,
        mixer=mixer,
        inverse_mixer=inverse,
        hover_motor_thrust_N=hover,
        profile_sha256=sha256(config_bytes).hexdigest(),
        source_pins=pins,
        provenance=config["provenance"],
    )


def allocate_ctbr(
    profile: FlightProfile,
    command: np.ndarray,
    body_rates: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Allocate batched physical [T_N, p, q, r] in FLU and flag motor clipping."""
    command = np.asarray(command, dtype=np.float64)
    body_rates = np.asarray(body_rates, dtype=np.float64)
    if command.ndim != 2 or command.shape[1] != 4 or body_rates.shape != (len(command), 3):
        raise ValueError("Expected a (N, 4) CTBR command and (N, 3) body rates")
    if not np.isfinite(command).all() or not np.isfinite(body_rates).all():
        raise ValueError("CTBR commands and measured body rates must be finite")
    collective = np.clip(command[:, :1], 0.0, 4.0 * profile.max_rotor_thrust_N)
    rate_limit = profile.native_rate_limit_rad_s
    setpoint = np.clip(command[:, 1:], -rate_limit, rate_limit)
    angular_accel = profile.rate_gain_per_s * (setpoint - body_rates)
    inertia = profile.inertia_kg_m2
    torque = angular_accel @ inertia.T + np.cross(body_rates, body_rates @ inertia.T)
    rotor_request = np.concatenate((collective, torque), axis=1) @ profile.inverse_mixer.T
    saturated = ((rotor_request < 0.0) | (rotor_request > profile.max_rotor_thrust_N)).any(axis=1)
    return np.clip(rotor_request, 0.0, profile.max_rotor_thrust_N), saturated


class Actuator:
    """Hold CTBR allocations for each control interval and filter thrust per physics step."""

    def __init__(self, profile: FlightProfile, num_envs: int):
        self.profile = profile
        self._alpha = -math.expm1(-profile.physics_dt_s / profile.motor_time_constant_s)
        self.motor_thrust_N = np.empty((num_envs, 4), dtype=np.float64)
        self.motor_command_N = np.empty_like(self.motor_thrust_N)
        self.mixer_saturated = np.empty(num_envs, dtype=bool)
        self.reset()

    def reset(self) -> None:
        self.motor_thrust_N[:] = self.profile.hover_motor_thrust_N
        self.motor_command_N[:] = self.profile.hover_motor_thrust_N
        self.mixer_saturated.fill(False)

    def set_ctbr(self, command: np.ndarray, body_rates: np.ndarray) -> np.ndarray:
        targets, saturated = allocate_ctbr(self.profile, command, body_rates)
        if targets.shape != self.motor_command_N.shape:
            raise ValueError("CTBR batch does not match the actuator environment count")
        self.motor_command_N[:] = targets
        self.mixer_saturated[:] = saturated
        return saturated

    def step_wrench(self) -> tuple[np.ndarray, np.ndarray]:
        """Apply current thrust over this interval, then advance the exact filter."""
        force = self.motor_thrust_N @ self.profile.axes
        torque = self.motor_thrust_N @ self.profile.mixer[1:].T
        self.motor_thrust_N += self._alpha * (self.motor_command_N - self.motor_thrust_N)
        return force, torque
