"""Prospective simulated navigation references for the ASM2 handover suite."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np
from numpy.polynomial import polynomial as poly

from .asset import ROOT
from .contact_task import ContactSpec
from .flight import FlightProfile
from .flight_suite import PositionController, flight_settings, rotation_matrix


CONFIG = ROOT / "configs/rm27_asm2_handover_navigation.json"


@dataclass(frozen=True)
class NavigationCase:
    """An initially stationary case with a fixed moving reference at handover."""

    name: str
    split: str
    kind: str
    start: np.ndarray
    nav_ticks: int
    dt: float
    hold_s: float
    settings: dict
    tracking_screen: dict
    recipe_sha256: str
    reference_derivatives: tuple[np.ndarray, np.ndarray, np.ndarray]

    def reference(self, seconds: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Supply world position, velocity, and acceleration to PositionController."""
        motion_s = self.nav_ticks * self.dt - self.hold_s
        phase = np.clip((seconds - self.hold_s) / motion_s, 0., 1.)
        position, velocity, acceleration = (poly.polyval(phase, terms)[None, :] for terms in self.reference_derivatives)
        return position, velocity, acceleration

    def controller(self, profile: FlightProfile) -> PositionController:
        if profile.policy_dt_s != self.dt:
            raise ValueError("Navigation reference and CTBR update times must agree")
        return PositionController(profile, self)


def _reference_derivatives(position: np.ndarray, end: tuple[np.ndarray, ...], duration: float) -> tuple:
    """Septic Hermite motion, with zero initial derivatives through jerk."""
    zeros = np.zeros(3)
    constraints = ((position, zeros, zeros, zeros), (*end, zeros))
    basis = np.eye(8)
    matrix, rhs = [], []
    for endpoint, derivatives in enumerate(constraints):
        for order, value in enumerate(derivatives):
            matrix.append(poly.polyval(endpoint, poly.polyder(basis, m=order)))
            rhs.append(value * duration**order)
    coefficients = np.linalg.solve(matrix, rhs)
    terms = tuple(poly.polyder(coefficients, m=i) / duration**i for i in range(3))
    for term in terms:
        term.setflags(write=False)
    return terms


def navigation_cases(profile: FlightProfile, spec: ContactSpec) -> tuple[NavigationCase, ...]:
    """Load all prospective development and held-out cases in frozen order."""
    source = CONFIG.read_bytes()
    config = json.loads(source)
    if config["schema"] != "rm27_asm2_simulated_navigation_v1":
        raise ValueError("Unknown ASM2 navigation recipe schema")
    dt = profile.policy_dt_s
    nav_ticks = round(config["handover_time_s"] / dt)
    hold = float(config["hold_s"])
    if not math.isclose(nav_ticks * dt, config["handover_time_s"], abs_tol=1e-12, rel_tol=0):
        raise ValueError("The handover time must fall on a CTBR update boundary")
    duration = nav_ticks * dt - hold
    if hold <= 0. or duration <= 0.:
        raise ValueError("Navigation requires a positive stationary hold and motion interval")
    recipes = config["recipes"]
    if (len({case["name"] for case in recipes}) != 12
            or tuple(case["split"] for case in recipes) != ("development",) * 8 + ("heldout",) * 4
            or any(case["kind"] not in ("nominal", "boundary") for case in recipes)):
        raise ValueError("Expected 8 development and 4 held-out navigation cases")
    bounds = spec.field_geometry["selection"]
    lower = np.asarray(bounds["safety_lower_target_frame_m"])
    upper = np.asarray(bounds["safety_upper_target_frame_m"])
    frame = spec.target_rotation
    settings = flight_settings()
    recipe_hash = hashlib.sha256(source).hexdigest()
    cases = []
    for recipe in recipes:
        start_local, goal_local, velocity, accel = (np.asarray(recipe[key], dtype=float) for key in (
            "start_m", "handover_m", "handover_velocity_m_s", "handover_acceleration_m_s2"
        ))
        if (not np.isfinite(np.r_[start_local, goal_local, velocity, accel]).all()
                or start_local[2] <= goal_local[2] or velocity[2] >= 0.):
            raise ValueError(f"Navigation recipe must start farther out and finish moving inward: {recipe['name']}")
        position = spec.target_center + frame @ start_local
        endpoint = spec.target_center + frame @ goal_local, frame @ velocity, frame @ accel
        terms = _reference_derivatives(position, endpoint, duration)
        # This validates the prospective COM reference only. The executed
        # swept vehicle and pre-handover contacts are checked in the plant.
        samples = poly.polyval(np.linspace(0., 1., nav_ticks+1), terms[0]).T
        local = (samples - spec.target_center) @ frame
        if np.any((local < lower) | (local > upper)):
            raise ValueError(f"Navigation reference exits the archived center bounds: {recipe['name']}")
        state = np.r_[position, 1., np.zeros(9)]
        state.setflags(write=False)
        cases.append(NavigationCase(recipe["name"], recipe["split"], recipe["kind"], state, nav_ticks, dt, hold,
                                    settings, config["tracking_screen"], recipe_hash, terms))
    return tuple(cases)


def measure_handover(case: NavigationCase, state: np.ndarray, spec: ContactSpec) -> dict:
    """Compare a real simulator state at the fixed handover tick to its reference."""
    state = np.asarray(state, dtype=float)
    if state.shape != (13,) or not np.isfinite(state).all() or not np.isclose(np.linalg.norm(state[3:7]), 1.):
        raise ValueError("Expected a finite native handover state with a unit wxyz quaternion")
    frame = spec.target_rotation
    reference, velocity, _ = case.reference(case.nav_ticks * case.dt)
    position_local = (state[:3] - spec.target_center) @ frame
    velocity_local = state[7:10] @ frame
    expected_position = (reference[0] - spec.target_center) @ frame
    expected_velocity = velocity[0] @ frame
    position_error = float(np.linalg.norm(position_local - expected_position))
    velocity_error = float(np.linalg.norm(velocity_local - expected_velocity))
    body_up = rotation_matrix(state[3:7])[:, 2]
    crop = spec.field_geometry["selection"]
    inside_center_bounds = bool(np.all(position_local >= crop["safety_lower_target_frame_m"])
                                and np.all(position_local <= crop["safety_upper_target_frame_m"]))
    tracking = case.tracking_screen
    tracked = (position_error <= tracking["position_error_max_m"]
               and velocity_error <= tracking["velocity_error_max_m_s"])
    return {
        "handover_tick": case.nav_ticks,
        "handover_time_s": case.nav_ticks * case.dt,
        "nominal_position_target_m": expected_position.tolist(),
        "nominal_velocity_target_m_s": expected_velocity.tolist(),
        "position_target_m": position_local.tolist(),
        "velocity_target_m_s": velocity_local.tolist(),
        "position_tracking_error_m": position_error,
        "velocity_tracking_error_m_s": velocity_error,
        "com_standoff_m": float(position_local[2]),
        "nominal_face_clearance_m": float(position_local[2] + spec.patch_face_z_m),
        "inward_speed_m_s": float(-velocity_local[2]),
        "tangential_speed_m_s": float(np.linalg.norm(velocity_local[:2])),
        "quaternion_wxyz": state[3:7].tolist(),
        "body_rate_rad_s": state[10:13].tolist(),
        "body_rate_norm_rad_s": float(np.linalg.norm(state[10:13])),
        "body_to_target_alignment_deg": float(np.rad2deg(np.arccos(np.clip(body_up @ spec.target_normal, -1., 1.)))),
        "world_tilt_deg": float(np.rad2deg(np.arccos(np.clip(body_up[2], -1., 1.)))),
        "inside_archived_center_bounds": inside_center_bounds,
        "navigation_tracking_pass": bool(tracked and inside_center_bounds and velocity_local[2] < 0.),
    }
