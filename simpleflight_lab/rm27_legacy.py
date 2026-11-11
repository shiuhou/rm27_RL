"""Fixed RM27 Flight Probe inputs for an Isaac Lab no-contact comparison.

The MuJoCo nominal hover point translates to local (0, 0, 1) without rotating
world axes. This preserves reference-relative flight quantities and the tilted
target frame used for the reset perturbations. It does not transfer the field
colliders, MuJoCo warning checks, or continuous 1 ms contact scoring.
"""

from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path

import numpy as np


RM27_RL_REVISION = "ed4ebc550800baf032348f67af5699b44e7953f2"
_SOURCE_ROOT = Path(__file__).resolve().parents[1] / "upstream" / "rm27_RL"


@dataclass(frozen=True)
class LegacyGate:
    window_s: float
    position_m: float
    speed_m_s: float
    tilt_deg: float
    body_rate_rad_s: float
    min_pass_count: int


@dataclass(frozen=True)
class LegacyFlight:
    position: np.ndarray  # Relative to local (0, 0, 1), world axes.
    quat: np.ndarray  # WXYZ, body to world.
    velocity: np.ndarray  # World linear (m/s) and angular (rad/s) velocity.
    target: np.ndarray  # Relative to local (0, 0, 1), world axes.
    names: tuple[str, ...]
    horizon_s: float
    gate: LegacyGate
    target_rotation: np.ndarray  # Target frame to world.
    normal_standoff_m: float
    safety_low: np.ndarray  # Target frame, relative to the historical plate center.
    safety_high: np.ndarray
    safety_max_speed_m_s: float
    safety_max_body_rate_rad_s: float
    policy_dt_s: float
    physics_dt_s: float


def load_working_hover() -> tuple[dict, dict]:
    """Load the historical working flight reward and speed/rate safety bounds."""
    working = json.loads((_SOURCE_ROOT / "configs/flight_working/attitude_baseline.json").read_text())
    probe = json.loads((_SOURCE_ROOT / working["base_config"]).read_text())
    stage = json.loads((_SOURCE_ROOT / probe["base_task_config"]).read_text())
    return working["reward"], stage["safety"]


def load_legacy_flight(task: str) -> LegacyFlight:
    """Load 19 fixed Hover starts or four cardinal Track shifts in source order."""
    if task not in ("hover", "track"):
        raise ValueError(f"Unknown RM27 legacy flight task {task!r}")
    probe = json.loads((_SOURCE_ROOT / "configs/flight_probe/task_b.json").read_text())
    stage = json.loads((_SOURCE_ROOT / probe["base_task_config"]).read_text())
    registry_path = _SOURCE_ROOT / stage["contact_surface_registration"]
    registry_bytes = registry_path.read_bytes()
    if sha256(registry_bytes).hexdigest() != stage["expected_contact_surface_registration_sha256"]:
        raise ValueError("RM27 legacy target registration hash mismatch")
    if probe["task_id"] != "state_flight_control_probe_v1" or stage["horizon_s"] != probe["horizon_s"]:
        raise ValueError("RM27 legacy flight contract mismatch")

    # stage1_geometry uses [-slope, across, normal] from the hashed registry.
    registered_frame = json.loads(registry_bytes)["target_reference"]
    frame = np.column_stack((-np.asarray(registered_frame["slope_axis"]),
                             registered_frame["across_axis"], registered_frame["armor_normal"]))
    frame /= np.linalg.norm(frame, axis=0)
    if not np.allclose(frame.T @ frame, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(np.linalg.det(frame), 1):
        raise ValueError("RM27 legacy target frame is not orthonormal")

    init = stage["initial_state"]
    references = np.asarray(probe["references_world_m"], dtype=np.float64)
    if task == "hover":
        # fixed_evaluation_initials varies one position, velocity or attitude axis at a time.
        halfwidth = np.r_[init["position_perturbation_halfwidth_target_frame_m"],
                          init["velocity_perturbation_halfwidth_world_m_s"],
                          np.deg2rad(init["rotation_vector_halfwidth_body_deg"])]
        perturbations = np.zeros((19, 9))
        perturbations[1::2] = -np.diag(halfwidth)
        perturbations[2::2] = np.diag(halfwidth)
        names = ("hover_nominal",) + tuple(
            f"hover_{group}_{axis}_{sign:+d}"
            for group in ("position", "velocity", "attitude") for axis in range(3) for sign in (-1, 1)
        )
        targets = np.repeat(references[:1], len(names), axis=0)
    else:
        names = ("shift_x_plus", "shift_x_minus", "shift_y_plus", "shift_y_minus")
        targets = references[1:]
        perturbations = np.zeros((len(names), 9))

    rotvec = perturbations[:, 6:9]
    angle = np.linalg.norm(rotvec, axis=-1, keepdims=True)
    # sin(angle / 2) / angle, including its continuous limit at angle = 0.
    quat = np.hstack((np.cos(angle / 2), 0.5 * np.sinc(angle / (2 * np.pi)) * rotvec))
    safety = stage["safety"]
    return LegacyFlight(
        position=perturbations[:, :3] @ frame.T,
        quat=quat,
        velocity=np.hstack((perturbations[:, 3:6], np.zeros_like(rotvec))),
        target=targets,
        names=names,
        horizon_s=probe["horizon_s"],
        # Suite-level thresholds are specified by the working-baseline SPEC §13.
        gate=LegacyGate(**probe["stability"], min_pass_count=18 if task == "hover" else 3),
        target_rotation=frame,
        normal_standoff_m=init["normal_standoff_m"],
        safety_low=np.asarray(safety["position_min_target_frame_m"]),
        safety_high=np.asarray(safety["position_max_target_frame_m"]),
        safety_max_speed_m_s=safety["max_speed_m_s"],
        safety_max_body_rate_rad_s=safety["max_body_rate_rad_s"],
        policy_dt_s=stage["policy_dt_s"],
        physics_dt_s=stage["physics_dt_s"],
    )
