"""Only these sampled data cross into autonomy. SI; local Z up; camera RDF."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class EgoEstimate:
    time: float
    position: np.ndarray
    velocity: np.ndarray
    camera_to_local: np.ndarray


@dataclass(frozen=True)
class ImageObservation:
    capture_time: float
    delivery_time: float
    uv_px: np.ndarray  # offsets from principal point, right/down
    angular_radius_rad: np.ndarray  # calibrated angular extent of spherical fixture
    valid: np.ndarray
    camera_position_estimate: np.ndarray  # exposure-time ego estimate, not true pose
    camera_to_local_estimate: np.ndarray
