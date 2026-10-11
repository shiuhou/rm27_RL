"""Image-only blue armor and green marker detector for MuJoCo SITL.

The detector consumes rendered RGB pixels and emits the existing
``TargetEstimate`` v2 contract.  It deliberately has no access to MuJoCo
poses, contacts or target identities.  HSV thresholds model the currently
rendered provisional target colors; they are simulation settings and do not
constitute OS04A10 exposure or lens calibration.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from ..guidance import TargetEstimate
from .mujoco_camera import CameraFrame


@dataclass(frozen=True)
class DetectorConfig:
    fovy_deg: float = 60.0
    blue_hue: tuple[int, int] = (90, 140)
    green_hue: tuple[int, int] = (35, 95)
    minimum_area_px: int = 4
    confidence_area_px: float = 120.0
    minimum_confidence: float = 0.15
    max_measurement_age_us: int = 200_000
    require_marker: bool = True

    def __post_init__(self):
        if not 0.0 < float(self.fovy_deg) < 180.0:
            raise ValueError("fovy_deg must be between 0 and 180")
        for name in ("blue_hue", "green_hue"):
            bounds = getattr(self, name)
            if len(bounds) != 2 or not 0 <= int(bounds[0]) <= int(bounds[1]) <= 179:
                raise ValueError(f"invalid {name}")
        if int(self.minimum_area_px) < 1 or float(self.confidence_area_px) <= 0:
            raise ValueError("detector area thresholds must be positive")
        if not 0.0 <= float(self.minimum_confidence) <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if type(self.max_measurement_age_us) is not int or self.max_measurement_age_us < 0:
            raise ValueError('max_measurement_age_us must be a non-negative integer')


def _mask(hsv: np.ndarray, hue: tuple[int, int]) -> np.ndarray:
    low, high = hue
    return cv2.inRange(hsv, np.array([low, 70, 45], dtype=np.uint8), np.array([high, 255, 255], dtype=np.uint8))


def _components(mask: np.ndarray, minimum_area: int):
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    result = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area >= minimum_area:
            result.append((area, labels == label, tuple(float(x) for x in centroids[label])))
    return result


class ImageTargetDetector:
    """Convert a camera frame into a target-relative angle estimate."""

    def __init__(self, config: DetectorConfig | None = None):
        self.config = config or DetectorConfig()

    def detect(self, frame: CameraFrame | np.ndarray, *, now_us: int | None = None) -> TargetEstimate:
        if isinstance(frame, CameraFrame):
            image = frame.image_rgb
            timestamp_us = frame.timestamp_us
            width, height = frame.width, frame.height
        else:
            image = np.asarray(frame)
            if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
                raise ValueError("image must be an HxWx3 uint8 RGB array")
            timestamp_us = 0
            height, width = image.shape[:2]
        if width < 2 or height < 2:
            raise ValueError("image is too small")
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        blue = _mask(hsv, self.config.blue_hue)
        green = _mask(hsv, self.config.green_hue)
        kernel = np.ones((3, 3), np.uint8)
        blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel)
        green = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
        blue_components = _components(blue, self.config.minimum_area_px)
        green_components = _components(green, self.config.minimum_area_px)

        selected = None
        associated = False
        candidates = []
        for armor in blue_components:
            ys, xs = np.nonzero(armor[1])
            box_width, box_height = np.ptp(xs) + 1, np.ptp(ys) + 1
            # Field stripes are blue too. Require a compact filled patch.
            if not .25 <= box_width / box_height <= 6 or armor[0] / (box_width * box_height) < .4:
                continue
            for marker in green_components:
                dx = abs(marker[2][0] - armor[2][0])
                dy = armor[2][1] - marker[2][1]
                if dx <= 1.5 * box_width and .2 * box_height <= dy <= 5 * box_height:
                    candidates.append(armor)
                    break
        if len(candidates) == 1:
            selected, associated = candidates[0], True
        elif len(candidates) > 1:
            # Ambiguous identity cannot authorize control.
            selected = max(candidates, key=lambda item: item[0])
        elif blue_components:
            selected = max(blue_components, key=lambda item: item[0])

        if selected is None:
            return TargetEstimate(timestamp_us=int(timestamp_us), measurement_age_us=self._age(now_us, timestamp_us), valid=False)

        area, component_mask, centroid = selected
        # Armor and marker are distinct objects. Never average their centers:
        # doing so biases the commanded bearing away from the contact patch.
        points = np.argwhere(component_mask)
        v, u = np.mean(points, axis=0)
        age = self._age(now_us, timestamp_us)
        fx = (height / 2.0) / math.tan(math.radians(self.config.fovy_deg) / 2.0)
        fy = fx
        cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
        yaw = math.atan2(float(u - cx), fx)
        pitch = -math.atan2(float(v - cy), fy)
        saturation = float(np.mean(hsv[component_mask, 1])) / 255.0
        confidence = float(np.clip((area / self.config.confidence_area_px) * saturation, 0.0, 1.0))
        valid = bool(math.isfinite(yaw) and math.isfinite(pitch) and area >= self.config.minimum_area_px)
        safe = bool(valid and isinstance(frame, CameraFrame) and (associated or not self.config.require_marker)
                    and confidence >= self.config.minimum_confidence and age <= self.config.max_measurement_age_us)
        return TargetEstimate(
            timestamp_us=int(timestamp_us),
            measurement_age_us=age,
            valid=valid,
            safe_for_control=safe,
            predicted=False,
            yaw_rad=yaw,
            pitch_rad=pitch,
            confidence=confidence,
        )

    @staticmethod
    def _age(now_us: int | None, timestamp_us: int) -> int:
        if now_us is None:
            return 0
        if type(now_us) is not int or now_us < 0:
            raise ValueError("now_us must be a non-negative integer")
        if now_us < timestamp_us:
            raise ValueError('image timestamp is in the future')
        return now_us - timestamp_us

