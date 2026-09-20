"""TargetEstimate - versioned schema adapter for maixcam2_dart schema v2.

The canonical definition lives in the vision repository
(maixcam2_dart, `main/include/dart/green_detector.hpp`, schema_version = 2).
This module is a transport/schema adapter: it carries the exact control
semantics of the canonical type into Python so that synthetic, truth
(Gazebo), and future real-camera producers all emit identical input to
UAVGuidance. It intentionally does NOT redefine or extend the semantics.

Frame conventions: see docs/frames.md.
"""

import json
import math
from dataclasses import dataclass

SCHEMA_VERSION = 2


@dataclass
class TargetEstimate:
    """Control-relevant subset of maixcam2_dart TargetEstimate (v2).

    Semantics identical to the canonical C++ struct:
    - yaw_rad > 0        : target is RIGHT of camera boresight
    - pitch_rad > 0      : target is ABOVE camera boresight
    - measurement_age_us : age of the underlying visual measurement
    - valid              : estimate exists at all
    - safe_for_control   : vision pipeline says this may drive control
    - confidence         : fused [0, 1] confidence
    """

    timestamp_us: int = 0
    measurement_age_us: int = 0
    valid: bool = False
    safe_for_control: bool = False
    predicted: bool = False
    yaw_rad: float = 0.0
    pitch_rad: float = 0.0
    confidence: float = 0.0
    schema_version: int = SCHEMA_VERSION

    @classmethod
    def from_json(cls, line):
        """Parse one JSON line emitted by maixcam2_dart target_json.cpp."""
        data = json.loads(line)
        if int(data.get("schema_version", -1)) != SCHEMA_VERSION:
            raise ValueError(
                f"unsupported TargetEstimate schema_version "
                f"{data.get('schema_version')}; expected {SCHEMA_VERSION}"
            )
        return cls(
            timestamp_us=int(data["timestamp_us"]),
            measurement_age_us=int(data["measurement_age_us"]),
            valid=bool(data["valid"]),
            safe_for_control=bool(data["safe_for_control"]),
            predicted=bool(data["predicted"]),
            yaw_rad=float(data["yaw_rad"]),
            pitch_rad=float(data["pitch_rad"]),
            confidence=float(data["confidence"]),
        )

    def angles_finite(self):
        return math.isfinite(self.yaw_rad) and math.isfinite(self.pitch_rad)
