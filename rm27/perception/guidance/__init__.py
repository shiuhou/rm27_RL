"""uav_guidance - RM27 V0-V2 visual guidance layer.

    TargetEstimate -> UAVGuidance -> GuidanceCommand -> (MAVLink adapter)

See docs/frames.md for frame and sign conventions.
"""

from .command import GuidanceCommand
from .config import GuidanceConfig
from .guidance import GuidanceOutput, GuidanceState, UAVGuidance
from .schema import SCHEMA_VERSION, TargetEstimate

__all__ = [
    "GuidanceCommand",
    "GuidanceConfig",
    "GuidanceOutput",
    "GuidanceState",
    "SCHEMA_VERSION",
    "TargetEstimate",
    "UAVGuidance",
]
