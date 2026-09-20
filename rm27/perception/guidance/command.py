"""GuidanceCommand - the frozen output of the guidance layer.

Body-frame FRD semantics (docs/frames.md):
- vx_body_mps     : forward
- vy_body_mps     : right   (V0-V2: always 0)
- vz_body_mps     : down    (V0-V2: always 0)
- yaw_rate_rad_s  : positive = yaw right

The MAVLink adapter consumes ONLY this type; it never sees TargetEstimate.
"""

from dataclasses import dataclass


@dataclass
class GuidanceCommand:
    timestamp_us: int = 0
    vx_body_mps: float = 0.0
    vy_body_mps: float = 0.0
    vz_body_mps: float = 0.0
    yaw_rate_rad_s: float = 0.0
    valid: bool = False

    @classmethod
    def zero(cls, timestamp_us, valid):
        return cls(timestamp_us=timestamp_us, valid=valid)
