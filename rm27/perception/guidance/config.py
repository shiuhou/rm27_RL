"""GuidanceConfig - all tunables for V0-V2 guidance in one place."""

import math
from dataclasses import dataclass


@dataclass
class GuidanceConfig:
    # Proportional yaw gain: rad/s of yaw rate per radian of yaw error.
    k_yaw_per_rad: float = 1.0
    # Hard clamp on commanded yaw rate (15 deg/s per the V0 freeze).
    max_yaw_rate_rad_s: float = math.radians(15.0)
    # Forward approach speed while in APPROACH.
    approach_vx_mps: float = 0.30
    # Hysteresis band: enter APPROACH at |err| <= align_threshold,
    # leave APPROACH at |err| > realign_threshold.
    align_threshold_rad: float = math.radians(2.0)
    realign_threshold_rad: float = math.radians(5.0)
    # Safety gates.
    stale_threshold_us: int = 200_000     # max visual measurement age
    producer_timeout_us: int = 500_000    # max age of the newest message
    min_confidence: float = 0.30
