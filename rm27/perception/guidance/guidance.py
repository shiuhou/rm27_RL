"""UAVGuidance - V0 visual-guidance state machine.

    TargetEstimate -> UAVGuidance -> GuidanceCommand

States:
    SEARCH     : no usable target yet (or target invalid before first
                 acquisition). Commands a zero-motion hold, valid=True.
    ALIGN      : target accepted; rotate in place, vx = 0.
    APPROACH   : aligned; drive forward at approach_vx_mps with yaw P-control.
    SAFE_HOLD  : a previously-acquired target became unusable. Zero motion,
                 valid=False. Recovers to ALIGN on a fresh safe estimate.

Safety: any rejected input produces zero motion immediately. No stale
forward command may persist. All timing comes from the caller-injected
`now_us` (monotonic or simulation time); this module never reads a clock.
"""

from dataclasses import dataclass
from enum import Enum, auto

from .command import GuidanceCommand
from .config import GuidanceConfig
from .schema import TargetEstimate


class GuidanceState(Enum):
    SEARCH = auto()
    ALIGN = auto()
    APPROACH = auto()
    SAFE_HOLD = auto()


@dataclass
class GuidanceOutput:
    state: GuidanceState
    command: GuidanceCommand
    reject_reason: str = ""  # empty when the input was accepted


class UAVGuidance:
    def __init__(self, config=None):
        self.config = config if config is not None else GuidanceConfig()
        self.state = GuidanceState.SEARCH
        self._acquired = False  # have we ever accepted a control target?
        self._last_estimate_timestamp_us = None

    def _reject_reason(self, estimate, now_us):
        """Return "" if the estimate may drive control, else the reason."""
        if not estimate.angles_finite():
            return "non_finite"
        if not estimate.valid:
            return "invalid_estimate"
        if not estimate.safe_for_control:
            return "unsafe_for_control"
        if estimate.measurement_age_us > self.config.stale_threshold_us:
            return "stale_measurement"
        if now_us - estimate.timestamp_us > self.config.producer_timeout_us:
            return "producer_timeout"
        if estimate.confidence < self.config.min_confidence:
            return "low_confidence"
        return ""

    def update(self, estimate, now_us):
        """Advance one control cycle.

        `estimate` is the newest TargetEstimate or None if the producer
        published nothing. `now_us` is monotonic or simulation time.
        """
        if estimate is None:
            return self._handle_missing_estimate(now_us)

        self._last_estimate_timestamp_us = estimate.timestamp_us
        reason = self._reject_reason(estimate, now_us)
        if reason:
            return self._handle_rejected(now_us, reason)

        return self._handle_accepted(estimate, now_us)

    def _handle_missing_estimate(self, now_us):
        # Motion is always zeroed without a fresh estimate.
        silent_for = (
            None
            if self._last_estimate_timestamp_us is None
            else now_us - self._last_estimate_timestamp_us
        )
        if (
            self._acquired
            and silent_for is not None
            and silent_for > self.config.producer_timeout_us
        ):
            self.state = GuidanceState.SAFE_HOLD
            command = GuidanceCommand.zero(now_us, valid=False)
            return GuidanceOutput(self.state, command, "producer_timeout")

        # Retain the tactical state briefly; only a fresh accepted
        # estimate may re-arm motion.
        command = GuidanceCommand.zero(now_us, valid=True)
        if not self._acquired:
            self.state = GuidanceState.SEARCH
        return GuidanceOutput(self.state, command, "awaiting_estimate")

    def _handle_rejected(self, now_us, reason):
        if self._acquired:
            self.state = GuidanceState.SAFE_HOLD
            command = GuidanceCommand.zero(now_us, valid=False)
        else:
            self.state = GuidanceState.SEARCH
            command = GuidanceCommand.zero(now_us, valid=True)
        return GuidanceOutput(self.state, command, reason)

    def _handle_accepted(self, estimate, now_us):
        if not self._acquired:
            self._acquired = True
            self.state = GuidanceState.ALIGN
        elif self.state in (GuidanceState.SEARCH, GuidanceState.SAFE_HOLD):
            self.state = GuidanceState.ALIGN

        yaw_err = estimate.yaw_rad

        if self.state == GuidanceState.ALIGN:
            if abs(yaw_err) <= self.config.align_threshold_rad:
                self.state = GuidanceState.APPROACH
        elif self.state == GuidanceState.APPROACH:
            if abs(yaw_err) > self.config.realign_threshold_rad:
                self.state = GuidanceState.ALIGN

        yaw_rate = self._clamp(
            self.config.k_yaw_per_rad * yaw_err,
            self.config.max_yaw_rate_rad_s,
        )
        vx = (
            self.config.approach_vx_mps
            if self.state == GuidanceState.APPROACH
            else 0.0
        )
        command = GuidanceCommand(
            timestamp_us=now_us,
            vx_body_mps=vx,
            vy_body_mps=0.0,
            vz_body_mps=0.0,
            yaw_rate_rad_s=yaw_rate,
            valid=True,
        )
        return GuidanceOutput(self.state, command, "")

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))
