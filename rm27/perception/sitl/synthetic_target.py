"""SyntheticTargetProducer - scripted V1 TargetEstimate source.

Replaces the camera for V1 SITL testing. Produces the exact same
TargetEstimate semantics as the vision pipeline (schema v2), driven by a
script of timed steps:

    ("bearing", duration_s, {"yaw_deg": float, "pitch_deg": float})
        fresh, valid, safe estimate at the given bearing
    ("unsafe", duration_s, {})
        valid estimate with safe_for_control=False
    ("stale", duration_s, {})
        valid estimate whose measurement_age_us exceeds the stale gate
    ("dropout", duration_s, {})
        no estimate at all (update returns None)

All timing is driven by the caller-injected `now_us`.
"""

import math

from ..guidance import TargetEstimate


class SyntheticTargetProducer:
    def __init__(self, steps, start_us, confidence=0.9):
        self._steps = list(steps)
        self._start_us = start_us
        self._confidence = confidence

        self._bounds = []  # (start_us, end_us, kind, params)
        cursor = start_us
        for kind, duration_s, params in self._steps:
            end = cursor + int(duration_s * 1_000_000)
            self._bounds.append((cursor, end, kind, params))
            cursor = end
        self._end_us = cursor

    def finished(self, now_us):
        return now_us >= self._end_us

    def update(self, now_us):
        for start, end, kind, params in self._bounds:
            if start <= now_us < end:
                return self._emit(kind, params, now_us)
        return None  # script exhausted: behave like a dead producer

    def _emit(self, kind, params, now_us):
        if kind == "dropout":
            return None

        base = dict(
            timestamp_us=now_us,
            measurement_age_us=20_000,
            valid=True,
            safe_for_control=True,
            predicted=False,
            yaw_rad=math.radians(params.get("yaw_deg", 0.0)),
            pitch_rad=math.radians(params.get("pitch_deg", 0.0)),
            confidence=self._confidence,
        )
        if kind == "bearing":
            pass
        elif kind == "unsafe":
            base["safe_for_control"] = False
        elif kind == "stale":
            base["measurement_age_us"] = 400_000  # beyond the 200 ms gate
        else:
            raise ValueError(f"unknown synthetic step kind: {kind}")
        return TargetEstimate(**base)
