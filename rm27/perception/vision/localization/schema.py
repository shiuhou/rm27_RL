"""The versioned ego-localization record used by future SLAM/VIO adapters."""

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import Any, Mapping

LOCALIZATION_SCHEMA_VERSION = 2
QUATERNION_NORM_TOLERANCE = 1e-6


class ScaleState(str, Enum):
    METRIC = "METRIC"
    ARBITRARY = "ARBITRARY"
    UNKNOWN = "UNKNOWN"


class TrackingState(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    INITIALIZING = "INITIALIZING"
    TRACKING = "TRACKING"
    RELOCALIZING = "RELOCALIZING"
    LOST = "LOST"
    RESET = "RESET"


@dataclass(frozen=True)
class TimestampEvidence:
    """A raw timestamp and interpretation claim; only VERIFIED is evidence of that claim."""

    raw_value: int
    unit: str = "UNKNOWN"
    clock_domain: str = "UNKNOWN"
    semantic: str = "UNKNOWN"
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        if isinstance(self.raw_value, bool) or not isinstance(self.raw_value, int):
            raise ValueError("timestamp raw_value must be an integer")
        if self.raw_value < 0:
            raise ValueError("timestamp raw_value must be non-negative")
        for name in ("unit", "clock_domain", "semantic", "evidence_status"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value:
                raise ValueError(f"timestamp {name} must be a non-empty string")

    def to_dict(self):
        return {
            "raw_value": self.raw_value,
            "unit": self.unit,
            "clock_domain": self.clock_domain,
            "semantic": self.semantic,
            "evidence_status": self.evidence_status,
        }

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]):
        if not isinstance(value, Mapping):
            raise ValueError("timestamp must be an object")
        required = ("raw_value", "unit", "clock_domain", "semantic", "evidence_status")
        missing = [key for key in required if key not in value]
        if missing:
            raise ValueError("timestamp missing: " + ", ".join(missing))
        return cls(**{key: value[key] for key in required})


def _finite_vector(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ValueError(f"{name} must contain real numbers, not bool or strings")
    try:
        result = tuple(float(item) for item in value)
    except OverflowError as exc:
        raise ValueError(f"{name} contains a non-finite value") from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


@dataclass(frozen=True)
class LocalizationEstimate:
    """A self-localization estimate, independent of target observations.

    ``T_parent_child`` maps coordinates in ``child_frame`` into
    ``parent_frame``.  Freshness is intentionally computed by consumers from
    source/publish timestamps; it is not serialized as a fixed measurement age.
    """

    source_time: TimestampEvidence
    publish_time: TimestampEvidence
    parent_frame: str
    child_frame: str
    translation: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    tracking_state: TrackingState
    valid: bool
    initialized: bool
    relocalizing: bool
    localization_epoch: int
    scale_state: ScaleState = ScaleState.UNKNOWN
    translation_unit: str = "UNKNOWN"
    velocity_unit: str | None = None
    velocity: tuple[float, float, float] | None = None
    velocity_frame: str | None = None
    map_id: str | None = None
    reset_reason: str | None = None
    quality: Mapping[str, Any] | None = None
    schema_version: int = LOCALIZATION_SCHEMA_VERSION

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != LOCALIZATION_SCHEMA_VERSION:
            raise ValueError(f"unsupported LocalizationEstimate schema_version {self.schema_version}")
        if not isinstance(self.source_time, TimestampEvidence) or not isinstance(self.publish_time, TimestampEvidence):
            raise ValueError("source_time and publish_time must be TimestampEvidence")
        for name in ("parent_frame", "child_frame"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name):
                raise ValueError(f"{name} must be a non-empty string")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent_frame and child_frame must differ")
        for name in ("valid", "initialized", "relocalizing"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be boolean")
        object.__setattr__(self, "translation", _finite_vector(self.translation, 3, "translation"))
        object.__setattr__(self, "orientation_wxyz", _finite_vector(self.orientation_wxyz, 4, "orientation_wxyz"))
        norm = math.hypot(*self.orientation_wxyz)
        if not math.isfinite(norm) or abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            raise ValueError("orientation_wxyz must be non-zero and unit within absolute norm tolerance 1e-6")
        if not isinstance(self.tracking_state, TrackingState):
            try:
                object.__setattr__(self, "tracking_state", TrackingState(self.tracking_state))
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid tracking_state") from exc
        if not isinstance(self.localization_epoch, int) or isinstance(self.localization_epoch, bool) or self.localization_epoch < 0:
            raise ValueError("localization_epoch must be a non-negative integer")
        try:
            object.__setattr__(self, "scale_state", ScaleState(self.scale_state))
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid scale_state") from exc
        unit, speed_unit = {ScaleState.METRIC: ("m", "m/s"),
                            ScaleState.ARBITRARY: ("arbitrary", "arbitrary/s"),
                            ScaleState.UNKNOWN: ("UNKNOWN", "UNKNOWN")}[self.scale_state]
        if self.translation_unit != unit:
            raise ValueError("translation_unit inconsistent with scale_state")
        state = self.tracking_state
        if self.relocalizing != (state == TrackingState.RELOCALIZING):
            raise ValueError("relocalizing must exactly identify RELOCALIZING state")
        if state in (TrackingState.UNINITIALIZED, TrackingState.INITIALIZING, TrackingState.RESET):
            if self.valid or self.initialized:
                raise ValueError("uninitialized/initializing/reset records cannot contain a valid initialized pose")
        elif not self.initialized:
            raise ValueError("tracking/lost/relocalizing requires initialized")
        if state == TrackingState.TRACKING and not self.valid:
            raise ValueError("TRACKING requires a current valid pose (not flight eligibility)")
        if self.velocity is not None:
            if self.velocity_unit != speed_unit:
                raise ValueError("velocity_unit inconsistent with scale_state")
            object.__setattr__(self, "velocity", _finite_vector(self.velocity, 3, "velocity"))
            if not isinstance(self.velocity_frame, str) or not self.velocity_frame:
                raise ValueError("velocity_frame is required when velocity is present")
        elif self.velocity_unit is not None:
            raise ValueError("velocity_unit requires velocity")
        elif self.velocity_frame is not None:
            raise ValueError("velocity_frame requires velocity")
        if self.quality is not None and not isinstance(self.quality, Mapping):
            raise ValueError("quality must be an object when present")
        if self.quality is not None:
            try:
                json.dumps(dict(self.quality), allow_nan=False)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError("quality must be strict JSON") from exc
        for name in ("map_id", "reset_reason"):
            value = getattr(self, name)
            if value is not None and (not isinstance(value, str) or not value):
                raise ValueError(f"{name} must be a non-empty string when supplied")

    def to_dict(self):
        data = {
            "schema_version": self.schema_version,
            "source_time": self.source_time.to_dict(),
            "publish_time": self.publish_time.to_dict(),
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "translation": list(self.translation),
            "translation_unit": self.translation_unit,
            "scale_state": self.scale_state.value,
            "orientation_wxyz": list(self.orientation_wxyz),
            "tracking_state": self.tracking_state.value,
            "valid": self.valid,
            "initialized": self.initialized,
            "relocalizing": self.relocalizing,
            "localization_epoch": self.localization_epoch,
        }
        if self.velocity is not None:
            data["velocity"] = list(self.velocity)
            data["velocity_frame"] = self.velocity_frame
            data["velocity_unit"] = self.velocity_unit
        for key in ("map_id", "reset_reason", "quality"):
            value = getattr(self, key)
            if value is not None:
                data[key] = dict(value) if key == "quality" else value
        return data

    def to_json(self):
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True, allow_nan=False)

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]):
        if not isinstance(value, Mapping):
            raise ValueError("LocalizationEstimate must be an object")
        version = value.get("schema_version", -1)
        if type(version) is not int or version != LOCALIZATION_SCHEMA_VERSION:
            raise ValueError(f"unsupported LocalizationEstimate schema_version {version}")
        required = ("source_time", "publish_time", "parent_frame", "child_frame",
                    "translation", "orientation_wxyz", "tracking_state", "valid",
                    "initialized", "relocalizing", "localization_epoch", "scale_state", "translation_unit")
        missing = [key for key in required if key not in value]
        if missing:
            raise ValueError("missing LocalizationEstimate fields: " + ", ".join(missing))
        return cls(
            schema_version=version,
            source_time=TimestampEvidence.from_dict(value["source_time"]),
            publish_time=TimestampEvidence.from_dict(value["publish_time"]),
            parent_frame=value["parent_frame"], child_frame=value["child_frame"],
            translation=value["translation"], orientation_wxyz=value["orientation_wxyz"],
            tracking_state=TrackingState(value["tracking_state"]),
            valid=value["valid"], initialized=value["initialized"],
            relocalizing=value["relocalizing"], localization_epoch=value["localization_epoch"],
            scale_state=value["scale_state"], translation_unit=value["translation_unit"],
            velocity_unit=value.get("velocity_unit"),
            velocity=value.get("velocity"), velocity_frame=value.get("velocity_frame"),
            map_id=value.get("map_id"), reset_reason=value.get("reset_reason"),
            quality=value.get("quality"),
        )

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))


class LocalizationStreamValidator:
    """Consumer diagnostic for one producer session, not a flight policy.

    A producer owns epochs and persists/increments them across restart, or starts
    a distinct session ID. Create one helper per session; never concatenate
    sessions implicitly. A jump threshold is caller-selected in translation units;
    it detects large displacement, not all possible hidden origin changes.
    """

    def __init__(self, session_id, max_translation_step=None):
        if not isinstance(session_id, str) or not session_id:
            raise ValueError("session_id is required")
        if max_translation_step is not None:
            _finite_vector([max_translation_step], 1, "max_translation_step")
            if max_translation_step <= 0:
                raise ValueError("max_translation_step must be positive")
        self.session_id = session_id
        self.max_translation_step = max_translation_step
        self.previous = None

    def push(self, record):
        if not isinstance(record, LocalizationEstimate):
            raise ValueError("record must be LocalizationEstimate")
        previous = self.previous
        if previous is not None:
            if record.localization_epoch < previous.localization_epoch:
                raise ValueError("epoch decreased within session")
            if record.localization_epoch == previous.localization_epoch:
                if record.tracking_state == TrackingState.RESET:
                    raise ValueError("RESET requires epoch increment")
                if any(getattr(record, key) != getattr(previous, key) for key in
                       ("parent_frame", "child_frame", "map_id", "scale_state", "translation_unit")):
                    raise ValueError("frame/map/scale change requires epoch increment")
                if self.max_translation_step is not None and previous.valid and record.valid:
                    step = math.dist(previous.translation, record.translation)
                    if step > self.max_translation_step:
                        raise ValueError("possible coordinate jump without epoch increment")
        self.previous = record
        return record
