"""The versioned ego-localization record used by future SLAM/VIO adapters."""

from dataclasses import dataclass
from enum import Enum
import json
import math
from typing import Any, Mapping

LOCALIZATION_SCHEMA_VERSION = 1


class TrackingState(str, Enum):
    UNINITIALIZED = "UNINITIALIZED"
    INITIALIZING = "INITIALIZING"
    TRACKING = "TRACKING"
    RELOCALIZING = "RELOCALIZING"
    LOST = "LOST"
    RESET = "RESET"


@dataclass(frozen=True)
class TimestampEvidence:
    """A timestamp plus its independently verified interpretation."""

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
    result = tuple(float(item) for item in value)
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
    velocity: tuple[float, float, float] | None = None
    velocity_frame: str | None = None
    map_id: str | None = None
    reset_reason: str | None = None
    quality: Mapping[str, Any] | None = None
    schema_version: int = LOCALIZATION_SCHEMA_VERSION

    def __post_init__(self):
        if self.schema_version != LOCALIZATION_SCHEMA_VERSION:
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
        norm = math.sqrt(sum(item * item for item in self.orientation_wxyz))
        if norm < 1e-9:
            raise ValueError("orientation_wxyz must be non-zero")
        if not isinstance(self.tracking_state, TrackingState):
            try:
                object.__setattr__(self, "tracking_state", TrackingState(self.tracking_state))
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid tracking_state") from exc
        if not isinstance(self.localization_epoch, int) or isinstance(self.localization_epoch, bool) or self.localization_epoch < 0:
            raise ValueError("localization_epoch must be a non-negative integer")
        if self.velocity is not None:
            object.__setattr__(self, "velocity", _finite_vector(self.velocity, 3, "velocity"))
            if not isinstance(self.velocity_frame, str) or not self.velocity_frame:
                raise ValueError("velocity_frame is required when velocity is present")
        elif self.velocity_frame is not None:
            raise ValueError("velocity_frame requires velocity")
        if self.quality is not None and not isinstance(self.quality, Mapping):
            raise ValueError("quality must be an object when present")

    def to_dict(self):
        data = {
            "schema_version": self.schema_version,
            "source_time": self.source_time.to_dict(),
            "publish_time": self.publish_time.to_dict(),
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "translation": list(self.translation),
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
        for key in ("map_id", "reset_reason", "quality"):
            value = getattr(self, key)
            if value is not None:
                data[key] = value
        return data

    def to_json(self):
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]):
        if not isinstance(value, Mapping):
            raise ValueError("LocalizationEstimate must be an object")
        version = value.get("schema_version", -1)
        if version != LOCALIZATION_SCHEMA_VERSION:
            raise ValueError(f"unsupported LocalizationEstimate schema_version {version}")
        required = ("source_time", "publish_time", "parent_frame", "child_frame",
                    "translation", "orientation_wxyz", "tracking_state", "valid",
                    "initialized", "relocalizing", "localization_epoch")
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
            velocity=value.get("velocity"), velocity_frame=value.get("velocity_frame"),
            map_id=value.get("map_id"), reset_reason=value.get("reset_reason"),
            quality=value.get("quality"),
        )

    @classmethod
    def from_json(cls, text):
        return cls.from_dict(json.loads(text))
