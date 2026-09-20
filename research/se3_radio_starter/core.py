"""Pure input/reference logic. No pygame, MuJoCo, or hardware required."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

CHANNELS = ("right", "forward", "up", "yaw_right")


def vector(values: Sequence[float], length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {length} finite numbers.")
    return result


@dataclass(frozen=True)
class AxisCalibration:
    index: int
    center: float
    low: float
    high: float
    sign: int
    deadzone: float = 0.08

    def __post_init__(self) -> None:
        if not isinstance(self.index, int) or self.index < 0:
            raise ValueError("Axis index must be a nonnegative integer.")
        if not all(math.isfinite(x) for x in (self.center, self.low, self.high, self.deadzone)):
            raise ValueError("Calibration contains a nonfinite value.")
        if not self.low < self.center < self.high:
            raise ValueError("The neutral point must be BETWEEN both measured endpoints.")
        if min(self.center - self.low, self.high - self.center) < 0.10:
            raise ValueError("Axis travel is too small on one side of neutral.")
        if self.sign not in (-1, 1) or not 0 <= self.deadzone < 0.5:
            raise ValueError("Invalid sign or deadzone.")

    def normalize(self, raw: float) -> float:
        if not math.isfinite(raw):
            raise ValueError("Nonfinite joystick reading; stopping input.")
        delta = raw - self.center
        span = self.high - self.center if delta >= 0 else self.center - self.low
        value = float(np.clip(delta / span, -1.0, 1.0)) * self.sign
        magnitude = abs(value)
        if magnitude <= self.deadzone:
            return 0.0
        return math.copysign((magnitude - self.deadzone) / (1.0 - self.deadzone), value)


def infer_axis(center: Sequence[float], positive: Sequence[float],
               negative: Sequence[float], used: set[int]) -> AxisCalibration:
    """Infer one stick axis from neutral and opposite held endpoints."""
    c = np.asarray(center, dtype=float)
    if c.ndim != 1 or len(c) == 0 or not np.isfinite(c).all():
        raise ValueError("Invalid neutral sample.")
    p = vector(positive, len(c), "positive")
    n = vector(negative, len(c), "negative")
    dp, dn = p - c, n - c
    opposite = dp * dn < 0
    score = np.where(opposite, np.minimum(abs(dp), abs(dn)), 0.0)
    # Do not hide duplicate mappings by removing used axes before selection.
    index = int(np.argmax(score))
    if score[index] < 0.20:
        raise ValueError("No bidirectional axis found. Use opposite endpoints, with neutral in the middle.")
    if index in used:
        raise ValueError("This axis was already assigned. Move ONLY the requested stick axis.")
    others = np.delete(score, index)
    if len(others) and np.max(others) > 0.65 * score[index]:
        raise ValueError("Multiple axes moved together. Repeat without moving the other stick axis.")
    return AxisCalibration(index, float(c[index]), float(min(p[index], n[index])),
                           float(max(p[index], n[index])), 1 if dp[index] > 0 else -1)


@dataclass
class RadioProfile:
    name: str
    guid: str
    axis_count: int
    axes: dict[str, AxisCalibration]

    def __post_init__(self) -> None:
        if not isinstance(self.axis_count, int) or self.axis_count < 4:
            raise ValueError("At least four independent joystick axes are required.")
        if set(self.axes) != set(CHANNELS):
            raise ValueError(f"Profile must contain exactly these channels: {CHANNELS}")
        indices = [self.axes[key].index for key in CHANNELS]
        if len(set(indices)) != 4 or max(indices) >= self.axis_count:
            raise ValueError("Axis mappings must be distinct and within the device axis count.")

    def validate_device(self, guid: str, axis_count: int) -> None:
        if self.guid != guid or self.axis_count != axis_count:
            raise ValueError("Device identity/axis count changed. Re-run calibration with this connection.")

    def normalize(self, raw: Sequence[float]) -> np.ndarray:
        data = vector(raw, self.axis_count, "raw axes")
        return np.array([self.axes[key].normalize(float(data[self.axes[key].index]))
                         for key in CHANNELS])

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        content = {"version": 1, "name": self.name, "guid": self.guid,
                   "axis_count": self.axis_count,
                   "axes": {name: asdict(value) for name, value in self.axes.items()}}
        if path.exists():
            path.with_suffix(path.suffix + ".bak").write_bytes(path.read_bytes())
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(content, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(path)

    @classmethod
    def load(cls, path: Path) -> "RadioProfile":
        obj = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if obj.get("version") != 1:
            raise ValueError("Unsupported profile version. Re-run calibration.")
        return cls(obj["name"], obj["guid"], obj["axis_count"],
                   {name: AxisCalibration(**values) for name, values in obj["axes"].items()})


@dataclass(frozen=True)
class MotionSettings:
    start_position: tuple[float, float, float] = (0.0, 0.0, 0.30)
    minimum_position: tuple[float, float, float] = (-1.5, -1.5, 0.15)
    maximum_position: tuple[float, float, float] = (1.5, 1.5, 1.5)
    horizontal_speed: float = 0.40
    vertical_speed: float = 0.25
    acceleration: float = 1.0
    yaw_speed_deg: float = 45.0
    yaw_acceleration_deg: float = 120.0

    def __post_init__(self) -> None:
        start = vector(self.start_position, 3, "start_position")
        lower = vector(self.minimum_position, 3, "minimum_position")
        upper = vector(self.maximum_position, 3, "maximum_position")
        if np.any(lower >= upper) or np.any(start < lower) or np.any(start > upper):
            raise ValueError("Invalid target bounds or initial position.")
        for name in ("horizontal_speed", "vertical_speed", "acceleration", "yaw_speed_deg",
                     "yaw_acceleration_deg"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive.")

    @classmethod
    def load(cls, path: Path) -> "MotionSettings":
        return cls(**json.loads(Path(path).read_text(encoding="utf-8-sig")))


class PositionReference:
    """Stick -> bounded reference rate -> integrated position; NOT direct attitude control.

    Coordinates: world +X = right, +Y = forward, +Z = up.
    Heading is the desired BODY +Y direction, matching the author's controller.
    """
    def __init__(self, settings: MotionSettings):
        self.settings = settings
        self.lower = vector(settings.minimum_position, 3, "minimum")
        self.upper = vector(settings.maximum_position, 3, "maximum")
        self.reset()

    def reset(self) -> None:
        self.position = vector(self.settings.start_position, 3, "start").copy()
        self.velocity = np.zeros(3)
        self.yaw = 0.0
        self.yaw_rate = 0.0

    def hold(self) -> None:
        """Freeze the current TARGET, not the current aircraft position."""
        self.velocity[:] = 0.0
        self.yaw_rate = 0.0

    @property
    def heading(self) -> np.ndarray:
        return np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.0])

    def update(self, command: Sequence[float], dt: float, enabled: bool = True) -> None:
        u = vector(command, 4, "command")
        if not math.isfinite(dt) or not 0 < dt <= 0.05:
            raise ValueError("Use a simulation step in (0, 0.05] seconds, not a large wall-clock gap.")
        if not enabled:
            self.hold()
            return
        u = np.clip(u, -1.0, 1.0)
        horizontal = u[:2] / max(1.0, float(np.linalg.norm(u[:2])))
        desired = np.r_[horizontal * self.settings.horizontal_speed,
                        u[2] * self.settings.vertical_speed]
        # Limit horizontal acceleration as a vector; vertical acceleration separately.
        dv = desired - self.velocity
        horizontal_step = self.settings.acceleration * dt
        dv[:2] *= min(1.0, horizontal_step / max(float(np.linalg.norm(dv[:2])), 1e-12))
        dv[2] = np.clip(dv[2], -horizontal_step, horizontal_step)
        self.velocity += dv
        proposal = self.position + dt * self.velocity
        clipped = np.clip(proposal, self.lower, self.upper)
        self.velocity[np.abs(proposal - clipped) > 1e-12] = 0.0
        self.position = clipped
        desired_yaw_rate = -u[3] * math.radians(self.settings.yaw_speed_deg)
        step = math.radians(self.settings.yaw_acceleration_deg) * dt
        self.yaw_rate += float(np.clip(desired_yaw_rate - self.yaw_rate, -step, step))
        self.yaw = (self.yaw + self.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi


@dataclass
class InputGate:
    enabled: bool = False
    disconnected: bool = False

    def toggle(self, command: Sequence[float]) -> str:
        u = vector(command, 4, "command")
        if self.disconnected:
            self.enabled = False
            return "DISCONNECTED: reconnect, then restart the program."
        if self.enabled:
            self.enabled = False
            return "HOLD: target movement disabled."
        if np.max(np.abs(u)) > 0.02:
            return "Center ALL four axes first (including throttle), then press G."
        self.enabled = True
        return "ACTIVE: radio now moves the position/heading target."

    def stop(self, disconnected: bool = False) -> None:
        self.enabled = False
        self.disconnected = self.disconnected or disconnected


def quat_wxyz_to_xyzw(quat: Sequence[float]) -> np.ndarray:
    q = vector(quat, 4, "quaternion")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("Zero quaternion.")
    return q[[1, 2, 3, 0]] / norm


def demo_target(t: float) -> np.ndarray:
    """Four six-second point-hold phases, then continue holding home."""
    if t < 6:
        return np.array([0.0, 0.0, 0.30])
    if t < 12:
        return np.array([0.40, 0.0, 0.30])
    if t < 18:
        return np.array([0.40, 0.40, 0.30])
    return np.array([0.0, 0.0, 0.30])
