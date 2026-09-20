"""GazeboPoseSource - read world poses from a running Gazebo.

Streams `/world/<world>/pose/info` through `gz topic -e` and parses the
protobuf text format into plain dicts:

    {model_name: {"pos": (x, y, z), "quat": (x, y, z, w)}}

This module knows NOTHING about guidance, MAVLink, or TargetEstimate; it
is a pure Gazebo-side pose mirror. Frame conventions live in
docs/frames.md (Gazebo world is ENU, yaw CCW-positive).
"""

import re
import subprocess
import threading

_FLOAT = r"[-+0-9.eEnaif]+"  # also tolerates nan/inf, rejected later
_LINE = re.compile(rf"^\s*([a-z_]+):\s*({_FLOAT}|\"[^\"]*\")\s*$")


def _section(lines, start_idx, keyword):
    """Return (inner_lines, end_idx) of `keyword { ... }` at start_idx."""
    assert lines[start_idx].strip() == f"{keyword} {{"
    depth = 1
    inner = []
    idx = start_idx + 1
    while idx < len(lines) and depth > 0:
        line = lines[idx]
        if line.rstrip().endswith("{"):
            depth += 1
            inner.append(line)  # keep nested markers for clarity
        elif line.strip() == "}":
            depth -= 1
            if depth == 0:
                return inner, idx
            inner.append(line)
        else:
            inner.append(line)
        idx += 1
    raise ValueError(f"unbalanced braces in section {keyword!r}")


def _fields(lines):
    """Parse flat `key: value` lines; ignores nested sections."""
    out = {}
    for line in lines:
        m = _LINE.match(line)
        if m and "{" not in line:
            key, raw = m.group(1), m.group(2)
            if raw.startswith('"'):
                out[key] = raw.strip('"')
            else:
                try:
                    out[key] = float(raw)
                except ValueError:
                    out[key] = raw
    return out


def parse_pose_message(text):
    """Parse one gz.msgs.Pose_V text message into the poses dict."""
    lines = text.splitlines()
    poses = {}
    idx = 0
    while idx < len(lines):
        if lines[idx].strip() == "pose {":
            inner, end = _section(lines, idx, "pose")
            pose = {}
            for i, line in enumerate(inner):
                stripped = line.strip()
                if stripped == "position {":
                    sub, _ = _section(inner, i, "position")
                    pose["pos"] = _fields(sub)
                elif stripped == "orientation {":
                    sub, _ = _section(inner, i, "orientation")
                    pose["quat"] = _fields(sub)
                elif stripped.startswith("name:"):
                    pose["name"] = _fields([line]).get("name")
            if "name" in pose:
                pos = pose.get("pos", {})
                quat = pose.get("quat", {})
                poses[pose["name"]] = {
                    "pos": (
                        float(pos.get("x", 0.0)),
                        float(pos.get("y", 0.0)),
                        float(pos.get("z", 0.0)),
                    ),
                    "quat": (
                        float(quat.get("x", 0.0)),
                        float(quat.get("y", 0.0)),
                        float(quat.get("z", 0.0)),
                        float(quat.get("w", 1.0)),
                    ),
                }
            idx = end + 1
        else:
            idx += 1
    return poses


class GazeboPoseSource:
    """Background `gz topic -e` reader keeping the latest poses dict."""

    def __init__(self, world="iris_runway", gz_bin="gz"):
        self._topic = f"/world/{world}/pose/info"
        self._gz_bin = gz_bin
        self._proc = None
        self._thread = None
        self._buffer = []
        self._lock = threading.Lock()
        self._poses = {}
        self._error = None  # reader thread exception, if it died

    def __enter__(self):
        self._proc = subprocess.Popen(
            [self._gz_bin, "topic", "-e", "-t", self._topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(
            target=self._reader, daemon=True
        )
        self._thread.start()
        return self

    def __exit__(self, *exc):
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def _reader(self):
        try:
            for line in self._proc.stdout:
                with self._lock:
                    self._buffer.append(line)
                    if line.strip() == "header {":
                        self._flush()
        except Exception as exc:  # pragma: no cover - diagnostic path
            self._error = exc

    def _flush(self):
        if not self._buffer:
            return
        text = "".join(self._buffer[:-1])  # drop the new "header {" line
        self._buffer = self._buffer[-1:]
        if "pose {" in text:
            poses = parse_pose_message(text)
            if poses:
                self._poses = poses

    def poses(self):
        with self._lock:
            return dict(self._poses)

    def error(self):
        return self._error
