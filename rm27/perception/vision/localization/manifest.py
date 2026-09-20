"""Validation for canonical localization experiment metadata.

The validator is intentionally conservative: unknown timing is valid evidence,
while a claim of exposure timing without verification is rejected.
"""

from hashlib import sha256
import json
from pathlib import Path
import re


class ManifestError(ValueError):
    pass


def load_manifest(path):
    path = Path(path)
    text = path.read_text()
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        try:
            import yaml
        except ImportError as exc:
            data = _load_simple_yaml(text)
        else:
            data = yaml.safe_load(text)
    validate_manifest(data, base_dir=path.parent)
    return data


def _load_simple_yaml(text):
    """Parse the starter manifest's map/scalar YAML without a dependency.

    This intentionally supports only the subset used by the checked-in
    experiment manifest: indented mappings and scalar values. A full YAML
    parser remains preferred when installed.
    """
    root = {}
    stack = [(-1, root)]
    for line_number, raw in enumerate(text.splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        if indent % 2:
            raise ManifestError(f"unsupported YAML indentation on line {line_number}")
        content = raw.strip()
        if content.startswith("-") or ":" not in content:
            raise ManifestError(f"unsupported YAML construct on line {line_number}")
        key, raw_value = content.split(":", 1)
        key = key.strip()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]*", key):
            raise ManifestError(f"invalid YAML key on line {line_number}")
        while stack[-1][0] >= indent:
            stack.pop()
        parent = stack[-1][1]
        value = raw_value.strip()
        if not value:
            child = {}
            parent[key] = child
            stack.append((indent, child))
            continue
        if value in ("null", "~"):
            parsed = None
        elif value.lower() in ("true", "false"):
            parsed = value.lower() == "true"
        elif (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
            parsed = value[1:-1]
        else:
            try:
                parsed = int(value, 10)
            except ValueError:
                try:
                    parsed = float(value)
                except ValueError:
                    parsed = value
        parent[key] = parsed
    return root


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManifestError(f"{label} must be numeric")
    if value != value or value in (float("inf"), float("-inf")):
        raise ManifestError(f"{label} must be finite")


def _hash_file(path):
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_timing(timing, label):
    if not isinstance(timing, dict):
        raise ManifestError(f"{label} must be an object")
    for key in ("raw_value", "unit", "clock_domain", "semantic", "evidence_status"):
        if key not in timing:
            raise ManifestError(f"{label} missing {key}")
    if isinstance(timing["raw_value"], bool) or not isinstance(timing["raw_value"], int) or timing["raw_value"] < 0:
        raise ManifestError(f"{label}.raw_value must be a non-negative integer")
    for key in ("unit", "clock_domain", "semantic", "evidence_status"):
        if not isinstance(timing[key], str) or not timing[key]:
            raise ManifestError(f"{label}.{key} must be a non-empty string")
    if "EXPOSURE" in timing["semantic"].upper() and timing["evidence_status"].upper() != "VERIFIED":
        raise ManifestError(f"{label} claims exposure timing without verified evidence")


def validate_manifest(data, base_dir=None):
    if not isinstance(data, dict):
        raise ManifestError("manifest must be an object")
    if data.get("schema_version") not in (1,):
        raise ManifestError(f"unsupported manifest schema_version {data.get('schema_version')}")
    camera = data.get("camera")
    if camera is not None:
        if not isinstance(camera, dict):
            raise ManifestError("camera must be an object")
        for key in ("width", "height", "stride_bytes"):
            if camera.get(key) is not None and (not isinstance(camera[key], int) or camera[key] <= 0):
                raise ManifestError(f"camera.{key} must be a positive integer")
        if camera.get("width") and camera.get("stride_bytes") and camera["stride_bytes"] < camera["width"]:
            raise ManifestError("camera.stride_bytes cannot be less than width")
        if camera.get("exposure_timestamp_verified") and camera.get("timestamp_source") in (None, "unknown"):
            raise ManifestError("verified exposure timestamp requires a verified timestamp source")
    frames = data.get("frames", [])
    if not isinstance(frames, list):
        raise ManifestError("frames must be a list")
    previous_sequence = previous_time = None
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ManifestError(f"frames[{index}] must be an object")
        if "sequence" in frame:
            sequence = frame["sequence"]
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ManifestError(f"frames[{index}].sequence is invalid")
            if previous_sequence is not None and sequence <= previous_sequence:
                raise ManifestError(f"frames[{index}].sequence is not increasing")
            previous_sequence = sequence
        for key in ("width", "height", "stride"):
            if key in frame and frame[key] is not None and (not isinstance(frame[key], int) or frame[key] <= 0):
                raise ManifestError(f"frames[{index}].{key} is invalid")
        if frame.get("width") and frame.get("stride") and frame["stride"] < frame["width"]:
            raise ManifestError(f"frames[{index}] stride is less than width")
        if "timestamp" in frame:
            _validate_timing(frame["timestamp"], f"frames[{index}].timestamp")
            current_time = frame["timestamp"]["raw_value"]
            if previous_time is not None and frame["timestamp"].get("monotonic", True) and current_time < previous_time:
                raise ManifestError(f"frames[{index}].timestamp is not monotonic")
            previous_time = current_time
        for key in ("exposure_us", "gain"):
            if key in frame and frame[key] is not None:
                _finite(frame[key], f"frames[{index}].{key}")
    calibration_file = camera.get("calibration_file") if isinstance(camera, dict) else None
    calibration_hash = camera.get("calibration_sha256") if isinstance(camera, dict) else None
    if calibration_file and calibration_hash and base_dir is not None:
        path = Path(base_dir) / calibration_file
        if not path.exists():
            raise ManifestError(f"missing calibration file: {path}")
        if _hash_file(path) != calibration_hash:
            raise ManifestError("calibration_sha256 does not match calibration_file")
    return data
