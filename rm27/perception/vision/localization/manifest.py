"""Validation for canonical localization experiment metadata.

The validator is intentionally conservative: unknown timing is valid evidence,
and unverified interpretation claims remain claims, never timing facts.
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
    from .schema import TimestampEvidence
    try:
        TimestampEvidence.from_dict(timing)
    except (TypeError, ValueError) as exc:
        raise ManifestError(f"{label}: {exc}") from exc
    if "monotonic" in timing and type(timing["monotonic"]) is not bool:
        raise ManifestError(f"{label}.monotonic must be boolean")


def _geometry(record, stride_key):
    for key in ("width", "height", stride_key, "uv_stride_bytes", "frame_bytes"):
        value = record.get(key)
        if value is not None and (type(value) is not int or value <= 0):
            raise ManifestError(f"{key} must be a positive integer")
    width, height, stride = (record.get(k) for k in ("width", "height", stride_key))
    fmt = record.get("pixel_format")
    if stride is not None:
        if width is None or fmt not in ("RGB888", "GRAY8", "NV21"):
            raise ManifestError("stride validation requires width and supported pixel_format")
        if stride < width * (3 if fmt == "RGB888" else 1):
            raise ManifestError("stride is too small for pixel_format")
    if fmt == "NV21":
        # One Y plane, then interleaved VU plane; byte strides, same row pitch.
        if width is None or height is None or width % 2 or height % 2:
            raise ManifestError("NV21 requires even width and height")
        if stride is not None and stride % 2:
            raise ManifestError("NV21 stride must be even")
        if record.get("uv_stride_bytes") is not None and record["uv_stride_bytes"] != stride:
            raise ManifestError("NV21 requires equal Y and VU stride")
        if record.get("frame_bytes") is not None:
            if stride is None or record["frame_bytes"] < stride * height * 3 // 2:
                raise ManifestError("NV21 frame_bytes does not contain both planes")


def validate_experiment_metadata(data):
    """Experiment metadata only; does not inspect frame files or qualify a dataset."""
    if not isinstance(data, dict):
        raise ManifestError("manifest must be an object")
    if type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ManifestError("unsupported manifest schema_version")
    try:
        json.dumps(data, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ManifestError("metadata must be strict JSON") from exc
    camera = data.get("camera")
    if camera is not None:
        if not isinstance(camera, dict):
            raise ManifestError("camera must be an object")
        _geometry(camera, "stride_bytes")
        if camera.get("timestamp_source") is not None and not isinstance(camera["timestamp_source"], str):
            raise ManifestError("timestamp_source must be a string")
        verified = camera.get("exposure_timestamp_verified", False)
        if type(verified) is not bool:
            raise ManifestError("exposure_timestamp_verified must be boolean")
        if verified and (camera.get("timestamp_source") or "UNKNOWN").upper() == "UNKNOWN":
            raise ManifestError("verified exposure timestamp requires a verified timestamp source")
        if verified and not camera.get("timestamp_source"):
            raise ManifestError("verified exposure timestamp requires a verified timestamp source")
        for key in ("exposure_us", "gain", "requested_fps", "observed_fps"):
            if camera.get(key) is not None:
                _finite(camera[key], key)
    return data


def validate_frame_records(frames):
    """Validate supplied frame metadata, not image existence or calibration.

    Ordering uses only explicitly monotonic, VERIFIED interpretations with known
    compatible units and identical domain + event semantic. Unknowns are retained.
    """
    if not isinstance(frames, list):
        raise ManifestError("frames must be a list")
    previous_sequence = None
    clocks = {}
    factors = {"ns": 1, "us": 1000, "ms": 1000000, "s": 1000000000}
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ManifestError(f"frames[{index}] must be an object")
        if "sequence" in frame:
            sequence = frame["sequence"]
            if type(sequence) is not int or sequence < 0:
                raise ManifestError("sequence must be a non-negative integer")
            if previous_sequence is not None and sequence <= previous_sequence:
                raise ManifestError("sequence is not increasing (one sequence space per frame list)")
            previous_sequence = sequence
        _geometry(frame, "stride")
        if "timestamp" in frame:
            t = frame["timestamp"]
            _validate_timing(t, f"frames[{index}].timestamp")
            if (t.get("monotonic") is True and t["evidence_status"] == "VERIFIED"
                    and t["unit"] in factors and t["clock_domain"].upper() != "UNKNOWN"
                    and t["semantic"].upper() != "UNKNOWN"):
                key = (t["clock_domain"], t["semantic"])
                current = t["raw_value"] * factors[t["unit"]]
                if key in clocks and current < clocks[key]:
                    raise ManifestError("timestamp is not monotonic in the same known clock/event")
                clocks[key] = current
        for key in ("exposure_us", "gain"):
            if frame.get(key) is not None:
                _finite(frame[key], key)
    return frames


def validate_calibration_reference(reference, base_dir=None):
    """Validate a claimed file/id/hash bundle; no intrinsic-fit qualification.

    A claim requires all three fields. UNKNOWN/null bundles are representable
    while capture is required. Hash is over original bytes, paths relative to
    the manifest directory. No current-directory fallback when base is omitted.
    """
    if not isinstance(reference, dict):
        raise ManifestError("calibration reference must be an object")
    identifier = reference.get("calibration_id")
    file = reference.get("calibration_file", reference.get("file"))
    digest = reference.get("calibration_hash", reference.get("calibration_sha256", reference.get("sha256")))
    for keys in (("calibration_file", "file"), ("calibration_hash", "calibration_sha256", "sha256")):
        supplied = [reference[k] for k in keys if reference.get(k) is not None]
        if supplied and any(v != supplied[0] for v in supplied):
            raise ManifestError("conflicting calibration reference aliases")
    known = lambda v: v is not None and v != "UNKNOWN"
    available = reference.get("status") in ("AVAILABLE", "CALIBRATED", "VERIFIED")
    if not available and not any(known(v) for v in (identifier, file, digest)):
        return reference
    if not all(isinstance(v, str) and v and known(v) for v in (identifier, file, digest)):
        raise ManifestError("claimed calibration requires calibration_id, calibration_file and calibration_hash")
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ManifestError("calibration hash must be lowercase SHA-256")
    path = Path(file)
    if not path.is_absolute():
        if base_dir is None:
            raise ManifestError("base_dir required to resolve calibration file")
        path = Path(base_dir) / path
    if not path.is_file():
        raise ManifestError(f"missing calibration file: {path}")
    if _hash_file(path) != digest:
        raise ManifestError("calibration hash does not match calibration_file")
    return reference


def validate_manifest(data, base_dir=None):
    """Compatibility composition of the three narrow validators; not qualification."""
    validate_experiment_metadata(data)
    validate_frame_records(data.get("frames", []))
    for reference in (data, data.get("camera"), data.get("calibration")):
        if reference is not None:
            validate_calibration_reference(reference, base_dir)
    return data
