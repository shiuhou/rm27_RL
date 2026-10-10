import hashlib
import json

import pytest

from rm27.perception.vision.localization import (
    LocalizationEstimate,
    ManifestError,
    TimestampEvidence,
    TrackingState,
    validate_manifest,
)


def stamp(value, semantic="UNKNOWN"):
    return TimestampEvidence(value, "UNKNOWN", "UNKNOWN", semantic, "UNVERIFIED")


def estimate(**changes):
    values = dict(
        source_time=stamp(10), publish_time=stamp(20), parent_frame="map",
        child_frame="camera", translation=(1, 2, 3),
        orientation_wxyz=(1, 0, 0, 0), tracking_state=TrackingState.TRACKING,
        valid=True, initialized=True, relocalizing=False, localization_epoch=4,
    )
    values.update(changes)
    return LocalizationEstimate(**values)


def test_round_trip_keeps_unknown_timing_and_has_no_age():
    restored = LocalizationEstimate.from_json(estimate().to_json())
    assert restored == estimate()
    assert "measurement_age" not in restored.to_dict()


def test_schema_and_pose_validation():
    payload = estimate().to_dict()
    payload["schema_version"] = 999
    with pytest.raises(ValueError, match="unsupported"):
        LocalizationEstimate.from_dict(payload)
    with pytest.raises(ValueError, match="non-finite"):
        estimate(translation=(float("nan"), 0, 0))
    with pytest.raises(ValueError, match="non-zero"):
        estimate(orientation_wxyz=(0, 0, 0, 0))
    with pytest.raises(ValueError, match="velocity_frame"):
        estimate(velocity=(1, 0, 0), velocity_unit="UNKNOWN")
    payload = estimate().to_dict()
    del payload["source_time"]["semantic"]
    with pytest.raises(ValueError, match="timestamp missing"):
        LocalizationEstimate.from_dict(payload)


def test_reset_epoch_is_explicit():
    reset = estimate(tracking_state=TrackingState.RESET, valid=False, initialized=False,
                     relocalizing=False, localization_epoch=5, reset_reason="restart")
    assert reset.to_dict()["localization_epoch"] == 5
    assert reset.to_dict()["tracking_state"] == "RESET"


def test_manifest_allows_unknown_timing_and_rejects_bad_frames():
    manifest = {"schema_version": 1, "camera": {"timestamp_source": "unknown"},
                "frames": [{"sequence": 1, "timestamp": {
                    "raw_value": 99, "unit": "UNKNOWN", "clock_domain": "UNKNOWN",
                    "semantic": "UNKNOWN", "evidence_status": "UNVERIFIED"}}]}
    assert validate_manifest(manifest) is manifest
    manifest["frames"].append({"sequence": 1})
    with pytest.raises(ManifestError, match="not increasing"):
        validate_manifest(manifest)


def test_manifest_rejects_fabricated_exposure_and_bad_stride():
    with pytest.raises(ManifestError, match="exposure"):
        validate_manifest({"schema_version": 1, "camera": {
            "timestamp_source": "unknown", "exposure_timestamp_verified": True}})
    with pytest.raises(ManifestError, match="stride"):
        validate_manifest({"schema_version": 1, "camera": {"width": 640, "stride_bytes": 320}})
    # Unverified exposure is preserved as a claim, consistently with the record.
    validate_manifest({"schema_version": 1, "frames": [{"timestamp": {
            "raw_value": 1, "unit": "us", "clock_domain": "host",
            "semantic": "EXPOSURE", "evidence_status": "UNVERIFIED"}}]})


def test_manifest_calibration_hash_is_checked(tmp_path):
    calibration = tmp_path / "camera.yaml"
    calibration.write_text("calibration: test\n")
    digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
    data = {"schema_version": 1, "camera": {
        "calibration_id": "test-only", "calibration_file": calibration.name, "calibration_sha256": digest}}
    assert validate_manifest(data, base_dir=tmp_path) is data
    data["camera"]["calibration_sha256"] = "0" * 64
    with pytest.raises(ManifestError, match="does not match"):
        validate_manifest(data, base_dir=tmp_path)
