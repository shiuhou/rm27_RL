#!/usr/bin/env python3
"""Freeze a provenance-preserving VSL-2 dataset view over VSL-1 artifacts."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vsl1", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.vsl1.resolve()
    out = args.output.resolve()
    if not (source / "video_stream.json").is_file():
        raise SystemExit(f"not a VSL-1 artifact directory: {source}")
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output: {out}")
    out.mkdir(parents=True, exist_ok=True)
    stream = json.loads((source / "video_stream.json").read_text())
    quality = list(csv.DictReader((source / "quality" / "frame_quality.csv").open()))
    intervals = json.loads((source / "intervals.json").read_text())
    source_video = Path(stream["source"]["path"])
    if not source_video.is_file() or sha256(source_video) != stream["source"]["sha256"]:
        raise ValueError("source MP4 missing or SHA-256 mismatch")
    upstream_files = ["video_stream.json", "quality/frame_quality.csv", "intervals.json"]
    upstream_identity = {name: sha256(source / name) for name in upstream_files}
    image_hashes = {}
    for item in quality:
        image = (source / item["image_file"]).resolve()
        if not image.is_relative_to(source) or not image.is_file():
            raise ValueError(f"missing or nonlocal referenced image: {item['image_file']}")
        image_hashes[item["image_file"]] = sha256(image)
    (out / "image_hashes.json").write_text(json.dumps(image_hashes, indent=2) + "\n")
    rows = []
    for item in quality:
        rows.append({
            "sample_index": len(rows),
            "source_frame_index": int(item["frame_index"]),
            "encoded_pts_s": float(item["pts_time_s"]),
            "image": item["image_file"],
            "width": stream["video_stream"]["width"],
            "height": stream["video_stream"]["height"],
            "pixel_format": "decoded BGR8; JPEG quality 95 derived image",
            "calibration_id": "UNKNOWN",
            "camera_mode_id": "UNKNOWN",
            "timestamp_semantic": "ENCODED_STREAM_PTS",
            "timestamp_unit": "seconds",
            "timestamp_clock_domain": "UNKNOWN",
            "quality": {key: item[key] for key in (
                "laplacian_variance", "feature_count", "empty_feature_cells",
                "underexposed_fraction", "overexposed_fraction", "median_flow_px",
                "rotation_proxy_deg")},
        })
    manifest = {
        "schema_version": 1,
        "dataset_id": "rm27-vsl2-0921-encoded-sampled-v1",
        "status": "CALIBRATION_REQUIRED",
        "source": {
            "artifact_directory": str(source),
            "upstream_metadata_sha256": upstream_identity,
            "image_path_base": "source.artifact_directory",
            "image_hashes_file": "image_hashes.json",
            "source_hash_verified": True,
            "source_video": stream["source"]["path"],
            "source_sha256": stream["source"]["sha256"],
            "producer_git_commit": stream["source"]["git_commit"],
            "extraction_tool": stream["decoder"],
            "derived_image_policy": stream["sampling"]["derived_images"],
        },
        "observed_geometry": {
            "width": stream["video_stream"]["width"],
            "height": stream["video_stream"]["height"],
            "nominal_fps": stream["video_stream"]["nominal_fps"],
            "crop_resize": "UNKNOWN",
            "lens_id": "UNKNOWN",
            "camera_mode_id": "UNKNOWN",
        },
        "calibration": {
            "calibration_id": "UNKNOWN",
            "status": "UNVERIFIED_CALIBRATION",
            "file": None,
            "sha256": None,
        },
        "timing": {
            "field": "ENCODED_STREAM_PTS",
            "exposure_timestamp": "UNKNOWN",
            "vin_pts": "UNKNOWN",
            "host_receive_time": "UNKNOWN",
            "sensor_capture_drop_rate": "UNKNOWN",
        },
        "limitations": [
            "encoded video has no sensor/VIN sequence evidence",
            "encoded PTS is not exposure or start-of-frame timing",
            "camera mode crop/lens/focus provenance is incomplete",
            "derived images are JPEG encodes of decoded frames",
        ],
        "frames_file": "frames.csv",
        "intervals_file": "intervals.json",
        "frame_count": len(rows),
    }
    (out / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    with (out / "frames.csv").open("w", newline="") as stream_out:
        fields = ["sample_index", "source_frame_index", "encoded_pts_s", "image", "width", "height",
                  "pixel_format", "calibration_id", "camera_mode_id", "timestamp_semantic",
                  "timestamp_unit", "timestamp_clock_domain", "quality_json"]
        writer = csv.DictWriter(stream_out, fieldnames=fields); writer.writeheader()
        for row in rows:
            out_row = dict(row)
            out_row["quality_json"] = json.dumps(out_row.pop("quality"), separators=(",", ":"))
            writer.writerow(out_row)
    (out / "intervals.json").write_text(json.dumps(intervals, indent=2) + "\n")
    subset_rows = {}
    for label in sorted({item["label"] for item in intervals}):
        selected = [row for row, item in zip(rows, quality)
                    if any(i["label"] == label and i["start_time_s"] <= float(row["encoded_pts_s"]) <= i["end_time_s"] for i in intervals)]
        subset_rows[label] = {"label": label, "frame_count": len(selected),
                              "source_frame_indices": [row["source_frame_index"] for row in selected],
                              "reason": next(i["reason"] for i in intervals if i["label"] == label),
                              "evidence": "VSL-1A offline decoded-frame diagnostic"}
    subset_rows["all_decoded_sampled"] = {"label": "all_decoded_sampled", "frame_count": len(rows),
                                          "source_frame_indices": [row["source_frame_index"] for row in rows],
                                          "reason": "complete deterministic sample at configured step",
                                          "evidence": "VSL-1A derived artifact"}
    (out / "subsets.json").write_text(json.dumps(subset_rows, indent=2) + "\n")
    print(json.dumps({"output": str(out), "frames": len(rows), "subsets": list(subset_rows)}))


if __name__ == "__main__":
    main()
