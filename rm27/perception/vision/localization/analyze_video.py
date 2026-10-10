#!/usr/bin/env python3
"""Deterministic VSL-1A qualification for an encoded video artifact.

This tool never interprets container PTS as exposure or sensor timing. It
decodes the source read-only, records encoded-stream observations available via
OpenCV, and writes derived diagnostics to a new output directory.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import shlex
import platform

import cv2
import numpy as np


def git_commit():
    result = subprocess.run(["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parents[4],
                            text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else "UNKNOWN"


def source_git_status(source):
    root = Path(__file__).resolve().parents[4]
    if not source.is_relative_to(root):
        return "EXTERNAL_SOURCE; see SHA-256"
    result = subprocess.run(["git", "status", "--short", "--", str(source)], cwd=root,
                            text=True, capture_output=True)
    return result.stdout if result.returncode == 0 else "UNKNOWN"


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else None


def stats(values):
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None,
                "p99": None, "min": None, "max": None, "stddev": None}
    a = np.asarray(values, dtype=np.float64)
    return {"count": int(a.size), "mean": float(a.mean()), "median": float(np.median(a)),
            "p95": percentile(values, 95), "p99": percentile(values, 99),
            "min": float(a.min()), "max": float(a.max()), "stddev": float(a.std())}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_simple_yaml(path, value, indent=0):
    """Write the simple scalar/list/map subset used by our manifests."""
    with path.open("w") as out:
        def emit(item, level):
            pad = " " * level
            if isinstance(item, dict):
                for key, val in item.items():
                    if isinstance(val, (dict, list)):
                        out.write(f"{pad}{key}:\n")
                        emit(val, level + 2)
                    else:
                        out.write(f"{pad}{key}: {json.dumps(val, ensure_ascii=False)}\n")
            elif isinstance(item, list):
                for val in item:
                    if isinstance(val, dict):
                        first = True
                        for key, sub in val.items():
                            if first:
                                out.write(f"{pad}- {key}: {json.dumps(sub, ensure_ascii=False)}\n")
                                first = False
                            else:
                                out.write(f"{pad}  {key}: {json.dumps(sub, ensure_ascii=False)}\n")
                    else:
                        out.write(f"{pad}- {json.dumps(val, ensure_ascii=False)}\n")
        emit(value, indent)


def acceptance_status(declared_count, decoded_count, timestamps, large_gaps,
                      write_failures, source_unchanged, timing_reference_available=True):
    """PASS qualifies encoded diagnostics only; unknown counts cannot prove EOF."""
    premature = declared_count > 0 and decoded_count < declared_count
    timing = (timing_reference_available and len(timestamps) >= 2 and all(math.isfinite(t) for t in timestamps)
              and all(b > a for a, b in zip(timestamps, timestamps[1:]))
              and large_gaps == 0)
    integrity = (declared_count > 0 and decoded_count == declared_count
                 and source_unchanged and not write_failures)
    return {"execution_completed": True, "declared_frame_count": declared_count,
            "decoded_frame_count": decoded_count, "premature_decode_failure": premature,
            "decode_completeness_known": declared_count > 0,
            "timing_checks_passed": timing, "integrity_checks_passed": integrity,
            "write_failures": write_failures, "source_unchanged": source_unchanged,
            "passed": timing and integrity}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--sample-step", type=int, default=6,
                        help="extract/analyse every Nth frame (default: 6, about 30 Hz at 180 fps)")
    parser.add_argument("--large-gap-factor", type=float, default=1.5)
    args = parser.parse_args(argv)
    if args.sample_step < 1 or not math.isfinite(args.large_gap_factor) or args.large_gap_factor <= 1:
        parser.error("sample-step must be positive and large-gap-factor must exceed 1")
    source = args.video.resolve()
    if not source.is_file():
        parser.error(f"video does not exist: {source}")
    out = args.out.resolve()
    if out.exists() and any(out.iterdir()):
        parser.error(f"refusing to overwrite non-empty output: {out}")
    (out / "frames").mkdir(parents=True, exist_ok=True)
    (out / "quality" / "plots").mkdir(parents=True, exist_ok=True)

    source_hash_before = sha256(source)
    provenance = {
        "source_video": str(source), "source_sha256_before": source_hash_before,
        "analyzer_version": 2, "analyzer_file_sha256": sha256(Path(__file__)),
        "git_commit": git_commit(),
        "interpreter": sys.executable, "python_version": platform.python_version(),
        "opencv_version": cv2.__version__, "numpy_version": np.__version__,
        "working_directory": str(Path.cwd()),
        "analysis_command": shlex.join([sys.executable, "-m", "rm27.perception.vision.localization.analyze_video",
                                       str(source), "--out", str(out), "--sample-step", str(args.sample_step),
                                       "--large-gap-factor", str(args.large_gap_factor)]),
        "parameters": {"sample_step": args.sample_step, "large_gap_factor": args.large_gap_factor,
                       "jpeg_quality": 95, "orb_nfeatures": 500, "feature_grid": [4, 6],
                       "near_duplicate_absdiff_threshold": 1.0, "luma_thresholds": [5, 250],
                       "flow_max_corners": 200, "flow_quality_level": .01, "flow_min_distance": 8,
                       "flow_min_points": 6, "diagnostic_percentiles": [10, 90]},
    }
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + "\n")
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot decode {source}")
    try:
        backend = capture.getBackendName()
    except (cv2.error, AttributeError):
        backend = "UNKNOWN"
    write_failures = []
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    declared_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    fourcc_value = int(capture.get(cv2.CAP_PROP_FOURCC) or 0)
    fourcc = "".join(chr((fourcc_value >> (8 * i)) & 0xFF) for i in range(4))
    nominal_dt = 1.0 / fps if fps > 0 else None
    timestamps, rows, intervals = [], [], []
    quality_rows = []
    exact_duplicates = near_duplicates = frozen_runs = decode_failures = 0
    current_freeze = 0
    previous_hash = previous_gray = None
    orb = cv2.ORB_create(nfeatures=500)
    grid_rows, grid_cols = 4, 6
    all_sampled = []
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            if frame_index < declared_count:
                decode_failures += 1
            break
        pts = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
        timestamps.append(pts)
        if len(timestamps) > 1:
            intervals.append(pts - timestamps[-2])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        digest = hashlib.sha1(frame.tobytes()).hexdigest()
        exact = previous_hash == digest if previous_hash is not None else False
        if exact:
            exact_duplicates += 1
            current_freeze += 1
        else:
            if current_freeze >= 2:
                frozen_runs += 1
            current_freeze = 0
        previous_hash = digest
        sampled = frame_index % args.sample_step == 0
        if sampled:
            blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0)
            gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1)
            gradient = float(np.mean(np.sqrt(gx * gx + gy * gy)))
            p01, p50, p99 = [float(x) for x in np.percentile(gray, [1, 50, 99])]
            under = float(np.mean(gray <= 5))
            over = float(np.mean(gray >= 250))
            keypoints = orb.detect(gray, None)
            counts = [0] * (grid_rows * grid_cols)
            for point in keypoints:
                col = min(grid_cols - 1, int(point.pt[0] / width * grid_cols))
                row = min(grid_rows - 1, int(point.pt[1] / height * grid_rows))
                counts[row * grid_cols + col] += 1
            motion_px = None
            rotation_deg = None
            if previous_gray is not None:
                near_delta = float(np.mean(cv2.absdiff(gray, previous_gray)))
                near = near_delta < 1.0
                if near:
                    near_duplicates += 1
                points = cv2.goodFeaturesToTrack(previous_gray, maxCorners=200,
                                                 qualityLevel=0.01, minDistance=8)
                if points is not None and len(points) >= 6:
                    tracked, status, _ = cv2.calcOpticalFlowPyrLK(
                        previous_gray, gray, points, None)
                    good = status.ravel().astype(bool) if status is not None else np.zeros(len(points), dtype=bool)
                    if int(good.sum()) >= 6:
                        old = points.reshape(-1, 2)[good]
                        new = tracked.reshape(-1, 2)[good]
                        motion_px = float(np.median(np.linalg.norm(new - old, axis=1)))
                        transform, _ = cv2.estimateAffinePartial2D(old, new, method=cv2.RANSAC)
                        if transform is not None:
                            rotation_deg = float(np.degrees(np.arctan2(transform[1, 0], transform[0, 0])))
            else:
                near_delta, near = None, False
            previous_gray = gray
            image_name = f"frame_{frame_index:06d}.jpg"
            try:
                written = cv2.imwrite(str(out / "frames" / image_name), frame,
                                      [cv2.IMWRITE_JPEG_QUALITY, 95])
            except cv2.error:
                written = False
            if not written:
                write_failures.append(f"frames/{image_name}")
            quality_rows.append({
                "frame_index": frame_index, "pts_time_s": pts,
                "laplacian_variance": blur, "gradient_mean": gradient,
                "luma_p01": p01, "luma_p50": p50, "luma_p99": p99,
                "underexposed_fraction": under, "overexposed_fraction": over,
                "feature_count": len(keypoints), "empty_feature_cells": sum(x == 0 for x in counts),
                "feature_cells": json.dumps(counts, separators=(",", ":")),
                "mean_absdiff_sampled": near_delta, "near_duplicate": near,
                "median_flow_px": motion_px, "rotation_proxy_deg": rotation_deg,
                "image_file": f"frames/{image_name}",
            })
            all_sampled.append((frame_index, pts, blur, gradient, len(keypoints), p50, under, over, motion_px, rotation_deg))
        rows.append({"frame_index": frame_index, "pts_time_s": pts, "exact_duplicate": exact,
                     "decode_ok": True})
        frame_index += 1
    capture.release()
    if current_freeze >= 2:
        frozen_runs += 1
    dt_stats = stats(intervals)
    threshold = nominal_dt * args.large_gap_factor if nominal_dt else None
    large_gaps = sum(x > threshold for x in intervals) if threshold is not None else 0
    zero_or_negative = sum(x <= 0 for x in intervals)
    first_pts = timestamps[0] if timestamps else None
    last_pts = timestamps[-1] if timestamps else None
    duration = last_pts - first_pts if timestamps else 0.0
    source_hash_after = sha256(source)
    status = acceptance_status(declared_count, frame_index, timestamps, large_gaps,
                               write_failures, source_hash_before == source_hash_after, nominal_dt is not None)
    file_size = source.stat().st_size
    bitrate = (file_size * 8 / duration / 1000.0) if duration > 0 else None
    metadata = {
        "source": {"path": str(source), "sha256": source_hash_before, "size_bytes": file_size,
                    "mtime_utc": datetime.fromtimestamp(source.stat().st_mtime, timezone.utc).isoformat(),
                    "git_commit": git_commit(),
                    "git_status": source_git_status(source)},
        "decoder": {"backend": backend, "opencv_version": cv2.__version__},
        "video_stream": {"codec_fourcc_observed": fourcc, "width": width, "height": height,
                         "pixel_format": "decoded BGR8 (source pixel format unavailable through OpenCV)",
                         "nominal_fps": fps, "declared_frame_count": declared_count,
                         "decoded_frame_count": frame_index, "start_time_s": timestamps[0] if timestamps else None,
                         "first_pts_s": first_pts, "last_pts_s": last_pts, "pts_span_s": duration,
                         "duration_from_pts_s": duration,
                         "container_duration_s": None,
                         "declared_count_over_fps_s": declared_count / fps if fps > 0 else None,
                         "decoded_count_over_fps_s": frame_index / fps if fps > 0 else None,
                         "duration_note": "PTS span excludes final presentation duration; count/fps is an estimate, not container duration",
 "bitrate_kbps_from_file_and_pts": bitrate,
                         "stream_count": "UNKNOWN", "time_base": "UNKNOWN", "dts": "UNAVAILABLE"},
        "timing": {"field": "ENCODED_STREAM_PTS", "unit": "seconds as reported by OpenCV",
                   "clock_domain": "UNKNOWN", "semantic": "encoded presentation timestamp",
                   "exposure_timestamp": "UNKNOWN", "sensor_capture_drop_rate": "UNKNOWN",
                   "interval_threshold_factor": args.large_gap_factor, "intervals": dt_stats,
                   "large_interval_count": large_gaps, "zero_or_negative_count": zero_or_negative},
        "frame_integrity": {"exact_duplicate_count": exact_duplicates,
                             "near_duplicate_count_sampled": near_duplicates,
                             "frozen_run_count": frozen_runs, "decode_failure_count": decode_failures},
        "sampling": {"sample_step": args.sample_step, "sampled_frame_count": len(quality_rows),
                     "derived_images": "decoded BGR frames re-encoded as JPEG quality 95"},
    }
    (out / "video_stream.json").write_text(json.dumps(metadata, indent=2) + "\n")
    with (out / "timing.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["frame_index", "pts_time_s", "dt_s", "exact_duplicate", "decode_ok"])
        writer.writeheader()
        for i, row in enumerate(rows):
            row = dict(row); row["dt_s"] = intervals[i - 1] if i else None; writer.writerow(row)
    with (out / "quality" / "frame_quality.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(quality_rows[0]) if quality_rows else ["frame_index"])
        writer.writeheader(); writer.writerows(quality_rows)
    # Machine-readable annotations: these are observations, not ground truth.
    def sampled_runs(predicate):
        runs, current = [], []
        for item in all_sampled:
            if predicate(item):
                current.append(item)
            elif current:
                runs.append(current); current = []
        if current:
            runs.append(current)
        return runs
    intervals_out = []
    for label, pred, reason in [
        ("severe_blur", lambda x: x[2] < percentile([y[2] for y in all_sampled], 10), "sampled Laplacian variance in lowest 10 percent"),
        ("high_motion_proxy", lambda x: x[8] is not None and x[8] > percentile([y[8] for y in all_sampled if y[8] is not None], 90), "sampled median optical-flow displacement in highest 10 percent; diagnostic only"),
        ("low_texture", lambda x: x[4] < percentile([y[4] for y in all_sampled], 10), "sampled ORB diagnostic count in lowest 10 percent"),
        ("exposure_transition_candidate", lambda x: x[5] < 20 or x[5] > 235, "sampled median luminance near an extreme"),
    ]:
        for run in sampled_runs(pred):
            if len(run) < 2:
                continue
            intervals_out.append({"start_time_s": run[0][1], "end_time_s": run[-1][1], "label": label,
                                  "reason": reason, "evidence": "offline decoded-frame diagnostic"})
    write_simple_yaml(out / "intervals.yaml", intervals_out)
    (out / "intervals.json").write_text(json.dumps(intervals_out, indent=2) + "\n")
    provenance.update({"source_sha256_after": source_hash_after, "backend": backend,
                       "status": status, "representative_examples": "NOT_GENERATED; use frame_quality.csv image_file"})
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + "\n")
    write_simple_yaml(out / "provenance.yaml", provenance)
    try:
        import matplotlib
        # These diagnostics are only ever written as files, never shown; select a
        # non-interactive backend so a headless host needs no display or Tk install.
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
        if quality_rows:
            t = [r["pts_time_s"] for r in quality_rows]
            for key, name in (("laplacian_variance", "sharpness"), ("luma_p50", "median_luminance"), ("feature_count", "feature_count")):
                plt.figure(figsize=(12, 3)); plt.plot(t, [r[key] for r in quality_rows]); plt.xlabel("ENCODED_STREAM_PTS (s)"); plt.ylabel(key); plt.tight_layout(); plt.savefig(out / "quality" / "plots" / f"{name}.png", dpi=120); plt.close()
    except ImportError:
        metadata["plots"] = "UNAVAILABLE: matplotlib not installed"
    (out / "analysis_complete.json").write_text(json.dumps({**status, "metadata": metadata}, indent=2) + "\n")
    print(json.dumps({"output": str(out), "decoded_frames": frame_index, "duration_s": duration,
                      "sha256": metadata["source"]["sha256"], **status}, indent=2))
    return 0 if status["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
