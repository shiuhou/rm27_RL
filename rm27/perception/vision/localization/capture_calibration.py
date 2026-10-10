#!/usr/bin/env python3
"""Select measured-target observations from M3C recordings; never fit intrinsics."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import shlex
import subprocess
import sys

import cv2
import numpy as np

from .analyze_video import sha256 as digest, git_commit


def positive_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a measured positive number in metres")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")


def validate_target(target):
    kind = target.get("type")
    if kind == "chessboard":
        counts, lengths = ("corners_x", "corners_y"), ("square_size_m",)
    elif kind == "circles_grid":
        counts, lengths = ("pattern_cols", "pattern_rows"), ("center_distance_m",)
    elif kind == "charuco":
        counts, lengths = ("squares_x", "squares_y"), ("square_length_m", "marker_length_m")
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
            raise ValueError("this OpenCV build has no usable ChArUco detector")
        name = target.get("dictionary", "")
        if not isinstance(name, str) or not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
            raise ValueError("unsupported ChArUco dictionary")
    else:
        raise ValueError("target type must be chessboard or charuco; AprilGrid is unsupported")
    for key in counts:
        if type(target.get(key)) is not int or target[key] < 3:
            raise ValueError(f"{key} must be an integer >= 3")
    for key in lengths:
        positive_number(target.get(key), key)
    if kind == "charuco" and target["marker_length_m"] >= target["square_length_m"]:
        raise ValueError("marker_length_m must be smaller than square_length_m")
    identity = target.get("physical_target_id")
    if not isinstance(identity, str) or not identity or identity in ("UNKNOWN", "REQUIRED_OPERATOR_VALUE"):
        raise ValueError("physical_target_id must identify the measured printed board")
    return target


def make_detector(target):
    if target["type"] in ("chessboard", "circles_grid"):
        return None
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, target["dictionary"]))
    board = cv2.aruco.CharucoBoard((target["squares_x"], target["squares_y"]),
                                 target["square_length_m"], target["marker_length_m"], dictionary)
    return board, cv2.aruco.CharucoDetector(board)


def detect(gray, target, charuco):
    if target["type"] == "chessboard":
        found, corners = cv2.findChessboardCornersSB(
            gray, (target["corners_x"], target["corners_y"]), flags=cv2.CALIB_CB_EXHAUSTIVE)
        return (corners, np.arange(len(corners))) if found else (None, None)
    if target["type"] == "circles_grid":
        pattern = (target["pattern_cols"], target["pattern_rows"])
        found, centers = cv2.findCirclesGrid(gray, pattern, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        return (centers, np.arange(len(centers))) if found else (None, None)
    corners, ids, _, _ = charuco[1].detectBoard(gray)
    return corners, ids


def observation_geometry(corners, ids, target, width, height):
    """Image-space proxies for centroid, scale, tilt/perspective, roll and edges.

    Homography describes the printed plane only; no camera calibration or metric
    camera pose is estimated. Require >=70% ChArUco corners and board extent.
    """
    nx = target.get("corners_x", target.get("pattern_cols", target.get("squares_x", 0) - 1))
    ny = target.get("corners_y", target.get("pattern_rows", target.get("squares_y", 0) - 1))
    ids = np.asarray(ids).reshape(-1)
    points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    if len(points) < max(6, math.ceil(nx * ny * .7)) or len(set(ids.tolist())) != len(ids):
        raise ValueError("insufficient_corner_coverage")
    if np.any(ids < 0) or np.any(ids >= nx * ny) or not np.isfinite(points).all():
        raise ValueError("invalid_corners")
    board = np.array([(i % nx, i // nx) for i in ids], dtype=np.float64)
    if np.ptp(board[:, 0]) < .7 * (nx - 1) or np.ptp(board[:, 1]) < .7 * (ny - 1):
        raise ValueError("insufficient_board_extent")
    transform, _ = cv2.findHomography(board, points, method=0)
    if transform is None:
        raise ValueError("degenerate_board")
    quad = cv2.perspectiveTransform(np.array([[[0., 0.], [nx-1., 0.],
                                             [nx-1., ny-1.], [0., ny-1.]]]), transform)[0]
    if not np.isfinite(quad).all() or np.any(quad < 0) or np.any(quad >= [width, height]):
        raise ValueError("board_extent_clipped")
    top, right, bottom, left = [np.linalg.norm(quad[(i+1) % 4] - quad[i]) for i in range(4)]
    area = abs(cv2.contourArea(quad.astype(np.float32))) / (width * height)
    if min(top, right, bottom, left) < 5 or area < .005:
        raise ValueError("board_too_small")
    center = quad.mean(axis=0) / [width, height]
    roll = math.atan2(*(quad[1] - quad[0])[::-1])
    aspect_proxy = math.log(((top+bottom)/(left+right)) / ((nx-1)/(ny-1)))
    features = [*center, math.log(area), aspect_proxy, math.log(top/bottom), math.log(left/right),
                math.cos(2*roll), math.sin(2*roll),
                quad[:, 0].min()/width, 1-quad[:, 0].max()/width,
                quad[:, 1].min()/height, 1-quad[:, 1].max()/height]
    return {"features": features, "centroid_normalized": center.tolist(), "area_fraction": area,
            "roll_rad_mod_pi": roll % math.pi, "board_quad_px": quad.tolist(),
            "tilt_note": "projective shape proxy, not a calibrated tilt angle"}


FEATURE_STEPS = np.array([.12, .12, .35, .25, .18, .18, .35, .35, .12, .12, .12, .12])


class DiversitySelector:
    def __init__(self, cap=50):
        if type(cap) is not int or cap < 30:
            raise ValueError("cap must be an integer >= 30")
        self.cap, self.features = cap, []

    def consider(self, features):
        features = np.asarray(features, dtype=float)
        if features.shape != FEATURE_STEPS.shape or not np.isfinite(features).all():
            raise ValueError("invalid diversity features")
        if len(self.features) >= self.cap:
            return False, "accepted_cap_reached"
        if any(np.max(np.abs(features-old)/FEATURE_STEPS) < 1 for old in self.features):
            return False, "similar_geometry"
        self.features.append(features)
        return True, "accepted"


def assign_holdout(records):
    """Deterministic farthest-point 20% holdout, ties broken by capture order.

    Samples geometry across the session; not an independent validation session.
    No image residual or fitted model is consulted.
    """
    accepted = [r for r in records if r["accepted"]]
    for record in records:
        record["split"] = "fit" if record["accepted"] else None
    if not accepted:
        return
    features = np.asarray([r["geometry"]["features"] for r in accepted]) / FEATURE_STEPS
    chosen = [0]
    while len(chosen) < math.ceil(len(accepted) * .2):
        distances = np.min(np.linalg.norm(features[:, None, :] - features[chosen][None, :, :], axis=2), axis=1)
        distances[chosen] = -1
        chosen.append(int(np.argmax(distances)))
    for index in chosen:
        accepted[index]["split"] = "validation"


def validate_session(session):
    required = ("platform", "sensor", "lens_id", "focus_note", "camera_mode_id", "crop_resize",
                "device_software_commit", "timestamp_evidence", "physical_measurements")
    if any(key not in session for key in required):
        raise ValueError("session missing provenance fields: " + ", ".join(k for k in required if k not in session))
    for key in required[:7]:
        if (not isinstance(session[key], str) or not session[key]
                or session[key].startswith(("REPLACE_", "REQUIRED_"))):
            raise ValueError(f"session {key} must be a non-empty string (UNKNOWN if unidentifiable)")
    if (session["platform"], session["sensor"], session["camera_mode_id"]) != ("M3C", "OS04A10", "full180"):
        raise ValueError("this protocol requires M3C OS04A10 full180")
    if type(session.get("width")) is not int or type(session.get("height")) is not int or (session["width"], session["height"]) != (1344, 760):
        raise ValueError("session geometry must be 1344 x 760")
    if not isinstance(session["timestamp_evidence"], dict) or not isinstance(session["physical_measurements"], dict):
        raise ValueError("timestamp_evidence and physical_measurements must be objects")
    for key in ("unit", "clock_domain", "semantic", "evidence_status"):
        if not isinstance(session["timestamp_evidence"].get(key), str) or not session["timestamp_evidence"][key]:
            raise ValueError("timestamp_evidence requires unit/domain/semantic/status")
    measures = session["physical_measurements"]
    for axis in ("horizontal", "vertical"):
        positive_number(measures.get(f"{axis}_span_m"), f"{axis}_span_m")
        count = measures.get(f"{axis}_square_count")
        if type(count) is not int or count <= 0:
            raise ValueError("measured span requires a positive integer square count")
    for key in ("exposure_us", "gain"):
        if session.get(key) is not None:
            positive_number(session[key], key)
    json.dumps(session, allow_nan=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, action="append", help="repeat for several raw recorded takes")
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--step", type=int, default=30)
    parser.add_argument("--max-views", type=int, default=50)
    parser.add_argument("--min-sharpness", type=float, default=50.0)
    args = parser.parse_args(argv)
    target = validate_target(json.loads(args.target.read_text()))
    session = json.loads(args.session.read_text())
    validate_session(session)
    measures = session["physical_measurements"]
    square = target.get("square_size_m", target.get("square_length_m", target.get("center_distance_m")))
    for axis in ("horizontal", "vertical"):
        measured_square = measures[f"{axis}_span_m"] / measures[f"{axis}_square_count"]
        if abs(measured_square / square - 1) > .01:
            raise ValueError("printed axis measurement differs from target square length by >1%; check scaling/units")
    positive_number(args.min_sharpness, "min_sharpness")
    if args.step <= 0:
        parser.error("step must be positive")
    selector = DiversitySelector(args.max_views)
    sources = [p.resolve() for p in args.source]
    if any(not p.is_file() for p in sources):
        parser.error("all sources must be existing recorded files")
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        parser.error("refusing to overwrite non-empty output")
    (out / "images").mkdir()
    source_records = [{"path": str(p), "sha256_before": digest(p)} for p in sources]
    provenance = {"git_commit": git_commit(),
                  "script_sha256": digest(Path(__file__)), "interpreter": sys.executable,
                  "python_version": platform.python_version(), "opencv_version": cv2.__version__,
                  "command": shlex.join([sys.executable, "-m", "rm27.perception.vision.localization.capture_calibration",
                                         *(sys.argv[1:] if argv is None else argv)]),
                  "working_directory": str(Path.cwd()), "step": args.step, "max_views": args.max_views,
                  "min_sharpness": args.min_sharpness, "feature_steps": FEATURE_STEPS.tolist()}
    (out / "capture_input.json").write_text(json.dumps({"sources": source_records, "session": session,
        "target": target, "software": provenance}, indent=2, allow_nan=False) + "\n")
    charuco = make_detector(target)
    records, failures = [], []
    for source_index, source in enumerate(sources):
        capture = cv2.VideoCapture(str(source))
        if not capture.isOpened():
            raise RuntimeError(f"cannot decode {source}")
        declared = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        width, height = [int(capture.get(prop)) for prop in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT)]
        if (width, height) != (1344, 760):
            capture.release()
            raise ValueError("recorded geometry must be 1344 x 760; do not resize")
        index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            current = index
            index += 1
            if frame.shape[:2] != (760, 1344):
                failures.append("decoded_geometry_changed")
                break
            if current % args.step:
                continue
            pts = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            corners, ids = detect(gray, target, charuco)
            record = {"source_index": source_index, "frame_index": current,
                      "encoded_pts_s": pts if math.isfinite(pts) else None,
                      "accepted": False, "reason": "target_not_detected", "image": None,
                      "sharpness": sharpness, "corner_ids": [], "corners_px": []}
            if corners is not None and ids is not None:
                record.update(corner_ids=np.asarray(ids).reshape(-1).tolist(),
                              corners_px=corners.reshape(-1, 2).tolist())
                try:
                    geometry = observation_geometry(corners, ids, target, width, height)
                    record["geometry"] = geometry
                    if sharpness < args.min_sharpness:
                        record["reason"] = "sharpness_below_threshold"
                    else:
                        record["accepted"], record["reason"] = selector.consider(geometry["features"])
                except ValueError as exc:
                    record["reason"] = str(exc)
            if record["accepted"]:
                relative = f"images/image_{len(selector.features)-1:04d}_s{source_index}_f{current:08d}.png"
                try:
                    written = cv2.imwrite(str(out / relative), frame)
                except cv2.error:
                    written = False
                if not written:
                    selector.features.pop()
                    record.update(accepted=False, reason="image_write_failed")
                    failures.append(relative)
                else:
                    record.update(image=relative, image_sha256=digest(out / relative))
            records.append(record)
            print(json.dumps({k: record[k] for k in ("source_index", "frame_index", "accepted", "reason", "image")}), flush=True)
        capture.release()
        source_records[source_index].update(declared_frame_count=declared, decoded_frame_count=index,
                                             sha256_after=digest(source))
        source_records[source_index]["decode_completeness"] = (
            "DECLARED_COUNT_MATCH" if declared > 0 and declared == index else
            "PREMATURE_OR_COUNT_MISMATCH" if declared > 0 else "UNKNOWN_NO_DECLARED_COUNT")
        if index == 0 or (declared > 0 and declared != index):
            failures.append(f"source_{source_index}_decode_failed_or_count_mismatch")
        if source_records[source_index]["sha256_before"] != source_records[source_index]["sha256_after"]:
            failures.append(f"source_{source_index}_changed")
    assign_holdout(records)
    accepted = sum(r["accepted"] for r in records)
    validation = sum(r["split"] == "validation" for r in records)
    summary = {"execution_completed": True, "accepted": accepted, "validation": validation,
               "rejected": len(records)-accepted, "failures": failures,
               "selection_checks_passed": accepted >= 30 and validation >= math.ceil(accepted*.2) and not failures,
               "calibration_exists": False, "vsl_2": "BLOCKED_ON_CALIBRATION_CAPTURE",
               "review_required": "inspect fit and validation geometry, flatness, corner detections and provenance before fitting"}
    manifest = {"schema_version": 2, "session_id": out.name, "created_utc": datetime.now(timezone.utc).isoformat(),
                "session": session, "sources": source_records, "target": target, "software": provenance,
                "timestamp_semantic": "ENCODED_STREAM_PTS (not exposure)", "records": records, "summary": summary,
                "limitations": ["image-space diversity is a heuristic, not calibration qualification",
                                "holdout is from the same capture session, not independent capture",
                                "raw device PTS interpretation remains UNKNOWN unless independently verified"]}
    (out / "capture_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    (out / "target_definition.json").write_text(json.dumps(target, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summary))
    return 0 if summary["selection_checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
