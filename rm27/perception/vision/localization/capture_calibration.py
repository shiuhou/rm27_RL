#!/usr/bin/env python3
"""Extract deterministic calibration observations from a recorded camera stream.

The device-side recording remains the existing M3C/OS04A10 producer. This host
tool only decodes that recording, detects a declared target, and writes PNG
observations plus provenance; it never invents target dimensions.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def validate_target(target):
    kind = target.get("type")
    if kind == "chessboard":
        required = ("corners_x", "corners_y", "square_size_m")
    elif kind == "charuco":
        required = ("squares_x", "squares_y", "square_length_m", "marker_length_m", "dictionary")
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
            raise ValueError("this OpenCV build has no usable ChArUco detector")
    else:
        raise ValueError("target type must be chessboard or charuco")
    for key in required:
        if key not in target or target[key] in (None, "UNKNOWN", "REQUIRED_MEASURED_VALUE"):
            raise ValueError(f"target definition requires measured {key}")


def make_detector(target):
    if target["type"] == "chessboard":
        return None
    dictionary_id = getattr(cv2.aruco, target["dictionary"])
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (int(target["squares_x"]), int(target["squares_y"])),
        float(target["square_length_m"]), float(target["marker_length_m"]), dictionary)
    return board, cv2.aruco.CharucoDetector(board)


def detect(gray, target, charuco):
    if target["type"] == "chessboard":
        pattern = (int(target["corners_x"]), int(target["corners_y"]))
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            return 0, None
        return len(corners), corners
    _, detector = charuco
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or corners is None:
        return 0, None
    return len(ids), corners


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True, help="recorded M3C video/H264-compatible source")
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--step", type=int, default=30, help="inspect every Nth decoded frame")
    parser.add_argument("--min-sharpness", type=float, default=50.0)
    args = parser.parse_args()
    source, out = args.source.resolve(), args.out.resolve()
    target = json.loads(args.target.read_text())
    validate_target(target)
    if not source.is_file():
        parser.error(f"source does not exist: {source}")
    if args.step <= 0:
        parser.error("--step must be positive")
    if out.exists() and any(out.iterdir()):
        parser.error(f"refusing to overwrite non-empty output: {out}")
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "rejected").mkdir(parents=True, exist_ok=True)
    charuco = make_detector(target)
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"cannot decode {source}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    records, covered_cells = [], set()
    frame_index = 0
    accepted_count = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % args.step:
            frame_index += 1
            continue
        pts = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        corner_count, corners = detect(gray, target, charuco)
        accepted = False
        reason = "target_not_detected"
        if corners is not None:
            points = corners.reshape(-1, 2)
            center = points.mean(axis=0)
            cell = (min(3, int(center[1] / max(1, height) * 4)), min(5, int(center[0] / max(1, width) * 6)))
            if sharpness < args.min_sharpness:
                reason = "sharpness_below_threshold"
            elif cell in covered_cells:
                reason = "duplicate_coverage_cell"
            else:
                accepted = True
                covered_cells.add(cell)
                accepted_count += 1
                reason = "accepted"
        if accepted:
            name = f"image_{accepted_count - 1:04d}_f{frame_index:08d}.png"
            relative = f"images/{name}"
        else:
            name = f"rejected_f{frame_index:08d}.png"
            relative = f"rejected/{name}"
        cv2.imwrite(str(out / relative), frame)
        records.append({"frame_index": frame_index, "encoded_pts_s": pts, "image": relative,
                        "accepted": accepted, "reason": reason, "corner_count": corner_count,
                        "sharpness": sharpness,
                        "split": "validation" if accepted and accepted_count % 5 == 0 else "fit"})
        frame_index += 1
    capture.release()
    manifest = {
        "schema_version": 1, "session_id": out.name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source": {"path": str(source), "sha256": digest(source)},
        "platform": "M3C", "sensor": "OS04A10", "camera_mode_id": "UNKNOWN",
        "width": width, "height": height, "fps_observed_by_decoder": fps,
        "lens_id": "UNKNOWN", "focus": "UNKNOWN", "crop_resize": "UNKNOWN",
        "target": target, "capture_software": "capture_calibration.py",
        "timestamp_semantic": "ENCODED_STREAM_PTS", "records": records,
        "limitations": ["encoded source has no device exposure/VIN timing",
                        "split is a deterministic holdout, not independent capture"],
    }
    (out / "capture_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out / "target_definition.json").write_text(json.dumps(target, indent=2) + "\n")
    print(json.dumps({"session": str(out), "accepted": accepted_count,
                      "rejected": len(records) - accepted_count, "resolution": [width, height]}))


if __name__ == "__main__":
    main()
