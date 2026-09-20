# VSL-1 gate review — `0921.mp4`

## Decision

| Track | Classification | Evidence |
|---|---|---|
| VSL-1A | **PASS FOR INITIAL OFFLINE LOCALIZATION** | 13,766/13,766 frames decode; 1344×760; regular 180 Hz encoded PTS; zero encoded gaps, exact duplicates, frozen runs, and decode failures in the generated analysis |
| VSL-1B | **NOT STARTED** | No direct M3C recording was executed; the existing VIN/NV21 procedure is documented only |

This is a pass for encoded-video evidence only. It does not pass camera
hardware qualification. The source SHA-256, all counters, timing statistics,
quality metrics, and derived-frame provenance are independently present in
`artifacts/vsl-1a-0921-20260921b/video_stream.json`, `timing.csv`,
`quality/frame_quality.csv`, `intervals.json`, and `analysis_complete.json`.

## Gate outcome

The image is usable for the justified parts of VSL-2: freeze a reproducible
encoded-frame dataset, preserve encoded PTS, define evidence-based diagnostic
subsets, and prepare a real calibration capture protocol. The lower image rows
are often weakly textured and several low-sharpness/high-flow intervals are
annotated. Those limitations are retained as subsets; they do not justify
discarding the entire recording.

VSL-2 is **`BLOCKED_ON_CALIBRATION_CAPTURE`** for a calibration-backed dataset.
No intrinsics, distortion model, crop transform, lens ID, or focus provenance
was found for the actual M3C + OS04A10 + current lens + 1344×760 recording.
`configs/real_hardware.json` has `camera_intrinsics: null`. The numeric values
in `green_detector_full180.conf` are explicitly provisional pixel-normalization
values and must not be promoted to calibration.

The final state is therefore:

```text
VSL-1A = PASS FOR INITIAL OFFLINE LOCALIZATION
VSL-1B = NOT STARTED
VSL-2  = BLOCKED_ON_CALIBRATION_CAPTURE (dataset preparation partial)
VSL-3  = NOT READY
```

## Evidence boundaries

The MP4 establishes only decoded encoded-stream behavior and image diagnostics.
Its PTS is `ENCODED_STREAM_PTS`; exposure, VIN PTS, host receive time, sensor
drops, IMU timing, and camera latency remain unknown. The complete VSL-1 report
is [VSL-1A_0921.md](VSL-1A_0921.md).
