# VSL-0A Repository / Platform Audit

Status: implemented (repository audit); no camera or SLAM execution.

## Architecture and paths

The normal MaixCDK path is `maix::camera::Camera` → RGB888 →
`maix::time::ticks_us()` after `camera.read()` → `VisualMotionEstimator` and
`GreenLightDetector` → versioned `TargetEstimate` JSON. The timestamp is an
application receive timestamp; exposure and sensor SOF semantics are not
established.

The high-FPS path is VIN → borrowed NV21 Y/VU planes → `Nv21View` → bounded ROI
RGB conversion → target detector. `FrameMetadata` preserves `sequence`, raw
`pts_raw`, and host `received_us`. The source explicitly leaves PTS units and
event semantics unverified.

`VinNv21Frame` retains the borrowed VIN lease through mapping, cache
invalidation, unmapping, and release. A consumer must retain the shared frame
lease for the entire read; `LatestFrameSlot` may destroy replaced pending
frames outside its mutex. The slot is appropriate for low-latency target work,
but is not yet qualified for visual odometry because it silently skips frames.

`visual_motion` selects gradient features, tracks patches with bounded search
and Lucas–Kanade refinement, estimates a 2-D similarity transform with robust
pair hypotheses, and emits image-space translation/rotation/scale and a host
timestamp. It is classified as a frontend/diagnostic primitive, not metric
visual odometry or SLAM.

## Evidence table

| Status | Evidence |
|---|---|
| CODE-OBSERVED | RGB888 and VIN/NV21 paths, `FrameMetadata`, leases, `LatestFrameSlot`, continuity counters, visual motion, TargetEstimate serializers, replay/recording tools, starter manifest and collector |
| TESTED | VSL-0B contract and manifest tests are run by the repository test command; no hardware test is implied |
| USER-STATED | M3C + OS04A10 target and separation of target perception, localization, guidance, and flight control |
| UNVERIFIED | M3C identity, camera module/lens, calibration, thermal limits, real camera rates, PTS interpretation, localization queue requirements |
| UNKNOWN | Exposure timestamp, PTS unit/clock/event, camera–IMU extrinsics and offset, canonical localization frame registry, metric scale and estimator covariance |

The starter package is an experiment protocol and metadata template. It is not
evidence that a camera, calibration, estimator, or board deployment passed.
