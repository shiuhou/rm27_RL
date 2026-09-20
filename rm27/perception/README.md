# Perception layer

`perception` is the complete sensing layer of the RM27 system. `vision` is its
camera-based specialization, not a separate research product. The vision work
therefore belongs under this namespace and includes:

- M3C/OS04A10 monocular localization and SLAM;
- camera and lens calibration, timing and frame-convention checks;
- green guidance-light, red/blue armor and target recognition;
- visual tracking, pose/target estimation and the handoff to ExternalNav or
  flight guidance.

The subdirectories separate responsibilities inside that one sensing chain:

| Directory | Responsibility |
|---|---|
| `vision/` | Visual algorithms, SLAM experiments and device-side MaixCAM2/OS04A10 code |
| `camera/` | The camera producer contract and calibration/transport boundary |
| `guidance/` | Converts a versioned visual target estimate into flight guidance commands |
| `sitl/` and `gazebo/` | Synthetic or truth-backed producers used to test the same perception contract |

The root runtime currently consumes the versioned `TargetEstimate` interface
for target-relative guidance.

Self-localization is a separate perception product. SLAM/VIO experiments must
publish through a dedicated versioned localization contract
(`PoseEstimate` / `LocalizationEstimate`) before any adapter is allowed to feed
ArduPilot ExternalNav.

A detector, tracker, SLAM run or replay is evidence for the perception stage
only; it does not become a flight-control input until timing, calibration,
frame semantics, estimator reset behavior and failure handling are verified.
