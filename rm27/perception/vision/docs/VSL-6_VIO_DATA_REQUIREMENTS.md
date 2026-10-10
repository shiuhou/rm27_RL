# Future VSL-6 VIO evidence contract

Implementation: **IMPLEMENTED** raw-IMU sample validation and explicit unknown
extrinsic/time-reference template. Validation: synthetic software checks only.
**VSL-6 remains NOT_STARTED.** OpenVINS, IMU acquisition, synchronization,
extrinsic calibration and fusion are not integrated or executed.
Physical blockers: actual raw IMU samples, sensor identity/configuration,
verified timing and camera–IMU calibration. Next gate: collect and inspect a
matched raw camera/IMU evidence dataset under a separately authorized protocol.
These fields are optional for current monocular datasets.

## Raw sample contract

`experiments.vio.validate_imu_sample` validates one schema-1 record:

- `measurement_kind: RAW_IMU`; device identity in `device_id` and nonnegative
  integer `sequence`. A reset/wrap starts an explicitly separate sequence space.
- `timestamp`: existing TimestampEvidence raw integer, unit, clock domain,
  semantic, evidence status. Preserve UNKNOWN interpretations. Clock alignment
  is not established by type validation or similar-looking numbers.
- `gyro_xyz`: three finite numbers in **rad/s**; `gyro_unit: rad/s`.
- `accel_xyz`: three finite numbers in **m/s^2**; `accel_unit: m/s^2`.
- Optional `saturation`: observed boolean; absent/null when unavailable.
  Preserve per-axis clipping flags in original device evidence if available.
- Dataset-level sensor variant, firmware, output rate/range/filter settings,
  axis directions/handedness, sample semantics and calibration references.

Booleans/strings are not numeric samples. Flight-controller attitude, quaternion,
filtered orientation or integrated deltas must not be relabeled raw gyro/accel.
Retain originals, hashes and exact unit conversion history. The sample validator
checks structure, not IMU noise, bias, synchronization, rate or observability.

## Spatial and temporal calibration

`localization/experiments/vio_calibration.template.json` contains only
**UNKNOWN / NOT_CALIBRATED**, null references and no numerical guesses for:

- `T_body_camera`: camera coordinates into body coordinates.
- `T_imu_camera`: camera coordinates into IMU coordinates.
- `temporal_offset`: UNKNOWN value, unit, source/destination domains and sign
  convention. Establish a documented mapping from measured evidence before use.

A future calibrated artifact must specify physical sensor identities, rigid
mounting, frame axes, transform direction, unit quaternion/translation units,
method and uncertainty evidence, dataset hashes and held-out validation.
A temporal mapping may require offset **and drift**, not an assumed scalar
constant; neither is chosen here. Raw device PTS, software receipt clocks and
flight-controller clocks are not interchangeable. Covariance/noise densities
must be measured or sourced from qualified evidence, never invented.

## Future ExternalNav gate, documentation only

Any later control integration must separately establish metric scale, reviewed
parent/child/world/body frames, clock-qualified freshness, estimator health,
producer-owned reset/epoch/session semantics, local continuity through map and
relocalization events, sustained update rate and bounded queueing, explicit
failure/stale handling and appropriate bench/SITL/hardware qualification.
Tracking is not flight safety. Final loop-closure-corrected trajectories do not
represent poses available online. No wire adapter or MAVLink localization
message is implemented or transmitted by this preparation work.

The immediate physical action is still **VSL-2 camera geometric-calibration
capture**, not IMU collection or flight. Preserve the first measured camera set
and stop for review before fitting or starting a later stage.
