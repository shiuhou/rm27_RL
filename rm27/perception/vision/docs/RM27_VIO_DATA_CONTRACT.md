# RM27 VIO data contract — preparation, 2026-10-07

Status updated 2026-10-08: raw-IMU validation/static analysis and OpenVINS runtime
adapter are implemented. Same-run native/adapter conversion and SE3 metrics agree;
native repeatability remains unresolved, so VIO-P stays PARTIAL. PX4 identity
and running IMUs are verified via Windows COM19; synchronized recording and
PX4-to-M3C raw transport remain unqualified (VIO-S0 PARTIAL).
See `OPENVINS_ADAPTER.md`, `NATIVE_REPEATABILITY.md` and `IMU_SOURCE_AUDIT.md`.

## Real sensor-chain evidence requirements (2026-10-08 clarification)

Latest physical offline evidence (2026-10-08): BMI270 instance1/device3604506
FIFO records now captured/exported; see `PX4_IMU_OFFLINE_VALIDATION.md`.
This supersedes no-capture claims below, not the synchronization prerequisites.
Every retained FIFO message had samples1; sample-time cadence633us differs from
nominal dt625us. Preserve timestamp_sample exactly; do not synthesize1600Hz times.
All timestamps ordered but391 ULog dropouts and unequal gyro/accel timestamp
sets make the current record INCOMPLETE, not a qualified VIO input. No gap filling
or silent row-wise pairing is allowed. Retain ULog/dropout/independent-transfer
hash evidence and distinguish capture completeness from file parse integrity.
Multi-sample FIFO timing remains untested. No schema or validator change.

Export clarification (2026-10-08; no schema/validator changes): first qualify
identified PX4 raw gyro/accel FIFO records in SD ULog, then host export. Current
default logging does not provide this: logger is idle, SDLOG_PROFILE=1, raw
sensor topics default to1000ms intervals. Raw FIFO profile bits select instance0,
whereas current BMI270 is instance1/device3604506. Future custom logger entries
must bind both instance and device identity, preserve necessary health topics,
and be explicitly approved; a custom file replaces profile selection. Interval0
means request all publications, not guaranteed complete capture.

Retain original FIFO arrays, samples, scale, dt, device_id, publication and sample
timestamps. SI values are counts*scale; driver rotation has already occurred.
Source supports a last-sample anchor and candidate expansion
timestamp_sample-(N-1-i)*dt; dynamic continuity, batch boundary and pairing
validation is still required. Preserve driver-estimated timing uncertainty;
do not label reconstructed times as hardware ADC timestamps. Raw here means
pre-estimator measurements, not absence of sensor filtering. Full-rate SI
sensor_* records may be FIFO averages and must preserve that distinction.

ULog download is an offline export, not a live or synchronized VIO path.
MAVLink ULog streaming over a later verified USB-host link is a source-supported
candidate, not a tested1600Hz delivery solution. Retain ULog dropout/sequence
evidence, queue/gap checks and measured payload/latency before accepting it.
SCALED_IMU is not a workaround: inspected implementation uses interval-derived
values and millisecond publication time. HIGHRES_IMU limitations below remain.
User says hardware is separate on the desk: no mounting transform or synchronized
capture is available; no assembly/wiring is requested during this audit.

No schema or validator is changed here. The user's latest correction selects
**PX4 flight-controller IMU as the priority source for OS04A10 VIO**. Live COM19
readback identifies MICOAIR_H743_V2, micoair-v1.15.2, reported source hash
4817c0618a1286846116e90c6eb8919efaa013cf. BMI088 is instance0 (accel6946834,
gyro6684690), BMI270 instance1 (both3604506, currently selected). The user confirms
MicoAir743v2-AIO-35A and that M3C and FC are NOT connected. Direct raw transport
and cross-device synchronization remain unverified; COM19 is Windows-only.
Do not assume M3C contains ICM-42688-P. Local ICM collector notes below describe
historical candidate formats only, not the selected physical data path.

### PX4 source semantics and provenance

Current source-semantic reference is upstream PX4 commit
`4817c0618a1286846116e90c6eb8919efaa013cf`, matching the runtime-reported hash.
This does not prove binary reproducibility or absence of vendor changes.
Earlier local dd0ad74 source is historical. Official manual target is
micoair_h743-v2 (also checked against upstream v1.16.0), not h743-aio.
Preferred source is sensor_gyro (rad/s) and sensor_accel (m/s^2), retaining
device_id, samples, clipping/error counters, timestamp_sample and timestamp.
These are FRD board-frame measurements; driver rotation and board-to-body
calibration must not be silently duplicated. FIFO publishers may average multiple
samples into sensor_* messages. Preserve that fact and on-chip filter settings.

Keep independent gyro/accel timestamps and instances before normalization.
Do not pair by host arrival order or silently assume identical acquisition time.
Any resampling/interpolation must retain source associations, timing assumptions
and uncertainty. Existing single-device RAW_IMU validation does not itself
perform or authorize this pairing. A future normalized device_id must bind the
actual accel+gyro pair, board and epoch, including split-chip implementations.

timestamp_sample is driver sample-related FC boot time in microseconds, not
automatically an ADC-latched timestamp. Keep publish timestamp separately;
validate IRQ/polling/FIFO anchor semantics for the actual driver. FIFO dt, scale,
sample count and ordering are required before expanding per-sample timestamps.
Do not invent nanosecond accuracy by multiplying a microsecond timestamp.

vehicle_imu delta_angle/delta_velocity are integrated, calibrated FRD body-frame
quantities with separate integration intervals. sensor_combined contains selected
interval-average values with a relative accel timestamp and invalid sentinel.
Keep these as processed/integrated records unless an explicit derived-data path
is separately reviewed; never silently tag delta/dt as raw instantaneous IMU.
In the inspected PX4 revision HIGHRES_IMU derives delta/dt, may subtract estimator
bias and uses timestamp_sample. This differs from historical ArduPilot send-time
behavior. Neither qualifies as a raw stream merely by its message name.

PX4 boot time, M3C monotonic time and VIN PTS are separate/unmapped domains.
Preserve all originals and boot/reset epochs; record any TIMESYNC mapping,
direction, units, drift, RTT and uncertainty. Middleware may already convert
timestamps; verify deployed uXRCE serialization before applying offsets again.
TIMESYNC cannot establish exposure midpoint or rolling-shutter row timing.
Measured ODR, uORB rate and transport rate must be reported separately. One live
1-second uORB status window measured gyro/accel instance0 at 666/802 Hz and
instance1 at 1580/1580 Hz; vehicle_imu at 199/196 Hz. These are internal topic
rates, not an IMU capture dataset, hardware ODR certificate, sustained throughput
or M3C delivery performance. FIFO batch count/rate must not be confused with
individual sample rate. Snapshot data-gap counters are nonzero; no loss-free
claim. IMU_INTEG_RATE=200 and IMU_GYRO_RATEMAX=800 are configuration, not measured
end-to-end output. Observed cutoff params (gyro80/accel30/dgyro30 Hz) describe
downstream processing, not guaranteed raw-topic bandwidth.

Windows COM19 at57600 is the current USB CDC inspection link, not a confirmed
57600-baud physical UART constraint. FC reports USB nominal2Mbps and tx budget
100000B/s; neither is a throughput test. Current TELEM ports are not connected
to M3C. DDS client is stopped and default source exports no raw gyro/accel.
Keep future raw export and clock mapping unqualified; do not repurpose stock
HIGHRES_IMU's bias-corrected interval averages as raw sensor observations.

- Preserve camera `pts_raw` as opaque integer with UNKNOWN unit/domain/event
  until verified. The current camera collector's `monotonic_us` is software
  receipt after VIN returns, NOT exposure time. Encoded frame timing and nominal
  180 fps do not establish exposure midpoint or rolling-shutter timing.
- Preserve ICM FIFO bytes, packet order, 16-bit timestamp and original register
  configuration. Its timestamp needs verified tick/event, wrap/reset handling
  and clock mapping before qualifying as acquisition evidence. Do not invent
  precise timestamps by distributing a batch over its nominal ODR.
- ICM batch `host_monotonic_ns` is taken after SPI read and raw file write;
  retain it as software batch receipt evidence, not sensor sample time. Sensor
  PLL and camera PTS are not proven shared clocks even on one M3C. Record boot
  epochs and each clock's origin, unit and mapping uncertainty separately.
- Counts-to-SI conversion requires verified range: inspected ICM code uses
  accel/16384 g and gyro/131 deg/s; convert using 9.80665 and pi/180 respectively
  only with matching configuration. Preserve originals. Record on-chip filtering
  separately from later software filtering/bias correction; RAW_IMU means
  measured angular rate plus specific force, not necessarily filter-free silicon.
- Capture-counter increments are not hardware loss evidence. Keep FIFO overflow,
  reset, lost-count and unknown saturation semantics explicit. Decompose sensor
  filter, FIFO residence, bus transfer, file-write and transport delays; unknown
  latency remains null, not zero. No physical sample stream is qualified yet.
- Historical ArduPilot HIGHRES_IMU findings do not describe the user's actual
  PX4 firmware. PX4-specific semantics above supersede them for this VIO line.
  EKF attitude remains ineligible as raw gyro+accel. Separate FC/M3C clocks need
  a measured mapping including drift; no such mapping currently exists.

Later calibration must keep rigid `T_imu_camera`, time offset, clock drift,
filter delay, exposure and row timing as separate evidence. Reuse existing
camera intrinsics subject to unchanged mode/lens/focus provenance. Do not start
new noise capture or calibration just to fill unknown metadata.

## Existing interfaces preserved

`LocalizationEstimate` schema v2 remains the sole pose contract. `T_parent_child`
maps child coordinates into parent coordinates; quaternion order is Hamilton
`wxyz`. Existing visual-only datasets do not acquire a mandatory IMU field.
The existing calibration and flight eligibility gates are unchanged.

Keep `T_imu_camera`: **camera coordinates into IMU coordinates**. An inverse
must be derived from a checked rigid transform, not relabeled. CAD mounting
is approximate evidence, not a calibrated transform. Spatial calibration,
temporal calibration, camera geometric calibration and noise characterization
are separate products.

## Raw IMU JSONL

Each line follows `experiments.vio.validate_imu_sample`: schema version 1,
nonnegative integer `sequence`, `measurement_kind=RAW_IMU`, non-UNKNOWN
`device_id`, finite `gyro_xyz` in `rad/s`, finite `accel_xyz` in `m/s^2`, and
`timestamp` using `TimestampEvidence` (integer raw value, unit, clock domain,
semantic and evidence status). `saturation` is boolean or absent/null.
Attitude/EKF quaternion telemetry and integrated deltas cannot be relabeled as
raw angular rate and acceleration. Preserve source units and conversion evidence.

The sequence validator requires one device, one sequence-counter space, one
timestamp unit/domain/semantic/evidence status and strictly increasing sequence
numbers and timestamps. A reset starts a separate recording. It reports gaps,
cadence and known/unknown saturation. A missing sequence value is NOT proof of a
physical sensor drop; `missing_sample_count` remains null without independent
counter semantics. Integer timestamps are subtracted before conversion to seconds.

`imu_capture.template.json` is an unfilled metadata template, not sensor evidence.
Record source identity, firmware, driver, transport, axes, handedness, ranges,
filters, rate, sequence semantics, temperature/warmup and rigid mounting.
Do not promote UNKNOWN metadata to VERIFIED merely to pass a validator.

## Offline audit and static analysis

Run from the research worktree with its Python environment:

```bash
python -m rm27.perception.vision.localization.experiments.vio \
  --samples /path/to/raw_imu.jsonl --output /new/path/imu_audit.json
```

Add `--stationary-confirmed` only for an operator-confirmed stationary capture.
This requests mean, sample standard deviation and overlapping Allan deviation;
it does not estimate production noise densities/random walks or detect stationarity.
It requires >=32 samples, VERIFIED acquisition timestamps with semantic
`ACQUISITION`, known clock domain, no sequence gaps, no observed saturation and
maximum interval deviation <=1% of median. These are analysis prerequisites,
not proof that a short recording is long enough for noise calibration. Unknown
saturation stays explicit. Acceleration mean includes gravity, not just bias.
The input SHA256 is recorded and existing output files are never overwritten.

For a future noise experiment, retain the entire warmup/static record and
temperature/vibration context. Select duration and Allan fitting intervals from
the noise timescales that must be identified; do not fit production parameters
from the short synthetic test fixtures or borrow another sensor's specification.

## Historical source findings and subsequent runtime qualification

The following initial ROS1 source review is historical. The current public
baseline uses ROS2 with explicit lifecycle compatibility patch; verified runtime
conventions are documented in `OPENVINS_ADAPTER.md`. It does not migrate RM27
to ROS1 or establish real-device timing.

Inspected official revision `69488123ed9362dd44b6f28e7f4680abbff1442b`:

- `ov_msckf/src/ros1_serial_msckf.cpp` selects IMU and configured camera topics.
  `max_cameras=1`, `use_stereo=false` selects cam0+imu0. If `path_gt` is supplied,
  lines 252–255 can initialize with truth: **do not set this node parameter**.
- `ROS1Visualizer.cpp:595–605` documents state time as camera time and publishes
  pose time as state time plus calibrated camera-to-IMU offset. Image time and
  published state time are therefore not automatically equal.
- `ov_core/src/utils/quat_ops.h:142–157` implements JPL rotation with the negative
  skew term. ROS message field names alone are insufficient to establish the
  Hamilton pose transformation. Verify world-to-IMU direction, translation and
  velocity frames together. Subsequent conversion tests and same-run parity are
  recorded in `OPENVINS_ADAPTER.md`; ROS1 findings alone are not their evidence.
- `ROS1Visualizer.cpp` has distinct visual-state publication and propagated
  odometry paths. Their rates must be measured separately.

Future observations must preserve original image timestamp, latest supplied IMU
timestamp (data horizon, not automatically integration endpoint), actual state
time, software completion and publication time. Keep raw values and clock
evidence. Never subtract dataset time from host steady time without a verified
mapping; unavailable latency stays null. Offset, drift, receipt delay, exposure
timing and rolling-shutter row timing must not be collapsed into one value.

Keep raw outputs, online projections and any final trajectory separate. A saved
online trace is not an optimized final trajectory. Unknown tracking/reset/map
state remains unknown. An adapter-owned run epoch is not a native reset counter.
Only materialize the full v2 estimate when all required fields are evidenced.

## Evaluation and remaining gates

Native EuRoC execution and adapter parity evaluation have completed. Continue to freeze
dataset/calibration/config/source hashes, use identical mono+IMU input windows,
and use SE3 without rescaling as the primary metric evaluation. Report scale
error separately. OpenVINS documents inaccurate original V1_01_easy orientation
truth (`docs/gs-datasets.dox:25–28`); record which truth is used and do not silently
substitute corrected truth. No ATE/RPE, coverage, initialization or CPU/RAM result
was available at the initial preparation checkpoint; current public-run metrics
are in `VIO_BASELINE_REPORT.md` and `NATIVE_REPEATABILITY.md`. Host performance
is not M3C performance.

The prior OS04A10 calibration result remains `GEOMETRIC_CHECKS_PASS` with its
original provenance gaps; do not upgrade it to a fully qualified synchronized
VIO calibration. Camera–IMU extrinsics, timing and IMU noise remain unknown.
