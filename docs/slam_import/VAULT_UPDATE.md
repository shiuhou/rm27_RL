# Vault update proposal — not ingested

## Local SLAM/VIO integration proposal — 2026-10-11

The published research repository `shiuhou/rm27_slam` was reviewed at exact
commit `3ea1e16086965176db68b88e1c42990f92c5913e`, extracted from
`96b72c28036aa9ea395f1b45cb7a58e450995abf`. A local review branch based on
`da396bef63ce51476a71af4b9b70abdda03eed0b` carries the reviewed localization,
OpenVINS/VIO experiment code, host tools, focused tests, templates and reports.
Flight/control/simulation code, device submodules, raw data, credentials,
virtual environments and generated artifacts were excluded. This proposal is
not a Vault write and does not promote any source PASS/PARTIAL status; M3C
calibration, live VIO, PX4 fusion and flight gates remain unresolved.

## Roadmap continuity proposal — 2026-10-11

`rm27/perception/vision/docs/PLAN_HISTORY.md` links the preserved 2026-09-20 SLAM
roadmap and 2026-10-07 VINS/VIO plan, explains candidate roles and planned vs
observed gates. Original plans/criteria unchanged; no new validation/deployment.
This repository note is not Vault ingestion.

## SSH workspace inventory supplement — 2026-10-11

`rm27/perception/vision/docs/WORKSPACE_LOCATIONS.md` consolidates saved Linux
research/worktree/evidence locations, M3C deployments and Windows editing mirrors.
It distinguishes historical path evidence from UNKNOWN current remote state and
GitHub's curated publication from full disk backup or deployment. No SSH, hardware
or Vault access in this documentation supplement; no credentials are published.

## Publication proposal — 2026-10-11 (not ingested)

The current portable summary is `rm27/perception/vision/docs/PX4_M3C_CONNECTION.md`
and `CODEX_RESUME.md`. Preserve UART2↔UART3 crossed wiring, verified115200/MAVLink2,
exact d6f12ad1 firmware, separate USB management, Stage A/B scope and later VIO gates.
User reports no spare UART; prior spare-pad next actions are superseded. Stock
single-UART MAVLink ULog remains a candidate with an offline reassembler only.
No new hardware observation or Vault access. Root handoff and selected research
are being published at the user's explicit request. Windows proposal history is
`.maixpy/VAULT_UPDATE_WINDOWS_20261011.md`; older Linux proposal below is retained.

## Latest authorized offline experiment, 2026-10-08

One props-off/user-confirmed, disarmed PX4 BMI270 FIFO ULog capture completed;
instance1/device3604506 verified in raw data.18.122s file,17.9s IMU span; two
independent downloads SHA256-identical. Raw export/parse proven, completeness
FAILED:391 logger dropouts and111.409ms max sample gap. Sensor local cadence
~1580Hz; recorded gyro/accel effective706.681/666.823Hz, not lossless capture.
See `PX4_IMU_OFFLINE_VALIDATION.md` for immutable-file hash and bounded conclusions.
Original logger config restored and verified; no flight/control/firmware or
camera/M3C change. VIO-S0 and VIO-P remain PARTIAL. Proposed durable finding:
raw IMU source exists, but this logger/SD configuration does not yet provide a
complete offline VIO input. This repository proposal is NOT a Vault update.

## Latest export/timing decision, 2026-10-08

Read-only logger query: idle, SDLOG_PROFILE1/SDLOG_MODE0/SDLOG_BOOT_BAT0;
microSD readable. Source at reported4817c061 shows default sensor topic interval
1000ms and raw FIFO profile default instance0, not selected BMI270 instance1.
Propose explicit instance/device-bound ULog on SD then host export as first
raw-data qualification, not an already-working synchronized dataset path.
MAVLink ULog streaming/USB is a later candidate; performance and M3C host mode
unverified. HIGHRES/SCALED_IMU remain processed interval-average paths.
FC sample timestamps are driver-related boot HRT, not proven ADC latch; camera
exposure timing and cross-device mapping remain open. No raw capture, firmware,
parameters, stream settings, schema, algorithm or wiring changed. VIO-P/S0 PARTIAL.
User says hardware is separate on the desk: prior photograph request superseded,
no physical action now; rigid mounting is needed later for synchronized capture.
This is a repository proposal only. The Vault was not read or modified.

## Latest live FC evidence, 2026-10-08 (supersedes unavailability below)

COM19 Windows read-only queries succeeded. MICOAIR_H743_V2; micoair-v1.15.2;
reported hash4817c0618a1286846116e90c6eb8919efaa013cf. Official manual confirms
the AIO uses micoair_h743-v2, BMI088 SPI2 and BMI270 SPI3; both drivers active.
Selected BMI270 device3604506. Internal one-second topic rates documented in
audit, not transport rate/ODR qualification. FC and M3C not connected; DDS stopped;
raw export/clock mapping/exposure timing unresolved. No captures or settings
changes. VIO-P and S0 stay PARTIAL. Next physical verification is a photograph
of current camera/FC mounting relationship, not wiring or calibration.
No Vault write; proposed durable facts must retain source/runtime distinction.

## CURRENT correction: PX4 real flight controller (2026-10-08)

User explicitly states PX4, superseding prior ArduPilot assumptions for VIO.
Prioritize PX4 sensor_gyro/accel (or qualified FIFO) + OS04A10; do not assume local
ICM hardware. Local clean PX4 dd0ad74 is source evidence only, not deployed
version. HIGHRES_IMU here is sample-timestamped but interval-derived and possibly
estimator-bias corrected; previous ArduPilot send-time interpretation must not be
reused. No actual FC USB/bridge identity or measured rates yet. VIO-S0 and VIO-P
stay PARTIAL. See current audit/contract/handoff; older recommendations below
are historical. No Vault write or broader hardware-ledger edit was performed.

## 2026-10-08 real sensor-chain audit proposal

VIO-P remains PARTIAL after completed repeatability experiment. S0 also PARTIAL:
current M3C SSH unavailable. Source snapshot c7d47b2 contains a local ICM-42688-P
SPI/FIFO collector with gyro+accel, nominal 1/4 kHz, filtered sensor output and
separate sensor/batch timestamps. This supports prioritizing OS04A10 + local ICM
for physical verification, not asserting installed hardware or synchronized data.
FC BMI088/BMI270 are firmware-supported alternatives; HIGHRES_IMU uses filtered
values and send-time timestamp. See updated audit/contract for hashes, gaps and
the single next physical action. No data collection, calibration, integration,
hardware changes, commit, push or Vault ingestion occurred.

## 2026-10-08 native repeatability evidence — proposal only

Ten frozen native runs: ATE 5.432--7.653 cm (mean 6.608, sample SD 0.689 cm),
15/45 pair differences >1 cm, 2800 poses each. Three separate OpenCV-thread=1 /
publisher-thread-off controls also vary (5.551--6.604 cm); fixed RNG seed already
exists. All process exits zero and frozen manifest verification passed.
See `rm27/perception/vision/docs/NATIVE_REPEATABILITY.md` for artifacts and
limits. Native variation is confirmed independently of adapter conversion;
root cause is not proved. Current 1 cm gate is empirically unreliable as an
adapter discriminator, remains unchanged, and VIO-P stays PARTIAL. Any future
Vault synthesis should retain these distinctions, not import transient logs.
No Vault was read or modified by this task.

## Latest runtime adapter result — PARTIAL

Runtime integration and exact same-run pose/velocity/time conversion are
implemented, not merely a saved-message importer. All 2912 images/29120 IMU
samples reached the observer unchanged. 2800 visual and 27980 propagated outputs
match independent raw MCAP; all processes exit 0. Tests 226 pass/1 skip plus
7/7 ROS2. Original binary/config/input hashes remain intact.

Do not record overall PASS: adapter03 ATE 5.7385 cm differs by 1.2729 cm from
native02 (7.0114 cm), exceeding unchanged 1 cm run-consistency tolerance. Same-run
adapter/native metrics are exact, and upstream states themselves diverge before
conversion. Specific execution-variation cause is unconfirmed. See
`OPENVINS_ADAPTER.md`; no scale correction or falsified complete v2 state.
Native-only control03 independently gives ATE6.2403 cm, confirming some native
run variability without proving its full cause or waiving the historical gate.
This is a proposal only. Vault and hardware were not touched.

## Latest proposal — native SE3 evaluation

The native clean run now has independently cross-checked SE3-only metrics:
2780 matched poses; ATE RMSE 7.0114 cm; 1s RPE 4.5238 cm / 0.47799 degrees.
Applied scale is 1. Path ratio 0.980586 is reported separately, not corrected.
Original orientation-truth limitation and explicit quaternion normalization
are documented in `NATIVE_SE3_EVALUATION.md`. Adapter output projection has
started, but runtime dispatch/new replay parity remain incomplete. Broader
VIO-P is PARTIAL. Tests: 208 passed/1 skip, separate ROS2 7/7. These supersede
the older pending-SE3 statements below. No hardware or Vault modification.

## Current addendum — 2026-10-07

ROS2 is the selected host baseline path; ROS1/Noetic below is historical.
The official-source-attributed ASL archive is now available and validated.
All 2,912 cam0 images and 29,120 IMU samples passed complete ASL-to-ROS2
read-back comparison; 18 calibration comparisons match pinned upstream.
A pinned ROS2 container dependency image now builds without host changes.
The earlier four-include-only build failed at shutdown. Gdb now proves the
root cause: global visualizer publishers were destroyed after the Fast DDS
static factory. The explicit ROS2 lifecycle patch moves only entry-point
ownership into main; algorithm-library hashes are unchanged. Ten no-data exits
and a full sequence with the same 2,800 online poses now return 0.
Designation: pinned OpenVINS + explicit ROS2 lifecycle compatibility patch.
See `rm27/perception/vision/docs/ROS2_LIFECYCLE_FIX.md` for evidence and patch.
VIO-P PASS is scoped to this latest native/lifecycle task; original broader
adapter/parity/SE3 work remains incomplete and was not started. Do not infer
accuracy, flight readiness or M3C performance from this result.
Regression tests: 190 passed, 1 ROS2-specific skip, with all 7 transport tests
passing separately under Jazzy. This is not metric accuracy qualification.
The prior M3C connectivity failure was superseded by a read-only connection,
but no actual raw IMU stream has been verified. VIO-S remains PARTIAL.

The Vault was not modified. This is a reviewable proposal for a separately
authorized future project-bridge task, not a dump of chat, logs or artifacts.

## Durable decisions

- Qualify native OpenVINS mono+IMU on public data before an RM27 adapter or M3C
  port. Selected Jazzy container keeps host dependencies unchanged; ROS1 was
  historical, not an RM27 architecture decision.
- ROS resource owners must be destroyed before their process-static middleware
  factories; main-scoped ownership fixed the measured OpenVINS shutdown defect.
- Raw IMU acquisition time, telemetry send/receive time and camera timing are
  different evidence. HIGHRES_IMU naming does not prove acquisition semantics.
- Keep `T_imu_camera` as camera-to-IMU and reuse LocalizationEstimate v2.

## Verified work worth retaining

Dedicated research branch merged Linux/Windows baselines at `13c7a77`; scoped
working-tree IMU sequence/static-analysis preparation has 184 passing tests.
Tests are synthetic and do not qualify the sensor or VIO algorithm. See root
HANDOFF and three VIO reports for exact source pins and evidence references.

## Open questions

The original broader plan and VIO-S remain PARTIAL. Adapter/parity/SE3 and real
IMU evidence remain absent. The native lifecycle failure is resolved. No claim of
M3C 15–30 Hz VIO or flight readiness. No new physical calibration was performed.

Do not ingest raw build/download logs, datasets, credentials, temporary files or
unverified hardware guesses. Import only reviewed synthesis with provenance.
