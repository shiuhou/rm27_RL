# Windows bench/offline handoff archive — through 2026-10-11

Archived intact before fast-forwarding main to the existing remote OpenVINS
checkpoint. Dated entries below describe their own working-tree states, not the
current Git or hardware state. Read root HANDOFF.md first for current priorities.

## Latest: offline ULog/MAVLink reassembly — 2026-10-11

- Repository state: main b8e7299cf090f5150f9f262ebd3b232a824cc526; same preexisting
  dirty work. New ignored research under `.maixpy/imu-design-20261010/single-uart/reassembly`.
- Objective/actual changes: finite reassembler, tests, saved-log replay, PLAN,
  VALIDATION and proposed trial. Reuses old CRC decoder, pyulog and semantic audit;
  no production dependency/framework or hardware access.
- Verification: existing pyulog venv `-m unittest discover` yields28 new,9 diagnostic
  and10 decoder PASS (47 total); replay_saved.py exit0. Exact commands/results in
  VALIDATION.md and tests-final/regression/replay logs. Stage A/B SHA unchanged.
- Verified facts: 6896444/5597928 saved bytes reconstruct exactly with original SHA;
  391/306 logger events and FIFO counts retained. Synthetic fault injection rejected.
  ACK retries deduplicated, sequence wrap handled, malformed/missing records rejected.
- Failed attempts: RED24 missing implementation; first GREEN offset test received
  earlier startup rejection, then corrected injection location; RED summary absent,
  added explicit UNKNOWN. All intermediate outputs retained, no relaxed gate.
- Assumptions/open questions/risks: framing is synthetic, not a live MAVLink log.
  Valid tail boundary cannot prove entire source/tail delivery; uORB overwrite and
  logger loss can precede packets. No selected BMI088, latency or clock qualification.
  Integrated means remain OpenVINS-inadmissible. ACK intents are values, not sends.
- Next actions: implement/review the single-owner live controller with offline
  timeout/fresh-heartbeat/start-stop/restore tests; then the separately scoped
  921600/datarate46080/MAVLink-only logger candidate in PROPOSED_TRIAL.md. Never stop-all.
  No communication or logger authorization is inferred from offline test completion.
- Rollback: only new research directory and additive latest notes; frozen evidence
  unchanged. No parameter/firmware/wiring/baud changes, push or Vault access.

## Latest: single-UART source review — 2026-10-10

- Objective/constraint: user reports no spare UART; retire the physical spare-pad
  verification action. Keep existing MAVLink link, no firmware/baud/settings changes.
- Actual changes: research at `.maixpy/imu-design-20261010/single-uart/REVIEW.md`,
  generated wire-check.txt, additive latest pointers only. No production edits.
- Verified: inspected exact d6f12ad1 sources and local pymavlink2.4.49; inline
  `py -3 -` synthetic checks in wire-check.txt show six groups PASS (schema sizes,
  zero trimming, ULog record size, LOGGING_DATA roundtrip, signing length, budget).
  Not a live ULog reassembly or OpenVINS test. Stage A/B VALIDATION SHA256 remains
  d9c79727ad749e8ec6ba970a2e41c0ef4aa5ccc3ac6e6e7bc53c0f1bbb7ffd0f.
- Decision: evaluate stock MAVLink ULog as complete-record bench prototype before
  custom firmware. Existing HIGHRES/SCALED telemetry cannot preserve all IMU metadata.
  IMU-only200Hz ULog wire model13.94kB/s excludes metadata/startup;115200 insufficient.
- Risks/open questions: logger/uORB/ULog queue loss, reliable-header blocking,
  batching latency, unqualified high baud, integrated-vs-point observation model.
  Complete records are not automatically OpenVINS-compatible. Current state not
  reread; EV0/Disarmed remain last observed, not fresh claims.
- Next: offline incremental ULog-over-MAVLink parser fixtures if proceeding with
  this prototype, then explicitly scoped communication/logger trial. No deployment
  or protocol ID chosen. Stage C remains closed; no same-step approval re-request.
- Failed attempts: one exploratory read used a nonexistent VehicleImu snapshot
  path (exit1); corrected to existing imu-input snapshot. No check failures.
- Repo/rollback: main b8e7299, preexisting dirty changes retained; new research
  directory plus additive pointers are the scoped changes. Prior frozen reports
  remain unchanged. No commit/push/Vault access; proposal in VAULT_UPDATE.md only.

## Latest: offline post-inventory VIO-S0 review — 2026-10-10

See `.maixpy/imu-design-20261010/followup/REPORT.md` and HANDOFF.md, also LATEST.md.
92 related offline tests pass this turn. New pinned-source finding: synchronous
DDS time-sync wait can stall the uORB copy loop; failed attempts may retry without
another1s delay. This is a queue-loss risk, not measured DDS loss.200Hz IMU plus
metadata minimum16.78kB/s;400Hz conservative data bound64.74kB/s. Conditional
dedicated-UART/DDS prototype retained; USB fallback now explicitly host-driver/
management-blocked. Integral means remain inadmissible point input. Prior33+48+13
manifest entries and Stage A/B report unchanged. No device access or changes.
Next approval: one power-off physical carrier-pin verification, not another live
software inventory. VIO-S0 PARTIAL; no Stage C, calibration, fusion or push.

## Latest: approved read-only interface inventory — 2026-10-10

See `.maixpy/interface-inventory-20261010/VALIDATION.md` and HANDOFF.md.
Fresh d6f12ad1,Disarmed/EV0; existing MAVLink2/optical-flow ports preserved.
M3C USB is the active SSH network gadget and kernel CONFIG_USB_ACM is not set:
USB-host/CDC fallback is not plug-and-play. ttyS4 is launcher-owned; ttyS5 pinctrl
assigned. ttyS1 only a spare candidate, requiring carrier pad confirmation.
No data UART open, capture, configuration/firmware/wiring change; sessions closed.
Prior33+48 manifest entries unchanged. Next: designer-confirmed spare UART pads,
not automatic rewiring/DDS launch. VIO-S0 PARTIAL; Stage C not started.

## Latest: offline high-rate IMU architecture — 2026-10-10

See `.maixpy/imu-design-20261010/VIO_S0_INPUT_DESIGN.md` and HANDOFF.md.
Pinned DDS/XRCE and BMI088 source audit, diagnostic integral validator and wire
budget completed offline:100 related tests pass,2 Linux PTY tests skipped.
vehicle_imu200 Hz requires at least15.8 kB/s serial/XRCE, beyond115200; integrated
means remain rejected as OpenVINS point samples. Conditional preferred prototype:
separate460800+ UART/DDS vehicle_imu; fallback full-native-rate SI/DDS over verified
USB-host/CDC with explicit processing. Neither deployed or qualified. Prior48-entry
transport manifest unchanged. Stage A/B retained; Stage C not started. No hardware
access/settings changes/Vault/push. Next approval: one read-only interface inventory.

## Latest: IMU rate-control / vehicle_imu audit — 2026-10-10

See `.maixpy/imu-transport-20261010/VALIDATION.md` and HANDOFF.md. At unchanged
115200/5760B/s, approved reduced-telemetry control yields50.001Hz;100Hz request
yields62.112Hz (multiplier0.621). Original10.11Hz is explained by stream-budget
scaling0.202. All target configured rates and temporary M3C termios restored;
Disarmed,EV_CTRL0. Startup invalid bytes retained; NOT loss-free/VIO qualification.
vehicle_imu198/194Hz snapshots preserve integrals/dt/IDs but require estimator-model
validation. Stock DDS sensor_combined defaults10ms; vehicle_imu not exported.
Faster dedicated DDS/full-rate export is a proposed approval-gated direction, not
implemented. Stage A/B unchanged; Stage C NOT STARTED. No Vault access or push.

## Latest: PX4 -> M3C IMU input qualification — 2026-10-10

See `.maixpy/imu-input-20261010/IMU_INPUT_VALIDATION.md` and HANDOFF.md. One
approved30s passive test receives303 HIGHRES_IMU at10.11255Hz. Current exact
v1.17.0 source establishes processed integral/bias-corrected semantics, not raw.
Communication decode VERIFIED; VIO input PARTIAL/NOT QUALIFIED, clock UNKNOWN.
Current115200/5760B/s budget cannot meet200–400+Hz goal. User decision/approval
needed for next transport/input choice. Stage A/B preserved; Stage C NOT STARTED.
Final Disarmed,EKF2_EV_CTRL0,receiver stopped,temporary M3C termios restored.

## Latest: MAVLink2 / synthetic ODOMETRY slice — 2026-10-10

Real M3C -> PX4 heartbeat/MAVLink2/ODOMETRY/vehicle_visual_odometry and
running-stopped-restarted attribution VERIFIED. See
`.maixpy/mavlink2-20261010/VALIDATION.md` and its HANDOFF.md.4 tests pass;
final Disarmed,EKF2_EV_CTRL0,senders stopped,termios restored. No PX4 parameter
writes. Synthetic transport only; no VIO/time-sync/EKF-fusion or loss-free claim.

## Latest retry: SD readable; parameter baseline changed — 2026-10-09

Fresh retry succeeds at SD directory reads, but boot diagnostic reports missing
SD params/import failure. CAL_ACC0_ID/1_ID now0 versus pre-swap6946834/3604506.
See `.maixpy/px4-sd-control-20261009/SD_CONTROL_PREFLIGHT.md` latest section.
No new capture or parameter mutation; recovery decision needed before same-condition
comparison. Earlier physical power-cycle request below is superseded.

## Prior: replacement-card preflight blocked — 2026-10-09

User reported card changed. New read-only evidence in
`.maixpy/px4-sd-control-20261009/SD_CONTROL_PREFLIGHT.md` and `HANDOFF.md` finds
SD stat/readdir `Connection timed out`, despite vfat mount/df output. LoggerPID2131
and commander671 remain from previous final readback; clean new-card initialization
not established. No logger/config/parameter mutation or new capture performed.
961advertised RAM parameter wire records preserved/validated because boot history
loads params from SD. Do not infer an empty card from an unsuccessful empty list.
Next step: fully remove all PX4 power with current card seated, reconnect USB only,
then fresh SD/config audit. M3C not needed. VIO-S0/P remain PARTIAL; media cause
UNKNOWN. Previous buffer matrix below remains unchanged. No Vault access.

## Prior: PX4-only actual-buffer comparison — 2026-10-09

After the completed offline task, the user reconnected PX4 and freshly confirmed
all propellers removed. The prepared64/128/64KiB logging-only experiment is now
executed; start with `.maixpy/px4-buffer-ab-20261009/HANDOFF.md` and
`PX4_BUFFER_COMPARISON.md`. M3C was not accessed. Original offline evidence below
and its hashes are retained unchanged; this supersedes only its next-action state.

Actual65536/131072/65536bytes verified. Full ULog dropout counts336/140/306;
128KiB has a1.546s dropout and lower retained FIFO rate, not an accepted fix.
Both writer loss and logger subscription discontinuities observed. All captures
double-downloaded/parsed;15new+8existing helper tests passed. Initial tool-parser
abort retained separately, never erased. Fresh config fully restored, loggeridle,
Disarmed; no parameter, firmware, IMU selection/rate or existing-log changes.
Only owned temporary topic/emptydirs removed, with local copies retained.

VIO-S0 remains PARTIAL; prior VIO-P unchanged. SD-card-only cause still UNKNOWN.
Next proposed decision: spare-microSD comparison at SAME64KiB, only PX4USB needed,
with PX4 fully unpowered during card substitution. Not authorized/executed here.
No commit/push or Vault access; full new evidence is in ignored `.maixpy` and must
be explicitly preserved when transferring this workspace.

## Prior: entirely offline VIO continuation — 2026-10-09

Both M3C and PX4 are disconnected; no physical access, new capture, dependency
installation, commit, push or Vault change in this task. Historical camera work
below is preserved. Actual VIO source remains in the separate research repository
`/home/shiuhou/Projects/rm27_slam_vio_openvins`, research/vio-openvins-baseline
@4637a5f589801f54ecab146a389db0c81ca1d13f plus scoped working-tree changes.
This Windows repository remains main @b8e7299; `.gitignore`, `new_plan.md` and
`tests/camera_capture.py` were not changed by this task.

Reviewable local evidence/code mirror: `.maixpy/offline-vio-20261009/`.
Start with its `OFFLINE_VALIDATION.md` and `HANDOFF.md`; detailed research reports
are under its `rm27/perception/vision/docs/`. Original saved ULog/calibration/raw
runs and prior failed experiments remain intact. The mirror is ignored by Git;
preserve it explicitly when transferring this Windows workspace.

Verified offline: strict independent BMI270 FIFO export reproduces the old391
dropouts; clock/epoch/bracketing/calibration interfaces tested without inventing
synchronization; four-coefficient camera refit is a diagnostic candidate only.
ASan dynamically reproduced a native callback stack-use-after-return; isolated
value-capture correction and reliable-input controls are documented in the native
report. Do not infer a PX4 loss-free result or real RM27 VIO qualification.

Final offline software validation: Windows282pass/1skip, Linux existing
venv282pass/1skip, ROS2 transport7pass. Reliable-input candidate has FIVE identical
state-file hashes, full adapter runtime/metric PASS against the ORIGINAL reference
and thresholds; not default/live adoption. S0/P rollups remain PARTIAL.
Detailed validation/results and remaining boundaries are in the mirror handoff;
its final review supersedes this pointer, not the camera history.
Prepared future PX4-only buffer comparison is `PX4_NEXT_CAPTURE_PLAN.md` there;
it does not require M3C and has not been physically executed.

Repository baseline: `b8e7299cf090f5150f9f262ebd3b232a824cc526`, main ahead/behind origin by one commit each. Existing `.gitignore` change and untracked `tests/camera_capture.py` preserved. No commit, push or device configuration change in this fitting pass.

## Completed and verified

- Operator confirmed symmetric 7x7 circles target with 180 mm center spans on both axes, six 30 mm intervals. Saved in `artifacts/calibration-board-01/{target,session}.json`.
- Retained source runs: `.maixpy/runs/os04a10-20261007-001551-ed632a` and `os04a10-20261007-003019-5277cc`. Captured/encoded/decoded counts 5405 and 5404, no sequence gaps or MIPI errors. Raw video hashes remained unchanged during extraction.
- Official observation extraction produced 34 full-resolution PNGs, frozen 27 fit / 7 validation split. Manifest is `artifacts/m3c-calibration-20261007/selected-01/capture_manifest.json`.
- Predeclared `fit_protocol.json` before fitting. `py -3 artifacts/m3c-calibration-20261007/fit_calibration.py` completed with `GEOMETRIC_CHECKS_PASS` using OpenCV 4.13.0, five-parameter Brown-Conrady, fit-only intrinsics. Validation estimates board poses only.
- Result: fx=861.977498, fy=861.647099, cx=650.590934, cy=404.650530 pixels, 1344x760. Distortion order k1,k2,p1,p2,k3: [-0.318048343,0.159930139,-0.000436116,0.000145221,-0.049109583].
- Fit RMS 0.156292 px. Validation RMS 0.150498 px, P95 0.265305 px, maximum 0.445882 px. All nine checks passed; positive distortion Jacobian minimum 0.448377. Focal std about 0.58 px; principal-point std about 0.66 px.
- Reviewed `validation_undistortion.jpg` for all seven validation views; corrected board boundaries appear straight, no gross folding or anomalous warp observed.

Result file: `artifacts/m3c-calibration-20261007/calibration_result.json`.
SHA256: `1ef473ffb4dbf024ff682b41907e6f499397199168fa49dc82fd9890365191f8`.

## Limits and next work

This result applies to the operator's current fixed-focus OS04A10 full180 optical setup, not the 512x320 preview mode. Focus was adjusted before capture; no subsequent change reported. Lens identity, board revision, instrument, flatness, sensor-internal crop and exact deployed software commit remain unknown; binary and driver hashes retained instead.

Validation is same-session board data, not independent-session validation or sensor timing certification. No outlier removal or model retuning on validation. Numerical geometric checks are not a blanket SLAM/VIO/controller qualification. Existing VSL-2 gate file and experiment calibration loader were not changed or bypassed; provenance fields need resolution before strict harness admission. No SLAM/backend or ExternalNav was started.

Scripts/results are in ignored artifacts and preview helpers in ignored `.maixpy`; preserve/copy these explicitly for transfer. No Vault modifications performed.

## Environment tracking follow-up

Run `os04a10-20261007-005702-52804b` decoded all 5404 frames; recorder showed
zero sequence gaps, encoder errors and MIPI errors. Full-rate diagnostic used
the fitted calibration, native-resolution remap, bidirectional LK and RANSAC.
Step1 report in `.maixpy/runs/<run>/environment_inspection_step1/` compares
against step6 nominal 30fps sampling. Median track retention improves from
84.41% to 96.07%, median tracks 556 to 636 and median flow 12.77 to 2.15 px.
Full diagnostic on Windows host runs 21.66 processed fps (249.44 s total),
41.93 ms median and 58.93 ms P95 processing. This is pairwise tracking only,
not a board benchmark or SLAM trajectory. No backend integration performed.

Lightweight experiment: `.maixpy/benchmark_frontend.py` processes all source
frames with persistent bidirectional LK, conditional feature replenishment,
half-resolution tracking and point-only undistortion (resize pixel-center
offset included). Retained 372 median active tracks; local retention median
99.72% without per-frame geometric RANSAC. Produced 282 candidate keyframes.
Fresh corrected run: 135.63 fps end-to-end including decode, processing median
4.338 ms/P95 6.600 ms. It does not meet full serial 180fps budget. Report and
scope limits in `.maixpy/runs/<run>/lightweight_frontend/RESULT.md`.

This Windows workspace lacks a WSL Linux distribution, Docker on PATH and
backend runtime/vocabulary in workspace inventory. Historical backend tests
refer to a separate Linux host. No new environment/dependencies installed.

## Live board follow-up

Board root@10.18.198.1 is two Cortex-A53 cores up to 1.2GHz with 446MiB
Linux-visible memory (~334MiB available), no swap. g++ and OpenCV headers/libs
present; CMake/Eigen/g2o/DBoW2 not found in checked standard paths. First 360
source frames benchmarked at 672x380: 11.26fps including decode, median
processing 68.36ms, P95 82.02ms. This is a short offline Python frontend
sample, not live-camera/full SLAM performance or an optimized-C++ limit.
Local evidence: `.maixpy/board-benchmark-20261007/{RESULT.md,report.json}`.
Board helper/calibration staged under `/root/slam-bench-20261007` only;
no dependencies installed and no SLAM backend started.

C++ follow-up built using existing OpenCV 4.11.0 and g++ 11.4.0 -O3. Native
VideoCapture unavailable; tested FFmpeg pipe then predecoded grayscale files.
360-frame raw-file samples: 336x190/180 points/1 thread 29.33fps, median/P95
33.75/35.88ms; 2 threads 27.33fps; 672x380/400 points/2 threads 10.09fps.
Software FFmpeg pipeline only 3.77fps. Includes bidirectional sparse LK and
conditional replenishment, excludes mapping and point undistortion. OpenCV
differs from Python's 4.12.0; no language-only speed inference. Full-board
180fps frontend/backend remains unqualified. Retained evidence under
`.maixpy/board-cpp-benchmark-20261007/RESULT.md`, helper sources under
`.maixpy/`, staged binary/gray files under `/root/slam-bench-20261007`.
