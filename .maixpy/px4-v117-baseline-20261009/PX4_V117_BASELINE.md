# PX4 v1.17.0 read-only baseline — 2026-10-09

## Subsequent source clarification (same date)

See `../px4-v117-source-audit-20261009/PX4_V117_SOURCE_AUDIT.md`. The observed
-b64 command is requested64KiB, not measured allocation: new logger can cap its
first allocation to available contiguous heap. VehicleIMU gap means a
nonconsecutive SI-topic generation read, not direct raw-sample count. Exact-hash
source comparison is complete for the scoped files; runtime trigger remains open.
Original baseline report preserved in the source audit repo-before directory.

## Result and scope

**New firmware identity and current sensor/logger state VERIFIED. Recording
completeness remains UNTESTED on this firmware; VIO-S0 and VIO-P stay PARTIAL.**
User confirmed Windows COM19 connected and QGroundControl closed. Used existing
bounded read-only MAVLink helpers, with DTR/RTS false and serial handles closed
in finally blocks. No logger start/on/off/stop, parameter write, reboot, stream
rate change, calibration, motor/arming command or M3C action was sent. Console
status observations are not a new raw IMU dataset or sustained transport test.

## Verified identity and state

- Board: `MICOAIR_H743_V2` (user's MicoAir743v2-AIO-35A).
- `ver all`: **Release 1.17.0**, full hash
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
- Build May 13 2026 18:38:26, variant default, GCC13.2.1; NuttX11.0.0,
  OS hash `fb2fadf6f599c1406f052db013efd00a2518e72c`.
- MAVLink AUTOPILOT_VERSION flight_sw_version17891583 agrees with the console;
  reported custom-version bytes correspond to the hash prefix. A reported hash
  is not proof of a reproduced build or absence of vendor modifications.
- Heartbeat base_mode29 (armed bit clear); commander Disarmed before/after the
  principal audit. No continuous disarm monitoring/capture qualification claimed.
- Logger before/after: **Not logging**, mode all, 183 subscriptions.
- Actual process: `logger start -b 64 -t -m all`; logger priority230,
  log_writer_file priority60. Do not restore the old process string blindly.
- SDLOG: BACKEND3, BOOT_BAT0, DIRS_MAX0, MISSION0, MODE0, PROFILE1,
  UTC_OFFSET0, UUID1. These are readback values, not an all-parameter backup.
- microSD root accessible; no `etc` entry in its listing. Both listing custom
  logging directory and reading logger_topics.txt returned `Not a directory`.
  No custom topic file was read/installed. Root also contains
  `param_import_fail.txt`; existence alone does not establish when it was made,
  which parameters it concerns or whether this firmware upgrade failed to import
  anything. Content was not inspected and no parameter repair attempted.

## Actual IMU selection and bounded rates

**Selection changed from the old capture:** selected gyro6684690 and
accel6946834, both instance0 (**BMI088**, running on SPI2). BMI270 remains
available on SPI3, instance1, gyro/accel device3604506, but is not selected.
Selection is an observation, not a recommendation to change priorities or EKF.

One `uorb top -1 sensor_gyro sensor_accel vehicle_imu` observation:

| Topic | BMI088 instance0 publications/s | BMI270 instance1 publications/s | Queue length |
|---|---:|---:|---:|
| sensor_gyro | 666 | 1580 | 8 |
| sensor_gyro_fifo | 666 | 1580 | 4 |
| sensor_accel | 802 | 1580 | 8 |
| sensor_accel_fifo | 802 | 1580 | 1 |
| vehicle_imu | 198 | 191 | 1 |
| vehicle_imu_status | 10 | 10 | 1 |

These are **one-second internal publication rates**, NOT physical sensor ODR,
retained SD sample rates or link-delivered IMU rates. FIFO memory size224 bytes
in uORB is not the on-disk ULog record size. No new FIFO sample-count or sample
timestamp series was captured. Do not apply the old samples=1 result to this
firmware without inspection.

For example, selected gyro filter status reports1997.3Hz while the BMI088 gyro
FIFO topic publishes666Hz. This is consistent with possible batching but does
NOT verify a particular batch size; no new batch payload was read here.
VehicleIMU status intervals: BMI088 accel~1247us/gyro~1502us; BMI270 both~633us.
Units/filter/timestamp semantics for the new exact source hash still need source
comparison; previous4817c061 source statements are historical, not newly verified.

## Counter observations: distinct from old SD-write loss

The following driver counts were unchanged at bounded recheck:

- BMI088 accel: bad register1, FIFO reset2, other displayed error counters0.
- BMI088 gyro: FIFO overflow1, FIFO reset2, other displayed error counters0.
- BMI270: FIFO reset1, bad register/transfer/empty/overflow/DRDY missed0.

These are accumulated snapshots, not evidence of when the incidents occurred.
No counter reset or logger restart was sent.

Between first and last `sensors status`:

| VehicleIMU consumer | Accel gap count | Gyro gap count |
|---|---|---|
| instance0 / BMI088 | 1 -> 1 | 2 -> 2 |
| instance1 / BMI270 | **7 -> 8** | **4 -> 5** |

This is a real observed counter increase while the logger was idle at checked
boundaries and no recording start was sent. It cannot be explained solely as
loss in the old SD recording. It does **not** establish hardware-level missed
samples, exact gap duration, VIO-dataset loss rate or the exact triggering
condition in v1.17.0. VehicleIMU subscription/scheduling/timing behavior and the
observer load itself remain candidates pending source tracing. Do not silently
switch to BMI088 and call the completeness issue solved.

## Evidence and verification

Windows evidence root `.maixpy/px4-v117-baseline-20261009/`:

- `identity-readback.txt`: bounded identity request (exit0).
- `console-readback.txt`: ver/commander/logger/ps/sensors/BMI270/SDLOG/custom-path/
  uORB inventory queries (exit0); expected absent-path command errors retained.
- `followup-readback.txt`: BMI088 driver status, bounded uORB rates, SD listing,
  logger and commander status (exit0).
- `counter-recheck.txt`: both BMI088 status queries, BMI270 and sensors status
  again (exit0).

Host mirror: `/home/shiuhou/Projects/rm27-vio-20261007/px4-v117-baseline-20261009`.
Raw console includes device identifiers; keep external/ignored, not in Git.
No production code or helper change. No software tests required/claimed for
these existing-helper status reads; evidence is command output and reviewed
allowlist, not simulated telemetry. Source research was not needed to establish
this runtime baseline, but is required before interpreting new gap semantics.

Research repository HEAD4637a5f, prior logger-diagnosis documentation dirty state
preserved. Added this report and current notices to IMU audit/data contract/
handoff/Vault proposal. Previous reports remain chronological history. No commit,
push or Vault update; no schema/validator/acceptance policy changed.

## Next action and rollback

Next is read-only comparison at full hash d6f12ad1: VehicleIMU gap condition,
BMI270 FIFO/timestamp code, logger/queue definitions, and relevant current rate
settings. Inspect param_import_fail.txt only as a read-only migration clue if
needed; do not reset/import parameters. No physical assembly or new acquisition
is required yet. A later capture needs a NEW baseline backup and separately
bounded approval; old v1.15.2 snapshots are never a restore target.

No device settings changed, so there is no device rollback. Pre-edit documents
are saved in `repo-before`; revert only this dated addition if desired while
preserving all historical evidence.
