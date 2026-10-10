# PX4 BMI270 offline ULog validation — 2026-10-08

## Replacement-card follow-up blocked — 2026-10-09

User reports card replaced. Read-only preflight in
`../px4-sd-control-20261009/SD_CONTROL_PREFLIGHT.md` found SD directory timeouts
with prior runtime still present; no new-card recording or configuration write.
Current advertised RAM parameters preserved before requesting full PX4 power
cycle and fresh SD/config audit. Card identity/health remains UNKNOWN; previous
matrix and old391-event result below unchanged. No M3C operation.

## New PX4-only hardware comparison — 2026-10-09

After the entirely offline continuation below, user reconnected PX4 and freshly
confirmed all propellers removed. New v1.17.0 actual64/128/64KiB results are in
`../px4-buffer-ab-20261009/PX4_BUFFER_COMPARISON.md` and
`../px4-buffer-ab-20261009/HANDOFF.md`.
All three remain lossy (336/140/306ULog dropouts);128KiB is NOT a loss fix.
Logger subscriber gaps now directly observed too. Capture/download/parse and
restoration VERIFIED; VIO-S0 remains PARTIAL. No M3C, firmware, flight-control
parameter, selection/rate or arming action. The old391-event experiment below
and its raw SHA256 are unchanged. Earlier "no new capture" statements describe
their historical tasks; this additive pointer supersedes only next-action status.

## Entirely offline continuation — 2026-10-09

Both M3C and PX4 are disconnected. New offline work is recorded in
`../offline-vio-20261009/OFFLINE_VALIDATION.md` and its `HANDOFF.md`.
Strict reusable FIFO export reproduces this original file's12648 gyro/11933
accel records and391 dropouts with the original SHA256 unchanged. No new hardware
capture, buffer change, loss-free PASS or synchronization claim. The prepared
new-firmware controlled capture is in
`../offline-vio-20261009/rm27/perception/vision/docs/PX4_NEXT_CAPTURE_PLAN.md`.
It needs only PX4 USB, not M3C, and has NOT been executed. This additive notice
does not alter the original experimental result or original-file manifests.

## Latest pointers — 2026-10-09

New v1.17.0 identity/IMU/logger readback is now verified in
`../px4-v117-baseline-20261009/PX4_V117_BASELINE.md`; exact-hash source comparison
is in `../px4-v117-source-audit-20261009/PX4_V117_SOURCE_AUDIT.md`. These supersede
the earlier not-yet-verified notice below. No new raw capture or loss-free PASS.

## Follow-up notice — 2026-10-09 (historical result retained)

Read-only reanalysis in
`../imu-logger-diagnosis-20261009/PX4_LOGGER_LOSS_DIAGNOSIS.md` extracted embedded
SD write counters: mean71.36544ms/max132.993ms, 248 calls; fsync max12.076ms.
This narrows the old failure to recording/storage-path buffer pressure, not a
proven SD-card-only fault. No second capture or fix was performed.
The user now reports flashing **PX4 v1.17.0**; its full hash, IMU instances and
logger settings are not yet verified. Everything below describes the OLD
micoair-v1.15.2 experiment. Its backup is NOT a restore target for new firmware.
VIO-P and VIO-S0 stay PARTIAL. Re-audit the new baseline before any comparison.
The original report is retained in the diagnosis `repo-before` directory;
the old manifest describes that original revision, not this additive notice.

## Result

**Capture/export/parse VERIFIED; complete loss-free offline IMU chain FAILED
for this configuration. VIO-S0 stays PARTIAL.** VIO-P reproducibility remains
PARTIAL unchanged. This is one short, stationary, disarmed bench test; not an
IMU noise calibration, synchronized camera dataset or flight qualification.

The user authorized logging-only changes, backup/restoration, and explicitly
confirmed all propellers removed. No arm, calibration, motor test, firmware,
flight-control parameter, camera, M3C, wiring or live-streaming operation was sent.

## Configuration backup and bounded changes

Actual FC: MICOAIR_H743_V2, micoair-v1.15.2, reported PX4 hash
4817c0618a1286846116e90c6eb8919efaa013cf. Inspection/capture control used Windows
COM19 USB CDC at its configured57600 setting, not a Linux port or UART rate test.
Preflight files: `identity-before.txt`, `preflight-backup.txt`, `ftp-before-v2.txt`.

Before: commander Disarmed, logger idle, `ps` command `logger start -b 64 -t`.
SDLOG values: BOOT_BAT0, DIRS_MAX0, MISSION0, MODE0, PROFILE1, UTC_OFFSET0, UUID1.
No `/fs/microsd/etc` directory/custom logger file existed (FTP root listing plus
console path failure). Thus the config backup includes original *absence*, not
an invented empty original file. Existing SD logs were not changed or deleted.

Temporary `/fs/microsd/etc/logging/logger_topics.txt`:

```text
sensor_gyro_fifo 0 1
sensor_accel_fifo 0 1
sensor_gyro 0 1
sensor_accel 0 1
vehicle_imu_status 100 1
sensor_selection 100 0
vehicle_status 100 0
actuator_armed 100 0
cpuload 1000 0
```

Intervals are milliseconds;0 requests every publication. This deliberately
replaced the default topic list only for this bounded test. SI/health/arming
topics support verification; FIFO records carry actual individual measurements.
Uploaded181 bytes were downloaded back with identical SHA256:
`c9c582bf74c856883a6d6b1a004f0e5f990a850e8cf2665dd09e18c2ff166328`.

Stopped the idle logger, started `logger start -b 64 -t -r 2500`, then used
`logger on` / `logger off` (logging-only override, not commander arming).
The default reader rate would undersample the shallow FIFO-topic queues;
2500Hz is logger polling, NOT sensor ODR. Buffer stayed64KiB. No parameter
set/save commands were sent. Intended dwell was15 seconds; status/command and
file startup/closure timing produced18.121956s ULog span and about17.9s observed
IMU span. Report measured intervals, not the intended dwell as exact duration.

## Preserved raw file and transfer verification

- FC original: `/fs/microsd/log/sess100/log100.ulg`, retained on SD.
- Local: `.maixpy/imu-offline-20261008/bmi270-disarmed-01.ulg`.
- Size6,896,444 bytes, matching logger close message and both FTP downloads.
- SHA256 `d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d`.
- Independent second download `bmi270-disarmed-01-check.ulg` is byte-identical
  by SHA256/size. Local CRC32 is081510f7; remote MAVFTP CRC query returned a
  generic failure, so no remote-CRC success is claimed. Two downloads and
  successful parsing are the actual transfer evidence.
- pyulog1.2.4 reports `file_corruption=false`; no repair/rewrite of original.
  Download timing is post-capture file transfer, not a live IMU throughput test.
- Linux evidence copy: `/home/shiuhou/Projects/rm27-vio-20261007/imu-offline-20261008`.

## Actual IMU records

Both FIFO topics have multi_id1 and device_id3604506: **BMI270**, not BMI088.
Every captured FIFO record contains samples=1. Counts*scale yields rad/s for
gyro and m/s^2 for accel; scale values are0.0010652969358488917 and
0.004788403399288654 respectively. Pre-estimator raw means sensor-filtered,
driver-rotated measurements, not untouched ADC output or calibrated body data.

| Topic instance1 | Records | Sample-time span s | Logged effective Hz | Median interval us | Max gap ms | Gaps >1.5x median |
|---|---:|---:|---:|---:|---:|---:|
| sensor_gyro_fifo | 12648 | 17.896324 | 706.681 | 633 | 110.776 | 202 |
| sensor_accel_fifo | 11933 | 17.893792 | 666.823 | 633 | 111.409 | 730 |
| sensor_gyro | 12712 | 17.899489 | 710.132 | 633 | 110.144 | 202 |
| sensor_accel | 12620 | 17.898856 | 705.017 | 633 | 110.776 | 202 |

These effective rates include recording gaps, not the sensor's actual ODR.
vehicle_imu_status reports approximately1579.762Hz for both raw gyro and accel;
the633us local cadence agrees. FIFO dt metadata remains625us (nominal1600Hz),
which differs from observed cadence by about1.28%. Do not retime the samples to
625us or call the metadata a measured clock. All batch sizes are1, so this run
does NOT validate multi-sample last-anchor reconstruction.

All four timestamp_sample series are strictly increasing, with zero duplicate
or backwards timestamps. Publication timestamps also ordered. FIFO publication
minus sample time: gyro median74us (70--132us), accel median70us (67--129us).
These are same-FC-clock software intervals, NOT end-to-end latency or precise
ADC-to-publication delay. FC HRT epoch is retained; no UTC/camera time mapping.

FIFO pairing:11930 identical gyro/accel sample timestamps and matching sample
counts;718 gyro-only timestamps,3 accel-only. Do not silently zip arrays by row.
No nonfinite values, integer-rail samples, SI clip counters or reported sensor
errors observed. Raw accel norm mean10.09758m/s^2, gyro norm mean0.0059104rad/s;
these uncalibrated bench values are not noise/extrinsic calibration results.

FIFO-to-SI trapezoidal-average cross-check (only contiguous retained FIFO pairs):
gyro12404 batches, max absolute error4.075e-10rad/s; accel11163 batches,
3.725e-7m/s^2. This verifies scaling/association with PX4's published SI path;
it also confirms sensor_* must not be mislabeled as identical raw FIFO values.

## Loss evidence and bounded diagnosis

ULog has **391 dropout events**, duration sum9804ms, maximum110ms. Console
mid-run status reported339 events before the end; this is a different observation
window, not a contradiction. Source logger increments write_dropouts when its
writer rejects a message. Thus recording-stage loss is established independently
of the later USB transfer. It does not yet isolate SD media speed versus buffer
capacity/scheduling or establish that every missing sample has the same cause.

Nominally comparing time gaps to observed633us cadence estimates15625 missing
gyro FIFO records and16336 missing accel FIFO records within their respective
observed spans. These are estimates, NOT hardware sequence-counter measurements;
no exact sensor-level loss percentage is claimed. Dropout-duration sums are not
an IMU loss fraction. No synthetic samples were inserted.

The message has fixed32-entry arrays but every captured record holds1 sample,
so padding/metadata create substantial logging load. Additional accel-only
gaps could involve its shallower queue as well as logger write losses; not
dynamically isolated. Post-run driver errors/overflow/DRDY counters are0, and
VehicleIMU gaps are0. Logger start calls `perf_reset_all()` in the inspected
source: pre/post perf counts must NOT be subtracted as if cumulative, and their
reset is not evidence of a reboot. FC commander PID remained618 in both ps lists.

No automatic second capture, buffer enlargement, sensor-rate reduction or
firmware change was performed to hide this failure.

## Safety and verified restoration

All observed heartbeat base_mode values29 (armed bit clear), ULog actuator_armed
values onlyfalse (31 records), vehicle_status arming_state only1 (17 records),
pre/post commander Disarmed. Due to log gaps these records alone cannot certify
every instant; combined with the monitored heartbeat and no arming commands,
they are the actual disarmed evidence. Propeller removal is USER-CONFIRMED.

Stopped recording and temporary logger; deleted only our temporary181-byte topic
file and two newly-created empty directories. Their contents are recoverable
from local config/readback evidence. Restored `logger start -b 64 -t`, verified
by fresh ps. logger status is Not logging/mode all. All seven SDLOG values match
backup; ULog changed_parameter_count=0. Custom logging path absent again.
Original SD ULog remains. Restoration record: `restoration-readback.txt`.

Runtime subscription count changed142 ->147 after restart; do not claim an
identical runtime snapshot. The source conditionally adds optional advertised
topics; exact five-topic difference is not resolved. Configuration restoration
is evidenced by original command, profile, all SDLOG values and file absence.

## Tooling validation and limitations

Windows Python3.12, pymavlink2.4.49, pyserial3.5, numpy2.5.3. Only pyulog1.2.4
installed in an isolated research venv using existing system scientific libs;
no production requirements changed.5 guard tests plus3 analysis synthetic tests
pass; these test command restrictions, disarmed/fresh heartbeat, bounded paths,
Windows download path/no-overwrite, timestamp ordering/gaps and FIFO scaling.
They are NOT substitutes for real ULog evidence. Main RM27/ROS2 regressions were
not rerun: no production adapter/estimator/schema implementation changed.

Preserved helper failures: missing MAVFTP master source fields (fixed locally),
library default Unix /tmp download path on Windows (fixed to guarded local .part),
remote CRC generic failure (not worked around by claiming CRC success). No
capture started before config readback succeeded. File helpers are one-off
bench tools, not a qualified production acquisition stack.

## Next step

Diagnose the offline logger write/buffer/SD bottleneck before accepting a VIO
dataset. A later logging-only controlled comparison may change buffer/topic
load, with the same backup/restore discipline; it is not executed in this run.
No physical action, assembly or camera calibration is required now. Keep VIO-S0
PARTIAL and do not start M3C/real-time transport or touch VIO-P's gate.
