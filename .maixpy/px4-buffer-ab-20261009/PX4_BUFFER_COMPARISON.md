# PX4 BMI270 actual-buffer comparison — 2026-10-09

## Scope and baseline

Logging-only experiment after the user reconnected PX4 and freshly confirmed all
propellers removed. M3C was not accessed. No arm, motor, calibration, firmware,
wiring, parameter set/save, IMU selection/rate, or live ULog streaming commands.
USB COM19, VID:PID1B8C:0036 is a USB CDC inspection/transfer path; its nominal
57600 setting is not a physical UART bandwidth measurement.

Fresh `identity-before.txt`, `preflight-readback.txt`, `heap-idle-before.txt`:
MICOAIR_H743_V2, PX4v1.17.0 hash
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, commander PID671/Disarmed.
Selected BMI088 instance0 remains gyro6684690/accel6946834; measured BMI270
instance1/device3604506 remains unselected. IMU_INTEG_RATE200 unchanged.
Original logger command `logger start -b 64 -t -m all`, Not logging.
SDLOG_BACKEND3, BOOT_BAT0, DIRS_MAX0, MISSION0, MODE0, PROFILE1,
UTC_OFFSET0, UUID1. Original `/fs/microsd/etc` absent, established by console
and MAVFTP root listing; absence is part of the fresh backup.

Initial free heap641248bytes, largest469472bytes. Quiet15s interval had no
growth in driver errors or VehicleIMU gap counters: instance0 accel1/gyro2,
instance1 accel3/gyro0 before and after. These historical counters are reset by
logger start; do not subtract later values or interpret reset as reboot.

## Controls and retained failure

Formal sequence A1R/B/A2 requests64/128/64KiB, logger polling2500Hz, same
201-byte10-topic file (see `logger_topics.txt`), same command cadence and15s
host target. Actual ULog/sample spans are authoritative. Each run restores the
original configuration before double download/parse and next run. Configuration
SHA256 `b629d918ba4fb6edef900b16b4b0999110e7950ed8e420ccdf5d69bb7d0f0dd1`.
This includes logger_status and is NOT a buffer-only comparison to oldv1.15.2.

Initial attempt `A1` is **ABORTED / excluded from the matrix**, preserved as
`A1-aborted.ulg` and independent check. It stopped because listener returned
Full AND Mission instances and the initial host parser required one flat
instance. Full allocation actually was65536bytes. File sess101/log100.ulg,
2175244bytes,5.974871s,106ULog dropouts; SHA256
`87ae746f986c7088af4f0e9798a32f03c61a9077f49acde0dc7371421ec08754`.
Restoration succeeded. The explicitly announced correction selects exactly one
Full type0 block, retaining active/backend/size checks. Added failing regression
tests then passed before new namedA1R. No failure erased or silently retried.

## Results

**Capture/export/restoration VERIFIED; loss-free chain FAILED for all three
configurations. VIO-S0 remains PARTIAL. 128KiB is NOT an accepted fix.**

| Measured metric | A1R (64KiB) | B (128KiB) | A2 (64KiB) |
|---|---:|---:|---:|
| Actual buffer bytes (live + ULog) | 65536 | 131072 | 65536 |
| Full ULog span s | 16.104257 | 15.599752 | 15.098964 |
| FIFO gyro/accel span s | 15.888478 / 15.886579 | 15.568983 / 15.567084 | 14.891709 / 14.889811 |
| ULog dropout events (including zero-ms) | 336 | 140 | 306 |
| Zero-ms events | 140 | 56 | 131 |
| Dropout duration sum / max ms | 8819 / 117 | 10379 / 1546 | 8325 / 190 |
| FIFO gyro / accel records | 11009 / 10383 | 8119 / 7693 | 10251 / 9626 |
| FIFO retained gyro / accel Hz | 692.829 / 653.508 | 521.421 / 494.120 | 688.302 / 646.415 |
| FIFO max gyro / accel gap ms | 118.372 / 119.005 | 1547.721 / 1547.721 | 216.492 / 216.492 |
| FIFO gaps >1.5x median, gyro / accel | 176 / 708 | 66 / 449 | 158 / 670 |
| FIFO common / gyro-only / accel-only timestamps | 10376 / 633 / 7 | 7689 / 430 / 4 | 9622 / 629 / 4 |
| Retained Full status records | 7 | 5 | 7 |
| Observed message_gap event first → last | 4 → 1199 | 4 → 1164 | 4 → 951 |
| Embedded SD write calls | 212 | 77 | 193 |
| SD write mean / max ms | 73.46032 / 133.477 | 198.39145 / 1610.903 | 75.93266 / 254.284 |
| Embedded fsync max ms | 18.838 | 51.209 | 13.042 |

B had fewer episodes but longer missing stretches and LOWER retained rate than
both A controls. Returning to64KiB recovered approximately the first A's retained
rate, still far below raw1579.74–1579.76Hz. This finite sequence provides no
evidence for adopting128KiB as a loss fix. It does not prove larger buffers
universally worsen logging: write chunking, SD internal state, file placement,
temperature and occasional long stalls were not independently controlled.
No further size/rate/topic/storage intervention was performed.

Writer rejection and logger subscription discontinuities are BOTH observed in
all runs. The status gap counters are event counts over different retained windows,
not exact lost-record totals; startup value4 also must not be ignored. Status
records themselves are lost. CPU load medians0.568/0.569/0.567 do not establish
global CPU saturation, but cannot rule out brief priority/scheduling starvation.
Live B reported buffer completely full131072bytes; retained status snapshots
missed that peak, illustrating why their maxima are not full-run maxima.

All retained FIFO records have samples1 and median633us anchor cadence versus
nominal dt625us. All four sample/publication series strictly increase; no
duplicates/backwards, nonfinite/rail samples or SI clipping/error counts observed.
Contiguous FIFO→SI trapezoidal association maximum errors across runs are
4.657e-10rad/s and3.679e-7m/s²; SI records are not identical instantaneous FIFO
samples. Time-gap estimates remain estimates, not hardware sequence counts.
No repaired, interpolated, row-zipped or retimed samples were created.

## Preserved files and transfer proof

Each new file was independently downloaded twice; exact byte equality, SHA256,
size versus FC close output, and pyulog parse integrity checked. Remote CRC was
not attempted; no remote-CRC claim. All SD originals are retained. Formal files:

| Local file (plus `-check.ulg`) | SD original | Bytes | SHA256 |
|---|---|---:|---|
| A1R.ulg | /fs/microsd/log/sess102/log100.ulg | 6007205 | e12d736ec15a940c4d5bf1e1d909108bc39055dea16e10eaffb23b120a725d06 |
| B.ulg | /fs/microsd/log/sess103/log100.ulg | 4467840 | b1d65035b4eba5e5044a6248cb6e119abe0da468f678522764130f3c27631e0e |
| A2.ulg | /fs/microsd/log/sess104/log100.ulg | 5597928 | 3815e9e8437d68c5ae1dd465c05d947937af31e30d097f6b24e1f30c3ebcbc20 |

Per-run `*-analysis.json`, `*-loss.json`, `*-export/report.json` and separate
gyro/accelJSONL preserve detailed metrics, scaling, times, association and hashes.
All parses report file_corruption=false and changed_parameter_count0. USB
post-capture download speed is not real-time IMU throughput or SD write speed.

## Source-supported interpretation

Exact-hash files reused from `../px4-v117-source-audit-20261009/source/new/`:

- `src__modules__logger__logger.cpp:444-462`: full-rate uORB generation jump
  increments message_gaps by ONE, not by the number of skipped messages. It is
  an aggregate event count, not per-topic or exact physical-sample loss.
- `logger.cpp:224-249,1035-1068`: console status resets writer interval counters;
  recorded status.dropouts is not cumulative across that reset. ULog O records
  count file dropout episodes; retain zero-ms episodes. Status publishes about
  once per second, regardless of requested100ms topic interval; lost status
  records and the tail prevent an exact final subscriber count.
- `src__modules__logger__log_writer_file.cpp:535-561`: insufficient free ring
  space rejects a record. Writer bytes are reclaimed only after write/optional
  fsync returns (`416-430,696-707`). Increased capacity may also change contiguous
  write sizes, so this is not a pure fixed-drain-rate reservoir experiment.
- `log_writer_file.cpp:649-673`: capacity may shrink to available contiguous
  heap. Both live Full status and retained ULog must establish actual allocation.

FIFO gyro queue4 / accel queue1 differ from SI gyro/accel queue8 used by
VehicleIMU. Zero VehicleIMU gaps cannot certify FIFO logger continuity. Observed
writer losses and subscriber gaps need not be statistically independent causes:
scheduling/recording pressure can affect both. No per-event scheduler trace or
SD-controller/media-only timing exists; a card-only diagnosis remains unsupported.

One-sample FIFO records occupy226bytes each versus49bytes each SI record.
At reported raw1579.762Hz, the four-topic model is868869bytes/s excluding other
metadata/topics. This is modeled offered load, not observed output or UART load.
Write perf is wall elapsed around `::write`, including scheduling/OS/SD path;
embedded postflight snapshots exclude final drain and are not full-file traces.

## Safety, restoration and limitations

Every run including the abort restored the fresh command, all8SDLOG values,
Disarmed state and original etc-path absence; see `*-session.json`, console
transcripts and final `restoration-final.txt`. Final logger Not logging/modeall;
commander PID671 unchanged. All observed heartbeat base_mode29; ULog armed only
false and arming_state only1. Sparse/gapped ULog alone cannot certify every instant;
these observations plus monitored heartbeat and no arm commands are the evidence.
Driver error/overflow/missed-DRDY and both VehicleIMU gap counters observed zero
after each formal capture (before restoration restart), not subtracted from
preflight counts. A1R embedded postflight perf omitted those counters, so console
post-run snapshots are the cited evidence for that run.

Existing old SD logs and local raw ULog remain
unchanged; old raw SHA256 still
`d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d`.
Only the session topic file and its newly created empty directories are removed
on restoration; content remains in local upload/readbacks. This restores config,
not identical runtime state: subscriptions180→187 after restart (optional topic
advertisement), not an unresolved parameter change or permission to claim exact
runtime identity. No synchronized camera data, clock mapping, ADC timestamp
accuracy, calibration or flight qualification is inferred.

## Tool verification and reproducible commands

New session helpers reuse original `bench_io.Link`, MAVFTP, heartbeat/path guards;
old capture CLI is not used because its selected-IMU and restore assumptions are
wrong for this firmware. `session_guard.py` restricts exact commands, named runs,
Full allocation/backend,8SDLOG values and remote log paths. `capture_session.py`
closes/stops recording in finally and restores only the owned temporary paths.
`analyze_session.py` reuses original analyzer/inventory and adds dropout-empty,
logger counter-reset and two-download checks. No production dependencies changed.

Executed from repository root:

```powershell
py -3 -m unittest discover -s .maixpy/px4-buffer-ab-20261009 -p 'test_*.py'
py -3 -m unittest discover -s .maixpy/imu-offline-20261008 -p 'test_*.py'
```

Results15/15 new and8/8 existing tests passed. RED failures were observed before
guard/session/status implementations and before the multi-instance correction.
An initially incomplete heap reply fixture was corrected to include actual
`free` output shape; no safety threshold was weakened. These tests are helper
checks, not substitutes for hardware records. Main VIO/ROS2 suites were not
rerun: no estimator/adapter/production-schema changes in this capture task.

For each formal label L=A1R,B,A2, executed `py -3
.maixpy/px4-buffer-ab-20261009/capture_session.py L` (DO NOT rerun completed
labels), then two guarded `bench_io.py get` operations using paths above.
Offline commands use existing `.maixpy/imu-offline-20261008/venv/Scripts/python.exe`:

```text
analyze_session.py L.ulg L-check.ulg L-analysis.json
../imu-logger-diagnosis-20261009/analyze_loss.py L.ulg --output L-loss.json
../offline-vio-20261009/rm27/perception/vision/localization/experiments/px4_ulog.py
  --ulog L.ulg --output L-export --instance 1 --gyro-device 3604506 --accel-device 3604506
```

Paths abbreviated relative to this evidence directory. All output targets were
new/no-overwrite. Console/download transcripts are PowerShell Tee output; source
JSON/exports are UTF-8. `MANIFEST.json` identifies final local evidence state.

## Remaining boundary and one proposed next physical action

**VERIFIED:** actual-buffer matrix, writer loss, logger generation discontinuity,
integrity/scaling and configuration restoration. **PARTIAL:** VIO-S0; prior VIO-P
status unchanged. **UNKNOWN:** SD-media-only cause versus filesystem/controller/
scheduling, exact physical sample loss, synchronized real-RM27 timing/calibration.
Further causal discrimination is **BLOCKED on a new controlled hardware condition
or user-approved implementation decision**, not on needing M3C for this test.

Recommended single next physical action, NOT performed/authorized here: with PX4
fully unpowered, substitute a user-approved spare microSD (old card preserved;
no formatting/deletion), then connect only PX4 USB for the SAME64KiB10-topic
control. It tests storage-medium/filesystem sensitivity of write stalls and loss;
a pass would not alone resolve subscriber gaps or qualify VIO. The new card must
have adequate space and its existing config/files backed up; stop if different
startup scripts/settings would invalidate the control. M3C remains unnecessary.
Selection/provision of a spare card is the user's next decision. No automatic
topic reduction, firmware patch, buffer enlargement, or claim that a card swap
will fix the issue. Detailed calibration/transport plans and completed offline
OpenVINS work remain in the unchanged offline handoff.
