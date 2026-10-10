# PX4 logger loss diagnosis — 2026-10-09

## Scope and current decision

**Historical v1.15.2 recording bottleneck localized; no fix or new capture
verified. VIO-S0 and VIO-P remain PARTIAL.** This is host-only reanalysis of
the 2026-10-08 ULog, not a fresh device observation.

During this analysis the user reported flashing **PX4 v1.17.0**. Its exact
hash/board target, current IMU instances, parameters and logging configuration
have NOT yet been read back. All source conclusions below are pinned to the
OLD reported hash `4817c0618a1286846116e90c6eb8919efaa013cf`. Do not apply the
old instance 1, restore old settings, or claim the new firmware retains or
fixes this failure. Establish a separate v1.17.0 baseline first.

No COM port, FC or M3C was opened in this diagnostic pass. No parameter,
logger setting, firmware, wiring, calibration, estimator or gate was changed.
No commit/push or Vault write. The original failed capture is retained.

## Immutable input and reproducible outputs

- Input: `.maixpy/imu-offline-20261008/bmi270-disarmed-01.ulg`, 6,896,444 bytes.
- SHA256: `d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d`.
- Evidence: `.maixpy/imu-logger-diagnosis-20261009/` on Windows; mirrored under
  `/home/shiuhou/Projects/rm27-vio-20261007/imu-logger-diagnosis-20261009/`.
- Final analysis: `loss-analysis-v2.json`; earlier `loss-analysis.json` is
  preserved and lacks only the embedded-performance extraction added in v2.
- Helper: `analyze_loss.py`, offline only. Independent binary-frame inventory
  accounts for every byte and agrees with pyulog topic counts. No sample repair,
  interpolation, retiming or ULog rewrite. Output creation is no-overwrite.

## New direct evidence: the ULog contains SD-call performance counters

The original report did not expand `perf_counter_postflight` metadata. It holds:

| Counter | Calls | Total elapsed | Mean | Maximum |
|---|---:|---:|---:|---:|
| `logger_sd_write` | 248 | 17.698629 s | 71.36544 ms | 132.993 ms |
| `logger_sd_fsync` | 30 | 0.230759 s | 7.69197 ms | 12.076 ms |

The old source wraps `::write()` and `::fsync()` separately with elapsed-time
performance counters. They include blocking and possible preemption; these are
NOT measured SD-card-only service times or CPU execution times. The write path
does not reclaim ring-buffer bytes until the write and optional fsync return.

The preflight SD counters are zero. Logger resets performance counters after
writing preflight metadata; postflight is a snapshot taken before final file
drain. Do not subtract unrelated preflight driver counters, or divide final
file bytes by this snapshot's elapsed time and call it a measured card benchmark.
The nonzero historical BMI270 FIFO-reset count is not a reset during this test.

This directly narrows the dominant problem to the **file-writer/storage path
failing to drain the 64 KiB ring buffer under this logging load**. Media speed,
filesystem/SD driver behavior, write-size effects and scheduler contribution
are still not isolated. A replacement SD card is not yet justified as a proven
fix. ULog is a lossy observation of sensors; it does not prove every absent
sample was lost at this same boundary.

## Actual record sizes and load

Binary parsing finds fixed 226-byte FIFO DATA records and 49-byte sensor SI DATA
records (each includes the 3-byte ULog header and 2-byte message ID). The FIFO
records contain all 32 slots even though every captured record has `samples=1`.
The unused array slots are not the same as compiler alignment padding.

| Topic, old BMI270 instance 1 | Bytes/record | Estimated full-rate bytes/s |
|---|---:|---:|
| `sensor_gyro_fifo` | 226 | 357,027 |
| `sensor_accel_fifo` | 226 | 357,027 |
| `sensor_gyro` | 49 | 77,408 |
| `sensor_accel` | 49 | 77,408 |
| Total four topics | 550 per gyro/accel sample epoch | 868,870 |

Model uses the median recorded raw-rate estimate 1579.7634 Hz and assumes one
record/sample throughout. It excludes metadata and other topics. The old
mid-run status recorded only 371.47 KiB/s file output; this is achieved output
for that window, not a general SD throughput ceiling.

An initially empty 65,536-byte buffer holds only **75.43 ms** of this modeled
input with zero drain; a 128 KiB buffer holds **150.85 ms**. These are optimistic
capacity calculations, not exact measured stall durations: in-flight writes
still occupy the buffer. Merely doubling a buffer cannot cure a sustained
input/output rate mismatch.

Removing the two SI cross-check topics reduces modeled load by only **17.82%**;
the FIFO pair still needs about 714,053 B/s. It is not a demonstrated fix.
Unused FIFO array entries alone account for about 587,672 B/s of the modeled
load. Firmware message/driver changes to batch or compact this data are outside
this task and were not made.

## Why status reported zero buffer use despite dropouts

At the pinned old source:

- `LogWriterFile::write()` returns `-1` when record plus dropout marker will not
  fit the remaining ring buffer (`log_writer_file.cpp:551-577`).
- `Logger::write_message()` starts a dropout and **sets `high_water=0`** on the
  first failed write (`logger.cpp:1198-1223`).
- High-water accounting skips periods with an active dropout
  (`logger.cpp:844-848`); printing status also resets the statistics.

Therefore the recorded `max used buffer: 0 / 65536 B` is not evidence of an
empty buffer or absence of overflow. Its accounting is compatible with an
ongoing/recent dropout. ULog's 391 O records independently preserve dropout
events; 158 have duration 0 ms due to integer-millisecond quantization. They
must not be discarded as no loss.

## Shared gaps and separate unresolved queue losses

- All **164 gyro-FIFO gaps longer than 10 ms** overlap a >10 ms gap in each of
  the other three high-rate topics. This supports a shared recording bottleneck;
  overlap alone does not identify individual write-call timing.
- In the common time window `[5951577709, 5969471501]` us, 720 accel SI timestamps
  lack a matching accel FIFO record, while 41 accel FIFO timestamps lack SI.
  Each timestamp refers to retained records, not a count of hardware samples lost.
- FIFO gyro/accel pairing within that common window is 11,930 shared, 714
  gyro-only, 3 accel-only. The original report's 718 gyro-only used whole-topic
  spans; the four-record difference is boundary trimming, not changed data.
- The old gyro FIFO explicitly declares queue length 4. Accel FIFO has no
  explicit queue constant; the default shallow queue remains a risk. At 633 us
  cadence, even a 400 us requested logger period is not a scheduling guarantee.
- Logger copies one update per topic per loop and tracks skipped generations
  separately as `_message_gaps`. No `logger_status` topic was captured, so
  logger subscription loss cannot be numerically separated from write losses.
- Custom FIFO entries with `SDLOG_PROFILE=1` do not activate the profile-bit
  branch that raises logger priority/sets interval 800 us. Actual embedded top
  snapshots show logger priority 230, file writer priority 60. Do not change
  priorities based on this observation alone.
- The 16 retained cpuload values span 47.94–50.67%; embedded top snapshots show
  roughly 49% idle. This does not support sustained whole-CPU saturation, but
  does not rule out short scheduling delays or SD I/O waiting.
- Postflight driver/VehicleIMU counters are zero for the observed relevant
  errors/gaps, as in the original report; these cannot certify losslessness.

ULog O messages contain duration only. pyulog attaches the last parsed DATA
timestamp to them. That is NOT a precise dropout start/end timestamp, so this
analysis does not invent exact dropout-to-sample-gap alignment.

## Source provenance

Read-only upstream retrieval through GitHub CLI/raw content at the old reported
hash, not the unrelated local PX4 checkout HEAD `dd0ad74`. Source binding to a
reported hash is not proof of a reproducible vendor binary.

Base: https://github.com/PX4/PX4-Autopilot/tree/4817c0618a1286846116e90c6eb8919efaa013cf

- `src/modules/logger/logger.cpp`: lines 444–462 (generation gaps), 532–545
  (profile-only FIFO scheduling), 755–795 (one copy/topic/loop), 844–848
  (high water), 1198–1223 (dropout), 1420–1465 (perf snapshot/reset).
- `src/modules/logger/log_writer_file.cpp`: lines 432–446 (write before reclaim),
  551–577 (overflow), 683–700 (write/fsync timer boundaries).
- `src/modules/logger/log_writer.cpp`: lines 163–180 (backend results).
- `src/modules/logger/logged_topics.cpp`: lines 558–569 (custom topic override).
- `msg/Sensor{Gyro,Accel}Fifo.msg`: fixed arrays/queue declaration.
- Local pyulog 1.2.4 `core.py:1096-1098`: parser-supplied dropout timestamp.

Fetched sources and SHA256 manifest are retained in the diagnostic evidence
directory; source files were not edited.

## Next verification — revised for user-reported v1.17.0

First read `ver all`, `logger status`, `ps`, `sensors status`, `bmi270 status`,
`param show SDLOG*`, and current custom logger-file presence/content via the
current USB connection. Confirm selected device IDs and topic instances anew.
No reboot, parameter writes, topic-rate commands or logging start is implied.
Use the new full hash to inspect logger/message/driver changes before planning
a comparison. Existing v1.15.2 configuration backups are **historical**, not a
restore target for v1.17.0. Wiring and MAVLink changes may also have happened
since the old capture; they must not be overwritten.

For the old firmware, a 64→128 KiB buffer-only comparison with the same topics,
polling, device, duration and SD would test transient-buffer sensitivity, subject
to free-memory checks. It would NOT prove media health or final dataset quality.
This is now a historical candidate, not an approved command for v1.17.0.
Any new capture needs a current-state backup, props-off/disarmed confirmation,
bounded logging-only approval and verified restoration to that NEW baseline.
Do not compare different firmware captures as a buffer-only experiment.

## Validation and limits

Commands (Windows repository root):

```powershell
.maixpy/imu-offline-20261008/venv/Scripts/python.exe -m unittest discover -s .maixpy/imu-logger-diagnosis-20261009 -p test_loss.py -v
.maixpy/imu-offline-20261008/venv/Scripts/python.exe .maixpy/imu-logger-diagnosis-20261009/analyze_loss.py .maixpy/imu-offline-20261008/bmi270-disarmed-01.ulg
```

Three synthetic parser/gap tests pass. Final JSON was recomputed and compared
with its preserved output; raw-file hash unchanged. These are diagnostic-tool
checks, not a loss-free capture result. No main RM27/ROS2 regression is claimed
for this host-only analysis/documentation task. No production implementation,
acceptance threshold, firmware or live settings changed.
