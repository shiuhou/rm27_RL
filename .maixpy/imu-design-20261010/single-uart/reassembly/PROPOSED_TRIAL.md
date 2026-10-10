# Proposed single-UART ULog qualification experiment

Status: PLANNED, NOT EXECUTED. No spare UART is required.
Approval scope would be one reversible60-second data receive window after startup,
with a bounded overall startup/cleanup timeout. Connect existing M3C/PX4 UART and
keep independent PX4 USB console control. All propellers removed, Disarmed, EV_CTRL0.

## Exact intended communication changes

- Temporarily restart ONLY the PX4 MAVLink instance owning `/dev/ttyS2` at921600,
  Onboard, max TX datarate46080B/s, no flow control. Pinned source stop_command supports
  `mavlink stop -d /dev/ttyS2`; never use stop-all. Preserve optical-flow/ToF port.
- M3C `/dev/ttyS2`921600/8N1/raw/no-flow during this trial, single exclusive owner.
  Preserve original termios for finally restoration. Verify baud support before change.
- Preserve or back up the actual current logger topic file/absence byte-for-byte.
  Proposed minimal file (topic intervals ms, final column instance):

```text
vehicle_imu 0 0
vehicle_imu_status 100 0
sensor_selection 100 0
timesync_status 100 0
logger_status 1000 0
cpuload 1000 0
vehicle_status 1000 0
actuator_armed 1000 0
```

- Temporarily use the existing logger MAVLink backend with `-p vehicle_imu`, which
  wakes on that topic instead of a fixed polling loop. This candidate requires fresh
  confirmation that selected BMI088 maps to vehicle_imu instance0 and the polling
  subscription is instance0. Do not change selection or integration rate to fit it.
- Stream owner sends MAV_CMD_LOGGING_START2510, LOGGING_ACK268 as required,
  then MAV_CMD_LOGGING_STOP2511, using existing enums and source1/component191.
  It also maintains1Hz companion heartbeat. ACK traffic means this is not passive.
  Do not send synthetic ODOMETRY in the first acquisition trial: source completeness
  and startup/tail behavior must be qualified first. Existing Stage A/B remains saved.
- No PARAM_SET/save, firmware, wiring, camera, setpoint, calibration or EKF operation.

These describe a reviewable candidate; no live runner has been implemented/deployed.
The offline reassembler cannot be used as an ACK controller without separate runtime
guard, timing, bounded stop and restoration checks. It buffers a finite log.

## Evidence and restoration

Before mutation, freshly read exact firmware hash, commander/EV0, device ownership,
full actual MAVLink startup command/configured stream mapping, logger command/topics,
selected sensor IDs, source publication rate and M3C termios. Back up only logging
configuration; do not restore unrelated user flight parameters. Retain existing logs.
If state differs from the proposed profile, report it rather than auto-apply older
backups. Retain independent USB control through UART restart/startup failures.

Store every wire byte from startup; annotate receipt monotonic time, logging sequence,
header ACK/retry, loss/CRC/binding/tail status. Report IMU intervals/rates and coverage,
sample/publication HRT, IDs/dt/clips/calibration changes, CPU and byte budgets.
Capture source/ulog/logger counters around the accepted interval where available.
They are discontinuity events unless proven sample counts. Source rate~198Hz is not
strict>=200Hz PASS, and reception alone does not establish a camera clock map.

On timeout/error: stop streaming first, close sole M3C owner, restore exact termios,
logger command/topic bytes or original absence, original PX4 UART command and stream
mapping. Verify Disarmed/EV0,115200 wired link and unrelated flow/ToF ownership.
Archive all partial bytes/errors; no automatic retry, rate reduction or sensor change.

Criterion: all observed faults reported, no unexplained discontinuities or decode/
ordering/clipping/calibration-epoch errors in a proposed accepted interval; independently
account startup and tail. This is transport evidence only. Unknown source completeness,
integrated observation model and camera time still prevent OpenVINS admission.
