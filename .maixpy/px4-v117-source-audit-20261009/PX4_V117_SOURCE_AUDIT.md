# PX4 v1.17.0 VehicleIMU gap and logger source comparison — 2026-10-09

## Result / authority boundary

**Gap semantics established; runtime trigger not isolated. Upgrade does not
establish loss-free capture. VIO-S0 and VIO-P remain PARTIAL.** This pass read
source and existing evidence only. No COM19/M3C session, firmware/parameter
write, IMU selection, logging start, new capture, calibration, commit or push.

Compared exact upstream commits, not moving release branches:

- OLD: `4817c0618a1286846116e90c6eb8919efaa013cf` (old FC-reported micoair-v1.15.2).
- NEW: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` (new FC-reported Release1.17.0).

Both hashes came from real device readback in prior passes. The new hash exists
in upstream PX4. Source-to-reported-hash matching is not binary reproduction.
Full source URLs/SHA256 and diffs are in `source-validation.json` / `diffs/`.

## 1. Exact VehicleIMU gap trigger

New `src/modules/sensors/vehicle_imu/VehicleIMU.cpp:288-292,417-421`:

```cpp
if (_sensor_accel_sub.update(&accel)) {
    if (_sensor_accel_sub.get_last_generation() != _accel_last_generation + 1) {
        _data_gap = true;
        perf_count(_accel_generation_gap_perf);
    }
    // ...
}
```

Gyro has the same test. After the read, the previous-generation value is updated.
The status labels `accel data gap` and `gyro data gap` name PC_COUNT counters
allocated in `VehicleIMU.hpp:198-199`. They count **successful subscription reads
whose publication generation is not consecutive**. They do NOT directly count:

- timestamp intervals over a threshold;
- individual raw sensor samples lost;
- FIFO chip overflow or sensor error_count;
- file-writer dropouts or missing ULog records.

One jump from delivered generation100 to103 counts **one event**, although two
uORB publications were skipped. Each publication may itself summarize multiple
samples. This counter alone supplies neither exact sample loss nor gap duration.
Consecutive generations can still have an abnormal timestamp interval without
incrementing this counter. Timestamp checks are in the non-gap branch and are
separate from the generation test.

### Where the jump comes from

`Subscription::update()` calls `Manager::orb_data_copy(..., only_if_updated=true)`.
Manager checks availability, then `DeviceNode::copy()` reads the per-topic ring.
If a subscriber cursor is outside the retained generation range, it is advanced
to the oldest available record. The returned generation jumps. The publisher
increments generation per uORB write, not per ADC sample.

Source: `platforms/common/uORB/Subscription.hpp:143-151`,
`uORBManager.cpp:448-458`, `uORBDeviceNode.hpp:225-260`,
`uORBDeviceNode.cpp:189-200`. Each subscriber has its own cursor; a logger read
does not remove a message from another subscriber's cursor. Scheduling/resource
interference remains possible, but subscribers do not consume a shared FIFO
destructively.

**Startup qualification:** VehicleIMU's own previous-generation fields start
at0. A subscription to an already-publishing topic starts near its latest
generation (`Subscription.cpp:45-59`, `DeviceNode.cpp:454-463`). Its first read
can therefore increment the counter without proving missed publications since
that subscription was created. Historical nonzero totals cannot all be labeled
steady-state overruns. This does not by itself explain the later7->8/4->5
increases; instance lifecycle and scheduling around those events were not traced.

### Which queue matters

VehicleIMU subscribes to **sensor_accel and sensor_gyro**, both queue length8,
not to their FIFO-array topics (`VehicleIMU.hpp:109-110`; both SI msg files:18).
The separately observed accel-FIFO queue1 and gyro-FIFO queue4 affect a FIFO
subscriber such as the logger; they are NOT VehicleIMU's queue sizes.

At the observed BMI270 publication cadence633us, eight publication periods are
about5.064ms. This is a rough retention budget, not a measured scheduling stall
or an exact universal deadline: existing backlog reduces margin.

## 2. Scheduling/recovery and the limit of causal attribution

`VehicleIMU::Run()` resets its per-run `_data_gap` flag, schedules a backup run,
drains bounded queues, and publishes when the integrators are ready. On a
generation gap it sets catch-up behavior. It still sends retained values to the
integrators with timestamp-derived dt; it does not recover omitted raw samples.
This is not a repaired loss-free dataset.

Callback registration is on gyro. `UpdateIntegratorConfiguration():681-740`
derives callback update count from the integration interval and measured gyro
publication period, and sets the backup timeout to half the smaller queue-time
budget, constrained1–20ms. Callback `ScheduleNow()` enqueues work; it does not
guarantee immediate execution (`SubscriptionCallback.hpp:163-171`).

Illustration ONLY: with IMU_INTEG_RATE200 and633us gyro messages, rounding gives
eight gyro messages per integration; the algorithm can request an8-update
callback, while the backup timeout is about2532us. The **current rate parameter
was not read** in the new baseline, so this is not a claim about its configured
callback count or a proven explanation for the live gap.

Old vs new function-text assertions establish that `Run`, `UpdateAccel`,
`UpdateGyro` and `UpdateIntegratorConfiguration` are identical. The only diff in
the full VehicleIMU.cpp moves `_notify_clipping` parameter refresh from the
constructor to ParametersUpdate; VehicleIMU.hpp is byte-identical. BMI270.cpp
and BMI270.hpp, four sensor message definitions, uORB DeviceNode and callback
implementation are also byte-identical. Subscription.hpp was refactored to
call subscribe() directly; that function still immediately returns true when
already subscribed. No changed ring-overrun algorithm was found in this scope.

Thus the existing idle-logger observation (BMI270 consumer counters accel7->8,
gyro4->5) is evidence of consumer-generation discontinuities under the inspected
logic, not proof of a new sensor-driver regression or SD dropout. Work-queue
latency, backlog, lifecycle and possible diagnostic observer load still need
time-correlated evidence. Driver errors staying constant does not prove all
upstream physical samples existed. No change of sensor or flight-control
settings is justified merely by these source observations.

## 3. Logger changes relevant to this investigation

| Area | Actual old -> new difference | Consequence / limit |
|---|---|---|
| Buffer allocation | NuttX first allocation now consults `mallinfo().mxordblk`, rounding available space minus1KiB and applying a minimum; requested size can be reduced with a warning | `-b64` is requested size, not proof of a65536-byte allocated buffer. Inspect actual capacity on any later authorized run. |
| Backend setup | `SDLOG_BACKEND` drives `rc.logging`:1 file,2 MAVLink,3 all,0 no startup | Explains observed explicit `-m all`; enables backends, not proof of active MAVLink streaming. Disable-by-SDLOG_MODE=-1 is not the new startup condition. |
| logger_status | Adds `is_logging`, zero-initializes status, publishes each type when this function is reached | Its only call in Run is still inside Full-log-started branch. Do not claim continuous idle status publication merely from the new field. |
| Default profile | More/generalized optional estimator topics plus other topic additions/removals | Can contribute to subscription-count differences, but183 vs old142/147 is not attributed exactly without per-topic snapshots. |
| High-rate sensors profile | New bit11 includes distance_sensor/optical_flow/GPS/mag | Not a raw BMI270 FIFO fix or all-IMU-full-rate mode. |
| Watchdog diagnostics | Preserves watchdog reason during pending load measurement and reports trigger type | Better diagnostics, not measured throughput improvement. |
| Metadata/build | Conditional boot_time_utc_us; configurable logger stack; log-root macro and encryption packaging changes | UTC metadata is not camera synchronization. Encryption changes are conditional, not evidence encryption is active here. |

Buffer detail (`log_writer_file.cpp:649-673`): for full log the minimum is
4096+300 bytes. The new cap uses the **largest contiguous free heap block**, not
total free RAM. Allocation can still fail. The cap runs when the buffer pointer
is null; do not assume it re-evaluates on every later file. The previous baseline
statement '64KiB buffer' must be read as **requested64KiB, actual unverified while
idle**. In contrast the OLD active logger status directly reported65536B.

## 4. Important logger behavior that did NOT change

- `copy_if_updated()` still counts subscriber generation skips separately in
  `_message_gaps` for full-rate subscriptions (`logger.cpp:444-462`). This is a
  LOGGER consumer count, not the VehicleIMU PC_COUNT and not writer dropout.
- File writer still rejects a record when record+dropout marker exceeds free
  ring space (`log_writer_file.cpp:535-561`). The method is identical.
- Writer still reclaims bytes only after write/optional fsync completes
  (`log_writer_file.cpp:416-430,696-707`). Core write_to_file method identical.
  fsync trigger remains100 writer polls, >1s elapsed, or requested sync.
- `Logger::write_message()` dropout bookkeeping is identical; high_water still
  resets on a new dropout and is not updated while one is active
  (`logger.cpp:1201-1226,844-848`). Zero high-water still does not rule out overflow.
- Default requested reader period remains3500us; -r sets1e6/r. FIFO PROFILE bits
  still override it to800us and raise priority (`logger.cpp:254-276,533-545`).
  Merely putting FIFO topics in a custom file with profile1 does not set those bits.
- Custom logger_topics.txt still replaces the profile list when nonempty.
  FIFO profile add_topic still defaults to instance0. Explicit identity/instance
  binding remains necessary to capture BMI270 instance1 instead.
- Default sensor_accel/sensor_gyro intervals remain1000ms, not1000Hz. Fixed32-entry
  FIFO definitions did not become compact variable-length messages. No automatic
  elimination of the previous high-overhead format was found.

Three distinct boundaries must stay separate:

| Counter/evidence | Boundary | Meaning |
|---|---|---|
| VehicleIMU accel/gyro data gap | SI uORB queue -> VehicleIMU | nonconsecutive generation on a read |
| logger_status.message_gaps | subscribed uORB queue -> logger | aggregate full-rate subscription generation skips |
| logger write_dropouts / ULog O | logger -> output writer | write rejection episode; old capture proves file recording loss |

None directly measures exact hardware sample-loss percentage. A larger logger
buffer addresses only the last boundary and cannot recover VehicleIMU messages
already skipped at its own subscription boundary. Conversely VehicleIMU gaps do
not prove the independent raw-FIFO logger must lose the same records.

## 5. Evidence, validation, next step

Windows root: `.maixpy/px4-v117-source-audit-20261009/`; host mirror:
`/home/shiuhou/Projects/rm27-vio-20261007/px4-v117-source-audit-20261009`.
Source line numbers above refer to NEW; full paths are encoded with `__` only
in the local evidence filenames. Canonical URL base:
https://github.com/PX4/PX4-Autopilot/tree/d6f12ad1c4f70ad3230afd7d86e971421e02fef4

Command: `.maixpy/imu-offline-20261008/venv/Scripts/python.exe
.maixpy/px4-v117-source-audit-20261009/verify_source.py`.
Observed result: **23 paired files,13 byte-identical;16 source assertions and7
illustrative queue-model cases passed**. Cases include consecutive reads,
retained backlog, overrun, late subscription/startup, uint32 wrap, no new update
and single-entry queue. This is source/model verification, NOT a compiled PX4
test or dynamic causal proof. Original source bytes are retained with SHA256;
generated diffs and manifest permit independent review. No main regression claim.

Retrieval exceptions retained in this record: new params.c and the guessed
src/modules/uORB/Subscription.hpp path returned404; GitHub tree resolved them
to logger/module.yaml and platforms/common/uORB. A PowerShell regex quoting
failure was bypassed by direct source reads; not evidence of absent code.
No source files were modified. Existing raw ULog and prior documentation retained.

Next minimally invasive check: read current IMU_INTEG_RATE, relevant work-queue
status and bounded before/after VehicleIMU/driver counters with a quiet interval
between them. Record process/instance continuity and host times. This can test
whether gap growth persists without the longer console inspection, but still
does not isolate exact run latency without further instrumentation. **Not executed
this pass.** No new logger capture or firmware change is needed for that check.
Any later capture needs new-version backup/restore and actual buffer-capacity
verification; do not reuse the old config backup or call cross-version data a
buffer-only comparison. No physical assembly requested.

Engineering state: research HEAD4637a5f plus prior dirty diagnostic docs preserved.
This source audit updates report/current notices/handoff only; no acceptance,
schema, algorithm, raw data, Vault or Git publication changes. Pre-edit docs
are retained in repo-before for additive-documentation rollback.
