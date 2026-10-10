# Single-UART IMU transport review — 2026-10-10

## Scope and decision

User reports no second available UART. Treat that as a design constraint, not a
measured pin inventory. It supersedes the spare-UART action in earlier reports.
No physical pin verification is now requested. Reuse existing wired MAVLink2 link.
This is source/schema/budget research only: no serial/SSH, capture, settings,
firmware, dependencies, calibration, fusion or Stage C. VIO-S0 remains PARTIAL.
Stage A/B results remain historical verified results, not repeated here.
Working tree main b8e7299cf090f5150f9f262ebd3b232a824cc526, existing dirty work retained.

**Recommendation:** before a custom firmware protocol, evaluate the stock MAVLink
ULog streaming path as a bounded acquisition/qualification prototype. It can carry
complete vehicle_imu records on the same MAVLink UART as reverse ODOMETRY.
It is not a ready real-time OpenVINS frontend, nor proven loss-free. If its logger
latency/loss behavior is unsuitable, the fallback is a minimal dedicated MAVLink
stream with an explicit versioned payload and counters, requiring a firmware change.
Do not put independent DDS and MAVLink owners on this UART.

## Existing messages: what is actually preserved

Source pin: PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4. Installed host decoder:
pymavlink 2.4.49, explicit dialects.v20.common; not a fresh M3C package inventory.
Reuse local imu-input/imu-transport exact-hash source and measurements.

| Option | Source and semantics | Important missing information / limitations |
|---|---|---|
| HIGHRES_IMU | Selected vehicle_imu by matching accel ID; delta/dt, then matching estimator bias subtraction; gyro sample-end HRT in time_usec | No two dt, full hardware IDs, clipping or calibration epochs. Source zero-initializes id but never assigns it. Increasing rate cannot restore these fields. |
| SCALED_IMU / 2 / 3 | Fixed vehicle_imu instances; integral means quantized to mg and mrad/s, no HIGHRES per-message bias subtraction | timestamp is publication timestamp/1000, not timestamp_sample; no two dt, IDs/calibration epoch. Magnetometer-only update can resend copied IMU. Not raw samples. |
| RAW_IMU | Common schema has microsecond time and small instance ID | No exporter registered in this pinned mavlink_messages.cpp; name alone does not provide an unfiltered raw-uORB stream. No integral dt or paired hardware IDs. |
| HIL_SENSOR | Simulator sensor-input schema | Not an output vehicle_imu transport; do not enable HIL or inject this into PX4. |
| TUNNEL / V2_EXTENSION | Existing generic byte envelopes | Would still require an IMU producer, defined payload/identity/units/sequence contract and firmware work. Not an existing vehicle_imu exporter or a reason to hide custom semantics. |
| MAVLink ULog | Existing logger serializes subscribed uORB records, including original times, deltas, IDs, dt and calibration counters | Requires live log start/ACK handshake, chosen topics and logger polling, incremental ULog framing/format parser, loss handling; logger and transport queues can lose data. |

Thus no reviewed stock IMU telemetry message is a complete metadata-preserving
substitute. This does NOT mean all stock transports require new firmware: ULog is
the important existing alternative. sensor_gyro/accel can also be logged, but their
much higher native rates and filtering/calibration remain separate obligations.

## Stock ULog: benefits and source-verified costs

Pinned logger supports `file|mavlink|all`. start_log_mavlink emits header, formats,
parameters and topic bindings reliably before normal data. Receiver must ACK
LOGGING_DATA_ACKED and use the 16-bit logging sequence and first_message_offset to
recover/mark gaps. Ordinary LOGGING_DATA has no retransmission guarantee.
ULog streaming needs writes for start/ACK/stop: a future trial is NOT receive-only.

ulog_stream queue length is16. mavlink_ulog checks generation discontinuities and
counts missed ulog_stream events, not exact sensor samples. Its quota is quantized
over100ms; max messages = ceil(0.1 * 0.7 * MAVLink datarate / 267).
The logger fills249-byte chunks; no per-IMU immediate flush in the inspected writer.
At200Hz,65-byte records alone fill a chunk in about19.15ms on average, before
scheduling/wire time. This is a batching scale, NOT measured or maximum latency.
Reliable writes block the logger awaiting ACKs, including startup and later reliable
metadata operations. Source warns concurrent file logging can miss samples.
vehicle_imu queue1 may already lose records before the ULog packet exists.

A minimal topic profile and MAVLink-only bench logger backend could avoid SD writes
for the trial, with actual preflight backup/restoration, but would not prove the
historical SD issue fixed. Stock full logging profile is NOT covered by budgets below.
Existing SD logs must be retained. Live streaming has not been started or tested.

## Directional wire budgets (8N1, unsigned MAVLink2 unless noted)

MAVLink2 overhead12B; signing adds13B per frame; no byte-stuffing multiplier.
Use full payload lengths for conservative fixed-frame budgets, not zero-heavy
synthetic packet lengths (MAVLink2 trims trailing zero bytes).

| Traffic | Calculation | B/s |
|---|---|---:|
| HIGHRES_IMU200Hz | (63+12)*200 |15000|
| SCALED_IMU200Hz | (24+12)*200 |7200|
| vehicle_imu ULog200Hz, packed full chunks | (60+5)*200/249*(255+12) |13939.76|
| Same ULog, signed full chunks |13000/249*280 |14618.47|
| Hypothetical direct60B IMU payload200Hz | (60+12)*200 |14400|
| Reverse ODOMETRY10Hz + HEARTBEAT1Hz |245*10+21 |2471|
| Reverse signed version |258*10+34 |2614|

ULog numbers are steady-state full-chunk models, not total budgets or guarantees;
headers, metadata, heartbeat, status, ACKs and partial packets need explicit reserve.
The hypothetical60B direct message is a lower-bound design comparison, NOT a defined
dialect: source/boot sequence and additional diagnostics increase its payload.
Reverse direction has its own full-duplex wire capacity; do not add reverse ODOMETRY
to outgoing IMU bytes as if this were a half-duplex channel. CPU/queues still shared.

| Baud | Physical B/s/direction | Default PX4 datarate if -r=0 | Nominal70% ULog share |
|---|---:|---:|---:|
|115200|11520|5760|4032|
|230400|23040|11520|8064|
|460800|46080|23040|16128|
|921600|92160|46080|32256|

The actual saved link budget was5760B/s. Explicit nonzero datarate does not
automatically track baud; verify both independently in any future approved test.
230400 physical capacity could fit a minimal200Hz stream but default software
budget/ULog quota would not.460800 is plausible for200Hz ULog but nominal remaining
ULog allowance only2188B/s before headers/metadata;921600 offers more useful bench
headroom. Neither speed has physical qualification here, and no setting is changed.
400Hz doubles the IMU budget, not the reverse ODOMETRY budget. Current~198Hz source
does not become200/400Hz merely by increasing baud; original acceptance stays intact.

Existing evidence remains:50Hz requested gave50.001Hz with reduced competing streams;
100Hz requested gave62.112Hz at multiplier0.621, both at115200/5760. This establishes
software-budget throttling for those tests, not a universal hardware speed limit.
Startup invalid bytes remain unresolved; they must not be excluded silently.

## Next implementation boundary

1. Reuse the existing parser/receiver diagnostics and ULog tooling. First implement
   offline ULog-over-MAVLink reassembly fixtures if this prototype is selected:
   partial records, lost/wrapped logging sequence, formats and topic instances,
   duplicate/retransmitted ACKed headers, malformed length and fail-closed admission.
   A read-only UART receiver alone is insufficient because startup requires ACKs.
2. A future approved single-UART trial must specify baud AND software budget,
   minimal topic profile, logger poll/backend and exact restoration. One serial owner
   on M3C handles parser, log ACKs and later ODOMETRY/HB scheduling. Keep Stage A/B
   artifacts unchanged and EV_CTRL0. No live test is implied by this document.
3. Measure source/copy/log/transport counts separately, startup and steady latency,
   rates, p95/p99/max gaps, ordering, CPU, calibration/selection transitions and
   ODOMETRY coexistence. Packet sequence continuity is not source continuity.
4. Only if logger routing fails required latency/continuity, design a dedicated
   MAVLink stream with native metadata and per-source generation/drop counters.
   No message ID or firmware patch is allocated/implemented in this research turn.
5. Both paths still carry integrated measurements. Existing offline counterexamples
   show delta/dt is not universally equivalent to OpenVINS point input. Unknown
   accel endpoint, filtering, bias/calibration and camera clock mapping remain gates.

## Evidence and fresh validation

Source reads and local pymavlink schema inspection; `wire-check.txt` records fresh
offline library pack/parse checks and arithmetic, using synthetic data only.
These are framing checks, NOT ULog reassembly tests, hardware capture or VIO PASS.
No production code changed. Original frozen reports/manifests are not rewritten.

Primary references (PX4 links pinned, MAVLink reference is current supplemental):
- [HIGHRES_IMU](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/HIGHRES_IMU.hpp)
- [SCALED_IMU](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/SCALED_IMU.hpp)
- [Stream registry](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_messages.cpp)
- [ULog MAVLink transport](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_ulog.cpp)
- [ULog writer](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/log_writer_mavlink.cpp)
- [70 percent allocation](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_main.h)
- [Queue16](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/UlogStream.msg)
- [MAVLink serialization](https://mavlink.io/en/guide/serialization.html)
