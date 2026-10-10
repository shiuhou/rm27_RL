# PX4 -> M3C IMU: rate control and vehicle_imu audit — 2026-10-10

## Verdict

- VERIFIED: the original ~10.11Hz HIGHRES_IMU rate is explained by the PX4
  MAVLink stream budget/scheduler. With competing periodic telemetry disabled,
  requested50Hz delivers50.001Hz without raising115200baud or5760B/s budget.
- VERIFIED: requested100Hz under that unchanged budget delivers62.112Hz, not100.
  The matching scheduler multiplier is0.621. A100Hz transport PASS is NOT claimed.
- PARTIAL: continuous, clean IMU input qualification. Both captures have startup
  invalid bytes;50Hz includes a CRC error. After first valid frame, observed packet
  sequences are continuous, but this is NOT proof of all sensor samples surviving.
- NOT QUALIFIED: HIGHRES_IMU as OpenVINS input, camera/IMU synchronization,
  complete vehicle_imu transport, spatial/temporal calibration. Stage C NOT STARTED.
- VERIFIED: all target-instance configured stream rates restored; exact temporary
  M3C termios restored after each test; final Disarmed and EKF2_EV_CTRL=0.

Working tree main at b8e7299cf090f5150f9f262ebd3b232a824cc526, uncommitted research.
Existing .gitignore/HANDOFF/VAULT_UPDATE/new_plan/tests/camera_capture.py/tmp edits
were preserved. No push/commit, dependencies, Vault access, camera, VIO, arming,
setpoints, parameter writes, firmware or wiring changes. Stage A/B evidence at
`../mavlink2-20261010/VALIDATION.md` is unchanged. Old micoair-v1.15.2 ULog evidence
is historical and was not rerun or used as a current configuration baseline.

## Experiment and restoration

Fresh `preflight.txt`, `control-session.jsonl`, `control-console.txt` identify
MICOAIR_H743_V2, PX4v1.17.0 hash
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
COM19 is USB console inspection, NOT the physical UART baud.
M3C and PX4 wired endpoints are /dev/ttyS2; PX4 instance1 is115200,Onboard,MAVLink2.

Explicit approval covers bounded temporary M3C115200/8N1/raw/no-flow and temporary
nonessential stream rates. Reused `../imu-input-20261010/receive_imu.py` unmodified:
O_RDONLY, zero UART bytes written, fresh disarmed heartbeat guard, <=30s, exact
termios restore. `run_control.py` uses the original USB Link with a narrow command
allowlist; it stores original stream rates before any change and restores all
attempted changes in finally. It refuses a second run with the same evidence file.

Protected HEARTBEAT,STATUSTEXT,SYS_STATUS,EXTENDED_SYS_STATE,PING unchanged.
TIMESYNC10->1Hz;25 other advertised periodic nonessential streams temporarily0.
Unlimited and inactive streams untouched. HIGHRES_IMU50 then100Hz. Original
software budget5760B/s, physical115200, sensor rates and selection unchanged.
Exact stream names/rates and all command replies are in `control-session.jsonl`.

Two30s captures, no extra trials. For comparable observer load, each contains
`listener cpuload -n 10`, status and stream-table reads over USB. CPU baseline
also uses10 samples. USB observer load is present; this is not an unloaded flight
performance benchmark. Status stream "current" rates are scheduler predictions,
not measurements; received raw bytes provide the separate measurement.

Restore record: `configured_rates_restored=true`, `restore_errors=[]`. The target
configured-rate mapping equals the saved mapping, including unlimited entries.
Final multiplier returns0.202; original HIGHRES50 again schedules~10.113Hz.
No claim that runtime counters or stream ordering are identical. M3C final
`stty -g` matches saved idle9600/canonical/echo, no receiver/sender process found
(`m3c-final.txt`; grep-no-match makes that compound command exit1, not a UART failure).
PX4 heartbeat observations only29, final commander Disarmed, EV_CTRL0. This is not
an exhaustive audit of every autonomous PX4 parameter change; no PARAM_SET was sent.

## Actual results

| Condition | Original passive baseline | Reduced traffic, request50 | Same reduced traffic, request100 |
|---|---:|---:|---:|
| Capture seconds |30.000706|30.003246|30.001860|
| HIGHRES records |303|1500|1864|
| Source-time effective Hz |10.11255|50.00105|62.11152|
| Scheduler multiplier |0.202|1.000|0.621|
| Source interval median / p95 / max ms |99.141 / not recomputed /105.150|19.529 /25.584 /30.045|15.022 /21.031 /25.586|
| Source interval stddev ms |not recomputed|4.063|2.748|
| Source gaps >1.5x median |0|4|63|
| Receipt median / p95 / max ms |not recomputed / not recomputed /176.660|20.775 /23.644 /25.133|15.269 /20.381 /43.216|
| All received bytes/s |4631.7|4225.709|4847.666|
| Valid framed bytes/s |not recomputed|4086.791|4846.133|
| PX4 CPU mean (10 samples) |51.4301% adjacent fresh baseline|50.5149%|50.5203%|

CPU maxima51.814%,50.823%,51.883%; TX error0 in observed status samples.
CPU is PX4 aggregate load, not M3C load or worst-case per-workqueue latency.
Source-time records strictly increase: no duplicates/backwards.100-request has
two equal software receipt timestamps (multiple frames in one read), NOT duplicate
sensor timestamps. P99 and complete interval statistics are in `comparison.json`.
Interval-gap heuristics measure irregularity; do not translate them to exact lost
sample counts, particularly with~5ms integration updates and asynchronous sending.

### Startup anomalies retained

50Hz:3 bad-data events totaling4168bytes, at13.339 and26.051ms after capture start,
all before first valid frame. Includes one HIGHRES CRC failure and an actual raw
read of4095 zero bytes. This is NOT merely a parser summary artifact: raw chunks
contain it and replay reproduces it. Cause UNKNOWN: UART reconfiguration/driver/
buffering or physical path not isolated. Do not erase or label it harmless.
100-request:46-byte invalid prefix4.306ms after start, before first valid frame.
Starting a receive stream partway through a frame can cause prefixes, but no
specific cause is proven for these records.

After first valid frame, both source1/1 packet sequences have0 observed gaps,
duplicates or resets (1930 and2177 frames). Known messages CRC-check successfully;
10 and18 frames from newer unknown dialect messages remain CRC UNVERIFIED.
8-bit sequence continuity cannot exclude whole wraps or unseen boundary packets,
and does not measure underlying IMU publication loss. Thus not an all-byte-clean PASS.

HIGHRES fields_updated includes all six inertial axes (63 or6719), units rad/s and
m/s^2; nonfinite0. Accel mean norms~9.8m/s^2, not gravity-removed acceleration or
noise calibration. Wire source1/1,id0 does not carry a hardware ID. Separate
`source-identity-final.txt` and source mapping establish selected BMI088:
gyro6684690/accel6946834,instance0; BMI2703604506,instance1 is NOT selected.
Selection timestamp remained the early-boot selection. No in-packet identity proof.

## Why10Hz, and what115200 can support

Exact source `../imu-input-20261010/src__modules__mavlink__mavlink_main.cpp`
update_rate_mult (1283–1352) budgets configured message sizes times rates, subtracts
constant-rate streams, then scales nonconstant streams. Default datarate is baud/20
(2203–2205):5760B/s, versus physical8N1 ceiling baud/10=11520B/s. The original
active theoretical periodic demand is~28292.5B/s (rounded displayed rates), including
ODOMETRY30*245, ATTITUDE100*40, ATTITUDE_QUATERNION50*60, HIGHRES50*75.
Original50*0.20226=10.113Hz agrees with measured10.11255Hz. Reducing competition
at unchanged baud/budget yields50Hz. Requested100 uses0.621 and yields62.112Hz.
This isolates configured stream budget contention as the primary rate limiter.
It does NOT explain all residual jitter or the startup byte anomaly; those remain open.

Conservative unsigned MAVLink2 HIGHRES wire reservation75B:
50Hz3750B/s;100Hz7500B/s;200Hz15000B/s;400Hz30000B/s, before other streams.
Observed payload truncation makes some frames shorter, not enough to make200Hz
fit115200.100Hz can fit physical115200 in principle with a larger software budget
and careful stream allocation, but that was NOT changed/tested. It is not enough
to request100Hz or raise baud while leaving the software budget unchanged.
At460800/921600, theoretical8N1 ceilings46080/92160B/s give headroom; these are
design budgets, not validated electrical throughput. No higher baud was attempted.

## Track B: measurements and exact semantics

Read-only `uorb top -1` in preflight (one short publication-rate snapshot):
sensor_gyro0=666Hz, sensor_accel0=802Hz; both instance1=1580Hz.
vehicle_imu0=198Hz,vehicle_imu1=194Hz,sensor_combined0=198Hz.
IMU_INTEG_RATE=200 is configuration, not measured exact output.
VehicleIMU queue1; SI gyro/accel queues8. Queue depth alone does not guarantee
delivery. `listener` polls and its five samples do NOT establish full-rate continuity.

Observed vehicle_imu0 gyro dt4460/6056/4459/6058/4507us versus accel4992–4994us;
instance1 both dt5064–5065us. IDs,clipbits0,calibration counts retained in console.
Final VehicleIMU counters0: accel1/gyro2 gaps;1:accel221/gyro0. These are cumulative
since the relevant counter reset; no pre/post matched delta was collected for
these tests, so do NOT attribute all to this experiment or claim sensor-level loss0.

| Source | Values and processing | Time / identity | OpenVINS implication |
|---|---|---|---|
| sensor_gyro / sensor_accel | Driver-rotated, SI-scaled; FIFO path trapezoid-averages a batch, includes sensor-side filtering | Separate timestamp_sample,publication timestamp,ID,samples,clips/errors | Not universally individual ADC samples; frame/calibration/filter and batch timing must be qualified |
| vehicle_imu | Trapezoidal delta_angle(rad),delta_velocity(m/s); gyro coning correction; offset/thermal/scale/board rotation | Gyro last sample anchor + publication time; separate dt(us),twoIDs,clips,calibration counters | Best of these for preserving integral semantics; NOT directly an OpenVINS ImuData message |
| sensor_combined | Selected vehicle_imu delta/dt, without HIGHRES current bias subtraction | Gyro anchor,relative accel anchor,dt,clips/calcounts; no hardwareIDs | Existing DDS diagnostic option, but stock export skips updates and omits device identity |
| HIGHRES_IMU | Selected delta/dt then matched EKF bias subtraction | Gyro anchor time_usec; no dt,hardwareIDs,clip/calcounts | Processed telemetry; unsuitable as an unqualified raw inertial input |

VehicleIMU uses `_gyro_timestamp_sample_last` for timestamp_sample and HRT at publish
for timestamp. Accel end time is not independently present. In VotedSensorsUpdate,
the accel anchor is also assigned vehicle_imu.timestamp_sample before computing
the relative timestamp; that field is NOT proof of exact ADC accel/gyro simultaneity.
Different integral lengths require separate interpretation, not assuming one dt.
Do not subtract gravity or treat delta_velocity as world-frame velocity.

Calibration caveat: VehicleIMU Publish does not do HIGHRES's per-message EKF bias
subtraction. However SENS_IMU_AUTOCAL is freshly read as1; VehicleIMU has gated
estimator-bias learning and disarmed calibration-save logic. Thus NEVER describe
vehicle_imu as universally EKF-independent or raw. Preserve calibration epochs and
reject/segment changes. This task did not alter autocal or initiate calibration.

Pinned OpenVINS69488123ed9362dd44b6f28e7f4680abbff1442b consumes ImuData timestamp,
wm(rad/s),am(m/s^2), interpolates values, integrates between timestamps, and estimates
its own bias. It does not accept PX4 delta_angle/delta_velocity directly.
Dividing an integral by its dt gives an interval average (gyro includes coning),
NOT an instantaneous sample. Assigning both means to the common end timestamp or
arbitrarily moving them to a midpoint is a model decision requiring validation.
Moreover HIGHRES samples only the latest vehicle_imu on send:50Hz does NOT aggregate
all four~200Hz integrals into20ms. Lost integration coverage cannot be reconstructed
by multiplying transmitted rates by the20ms packet interval. The same hazard exists
when stock DDS copies latest queue1 data slower than publication.

## Transport recommendation and costs

**Current practical diagnostic:** existing HIGHRES_IMU50Hz with reduced telemetry
works as a transport experiment. It has been restored, not adopted as a permanent
configuration or production VIO input. Raising this to100Hz would require a separately
approved budget change and still would not repair its measurement semantics.

**Preferred real-input qualification candidate:** reuse PX4's existing uXRCE-DDS
mechanism to export a selected vehicle_imu at its full publication rate on a
dedicated sufficiently fast link, keeping MAVLink ODOMETRY/health available for the
later Stage C. This is a proposed next engineering direction, NOT an implemented
or accepted solution. Prefer existing messages/client/agent over a custom MAVLink
dialect or bespoke serial protocol. No current option is qualified yet.

Read-only `uxrce_dds_client status` says not running (command exists in this build).
Exact dds_topics.yaml exports sensor_combined, not vehicle_imu or sensor_selection.
dds_topics.h.em defaults unspecified rate_limit to10ms, so stock sensor_combined
subscription is limited to~100Hz even though it publishes~198Hz. Queue1/latest-copy
skips intermediate integrals; adding rate_limit:200/250 alone is not a proven
loss-free capture mechanism. Publication path uses best-effort output, not guaranteed
reliable delivery. Full vehicle_imu export requires build-time topic configuration,
matching px4_msgs, rate/queue/scheduling validation and therefore firmware approval.
MAVLink and XRCE cannot independently own the same UART at once; no spare link or
electrical high-baud margin is assumed available. Same-UART protocol switching would
also remove the present MAVLink path during the switch and is NOT performed.

Payload-only lower bounds: vehicle_imu logical fields60B*200=12000B/s, already over
1152008N1 before CDR/XRCE/serial framing. At400Hz24000B/s. In-memory64B is not wire
framing. SensorCombined fields48B*200=9600B/s before overhead/other topics;100Hz4800.
Full SI gyro+accel at observed666+802Hz and48B in-memory would be~70.5kB/s even
before framing (illustrative memory-record model, NOT wire-size measurement).
Complete raw FIFOs cost still more unless deliberately packed/batched; existing
ULog/dropout history rules out assuming logging/USB streaming is loss-free.

Implementation costs (relative, no invented schedule/performance):

- HIGHRES50 diagnostic: low, already implemented/tested; metadata/bias limitation remains.
- Existing DDS sensor_combined: medium (agent/schema/link/time-domain audit), no topic
  firmware edit needed in source configuration, but stock100Hz/latest-only and missing
  IDs do not meet current200–400+Hz/full-integral objective.
- DDS vehicle_imu: medium/high (approved topic/rate build, link, selected instance,
  drop diagnostics, calibration/time provenance, integral-aware estimator adapter).
  Retains more source information; preferred candidate if those changes are approved.
- Custom MAVLink/native raw batching: high (firmware/dialect/parser/versioning/testing).
  Not justified or implemented before evaluating existing DDS mechanism.

DDS time warning: generated serializer adds session time_offset to both timestamp
and timestamp_sample. Agent time sync is not camera exposure synchronization.
Store raw FC epoch (or a rigorously reversible offset), transformed domain, offset
history/convergence/uncertainty, host monotonic receipt, and boot/selection/calibration
segments. Do not feed shifted timestamps into the existing PX4_HRT contract unchanged.
Source mapping and uncertainties must precede any OpenVINS integration.

## Tests, files and provenance

`py -3 -m unittest discover -s .maixpy/imu-transport-20261010 -p test_*.py -v`:
5 pass. Parser/guard TDD RED missing module retained in tests-red.txt; metrics RED
in metrics-red.txt; GREEN in tests-green.txt. Synthetic tests, not hardware evidence.
Existing IMU suite: Windows10 pass/2Linux-PTY skipped; M3C11 pass/1host-interface
skipped. Linux tests use synthetic pseudo-terminals, not the physical UART.
Original tests and receiver/analyzer reused; production adapter/evaluator unchanged.
Raw replay exact on both captures. SHA256 matches M3C sha256sum and host:

- imu-controlled-50-01.jsonl:
  feb34f0990340df459e4e574e69a19d4b85bc40e588f0cdf8858e660d9cb546b
- imu-controlled-100-01.jsonl:
  1d739bee1fb03eaeec28aca96d027db5169bda862d6583991a327cb5c9dfe57b

`metrics.py` reports all intervals, including zero receipt deltas and startup errors;
`comparison.json` and analysis-50/100 preserve details. No raw capture edited/repaired.
Source fetched from official GitHub at the exact pinned commits; flattened UTF8
text snapshots are local inspection copies, not claimed byte-identical git blobs.
An attempted wrong Tools path returned404; corrected via exact tree listing.
One local text read mistakenly tried UTF16 on UTF8 and failed; no evidence changed.

Key source anchors (all PX4 at the hash above):

- [MAVLink budget](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_main.cpp#L1283)
- [HIGHRES_IMU](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/HIGHRES_IMU.hpp#L175)
- [VehicleIMU](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_imu/VehicleIMU.cpp#L552), [integrator](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/Integrator.hpp)
- [selected means](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/voted_sensors_update.cpp#L157)
- [DDS default interval/export](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em), [timestamp shift](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/Tools/msg/templates/ucdr/msg.h.em#L138)
- [OpenVINS propagation](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L179)

## Next boundary

Current permissioned comparison is finished; no more automatic captures/settings
escalation. Need a user decision before a dedicated faster-link/DDS topic-build
prototype or any baud/budget/firmware/wiring change. Startup corruption also needs
a controlled UART-opening investigation before claiming a reliable acquisition path.
Keep existing Camera–IMU contract, unknown clock map and processed-input rejection.
Do not calibrate/run real VIO/full Stage C until full measurement delivery, identity,
calibration epochs, timing semantics and clock relationship are qualified.
