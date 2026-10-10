# VIO-S0 offline follow-up after interface inventory — 2026-10-10

## Decision and status

**VIO-S0 PARTIAL.** Offline source analysis/diagnostic tools VERIFIED within the
tests below; high-rate acquisition, camera clock/exposure relationship and
OpenVINS measurement model remain UNKNOWN/unqualified. Deployment is BLOCKED
on physical-interface evidence and separately approved changes. Nothing in this
turn accessed M3C/PX4, opened serial/SSH, captured data, changed firmware/settings/
wiring, started calibration/VIO/fusion/Stage C or touched the Vault. No push.
EKF2_EV_CTRL0 and Disarmed are last verified by the preceding inventory, not
fresh observations from this offline turn. Stage A/B is retained, not rerun.

This is an additive review, not a replacement experimental result. Reuse:

- `../VIO_S0_INPUT_DESIGN.md`: full source/semantics/transport comparison.
- `../../interface-inventory-20261010/VALIDATION.md`: actual interfaces and USB limits.
- `../../imu-transport-20261010/VALIDATION.md`: measured50/100-request comparison.
- Existing Camera–IMU contracts, OpenVINS adapter/evaluator and original new_plan
  acceptance gates. No thresholds relaxed, no duplicate acquisition framework.

**Preferred implementation candidate remains:** dedicated UART + existing
uXRCE-DDS full-rate vehicle_imu, preserving the current115200 MAVLink2 ODOMETRY
UART. 460800 is a reasonable ~200 Hz prototype budget; prefer921600 headroom if
the physical link supports it and the original200–400+ Hz objective is pursued.
This is the preferred *transport qualification candidate*, not a qualified
OpenVINS frontend: integrated data needs its own observation-model acceptance.

**Fallback:** full-native-rate sensor_gyro + sensor_accel using DDS over USB CDC,
with explicit calibration, asynchronous timestamps and qualified filtering.
It is now a **higher-cost blocked alternative**, not plug-and-play: the inventory
found M3C's active USB is the SSH network gadget and CONFIG_USB_ACM is not enabled.
It needs a host-driver solution, management/VBUS/role plan and PX4 USB ownership
change, all separately approved. Neither source is accepted automatically.

## Source comparison retained and rechecked

PX4 exact d6f12ad1c4f70ad3230afd7d86e971421e02fef4; XRCE submodule
711aef423edd1820347b866d1e4164832df35d04; OpenVINS
69488123ed9362dd44b6f28e7f4680abbff1442b. Existing local snapshots reused;
only missing SensorSelection/TimesyncStatus messages fetched at the PX4 pin.

| Property | vehicle_imu | sensor_gyro / sensor_accel |
|---|---|---|
| Measurement | Delta angle rad with coning; delta velocity m/s; separate dt us | Angular rate rad/s and specific force m/s^2, but FIFO driver performs batch trapezoidal averaging |
| Processing/frame | Sensor filtering then integration, offset/thermal/scale and body-FRD rotation | Sensor filtering plus driver SI scaling/rotation, board FRD; full body calibration is not assumed |
| Bias | No HIGHRES per-packet EKF bias subtraction; gated autocal can still learn calibration from estimator bias | Not HIGHRES bias-subtracted; calibration still required before estimator input |
| Time | Gyro integration end, publication time, two dt; no independent accel endpoint | Separate gyro/accel sample/publication stamps, samples-per-batch; not row-aligned ADC instants |
| Identity/changes | Two hardware IDs, calibration counters, clip bits | Hardware ID, temperature/errors/clip counters; external calibration epoch needed |
| Queue / saved rates | Queue1; instance0~198 Hz snapshot | Queue8 each; saved666/802 Hz publication snapshots |
| OpenVINS | delta/dt diagnostic means not interchangeable with timestamped point input | Closer to existing wm/am interface, but still requires batch/filter/timing/calibration qualification |

Saved selected BMI088 is gyro6684690/accel6946834 instance0. Historical BMI270
ULog is not used as its timing/noise/continuity evidence. Source data rates above
are not sensor ODR guarantees or a new capture. No unknown accel timestamp is
invented and no gravity subtraction is added. Calibration/selection/boot changes
must segment data; 8-bit calibration counters alone cannot exclude a whole wrap.

## New scheduling finding: sync can block the subscriber

The pinned PX4 main loop calls `uxr_sync_session(&session, 10)` from the same loop
that copies/sends uORB. The pinned XRCE implementation sends a time request and
runs a synchronous listen loop until synchronization or the requested timeout.
This is a nominal10 ms wait path, not a guarantee of wall-clock execution<=10 ms
under OS scheduling. It is separate from the10 ms poll timeout, which can wake early.

PX4 only advances last_sync_session when that call succeeds AND the filter is
converged. A failed/unconverged attempt therefore need not wait another second
before retrying on a later loop iteration. For queue1 at~5 ms publications, a
subscriber pause may overwrite integrals before any serial packet exists.
The synthetic queue test illustrates two arrivals during a pause leaving only
the latest item. It does NOT measure this happening on the board; DDS was not run.

Consequences for the future prototype:

- A YAML rate_limit change alone is insufficient. Observe source generations,
  copy/prepare-output drops, sync waits, host sequence/coverage and queue age.
- Preserve raw HRT via an approved no-adjust mode or reversible per-record offset;
  merely recording low-rate timesync_status is not an exact per-record inverse.
- If sync waits cause drops, evaluate an approved nonblocking/bounded sync design
  or queue/drain changes using the existing client, not arbitrary retries/large
  queues. Memory, latency and drop counters must accompany any such patch.
- Disabling timestamp adjustment alone does not establish a camera clock map.
  TimesyncStatus.source_protocol must also identify the relevant time-sync source;
  do not mistake MAVLink sync observations for DDS offsets.

Other retained source constraints: stock DDS exports sensor_combined, not the
proposed full vehicle_imu/selection streams; unspecified or0 rate_limit =>10 ms;
orb_subscribe chooses instance0, not dynamically voted IMU; best-effort output
does not become loss-free when the ROS subscriber requests reliable QoS. The
stock full DDS topic list is NOT the minimal budget below.

Source anchors: [PX4 client main loop](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp#L684),
[pinned XRCE synchronous wait](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/core/session/session.c#L527).

## Metadata-inclusive bandwidth

Proposed *minimal export profile*, not stock configuration: vehicle_imu at200
or400 Hz, plus an allocation ceiling10 Hz each for sensor_selection and
timesync_status. Selection is event-driven, not actually published periodically
at10 Hz. Reserve permits metadata bursts; setup/control/other topics still need
measurement and explicit budgets. Keeping all stock exports invalidates this total.

CDR sizes: vehicle_imu60 B, SensorSelection16 B, TimesyncStatus44 B. TimesyncStatus
includes7 alignment bytes after source_protocol; no trailing memory padding is
counted. Source schemas are checked by the helper before calculation. [Selection
schema](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/SensorSelection.msg),
[time-sync schema](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/TimesyncStatus.msg).

Reuse previous exact framing model:12 B XRCE +7 B minimum serial envelope per
unfragmented topic flush; escape all but opening flag in conservative maximum.
Planning estimate assumes1% escaping and20% extra allocation, NOT measured traffic.

| Vehicle rate + metadata caps | Minimum B/s | Planning B/s | All-escaped data bound B/s | Bound /460800 capacity | Bound /921600 capacity |
|---|---:|---:|---:|---:|---:|
|200 Hz|16780|20334.72|33340|72.35%|36.18%|
|400 Hz|32580|39481.92|64740|140.49%|70.25%|

1152008N1 capacity11520 B/s fails even the200 Hz minimum. 460800 has headroom
for the specified200 Hz profile, not an unconditional loss-free claim. 921600
supports a more conservative400 Hz *wire* budget, but does not make the current
~198 Hz source publish400 Hz; sensor/integration rate changes are not authorized.

The fallback native SI source remains at least92484 B/s at the previously observed
666+802 Hz, before metadata/stuffing; it does not fit9216008N1. Naive dropping to
200 Hz destroys bandwidth/antialias information. Qualification must retain full
source data first or explicitly validate any deliberate filtering/decimation.

ODOMETRY10 Hz + HEARTBEAT1 Hz budget on the existing reverse MAVLink direction
remains2471 B/s conservative unsigned MAVLink2. Full-duplex budgets are separate.
No independent MAVLink/DDS processes may share the same UART byte stream.
If no spare physical interface exists, a custom MAVLink IMU message on the current
UART is a possible later user decision, not this implementation: it requires a
dialect, firmware and matched single-owner M3C endpoint, a higher approved baud/
software budget, and still has the same measurement/clock/model obligations.
Existing HIGHRES/SCALED messages are not a metadata-preserving substitute.

## New offline conversion/model checks

model_review.py imports the frozen prior diagnostic/budget helpers. It does not
replace an estimator, camera contract or transport. All examples are explicitly
SYNTHETIC_SCALAR_MODEL_NOT_CAPTURE_NOT_OPENVINS_RUN, single-axis commuting rotation,
zero filter/noise/calibration errors. Device IDs in fixtures are labels, not evidence
that a real BMI088 produced these values. Output always remains inadmissible.

For w(t)=100*t rad/s, two consecutive5 ms intervals produce integral means0.25
and0.75 rad/s. The actual second-interval delta is0.00375 rad. Treating these as
endpoint samples and applying the pinned OpenVINS adjacent-average operation over
5 ms gives0.0025 rad, difference-0.00125 rad. Unequal4/6 ms intervals produce
0.0042 versus0.0027 rad. A constant1 rad/s control agrees at0.005 rad.

These are counterexamples to universal equivalence, **not real OpenVINS runs or
predicted VIO accuracy**. Moving means to midpoints changes temporal support and
does not recover arbitrary within-interval motion or the absent accel endpoint.
The previous sine counterexample, unit/dt/order/gap/calibration checks and Camera–
IMU semantic rejection are reused, not weakened. [Pinned OpenVINS operation](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp#L394).

Current verification:7 new tests,9 existing diagnostic tests and76 existing
clock/calibration/sequence tests passed: **92 pass this turn**. No hardware/PTY
or native OpenVINS run. Prior100-pass result remains a separate earlier run.
New logs/results reside here; prior frozen manifests remain untouched.

Initial missing-module RED retained. First implemented run had one wrong manual
planning expectation20334.96; independent arithmetic gives
19147.2 +424.08 +763.44 =20334.72. Corrected the fixture, not the model or an
acceptance threshold. Failure is retained in tests-green.txt despite its original
filename; tests-final-7.txt is the actual final result. Windows PowerShell decorates
unittest stderr with NativeCommandError text; test summary and process exit0 show
the successful run. No failure is counted as a pass.

## Implementation plan and measurable gates

1. Resolve the physical spare UART, not by another identical software inventory.
2. After separate approvals, create a minimal existing-DDS export build/profile:
   IDs/selection, native time, explicit rate policy, counters and bounded queue/
   sync behavior. Match Agent/px4_msgs and preserve MAVLink/optical-flow ownership.
3. Transport qualification before VIO: proposed three60 s disarmed captures,
   EV_CTRL0, retaining startup bytes. Report actual publication/receive counts and
   rates, source and arrival p50/p95/p99/max intervals, byte counts, queue/parse/
   generation errors, CPU task/total load. Require zero observed unexplained
   discontinuities/duplicates/reversals/decode errors/clips across the accepted
   interval. Report startup anomalies separately but do not erase them to pass.
   Loss counters must distinguish sensor/uORB/transport; endpoint deltas alone are
   not exact missing-sample counters. ~198 Hz is not strict >=200 Hz PASS.
4. Source-model qualification: concurrent reference data from the SAME selected
   BMI088 and calibration epoch, preserved units/intervals/filtering/body transform;
   resolve accel timing and demonstrate an admissible OpenVINS observation model.
   If unsupported, use the higher-cost SI fallback rather than a silent retiming fix.
5. Clock/exposure/calibration gate then real estimator work: use existing mapping,
   bracketing, calibration admission and original evaluator thresholds. Reject
   unknown/extrapolated clocks, mixed epochs and unqualified semantics. No invented
   zero offset, interpolation across gaps or threshold relaxation. Later coexistence
   verification must keep ODOMETRY/heartbeat updating alongside full IMU delivery.
   Full Stage C remains a separate later gate, not triggered by transport success.

Relative costs: preferred transport medium but observation-model qualification
potentially high; SI/USB fallback high because of native throughput, host-driver/
management changes and calibration/asynchronous processing. No reliable production
solution is claimed without these gates. Remaining data needed cannot be generated
honestly by additional synthetic tests, so no further speculative framework is built.

## Exactly one next hardware verification step — approval required

Approve **one power-off carrier-pin verification session for both M3C and PX4**,
performed by you/the hardware designer: correlate actual accessible pads/connectors
with the carrier drawings and check continuity for M3C UART1 TX/RX/GND and one
candidate unused PX4 UART. Both boards unpowered, no new inter-board connection,
no computer connection required. Record photos/pad labels and documented I/O voltage
domains; power-off continuity alone cannot measure operating voltage or high-baud
signal integrity. This replaces repeating the already completed live inventory.
Do not execute until approved; it does not approve baud, firmware or wiring changes.
