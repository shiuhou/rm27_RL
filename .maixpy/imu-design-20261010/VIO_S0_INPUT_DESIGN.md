# VIO-S0: high-rate IMU input design — 2026-10-10

## Verdict and scope

**VIO-S0 PARTIAL; no high-rate OpenVINS input is qualified.** This continuation
is entirely offline: no SSH, serial open, physical capture, firmware build/flash,
parameter/baud/wiring change, camera operation, EKF fusion or Stage C. No new
dependencies, commit, push or Vault access. Stage A/B evidence is preserved.
Disarmed and EKF2_EV_CTRL=0 are the last recorded hardware state, not a new readback.

VERIFIED here means pinned-source inspection or a specified offline test, NOT
new hardware performance. Hardware availability, loss-free DDS delivery, camera
clock mapping and estimator suitability remain UNKNOWN/unqualified. Deployment
is BLOCKED on interface verification and separate approval for changes.

**Preferred qualification architecture:** existing uXRCE-DDS carrying full-rate
`vehicle_imu` with identity/calibration/time provenance on a **separate UART at
460800 or higher**, leaving the verified 115200 MAVLink UART for HEARTBEAT and
ODOMETRY. This is the smallest metadata-preserving ~200 Hz transport candidate,
not permission to feed integral means into OpenVINS. A source-model acceptance
gate is mandatory. 921600 offers more margin if the electrical link supports it.

**Fallback if the integral model cannot be qualified:** full native-rate
`sensor_gyro` + `sensor_accel` over existing XRCE serial transport on a verified
M3C USB-host -> PX4 CDC connection, with explicit calibration, asynchronous timing
and antialias processing. Keep MAVLink on the existing UART. This preserves more
temporal information but costs more bandwidth and preprocessing; USB host/port
availability is NOT yet verified. If neither interface exists, stop for a design
decision rather than taking over the optical-flow or MAVLink port.

The preferred route is a **conditional engineering recommendation**, not a claim
that it is already a practical/accurate OpenVINS frontend. If direct OpenVINS
integration is required without validating a new integral observation model,
use the fallback investigation; do not silently relabel the preferred source.

## Evidence and version boundary

- Firmware/source: PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, v1.17.0,
  reported MICOAIR_H743_V2; use `boards/micoair/h743-v2`, not a similarly named
  AIO board configuration based solely on the retail name.
- Existing real measurements: `../imu-transport-20261010/VALIDATION.md`,
  `control-session.jsonl`, `comparison.json`, and the two saved raw receives.
- Existing Stage A/B: `../mavlink2-20261010/VALIDATION.md`; no repeat.
- OpenVINS inspected revision: `69488123ed9362dd44b6f28e7f4680abbff1442b`.
- PX4's pinned Micro-XRCE-DDS-Client submodule:
  `711aef423edd1820347b866d1e4164832df35d04` from PX4's fork. New source inspection
  copies are in this directory; earlier PX4 snapshots are reused in the input
  and transport evidence directories. UTF-8 copies are not asserted to be Git blobs.
- Historical v1.15.2 BMI270 ULog (391 dropouts) is neither current selected BMI088
  evidence nor an input timing/noise calibration. It was not reprocessed here.

The existing reduced-telemetry experiment already isolated MAVLink budget scaling:
50 Hz requested -> 50.001 Hz; 100 requested -> 62.112 Hz, multiplier 0.621 at
unchanged 115200/5760 B/s software budget. Original 50*0.202 explains ~10.11 Hz.
Its startup invalid bytes/CRC anomaly remain unresolved. This report does not
turn that transport result into raw-sensor delivery or a loss-free PASS.

## Measurement semantics: source determines the model

| Property | sensor_gyro / sensor_accel | vehicle_imu | HIGHRES_IMU |
|---|---|---|---|
| Units | rad/s; m/s^2 | delta_angle rad; delta_velocity m/s; separate dt us | rad/s; m/s^2 |
| Processing | Sensor filtering, driver rotation/scale; FIFO path trapezoid-averages batch including prior boundary sample | Trapezoidal integration, gyro coning, offset/thermal/scale and board-to-body rotation | Selected integral/dt with matched estimator bias subtraction |
| Frame | FRD board | FRD body | Processed body-frame telemetry |
| Time | Separate gyro/accel timestamp_sample and publication timestamp; driver software anchor | timestamp_sample is last gyro anchor; separate gyro/accel integration lengths; accel end not independently encoded | Gyro sample anchor; integration lengths discarded |
| Identity/health | Device ID, samples, temperature, error_count, clip counters | Two device IDs, clip bits, two calibration counters | No hardware IDs, dt or calibration counters |
| Estimator bias | No HIGHRES per-message estimator bias subtraction; needs external sensor calibration | No HIGHRES per-message subtraction, but autocal can learn estimator bias into calibration | Explicit bias subtraction can conflict with a second estimator's bias model |
| OpenVINS | Closer to required rate/acceleration values, but not unfiltered ADC points; calibration and sample-time model still needed | NOT a direct ImuData input; division loses neither coning nor averaging semantics | Not accepted as unqualified raw VIO input |

Current saved selection: BMI088 gyro6684690 / accel6946834, instance0. The saved
uORB snapshot shows gyro666 Hz, accel802 Hz, vehicle_imu198 Hz (instance1 ~194 Hz).
These are short publication observations, not exact sensor ODR or full streams.
`IMU_INTEG_RATE=200` does not prove 200 delivered records/s.

Pinned BMI088 source configures gyro bandwidth using the `gyro_bw_532_Hz` mask
and accel normal filter/1600 Hz ODR; FIFO watermark grouping is driven by the
driver's requested publication rate. The mask is cleared in the gyro register
configuration: do not mistake its enum bit-mask value for a written bandwidth
code. These source settings do not measure the active hardware frequency response
or ADC delay. Accel FIFO sensor-time frames are skipped, not an established
Bosch-clock-to-HRT mapping. Separate gyro and accel batches need not align.

VehicleIMU's saved instance0 gyro dt is ~4459–6058 us versus accel ~4992–4994 us.
It does not supply a separate accel endpoint. The sensor_combined relative accel
anchor is constructed from vehicle_imu's gyro anchor in VotedSensorsUpdate;
it is not independent evidence of ADC simultaneity. Do not subtract gravity or
treat delta_velocity as world-frame motion.

SENS_IMU_AUTOCAL was read as1. VehicleIMU contains gated estimator-bias learning
and disarmed calibration saving. Preserve calibration counts and segment on any
change, including 255 -> 0 wrap; equal 8-bit counters cannot exclude a full wrap.
Device IDs alone do not identify a calibration epoch. No autocal setting changed.

OpenVINS takes one timestamp plus wm/am, interpolates/averages adjacent values,
integrates using timestamp differences, and estimates its own bias. It has no
standard field for PX4 delta integrals, unequal dt or separate interval endpoints.
`delta/dt` is a diagnostic interval mean, NOT a recovered instantaneous sample.
Assigning it to interval end or midpoint is a model assumption, not a clock fix.
An entire-cycle sinusoid and zero signal can have equal integrals but different
instantaneous values (tested). Missing integrals cannot be reconstructed by
multiplying a retained 5 ms average by a 20 ms packet interval.

## Existing DDS: available mechanism, not a drop-in configuration

Pinned client is compiled in the target; the saved status says not running.
Default dds_topics.yaml exports sensor_combined but not vehicle_imu, sensor_gyro,
sensor_accel or sensor_selection. Agent/px4_msgs schema must match this build.
Exporting the proposed topics requires a reviewed firmware build; no firmware
patch/build has been made in this task.

Important generated-code behavior:

1. Unspecified **or zero** rate_limit becomes a 10 ms subscription interval.
   Stock sensor_combined therefore cannot preserve all ~198 Hz updates. Positive
   200 yields5 ms,250 yields4 ms; even5 ms may skip a faster local interval.
   Zero does NOT mean unlimited in this template.
2. orb_subscribe selects instance0, not whatever the sensor voter currently
   selects. Instance0 matches saved BMI088 selection only in that snapshot.
   Minimum prototype: pin its IDs, export selection metadata and stop/segment
   if selection changes. Automatic instance switching is additional work.
3. vehicle_imu queue1 is latest-only. SI queues8 provide about10–12 ms at the
   observed rates, not a delivery guarantee. One orb_copy occurs per poll event.
   Scheduling/queue-drain behavior must be instrumented, not fixed by assertion.
4. Data writes use a **best-effort XRCE stream** and flush each topic immediately.
   Reliable setup streams or ROS reliable QoS cannot recover an overwritten uORB
   sample or a lost best-effort serial data frame. Use compatible subscriber QoS,
   explicit queue/age limits, and observable generation/write/receive counters.
5. The main poll timeout is at most10 ms, but it wakes on updates. It is not by
   itself a fixed100 Hz main-loop limit. The explicit per-topic interval is the
   default limiting mechanism. Output-preparation failures currently lack useful
   error logging; qualification needs counters before claiming exact completeness.

### Timestamp conversion and camera interface

The serializer adds session time_offset/1000 to BOTH timestamp and timestamp_sample.
Agent time synchronization does not prove camera exposure synchronization.
Preferred prototype requirement: retain native FC HRT timestamps (using a separately
approved no-adjust mode, or an exactly recorded per-record reversible offset), plus
boot epoch and host monotonic receipt. Do not label adjusted DDS timestamps PX4_HRT.
The 10 Hz timesync_status history is not guaranteed to identify the exact offset
applied to every high-rate record during convergence. Keep convergence, drift,
uncertainty and time-domain labels; reject unknown maps rather than invent zero.

Reuse `../offline-vio-20261009/.../experiments/vio_dataset.py` map_timestamp,
imu_windows, bracket_mapped_streams and validate_calibration_interface. Preserve
integer epochs; subtract anchors before scale conversion; never extrapolate beyond
the evidenced clock interval. Camera timestamp must have an evidenced exposure
semantic, not arrival time. Calibration needs explicit T_imu_camera, source-frame
rotation, intrinsics, timing and noise artifacts tied to device/calibration epochs.
Existing OpenVINS -> LocalizationEstimate adapter/evaluator are unchanged.

Our diagnostic means remain `openvins_admissible=false`, mapped time UNKNOWN,
accel endpoint null. A test proves the existing semantic check rejects them even
when a fixture supplies a syntactically verified clock map. Synthetic map/pose
fixtures do not create a real synchronized dataset. No calibration starts here.

## Wire budget, not sizeof(struct)

Pinned ucdr scalar alignment produces vehicle_imu60 B, sensor_combined48 B and
sensor_gyro/accel44 B each (not48 B memory allocation). XRCE session0x81 has a
4 B header +4 B subheader +4 B WRITE_DATA request/object fields. Serial framing
adds flag1 + addresses2 + length2 + CRC2 =7 B minimum. Each topic is flushed;
one unfragmented vehicle_imu frame is therefore **79 B minimum**.
Byte stuffing escapes0x7d/0x7e; conservative maximum is1+2*(79-1)=157 B.
No extra DDS/UDP/IP header is charged to this serial hop: the Agent handles DDS
on the host. Setup/control and other topics still consume link capacity.

| 200 Hz vehicle_imu | 115200 | 230400 | 460800 | 921600 |
|---|---:|---:|---:|---:|
| 8N1 capacity B/s |11520|23040|46080|92160|
| Minimum15800 B/s utilization |137.2%|68.6%|34.3%|17.1%|
| Planning19147 B/s utilization |166.2%|83.1%|41.6%|20.8%|
| All-escaped bound31400 B/s utilization |272.6%|136.3%|68.1%|34.1%|

Planning row assumes1% stuffing and20% reserve: **engineering assumptions, not
measured averages**. Worst row bounds this data frame only, not setup/retries or
arbitrary other DDS traffic. 460800 is a defensible starting bandwidth budget,
not proof the wiring/CPU/driver sustains it. At400 Hz the data costs double;
460800 cannot cover the all-escaped bound. Original200–400+ Hz goal is not relaxed.

Other useful bounds:

- sensor_combined200 Hz: (48+19)*200 =13400 B/s minimum, already too much115200.
- Two SI streams at200 Hz each: (44+19)*400 =25200 B/s minimum. This is a design
  rate, NOT full retention of the observed BMI088 publications.
- Full observed SI666+802 Hz:63*1468 =92484 B/s minimum, already above9216008N1
  before other traffic/stuffing. USB is the fallback candidate, not a faster-baud
  assertion. USB CDC ignores the simple UART baud/10 ceiling; USB packet overhead,
  host scheduling and actual sustainable rate remain hardware-dependent.
- Signed/unsigned MAVLink and reverse direction must be budgeted separately.
  Conservative unsigned MAVLink2 ODOMETRY245 B*10 Hz + HEARTBEAT21 B/s =2471 B/s
  M3C -> PX4; signing adds13 B/frame if enabled. UART is full-duplex, so do not
  add this to PX4 -> M3C bytes as if half-duplex. Existing opposite-direction
  telemetry and its software budget remain separate constraints.

The pinned framing implementation uses a CRC table labelled polynomial0x8005;
generic eProsima transport documentation describes a different CRC detail. Use
the pinned library, not a hand-coded framing/CRC implementation. No custom protocol
has been written. [Official transport overview](https://micro-xrce-dds.docs.eprosima.com/en/stable/transport.html)
supports the framing/escaping concept; exact fork source takes precedence.

## Available interfaces: do not turn a pin list into a wiring claim

Saved evidence only; no live inventory this turn:

- Known working: M3C UART2 /dev/ttyS2 -> PX4 /dev/ttyS2,115200,MAVLink2.
  Preserve it. DDS and MAVLink cannot independently own that same byte stream.
  Full duplex is not a multiplexer. Do not run two readers/writers on it.
- PX4 target maps GPS1 ttyS2, GPS2 ttyS1, TEL1 ttyS0, TEL2 ttyS3, TEL3 ttyS4,
  TEL4 ttyS7, URT6 ttyS6, RC ttyS5. Existing ttyS3 carries optical-flow/ToF;
  leave it and RC untouched. Remaining mappings do NOT prove accessible free pads.
- Cached M3C_378C V1.0 schematic pages6/12 (`tmp/pdfs/m3c-io-06.png` and
  `m3c-pinout-12.png`) show UART2 RX GPIO1_A1/J1pin2,TX GPIO1_A0/J1pin4,
  in a3.3 V domain. These are module-connector pins, not carrier pad directions.
  Other UART aliases/mux options are not automatically spare ports; some have
  different voltage domains. Do not substitute Maix pin aliases by guesswork.
- M3C J2 USB_DP/DN pins20/22 and PX4 USB-device support make a USB-host/CDC
  candidate plausible. M3C host mode, VBUS power role, accessible carrier connector,
  cdc_acm support and PX4 CDC ownership are UNKNOWN. PC COM19 evidence proves PX4
  USB-device console worked, not that M3C can host it concurrently with the camera.
- M3C Ethernet signals and client UDP compile support do not establish a PX4
  Ethernet connector/PHY/network path. UDP/Ethernet is not the default proposal.

## Implementation plan and acceptance gates — NOT executed

1. Read-only interface feasibility inventory after approval. Prefer a confirmed
   spare3.3 V UART pair; evaluate USB host as fallback. Do not assign pins from
   software names alone. If unavailable, obtain a user interface decision.
2. Separately approve a minimal DDS topic/export build and chosen link settings.
   Keep existing MAVLink ODOMETRY link. Pin px4_msgs/Agent compatibility; export
   vehicle_imu + selection metadata, preserve FC time, avoid the default10 ms
   subscription, instrument producer generations/subscriber drops/output errors.
   Rate setting alone is insufficient; queue/copy behavior needs qualification.
   Do not implement custom MAVLink or a bespoke binary protocol at this stage.
3. Proposed transport validation: three bounded60 s disarmed trials with current
   selected IDs and EV_CTRL0, recording source + receipt times, counters, byte
   rate, CPU and all startup bytes. Require zero observed unexplained sequence/
   coverage gaps, duplicates, reversals, clips and decode errors; account separately
   for sensor publication, uORB loss and transport loss. Queue1 timestamp heuristics
   alone cannot establish exact sample completeness. Report actual rate;198 Hz is
   not a strict >=200 Hz PASS. Record p50/p95/p99/max interarrival and age, and
   CPU per relevant task plus total. No new arbitrary latency/CPU threshold is
   substituted for the project's original criteria. Capacity must include all
   advertised DDS topics, not just IMU. Resolve startup corruption rather than
   deleting a warm-up interval and calling the complete capture clean.
4. Before OpenVINS admission, compare preserved integral records against concurrent
   SI references for the SAME selected BMI088/calibration epoch. Unequal accel/gyro
   endpoints must be resolved, not fabricated. Establish sensor filter/timing and
   noise model and validate any integral-to-rate interpretation against the existing
   evaluator/original acceptance criteria. If not supported, use full-rate SI/USB
   fallback; its calibration/asynchronous pairing/antialias filter and delay model
   must be explicit. Naive drop-to200 Hz is not an acceptable antialias solution.
5. Only after delivery and source semantics pass, qualify camera clock/exposure and
   calibration interfaces. Eventually demonstrate concurrent10 Hz ODOMETRY output
   and full IMU input without sharing devices or overruns. This is future transport
   coexistence testing, not permission for real VIO, fusion or Stage C now.

Relative cost: preferred DDS/integral prototype medium for transport, potentially
high for estimator-model qualification; full SI fallback higher for USB/interface,
calibration and asynchronous processing. Existing sensor_combined is lower effort
for a diagnostic but retains integral semantics, lacks IDs and defaults100 Hz.
HIGHRES/SCALED telemetry loses needed provenance; neither is the fallback.
Custom firmware/protocol batching would add dialect/CRC/versioning/maintenance and
still not solve source timing; no evidence currently justifies it.

## Completed offline work and limitations

`offline.py` reuses previous interval analysis and the existing dataset semantic
checks. It validates integral units, wire ranges, dt, source/publication order,
identity, calibration transitions and clipping; produces diagnostic means and
gyro coverage residuals only. Accel coverage remains UNKNOWN. No interpolation,
retiming, synthetic repair, OpenVINS export or new acquisition framework.

`offline-results.json` audits five saved console records per instance, preserving
the label REAL_SAVED_CONSOLE_ROUNDED_5_DECIMAL_VECTORS_NOT_FULL_STREAM. Both show
coverage gaps because the listener is sparsely polled; this is NOT evidence of
DDS transport loss. Means inherit console rounding and cannot validate noise or
the integral model. Bandwidth figures are analytical/synthetic, not UART tests.

Verification:9 new unittest cases pass (`tests-final.txt`),10 existing input tests
pass/2 Linux PTY tests skipped on Windows (`regression-input.txt`),5 transport
tests pass,76 existing timestamp/calibration/sequence pytest cases pass. Total
100 pass,2 skipped. First contract invocation from repository root imported the
different production package and failed collection; rerun from the saved offline
snapshot resolved the path without editing production code. Failure retained in
regression-contract.txt; retry in regression-contract-retry.txt. TDD initial
missing-module and numeric-bounds failures are retained. No Linux/M3C rerun.

### Exactly one next hardware verification step — approval required

Approve **one bounded10-minute read-only interface inventory on M3C and PX4,
using their existing management connections**: verify exact firmware/Disarmed/
EV_CTRL0, existing serial owners, device-tree/pinmux readbacks and USB role/driver
availability, correlating with the carrier schematic. Do not open data UARTs,
run pinmux-changing imports, start DDS/capture, change settings or move wires.
Purpose: determine whether a dedicated UART or USB-host/CDC path is actually
available while preserving current MAVLink and optical flow. Software evidence
alone will not certify pad voltage or electrical bandwidth. This step is proposed,
NOT approved or executed by this report.

## Exact source anchors

- [VehicleIMU integration/calibration](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_imu/VehicleIMU.cpp)
  and [message](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/VehicleImu.msg).
- [DDS topic list](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml),
  [subscription/stream template](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em),
  [time serializer](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/Tools/msg/templates/ucdr/msg.h.em).
- [Target interfaces](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/boards/micoair/h743-v2/default.px4board).
- [Pinned XRCE WRITE_DATA](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/core/session/write_access.c),
  [serial framing](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/profile/transport/stream_framing/stream_framing_protocol.c).
- [OpenVINS propagation](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp).
