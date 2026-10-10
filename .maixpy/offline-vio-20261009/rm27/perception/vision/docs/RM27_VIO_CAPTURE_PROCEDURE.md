# Real RM27 VIO capture/calibration procedure — offline preparation

Status: **PREPARED; hardware qualification BLOCKED on new evidence**.
No device operation in this task. This document is a later procedure, not a list
of physical actions requested now. VIO-P/S0 remain PARTIAL. Existing pose schema,
OpenVINS adapter, evaluator and acceptance thresholds are unchanged.

## Admission and retained artifacts

Keep one immutable session manifest referencing camera canonical view, original
images/video and frame map, independent gyro/accel exports, original ULog, source
hashes, exact firmware/runtime/config, calibration artifacts and clock maps.
Reuse CanonicalDataset/calibration admission and LocalizationEstimate v2.
Do not make optional IMU mandatory for legacy visual-only data. A bundle is not
admitted just because it is parseable or its status strings say VERIFIED.

Camera: physical module/lens/focus/mode, actual image size, sensor crop/resize,
exposure/gain, frame counter semantics, raw VIN PTS with integer/unit/domain/event,
software receipt separately, encoded PTS and image hashes. Existing receipt and
encoded timing do not supply exposure midpoint. Rolling/global shutter must be
evidenced, not guessed from frame rate. Existing full1801344x760 calibration has
unknown lens/revision/crop/timing provenance and remains geometrical evidence only.

IMU: bind board, driver/firmware, separate accel+gyro devices and instances,
driver-rotated axes, ranges/filter configuration, scale, samples, dt, original
sample/publication timestamps, counts, transport/recording errors, sequence
semantics and temperature. New firmware must revalidate batch/anchor behavior.
Keep separate streams until a reviewed pairing/resampling policy exists; never
zip by row, discard unpaired samples silently or promote export indices to hardware
counters. Counts*scale follows inspected source; it is not EKF bias compensation.

Epochs: every reset/device swap starts a new segment. Integer subtraction BEFORE
conversion to seconds; retain raw units and decimals where dt is fractional.
An affine map is target_ns = target_anchor_ns +
(source_raw-source_anchor)*scale_ns_per_tick. It must name source/target clock
and epoch, direction, validity interval, evidence hash and bounded uncertainty.
Do not extrapolate, double-apply middleware time conversion, or convert UNKNOWN
into zero offset. Mapping clocks does not convert receipt to exposure events.

Use experiments.vio_dataset.map_timestamp and bracket_mapped_streams for explicit
maps and separate gyro/accel bracketing, common-clock/epoch/event checks and an
explicit uncertainty budget. Lower-level imu_windows expects qualified inputs.
No interpolation or sample synthesis. These helpers are offline structural
checks, NOT hardware admission. Artifact bytes and calibration identity are
validated by the existing calibration gate, not trusted from hashes' syntax.

## Calibration artifacts and frozen decisions

1. Intrinsics/model: preserve original five-parameter Brown model. OpenVINS
   CamRadtan has k1,k2,p1,p2 only. Never truncate k3. The offline fixed-k3=0 refit
   candidate uses original27 fit/7 validation views without outlier removal;
   fit RMS0.171px, validation RMS0.202px, all original geometric checks pass.
   This is a same-session model diagnostic, not new blind validation or adoption.
   Alternate pre-undistortion must retain map/K/new pixel geometry/crop/masks and
   interpolation provenance; it is not implemented or silently selected here.
2. Rigid extrinsics: obtain a rigid camera/FC mounting first in a later session.
   T_imu_camera maps camera points INTO the identified driver IMU frame, translation
   metres, proper rotation (no reflection). Distinguish board/body/IMU rotation;
   don't apply PX4 driver rotation twice. CAD is an initial estimate, not truth.
3. Temporal calibration: independently qualify camera timestamp event, clock map
   and drift/uncertainty, exposure duration/row timing; only then estimate residual
   camera-to-IMU delay. Keep convention t_imu = t_camera + offset explicit. A fitted
   constant offset cannot repair missing frames, unknown clock drift or rolling
   shutter. Current stock OpenVINS path does not thereby gain row-timing support.
4. Excitation recording: later record stationary initialization plus broad camera
   scene/target and safe multi-axis rotation/translation, fixed focus/exposure and
   unchanged rigid mount. Retain all attempts and predeclare held-out segments;
   verify completeness before running any solver. Use no truth in initialization.
5. Noise: later warm up and collect sufficiently long stationary, unsaturated,
   gap-free raw data with temperature/vibration context. Existing Allan helper
   only computes descriptive statistics; old18s lossy log cannot establish noise
   densities/random walks. Fit ranges/duration must match identifiable timescales.
   Units: gyro density rad/s/sqrt(Hz), gyro bias RW rad/s^2/sqrt(Hz), accel density
   m/s^2/sqrt(Hz), accel bias RW m/s^3/sqrt(Hz). Keep continuous/discrete conventions
   separate; no copied datasheet or synthetic values as measured calibration.
6. Validation: retain spatial/temporal residuals, coverage/observability, repeat
   session consistency and held-out metrics. Reject weak excitation or unmodeled
   timing. Run the existing native/adapter same-run parity and SE3 evaluator on
   admitted data; unknown latency/tracking/reset fields stay null, not invented.

No actual synchronized RM27 Camera+IMU recording exists in the inspected evidence.
Synthetic fixtures validate code only. The old M3C camera recordings and PX4 ULog
are different sessions/clocks and must never be combined as synchronized data.

## Future PX4 → M3C transport decision

User-reported custom carrier VGTR ↔ FC TX3/RX3 identifies physical labels only.
Pin direction, voltage/power, M3C UART mapping and PX4 logical serial mapping are
unverified. This document neither assumes a `/dev/tty*` nor asks for wiring now.

| Candidate | What can be reused | Remaining evidence / cost |
|---|---|---|
| SD ULog + host export | Exact raw topics/parser/hash tools already exercised | Loss-free recording unresolved; no live M3C or camera sync |
| USB CDC/MAVLink ULog | PX4 logging backend exists | Real M3C USB host/role, sustained bandwidth, sequence/dropout/latency qualification |
| Compact/batched UART | FC sample anchors, dt, counts, identity can be retained | New message/sender/receiver design and firmware decision; framing/CRC/sequence/backpressure tests |
| uXRCE-DDS custom raw topics | Existing PX4 client architecture | Default exports do not provide this raw pair; topic/config/firmware and time-conversion audit required |

115200 baud8N1 provides at most11520 bytes/s BEFORE protocol overhead. Six
float32 axes alone at1580Hz need37920B/s; this link cannot carry that format.
Illustrative32-byte/sample payload + unsigned MAVLink2 overhead12 bytes at1600Hz
needs704000bit/s:76.39% of921600,152.78% of460800. Signing adds13bytes/packet,
making912000bit/s (98.96% of921600) before other traffic. These are format budgets,
not implemented messages or measured throughput. Multiple sensor packets, sequence,
CRC, loss flags and batching change the accounting; test transport_budget.

Old ULog four-topic modeled869kB/s implies~8.69Mbit/s with8N1 even before transport
framing; fixed32-slot single-sample FIFO messages are inefficient to forward whole.
Do not choose a rate from nominal UART capability without measured electrical/link
margin. Stock HIGHRES_IMU/SCALED_IMU are processed/interval outputs in inspected PX4
source, not raw replacements. Baud-rate changes alone do not solve timestamp loss.

Every candidate must retain FC sample timestamps, sensor identity, per-stream or
batch sequence semantics, batch anchor/dt/count, errors and resets. Host receipt
time is for latency diagnostics, not replacement sample time. Cross-device sync
needs offset+drift estimates with request/response timestamps, RTT/asymmetry bounds,
outlier and reset handling; separate camera exposure/row-event qualification.
TIMESYNC/boot_time_utc_us alone cannot establish camera exposure synchronization.

Decision boundary: no production transport or calibration candidate is selected
by this preparation. The next controlled PX4-only capture is detailed in
PX4_NEXT_CAPTURE_PLAN.md; M3C and live transport are not its prerequisites.
