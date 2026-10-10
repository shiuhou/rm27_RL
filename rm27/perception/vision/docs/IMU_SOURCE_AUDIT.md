# IMU source audit — updated 2026-10-08

## CURRENT: authorized disarmed offline capture completed (2026-10-08)

See `PX4_IMU_OFFLINE_VALIDATION.md` for exact config, raw-file hash, commands,
failures and restoration. User confirmed props removed. One short bench ULog
was captured from BMI270 gyro/accel FIFO **instance1/device3604506** and exported
twice byte-identically. This supersedes older NO_CAPTURE/unavailable-export
statements below. It does not establish a complete loss-free dataset path.

Actual file18.121956s,6,896,444 bytes, SHA256
d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d.
Gyro FIFO12648 records/effective706.681Hz; accel11933/effective666.823Hz.
Local cadence633us (~1579.76Hz in VehicleIMU status), but **391 ULog dropouts**,
max110ms dropout and111.409ms sample gap. No duplicate/backwards times,
nonfinite values or clipping observed. All FIFO samples=1; nominal dt625us must
not replace observed timestamps.11930 paired timestamps,718gyro-only/3accel-only.
FIFO scaling and PX4 SI averaging cross-check agrees on retained adjacent pairs.
Raw SI topics are not identical to FIFO measurements even with batch size1.

Temporary9-topic list and logger2500Hz polling were used; no params changed.
Original file absence, `logger start -b 64 -t`, SDLOG values and idle state restored
and re-read. Original SD log retained. Props USER-CONFIRMED; observed heartbeats
and ULog arming state stayed disarmed. No firmware/control/camera/M3C changes.
VIO-S0 remains PARTIAL: raw source/export verified, recording completeness FAILED
for this config, precise acquisition timing and Camera/IMU sync remain unqualified.
Next is offline logging bottleneck diagnosis; no physical action needed now.
VIO-P remains PARTIAL with unchanged reproducibility gate.

## CURRENT addendum: raw export, timestamps and transport (2026-10-08)

This supersedes the earlier request to photograph assembled hardware: the user
confirms camera and FC are separate on the desk. No mounting or wiring needed
for this audit. VIO-P remains PARTIAL; no acceptance policy was changed.

### Fresh read-only evidence

Windows helper `px4_console_readonly.py` ran bounded SERIAL_CONTROL queries at
COM19/57600, exited 0 and released the shell/port. Commands and results:

| Command | Observed result |
|---|---|
| `logger status` | running mode all, 142 subscriptions, **Not logging** |
| `param show SDLOG_PROFILE` | 1 |
| `param show SDLOG_MODE` | 0 |
| `param show SDLOG_BOOT_BAT` | 0 |
| `ls /fs/microsd/etc/logging` | stat failed: Not a directory |
| `cat /fs/microsd/etc/logging/logger_topics.txt` | open failed: Not a directory |
| `ls /fs` and `ls /fs/microsd` | microsd directory readable, including log/ |

The missing custom path does NOT mean SD absent. No existing logs were parsed,
no recording started, no parameter/stream/file/firmware changes on FC. The shell
sysinit warning recurred, but listed commands worked. Readable SD is not a
write-throughput or logger-dropout qualification.

### Export decision: supported, not yet capture-qualified

Recommend the first raw-export qualification use **FC ULog on SD, then download
the completed file over USB/MAVLink or read the SD on a host**. This avoids making
live link throughput a prerequisite for raw-data inspection. It is not yet a
synchronized camera+IMU dataset path. Source supports MAVLink log download;
actual download and raw recording have NOT been tested here.

At reported hash `4817c0618a1286846116e90c6eb8919efaa013cf`:
- Logger default profile limits sensor_gyro and sensor_accel to **1000 ms**
  intervals, not 1000 Hz. Sensor-comparison profile uses100 ms, not100 Hz.
- Raw gyro/accel FIFO profile bits8/9 call add_topic with default **instance0**.
  Currently selected BMI270 is instance1; blindly enabling those bits would
  target BMI088, not the selected BMI270. BMI270 is the simplest identified
  first candidate, not a demonstrated lower-noise/better-VIO sensor.
- Custom `/fs/microsd/etc/logging/logger_topics.txt` supports
  `topic interval_ms instance`; 0 requests every update (not guaranteed lossless).
  A nonempty custom list **replaces** profile topic selection. Future changes
  require explicit approval and preservation of health/default evidence topics.
- Proposed entries, NOT installed: `sensor_gyro_fifo 0 1` and
  `sensor_accel_fifo 0 1`; sensor_gyro/accel instance1 at interval0 can provide
  SI/health cross-checks, but can be batch-averaged and are not interchangeable
  with individual FIFO samples. Confirm device_id3604506 at each boot; instances
  must not be treated as stable hardware identity.
- FIFO messages contain fixed32-entry arrays. At high publication rates their
  logging/streaming overhead can be large even with small sample counts. SD
  dropouts, uORB queue overrun, valid sample counts, clips, time gaps and actual
  sustained rates must be measured before qualifying any export.

### Time and units: what can and cannot be asserted

`timestamp_sample` is FC HRT boot-domain microseconds, distinct from publication
`timestamp`. BMI270's driver uses DRDY HRT when fresh, otherwise scheduled-read
time, with FIFO correction. It is NOT an independently ADC-latched timestamp.
The driver parses FIFO entries in order. The gyro consumer filters entries in
array order and publishes the last entry at timestamp_sample, supporting an
end-of-batch anchor. Candidate reconstruction is
`t_i = timestamp_sample - (N-1-i)*dt`, with dt in microseconds; this is a
source-supported convention, NOT dynamically validated per-sample timing.
It must pass cross-batch ordering/continuity and gyro/accel association checks
on a future capture before use. Nominal BMI270 dt625us comes from source RATE1600;
do not overwrite measured timing/clock drift to enforce that nominal rate.

FIFO int16 arrays require their supplied scale: gyro rad/s, accel m/s^2.
Driver/library already rotate samples; do not apply the same rotation twice.
These are pre-estimator sensor measurements, not untouched ADC output: source
configures BMI270 normal sensor filtering/high-performance gyro mode, and sensor_*
can additionally represent trapezoidal FIFO averages. Exact on-silicon bandwidth
and acquisition-delay uncertainty are not measured. Downstream80/30Hz cutoff
parameters do not describe FIFO bandwidth.

### Future live transport and synchronization

| Path | Evidence / disposition |
|---|---|
| Stock HIGHRES_IMU | vehicle_imu delta/dt minus matching estimator bias when available; time_usec=timestamp_sample. Processed, not raw. |
| Stock SCALED_IMU family | integrated vehicle_imu delta/dt, mG/mrad/s quantization; SCALED_IMU uses publication timestamp/1000 in time_boot_ms. Not a raw precise-time substitute. |
| MAVLink ULog streaming | source supports LOGGING_DATA/ACKED and rate limiting; could retain selected uORB records without estimator changes. Deployed availability, sustained rate, buffering and losses untested. |
| USB CDC to M3C | first candidate link for later bench qualification, avoiding UART pin/voltage work. M3C USB-host support, power arrangement and enumeration still need verification; NOT connected now. |
| UART MAVLink | possible later; current57600/115200 ports and tx budgets not qualified for full raw data. No wiring/pin assignment proposed. |
| uXRCE-DDS/custom raw bridge | client stopped; stock DDS list lacks raw pair. Extra configuration/build/interface work requires separate approval, not needed for initial offline audit. |

USB COM19's57600 setting is not USB physical throughput. The FC's100000B/s USB
MAVLink budget is configuration, not achieved throughput. Do not claim200Hz or
1600Hz M3C delivery from internal uORB rates or ask HIGHRES_IMU for a faster rate
and call it raw. Stock ULog streaming should be qualified before proposing a
custom bridge; SD logging alone cannot establish a live OpenVINS input path.

MAVLink TIMESYNC source replies with FC HRT nanoseconds (microsecond-origin
precision). Future M3C receiver can fit offset/drift using request/reply times,
reject high RTT samples and retain uncertainty and reboot epochs. No mapping
has been measured. OS04A10 pts_raw remains unqualified; camera monotonic_us is
post-VIN software receipt, not exposure. Clock synchronization alone does not
resolve camera exposure midpoint, rolling-shutter time or IMU filter delay.

Next software prerequisite: approve a reversible logger-topic/configuration and
bounded disarmed capture test; none executed. No physical action is required
now. The single later physical step, when synchronized capture is authorized,
is rigidly mounting camera and FC together so their relative transform is fixed;
do not assemble or connect them yet. Wiring follows its own verified interface plan.

### Source references (all pinned to the reported firmware hash)

Base: https://github.com/PX4/PX4-Autopilot/tree/4817c0618a1286846116e90c6eb8919efaa013cf

Inspected paths: src/modules/logger/{params.c,logged_topics.h,logged_topics.cpp};
src/drivers/imu/bosch/bmi270/{BMI270.hpp,BMI270.cpp};
src/lib/drivers/gyroscope/PX4Gyroscope.cpp;
src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp;
src/modules/mavlink/streams/{HIGHRES_IMU.hpp,SCALED_IMU.hpp};
src/modules/mavlink/{mavlink_timesync.cpp,mavlink_log_handler.cpp,mavlink_ulog.cpp}.
Source matching a reported hash is CODE-SUPPORTED, not proof of a reproducible
vendor binary. Live logger/parameter/filesystem results above are VERIFIED;
future transfer performance and sample-time accuracy remain UNKNOWN.

## CURRENT: COM19 live identity and status audit (2026-10-08)

VIO-P stays PARTIAL with its unchanged reproducibility gate. VIO-S0 remains
PARTIAL for the paired transport/timing path, but FC identity and running IMU
drivers are now VERIFIED. This section supersedes earlier missing-FC and
connect-USB requests below. The user confirms M3C and FC are NOT connected.

### Official manual and upstream check (read before device queries)

Manual: https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual
(page shows updated 2026-09-15; retrieved 2026-10-08). Reader proxy failed;
direct HTTPS retrieval and full text inspection succeeded.
Manual specifies STM32H743VIH6, 480 MHz, 2 MB flash; BMI088 + BMI270; SPL06;
microSD; seven UARTs and USB-C. BMI088 is on SPI2, BMI270 on SPI3. These are
manufacturer specifications, now corroborated by running driver/bus reports.
The named AIO board uses **micoair_h743-v2**, NOT similarly named h743-aio.
The manual states official PX4 support from 1.16.0. Upstream v1.16.0 resolves to
commit `6ea3539157ca358c70a515878b77077af7d4611d`; its
boards/micoair/h743-v2/{default.px4board,init/rc.board_sensors} confirms the
target, enabled BMI drivers and buses. No build/flash was performed.

### Direct Windows readback, not a Linux COM-port assumption

COM19 enumerated as USB VID:PID 1B8C:0036. Opened successfully on Windows at
the user-specified 57600; no QGC process was stopped. Bounded MAVLink identity
request and read-only SERIAL_CONTROL shell queries were used, with ports closed
and shell ownership released afterwards. No stream-rate requests, parameter
writes, reboot, calibration, logger start or raw measurement capture occurred.
Diagnostic status snapshots are not a collected VIO dataset.

`ver all` and AUTOPILOT_VERSION agree:
- architecture MICOAIR_H743_V2; user model MicoAir743v2-AIO-35A;
- version 1.15.2, release-type field 0; branch micoair-v1.15.2;
- PX4 git hash `4817c0618a1286846116e90c6eb8919efaa013cf`;
- build Dec 21 2024 17:29:09, default variant; NuttX 11.0.0;
- NuttX hash `5d74bc138955e6f010a38e0f87f34e9a9019aecc`, GCC 10.2.1.

This is not the previously inspected dd0ad74 source or an official 1.16 install.
The reported hash is retrievable in upstream PX4 (commit subject: mRo boards:
Fix for USART clock selection). Source reads at this hash corroborate message,
driver and HIGHRES_IMU semantics below; a reported git hash does NOT prove no
vendor/uncommitted changes exist in the binary. No binary reproducibility claim.
The shell prints `nsh: sysinit: fopen failed: No such file or directory` at
session creation but executes status commands; preserved as a diagnostic warning,
not fixed or presented as proof of IMU failure.

### Live devices, topics and rates

`bmi088 -A status`, `bmi088 -G status`, `bmi270 status` report active SPI2,
SPI2 and SPI3 respectively. `sensors status` supplies device IDs:

| Instance | Accel / gyro device IDs | Device mapping | One-second sensor_accel / sensor_gyro publication Hz | vehicle_imu Hz |
|---|---|---|---:|---:|
| 0 | 6946834 / 6684690 | BMI088 accel 0x6a / gyro 0x66 | 802 / 666 | 199 |
| 1 | 3604506 / 3604506 | BMI270 type 0x37, currently selected | 1580 / 1580 | 196 |

Mappings use runtime IDs plus drv_sensor.h at the reported hash. `uorb status`
confirms sensor_gyro, sensor_accel, both FIFO topics and vehicle_imu each have
instances 0 and 1. Queue depths: raw SI topics 8, gyro FIFO 4, accel FIFO 1,
vehicle_imu 1. `uorb top -1 sensor_gyro sensor_accel vehicle_imu` gives the table;
FIFO batch publication rates match their sensor topics in this short snapshot.
These are measured INTERNAL publication rates, not sustained delivery rates,
per-sample hardware ODR or a no-loss guarantee. BMI270 source config requests
1600 Hz; actual status estimates about 1579--1581 Hz. No synthetic adjustment
was applied. The driver-reported nominal FIFO-empty interval is 800 Hz for
BMI088 accel/BMI270 and 666.7 Hz for BMI088 gyro; requested service cadence must
not replace observed topic cadence.

Drivers show zero bad-register/transfer/overflow/DRDY-missed events in this
snapshot and one FIFO reset each. VehicleIMU counters show accel/gyro gaps 1/1
for instance0 and 3/0 for instance1; these are accumulated counters, not evidence
of losses within a recorded dataset (none recorded). Do not claim loss-free IMU.

Read-only parameters: IMU_GYRO_RATEMAX=800, IMU_INTEG_RATE=200,
IMU_GYRO_CUTOFF=80 Hz, IMU_ACCEL_CUTOFF=30 Hz, IMU_DGYRO_CUTOFF=30 Hz,
SENS_BOARD_ROT=0. The cutoff parameters apply to downstream vehicle angular
velocity/acceleration processing in inspected source; they do not establish
sensor_* raw-topic bandwidth. Sensor-internal filtering and FIFO averaging
remain part of source provenance. Manual says 45-degree mounting; a parameter
value of zero alone does not determine actual physical mounting. No rotation
parameter was changed or recommended from that comparison alone.

### Data and clock semantics at the reported source revision

sensor_gyro = rad/s, sensor_accel = m/s^2, FRD board frame; both have driver
timestamp_sample and publication timestamp (FC HRT microseconds), device_id,
samples and clipping/error metadata. updateFIFO rotates counts, retains scale
and batch timing, and publishes a trapezoidal average to sensor_* when samples>1.
Thus measured gyro/accel is available, but single-sample versus averaged support
must remain explicit. BMI270 timestamp derives from HRT DRDY time with fallback
to scheduled read time/FIFO correction; exact ADC event uncertainty is unmeasured.
FIFO expansion requires verified first/last anchor and dt, not nominal ODR alone.

vehicle_imu is calibrated/integrated delta_angle [rad], delta_velocity [m/s],
not an instantaneous RAW_IMU rate sample. HIGHRES_IMU at this hash converts
delta/dt, may subtract estimator bias, and stamps imu.timestamp_sample. It is
NOT the earlier ArduPilot send-time path and NOT a raw-topic transport.
Default DDS source at this hash exports sensor_combined/timesync_status, not
sensor_gyro/accel or their FIFO topics. `uxrce_dds_client status`: not running.

### Transport: actual configuration versus future capability

`mavlink status` shows:
- /dev/ttyS0 @57600, Normal/MAVLink1, tx max 1200 B/s, observed tx 676.2 B/s;
- /dev/ttyS7 @115200, Normal/MAVLink1, tx max 5760 B/s, observed tx 1445.9 B/s;
- /dev/ttyACM0 USB CDC, Onboard/MAVLink2, nominal @2000000, tx max 100000 B/s,
  observed tx 1846.8 B/s during inspection.
These are status values, not capacity benchmarks or raw IMU transport rates.
COM19's Windows 57600 setting and USB CDC's nominal speed are not a physical
TELEM UART throughput measurement. Some MAVLink receive-loss counters include
discontinuous sequence numbers across diagnostic-client sessions (same sysid
254); they must not be relabeled as sensor sample drops.

Manufacturer mapping: FC ttyS0=TELEM1/UART1, ttyS3=TELEM2/UART4,
ttyS4=TELEM3/UART5, ttyS7=TELEM4/UART8, ttyACM0=USB. These are FC names,
NOT M3C pin assignments. No cable connects FC to M3C, per current user evidence.
Future USB CDC or UART may carry MAVLink; however stock HIGHRES_IMU does not
satisfy the strict raw-source requirement. DDS would need running client/agent
and appropriate exports; source defaults alone do not expose raw topics.
An existing raw/FIFO ULog could be an offline alternative, but logging profile
and suitable saved raw content have not been verified and were not requested.

Preferred future dataset path: OS04A10 + one fixed identified PX4 IMU (BMI270
instance1 is the currently selected candidate), preserve sensor_* or qualified
FIFO measurements/device IDs/sample times through a timestamp-preserving export,
then replay on host OpenVINS. Do not claim this export exists today, silently
substitute HIGHRES_IMU, or change firmware/parameters to create it in this task.
Choose USB versus UART only after checking M3C host-port/power and electrical
compatibility; no wiring action authorized. Raw-data export remains a separate
software design/approval step.

PX4 HRT and M3C monotonic/VIN clocks are unsynchronized separate domains.
TIMESYNC is a documented mechanism, not a verified mapping: no M3C link,
clock-offset/drift/RTT validation exists. Camera PTS remains opaque and receipt
timestamp is not exposure time. Later need clock mapping, camera exposure/row
timing, rigid T_imu_camera and bias/noise characterization; none starts now.

### Current next physical verification step (exactly one)

Photograph the current OS04A10 camera and flight-controller placement together,
with the FC direction arrow visible, to verify their relative orientation and
whether they share a rigid mounting. Keep existing wiring unchanged. This is
documentation of the current arrangement, NOT an instruction to assemble,
calibrate, connect M3C or apply a rotation parameter.

## Historical PX4-first correction before COM19 became available

**VIO-S0 PARTIAL; VIO-P remains PARTIAL, unchanged.** User explicitly corrected
the actual flight firmware to PX4 on 2026-10-08. This USER-STATED fact supersedes
ArduPilot assumptions for this VIO line. Do not assume an onboard M3C ICM-42688-P.
All local-ICM recommendations and ArduPilot firmware interpretations later in
this file are HISTORICAL, not current decisions. The broader rm27_drones hardware
ledger was not edited; its ArduPilot entry is stale for this task.

### Actual device identification

- M3C SSH is available at 10.18.198.1. Read-only inventory found ttyS0--ttyS5
  (ax-apb-uart), no ttyACM/ttyUSB or serial/by-id endpoint, and no identifiable
  MAVLink/uXRCE bridge process or listening UDP endpoint in the inspected list.
  UART device existence does not verify wiring, pin mux, baud rate or data flow.
  No serial port was opened or probed. `fuser` was unavailable, so port ownership
  is NOT established by that attempted check. USB product enumeration returned
  no FC identity. None of this proves a physical UART cable is absent.
- Windows Win32_SerialPort inventory returned Bluetooth COM ports only; no
  identified PX4 FC endpoint. Linux research host had no ttyACM/ttyUSB/by-id
  endpoint in the scoped check. Thus live `ver all`, AUTOPILOT_VERSION, uORB
  device IDs, driver status, parameters and measured rates are UNAVAILABLE.
- Actual firmware family: PX4 (USER-STATED). Exact board/target, firmware version,
  git hash and physical IMUs: UNKNOWN. Old MicoAir743v2-AIO-35A model statement
  is historical USER-STATED evidence, not reconfirmed hardware identity.
- Local PX4 source checkout:
  `/home/shiuhou/Projects/rm27_drones/rm27_drones_legacy/rm_uav_lab/PX4-Autopilot`,
  clean HEAD `dd0ad74fdadc68a62b479129ca3f382c006426f4`, local commit subject
  `RM27 C1: mini_quad_v0 airframe 4022 (gz, default world)`. This is a research
  source reference, NOT proof of the installed FC version or a flashed build.
  No .ulg or .px4 artifact was found in the bounded max-depth-5 project search;
  that search is not an exhaustive disk inventory.
- This checkout has distinct micoair/h743-v2 and h743-aio targets. Both enable
  BMI088/BMI270, MAVLink and uXRCE-DDS in default.px4board. rc.board_sensors uses
  BMI088 SPI2; BMI270 SPI3 for h743-v2 but SPI2 for h743-aio. This difference is
  why the board name must be read from the actual FC, not inferred from a label.
  Those sensors/buses are CODE-SUPPORTED candidates, not VERIFIED installation.

### PX4 measurement sources (all semantics below scoped to the inspected commit)

| Source | Values/frame | Timestamp/rate evidence | VIO use |
|---|---|---|---|
| sensor_gyro / sensor_accel | rad/s / m/s^2, FRD board frame; device_id, samples, clipping/error info | timestamp is publish time, timestamp_sample supplied by driver, FC boot microseconds; actual rate UNKNOWN; queue length 8 in these definitions | Preferred rate/accel source with identity and driver timing retained; samples>1 can represent FIFO averaging, not one physical sample |
| sensor_gyro_fifo / sensor_accel_fifo | rotated board-frame int16 arrays, scale in SI/count, samples and dt in microseconds | driver timestamp_sample and dt describe a batch; exact anchor/order must be checked for installed driver | Best preservation of per-sample evidence if already exposed/logged; retain counts/scale and reconstruct only with verified timing semantics |
| vehicle_imu | calibrated/integrated delta_angle rad and delta_velocity m/s, FRD body frame, separate integration dt, both device IDs | timestamp_sample is last gyro sample time in VehicleIMU; timestamp is publication; IMU_INTEG_RATE controls integration, not sensor ODR | Preserve as integrated data, not RAW_IMU. Do not divide by dt and silently relabel instantaneous raw readings |
| sensor_combined | selected, scaled/offset-compensated interval-average rates/acceleration, FRD body; accel relative timestamp and integral dt | gyro sample time in timestamp; accel relative time can be invalid sentinel; actual rate UNKNOWN | Possible processed fallback if timing/provenance accepted explicitly; not raw per-device FIFO and may switch sensors |
| vehicle_attitude / odometry | estimated orientation/state | estimator timing | NOT a replacement for measured gyro+accel |

PX4Gyroscope.cpp/PX4Accelerometer.cpp update() rotate and scale single samples;
updateFIFO() publishes arrays then derives trapezoidal batch-average sensor_*
values with samples=N. Hardware filtering may still exist. For raw topics,
board-frame rotation is not automatically vehicle/body mounting calibration.
The inspected BMI088/BMI270 drivers derive sample stamps from HRT DRDY interrupt
time when usable, with FIFO adjustments and polling fallbacks. These are not
automatically hardware-latched ADC acquisition times. Actual driver, IRQ mode,
FIFO anchoring, filter delay and timestamp uncertainty remain to be verified.

### Transport findings and rate limits

1. **MAVLink HIGHRES_IMU is not the previous ArduPilot implementation.** Here it
   selects vehicle_imu, converts delta quantities to interval-average SI rates,
   may subtract estimator_sensor_bias for matching device IDs, and uses
   `time_usec = imu.timestamp_sample`. Therefore its timestamp is sample-related,
   not send time, but it is still NOT raw sensor_* output. It loses separate
   integration-window detail and can follow sensor selection. Avoid introducing
   estimator-bias feedback into an allegedly independent raw OpenVINS baseline.
   SCALED_IMU also derives from vehicle_imu in this checkout, using millisecond
   imu.timestamp and quantized mg/mrad/s; a RAW/HIGHRES/SCALED name is insufficient.
2. MAVLink source offers HIGHRES_IMU 50 Hz in some modes, and unlimited_rate in
   EXTVISION mode. These are requested policies, not observed rates. Generation,
   integration, scheduler, link bandwidth and competing traffic limit delivery.
   No stream request or MAVLink parameter change was sent. **Achievable rate,
   latency, jitter and drop count on actual PX4--M3C link remain UNVERIFIED.**
3. uXRCE-DDS is compiled in the inspected candidate board configurations, but the
   inspected dds_topics.yaml exports sensor_combined, NOT sensor_gyro,
   sensor_accel, their FIFO topics or vehicle_imu. Enabling DDS does not expose
   arbitrary uORB messages. Installed-version support is unknown; adding topics
   could require a firmware build, outside this read-only task. Do not deploy
   ROS2 on M3C merely to preserve a proposed transport.
4. Existing ULog could preserve raw/FIFO topics without a live bridge; logger
   code supports gyro/accel FIFO profiles. Current logging profile and actual
   logs are unknown. Ordinary logged sensor_* may be decimated. No logging
   profile was changed and no new log was requested. ULog alone would still
   need camera-to-FC time association for a paired dataset.

Later rate verification must distinguish sensor ODR, FIFO batch rate, uORB
publication rate and transport receive rate; report timestamp intervals,
device IDs, integral/sample counts, loss/reset evidence and receipt jitter.
MAVLink packet sequence covers link traffic, not all physical IMU samples.
No numerical throughput or UART capacity estimate here is a measured result.

### Clock and camera chain

PX4 boot/HRT time and M3C Linux monotonic time are separate clocks. MAVLink
TIMESYNC in the inspected source exchanges ns fields based on HRT microseconds
and supports offset estimation. It does not establish camera exposure time or
prove current synchronization. No live exchanges, RTT/offset/drift/residuals,
reset behavior or synchronization software on M3C were verified. A later mapping
must preserve original integer FC times, M3C receipt times, epochs, uncertainty
and clock drift; never subtract the two domains directly.

uXRCE-DDS serialization receives session time_offset/1000; any generated
timestamp conversion (including timestamp_sample handling) must be checked for
the deployed message version, rather than assuming original FC time survived.
Do not apply a second offset blindly after a middleware conversion.

Existing OS04A10 evidence remains unchanged: VIN u64PTS is opaque with unknown
unit/domain/exposure event; monotonic_us is recorded after VIN frame retrieval.
That is receipt evidence, not exposure midpoint. Even correct PX4--M3C TIMESYNC
does not solve VIN-to-host/exposure mapping, rolling-shutter line timing or
filter/transport delay. No trigger/FSYNC/shared clock is verified. Later keep
camera/IMU spatial transform, time offset, clock drift and row timing distinct.

### Recommended path and decision boundary

Prioritize **OS04A10 on M3C + physical PX4 IMU**, not assumed M3C ICM hardware.
The simplest reliable target is one identified PX4 gyro/accel pair, preserving
sensor_* (or verified FIFO) sample timing/device IDs, over an existing supported
timestamp-preserving link, recorded alongside M3C camera receipt/PTS and an
explicit clock mapping, then replayed by host OpenVINS. First inspect deployed
firmware/link capabilities. If its existing raw bridge is absent, do not claim
HIGHRES_IMU satisfies the raw-data requirement or automatically flash a bridge.
An already-existing suitable ULog is the lowest-change offline alternative;
otherwise a separately reviewed transport/logging change would be required.
There is currently no evidence-qualified, plug-and-play live raw path to select.
No M3C estimator port, EKF2/flight integration, calibration or capture started.

### Evidence hashes and next physical step

SHA256 in the local PX4 reference checkout:
- msg/SensorGyro.msg: `7c7abd7147475f1a62c5b4a6f90ee056869cf107b2da732bc191d335d93d4984`
- msg/SensorAccel.msg: `172b4c666cc92e7f144e281aa53b7ccfb811b591ec270ab6721af44f3d592f40`
- msg/VehicleImu.msg: `36cd02728d1f825bb7da37e03624b8e2e7d055ae76aae1576aaffad6197d92bb`
- HIGHRES_IMU.hpp: `43ab7eb355b812b6a35c0537e9e170b414378c8a7bec84e754fe8238d7085336`
- dds_topics.yaml: `01b21d9d8a9773dbfcce1ce4127ee2cb72b6cad9aa8cedeefcf76f7c4f242120`

**Exactly one next physical verification step:** connect the PX4 flight
controller to the Windows computer using its USB data cable, without connecting
the propulsion battery, so board/firmware/IMU identity can be read without changing
flight parameters. Do not infer a UART pinout or move any M3C pins.

## HISTORICAL PRE-CORRECTION MATERIAL — not the current sensor selection

## Connectivity restored — subsequent read-only check

After the user's reconnection, MaixPy `device check` succeeded at 10.18.198.1:
Linux maixcam2-9d05, aarch64, kernel 4.19.125. Earlier connection failures below
are historical, no longer the live blocker. `/dev/spidev1.0`, `/dev/spidev2.1`
and `/dev/spidev2.3` exist; spi1.0 binds generic spidev, not an identified IMU
driver. IIO inventory exposes a platform ADC node, not a verified six-axis IMU.
Scoped `/root` search found no imu-tests directory or IMU summary/collector.
`/maixapp/apps/imu_ahrs` contains a bundled AHRS demo importing maix.ext_dev.imu;
its presence/README do not prove an installed or working sensor. No demo executed,
no registers read, no mux changed, no samples acquired. S0 stays PARTIAL pending
sensor identity evidence; the USB reconnection action below has been completed.

## Current S0 result: PARTIAL; no physically qualified VIO IMU yet

The source audit below supersedes the older candidate inventory. VIO-P's
repeatability study is closed as an experiment, not as root-cause resolution:
10 native runs ATE 5.432--7.653 cm, 15/45 pairs over the unchanged 1 cm gate;
three configuration controls did not remove variation. VIO-P remains PARTIAL.
See `NATIVE_REPEATABILITY.md`. No new public runs are needed for this audit.

Read-only checks on 2026-10-08: the MaixPy helper `device check` from Windows
returned SSH timeout to root@10.18.198.1. A bounded check from the Linux research
host returned No route to host. Therefore current board identity, installed IMU,
device nodes, driver bindings and existing `/root/imu-tests` evidence are
UNAVAILABLE, not absent. No sensor reads, pin mux changes, services stopped,
bus scans, data capture or calibration were performed.

## Candidate comparison

| Candidate | Device / gyro and accel | Rate / units / filtering | Time, buffering and direct M3C access | Qualification |
|---|---|---|---|---|
| M3C/local custom baseboard | ICM-42688-P candidate; six-axis SPI FIFO decoder exists | CODE-SUPPORTED nominal 1 kHz baseline, 4 kHz stress; raw integer accel/gyro scaled to g and deg/s; sensor UI filtering configured | CODE-SUPPORTED `/dev/spidev1.0`, 16-byte FIFO packets with 16-bit sensor timestamp; host batch monotonic timestamp; actual access and latency UNAVAILABLE | Preferred candidate to verify, not an identified current installed sensor |
| Camera-board IMU | No inspected board schematic/BOM/device ID binds a separate IMU to the OS04A10 camera module | UNKNOWN gyro/accel, rate, units, filters | UNKNOWN bus, clock, FIFO and M3C access; do not equate the image sensor with a six-axis IMU | UNKNOWN; cannot recommend as an available source |
| Flight-controller IMUs | USER-STATED MicoAir743v2-AIO-35A; CODE-SUPPORTED BMI088 (SPI2) and BMI270 (SPI3) | Firmware exposes gyro+accel; actual selected instance/rate/range/filter UNKNOWN; HIGHRES_IMU uses SI rates/accel from filtered frontend | FC boot clock; send-time timestamp in inspected HIGHRES_IMU; no verified M3C link or raw-sample transport; FIFO/telemetry delay UNKNOWN | Physically plausible fallback, not currently qualified or directly accessible |
| Other external IMU | No identified available sensor in scoped evidence | UNKNOWN | UNKNOWN | Do not invent a device or recommend buying one from this audit |

MTF-02P flow/range and M10G-5883 compass records do not establish a gyro+accel
source. EuRoC IMU and SITL logs are not real RM27 sensor-chain evidence.
EKF attitude/orientation, MotionPrior and TargetEstimate fields are not raw IMU.

## Newly identified local path: ICM-42688-P

Local source snapshot: Windows repository
`C:/Users/USER/OneDrive/RM27/rm27_slam/.maixpy/source-inspect/maixcam2_dart`,
HEAD `c7d47b20a5a948db1e55f99fb128ee7616d02e1c`, clean at inspection.
The files `tools/imu_icm42688/{README.md,spi.py,stress.py}` were read; existence
and contents are VERIFIED artifacts, hardware claims remain CODE-SUPPORTED.
This is stronger than the old conversational SPI description but still does
not prove the user's replacement mainboard has the same sensor/wiring.

- SPI1 A0=MOSI, A1=MISO, A2=CS0, A4=SCK; Linux ioctl path `/dev/spidev1.0`,
  mode 0, 8 bits, 1 MHz baseline / 4 MHz stress. Code expects WHO_AM_I 0x47.
  These are source settings, NOT instructions to alter current wiring.
- Both accel and gyro appear in `>B6hbH` 16-byte FIFO packets, plus temperature
  and 16-bit timestamp. Nominal full scale is +/-2 g and +/-250 deg/s. Scaling
  is accel raw/16384 g, gyro raw/131 deg/s. Future SI conversion is g*9.80665
  and deg/s*pi/180, retaining source counts and range/register evidence.
- `stress.py` writes filter register 0x52=0x44; README describes ODR/10 UI
  bandwidth, not independently measured ENBW. Thus raw FIFO bytes are not
  evidence of unfiltered analog measurements. No software bias correction is
  applied by this collector; separate drift-filter replay must not replace them.
- It selects internal PLL, configures timestamp register 0x54=0x21 and preserves
  raw 16-bit time. Exact tick resolution, event, rollover/reset interpretation,
  drift and acquisition alignment need authoritative register and live evidence.
  A modulo timestamp difference alone cannot certify lost-sample absence.
- FIFO count is read coherently from 0x2E; code bounds count at 2048 bytes and
  reads whole 16-byte packets. It sleeps 2 ms at nominal 1 kHz and 1 ms at 4 kHz.
  These are requested poll intervals, not measured latency or ODR guarantees.
  Host `time.monotonic_ns()` is saved after FIFO read AND `raw.write(data)`;
  the batch timestamp therefore includes software/file-write delay, not a
  per-sample acquisition time. Buffer occupancy, overflow observations, max poll
  gap and lost-count registers are recorded by code, but no current values verified.
- No hardware sample sequence is provided by this format. Repeated axis values
  are not necessarily duplicate packets. Summary sample_count/elapsed is a
  host-clock rate estimate. FIFO backlog at start/end can affect that estimate.
- Collector changes mux/registers and performs acquisition; it was deliberately
  NOT executed for this read-only audit. README's older test claims were not
  promoted to live hardware evidence or to verified noise parameters.

## Camera and clock evidence

Existing local run `.maixpy/runs/os04a10-20261007-005702-52804b/capture.json`
records 5404 frames, 180.134784 fps, 1344x760 frame CSV, zero reported acquisition,
release and encoder drops, and exit 0. This is VERIFIED saved camera-run evidence,
not a fresh capture or proof of all exposure events. No new recording was made.
The inspected `official_record.cpp` in `.maixpy/repos/maixcam2_dart_vision`
(HEAD `6d2f6251dd839c8673cfcf8a977e22a09433afcf`) copies VIN `u64PTS` unchanged
and calls C++ steady_clock after `AX_VIN_GetYuvFrame`/`GetRawFrame` returns.
It supports interpreting `monotonic_us` as application receipt/return time;
exact source-to-binary binding of the historical run is not newly established.
PTS unit, clock domain, exposure-start/mid/end and row timing remain UNKNOWN.
Nominal frame rate or H.264 PTS cannot supply those missing semantics.

Even a local IMU is NOT proven synchronized with this camera. IMU internal PLL,
VIN PTS and Linux host time are distinct or unproven domains. C++ steady_clock
and Python monotonic on the same board may offer a software comparison domain,
but require actual clock implementation/boot-epoch evidence; they do not make
sensor acquisition clocks shared. FC timestamps are from a separate MCU clock.
No FSYNC/trigger/clock wiring, offset or drift map has been verified.

## Flight-controller path rechecked

Existing `rm27_drones/docs/REAL_HARDWARE.md` and `configs/real_hardware.json`
still mark firmware/parameters/installation pending. Pinned ArduPilot below is
unchanged. `AP_InertialSensor.h` get_gyro/get_accel return frontend arrays;
`AP_InertialSensor_Backend.cpp` publishes `_gyro_filtered`/`_accel_filtered`
(around lines 807/869). `GCS_Common.cpp::send_highres_imu` stamps the send function
with `AP_HAL::micros64()`, not each hardware acquisition. Rate, batching, UART
queueing and delivery losses are not measured. Direct FC SPI access from M3C
is not established and is not proposed. Scoped `flight_logs/data` contains SITL
logs, no physical .BIN/parameter intake found. Do not classify telemetry named
RAW/HIGHRES as an unfiltered, synchronized sample stream without evidence.

## Pairing recommendation and later gates

Most practical *conditional* path is existing OS04A10 + local ICM-42688-P FIFO
on the same M3C, **if actual installation and access are confirmed**: it reuses
an existing direct collector and avoids adding FC telemetry transport. This is
not a final sensor selection or proof that a suitable IMU is currently available.
If unavailable, reassess the identified FC candidate without starting integration.

Later required evidence: sensor identity/firmware/registers/rate, signed axes and
units, FIFO loss/reset semantics, filter/group delay, sample-clock to host mapping,
camera PTS event and exposure/row timing. Later spatial calibration estimates
`T_imu_camera` (camera to IMU); temporal calibration must separately identify
offset, drift and transport/receipt delay. Confirm rigid mounting and characterize
IMU bias/noise/temperature; preserve existing camera intrinsics, recheck mode/lens
binding rather than redo them automatically. No calibration begins in this task.

## Fresh artifact hashes (SHA256)

- ICM stress.py: `9c885953b6ab2efb3107c47efde008a0fabd4cc76c4fdc8449bc0bfdb7bfc163`
- ICM spi.py: `c43a1d844209030d08da411ce117c87ab1917b1cc48a7554dfb5630f38032a0e`
- Camera capture.json: `78d97def5ea41175ae0ea43ce06940d8bea8d738d8653a7da07e418a82a59ba3`
- Camera frames.csv: `2c505cf38967396284dd5baf7576e75404e4c7c1ff1ec773a75ac7f281294d8e`

## Exactly one next physical action (current)

Reconnect the M3C's existing USB data link to the Windows computer so its
previous SSH endpoint `10.18.198.1` becomes reachable for read-only inventory.
This is not permission to rewire SPI, connect the flight controller, or record data.

## Historical 2026-10-07 audit (superseded where noted above)

**VIO-S: PARTIAL.** Repository/source audit is complete for the scoped candidates;
live M3C inventory could not be obtained. No physical IMU recording was acquired.

## Evidence classification

VERIFIED = directly observed artifact/runtime fact; USER-STATED = reported by
the user; CODE-SUPPORTED = supported by inspected source, not the current board;
UNKNOWN = evidence absent; UNAVAILABLE = attempted access failed. File existence
does not verify the hardware claim contained in that file.

| Item | Finding | Status / evidence |
|---|---|---|
| Flight controller candidate | MicoAir743v2-AIO-35A, ArduPilot | USER-STATED, main project `docs/REAL_HARDWARE.md`, recorded 2026-09-12 |
| Supported IMUs | BMI088 and BMI270 | CODE-SUPPORTED, pinned ArduPilot hwdef below |
| BMI088 connection in firmware | accelerometer/gyro chip selects on SPI2; driver BMI088 | CODE-SUPPORTED, hwdef `SPIDEV` and `IMU` entries |
| BMI270 connection in firmware | SPI3; driver BMI270 | CODE-SUPPORTED, hwdef `SPIDEV` and `IMU` entries |
| Firmware sensor rotations | BMI088 ROLL_180_YAW_270; BMI270 ROLL_180 | CODE-SUPPORTED only; not the actual aircraft orientation parameter |
| Installed sensor/firmware/selected IMU | No actual ID, parameter export or physical DataFlash record verified | UNKNOWN |
| M3C onboard/attached IMU | SSH to root@10.18.198.1:22 timed out | UNAVAILABLE; no sysfs/device result obtained |
| FC to M3C transport | No verified raw-IMU stream/path | UNKNOWN; earlier SPI discussion is not proof |
| Candidate MAVLink HIGHRES_IMU | Compiled conditionally; reads `ins.get_accel()/get_gyro()` and sets `time_usec=AP_HAL::micros64()` inside the send function | CODE-SUPPORTED, `GCS_Common.cpp:2269–2313`; not a sensor acquisition timestamp or proof of enabled live streaming |
| Raw gyro + accel samples | None verified from real hardware | UNKNOWN; no telemetry fabricated |
| Units/rate/range/filter settings | Not observed for actual recording | UNKNOWN |
| Axes/handedness/body mounting | Firmware defaults insufficient | UNKNOWN |
| Acquisition timestamp/clock | Not observed | UNKNOWN |
| Sequence/drop/saturation semantics | Not observed | UNKNOWN |
| Camera–IMU transform/offset/drift | No qualified artifact | UNKNOWN |
| Noise/bias parameters | No real stationary capture analyzed | UNKNOWN |

## Scoped sources and provenance

Linux repository root: `/home/shiuhou/Projects/rm27_drones`.

- `docs/REAL_HARDWARE.md` SHA256
  `31fb5f9dff4235a9ef126902754b38a13d2e880429cbbb59a33052aea43e2ca7`.
- `configs/real_hardware.json` SHA256
  `bc6a9bbff0bba04b1d438967d48f5cf982cee896485353508d7e4e4cc5e25e10`.
- ArduPilot checkout `rm27/autopilot/vendor/ardupilot`, HEAD
  `96f98857f7fc4bd564fbbdc43c5f26114246369f`.
- `libraries/AP_HAL_ChibiOS/hwdef/MicoAir743v2/hwdef.dat`, SHA256
  `e1760d215c88bd185e1c8e1c70f77a062dd56dd690816c312a2fbc7bcf0ed99c`.

The main project records pending physical DataFlash/parameter intake. Scoped
inspection of `flight_logs/data` found no `.BIN`; simulation artifacts elsewhere
contain `.BIN` files but were not reclassified as physical evidence. The
`references/realcontroller` directory contains a video, not a verified raw IMU
sample stream. This is not an exhaustive claim about every file on the host.

The inspected `GCS_MAVLink/GCS_Common.cpp` HIGHRES_IMU implementation is a
candidate telemetry source, not an approved VIO capture source: verify filtering,
sample selection, rates and timestamps from actual firmware/data before use.
Its send-time clock must not be silently marked `ACQUISITION`. A message name
containing RAW or HIGHRES does not qualify its acquisition timing.

All inspection was read-only. No bus probing, device configuration, camera app
stop, flight-controller connection, firmware installation or parameter writes.
The old hardware document's camera-calibration/pipeline status predates the
2026-10-07 OS04A10 experiments and is not used to invalidate newer evidence.

## Implemented preparation

Existing raw sample validation is preserved. `experiments.vio` now validates a
single-device sequence and provides optional static descriptive statistics and
overlapping Allan deviation. `imu_capture.template.json` records unknown capture
metadata explicitly. Tests are synthetic, not sensor qualification.

## Historical physical blocker

Restore the M3C's power/network connection so SSH at `10.18.198.1` is reachable
for a read-only device inventory. Do not change wiring or start IMU recording
based only on these firmware definitions.
