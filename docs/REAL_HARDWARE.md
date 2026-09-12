# Current real-aircraft hardware baseline

Status: user-reported configuration recorded on 2026-09-12. Exact installed
firmware, parameters, sensor ports and orientations will be verified from the
pending original ArduPilot DataFlash `.BIN` and full parameter export. The
machine-readable counterpart is [`configs/real_hardware.json`](../configs/real_hardware.json).

## Current aircraft

| Item | Current information | Evidence status |
|---|---|---|
| Ready-to-fly mass | approximately 180 g | User reported; exact measurement pending |
| Envelope | 120 × 120 mm provisional | Temporary authorized model |
| Motors | 1103 9600KV | User reported |
| Propellers | 2 inch | User reported; model/blade count pending |
| Battery | Unknown | Cell count, capacity and voltage pending |
| Flight controller | MicoAir743v2-AIO-35A | User reported |
| Firmware family | ArduPilot | User reported; version pending |
| Flow/range sensor | MTF-02 or MTF-02P | Exact variant pending |
| GNSS/compass | M10G-5883 | User reported |

The current values above supersede candidate combinations found in old project
conversation records. In particular, do not substitute 9500KV motors, 2.5-inch
props, or an assumed 2S/3S battery without new evidence.

## Flight controller

The manufacturer's current manual identifies an STM32H743VIH6 MCU, BMI088 and
BMI270 IMUs, SPL06 barometer, microSD logging and an integrated four-channel
35 A Bluejay ESC. The ArduPilot firmware target is `MicoAir743v2`, shared with
the non-AIO board. The manufacturer specifies a 45° physical installation for
the AIO, but the aircraft's actual orientation parameter must be read from its
parameter set before interpreting body axes. Do not infer it from the board
manual alone.

Official source:
[MicoAir743v2-AIO-35A manual](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual).

## Optical flow and rangefinder

MTF-02 and MTF-02P combine downward optical flow and ranging over a 3.3 V LVTTL
UART, nominally at 115200 baud and 50 Hz. Manufacturer specifications relevant
to modeling are:

| Property | MTF-02 | MTF-02P |
|---|---:|---:|
| Nominal range at stated reflectivity/light | 0.02–2.5 m | 0.02–6 m |
| Range field of view | 20° | 2° |
| Mass | 1.0 g | 1.5 g |
| Minimum optical-flow distance | 8 cm | 8 cm |
| Optical-flow field of view | 42° | 42° |
| Nominal data rate | 50 Hz | 50 Hz |

The manual quotes 2 cm ranging accuracy from 0.02–2 m under its stated target
reflectivity, and optical-flow operation above 60 lux. Those are manufacturer
conditions, not measured aircraft performance. The exact sensor variant,
orientation, surface texture, lighting and vibration must accompany log analysis.

Official source:
[MTF-02/02P manual](https://micoair.cn/zh/docs/sensors/sensors/mtf-02-02p-sensors).

## GPS and compass

The M10G-5883 uses a u-blox M10/m10050 receiver and QMC5883L compass. The
manufacturer lists 5 Hz navigation, NMEA/UBX, a 20 × 20 × 8 mm package and
7 g mass. GPS can provide an outdoor reference for comparing estimated motion,
but the intended indoor competition localization remains based on other
sensors. Compass analysis must account for mounting orientation and magnetic
interference from battery wiring, motors and the integrated ESC.

Official source:
[MicoAir GPS module manual](https://micoair.cn/zh/docs/gps-rtk/gps).

## Relationship to the current simulation

The root MuJoCo micro-aircraft currently uses the reported mass, motor KV class,
propeller diameter and provisional envelope. The following remain engineering
estimates rather than properties calibrated from this aircraft:

- inertia tensor;
- maximum thrust and thrust curve;
- motor/ESC response time;
- yaw torque per unit thrust;
- aerodynamic drag and ground/guard effects;
- IMU noise, bias, vibration and timing;
- barometer, optical-flow, rangefinder, GPS and compass error models;
- controller gains suitable for the real aircraft.

The simulation's millimetre dock return uses MuJoCo truth in its outer guidance
loop. It is not evidence of MTF, GPS, vision or real-aircraft landing accuracy.
Never write `configs/micro_hover.parm` directly to the real flight controller.

## Pending flight-log intake

Keep the first received `.BIN` byte-for-byte unchanged. Record its SHA-256 and
analyze it read-only. Ideally receive these companion facts/files:

- complete Mission Planner parameter export;
- ArduPilot version and board target;
- MTF-02 versus MTF-02P;
- measured ready-to-fly mass;
- battery cell count/capacity and propeller model/blade count;
- sensor port and physical orientation;
- flight mode, indoor/outdoor setting, floor surface and lighting;
- short description of each maneuver and any synchronized video.

Initial analysis will inspect IMU clipping/noise/vibration, attitude target and
response, motor output, hover throttle, battery voltage/current, estimator
innovations, range height, flow quality/velocity and GPS reference when present.
One log may constrain several model parameters, but it cannot uniquely identify
every thrust, inertia, drag and latency term. Results must label direct
measurements separately from fitted or unresolved values.

The local intake convention is documented in
[`flight_logs/README.md`](../flight_logs/README.md). Raw data belongs under the
Git-ignored `flight_logs/data/`; analysis artifacts belong under `artifacts/`.

## Historical material

[`chatgptre.md`](../chatgptre.md) is a conversation archive from an earlier
agent and may contain obsolete hardware candidates, plans and completion claims.
Use it only to locate ideas or possible sources. The evidence order is defined
in the root [`AGENTS.md`](../AGENTS.md); current user statements, raw data and
primary sources take priority.

## Provisional onboard vision

User-selected hardware: **Sipeed M3C 自研視覺核心板 + OS04A10**.
The user explicitly confirmed OS04A10 as the selected image sensor.
Exact board revision, lens, camera calibration, mounting transform, actual frame
rate, capture-to-control latency and flight-controller transport remain unknown.
Do not substitute the OS04D10 reference configuration for this selected sensor.

Development reference: https://github.com/blademaster679/maixcam2_dart .
Local checkout inspected at `0f3018e989166a314def24d3b53fc492fdfcdcc5`.
That older local README describes an OS04D10 baseline. Subsequent fetch found
`origin/main` at `f4c153ff7a8f367c7bd6c470e01719d33b359647`, whose README
uses OS04A10 full-field 1344×760@180fps as the development main mode.
The working checkout was not advanced by that fetch. Capture rate is distinct
from detection/control rate; actual M3C performance and calibration remain
unverified. OS04D10 is historical reference, not the selected hardware.

The local candidate interface is `TargetEstimate` JSON v2, carrying timestamps,
measurement age, tracking state, predicted/measured status, `safe_for_control`,
marker/armor observations, camera-frame line of sight and optional pose.
A future adapter must validate freshness and frame conventions and respect
`safe_for_control`; green-marker detection alone does not establish armor
position, metric range or valid flight guidance. UART/CAN integration remains
listed as future work in that README. No vision-to-ArduPilot connection has
been added to RM27 by recording this hardware update.
