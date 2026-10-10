# PX4 -> M3C IMU input qualification — 2026-10-10

## Follow-up: controlled rate comparison

`../imu-transport-20261010/VALIDATION.md` records approved reduced-telemetry tests:
unchanged115200/5760B/s, request50 actual50.001Hz; request100 actual62.112Hz.
Stream-budget scaling explains this original10.11Hz observation. All temporary
settings restored. New startup invalid bytes preserved; not a clean/raw/VIO PASS.
This original passive capture and its analysis remain unchanged; Stage C not started.

## Result

**VERIFIED:** current-firmware passive UART reception, HIGHRES_IMU decode/units,
source-time ordering and raw-byte replay. **PARTIAL:** real IMU input path.
**NOT QUALIFIED:** complete raw IMU stream, VIO suitability, clock synchronization.
**BLOCKED:**200–400+Hz goal from new_plan.md section21 cannot advance on the
unchanged115200/current5760B/s stream budget. Transport/measurement choice requires
user approval before any settings/firmware/wiring change. Full Stage C NOT STARTED.
Stage A/B results preserved, not rerun. v1.15.2/391-dropout capture is historical.

## Current firmware, safety and authority

Fresh `px4-preflight-retry.txt`: MICOAIR_H743_V2,v1.17.0,
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, same build as Stage A/B.
Pre/post commander Disarmed and EKF2_EV_CTRL=0. No PX4 parameter sets, firmware,
calibration, setpoints, arm, fusion or flight operation. PX4 USB inspection COM19
is not the physical UART. M3C maixcam2-9d05 /dev/ttyS2 and FC /dev/ttyS2@115200.

M3C idle termios was9600/canonical/echo after prior restoration. User explicitly
approved temporary115200/8N1/raw/no-flow for ONE30-second passive test, then restore.
`receive_imu.py` opens O_RDONLY, refuses occupied/console ports, reuses existing
probe owners/fresh_disarmed helpers, checks FC1/1 PX4 heartbeat, stops if armed/stale,
and restores exact saved termios in finally. No MAVLink heartbeat, stream request,
TIMESYNC reply or other UART transmission. Pre-session stale input explicitly
flushed; no evidence claim before capture window. No active process killed.

`px4-after.txt`: Disarmed,EV_CTRL0,UART RX0,comp191 Total933/lost163 unchanged from
prior Stage B. Remote completion: termios_restored=true; no Python receiver remains.
This restoration is9600/canonical idle state, not a permanent115200 configuration.

## Exact-source semantics (not message-name inference)

Retrieved source files are UTF-8 text renderings from the exact hash, stored here;
VehicleIMU.cpp reused from ../px4-v117-source-audit-20261009/source/new.

| Candidate | Actual PX4 path | Time/identity limitations |
|---|---|---|
| HIGHRES_IMU | Selected vehicle_imu by accel ID; delta_velocity/dt and delta_angle/dt, then subtract matching estimator_sensor_bias | time_usec=vehicle_imu.timestamp_sample=last gyro sample; dt/accel end/device IDs/clipping omitted; wire id remains0 |
| SCALED_IMU / SCALED_IMU2 | vehicle_imu instances0/1; calibrated integral/dt, quantized mG and mrad/s; no additional EKF-bias subtraction in stream | time_boot_ms=publication timestamp/1000, not acquisition; mag update may resend IMU; device IDs/dt omitted |
| sensor_gyro/accel_fifo via existing ULog tools | Driver-filtered/rotated pre-estimator samples; not untouched ADC | Keeps sample/scale/batch metadata offline; earlier SD losses unresolved; not current live UART |
| Future compact/batched raw transport | Explicit per-sensor sample timestamps,axes,scale,IDs,sequence,batch semantics | Requires separately authorized implementation/firmware or export path; NOT implemented here |

VehicleIMU integrates and applies calibration; gyro integration includes coning
corrections. Therefore even SCALED_IMU is not raw. HIGHRES_IMU is the simplest
already-active, no-firmware-change timestamped gyro+accel diagnostic path, but
not an independent unbiased VIO input qualification. No attempt to reconstruct
missing samples or undo estimator bias from rounded telemetry.

Primary source:
https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/streams/HIGHRES_IMU.hpp
and adjacent SCALED_IMU.hpp/SCALED_IMU2.hpp; mavlink_main.cpp rate scaling/defaults.

## One real passive capture

`imu-input-passive-01.jsonl`, identical local/remote SHA256:
`fa45502623a4d2667b2bfdbca06c0418bfdfb2581fd16d94334376d42d3dff51`.
Raw byte chunks and receipt timestamps are immutable, including legacy decoder
metadata. `analysis-final/summary.json` and `imu-telemetry.jsonl` are final derived
outputs. analysis-01/02 retained as superseded development analyses.

| Measurement | Observed |
|---|---|
| Dwell / bytes |30.000705556s /138955B (~4631.7B/s across all traffic) |
| Framed packets |2357, header source1/1;2240 known-dialect CRC-checked,117 opaque |
| HIGHRES_IMU |303, all MAVLink2,74B each; fields_updated6719 (all six IMU bits set) |
| Source-time span/rate |29.863883s /10.112550Hz |
| Source interval median/max |99141us /105150us |
| Source duplicate/backward /gap>1.5median |0/0/0 |
| Receipt rate /max interval |10.116672Hz /176.660583ms;1 gap>1.5median |
| HEARTBEAT |30,~1.00051Hz,base_mode29 |
| Numeric IMU checks |No nonfinite accel/gyro; accel mean norm9.805302m/s²,gyro0.016670rad/s |
| UART application writes |0 |

Rates use(N-1)/first-to-last span, not count divided by requested dwell. Receipt
is chunk/software timing; its larger gap is not a source-time gap and not proven
physical sample loss. Descriptive norms are not stationary/noise calibration.
No SCALED_IMU/RAW_IMU packets observed in this window.

Source binding: pre/post sensor_selection timestamp588236 unchanged; selected
gyro6684690 and accel6946834, consistent with BMI088 instance0 baseline, NOT
BMI270 instance1. Fresh vehicle_imu0 readback has these IDs,gyro dt4507us,accel
dt4988us. HIGHRES packets themselves do not carry these IDs or durations, so the
binding is source-code plus surrounding readback evidence, not per-packet identity.
The source-time gaps~99ms skip many~5ms integration windows; raising telemetry
frequency does not retrospectively recover those windows.

### Packet loss interpretation and decoder correction

Installed pymavlink2.4.49 common dialect lacks types8/380/410/411/436. Its UNKNOWN
object returns default sysid/compid/seq0 and msgid-2. Initial analysis therefore
misreported117 missing known packets and a fake0:0 source. A regression test first
reproduced the error; analyzer now extracts envelope headers directly from preserved
wire bytes and flags unknown-dialect CRC as UNVERIFIED. All2357 envelope sequence
numbers are contiguous modulo256; no duplicates/backward steps observed.

All303 HIGHRES CRCs verified; parser BAD_DATA bytes0. Unknown117 frames cannot be
CRC-qualified by this dialect. Header continuity is bounded packet evidence, NOT
exact sensor loss or universal loss-free transport (whole256-wrap losses and
upstream decimation cannot be counted). Original raw records untouched; final
replay matches after exactly117 explicitly reported legacy header corrections.

## Bandwidth and feasible alternatives BEFORE changes

115200/8N1 is11520B/s per direction before protocol overhead. Actual FC software
budget is5760B/s and Onboard preset enables many topics. Fresh `mavlink status
streams`: HIGHRES_IMU50Hz configured,10.113Hz scheduled; rate multiplier~.202.
Actual reception10.11255Hz agrees. This is confirmed source scheduling reduction,
not evidence of~80% UART loss. Current TX~4.5–4.7kB/s,TXerr0; no saturation claim
based only on UART theoretical maximum.

Conservative unsigned MAVLink2 HIGHRES full payload63+12=75B (this run truncated
zero extension to74B). `bandwidth-model.json` reproduces these budgets:

| Rate | HIGHRES alone |115200 wire utilization |
|---|---:|---:|
|50Hz|3750B/s|32.55%|
|100Hz|7500B/s|65.10%|
|200Hz|15000B/s|130.21% (impossible)|
|400Hz|30000B/s|260.42% (impossible)|

100Hz exceeds current5760B/s software budget even before other traffic. A temporary
50Hz focused-stream control could fit only by freeing other stream budget; it
would diagnose scheduling, NOT meet the original200–400+Hz target. Not executed.
200Hz HIGHRES uses65.10% of230400 or32.55% of460800 physical capacity;400Hz at
460800 uses65.10%. Both ends, effective software budget and other traffic must be
planned and measured. No supported-rate/electrical-margin claim yet. Signing adds
13B/packet. Reverse-direction ODOMETRY does not consume the same full-duplex TX
capacity, though CPU/scheduling resources can be shared.

Raw illustrated paired six-float axes alone at1600Hz require38400B/s, already
>11520B/s before timestamps/framing. Adding8-byte time+12-byte MAVLink2 envelope
gives70400B/s: needs higher bandwidth/batching. This is a FORMAT MODEL, not selected
BMI088 paired rate or an implemented message. USB may avoid UART limits but M3C
host wiring/roles remain unqualified and require approval. SD export reuses tooling
but is offline and its recording completeness is unresolved.

**Recommendation:** retain current transport diagnostic as PASS only. To pursue
new_plan's200–400+Hz target without firmware work, review a higher-bandwidth,
focused HIGHRES stream as a processed-input CANDIDATE first; accepting its bias,
integration and timing semantics for an estimator requires a separate decision.
If true individual pre-estimator samples are required, stock HIGHRES/SCALED cannot
meet that semantic contract at ANY baud; choose a raw/batched path explicitly.
Neither communication change nor production input choice is authorized by this report.

## Implementation, tests and limitations

receive_imu.py reuses probe guard functions and Stage A/B venv; imu_input.py reuses
pymavlink decoder and existing vio_dataset boundary (via tests/docs), not a new
estimator framework. Camera interface is CAMERA_IMU_INTERFACE.md. Unknown maps,
receipt-as-exposure and processed-as-acquisition remain rejected. No time fitting,
TIMESYNC responses, spatial/temporal calibration or real VIO run.

Host10 new protocol/interface tests pass; existing timestamp/calibration interface
29 tests pass. M3C16 tests:15pass/1host-interface skip, including2 pseudo-terminal
receive-only/restoration tests and4 preserved Stage A/B software tests. These are
offline tests, not repeat Stage A/B hardware tests. No production suite blanket claim.

Failures preserved: COM19 configuration I/O error on one preflight (retry succeeds,
no restart); pseudo-terminal fixture initially sent before ready/flush, fixed with
ready marker; first existing-interface pytest invocation used wrong cwd and imported
main package, rerun from correct offline mirror passes. Test failures not hidden.
No production dependency, commit/push, Vault or original Stage A/B report change.

## Next step / approval boundary

Before another hardware experiment, choose processed telemetry candidate versus
strict raw-sample input, then approve its explicit communication budget/settings
plan. Recommended next decision: whether higher-bandwidth focused stock telemetry
may be evaluated as processed input; do not automatically enable fusion or relax
200–400+Hz goals to match this10Hz result. Firmware/raw transport is a distinct
approval if required. Stage C remains the later real VIO -> PX4 gate.
