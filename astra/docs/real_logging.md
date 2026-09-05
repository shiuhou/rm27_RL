# Real campaign logging contract

Companion to [the four experiments](real_experiments.md). This is a recording
specification, not a deployable ArduPilot parameter set. Firmware on the real FC
is unreported. The local reference source is ArduCopter V4.7.0 at `1511f27`;
its message names/rates may differ from the installed release.

## FC setup and rate check

Save the full parameters before any change. For props-off recordings, select
`LOG_DISARMED=1` after checking the installed firmware's parameter metadata;
other disarmed modes may omit USB sessions or discard never-armed logs.
Download the original DataFlash `.BIN`, not only MAVLink telemetry or plot images.

Inspect `LOG_BITMASK` by **named options**, preserving existing necessary bits:
FastAttitude, SystemPerformance, ControlTuning, NavigationTuning, RCinput, RCoutput,
IMU, BatteryMonitor, OpticalFlow, PID and Motors. Preserve mission/mode/error
records. Avoid copying an integer bitmask from another firmware. Capture all EKF
lanes and source switches. `LOG_REPLAY=1` is optional for a designated diagnostic
run after verifying bandwidth; recording EKF plots alone is not an EKF replay.

Before flight, record **60 s props off** with intended camera/logging load.
Inspect actual message intervals, `DSF.Dp`, performance and storage gaps. Require
no increasing dropped-log counter, no loop-rate/rail fault and adequate SD space
before carrying the configuration into a flight. If logging overloads the FC,
reduce diagnostic volume and repeat; do not silently accept gaps as sensor loss.

The reference source logs fast attitude/rate at the applicable control-loop
rate, EKF at 25 Hz with FastAttitude, RC output at 10 Hz and normal IMU at 25 Hz.
Battery updates are 10 Hz there. Actual scheduling, rate limits and selected
firmware govern the real file. Aim for ≥100 Hz desired/achieved attitude and
body rate for response fitting; otherwise report the available resolution.
Native flow is nominally 50 Hz; determine whether FC logging preserves that rate.
Do not upsample 10 Hz RC output and claim a 40 ms actuator response measurement.

For vibration, use one selected IMU's pre/post-filter batch samples, verifying
`INS_LOG_BAT_MASK` for its actual index and `INS_LOG_BAT_OPT` meanings. The documented
initial combination is mask 1 for IMU1 and option 4 for pre/post sampling. Avoid
also enabling the incompatible raw-IMU logging path. Preserve `ISBH` and `ISBD`
and their recorded sample rates. Frequencies above Nyquist are unobservable or
aliased; a 1 kHz recording cannot resolve all small-prop vibration. Higher
sensor-rate logging requires a separate load check. Do not change gyro loop
rate or filters just to obtain this recording. Restore diagnostic logging when
finished. See [ArduPilot batch-sampling documentation](https://ardupilot.org/copter/docs/common-imu-batchsampling.html).

## Signals to retain

Parse field names and units from each log's `FMT`, `FMTU`, `UNIT` and `MULT`,
using [the release-appropriate message definitions](https://ardupilot.org/copter/docs/logmessages.html).
The following are reference names, not permission to substitute zeros when a
message is absent. State missing capability explicitly.

| Group | Required content / reference names | Purpose and limitation |
|---|---|---|
| Identity/timing | `VER`, `MSG`, `PARM`, `FMT/FMTU/UNIT/MULT`, `TimeUS`, boot identity | Firmware/configuration, units and clock domain |
| State/control | `ATT` desired/achieved roll, pitch, yaw; `RATE` desired/actual rates and outputs; `PIDR/PIDP/PIDY`; `RCIN`, `RCOU`; `CTUN` | Distinguish command, tracking and saturation; PWM is not thrust |
| Inertial | `IMU` all instances: gyro, acceleration, temperature, health/sample metadata; `VIBE` axes and clipping counters | Bias, mount vibration and estimator contamination; counter layout varies by version |
| Flow | `OF.Qual,flowX,flowY,bodyX,bodyY`; timestamped native UART if captured | Angular flow and rotation, quality/latency; not direct world velocity |
| Ranging | `RFND.Instance,Dist,Stat,Orient,Quality`; active range instance | Surface/height/tilt validity and bias; do not silently select the wrong instance |
| EKF motion | `XKF1` lane `C`, `PN/PE/PD`, `VN/VE/VD`, roll/pitch/yaw, gyro bias | Estimated motion/bias; independent reference required for error |
| EKF fusion | `XKF2`, `XKF3`, `XKF4`, `XKF5`, `XKFS`, or installed equivalents | Accel bias, innovations/test ratios, health/reset/source switches; flow innovations `FIX/FIY`, height/range diagnostics |
| Energy/rails | `BAT` instance, `Volt,Curr,CurrTot,EnrgTot,Temp`; `POWR.Vcc,Flags,AccFlags`; cell measurements if available | Calibrate current against external known load; `VoltR`/`Res` are estimates, not independent battery truth |
| Motors | Motor/thrust-limit records (`MOTB` and installed equivalents); optional `ESC` per motor RPM/V/I/temp/error | Record absent telemetry; bidirectional DShot does not ensure all telemetry fields exist |
| Execution/safety | `MODE`, `EV`, `ERR`, `MSG`, `PM`, `DSF` | Actual mode, failsafes, loop load, dropped logging and reboot events |
| Diagnostic raw IMU | `ISBH/ISBD`, actual sample rate/filter stage/index | Limited vibration spectra; not simultaneous proof of unaliased motor RPM |

Keep all original fields even if analysis uses a subset. EKF local coordinates
and simulator coordinates differ: explicitly store world alignment, NED/ENU or
other axis conventions and units before comparing trajectories. Fit alignment
once at initialization; do not fit a transform over the whole flight to hide drift.

## Non-FC artifacts

**Reference CSV:** `clock_id,timestamp_s,x_m,y_m,z_m,qw,qx,qy,qz,valid`, plus
measured uncertainty, camera/rail calibration and world-frame definition.
Velocity is derived with the documented filter and edge handling; retain raw
positions. For a rail, record encoder displacement and orientation instead of
fabricated unmeasured six-degree-of-freedom pose.

**Camera frame index:** `frame_id,clock_id,capture_start_s,exposure_s,dequeue_s,
infer_start_s,infer_end_s,publish_s,receive_s,width_px,height_px,gain,valid,path`.
Missing/unavailable timestamps are null, never zero or duplicated delivery time.
Reference the original calibration and mount transform. Preserve sensor frames
before overlays/resize; lossless or documented low-distortion encoding, including
dropped-frame IDs and queue policy. Store detector version/config hash, all
candidates/abstentions and association/track age. Record system CPU/NPU use,
temperature and supply measurements at ≥1 Hz, noting unsupported counters.

**Timing events:** `clock_id,timestamp_s,event_id,event_type,frame_id`, with
optical transition/photodiode, camera metadata and publication/receiver marker
events as available. Save clock offset/drift fit and residual uncertainty.
Measure one-way age on synchronized clocks; RTT/2 is not a measured asymmetric
one-way latency. Inert diagnostic transport does not authorize flight commands.

**Stand CSV:** `clock_id,timestamp_s,command_ch1,command_ch2,command_ch3,
command_ch4,thrust_N,pack_V,current_A,rpm1,rpm2,rpm3,rpm4,pack_temp_C,
motor_case_temp_C,esc_temp_C`. Include instrument calibration, filtering and
bandwidth; nullable unavailable channels and slower temperature sampling retain
their actual timestamps. A filtered display value sampled quickly is not a
high-bandwidth measurement.

**Events CSV:** monotonic timestamp and clock, run/segment ID, start/stop, surface
or target change, occluder entry/exit, pack/prop change, mode, human intervention,
abort and reason. Preserve unplanned events. Save hashes for all artifacts and
use a new run ID after firmware, filter, tune, mount or target changes.
