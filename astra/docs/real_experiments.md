# First real RM27 evidence campaign

Status: **operator protocol, not executed**. Revision 1, 2026-09-05.

The assembled prototype has 1103 **9600KV** motors, MicoAir743v2-AIO-35A,
MTF-02P, and available 1.8-inch/2.0-inch propellers. Battery, flying mass,
installed firmware, proven flight modes, and installed camera remain unreported.
The old 9500KV assumption and 163.5 g allocation are not measurements of this aircraft.

Run four experiments. E1/E2 can share a props-off field session. E3 precedes
propeller comparison in flight; E4 uses only combinations cleared by E1/E3.
The first useful deliverable is a set of measured operating envelopes and
architecture decisions, not a real interception attempt.

| Experiment | Minimum acquisition | Decision it distinguishes |
|---|---|---|
| E1: field flow/ToF | 27 height holds; 36 translation round trips; rotation check | Flow dead reckoning versus a need for field references or another sensor |
| E2: field camera | 12 range clips, 6 negative clips, 24 motion/occlusion clips; timing samples | Mount/exposure/FOV; bearing versus range; onboard timing versus external assistance |
| E3: guarded propulsion | 12 short stand sequences: two props × two battery states × three repetitions | 1.8 versus 2.0 inches; credible thrust, energy and lag limits |
| E4: field flight | Six paired 70 s profiles, then three 70 s field routes | Whether bench sensing and propulsion survive vibration, attitude changes and field transitions |

E4 totals **10.5 minutes of planned flight**, divided across batteries; never one
continuous flight. Add a 20 s qualification hover for each previously unproven
propeller, only after E3. Shorten/split profiles before takeoff if the measured
pack reserve requires it; use identical segments for each paired comparison.
Three repetitions screen mechanisms; they cannot certify competition reliability.

## Shared configuration, field and recording

**Before spinning props or flight:** fill in cell count, chemistry, rated capacity, exact
motor/prop ratings, flying mass, firmware version, current parameter export,
proven prop/mode combination, and operator-approved loaded-voltage/current limits.
Do not infer a safe battery voltage from the FC's voltage rating. Check the actual
yaw source: this FC has no onboard compass according to its
[manufacturer](https://micoair.com/flightcontroller_micoair743v2_aio_35a/).
An EKF source configured as compass is not evidence that a compass exists.

Freeze firmware, PIDs, filters, ESC settings, guards, mounts, payload and battery
class across a comparison. Log all automatic learned-parameter changes; if a
retune or repair is necessary, start a new comparison block. Added cameras,
markers or cables change mass/COM: weigh the complete flying configuration.

Define three **measured RM field conditions**:

- **F0:** normal route surface under normal competition lighting.
- **F1:** the least textured or most reflective real route patch, line, seam or
  ramp, if present. Record its actual material and geometry.
- **F2:** the limiting permitted lighting condition on that route: shadow,
  glare, LED illumination or lighting flicker, as actually observed.

Use the actual field, or documented samples/fixtures from the intended field.
Record which is used; a generic room only supports commissioning claims. Map
launch, travel, search and terminal-view regions, floor transitions, target
heights, sight-line obstruction and confusing illuminated objects. Record lux
at the flow sensor and target, lighting settings, photographs and flicker evidence
(a lux reading alone misses flicker). Preserve competition floor texture: no
calibration tape under the flow footprint. External measurement markers must not
silently improve the onboard scene. Target dimensions, surfaces, contact rules
and permission for operational external cameras remain subject to the actual
RM27 specification; label substitute targets as surrogates.

Use independent reference measurements. A surveyed translation rail suffices
for E1; calibrated overhead **and** side video at ≥60 fps can cover E4's small
working volume. Prefer 120 fps. Check against withheld known positions across
the volume, targeting ≤2 cm position error; record the achieved uncertainty.
Derive attitude only from a calibrated marker/pose method with checked angular
error. Synchronize at the start and end and estimate clock offset **and drift**.
Video frame counting alone has at least frame-time uncertainty. Reference data
are for scoring, never sent to the onboard estimator or guidance in this campaign.

For every run retain the raw `.BIN`, before/after `.param`, unedited reference
video/data, camera frames and timing when applicable, bench CSV and an event log.
Use [the run template](../configs/real_trial_template.json) and
[signal dictionary](real_logging.md). Retain failed starts, aborts and missing
data with reasons. Record actual rates/gaps, not just requested rates.

## E1 — Does MTF-02P work on the field we must fly over?

**Configuration:** props removed; installed FC/sensor orientation, mount,
electrical supply and harness unchanged. Put the vehicle on a non-obscuring
surveyed carriage, not in a hand covering the flow view. Warm up for 120 s and
record a further 60 s stationary IMU/flow/range baseline. This is a short bias
check, not a temperature model or Allan-variance identification.
Use the existing proven supply or supported USB power for props-off recording;
do not improvise a battery/regulator connection to start this session.

**Procedure:**

1. On F0, rotate about the sensor center through approximately ±15° roll and
   pitch, five cycles per axis. Record independent pose. Check flow/body/IMU
   axes, signs and rotational agreement before interpreting translation. This
   follows the props-off portion of [ArduPilot's flow setup procedure](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html).
2. For F0/F1/F2, hold level at **0.5, 1.0 and 1.5 m**, **10 s × three independently
   repositioned repeats** at each height: 27 holds, 270 s total. Measure distance
   from the sensor aperture. At each surface add one slow ±15° tilt to expose
   slant-range behavior; use a surveyed plane, not vertical height as slant truth.
3. At 1.0 m on each of F0/F1/F2, translate
   **2 m out and 2 m back**, at nominal **0.25 and 1.0 m/s**, body headings
   **0° and 90°**, three repetitions: 36 round trips. Hold stationary 3 s at
   each end. Measure actual motion rather than assuming commanded rail speed.
   Keep the weak surface under the entire sensor footprint where possible;
   otherwise mark exactly when the footprint straddles materials.

**Record:** `OF` quality, angular flow and body rates; raw `RFND` distance,
status/quality; `IMU` gyro/acceleration/temperature; reference pose, velocity,
sensor height and surface boundaries. Record UART configuration, sensor firmware,
mount yaw/lever arm and range settings. Capture the sensor's native UART stream
with a non-driving tap if available; FC logging may omit native samples. The
manufacturer specifies 50 Hz and lighting-dependent operation; these are
[published limits to check](https://micoair.com/optical_range_sensor_mtf-02p/), not achieved results.

**Analysis and decision:** fit axis alignment, scale, bias and delay using the
first two repetitions; test on the third. Use the actual flow projection and
gyro compensation, including lever-arm rotation. `OF.flowX/Y` are angular rates,
not world velocity. Fit range bias separately for surface, height and tilt.
Report velocity error, outbound/inbound displacement errors separately, peak
excursion error, loop closure, longest invalid streak, and quality-versus-error.
A closed round trip can cancel a scale error, so loop closure alone is inadequate;
high quality with wrong velocity is a dangerous failure.

Consistent scale/bias with small held-out residual supports calibration. Texture-
or light-dependent failures support a route restriction or another localization
source; a single scalar correction cannot repair them. If the weak-condition
traverse is unreliable, do not deliberately fly it in a flow-dependent mode.
Add height-dependent translation tests only if static/translation residuals show
an unexplained range interaction. This campaign does not establish performance
above 1 m/s or outside its height/light envelope.

**Simulation update:** replace the provisional ego-error abstraction with measured
flow/range errors, dropout runs and yaw/height dependence. Preserve structured
errors across time and material transitions. Check resulting navigation drift
against E4 reference, rather than equating sensor fit with an operating EKF.

## E2 — What visual information arrives in time to control?

**Configuration:** actual intended camera/lens and compute board, props off;
record exact models. If no camera is installed, use that assembly on a separate
jig first. Phone footage can document the experiment but cannot characterize the
onboard pipeline. No camera available means this evidence gap stays open.

Calibrate intrinsics/distortion and camera-to-body rotation. Use native lens FOV,
prefer **640×480 at 30 fps if actually supported**, and record readout/crop mode.
Compare **auto exposure** against **1 ms manual exposure** with a fixed recorded
gain selected on calibration clips. If 1 ms is unsupported or too dark, retain
that failure and measure the shortest usable supported exposure as a new condition.
Freeze white balance and detector thresholds after calibration.

Use measured benign fixtures representing **an illuminated circular target and
an armor-like panel**, with genuine field backgrounds and confusing lights.
Keep shape identity separate in analysis. The existing sphere-contour range
estimator is not a panel pose estimator. Compare centroid bearing first; compare
size-based range only for a valid geometric model, and planar pose only when
corners are actually observable.

**Procedure:**

1. First survey both fixtures at 4 m for 10 s each under F0 and each distinct
   lighting condition identified in the field survey. Use these calibration
   clips to select and freeze the limiting camera condition; it may differ
   from the weakest condition for flow. Under that lighting, record both fixtures at
   **2, 4 and 8 m**, 20 s at each range. Two exposure policies × two shapes ×
   three independent repetitions = **12 one-minute clips**. At 4 m include
   front-on and 45° oblique views within its 20 s segment. If the actual field
   maximum is below 8 m, retain 8 m as a separate stress condition, not a field
   requirement. Record six **30 s target-absent clutter clips**, three per
   exposure policy, to measure false detections/associations.
2. At 4 m, use a measured rotation jig with both **0° and +20° upward camera
   mounts**. For each shape/exposure/mount, record three **40 s** clips:
   stationary body pitch 0° then +20° nose-down; a −60°→+60°→−60° yaw cycle at
   30°/s, then at 90°/s; a **300 ms then 800 ms** opaque-screen occlusion during
   continued motion, allowing ≥3 s to reacquire after each. Record actual pose
   and occlusion timing. This is **24 clips**, about 16 minutes. No UAV flight
   or flying target is needed. Do not put an operator in a propeller arc.
3. Measure photon-to-result timing on the actual pipeline. An LED in the image,
   photodiode and logic analyzer establish optical transition time; toggle an
   output marker when the corresponding processed frame is published. Record
   **three blocks of 100 randomly spaced transitions**, first under normal
   logging load and again after **five minutes of representative perception/
   recording load**. Space transitions 0.1–0.3 s to avoid confusing successive
   events. Record capture metadata, dequeue, inference start/end and publish
   clocks. Report p50/p95/max and observed queue/drop behavior; 300 transitions
   do not justify a stable extreme-tail claim. Include serial/radio delivery
   using inert diagnostic packets only if that path is proposed for control.

The rotation sequence tests attitude-induced image motion and visibility. It
does not identify translational target dynamics, range-rate estimation or a
three-dimensional interception probability. Escalate to a measured moving
fixture/rail only if the first analysis cannot separate the remaining predictor
choices. Compare a different physical lens only if neither mount/exposure setting
meets the task envelope; digital resizing cannot create a wider optical FOV.

**Record:** original frames including misses; frame IDs, exposure/gain, FPS,
readout/crop, all pipeline timestamps, detections and rejected candidates,
reference target pose/dimensions, CPU/NPU load, temperature, supply voltage and
power. Log confidence/track age and stale-result rejection. If the camera has no
exposure timestamp, report a measured capture-age interval; do not rename dequeue
time as capture time. Verify clock uncertainty before interpreting a 10 ms effect.

**Analysis and decision:** calibrate on repetitions 1–2; score repetition 3
without changing thresholds. Report detection/false-association rate, bearing
error, range bias, capture-to-consumption age, complete loss durations, first
acquisition and correct reacquisition time. Include misses in denominators; also
report error conditional on detection. Replay immediate start, 500 ms fixed
hold and a preregistered readiness rule (three consecutive associated detections,
age <100 ms, prediction residual <2°). These are shadow decisions, not commands.
Report which readiness rules never become ready.

Test whether +20° mounting preserves the terminal view without losing launch/
near-ground views. If short exposure reduces range bias but loses detections,
choose using the combined failure rate. If bearing is stable while inferred
range changes materially with blur or panel angle, favor bearing-based terminal
correction and challenge lead guidance's range prior. If detection fails on
clutter, compare a learned detector only against the recorded classical baseline.
If an offboard link consumes the image-age budget, keep that path for coarse cues;
do not make terminal control depend on it. External operational perception also
requires rule permission and measured field line of sight.

**Simulation update:** use measured intrinsics/extrinsics and correlated error
tables conditioned on pixel size, angle, exposure, angular speed and illumination;
sample measured timing/dropout sequences. Preserve abstention, false association
and acquisition delays. The prior 96.2% geometric lead result and blurred-sphere
range bias must be retested on these models; neither is a real-image success rate.

## E3 — Which propeller buys useful authority at acceptable power?

**Configuration:** exact 9600KV motors, installed guards, weighed assembly,
unchanged ESC settings, and the **already rated battery cell count**. A=1.8-inch,
B=2.0-inch; record pitch, blade count, manufacturer and guard clearance. An
incompatible/unrated propeller is excluded with a reason, not flight tested.

Use a rated enclosed thrust stand with unobstructed inflow/outflow, calibrated
force and external voltage/current measurement. Secure by an engineered fixture;
never hand-hold, improvise a free tether, or arm a stabilizing controller against
a rigid clamp. Use a stand/motor-test controller verified with props removed,
an independent remote power cutoff and an operator outside containment. Record
stand geometry, tare, calibration, filtering, sample rates and physical bandwidth.

**Procedure:** test fresh packs (manufacturer-approved charge, rested 10 min) and
a mid-sortie state (30% nominal capacity removed by a suitable monitored
charger/discharger, within pack limits; rest 10 min). Record actual loaded voltage
throughout; these are two operating bands, not a complete battery model.

At each band run **A–B, B–A, A–B** matched pairs: three repetitions per prop,
**12 sequences total**. Match starting cell voltage within 0.02 V/cell and pack
temperature within 3°C; record/cross over pack IDs so one prop does not inherit
the better pack. If matching is impractical, retain measured voltage as a
covariate and label the comparison less decisive.

Each sequence is approximately **40 s**: tare; gradually locate fixed motor
commands producing total thrust **0.6, 1.0, 1.2 and 1.4 × measured weight**, dwell
3 s at each; then five **0.8↔1.2 × weight** step cycles with 2 s per plateau;
return to zero. Target thrust defines calibration points, not a closed-loop
force controller during the steps. Approach each new level slowly. If a point
exceeds a rating, temperature/current/voltage limit or stand capacity, stop there
and mark it censored. Cool to within 5°C of starting ambient between sequences.

**Record:** total thrust in N, motor command/channel, optical/ESC RPM if available,
external loaded V/I, FC battery telemetry, battery/motor-case/ESC temperatures,
ambient conditions and pack state. Aim for force ≥500 Hz and V/I ≥1 kHz for the
40 ms lag hypothesis, with verified instrument bandwidth and clock alignment.
A slow load cell still identifies steady thrust/power; it does **not** identify
a fast motor time constant. FC battery logging and PWM alone cannot fill this gap.

**Analysis and decision:** fit monotone command–thrust–voltage curves, hover
power, incremental power for thrust, sag and response delay/rise/fall times;
test the third paired repetition without refitting. Do not force a first-order
lag if residuals show overshoot, filtering or voltage dependence. Prefer the
propeller that meets the measured task authority at lower energy/heat and
acceptable E4 vibration. Differences within paired repeatability and instrument
uncertainty remain unresolved; add one matched pair at a time only for that tie.

**Simulation update:** replace nominal mass, thrust curve, motor lag and supply
scaling with measured intervals. Bench thrust is not a free-flight calibration;
E4 checks hover consistency. For a demonstrated safe ratio r=T/W, level-altitude
tilt is bounded by acos(1/r), and horizontal acceleration by
g·min(tan(tilt limit), sqrt(r²−1)), before drag and control reserve. The 1.4×
screen is below even the ideal 45° hover requirement (~1.414×). Passing it does
not validate Astra's nominal 2× or 3× authority. Establish a higher safe stand
envelope only after electrical/thermal margins are reviewed; do not extrapolate
the curve or perform maximum-throttle free flight to obtain it.

## E4 — Do the field and the flying airframe invalidate the bench results?

**Entry gate:** E1 passes axes/range checks for the flight surfaces; E3 supports
hover with reserve; configuration and manual recovery mode are already proven
by a qualified pilot. A first flow-Loiter commissioning flight is a separate
operation: ArduPilot specifies **15 m clear space around the vehicle** for that
test because bad flow can cause rapid acceleration. A 2 m test route is not
that clearance. Follow the [official commissioning procedure](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
if Loiter is unproven; do not bypass it to execute this profile indoors.

**Procedure A: paired flight, six sorties.** Use A–B, B–A, A–B, matched battery
states, unchanged payload and tune. Fly on F0 in the already proven mode. Each
**70 s** profile: takeoff/settle 10 s at nominal 1 m; hover 20 s; travel 1 m
out/back in X at 0.25 m/s (8 s), settle 4 s; repeat in Y (12 s); yaw
0→+30→−30→0° at ≤30°/s plus settle (6 s); land 10 s. Speeds are pilot targets,
not untested autonomous setpoints. The reference measures what actually happened.
Keep commanded maneuvers within 10° tilt and 0.5–1.5 m height.

**Procedure B: selected prop, three field routes.** Use only an E1-validated
transition between F0 and a representative field patch, with field target/clutter
visible to the camera. Each 70 s run: takeoff 10 s; hover 10 s; 2 m out/back at
0.25 m/s (16 s); pause 5 s; same at 0.5 m/s (8 s); pause 5 s; hover 6 s; land
10 s. Leave ≥1.5 m separation from any target fixture and all site-required
clearance from obstacles/containment. If flow-position flight is not commissioned,
use the proven pilot-controlled mode and label the result as sensing/dynamics
evidence, **not autonomous navigation validation**. Perception/tracking/guidance
run in logging-only shadow mode if installed, with no actuator connection.

**Record:** independent pose/video, `ATT`, `RATE`, PID terms, `RCOU`, `RCIN`,
`CTUN`, IMU/vibration, battery/power, raw flow/range, all EKF lanes/innovations/
health, mode changes and errors. Add one-IMU pre/post-filter batch logging to
paired runs only after checking SD/CPU headroom props off. With a camera, record
the same E2 frame/timing stream on its actual flying mount. Any unpowered ballast
test assesses mass/COM only, not compute power or vibration-corrupted imagery.

**Analysis and decision:** align reference and estimator once during the initial
stationary segment; do not repeatedly align away drift. Report endpoint and
time-series position/velocity/yaw errors, reset events, sensor validity streaks,
hover W, Wh per profile, sag, vibration/clipping, tracking error and latency.
Fit small-signal desired-to-achieved attitude response from `ATT`/`RATE`. This is
closed-loop airframe response, not a separate measurement of motor lag or inertia.
Use E3 lag to avoid counting motor response twice in the surrogate.

If 2.0-inch gains bench efficiency but degrades vibration, flow or bearing error,
the extra thrust has not earned selection. Good props-off sensing with bad
powered sensing points to mounting/vibration/power coupling before an EKF or
detector redesign. Persistent field drift with sound raw flow requires checking
yaw, timing, fusion and height sources. A camera view lost during this gentle
profile invalidates a confident high-speed transfer claim immediately.

**Simulation update:** calibrate on the first two paired flights/routes; reserve
the third for prediction checks. Fit only the measured low-speed envelope. Add
measured pose/exposure synchronization error, attitude response and coupled
sensor failures. No claim about high-speed drag, maximum tilt, contact survival,
long-duration gyro bias or dynamic-target prediction follows from these flights.

## Common safety and decision boundaries

- Qualified pilot plus observer, approved netted test area/clear field slot,
  no other teams operating in the volume, substantial guards and inspected props.
  No people/animals/vehicles as targets. Bench and flight fixtures are benign.
- Confirm a pilot recovery mode that does not need optical flow, RC override and
  the actual battery/RC/EKF failsafes. Preserve arming checks; this protocol is
  not a parameter file. Keep automatic guidance disconnected during this campaign.
- Abort and land for unexpected acceleration, oscillation, height excursions,
  sensor-health warnings, loss of pilot/reference view or abnormal propulsion.
  Do not deliberately inject flow, ToF, camera or communication failures in air.
  Use props-off occlusion, recordings and replay for the initial fault checks.
- Stop powered tests at the **lower** of manufacturer/operator limits and these
  conservative campaign surface-temperature stops: pack 40°C, motor case 50°C.
  These are experiment stops, not certified component limits; hidden winding/ESC
  temperatures can be higher. Use approved loaded-voltage/current limits and
  land with reserve; never explore the low-voltage cutoff in flight.
- Review each flight before the next. In-air accelerometer clipping, sustained
  VIBE above 30 m/s², logging loss, rail faults, prop/guard contact or worsening
  tracking stops escalation. Exclude touchdown transients only with marked
  segments. The vibration threshold is an [ArduPilot diagnostic guideline](https://ardupilot.org/copter/docs/common-measuring-vibration.html),
  not proof that values below it make this airframe safe at high speed.

## How the four experiments change the architecture decision

Freeze the present simulation results. Create a separately versioned real-data
calibration with raw-data hashes, fit uncertainty, measured operating envelope
and held-out residuals. Reuse paired scenario seeds and all three guidance
baselines with measured visibility, timing, range, ego-error and propulsion
models. Preserve the original failures and thresholds. Change target geometry
only in a new explicitly field-representative benchmark, not to rescue a score.

Use actual camera acquisition distance and a route/search tolerance to decide
how much localization accuracy is needed. For illustration, allocate half the
visible lateral margin at acquisition to navigation uncertainty: require
e_nav,95 + R·tan(e_yaw,95) to stay below that allocation. Compute the margin from
the measured usable FOV, target extent, obstructions and expected route bearing;
a target near the image edge has little margin. This conservative budget is a
screen, not a probabilistic guarantee; test joint errors in replay. If the budget
fails, compare shorter dead-reckoning legs or permitted field references before
committing to full VIO. A 5 cm terminal lateral-error allocation, if adopted for
research, corresponds to about 0.72° at 4 m; it is **not** an asserted RM27 rule.

The calibration report must answer:

1. Which propeller/payload combination has measured hover reserve and useful
   power/thermal margins? Which unmeasured high-thrust assumptions remain?
2. Can flow/ToF bridge the measured field route into the camera's acquisition
   region? Does a separate yaw reference or global reset become necessary?
3. Is bearing adequate while range is biased? Does measured latency/visibility
   preserve lead's advantage over compensated pursuit?
4. Does a local camera meet the image-age budget? Would an external camera
   improve acquisition enough to justify its measured link delay and rules burden?

Report uncertainty over independent runs/clips, not thousands of correlated
frames treated as independent trials. If held-out data invalidate the error
model, expand the specific failed condition before rerunning large campaigns.
Offline shadow guidance cannot prove closed-loop contact: its next commands
would change future images. Contact and moving-fixture flight tests follow only
after these measurements support a bounded, independently reviewed envelope.
