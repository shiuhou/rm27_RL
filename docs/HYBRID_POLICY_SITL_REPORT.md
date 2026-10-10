# RM27 Phase 4 — actual ArduPilot integration and estimator gap

2026-09-23. Branch: `astra/research`; root HEAD: `da396bef63ce51476a71af4b9b70abdda03eed0b`.

## Result and evidence boundary

This phase now has **actual ArduCopter firmware in the closed loop**, not the previous rate-P proxy. In one nominal balanced whole-maneuver run, attitude + thrust with firmware input shaping disabled achieves bottom-bumper first contact, separation and recovery. Contact error is **7.47 mm**, region margin **23.03 mm**, recovery entry **0.493 s after contact**. The same interface using real SITL EKF telemetry hits the rim: error **82.72 mm**, margin **−27.58 mm**. It separates and recovers, but the task correctly FAILS.

This is a controlled nominal interface diagnosis, **not a reliability benchmark or hardware validation**. No RL, retraining, reference redesign, firmware patch or hardware parameter write was performed. The intended GPS-free onboard estimator remains unvalidated. The run begins airborne above the dock; takeoff, return and landing were not added to this SITL experiment.

## What was actually connected

The retained student is a learned fixed B-spline reference plus model feedback, causal contact switching and an actuator observer. It is not an additive neural motor residual. The new chain is:

`registered static target + timestamped AP EKF telemetry → existing DeploymentPolicy → desired attitude/rate + physical collective force → MAVLink SET_ATTITUDE_TARGET → real Guided/attitude/PID/mixer → PWM → current MuJoCo motors/6-DoF/physical armor → synthetic SITL sensors → real EKF`.

Two diagnostic inputs remain explicit: `--observation truth` supplies privileged ego state; `--observation telemetry` supplies only decoded firmware ego telemetry to the actor after initialization. Both still use the nominal target map and nominal vehicle parameters. There is no visual target estimate, dynamic target update, sensor calibration or real onboard navigation source.

The isolated process is the existing local `rm27/autopilot/vendor/ardupilot/build/sitl/bin/arducopter`, reporting ArduCopter 4.8.0-dev (96f98857), SHA-256:

`da030d364cf22c34403f7707d1a3d755263fd81dd59a9007b31e75538c0a9473`.

The vendor tree was already dirty. The executable was not rebuilt; its hash and received parameters, rather than an assumption of a pristine source build, identify this evidence. No claim that the current dirty source is byte-for-byte the binary's build input.

The existing root bridge exchanges servo packets at 500 Hz; the plant advances two 1 ms steps per packet. The verified firmware scheduler is 250 Hz. Policy packets average 200 Hz with 4/6 ms send intervals on the 500 Hz bridge clock. AP motor order maps to research motors [2,0,3,1]. Field/dock/target composition comes from existing `teacher.prepare`; field/base visual geometry is not newly certified as collision geometry.

An explicitly recorded initialization fixture holds the airborne state while the EKF starts, the vehicle arms and motors spool. It releases at about 47.062 s simulation time, with no later teleport/reset. The launch binds only localhost, has no remote/hardware endpoint option, and stores the firmware parameter store and DataFlash logs in its new artifact directory.

## Interface result: Guided rate is not a bare rate controller

Current `ArduCopter/mode_guided.cpp` routes a zero-quaternion request to `input_rate_bf_roll_pitch_yaw_rads`; that integrates an attitude target and invokes the attitude controller. It does not simply replace the old research rate-P inner loop. The inherited rate interface has strong yaw oscillation even in hover.

With a quaternion request, `AC_AttitudeControl::input_quaternion` also shapes attitude/rate targets when `ATC_RATE_FF_ENAB=1`. In this firmware source, setting it to 0 directly assigns desired quaternion and angular-velocity targets before the retained quaternion controller. This is not disabling stabilization. The experiment changes only that parameter relative to the shaped attitude case; the reference and inherited inner PID remain unchanged. The actual received value is checked before release.

`GUID_OPTIONS=8` selects thrust semantics; `ATC_ANGLE_BOOST=0` avoids a second tilt compensation. Throttle is computed from the nominal simulation PWM-force curve with EXPO=0 and spin range .15–.95. The armed spin floor means zero normalized thrust is not zero rotor force. This is **not a measured hardware thrust calibration**. The simulation-only parameter files must not be copied to a real controller.

## Closed-loop comparison

All maneuver rows use the same balanced reference, nominal plant, initial fixture and nine-second flight budget. Each row is one run, not a success-rate estimate. There was no final independent holdout after selecting the interface.

| Actual AP interface | Actor ego input | Contact error | Margin | Contact attitude error | Recovery entry after contact | Joint task |
|---|---|---:|---:|---:|---:|---|
| Guided rate + thrust | truth | 82.62 mm | −24.40 mm | 9.80° | timeout | FAIL |
| shaped attitude + thrust | truth | 126.91 mm | −99.04 mm | 20.09° | timeout | FAIL |
| shaped attitude + thrust | EKF telemetry | 141.23 mm | −112.60 mm | 16.40° | timeout | FAIL |
| unshaped attitude + thrust | truth | **7.47 mm** | **23.03 mm** | **0.52°** | **0.493 s** | **PASS** |
| unshaped attitude + thrust | EKF telemetry | 82.72 mm | −27.58 mm | 1.66° | 0.681 s | FAIL: rim first contact |

The other rate/telemetry run exceeded the evaluation envelope at 6.132 s. Its task failed; the detailed unchanged contact evaluator was not run on that interrupted recording before execution became unavailable.

Successful truth run: contact 4.485 s, normal velocity 0.654 m/s, contact body rate 0.0874 rad/s, post-contact maximum rate 4.900 rad/s, max flight speed 6.129 m/s, confirmed recovery at 5.978 s (includes dwell), altitude loss 1.41 mm. It has no secondary contact. Recorded rotor commands do not hit the plant force rails; this does not establish absence of internal AP mixer/PID limiting.

Hover is a useful negative control. Rate/telemetry hover ended only 28.4 mm from its start but had maximum body rate 6.063 rad/s and settled (>2 s) RMS rate **3.204 rad/s**. Attitude/telemetry hover had maximum rate 0.134 and RMS **0.0462 rad/s**. A later diagnostic gate requires RMS <0.3 rad/s; it rejects the first hover despite its old completion-only PASS. This gate was added after seeing the oscillation, not preregistered.

## Estimation and timing: the remaining error is not only message delay

`TelemetryEstimator` accepts `LOCAL_POSITION_NED` and `ATTITUDE_QUATERNION`, transforms NED/FRD to NWU/FLU, aligns attitude to the position publication timestamp using received samples, and publishes each position/velocity sample once. Faster IMU/attitude packets cannot falsely freshen held velocity. A fixed boot-clock offset is calibrated before release; one-time local/world registration is an explicit task input. There is no ongoing truth correction.

It rejects malformed, nonfinite, duplicate/backward and stale samples; attitude extrapolation is bounded to 25 ms. It has no timestamp-wrap/reboot recovery. The nominal velocity covariance is a configured assumption, not a calibrated EKF uncertainty estimate. The downstream actor predicts at most 35 ms even though the transport contract accepts up to 100 ms; maximum observed age was 38 ms, so a small residual age is deliberately unpredicted. A real integration still needs tested stale-data/failsafe behavior.

In the unshaped EKF run, actual publication intervals were:

| Stream | Messages in 9 s | Min / median / max source interval |
|---|---:|---|
| LOCAL_POSITION_NED | 450 | 16 / 20 / 23 ms |
| ATTITUDE_QUATERNION | 1,801 | 3 / 4 / 8 ms |

`time_boot_ms` is the published estimate/message clock, **not the raw GPS/IMU acquisition time**. The fixed minimum receipt-minus-boot alignment does not measure exact one-way network latency. The following offline diagnostic interpolates recorded truth at that publication clock; it cannot undo an internal EKF lag.

Using 449 distinct valid estimate publications, EKF position RMSE is **89.14 mm against current truth** and **71.92 mm against timestamp-aligned truth**. At physical contact, the 30 ms-old estimate differs from current truth by [77.56, −27.49, −12.22] mm. Even after timestamp alignment, the error is [58.54, −26.72, −26.75] mm. Timestamp propagation alone therefore cannot be assumed to remove the contact miss. These are diagnostic correlations; they do not identify each EKF noise/bias/lag contribution causally.

The firmware logs explicitly say **EKF3 is using GPS**. These synthetic SITL GPS/IMU measurements run through a real EKF, but GPS is not the intended RM27 hardware navigation input. This run is an interface diagnostic, not accepted GPS-free mission validation. A synthetic external-navigation producer feeding the actual EKF was considered but not implemented or tested. Actual optical flow/range, VIO, target perception, dropout campaigns and impact-estimator validation remain open.

## Gates and preserved evidence

Each new maneuver now requires both runner completion and the unchanged contact/separation/recovery evaluator to pass. The evaluator replays recorded firmware motor inputs on the saved MJB and initial actuator state; all five evaluated flight comparisons have maximum state mismatch **0**. This establishes that the contact metrics belong to those recorded inputs. It is not a second independent firmware flight or a guarantee of asynchronous repeatability.

Older raw summaries retain their original completion-only `passed` flags. Do not read those as task success. The adjudicated table is in [phase4-audit-02/summary.json](../artifacts/phase4-audit-02/summary.json). Its own `passed` means offline audit completion; individual `task_passed` fields distinguish flights. Unadjudicated interrupted runs remain explicitly marked.

The evaluator currently inherits a legacy `HEADLESS_MUJOCO_CLOSED_LOOP_MOTOR_CONTROL_TRUTH_TEACHER` mode label and shadow-controller reference/adaptation fields. Its `scope`, original source and replay checks identify the real mode: recorded-SITL-motor-input evaluation. Shadow `estimated_specific_thrust_ratio`, phase/reference diagnostics are **not measurements of the actual actor**; use original `student.npz` for that. These legacy fields were not used to infer the estimator result above.

Artifacts (all preserved):

- `phase4-sitl-hover-01`: socket permission failure, no flight.
- `phase4-sitl-hover-02`, `phase4-sitl-hover-attitude-01`: the two actual hover tests.
- `phase4-sitl-maneuver-truth-01`, `phase4-sitl-maneuver-telemetry-01`: rate flights.
- `phase4-sitl-attitude-truth-01`, `phase4-sitl-attitude-telemetry-01`: shaped attitude flights.
- `phase4-eval-rate-truth-01`, `phase4-eval-attitude-truth-01`, `phase4-eval-attitude-telemetry-01`: their contact adjudications.
- `phase4-unshaped-truth-01`, `phase4-unshaped-telemetry-01`: parameter intervention, including integrated `evaluation/` and viewer-compatible `evaluation/flight/`.
- `phase4-audit-01`: initial strict JSON audit failure caused by optional MAVLink NaN fields; retained.
- `phase4-audit-02`: completed timing/result audit, input hashes and exact JS source. Optional bare nonfinite tokens are normalized only in the offline parser; original JSONL remains unchanged.
- `phase4-checks-01`: actual test results and source hashes.

## Files, checks and provenance

New root package `rm27/experiments/aggressive_contact/flight_control/` contains the current-plant adapter, timestamped telemetry estimator, isolated SITL runner, common-evaluator replay adapter and offline JSONL audit. New configs are `configs/contact_phase4/sitl.parm` and `attitude_unshaped.parm`. New tests are `tests/test_contact_flight_control.py`. The evidence snapshot includes phase4 configs/tests. README and START_HERE link this phase and the playback commands.

Read/reused sources include README, START_HERE, INTEGRATION, REAL_HARDWARE, prior teacher/distillation/Phase3 reports and artifacts; `distillation/deployment.py`, `policy.py`, `teacher_interface.py`, `teacher.py`, `experiment.py`, `maneuver.py`, `viewing.py`; root bridge/frames/launch and micro-flight parameters; current AP Guided, MAVLink, attitude-control and mixer sources. Original provenance records root status, worktrees, source/config hashes, binary hash, launch arguments and received parameters. No existing dirty nested checkout was cleaned or rewritten. Work remains in the current root worktree; no commit was created.

Actually run in this continuation:

~~~bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_contact_flight_control.py -q
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation truth --action-layer attitude --parameters configs/contact_phase4/attitude_unshaped.parm --instance 45 --duration 9 --output artifacts/phase4-unshaped-truth-01
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation telemetry --action-layer attitude --parameters configs/contact_phase4/attitude_unshaped.parm --instance 46 --duration 9 --output artifacts/phase4-unshaped-telemetry-01
~~~

**6 Python tests passed (0.51 s)**: frame/registration, asynchronous attitude alignment, held-sample timestamps, stale/reordered/duplicate data, malformed-packet clock poisoning, extrapolation bounds and simulation thrust mapping. Six additional offline JS assertions passed for interpolation and artifact adjudication. The prior Phase3 report records 91 root tests; this continuation did **not** complete a fresh whole-root run, and does not claim 97 passing tests.

Both launched SITL experiments completed before the execution environment changed. Subsequent terminal and patch-helper invocations fail even for `pwd` with `error building bubblewrap command: mountinfo path is not absolute`. File-access tools could still read existing evidence and write the offline audit/report within the repository. No alternate subprocess launcher was used. This blocked additional firmware experiments, a final Python rerun, fresh rendering and external-navigation implementation. Phase4 is therefore a substantive, runnable integration slice, **not completion of realistic-estimator sim-to-real validation**.

## Reproduce and watch

Use a new output directory. Existing research dependencies, local field assets, the phase3 bundle and local SITL binary are required; this is not a fresh-clone-only artifact. Exact source snapshots accompany each historical run. Current malformed/duplicate-telemetry rejection was unit-tested after the two last processes launched; it did not alter those archived process snapshots.

Run the successful privileged diagnostic on a machine where local SITL sockets work:

~~~bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation truth --action-layer attitude --parameters configs/contact_phase4/attitude_unshaped.parm --instance 45 --duration 9 --output artifacts/phase4-truth-new
~~~

Replace `truth` with `telemetry`, use another output directory, and expect the documented nominal rim failure rather than claiming successful deployment. Exit status is nonzero for failed task evaluation.

Watch the already recorded successful firmware flight in MuJoCo:

~~~bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase4-unshaped-truth-01/evaluation/flight --speed 1 --loop
~~~

The viewer displays the state-matched motor replay: Space pauses, V switches overview/follow. It is not a new flight. The files and viewer loader contract were inspected; this continuation did not open a GUI. To watch the failed EKF flight, substitute `phase4-unshaped-telemetry-01`.

Reproduce the offline audit using Node (no firmware launch):

~~~bash
node --input-type=module -e 'import {audit} from "./rm27/experiments/aggressive_contact/flight_control/audit.mjs"; await audit(".", "artifacts/phase4-audit-new");'
~~~

The same exported function was actually executed via the session's Node runtime. The shell form above could not be launched after the terminal fault.

## What changed our conclusion

Stable position alone hid an unstable control interface. Merely making a MAVLink packet syntactically compatible did not make it dynamically equivalent to the research inner loop. A one-parameter input-shaping intervention recovered the nominal maneuver under privileged state, without enlarging the student or changing its reference. This supports using attitude + thrust for the next interface study; one nominal success does not certify robustness or real gains.

With EKF input, good contact attitude and successful recovery can coexist with wrong first contact. The estimator error has a substantial component even after publication-time alignment. The next most valuable experiment is a **GPS-free external-navigation measurement source fused by the actual AP EKF**, with explicit latency/noise and target-relative accuracy, compared against the same frozen interface/reference. That will separate localization quality from input shaping and expose whether the current actuator observer/contact detector remain valid under the real inner loop. Residual RL is not justified by the present evidence.
