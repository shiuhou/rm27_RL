# PX4 v1.17.0 migration status

Updated 2026-10-11. Worktree: `/home/shiuhou/Projects/rm27_drones_px4`;
branch: `codex/px4-v1.17-migration`. The full migration is **in progress**.
Hardware operations remain outside this software task.

## Fixed inputs and physical scope

PX4 v1.17.0 is pinned to `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
The SITL binary SHA-256 is
`5646c151d002f988d73b533d6377685fa65ff5949b62f3cc869704075ec47c54`.
The target profile is `configs/dynamics_150g.json`. User-confirmed total mass
includes the 2S 450 mAh battery, M3C, OS04A10 camera and wiring. Geometry,
inertia, thrust, motor lag and yaw torque remain engineering estimates.
The original 180 g configuration and G1 ArduPilot evidence retain their scope.
The new dedicated downward return camera is estimated by the user at 2-3 g.
Its mass is not yet included in the 150 g simulation model; total real takeoff
mass after installation has not been measured. No flight controller parameters
or hardware have been changed.

The simulator uses HIL_SENSOR and HIL_ACTUATOR_CONTROLS at 250 Hz, with 2 ms
physical steps for dock-only flights and 1 ms for route/contact flights.
Downward range uses `mujoco.mj_ray()` against compiled collision geometry;
vehicle geometry is excluded from the ray. Numerical flow uses a simulated
compass, range, gyro and velocity. It is not image-derived optical flow or
measured MTF-02P behavior. `flow` assumes a 1 cm minimum range; `mtf` uses the
manufacturer's 8 cm minimum and attitude/thrust takeoff before flow handover.

## Inspected results

| Area | Artifact | Result and scope |
| --- | --- | --- |
| Build | `px4-build-08/summary.json` | Passed, pinned PX4 binary |
| S0 | `px4-s0-batch-03/summary.json` | 10/10 independent diagnostic launches, synthetic external vision and truth-assisted precision guidance |
| Exact physics replay | `px4-replay-02/summary.json` | Passed, zero error over 19,684 recorded input frames; no independent-flight determinism claim |
| EKF telemetry micro-flight | `px4-micro-ekf-05/summary.json` | Passed, synthetic EV fused by EKF2; telemetry controls task phases |
| 150 g scene | `px4-base-target-150g-02/target.json` | Compiled full-field scene; builder emits `target.json`, not `summary.json` |
| Corridor | `px4-corridor-audit-02/summary.json` | Passed geometry audit for the selected route |
| Old route batch | `px4-s2-batch-01/summary.json` | Return/disarm 20/20, but strict recovery 0/20 in `px4-s2-eval-batch-04` |
| Separation controller, 2 ms | `px4-s2-eval-batch-05/summary.json` | Strict recovery 20/20; max entry 0.468 s, held confirmation 1.468 s, no repeated armor collisions |
| Current-aircraft contact convergence | `px4-contact-convergence-03/summary.json` | Passed at native 1 ms: impulse difference 3.38%, peak difference 2.10% versus 0.5 ms. 2 ms fails peak convergence (11.55%) |
| Current-aircraft contact classification | `px4-contact-classification-02/summary.json` | Passed front/back/grazing/non-target probes; motors off, zero gravity; full local forbidden-region audit remains pending |
| Separation controller, 1 ms | `px4-s2-eval-batch-06/summary.json` | Strict PX4 S2 recovery **20/20**; every hold sample and subsequent contact checked, no secondary collisions, all returned and auto-disarmed |
| Numerical flow micro-flight | `px4-flow-micro-03/summary.json` | Passed without GPS/ODOMETRY: 10 s hover maximum XY 5.24 mm, height 46.70 mm; independent physical scorer |
| 8 cm flow minimum micro-flight | `px4-mtf-micro-06/summary.json` | Passed return/rest/auto-disarm: hover maximum XY 5.53 mm, height 24.85 mm, physical resting XY 2.26 mm. PX4 failsafe occurred after near-ground flow loss; further qualification required |
| Rate and attitude response | `px4-response-{rate,attitude}-{roll,pitch,yaw}-*/summary.json` | Six passing development axis probes; exact selected runs below; airborne response fixture, not landing/contact qualification |
| Recorded images/IMU | `px4-current-flight-images-02/summary.json` | 2,201 images, 27,501 IMU samples, 529 safe target frames; images did not control the recorded flight |
| Native publication instrumentation | `openvins-native-evidence-build-01/summary.json` | Passed separate pinned build; original research source and binary remain read-only |
| Native recorded VIO | `px4-current-flight-openvins-06/summary.json` | Execution passed; observed initialization, features, covariance, source/publication clocks and epochs |
| Recorded VIO development score | `px4-current-flight-vio-score-05/summary.json` | Passed development thresholds: XY p95 0.19599 m, height p95 0.01623 m, coverage 87.18%; one initial SE3 registration, scale fixed to 1 |
| Root tests | `px4-root-tests-12.log` | 396 passed, 2 ROS-only skips; includes close named-camera projection and visual-return admission contracts |
| Four slot AprilTags | `px4-dock-tags-05/summary.json` | Airborne visual audit passes for IDs 21-24; 80 mm codes on 96 mm panels replace green slot visuals, without changing dock collisions. Assigned tag visible at 0.6, 0.4, 0.3, 0.2 and 0.12 m above slot; static image/PX4-attitude XY error below 0.6 mm, not flight accuracy |
| Live downward return camera | `px4-live-vio-tags-04/summary.json` | Passed short current-flight VIO/image return, normal LAND and automatic disarm; independent resting XY error 0.0724 mm. Fresh assigned-tag admission and landing commitment enforced. A development run does not establish S4 completion |
| Near-ground LAND diagnosis | `px4-live-vio-tags-03-ulog/summary.json` | Read-only ULog shows EKF2 actual EV position/velocity fusion and land-detector states; original failed attempts remain in `px4-live-vio-tags-01` and `-02` |

Artifact paths in this table are relative to `artifacts/`.
The response runs selected on development input are `rate-roll-02`,
`rate-pitch-01`, `rate-yaw-05`, `attitude-roll-02`, `attitude-pitch-01` and
`attitude-yaw-03`, each prefixed `px4-response-`. Development yaw tuning lives
only in `configs/px4/response.json`, including the 30 Hz yaw torque filter.
No response gain is automatically transferred into mission or hardware files.
The custom PX4 rotor allocator matches physical root actuator order and yaw
signs; the ArduPilot PWM permutation is not used.

The V2 recovery scorer examines every sample in its continuous hold and all
subsequent contacts, retaining the original 30 ms separation and recovery
limits. Earlier evaluator outputs `px4-s2-eval-batch-01` through `-03` are
superseded: they masked repeat collisions or counted unsafe hold samples.
The frozen separation controller is `configs/px4_corridor_mission_s2.json`:
0.03 m/s approach, 0.025 m retreat bias, 0.12 m/s retreat reference and
4 m/s² outward acceleration for 0.12 s dispatched on the impact tick. The
strict batch is `artifacts/px4-s2-eval-batch-06`; its inputs are
`artifacts/px4-s2-batch-03` (20/20), all at the 150 g target model and 1 ms
route physics timestep.

Failed attempts remain available: `px4-s0-batch-02` (9/10), early Corridor
tracking/deadline failures, `px4-mtf-micro-01` through `-05`, response yaw
failures, the superseded camera projection dataset `images-01`, and VIO
scores `vio-score-02` and `-04`. Conservative VIO filter noise margins are
explicit development tuning, not additional injected noise or real calibration.

## Localization and task boundary

Non-diagnostic micro-flight phases use fresh PX4 telemetry, range and landed
state. Physics truth supplies simulated sensors and independent scoring;
physical contacts are excluded from those mission inputs. The strict physical
resting gate remains 7 mm XY and 5 mm height. In the `mtf` profile, horizontal
position is committed while flow is valid before normal LAND; below its minimum,
range and landed telemetry establish rest and the independent scorer assesses
actual slot accuracy. The no-aiding timeout is explicitly 10 s in that profile.
This demonstration does not establish every localization-loss response.

Native OpenVINS has a complete observed publication adapter, but its
`flight_qualified` flag remains false. Recorded run 06 contains 824 TRACKING,
141 INITIALIZING and 136 LOST publications with no contract parse failures;
p95 native camera processing is 21.09 ms and maximum 39.66 ms. Receipt of a ROS
pose alone does not establish tracking. Native orientation covariance is
converted from IMU error axes into the parent frame before NED registration.
Source simulation and native monotonic publication clocks remain distinct.

The image detector sees only pixels and their time. It associates provisional
blue armor with the green marker and rejects ambiguous/stale observations.
Camera mount/FOV and pinhole calibration are simulation assumptions, not
OS04A10 calibration. EuRoC and recorded current-flight datasets are offline
regression inputs, not a live flight or M3C runtime qualification.

The dedicated downward camera is centered in body X/Y, 25 mm below the body
origin, at a simulated 640x480/70-degree FOV and 20 Hz. It is only active for
RETURN, REACQUIRE, DESCEND, ALIGN and FINAL_DESCEND. AprilTag 36h11 IDs 21-24 identify
slots 1-4; the current assigned slot is 1. The black code is 80 mm wide and
the white panel is 96 mm, within the former 110 mm green site. Return guidance
accepts only the assigned ID, fresh image and PX4 attitude. Repeated cached
images cannot extend freshness. Loss before landing commitment triggers a
bounded reacquisition climb while navigation is valid; final LAND requires a
fresh aligned tag, then freezes that registration for near-ground handover.
Reacquisition returns to the reviewed 0.6 m observation altitude, requires
0.5 s stable fresh identification, then resumes descent. Loss before the final
LAND request also enters this path. Reacquisition exhaustion is a failed run.
A bounded permanent-camera-loss policy now passes a separate short-return
5/5 termination batch (0/5 task successes); complete-route and navigation-loss
handling remain separate.

Frozen independent short-route batches now pass: sensor-only MTF 20/20
(`px4-s3-mtf-batch-02`, maximum resting XY 4.154 mm, p95 4.098 mm) and
current-flight OpenVINS plus downward AprilTag return 20/20
(`px4-vio-return-batch-01`, maximum 3.743 mm, p95 2.978 mm). All attempts
complete normal LAND and automatic disarm without an unexpected PX4 failsafe.
These errors come from the independent physical scorer. The latter establishes
image-guided short return, not target approach/contact or S4/S5 acceptance.
The recorded batch manifests freeze the exact pre-fault-study implementation.

The subsequent frozen downward-camera fault matrix passes 15/15
(`px4-return-fault-batch-01`): one-second pixel blackout, repeated old image
with its original acquisition time, and pixel replacement of ID21 by ID22,
five independent starts each. Each run pauses descent, reacquires on a new
assigned image and completes normal LAND/auto-disarm without a failsafe.
Maximum physical resting XY is 2.836 mm (p95 2.734 mm). Inputs, rendered images,
source/receipt times and pixel hashes are retained. This qualifies three
temporary return-camera faults only; the full S3/S4 fault matrix remains open.

The earlier report of missing tags at 0.2/0.12 m was a rendering bug: named
cameras inherited the field's 20 cm near clipping plane. The corrected renderer
uses a 2 mm near plane with projection preserved. `px4-dock-tags-04` visibility
limits are superseded, and existing failed flight records are retained.
Near-ground loss of flow/VIO left the position controller reducing thrust too
slowly to satisfy the land detector before the 10 s no-aiding failsafe. Development
profiles `mtf` and `vio` now use `MPC_Z_VEL_I_ACC=4` (previous default 2).
This is simulation tuning; remaining gates and timeout are unchanged.

## Remaining implementation and acceptance

- S1: complete local forbidden-region audit. Current-aircraft classification
  and converged front impact evidence are present; the remaining prerequisite
  cannot be inferred from these probes. The new V4 recorded-pose capsule audit
  fails: COM alignment lets guards cross visual tower surfaces outside the
  armor patch. Guard-tip alignment and its recovery-volume audit are in
  development; full S1 remains open.
- S2: the fixed low-speed 1 ms recovery batch passes 20/20 under the strict
  V2 scorer. S1's complete forbidden-region audit remains a separate partial
  prerequisite; this result does not extend the claim to other contact speeds.
- S3: nominal numerical flow/MTF 20/20 passes; the pre-registered fault matrix
  remains unfinished.
- P4: Teacher/Student attitude/rate and synthetic-navigation studies now have
  a PX4 backend. Hover passes, but frozen Teacher contact holdout v1 fails
  (nominal 0/8; overall 1/14 task successes and one safe abort).
  [Research evidence](PX4_CONTACT_RESEARCH.md) preserves the failing attempts.
  Fresh conservative hybrid-reference fitting is complete, but its separate
  holdout fails 9/14 (nominal 7/8). Current paired Teacher development also
  fails overall: PX4 rate 3/3, the other three backend/interface groups 0/3.
  Aggressive strategy qualification remains unfinished; the complete image
  mission uses a separate low-speed controller.
- P5/S4/S5: current-flight native VIO/ODOMETRY, pixel-derived metric armor pose,
  causal IMU contact inference and dedicated downward AprilTag return are now
  connected. Frozen nominal V1 is complete and fails **11/20**, including six
  recovery failures, one reacquisition exhaustion and two invalid LAND alignments.
  Guard-tip/image-height nominal V2 is complete and fails **19/20**: attempt 15
  has a secondary contact after 20 ms impact inference. Return/rest/disarm
  completes 20/20 (maximum resting XY 3.616 mm), but the whole batch remains
  failed. Startup-configured 250 Hz observer development passes two fresh
  physical tasks, with 5/7 ms inference and 3.593/1.480 mm resting XY. Primary
  camera source-time laundering is repaired and regression tested. A new
  reserved nominal V3 batch now passes **20/20**, resumed without repeating
  completed flights after its controller was interrupted. The bounded V4
  continuous corridor and all20 native-step memberships pass. Maximum
  independent resting XY is3.482 mm, p95 3.469 mm. The50-condition V2 batch
  is running but has an unhandled takeoff
  contact failure, so it cannot pass. Complete-route downward faults continue.
  See [complete image mission](PX4_IMAGE_MISSION.md).
  Stochastic noise development also exposes native covariance failures. An
  isolated Joseph variant completes one formerly failing recorded segment,
  but a fresh flight still fails; [numerical diagnosis](PX4_NATIVE_NUMERICS.md)
  records both and continued physical termination development.
  The bounded initial dock VIO-disconnect matrix separately passes5/5
  physical terminations,0/5 tasks, with actual range-height fusion and no
  unexpected failsafe. Full-route interruption and navigation reset remain open.
- P6: ULog intake, finite logging session and M3C single-UART tools are present;
  six fake-wire root tests pass. See [software report](PX4_M3C_SOFTWARE.md).
  Full saved-SITL-log wire replay and loss rejection now pass; end-to-end
  live device capture and device runtime qualification remain pending.
  UART 115200 and ~50 Hz HIGHRES_IMU do not qualify 200 Hz raw IMU or real VIO.
- Default backend remains explicit during migration. Complete mission and
  hardware qualification are separate decisions; no hardware is modified.

## Supported commands

See [START_HERE.md](../START_HERE.md) for launch and viewer commands and
[perception adapters](../rm27/perception/sitl/README.md) for recorded/native VIO.
Each build/run requires a fresh output directory. Inspect actual summaries;
process exit and artifact names are not acceptance evidence.
