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
| Separation controller, 1 ms | `px4-s2-1ms-score-01/summary.json` | Development pass: entry 0.412 s, held confirmation 1.412 s; independent 20-run batch `px4-s2-batch-03` pending |
| Numerical flow micro-flight | `px4-flow-micro-03/summary.json` | Passed without GPS/ODOMETRY: 10 s hover maximum XY 5.24 mm, height 46.70 mm; independent physical scorer |
| 8 cm flow minimum micro-flight | `px4-mtf-micro-06/summary.json` | Passed return/rest/auto-disarm: hover maximum XY 5.53 mm, height 24.85 mm, physical resting XY 2.26 mm. PX4 failsafe occurred after near-ground flow loss; further qualification required |
| Rate and attitude response | `px4-response-{rate,attitude}-{roll,pitch,yaw}-*/summary.json` | Six passing development axis probes; exact selected runs below; airborne response fixture, not landing/contact qualification |
| Recorded images/IMU | `px4-current-flight-images-02/summary.json` | 2,201 images, 27,501 IMU samples, 529 safe target frames; images did not control the recorded flight |
| Native publication instrumentation | `openvins-native-evidence-build-01/summary.json` | Passed separate pinned build; original research source and binary remain read-only |
| Native recorded VIO | `px4-current-flight-openvins-06/summary.json` | Execution passed; observed initialization, features, covariance, source/publication clocks and epochs |
| Recorded VIO development score | `px4-current-flight-vio-score-05/summary.json` | Passed development thresholds: XY p95 0.19599 m, height p95 0.01623 m, coverage 87.18%; one initial SE3 registration, scale fixed to 1 |
| Root tests | `px4-root-tests-09.log` | 383 passed, 2 ROS-only skips; headless EGL rendering included |

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
4 m/s² outward acceleration for 0.12 s dispatched on the impact tick.

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

## Remaining implementation and acceptance

- S1: current-aircraft grazing/back/non-target classifications and complete
  local forbidden-region audit. One converged front impact is insufficient.
- S2: finish and strictly score the independent 1 ms batch; S1 prerequisites
  must pass before reporting full S2 acceptance.
- S3: 20 independent sensor-only starts and the pre-registered fault matrix.
  A single numerical flow/MTF development run is insufficient.
- P4: PX4 Teacher/Student/Phase 4/5 integration, thrust/strategy evaluation and
  AP/PX4 paired comparison remain partial.
- P5/S4/S5: implement current-flight native VIO/ODOMETRY, image-driven approach,
  recovery, dock identification and slot guidance; complete 20 nominal and
  50 variable-condition runs plus fault handling. Recorded images and VIO
  scores do not qualify these gates.
- P6: existing ULog intake, finite UART logging session and M3C single-UART tools
  are present; end-to-end coverage and operation report still need completion.
  UART 115200 and ~50 Hz HIGHRES_IMU do not qualify 200 Hz raw IMU or real VIO.
- Default backend remains explicit during migration. Complete mission and
  hardware qualification are separate decisions; no hardware is modified.

## Supported commands

See [START_HERE.md](../START_HERE.md) for launch and viewer commands and
[perception adapters](../rm27/perception/sitl/README.md) for recorded/native VIO.
Each build/run requires a fresh output directory. Inspect actual summaries;
process exit and artifact names are not acceptance evidence.
