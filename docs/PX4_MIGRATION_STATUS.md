# PX4 v1.17.0 migration status

Updated 2026-10-11. This file records the implementation state in the
independent worktree `rm27_drones_px4`, branch
`codex/px4-v1.17-migration`.

## Fixed software inputs

- PX4 source tag `v1.17.0`, commit
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
- SITL binary SHA-256:
  `5646c151d002f988d73b533d6377685fa65ff5949b62f3cc869704075ec47c54`.
- Current target dynamics profile: `configs/dynamics_150g.json`.
  The 150 g mass is user supplied; inertia, thrust, motor lag and yaw torque
  remain engineering estimates.
- MuJoCo bridge uses 2 ms physical stepping, HIL_SENSOR,
  DISTANCE_SENSOR, ODOMETRY and HIL_ACTUATOR_CONTROLS. It enforces physical
  motor order, timestamps, lockstep and inactive-aircraft isolation.

## Completed evidence

| Area | Evidence | Result |
| --- | --- | --- |
| PX4 build | `artifacts/px4-build-08/summary.json` | `passed: true` |
| PX4 micro-flight | `artifacts/px4-micro-15/summary.json` | `passed: true`; automatic LAND disarm and resting gate |
| PX4 S0 batch | `artifacts/px4-s0-batch-01/summary.json` | `passed: true`; 10/10 independent launches |
| Exact physical replay | `artifacts/px4-replay-01/summary.json` | `passed: true`; 18,750 frames, zero state error |
| ULog intake and EKF2 | `artifacts/px4-ulog-analysis-02/summary.json` | `passed: true`; range and external-vision fusion topics present |
| Backend contracts | `tests/test_px4_backend.py` | 11 focused tests pass |
| Integrated tests | `artifacts/px4-root-tests-03.log` | 353 passed, 2 ROS-only skips |
| PX4 Corridor/contact return | `artifacts/px4-corridor-04/summary.json` | `passed: true`; low-speed contact, retreat and dock disarm |

The micro-flight is a closed-loop SITL diagnostic run. Its external position
source is synthetic ODOMETRY fused by EKF2, and its precision guidance is
truth-assisted. It does not qualify camera, optical-flow, GPS or real-aircraft
accuracy. Automatic disarm is confirmed from PX4 `vehicle_land_detected` /
arming telemetry; PX4 returns the user mode after disarm, so the post-land gate
does not require the heartbeat mode to remain `LAND`.

## Remaining gates

The Corridor first two PX4 attempts are retained as failures in
`artifacts/px4-corridor-01` and `artifacts/px4-corridor-02` (turn tracking just
over the unchanged 0.3 m abort gate). `px4-corridor-03` retained a runner time
limit failure after the tracking fix. `px4-corridor-04` is the current slower,
explicit-deadline rerun and passes the low-speed contact/retreat/dock route.
This does not complete the S1/S2 batch thresholds, sensor-only navigation,
visual target return or complete mission batches. Those gates must use fresh
PX4 evidence on the selected model; ArduPilot results are not transferred.
The OpenVINS artifacts and M3C/OS04A10 documents are imported research inputs,
not a qualified current-flight camera pipeline. Hardware flashing, parameter
writes and real flight remain outside this software migration.

## Commands

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
/home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q

/home/shiuhou/venvs/mujoco/bin/python -m rm27 micro-flight \
  artifacts/px4-dock-150g-01/scene.xml --autopilot px4 \
  --dynamics configs/dynamics_150g.json --output artifacts/px4-micro-new

/home/shiuhou/venvs/mujoco/bin/python -m rm27.autopilot.px4.batch \
  artifacts/px4-dock-150g-01/scene.xml --dynamics configs/dynamics_150g.json \
  --output artifacts/px4-s0-batch-new --count 10 --instance 0
```
