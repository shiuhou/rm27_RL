# PX4 complete image/VIO mission

Updated 2026-10-11. This is host MuJoCo/PX4 simulation software. The primary
camera feeds current-flight OpenVINS and armor pose detection; a separate camera
at the aircraft's underside centre is used only for AprilTag return guidance.
The existing downward identification/landing implementation is in
`perception/sitl/return_camera.py`, `autopilot/px4/visual_return.py` and
`autopilot/px4/flight.py`: it checks assigned ID21, source age and registration,
aligns, descends, reacquires after temporary loss and hands over to normal PX4
LAND before confirming automatic disarm. IDs22–24 identify the other slots.

`image_route.py` adds surveyed corridor travel, image acquisition and metric
pose approach to the simulated blue armor, bounded contact commitment, fresh
IMU/rate contact inference, recovery and corridor return. Actor inputs contain
RGB poses and PX4 telemetry; simulator target pose/contact flags are forbidden.
Surveyed plane orientation and the 90×55 mm simulated rectangle are reviewed
priors. Independent physical scores retain truth separately.

The primary simulation camera is mounted 55 degrees downward. A development
render sweep found the inherited 20-degree mount could not see the armor on
this route. Both this mount and camera calibration are simulation assumptions;
they are not OS04A10 hardware measurements. The dedicated return camera uses
80 mm AprilTag 36h11 codes on 96 mm white panels, per the user's choice.

The dedicated navigation profile uses external vision height plus barometer
fallback. Using terrain range as the height reference across the platform
created about 0.24 m vertical bias in development. Range still supplies optical
flow terrain distance and landing observations. Every run reads back its PX4
parameters. Actual external position/velocity/height fusion is checked in ULog.

## Evidence and limits

All directories below are under `artifacts/`; read actual `summary.json`.

| Attempt | Result |
|---|---|
| `px4-image-mission-development-01` | Failed target acquisition/navigation registration |
| `px4-image-mission-development-02` | Contact lease exhausted; safe corridor return and AprilTag LAND completed, task failed |
| `px4-image-mission-development-03` | Failed: secondary collision and contact inference 208 ms late |
| `px4-image-mission-development-04` | One complete development success; first-hit detection delay 4 ms, recovery entry 0.433 s after impact, continuous hold ≥1 s; physical resting XY error 0.650 mm |

V3's original success summary was incorrect: its scorer began at the inferred
hit, omitting the actual first hit and subsequent collision. The original bytes
and hashes are retained in `px4-image-mission-v3-independent-audit-01`; the run
summary is explicitly superseded and failed. V4 saves independent 1 ms physical
samples from first impact through recovery and all later non-dock collisions.
The regression test checks that late inference cannot hide a second impact.
Recovery thresholds remain unchanged; transient rates are recorded while the
required stable hold is scored separately.

The single successful V4 run is **headless closed-loop SITL development**.
Its ULog confirms actual external position, velocity and height fusion, and its
current-flight OpenVINS summary retains source ages and tracking losses. It does
not establish 20/20 reliability, the 50-condition gate, full fault handling,
complete S1 geometry approval, hardware imaging or M3C real-time qualification.
The separate downward short-return/fault batches retain their narrower scope.

## Run

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.autopilot.px4.image_mission artifacts/px4-dock-tags-05/scene.xml \
  --audit artifacts/px4-image-corridor-audit-01/summary.json \
  --instance 40 --output artifacts/px4-image-mission-new

MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.autopilot.px4.image_batch artifacts/px4-dock-tags-05/scene.xml \
  --audit artifacts/px4-image-corridor-audit-01/summary.json \
  --config configs/px4_image_nominal_v1.json --workers 3 --instance 50 \
  --output artifacts/px4-image-nominal-new
```

The batch freezes code, camera, reviewed corridor/assets, estimator, native VIO
and PX4 binary before launch and retains every failed attempt. Each instance
has separate processes, parameters, VIO session and logs. The public batch
`passed` requires mission, hover, recovery, slot/rest, assigned tag, LAND,
automatic disarm and actual EKF2 fusion evidence; receiving ODOMETRY alone
cannot pass it. Add `--view` to the single mission command for a live viewer.
