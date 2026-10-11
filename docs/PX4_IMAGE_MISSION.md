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

The frozen V1 primary simulation camera is mounted 55 degrees downward. A development
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
| `px4-image-mission-development-04` | Physical development gates pass once; first-hit detection delay 4 ms, recovery entry 0.433 s, resting XY 0.650 mm; subsequent local visual-surface audit fails |
| `px4-image-mission-guard-tip-development-05` | Canonical V10 physical development passes; first-hit inference 18 ms, recovery entry 0.451 s, resting XY 6.314 mm; every-recorded-pose local surface audit passes 5,955 poses |
| `px4-image-mission-guard-tip-development-06` | Canonical V11 physical development passes; first-hit inference 8 ms, recovery entry 0.431 s, resting XY 1.932 mm; ULog confirms actual EV position/velocity/height fusion |

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

The frozen nominal V1 batch is complete and failed: **11/20** full task successes,
six physical recovery failures, one exhausted AprilTag reacquisition and two
invalid LAND alignments. All twenty attempts are retained in
`px4-image-nominal-batch-01`; this set is not used to select a new policy.
A subsequent exact capsule/triangle audit of V4 checks 1,128 recorded
local poses against 192,431 nearby visual triangles. It finds guard crossings
outside the configured armor patch, including the visual tower. These surfaces
have no physical collision, so the original flight gates did not detect them.
This prevents S1 or complete-mission acceptance. `target_region_audit` performs
this check separately; it does not add physics to the upstream field model.

New guard-tip development aligns the configured simulated guard corner with
the image-derived armor centre, using fresh PX4 attitude. This replaces COM
alignment and requires a separate primary-camera mount study. The provisional
guard point and mount remain simulation assumptions. Development takes place
in the canonical `autopilot.px4.image_route` with an explicit configured contact
point; the absent option retains the legacy COM behavior and the V1 manifest
retains its exact source hashes. The development module is a compatibility
entry point. Full recovery-volume approval remains required even
if a recorded trajectory clears the surfaces.

`px4-guard-tip-planned-poses-01` / `px4-guard-tip-planned-surfaces-01` check 504
configured static poses: tip normal distances 2 mm through 0.6 m, ±10 mm local
offsets and selected ±3-degree attitudes. No forbidden triangle intersections
were found. This is a sampled planned-pose check, not a continuous recovery
volume certificate or a flight. The first actual guard-tip attempts failed
target acquisition at 30 degrees and image boundary clipping at 35 degrees.
A new 40-degree/70-degree-FOV 800×600 development camera keeps similar metric
pixel scale with more edge margin. These are separate simulation assumptions.

Long return attempts also expose near-ground height drift, tag reacquisition
exhaustion and invalid alignment at LAND. New tag-height development derives
body height from the assigned metric tag, PX4 attitude and configured camera
mount, retaining raw EKF NED positions. It translates both task height and
outgoing goals consistently, with a 25 cm registration bound. The tag panel's
configured top lies 1.5 mm above the slot reference. This is image registration,
not physical truth or a new EKF fusion claim. The canonical `tag_height.py`
rejects repeated, backwards and stale image times, wrong assigned slots and
stale navigation. Image and raw EKF heights are compared at acquisition time.
Fresh HIGHRES_IMU acceleration and gyro are paired for contact inference;
cached attitude rates previously delayed reaction by one 20 ms sensor period.

Guard-tip development V9 (`px4-image-mission-guard-tip-development-04`) passes
one complete physical task: first-impact inference delay 16 ms, recovery entry
0.438 s and independent resting XY 1.554 mm. Its closed-recording surface audit
passes 1,200 sampled poses (`px4-guard-tip-flight-surfaces-development-03`).
That is development evidence, not 20/20 reliability or continuous geometry
approval. Fresh canonical V10/V11 flights above validate the timing fixes.
The V10 guard-tip impact converges at native 1 ms against 0.5 ms: impulse
difference 2.118%, peak difference 1.097%; finer comparisons also pass
(`px4-guard-tip-convergence-development-01`). This applies to one recorded
front impact, not every variant. Root tests pass **418**, with five unavailable
ROS/environment cases skipped (`px4-root-tests-19.log`).

## Run

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.autopilot.px4.image_mission artifacts/px4-dock-tags-05/scene.xml \
  --audit artifacts/px4-guard-tip-corridor-audit-01/summary.json \
  --config configs/px4_image_guard_tip_dev_v11.json \
  --camera-config configs/px4_sim_camera_guard_tip_dev_v8.json \
  --instance 40 --output artifacts/px4-image-mission-new

MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.autopilot.px4.image_batch artifacts/px4-dock-tags-05/scene.xml \
  --audit artifacts/px4-guard-tip-corridor-audit-01/summary.json \
  --config configs/px4_image_nominal_v2.json --workers 3 --instance 50 \
  --output artifacts/px4-image-nominal-new
```

The frozen V2 specification reserves twenty distinct camera/IMU noise seeds
313001–313020 before its first launch. It adds the repaired guard-tip approach,
image-height registration and an independent local surface gate. V1's failed
set remains preserved separately; V2 acceptance is not implied by development.
New batch specifications can require `surface_configuration`, which adds an
independent closed-recording local surface check to every attempt. The V2
surface configuration checks every recorded 250 Hz local pose; it does not
certify the continuous swept recovery volume between observations.

The batch freezes code, camera, reviewed corridor/assets, estimator, native VIO
and PX4 binary before launch and retains every failed attempt. Each instance
has separate processes, parameters, VIO session and logs. The public batch
`passed` requires mission, hover, recovery, slot/rest, assigned tag, LAND,
automatic disarm and actual EKF2 fusion evidence; receiving ODOMETRY alone
cannot pass it. Add `--view` to the single mission command for a live viewer.
