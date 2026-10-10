# Drone intercepts a red-side dart

This independent MuJoCo experiment adds one motor-driven 180 g UAV and a dynamic dart to the existing blue-base/red-dart scene. The current version restores the 13 original hangar pedestal and footing geoms from v6 at their original poses. The separate dock stays on top of that pedestal. The UAV starts in `slot_1`, aligned with the dock's 45-degree heading, with its center 22 mm above the slot floor. It climbs vertically about 0.39 m, waits over the dock, and starts its interception maneuver after the dart launches at 1.10 s. The source scene and original Darts run stay unchanged.

The dart's visual mesh, launch position, initial velocity, gravity and original 1.770383528 s flight time come from `flight-04-apex-5m`. Since 2026-10-09, both its nose position and its velocity-aligned attitude follow that original replay exactly until first physical contact: nose up during ascent, horizontal at the 5 m apex, nose down during descent. Its roll convention also matches the original `basis(velocity)` function. This prescribed flight rule is not an aerodynamic model.

At every physics sample before contact, the same free body receives the original nose position, orientation, world-frame nose velocity and body-frame angular velocity derived from the changing orientation. MuJoCo detects contact at that pose. At the first active penetrating contact with any object, the code permanently stops overwriting that body before the contact impulse is solved. Its position, attitude and both velocities carry through the handoff; gravity and rigid-body contact then determine subsequent motion. There is no second dart or post-impact return to the prescribed path. A capsule supplies the collision shape, and its center of mass is offset from the nose.

The UAV uses MuJoCo truth to fly through a predicted crossing at 1.20 s after launch, then brakes. The attitude controller's assumed frequency is 22 rad/s for this independent experiment; motor thrust limits stay unchanged.

## Reproduce

Run these commands from the repository root (`/home/shiuhou/Projects/rm27_drones` on the current machine):

```bash
cd /home/shiuhou/Projects/rm27_drones
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.dart_interception run --output artifacts/dart-interception-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.dart_interception run --output artifacts/dart-interception-passive-new --passive
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.dart_interception view artifacts/dart-interception-new
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.dart_interception render artifacts/dart-interception-new --output artifacts/dart-interception-video-new --video
```

Each build/render command needs a new output directory. `view` replays the recorded states, and `render` writes PNG frames and `flight.mp4`. The headless run writes `scene.xml`, a compiled `model.mjb`, `trace.npz`, input hashes and `summary.json`. The trace includes the dart's `waiting`, `prescribed` or `dynamic` mode and an exact handoff frame. The summary records handoff state, colliding geometry names, pre-contact pose errors and normal impulses from actual solver steps. The scene's asset paths are absolute to the local workspace; `model.mjb` and `trace.npz` are the replay pair.

The interactive `view` command requires a graphical desktop session. The default `--camera free` starts with the whole scene and preserves mouse rotation, pan and zoom. Keys `1`, `2`, `3`, `4`, `5` select whole-scene, UAV, dart, automatic and free views. Pressing `5` releases camera control at the current pose. `--speed 0.5` slows playback. The MP4 plays at 0.5x and includes a whole-scene inset. For headless SSH, open the generated MP4 with a player on a graphical machine. The current video is `artifacts/dart-interception-aligned-render-20261009-v1/flight.mp4`.

## Current evidence

The current active run is `artifacts/dart-interception-aligned-20261009-v1`; its `summary.json` reports `passed: true`. The UAV starts at `(0.651751442, 1.082427939, 1.762)` m in `slot_1`. At the 1.10 s launch, its center was at 2.143 m. Maximum horizontal drift before launch was less than 0.001 mm. Its first 5 cm of lateral motion after launch occurred at 1.377 s. Physical UAV/dart contact and dynamic handoff occurred at 2.294 s, near `(-3.786, -2.016, 4.557)` m. Accumulated normal contact impulse was 0.393 N s. The dart did not contact the added blue-base target proxy during the 3.20 s run.

The matching passive-hover control is `artifacts/dart-interception-aligned-passive-20261009-v1`; its `summary.json` also reports `passed: true`, meaning that the control behaved as expected. There was no UAV/dart contact and the dart contacted the blue-base target proxy at 2.870 s, then switched to dynamic motion. Both runs have zero nose-position error at prescribed samples and a maximum rotation-matrix error below 6e-16 relative to the original rule. Regression tests compare position and quaternion against the original Darts `pose` routine, verify matching angular velocity, and check that the handoff is permanent with a subsequent departure from the prescribed path. Dock departure and opposite active/passive outcomes are also checked.

The 0.5 ms step checks in `artifacts/dart-interception-aligned-halfstep-20261009-v1/active` and `passive` both report `passed: true`, with interception at 2.2935 s and passive target contact at 2.8695 s. They reproduce the event outcomes; impact forces and real-world collision response are not calibrated. The target proxy is an experiment-defined rigid patch, not certified base collision geometry or a damage model. Earlier `dart-interception-dock-20261007-v4` used unguided rigid-body attitude and is superseded for this pose requirement; `dart-interception-20261007-v3` used a ground start near the base.

The original launch and ballistic path come from `research/Darts/artifacts/flight-04-apex-5m/summary.json`; the original CAD-derived dart visual mesh is verified by SHA-256. The isolated source scene is `artifacts/isolated-blue-base-red-dart-20260927-v1/scene.xml`. Input SHA-256 values are in each build's `build.json` and `summary.json`. Root and Darts tests: 138 passed on 2026-10-09.

## Limits

The dock run demonstrates interception, not stable impact recovery. The planner applies a scheduled braking reference, without a dedicated impact recovery controller. The `passed` gate checks interception with positive contact impulse and absence of target contact, but does not check recovery stability. A passing run can visibly wobble or tumble after contact. This pose correction does not establish stable recovery.

The 60 g dart mass, 14 mm radius rigid capsule, target contact patch and launch timing are engineering assumptions. The rigid dart has no measured aerodynamic forces, spin stabilization, flexibility or calibrated muzzle speed. The UAV uses the provisional 180 g motor/airframe estimates and MuJoCo truth with a known nominal projectile path; it does not detect the dart with a camera, estimate launch timing, run SITL, or establish real-aircraft interception capability. Contact with the UAV and absence of target-proxy contact are the demonstrated outcomes; referee scoring and protection of the actual base are unverified.
