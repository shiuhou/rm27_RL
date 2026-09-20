# Legacy and source-only components

The following directories share the RM27 research goal but are not current
root runtime code:

- `../rm27_drones_legacy/rm_uav_lab/` contains the former PX4 and older RM27
  scenes: earlier simulator and controller experiments with different vehicle
  and frame assumptions.
- `rm27/simulation/legacy/mujoco_uav`: M0–M4 standalone MuJoCo experiments and
  their own tests.
- The former `rm_uav_lab_m600/visual_guidance` checkout supplied the migrated
  Python guidance interface. Its source commit is recorded in
  `docs/MIGRATION_PROVENANCE.md`; the old checkout has been removed and is not a
  runtime entry.
- `rm27/perception/vision/maixcam2_dart_vision`: canonical board-side
  C++/MaixCDK vision checkout. It owns camera capture, green-light/armor
  recognition and device-side perception tools; root Python code consumes its
  documented `TargetEstimate` schema.
- The former `rm_uav_lab_m600/maixcam2_dart` checkout is superseded by the
  canonical nested checkout under `rm27/perception/vision/`; its source commit
  is retained in the provenance table. Do not recreate it as a second vision
  entry point.
- `rm27/experiments/legacy_sitl/` and the old Gazebo visual benchmarks:
  standalone SITL/Gazebo mission runners with their own world and vehicle
  assumptions; current root missions use `rm27.autopilot` and
  `rm27.experiments`.

These directories may be removed from the working tree only after the root
implementation has a reproducible comparison run and the source commit is
recorded in `docs/MIGRATION_PROVENANCE.md`. Their historical reports do not
become current RM27 evidence through migration.
