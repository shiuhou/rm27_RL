# Migrated ArduPilot bridge

This package is the first selected migration from
`rm27/autopilot/baselines/mujoco_uav_ardupilot/ardupilot_bridge`. It owns the JSON servo
protocol, frame transforms, actuator mapping and SITL launch contract used by
the root `rm27.autopilot` package.

The source checkout remains available as a migration reference until the
copied implementation and its tests are committed and audited. New root code
must import this package; it must not add the old checkout to `sys.path`.
