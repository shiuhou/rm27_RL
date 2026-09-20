# Autopilot baselines

`mujoco_uav_ardupilot/` is the frozen G1 ArduPilot–MuJoCo baseline. It is
kept under the unified RM27 tree so its bridge and evidence are easy to find,
but it is a historical baseline rather than the calibrated 180 g RM27 plant.

Keep its submodule commit and reports intact. Current root adapters are copied
under `rm27/autopilot/bridge/` and must not silently inherit G1 claims.
