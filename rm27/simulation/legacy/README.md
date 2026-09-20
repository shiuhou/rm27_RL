# Legacy simulation sources

`mujoco_uav/` contains the standalone M0–M4 experiments and their original
tests. They are retained as source evidence because their vehicle, frames and
controller assumptions differ from the current RM27 integration plant.

Current simulation entry points are under `rm27/simulation/` and are tested by
the root suite. Do not use these scripts as a second runtime interface.
