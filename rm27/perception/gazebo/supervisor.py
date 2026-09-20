"""TruthSupervisor - V2 stop-distance gate (lives OUTSIDE guidance).

Watches the TRUE Gazebo distance between UAV and target and forces a
zero-motion GuidanceCommand when the UAV comes inside the configured
stop distance. This is a benchmark safety net, not part of the guidance
loop: guidance itself knows nothing about distance or truth.

Boundary discipline:
- consumes: Gazebo poses (truth) + GuidanceCommand
- produces: GuidanceCommand (possibly zeroed)
- never touches TargetEstimate, UAVGuidance, or the MAVLink adapter's
  internals.
"""

import math

from ..guidance import GuidanceCommand


class TruthSupervisor:
    def __init__(self, pose_source, stop_distance_m,
                 uav_name="iris_with_gimbal", target_name="rm27_target"):
        if stop_distance_m <= 0.0:
            raise ValueError("stop_distance_m must be positive")
        self._source = pose_source
        self._stop_distance_m = float(stop_distance_m)
        self._uav_name = uav_name
        self._target_name = target_name

    def distance_m(self):
        """True horizontal UAV<->target distance, or None if unknown."""
        poses = self._source.poses()
        uav = poses.get(self._uav_name)
        target = poses.get(self._target_name)
        if uav is None or target is None:
            return None
        return math.hypot(
            target["pos"][0] - uav["pos"][0],
            target["pos"][1] - uav["pos"][1],
        )

    def apply(self, command, now_us):
        """Return (command_to_send, gated). Inside the stop distance the
        command is replaced by an explicit zero-motion hold."""
        distance = self.distance_m()
        if distance is not None and distance < self._stop_distance_m:
            return GuidanceCommand.zero(now_us, valid=False), True
        return command, False
