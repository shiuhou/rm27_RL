"""GuidanceMavlinkAdapter - GuidanceCommand -> MAVLink body-frame setpoint.

The ONLY thing this adapter understands is GuidanceCommand. It never sees
TargetEstimate and knows nothing about motors, mixers, or PIDs; ArduPilot
remains responsible for all low-level control.

Wire format (docs/frames.md section 4):
    SET_POSITION_TARGET_LOCAL_NED
    coordinate_frame = MAV_FRAME_BODY_NED (8)
    type_mask        = 0x05C7  (ignore position, accel, yaw angle;
                                honor vx, vy, vz, yaw_rate)
"""

import math

from ..guidance import GuidanceCommand, TargetEstimate

MAV_FRAME_BODY_NED = 8

# POSITION_TARGET_TYPEMASK: ignore pos xyz (0x0007), ignore accel xyz
# (0x01C0), ignore yaw angle (0x0400). Velocity xyz and yaw_rate honored.
GUIDANCE_TYPE_MASK = 0x05C7


class GuidanceMavlinkAdapter:
    def __init__(self, master):
        self.master = master

    def send(self, command):
        if isinstance(command, TargetEstimate):
            raise TypeError(
                "adapter boundary violation: send() takes GuidanceCommand, "
                "never TargetEstimate"
            )
        if not isinstance(command, GuidanceCommand):
            raise TypeError(f"expected GuidanceCommand, got {type(command)}")

        if command.valid:
            vx, vy, vz, yaw_rate = (
                command.vx_body_mps,
                command.vy_body_mps,
                command.vz_body_mps,
                command.yaw_rate_rad_s,
            )
        else:
            # Invalid/failsafe command: explicit zero motion.
            vx = vy = vz = yaw_rate = 0.0

        # Never propagate non-finite values onto the wire.
        motion = [v if math.isfinite(v) else 0.0 for v in (vx, vy, vz, yaw_rate)]

        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms: 0 = let the FC use its own clock
            self.master.target_system,
            self.master.target_component,
            MAV_FRAME_BODY_NED,
            GUIDANCE_TYPE_MASK,
            0, 0, 0,                # position ignored
            motion[0], motion[1], motion[2],
            0, 0, 0,                # accel ignored
            0.0,                    # yaw angle ignored
            motion[3],              # yaw_rate rad/s, positive = right
        )
