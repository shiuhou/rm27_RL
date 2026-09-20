"""TruthTargetAdapter - Gazebo poses -> TargetEstimate (the V2 "camera").

This is the deliberate truth boundary of V2: Gazebo ground truth enters
the guidance chain ONLY through this adapter, which converts world poses
into the exact TargetEstimate schema the real maixcam2_dart camera will
emit later. UAVGuidance never sees poses, Gazebo, or this module.

Frame math (docs/frames.md section 5, the single ENU/vision sign flip):

    bearing_enu     = atan2(ty - uy, tx - ux)          (CCW from East)
    heading_err_enu = wrap_pi(bearing_enu - uav_yaw_enu)  (+ = target LEFT)
    yaw_rad (vision, + = target RIGHT) = -heading_err_enu

Pitch follows the same convention: pitch_rad = -atan2(down, forward), so
a target below the boresight (UAV above it) reports pitch_rad < 0.
"""

import math

from ..guidance import TargetEstimate

# Simulated visual latency for a "perfect" truth camera. The stale mode
# uses the same 400 ms as the V1 synthetic producer (beyond the 200 ms
# guidance stale gate).
NORMAL_LATENCY_US = 20_000
STALE_LATENCY_US = 400_000


def wrap_pi(angle_rad):
    """Wrap to (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def enu_yaw_rad(quat):
    """Yaw (CCW from East, z-up) from an ENU quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quat
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def vision_yaw_rad(uav_pos, uav_quat, target_pos):
    """TargetEstimate.yaw_rad from ENU poses (right of boresight > 0)."""
    uav_yaw_enu = enu_yaw_rad(uav_quat)
    bearing_enu = math.atan2(
        target_pos[1] - uav_pos[1], target_pos[0] - uav_pos[0]
    )
    return -wrap_pi(bearing_enu - uav_yaw_enu)


def vision_pitch_rad(uav_pos, target_pos):
    """TargetEstimate.pitch_rad (above boresight > 0); planar-world sane."""
    horizontal = math.hypot(
        target_pos[0] - uav_pos[0], target_pos[1] - uav_pos[1]
    )
    # z-up world: target below the UAV => positive "down" component.
    down = uav_pos[2] - target_pos[2]
    return -math.atan2(down, horizontal)


class TruthTargetAdapter:
    """Convert latest Gazebo poses into a TargetEstimate.

    Modes (mirroring the V1 synthetic producer):
        normal  : fresh, valid, safe estimate
        unsafe  : valid estimate with safe_for_control=False
        stale   : valid estimate with measurement age beyond the gate
        dropout : no estimate at all (returns None)
    Missing/non-finite poses also return None (target not visible).
    """

    def __init__(
        self,
        pose_source,
        uav_name="iris_with_gimbal",
        target_name="rm27_target",
        confidence=1.0,
    ):
        self._source = pose_source
        self._uav_name = uav_name
        self._target_name = target_name
        self._confidence = confidence

    def update(self, now_us, mode="normal"):
        if mode == "dropout":
            return None

        poses = self._source.poses()
        uav = poses.get(self._uav_name)
        target = poses.get(self._target_name)
        if uav is None or target is None:
            return None

        pos_u, quat_u = uav["pos"], uav["quat"]
        pos_t = target["pos"]
        values = (*pos_u, *quat_u, *pos_t)
        if not all(math.isfinite(v) for v in values):
            return None

        yaw_rad = vision_yaw_rad(pos_u, quat_u, pos_t)
        pitch_rad = vision_pitch_rad(pos_u, pos_t)

        if mode == "normal":
            measurement_age_us = NORMAL_LATENCY_US
            safe_for_control = True
        elif mode == "unsafe":
            measurement_age_us = NORMAL_LATENCY_US
            safe_for_control = False
        elif mode == "stale":
            measurement_age_us = STALE_LATENCY_US
            safe_for_control = True
        else:
            raise ValueError(f"unknown truth adapter mode: {mode}")

        return TargetEstimate(
            timestamp_us=now_us,
            measurement_age_us=measurement_age_us,
            valid=True,
            safe_for_control=safe_for_control,
            predicted=False,
            yaw_rad=yaw_rad,
            pitch_rad=pitch_rad,
            confidence=self._confidence,
        )
