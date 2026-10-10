"""Planned forward-facing return, independent of camera and estimator code."""
import numpy as np

from .control import Septic


class HeadingTurn:
    """Shortest yaw turn with zero rate/acceleration/jerk at both ends.

    The scalar seventh-order smoothstep has peak derivative 35/16 and
    peak second derivative 84/(5*sqrt(5)). Duration enforces both bounds.
    At exactly pi choose the positive direction deterministically.
    """
    def __init__(self, start_yaw, end_yaw, duration_s=3., max_rate_rad_s=2., max_accel_rad_s2=3.):
        if not np.isfinite([start_yaw, end_yaw, duration_s, max_rate_rad_s, max_accel_rad_s2]).all():
            raise ValueError('Finite heading parameters required')
        if min(duration_s, max_rate_rad_s, max_accel_rad_s2) <= 0:
            raise ValueError('Positive heading limits required')
        delta = float((end_yaw-start_yaw+np.pi) % (2*np.pi)-np.pi)
        if abs(delta+np.pi) < 1e-12:
            delta = np.pi
        self.duration = max(duration_s, 35/16*abs(delta)/max_rate_rad_s,
                            np.sqrt(84/(5*np.sqrt(5))*abs(delta)/max_accel_rad_s2))
        self.path = Septic([start_yaw, 0., 0., 0.], [start_yaw+delta, 0., 0., 0.], self.duration)

    def sample(self, elapsed):
        return self.path.sample(elapsed)


def course_yaw(direction_world, forward_body):
    """Horizontal body yaw aligning an explicit body travel axis to a course."""
    direction, forward = np.asarray(direction_world, float), np.asarray(forward_body, float)
    if direction.shape != (3,) or forward.shape != (3,) or not np.isfinite(np.r_[direction, forward]).all():
        raise ValueError('Finite three-dimensional course and body axis required')
    if min(np.linalg.norm(direction[:2]), np.linalg.norm(forward[:2])) < 1e-8:
        raise ValueError('Nonzero horizontal course and body axis required')
    return float(np.arctan2(direction[1], direction[0])-np.arctan2(forward[1], forward[0]))
