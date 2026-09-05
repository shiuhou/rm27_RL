"""Observation-only baselines; no simulator access or privileged initialization."""
import numpy as np
from .geometry import unit, unproject


class Guidance:
    def __init__(self, n, method, config):
        if method not in ('pursuit', 'compensated_pursuit', 'lead'):
            raise ValueError(method)
        self.method, self.cfg = method, config
        self.position = np.zeros((n, 3))
        self.velocity = np.zeros((n, 3))
        self.epoch = np.full(n, -np.inf)
        self.seen = np.zeros(n, bool)

    def observe(self, obs, now):
        if obs.delivery_time > now+1e-9 or obs.capture_time > obs.delivery_time:
            raise ValueError('observation not yet delivered or invalid timestamps')
        measurement = obs.camera_position_estimate + unproject(obs.uv_px, obs.angular_radius_rad, obs.camera_to_local_estimate,
                                                              self.cfg.assumed_target_radius_m, self.cfg.fov_deg, self.cfg.width)
        valid = obs.valid & np.all(np.isfinite(measurement), axis=1) & (obs.capture_time > self.epoch)
        initial = valid & ~self.seen
        update = valid & self.seen
        elapsed = np.where(self.seen, obs.capture_time-self.epoch, 0.)
        predicted = self.position + elapsed[:, None]*self.velocity
        innovation = measurement - predicted
        self.position[update] = (predicted+self.cfg.alpha*innovation)[update]
        self.velocity[update] += (self.cfg.beta*innovation/np.maximum(elapsed[:, None], 1e-6))[update]
        self.position[initial] = measurement[initial]
        self.velocity[initial] = 0
        self.epoch[valid] = obs.capture_time
        self.seen |= valid

    def target_estimate(self, now):
        age = np.where(self.seen, now-self.epoch, 0.)
        return self.position + self.velocity*age[:, None]

    def command(self, ego):
        c = self.cfg
        if ego.time < c.guidance_start_s:
            yaw = np.arctan2(ego.camera_to_local[:, 1, 2], ego.camera_to_local[:, 0, 2])
            return -c.velocity_gain*ego.velocity, yaw
        # Pursuit uses filtered position at exposure time. Lead compensates age
        # and predicts a constant-velocity intercept. Neither knows target future.
        target = self.position if self.method == 'pursuit' else self.target_estimate(ego.time)
        relative = target - ego.position
        direction = relative.copy()
        if self.method == 'lead':
            a = np.sum(self.velocity**2, axis=1)-c.cruise_m_s**2
            b = 2*np.sum(relative*self.velocity, axis=1)
            cc = np.sum(relative**2, axis=1)
            discriminant = b*b-4*a*cc
            root = np.sqrt(np.maximum(discriminant, 0.))
            safe_a = np.where(np.abs(a)>1e-8, a, 1.)
            roots = np.stack([(-b-root)/(2*safe_a), (-b+root)/(2*safe_a)], axis=1)
            roots[(roots <= 0) | (discriminant[:, None] < 0)] = np.inf
            tgo = roots.min(axis=1)
            tgo[np.abs(a)<=1e-8] = np.inf
            linear = (np.abs(a)<=1e-8) & (b < -1e-8)
            tgo[linear] = -cc[linear]/b[linear]
            tgo = np.minimum(tgo, c.max_prediction_s)
            direction += self.velocity*tgo[:, None]
        desired_velocity = c.cruise_m_s*unit(direction)
        healthy = self.seen & ((ego.time-self.epoch) <= c.track_timeout_s)
        desired_velocity[~healthy] = 0  # local braking on track loss, no blind continuation
        acceleration = c.velocity_gain*(desired_velocity-ego.velocity)
        yaw = np.arctan2(relative[:, 1], relative[:, 0])
        yaw[~healthy] = np.arctan2(ego.camera_to_local[~healthy, 1, 2], ego.camera_to_local[~healthy, 0, 2])
        return acceleration, yaw
