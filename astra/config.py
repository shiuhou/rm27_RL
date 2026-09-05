"""Engineering assumptions, not measured micro-UAV specifications."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    dt: float = .005
    guidance_hz: float = 100.
    duration_s: float = 12.
    camera_hz: float = 30.
    latency_s: float = .05
    fov_deg: float = 90.
    camera_pitch_deg: float = 0.
    width: int = 640
    height: int = 480
    pixel_noise: float = .7
    radius_noise_fraction: float = .03
    min_diameter_px: float = 3.
    camera_dropout: float = .02
    occlusion_s: float = 0.
    tw: float = 2.
    tilt_deg: float = 45.
    attitude_tau_s: float = .10
    motor_tau_s: float = .04
    yaw_rate_deg_s: float = 180.
    drag_per_s: float = .12
    mass_g: float = 200.
    performance_spread: float = .10
    cruise_m_s: float = 6.
    velocity_gain: float = 4.
    alpha: float = .65
    beta: float = .08
    max_prediction_s: float = 3.
    track_timeout_s: float = .5
    target_radius_m: float = .1
    assumed_target_radius_m: float = .1
    vehicle_radius_m: float = .07
    ego_velocity_noise: float = .03
    ego_drift_m_s: float = .03
    ego_scale_error: float = .02
    ego_yaw_drift_deg_s: float = .1

    def __post_init__(self):
        if not (0 < self.dt <= .05 and self.duration_s > 0 and 0 < self.camera_hz <= 1/self.dt):
            raise ValueError('invalid simulation/sensor time base')
        if not (0 < self.guidance_hz <= 1/self.dt):
            raise ValueError('invalid guidance time base')
        if not (0 <= self.latency_s and 5 <= self.fov_deg < 170 and 0 < self.tilt_deg < 85 and self.tw > 1):
            raise ValueError('invalid latency/FOV/flight limits')
        if not (0 <= self.camera_dropout <= 1 and 0 <= self.performance_spread < 1):
            raise ValueError('invalid probability or performance spread')
        if min(self.target_radius_m, self.assumed_target_radius_m, self.vehicle_radius_m, self.attitude_tau_s, self.motor_tau_s) <= 0:
            raise ValueError('sizes and time constants must be positive')
