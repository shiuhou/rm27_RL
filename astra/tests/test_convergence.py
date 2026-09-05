from dataclasses import replace
import numpy as np
from astra.config import Config
from astra.simulation import simulate


def test_halving_step_preserves_guidance_comparison():
    # Ideal observations at exactly representable exposure/delivery times isolate
    # integration error from camera timing jitter. Guidance stays at 100 Hz.
    c = Config(camera_hz=25, pixel_noise=0, radius_noise_fraction=0, camera_dropout=0,
               ego_velocity_noise=0, ego_drift_m_s=0, ego_scale_error=0, ego_yaw_drift_deg_s=0)
    for method in ('pursuit', 'compensated_pursuit', 'lead'):
        a = simulate(c, method, 200, 27001)
        b = simulate(replace(c, dt=.0025), method, 200, 27001)
        assert sum(x['success'] != y['success'] for x, y in zip(a, b)) <= 2
        assert np.quantile([abs(x['miss_m']-y['miss_m']) for x, y in zip(a, b)], .95) < .03
