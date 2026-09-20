import numpy as np
from research.astra.config import Config
from research.astra.simulation import simulate


class RecordedFeatures:
    def measure(self, relative, basis, cfg):
        n = len(relative)
        # Same prerecorded sensor history regardless of hidden target state.
        return np.zeros((n, 2)), np.full(n, .025), np.ones(n, bool)


def test_hidden_target_changes_cannot_change_commands_with_same_sensor_history():
    c = Config(duration_s=.5, performance_spread=0, ego_velocity_noise=0,
               ego_drift_m_s=0, ego_scale_error=0, ego_yaw_drift_deg_s=0,
               pixel_noise=0, radius_noise_fraction=0, camera_dropout=0)
    trace_a, trace_b = [], []
    simulate(c, 'lead', 1, 1, trace=trace_a, camera=RecordedFeatures())
    simulate(c, 'lead', 1, 99, trace=trace_b, camera=RecordedFeatures())
    assert trace_a[0]['target'] != trace_b[0]['target']
    np.testing.assert_array_equal([r['p'] for r in trace_a], [r['p'] for r in trace_b])
