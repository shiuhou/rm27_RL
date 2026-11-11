"""Behavior checks for optional monitoring without simulation or network access."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from simpleflight_lab.monitoring import Monitor, evaluation_metrics, training_metrics, probe_service

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / '.agents/runs/rm27-stage1/pgs-braking-pilot'


class MonitoringTest(unittest.TestCase):
    def test_disabled_monitor_has_no_side_effects(self):
        with tempfile.TemporaryDirectory() as tmp:
            destination = Path(tmp) / 'absent'
            with patch.dict('os.environ', {}, clear=True):
                monitor = Monitor(destination, 'disabled')
                monitor.log({'loss': 1.0})
                monitor.finish()
            self.assertFalse(destination.exists())

    def test_proxy_and_optimizer_values_are_separate(self):
        data = training_metrics({'frames': 8192, 'episodes': 10, 'qualified': 4, 'samples_per_s': 3000},
                                optimization=[.1, .2, 1, 2, .001], log_std=[0] * 4)
        self.assertEqual(data['isaac_proxy/qualified_rate'], .4)
        self.assertEqual(data['ppo/critic_loss'], .2)
        self.assertAlmostEqual(data['ppo/raw_gaussian_entropy_nats'], 2 * math.log(2 * math.pi * math.e))
        self.assertFalse(any(k.startswith('mujoco_independent/') for k in data))

    @unittest.skipUnless((RUN / 'frontier-selected.json').exists(), 'requires preserved local evidence')
    def test_actual_frontier_keeps_every_trial_and_physical_reasons(self):
        data = evaluation_metrics(json.loads((RUN / 'frontier-selected.json').read_text()),
                                  source='mujoco_independent', controller='ppo')
        prefix = 'mujoco_independent/ppo/frontier/'
        self.assertEqual(data[prefix + 'episodes'], 20)
        self.assertEqual(data[prefix + 'qualified_contacts'], 20)
        self.assertLess(data[prefix + 'contact_normal_speed_m_s_max'], 1.)
        rejected = evaluation_metrics(json.loads((RUN / 'frontier-trim.json').read_text()),
                                      source='mujoco_independent', controller='constant_control')
        self.assertEqual(rejected['mujoco_independent/constant_control/frontier/episodes'], 20)
        self.assertEqual(rejected['mujoco_independent/constant_control/frontier/qualified_contacts'], 0)
        self.assertEqual(rejected['mujoco_independent/constant_control/frontier/rejections/CONTACT_NORMAL_SPEED'], 20)

    @unittest.skipUnless((RUN / 'flight-hover.json').exists(), 'requires preserved local evidence')
    def test_flight_failure_never_counts_as_contact(self):
        data = evaluation_metrics(json.loads((RUN / 'flight-hover.json').read_text()),
                                  source='mujoco_independent', controller='ppo')
        prefix = 'mujoco_independent/ppo/hover/'
        self.assertEqual(data[prefix + 'episodes'], 19)
        self.assertEqual(data[prefix + 'flight_passes'], 0)
        self.assertEqual(data[prefix + 'safety_exits'], 19)
        self.assertFalse(any('qualified_contact' in k for k in data))

    @unittest.skipUnless((ROOT / '.agents/runs/rm27-stage1/ppo-contact/baseline-hold.json').exists(),
                         'requires preserved local hold evidence')
    def test_native_hold_is_flight_success_not_failed_contact(self):
        path = ROOT / '.agents/runs/rm27-stage1/ppo-contact/baseline-hold.json'
        summary = json.loads(path.read_text())
        data = evaluation_metrics(summary, source='mujoco_independent', controller='track_control')
        prefix = 'mujoco_independent/track_control/hold_regression/'
        self.assertEqual(data[prefix + 'flight_passes'], summary['success_count'])
        self.assertEqual(data[prefix + 'flight_passes'], 7)
        self.assertFalse(any('qualified_contact' in key or 'CONTACT_TIMEOUT' in key for key in data))

    def test_evaluations_own_distinct_monitor_status_locations(self):
        from simpleflight_lab.monitoring import publish_evaluation
        with patch.dict('os.environ', {'SF_WANDB': '1'}), patch('simpleflight_lab.monitoring.Monitor') as monitor:
            for name in ('frontier', 'hover'):
                publish_evaluation({'episodes': 1}, Path('/research/results') / name,
                                   source='mujoco_independent', controller='ppo')
        paths = [entry.args[0] for entry in monitor.call_args_list]
        self.assertEqual(paths, [Path('/research/results/monitoring/frontier'), Path('/research/results/monitoring/hover')])

    def test_unconfigured_endpoint_does_not_contact_dashboard_or_cloud(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'settings.json'
            config.write_text(json.dumps({'base_url': None}))
            with patch.dict('os.environ', {}, clear=True), patch('simpleflight_lab.monitoring.urlopen') as request:
                result = probe_service(config)
            self.assertEqual(result['status'], 'UNCONFIGURED_ENDPOINT')
            request.assert_not_called()

    def test_credential_in_endpoint_is_rejected_without_network(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'settings.json'
            config.write_text(json.dumps({'base_url': 'https://user:private@example.invalid'}))
            with patch.dict('os.environ', {}, clear=True), patch('simpleflight_lab.monitoring.urlopen') as request:
                result = probe_service(config)
            self.assertEqual(result['status'], 'INVALID_ENDPOINT')
            self.assertNotIn('endpoint', result)
            request.assert_not_called()

    def test_unknown_provenance_is_rejected(self):
        with self.assertRaises(ValueError):
            evaluation_metrics({'episodes': 1}, source='combined', controller='ppo')


if __name__ == '__main__':
    unittest.main()
