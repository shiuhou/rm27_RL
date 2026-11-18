"""CPU-only R9-A exchange tests; never construct a plant or invoke a policy."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from rm27_asm2.shared_handover import (
    PACKAGE_ROOT, RECEIPT_PATH, SharedScenarioConsumer,
)


class SharedHandoverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.consumer = SharedScenarioConsumer()
        cls.fixture = cls.consumer.validation.read_json(
            PACKAGE_ROOT / 'tests/contracts/fixtures/handover_synthetic.json')
        cls.geometry = cls.consumer.validation.read_json(
            PACKAGE_ROOT / 'contracts/v0.1.0/examples/valid/'
            'geometry-manifest-stable-patch-ids.json')

    def synthetic_target(self):
        """Explicit format-only test target, not an acquired scenario."""
        snapshot = deepcopy(self.fixture['snapshot'])
        registry = self.consumer.api.HandoverRegistry.from_dict(self.fixture['registry'])
        target = deepcopy(self.geometry['record']['target'])
        target['registration_epoch'] = registry.epoch
        target['target_frame_WT']['parent_frame_id'] = registry.frame.destination_id
        snapshot['target_geometry'] = {
            'schema': 'geometry.schema.json#/$defs/TargetGeometry', 'record': target}
        return snapshot, replace(registry, targets={target['target_id']: deepcopy(target)})

    def encode_fixture(self, snapshot, registry, now_s=10.01):
        return self.consumer.synthetic_actor_observation(
            snapshot, registry, now_s=now_s, goal_offset_m=.01786282)

    def test_published_records_replay_without_promoting_state_or_distribution(self):
        dataset = self.consumer.load()
        snapshots = self.consumer.replay()
        self.assertEqual(dataset['dataset_readiness'], 'BLOCKED')
        self.assertFalse(dataset['originals_verified'])
        self.assertEqual(len(snapshots), 3)
        self.assertEqual([s['snapshot_id'] for s in snapshots], [
            f'fixed-waypoints-half-height-001:line-{i}' for i in (1176, 1177, 1178)])
        self.assertEqual({r['split'] for r in dataset['records']}, {'dev'})
        self.assertEqual(len({r['source']['source_family_id'] for r in dataset['records']}), 1)
        for snapshot in snapshots:
            self.assertIsNone(snapshot['state'])
            restored = self.consumer.api.decode_snapshot(self.consumer.api.encode(snapshot))
            self.assertEqual(restored, snapshot)
            self.assertEqual(self.consumer.api.digest(restored), self.consumer.api.digest(snapshot))
            self.assertEqual(self.consumer.assess(snapshot)['actor_observation']['status'], 'BLOCKED')
            self.assertEqual(self.consumer.assess(snapshot)['physical_branching']['status'], 'BLOCKED')

    def test_replay_has_copy_ownership_and_does_not_mutate_acquired_evidence(self):
        first, second = self.consumer.replay(), self.consumer.replay()
        self.assertEqual(first, second)
        first[0]['native_record']['payload']['controls'][0] = 999
        first[0]['native_record']['registration']['clock_mapping'] = 'invented'
        self.assertEqual(self.consumer.replay(), second)
        self.assertNotEqual(first, second)

    def test_diagnostic_ground_truth_changes_cannot_create_actor_observations(self):
        snapshot = self.consumer.replay()[0]
        before = self.consumer.assess(snapshot)
        changed = deepcopy(snapshot)
        changed['native_record']['payload']['physical_state'] = {'position': [999, 999, 999]}
        after = self.consumer.assess(changed)
        self.assertEqual(before['actor_observation'], after['actor_observation'])
        self.assertEqual(after['actor_observation']['status'], 'BLOCKED')
        registry = self.consumer.api.HandoverRegistry.from_dict(self.fixture['registry'])
        with self.assertRaises(ValueError):
            self.encode_fixture(changed, registry)

    def test_missing_state_and_duplicate_or_nonfinite_json_fail_closed(self):
        snapshot = deepcopy(self.fixture['snapshot'])
        del snapshot['state']
        with self.assertRaises(ValueError):
            self.consumer.api.decode_snapshot(json.dumps(snapshot))
        for text in ('{"format":"a","format":"b"}', '{"value":NaN}', '{"value":Infinity}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.consumer.api.decode_snapshot(text)

    def test_imported_schema_and_python_bytes_must_match_acquisition(self):
        for relative in ('rm27_exchange/handover.py',
                         'contracts/v0.1.0/schemas/state.schema.json'):
            with self.subTest(relative=relative), TemporaryDirectory() as tmp:
                package = Path(tmp) / 'package'
                shutil.copytree(PACKAGE_ROOT, package)
                with (package / relative).open('a') as stream:
                    stream.write('\n')
                with self.assertRaises(ValueError):
                    SharedScenarioConsumer(package_root=package, receipt_path=RECEIPT_PATH)

    def test_rewritten_acquisition_receipt_cannot_repin_the_release(self):
        with TemporaryDirectory() as tmp:
            receipt = Path(tmp) / 'acquisition.json'
            changed = json.loads(RECEIPT_PATH.read_text())
            changed['source_revision'] = '0' * 40
            receipt.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                SharedScenarioConsumer(receipt_path=receipt)

    def test_unlisted_python_cannot_enter_the_pinned_import_package(self):
        with TemporaryDirectory() as tmp:
            package = Path(tmp) / 'package'
            shutil.copytree(PACKAGE_ROOT, package)
            (package / 'rm27_exchange/unlisted.py').write_text('unverified = True\n')
            with self.assertRaisesRegex(ValueError, 'unexpected code'):
                SharedScenarioConsumer(package_root=package)

    def test_preloaded_foreign_exchange_module_is_rejected(self):
        program = (
            'import sys,types; '
            'from rm27_asm2.shared_handover import SharedScenarioConsumer; '
            'foreign=types.ModuleType("rm27_exchange"); '
            'foreign.__file__="/tmp/foreign-g0/__init__.py"; '
            'sys.modules["rm27_exchange"]=foreign; '
            'SharedScenarioConsumer()'
        )
        result = subprocess.run([sys.executable, '-c', program], cwd=Path(__file__).resolve().parents[1],
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'),
            capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('foreign rm27_exchange module', result.stderr)

    def test_synthetic_target_is_required_and_never_establishes_physical_branching(self):
        snapshot = deepcopy(self.fixture['snapshot'])
        registry = self.consumer.api.HandoverRegistry.from_dict(self.fixture['registry'])
        self.assertIsNone(snapshot['target_geometry'])
        with self.assertRaises(ValueError):
            self.encode_fixture(snapshot, registry)
        snapshot, registry = self.synthetic_target()
        self.assertEqual(self.consumer.assess(snapshot)['physical_branching']['status'], 'BLOCKED')

    def test_exact_45d_actor_order_flu_and_target_centered_frame(self):
        snapshot, registry = self.synthetic_target()
        frame = replace(registry.frame,
            rotation_ned_native=((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)),
            translation_ned_m=(.3, -.4, .1))
        registry = replace(registry, frame=frame)
        observation = self.encode_fixture(snapshot, registry)
        self.assertEqual(observation.shape, (45,))
        self.assertEqual(observation.dtype, np.float32)
        # Analytic fixture result, including the nonzero world translation.
        expected = np.array(
            [-.2, -.1, .81786282] * 10 + [-.03, -.2, .01] +
            [0., -1., 0., -1., 0., 0., 0., 0., -1.] + [.1, .2, -.05],
            dtype=np.float32)
        np.testing.assert_allclose(observation, expected, rtol=0, atol=1e-7)
        np.testing.assert_array_equal(observation[:30].reshape(10, 3),
                                      np.tile(observation[:3], (10, 1)))

    def test_quaternion_sign_is_observation_invariant(self):
        snapshot, registry = self.synthetic_target()
        expected = self.encode_fixture(snapshot, registry)
        snapshot['state']['record']['q_WB_wxyz'] = [-1, 0, 0, 0]
        np.testing.assert_array_equal(self.encode_fixture(snapshot, registry), expected)

    def test_motor_history_is_not_an_actor_input_and_does_not_grant_branching(self):
        snapshot, registry = self.synthetic_target()
        expected = self.encode_fixture(snapshot, registry)
        del snapshot['state']['record']['actuator_state']
        snapshot['control_history'] = {
            'availability': {'status': 'UNKNOWN', 'reason': 'Not supplied in this negative fixture'},
            'records': []}
        np.testing.assert_array_equal(self.encode_fixture(snapshot, registry), expected)
        assessment = self.consumer.assess(snapshot)
        self.assertEqual(assessment['physical_branching']['status'], 'BLOCKED')
        self.assertIn('actual_rotor_thrust', assessment['missing'])
        self.assertIn('held_motor_target_and_timing', assessment['missing'])
        self.assertIn('control_history', assessment['missing'])

    def test_missing_actor_channels_or_unknown_validity_are_not_zero_filled(self):
        for channel in ('position_W_m', 'velocity_W_m_s', 'q_WB_wxyz', 'body_rates_B_rad_s'):
            for modification in ('remove', 'invalid'):
                snapshot, registry = self.synthetic_target()
                state = snapshot['state']['record']
                if modification == 'remove':
                    del state[channel]
                else:
                    state['validity'][channel]['current'] = None
                with self.subTest(channel=channel, modification=modification), self.assertRaises(ValueError):
                    self.encode_fixture(snapshot, registry)

    def test_runtime_ground_truth_and_synthetic_profiles_cannot_be_confused(self):
        snapshot, registry = self.synthetic_target()
        with self.assertRaises(ValueError):
            self.encode_fixture(snapshot, replace(registry, mode='RUNTIME_ESTIMATED'))
        truth = deepcopy(snapshot)
        truth['state']['schema'] = 'state.schema.json#/$defs/GroundTruthState'
        truth['state']['record']['metadata']['input_kind'] = 'GROUND_TRUTH'
        with self.assertRaises(ValueError):
            self.encode_fixture(truth, registry)
        contamination = deepcopy(snapshot)
        contamination['state']['record']['metadata']['lineage'].append(
            {'input_kind': 'GROUND_TRUTH', 'source': 'forbidden diagnostic sibling'})
        with self.assertRaises(ValueError):
            self.encode_fixture(contamination, registry)

    def test_unpublished_admission_mode_cannot_extend_the_pinned_contract(self):
        snapshot, registry = self.synthetic_target()
        with self.assertRaisesRegex(ValueError, 'unknown admission mode'):
            self.consumer.api.admit_snapshot(
                snapshot, replace(registry, mode='RECORDED_SIM_ESTIMATED'), now_s=10.01)

    def test_unregistered_frame_clock_epoch_or_body_origin_are_rejected(self):
        snapshot, registry = self.synthetic_target()
        alternatives = (
            replace(registry, epoch=registry.epoch + 1),
            replace(registry, frame=replace(registry.frame, destination_id='unknown-frame')),
            replace(registry, frame=replace(registry.frame, body_origin='UNKNOWN')),
            replace(registry, frame=replace(registry.frame, source_clock_id='unknown-clock')),
            replace(registry, clock=replace(registry.clock, destination_id='unknown-clock')),
            replace(registry, clock=replace(registry.clock, source_valid_s=(0., 1.))),
        )
        for changed in alternatives:
            with self.subTest(registry=changed), self.assertRaises(ValueError):
                self.encode_fixture(snapshot, changed)

    def test_future_stale_and_wrong_measurement_clocks_are_rejected(self):
        snapshot, registry = self.synthetic_target()
        for now in (9.999, 10.3):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.encode_fixture(snapshot, registry, now_s=now)
        for time, clock in ((9., registry.clock.destination_id),
                            (10.005, registry.clock.destination_id), (9.995, 'unknown-clock')):
            changed = deepcopy(snapshot)
            changed['state']['record']['measurement_times_s']['position_W_m'] = {
                'time_s': time, 'clock_id': clock}
            with self.subTest(time=time, clock=clock), self.assertRaises(ValueError):
                self.encode_fixture(changed, registry)

    def test_model_identity_and_registered_target_must_match(self):
        snapshot, registry = self.synthetic_target()
        for key in ('model', 'geometry', 'plant_profile'):
            changed = deepcopy(snapshot)
            changed['identity'][key]['sha256'] = '0' * 64
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.encode_fixture(changed, registry)
        with self.assertRaises(ValueError):
            self.encode_fixture(snapshot, replace(registry, targets={}))
        changed = deepcopy(snapshot)
        changed['target_geometry']['record']['target_frame_WT']['translation_parent_m'][0] += .001
        with self.assertRaises(ValueError):
            self.encode_fixture(changed, registry)

    def test_target_pose_normal_axes_epoch_and_motion_require_consistency(self):
        cases = [
            ('normal', lambda t: t.update(surface_normal_W=[1., 0., 0.])),
            ('quaternion', lambda t: t['target_frame_WT'].update(rotation_parent_child_wxyz=[2., 0., 0., 0.])),
            ('axes', lambda t: t.update(target_axes=['slope', 'across', 'normal'])),
            ('epoch', lambda t: t.update(registration_epoch=999)),
            ('moving', lambda t: t.update(motion='UNKNOWN')),
        ]
        for name, mutate in cases:
            snapshot, registry = self.synthetic_target()
            target = snapshot['target_geometry']['record']
            mutate(target)
            registry = replace(registry, targets={target['target_id']: deepcopy(target)})
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.encode_fixture(snapshot, registry)

    def test_same_navigation_family_cannot_leak_across_dataset_splits(self):
        with TemporaryDirectory() as tmp:
            package = Path(tmp)
            manifest = deepcopy(self.consumer.load()['manifest'])
            manifest['sources'].append(deepcopy(manifest['sources'][0]))
            manifest['sources'][1]['split'] = 'train'
            fixture_relative = manifest['sources'][0]['fixture_path']
            (package / fixture_relative).parent.mkdir(parents=True)
            shutil.copyfile(PACKAGE_ROOT / fixture_relative, package / fixture_relative)
            path = package / 'configs/handover_scenarios_v1.json'
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'source-family split leakage'):
                self.consumer.scenarios.load_scenario_set(package)

    def test_fixed_protocol_keeps_all_named_cases_and_reports_unevaluated_metrics(self):
        first = self.consumer.fixed_protocol()
        self.assertEqual(first, self.consumer.fixed_protocol())
        self.assertEqual(first['denominators'], {
            'catalogued': 3, 'eligible': 0, 'blocked': 3, 'evaluated': 0,
            'independent_held_out_families': 0})
        self.assertEqual(first['r9_b'], {
            'mechanism': 'UNSELECTED', 'training': 'NOT_AUTHORIZED', 'native': 'NOT_AUTHORIZED'})
        snapshots = self.consumer.replay()
        for case, snapshot in zip(first['cases'], snapshots):
            self.assertEqual(case['scenario_id'], snapshot['snapshot_id'])
            self.assertEqual(case['snapshot_sha256'], self.consumer.api.digest(snapshot))
            self.assertEqual(case['eligibility'], 'BLOCKED')
            self.assertEqual(set(case['metrics'].values()), {'NOT_EVALUATED'})
        first['cases'].clear()
        self.assertEqual(len(self.consumer.fixed_protocol()['cases']), 3)

    def test_loader_replay_stays_cpu_only_in_a_clean_process(self):
        program = (
            'import json,sys; '
            'from rm27_asm2.shared_handover import SharedScenarioConsumer; '
            'c=SharedScenarioConsumer(); c.replay(); '
            'print(json.dumps(sorted(n for n in sys.modules '
            'if n.split(".")[0] in {"torch","mujoco","isaacsim","omni"})))'
        )
        result = subprocess.run([sys.executable, '-c', program], cwd=Path(__file__).resolve().parents[1],
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'),
            capture_output=True, text=True, check=True, timeout=30)
        self.assertEqual(json.loads(result.stdout), [])


if __name__ == '__main__':
    unittest.main()
