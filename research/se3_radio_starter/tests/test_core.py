from __future__ import annotations

import io
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import numpy as np

from core import (AxisCalibration, CHANNELS, InputGate, MotionSettings,
                  PositionReference, RadioProfile, demo_target, infer_axis,
                  quat_wxyz_to_xyzw)
from fetch_upstream import extract_archive


class CalibrationTests(unittest.TestCase):
    def test_neutral_deadzone_is_exact_zero(self):
        axis = AxisCalibration(0, 0.05, -0.9, 1.0, 1)
        self.assertEqual(axis.normalize(0.05), 0)
        self.assertEqual(axis.normalize(0.06), 0)

    def test_asymmetric_endpoint_scaling(self):
        axis = AxisCalibration(2, 0.1, -0.7, 1.0, 1, 0)
        self.assertAlmostEqual(axis.normalize(-0.3), -0.5)
        self.assertAlmostEqual(axis.normalize(0.55), 0.5)

    def test_sign_inversion_and_clamping(self):
        axis = AxisCalibration(0, 0, -1, 1, -1)
        self.assertEqual(axis.normalize(1), -1)
        self.assertEqual(axis.normalize(-2), 1)

    def test_deadzone_rescaling(self):
        axis = AxisCalibration(0, 0, -1, 1, 1, 0.1)
        self.assertAlmostEqual(axis.normalize(0.55), 0.5)
        self.assertAlmostEqual(axis.normalize(-0.55), -0.5)

    def test_bad_calibration_rejected(self):
        for values in [(0, -1, -1, 1, 1), (0, 0, -1, 1, 0), (0, float('nan'), -1, 1, 1)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                AxisCalibration(*values)

    def test_nonfinite_input_rejected(self):
        with self.assertRaises(ValueError):
            AxisCalibration(0, 0, -1, 1, 1).normalize(float('nan'))

    def test_axis_inference_inverted_direction(self):
        center = np.zeros(6)
        positive, negative = center.copy(), center.copy()
        positive[3], negative[3] = -0.9, 1
        axis = infer_axis(center, positive, negative, set())
        self.assertEqual((axis.index, axis.sign), (3, -1))
        self.assertEqual(axis.normalize(-0.9), 1)

    def test_duplicate_axis_rejected(self):
        with self.assertRaises(ValueError):
            infer_axis([0]*4, [1,0,0,0], [-1,0,0,0], {0})

    def test_simultaneous_axes_rejected(self):
        with self.assertRaises(ValueError):
            infer_axis([0]*4, [1,0.8,0,0], [-1,-0.8,0,0], set())

    def test_noncentered_throttle_rejected(self):
        with self.assertRaises(ValueError):
            infer_axis([0,0,-1,0], [0,0,1,0], [0,0,-1,0], set())

    def test_profile_round_trip_and_backup(self):
        profile = RadioProfile("Test", "GUID", 4,
                               {key: AxisCalibration(i,0,-1,1,1) for i,key in enumerate(CHANNELS)})
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "radio.json"
            profile.save(path)
            profile.save(path)
            reloaded = RadioProfile.load(path)
            self.assertTrue(path.with_suffix('.json.bak').exists())
            np.testing.assert_array_equal(reloaded.normalize([0,0,0,0]), np.zeros(4))
            reloaded.validate_device("GUID", 4)
            with self.assertRaises(ValueError):
                reloaded.validate_device("DIFFERENT", 4)

    def test_profile_refuses_duplicate_mappings(self):
        with self.assertRaises(ValueError):
            RadioProfile("bad", "x", 4, {key: AxisCalibration(0,0,-1,1,1) for key in CHANNELS})

    def test_profile_input_length_checked(self):
        profile = RadioProfile("Test", "GUID", 4,
                               {key: AxisCalibration(i,0,-1,1,1) for i,key in enumerate(CHANNELS)})
        with self.assertRaises(ValueError):
            profile.normalize([0,0,0])


class ReferenceTests(unittest.TestCase):
    def test_initial_heading_is_body_y(self):
        reference = PositionReference(MotionSettings())
        np.testing.assert_allclose(reference.heading, [0,1,0])

    def test_disabled_reference_does_not_move(self):
        reference = PositionReference(MotionSettings())
        old = reference.position.copy()
        for _ in range(100):
            reference.update([1,1,1,1], 0.001, False)
        np.testing.assert_array_equal(reference.position, old)

    def test_world_axis_mapping(self):
        reference = PositionReference(MotionSettings())
        for _ in range(500):
            reference.update([1,0,0,0], 0.001)
        self.assertGreater(reference.position[0], 0)
        self.assertEqual(reference.position[1], 0)
        self.assertEqual(reference.position[2], 0.3)

    def test_yaw_right_is_clockwise(self):
        reference = PositionReference(MotionSettings())
        for _ in range(1000):
            reference.update([0,0,0,1], 0.001)
        self.assertLess(reference.yaw, 0)
        self.assertGreater(reference.heading[0], 0)
        self.assertAlmostEqual(np.linalg.norm(reference.heading), 1)

    def test_diagonal_speed_is_limited(self):
        reference = PositionReference(MotionSettings())
        for _ in range(1000):
            reference.update([1,1,0,0], 0.001)
        self.assertLessEqual(np.linalg.norm(reference.velocity[:2]), 0.40000001)

    def test_target_limits_not_infinite_integration(self):
        reference = PositionReference(MotionSettings())
        for _ in range(1200):
            reference.update([1,1,1,0], 0.01)
        self.assertTrue(np.all(reference.position <= reference.upper))
        self.assertTrue(np.all(reference.position >= reference.lower))

    def test_reverse_away_from_limit(self):
        reference = PositionReference(MotionSettings())
        reference.position[0] = reference.upper[0]
        for _ in range(500):
            reference.update([1,0,0,0], 0.001)
        self.assertAlmostEqual(reference.velocity[0], 0)
        reference.update([-1,0,0,0], 0.001)
        self.assertLess(reference.position[0], reference.upper[0])

    def test_release_decelerates_then_holds(self):
        reference = PositionReference(MotionSettings())
        for _ in range(1000):
            reference.update([1,0,0,0], 0.001)
        for _ in range(1000):
            reference.update([0,0,0,0], 0.001)
        old = reference.position.copy()
        for _ in range(1000):
            reference.update([0,0,0,0], 0.001)
        np.testing.assert_array_equal(reference.position, old)
        np.testing.assert_allclose(reference.velocity, 0, atol=1e-12)

    def test_hold_immediate(self):
        reference = PositionReference(MotionSettings())
        reference.update([1,1,1,1], 0.01)
        old = reference.position.copy()
        reference.hold()
        np.testing.assert_array_equal(reference.position, old)
        np.testing.assert_array_equal(reference.velocity, [0,0,0])
        self.assertEqual(reference.yaw_rate, 0)

    def test_reset_restores_reference(self):
        reference = PositionReference(MotionSettings())
        reference.update([1,1,1,1], 0.01)
        reference.reset()
        np.testing.assert_array_equal(reference.position, [0,0,0.3])
        self.assertEqual(reference.yaw, 0)

    def test_acceleration_limit(self):
        reference = PositionReference(MotionSettings())
        reference.update([1,1,1,1], 0.01)
        self.assertLessEqual(np.linalg.norm(reference.velocity[:2]), 0.01000001)
        self.assertLessEqual(abs(reference.velocity[2]), 0.01000001)

    def test_bad_timestep_rejected(self):
        reference = PositionReference(MotionSettings())
        for dt in (0, -0.1, 1.0, float('nan')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                reference.update([0,0,0,0], dt)

    def test_settings_validation(self):
        with self.assertRaises(ValueError):
            MotionSettings(horizontal_speed=-1)
        with self.assertRaises(ValueError):
            MotionSettings(start_position=(0,0,10))

    def test_substep_partition_equivalence(self):
        a, b = PositionReference(MotionSettings()), PositionReference(MotionSettings())
        # Physics/reference cadence is identical regardless of render batch size.
        for _ in range(1000):
            a.update([0.5,0.3,0.2,-0.1], 0.001)
        for _ in range(100):
            for _ in range(10):
                b.update([0.5,0.3,0.2,-0.1], 0.001)
        np.testing.assert_array_equal(a.position, b.position)
        self.assertEqual(a.yaw, b.yaw)

    def test_demo_target_sequence(self):
        np.testing.assert_array_equal(demo_target(0), [0,0,0.3])
        np.testing.assert_array_equal(demo_target(7), [0.4,0,0.3])
        np.testing.assert_array_equal(demo_target(13), [0.4,0.4,0.3])
        np.testing.assert_array_equal(demo_target(24), [0,0,0.3])


class GateAndQuaternionTests(unittest.TestCase):
    def test_gate_requires_center(self):
        gate = InputGate()
        gate.toggle([0,0,-1,0])
        self.assertFalse(gate.enabled)
        gate.toggle([0,0,0,0])
        self.assertTrue(gate.enabled)

    def test_disconnect_is_latched(self):
        gate = InputGate()
        gate.toggle([0,0,0,0])
        gate.stop(disconnected=True)
        gate.toggle([0,0,0,0])
        self.assertFalse(gate.enabled)
        self.assertTrue(gate.disconnected)

    def test_gate_toggle_off_with_deflected_sticks(self):
        gate = InputGate(enabled=True)
        gate.toggle([1,1,1,1])
        self.assertFalse(gate.enabled)

    def test_quaternion_reordering(self):
        np.testing.assert_array_equal(quat_wxyz_to_xyzw([1,0,0,0]), [0,0,0,1])
        np.testing.assert_allclose(quat_wxyz_to_xyzw([2,0,0,2]), [0,0,2**-0.5,2**-0.5])
        with self.assertRaises(ValueError):
            quat_wxyz_to_xyzw([0,0,0,0])


class ArchiveTests(unittest.TestCase):
    @staticmethod
    def archive(extra=None):
        memory = io.BytesIO()
        with zipfile.ZipFile(memory, 'w') as archive:
            for name in ("se3_controller.py", "motor_mixer.py", "geometry.py", "crazyfile/scene.xml"):
                archive.writestr("repo-main/" + name, "fixture")
            if extra:
                archive.writestr(extra, "malicious fixture")
        return memory.getvalue()

    def test_normal_archive_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            dest = Path(temporary) / 'upstream'
            extract_archive(self.archive(), dest)
            self.assertTrue((dest / 'crazyfile/scene.xml').exists())
            with self.assertRaises(FileExistsError):
                extract_archive(self.archive(), dest)

    def test_parent_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            dest = Path(temporary) / 'upstream'
            with self.assertRaises(ValueError):
                extract_archive(self.archive('repo-main/../../escape'), dest)
            self.assertFalse(dest.exists())

    def test_case_collision_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                extract_archive(self.archive('repo-main/GEOMETRY.py'), Path(temporary)/'upstream')


if __name__ == '__main__':
    unittest.main()
