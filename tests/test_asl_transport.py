"""Synthetic transport contract tests, not estimator qualification."""
import importlib.util
from pathlib import Path
import unittest
import tempfile
import zipfile
import importlib.util as util


class TransportContract(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / 'tools/openvins/asl_transport.py'
        self.assertTrue(path.is_file(), 'ASL transport implementation missing')
        spec = importlib.util.spec_from_file_location('asl_transport', path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_integer_nanoseconds_round_trip(self):
        ns = 1403715273262142977
        sec, nano = self.module.split_stamp(ns)
        self.assertEqual(sec * 10**9 + nano, ns)

    def test_no_float_or_bool_timestamp(self):
        for value in [1403715273.25, True, -1]:
            with self.assertRaises(ValueError):
                self.module.split_stamp(value)

    def test_camera_filename_must_match(self):
        with self.assertRaises(ValueError):
            self.module.parse_rows('1,2.png\n', 'cam0')

    def test_duplicate_and_reversed_are_rejected(self):
        for text in ['2,2.png\n2,2.png\n', '2,2.png\n1,1.png\n']:
            with self.assertRaises(ValueError):
                self.module.parse_rows(text, 'cam0')

    def test_nonfinite_imu_rejected(self):
        with self.assertRaises(ValueError):
            self.module.parse_rows('1,0,0,nan,0,0,0\n', 'imu0')

    def test_merge_stable_imu_first_on_ties(self):
        imu = self.module.parse_rows('#header\n1,1,2,3,4,5,6\n2,1,2,3,4,5,6\n', 'imu0')
        cam = self.module.parse_rows('1,1.png\n3,3.png\n', 'cam0')
        events = self.module.merge_events(cam, imu)
        self.assertEqual([(t, topic) for t, topic, _ in events],
                         [(1, 'imu0'), (1, 'cam0'), (2, 'imu0'), (3, 'cam0')])

    @unittest.skipUnless(util.find_spec('rosbag2_py'), 'ROS2 integration requires sourced ROS2 environment')
    def test_actual_ros2_roundtrip_and_no_overwrite(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            archive = root / 'synthetic.zip'
            ns = 1403715273262142977
            pixels = np.arange(12, dtype=np.uint8).reshape(3, 4)
            ok, png = cv2.imencode('.png', pixels)
            self.assertTrue(ok)
            with zipfile.ZipFile(archive, 'w') as z:
                z.writestr('mav0/cam0/data.csv', f'{ns},{ns}.png\n')
                z.writestr('mav0/cam0/data/{ns}.png'.format(ns=ns), png.tobytes())
                z.writestr('mav0/imu0/data.csv', f'{ns},.1,-.2,.3,4,5,6\n')
            result = self.module.build_bag(archive, root / 'bag')
            self.assertEqual(result['verified_counts'], {'cam0': 1, 'imu0': 1})
            self.assertTrue(result['readback_exact'])
            with self.assertRaises(FileExistsError):
                self.module.build_bag(archive, root / 'bag')


if __name__ == '__main__':
    unittest.main()
