"""Synthetic scalar counterexamples and analytical budgets, never real VIO data."""
import unittest
import model_review as m

class ReviewTest(unittest.TestCase):
    def test_metadata_layout_and_total(self):
        r=m.profile(200,460800)
        self.assertEqual(r['payload_sizes'],{'vehicle_imu':60,'sensor_selection':16,'timesync_status':44})
        self.assertEqual(r['minimum_bytes_s'],16780)
        self.assertEqual(r['max_stuffed_bytes_s'],33340)
        self.assertAlmostEqual(r['planning_bytes_s'],20334.72)
        self.assertLess(r['max_utilization'],1)
    def test_limits_and_400hz_headroom(self):
        self.assertGreater(m.profile(200,115200)['minimum_utilization'],1)
        self.assertGreater(m.profile(400,460800)['max_utilization'],1)
        self.assertLess(m.profile(400,921600)['max_utilization'],1)
    def test_constant_control(self):
        r=m.scalar_case(1,0,[0,5000,10000])
        self.assertAlmostEqual(r['retained_interval_delta_rad'],.005)
        self.assertAlmostEqual(r['endpoint_point_model_delta_rad'],.005)
        self.assertFalse(r['openvins_admissible'])
    def test_linear_ramp_counterexample(self):
        r=m.scalar_case(0,100,[0,5000,10000])
        self.assertAlmostEqual(r['retained_interval_delta_rad'],.00375)
        self.assertAlmostEqual(r['endpoint_point_model_delta_rad'],.0025)
        self.assertAlmostEqual(r['difference_rad'],-.00125)
        self.assertFalse(r['openvins_admissible'])
    def test_unequal_intervals(self):
        r=m.scalar_case(0,100,[0,4000,10000])
        self.assertAlmostEqual(r['retained_interval_delta_rad'],.0042)
        self.assertAlmostEqual(r['endpoint_point_model_delta_rad'],.0027)
    def test_synthetic_queue_one_stall_overwrites(self):
        # Hypothetical 10ms subscriber pause; NOT a measured PX4 scheduler trace.
        from collections import deque
        latest=deque(maxlen=1)
        for publication in (5000,10000):latest.append(publication)
        self.assertEqual(list(latest),[10000])
        self.assertNotIn(5000,latest)
    def test_refuse_invalid_model_inputs(self):
        for ts in ([0,0,5000],[0,10000,5000],[0.,5000,10000],[0,5000],[-1,5000,10000]):
            with self.assertRaises(ValueError):m.scalar_case(0,100,ts)
        with self.assertRaises(ValueError):m.scalar_case(float('nan'),0,[0,5000,10000])
        with self.assertRaises(ValueError):m.profile(True,460800)

if __name__=='__main__':unittest.main()
