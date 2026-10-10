"""Synthetic fixtures only, except separately labeled saved-console audit."""
import copy
import importlib.util
from pathlib import Path
import unittest
import offline as o

def row(t=10000):
    return dict(timestamp=t+100,timestamp_sample=t,accel_device_id=6946834,gyro_device_id=6684690,
        delta_angle=[.005,0,0],delta_velocity=[0,0,-.04903325],delta_angle_dt=5000,delta_velocity_dt=5000,
        delta_angle_clipping=0,delta_velocity_clipping=0,accel_calibration_count=1,gyro_calibration_count=1)

class OfflineTest(unittest.TestCase):
    def test_means_are_not_points(self):
        r=o.diagnostic(row(),units=o.UNITS,epoch='synthetic')
        self.assertEqual(r['gyro_mean_rad_s'],[1,0,0])
        self.assertAlmostEqual(r['accel_mean_m_s2'][2],-9.80665)
        self.assertIsNone(r['accel_end_us']);self.assertFalse(r['openvins_admissible'])
        self.assertEqual(r['measurement_semantic'],'CALIBRATED_CONING_INTEGRAL_MEAN')
    def test_units_and_invalid_data_rejected(self):
        for k,v in [('delta_angle_dt',0),('delta_velocity_dt',True),('timestamp_sample',1.2),
                    ('gyro_device_id',0),('delta_angle',[float('nan'),0,0]),('delta_velocity',[1,2])]:
            x=row();x[k]=v
            with self.subTest(k=k),self.assertRaises(ValueError):o.diagnostic(x,units=o.UNITS,epoch='s')
        with self.assertRaises(ValueError):o.diagnostic(row(),units={**o.UNITS,'dt':'ms'},epoch='s')
    def test_wire_ranges_and_derived_overflow_rejected(self):
        for k,v in [('timestamp',2**64),('gyro_device_id',2**32),
                    ('delta_velocity_dt',2**32),('delta_angle',[1e308,0,0])]:
            x=row();x[k]=v
            with self.subTest(k=k),self.assertRaises(ValueError):o.diagnostic(x,units=o.UNITS,epoch='s')
    def test_segment_events_do_not_become_missing_sample_counts(self):
        r=o.audit([dict(row(),gyro_calibration_count=255),
                   dict(row(15000),gyro_calibration_count=0)],units=o.UNITS,epoch='s')
        self.assertIn('calibration',r['issues'])
        self.assertFalse(r['openvins_admissible'])
        overlap=o.audit([row(),row(14000)],units=o.UNITS,epoch='s')
        self.assertIn('overlap',overlap['issues'])
        with self.assertRaises(ValueError):o.audit([],units=o.UNITS,epoch='s')
    def test_gap_order_clip_calibration_identity(self):
        for kind,rows in [('gap',[row(),row(20000)]),('ordering',[row(),row()]),
            ('calibration',[row(),dict(row(15000),accel_calibration_count=2)]),
            ('identity',[row(),dict(row(15000),gyro_device_id=3604506)]),
            ('clipping',[dict(row(),delta_angle_clipping=1),row(15000)])]:
            self.assertIn(kind,o.audit(rows,units=o.UNITS,epoch='s')['issues'])
        self.assertEqual(o.audit([row(),row(15000)],units=o.UNITS,epoch='s')['issues'],[])
    def test_existing_admission_still_rejects(self):
        p=Path(__file__).parents[1]/'offline-vio-20261009/rm27/perception/vision/localization/experiments/vio_dataset.py'
        spec=importlib.util.spec_from_file_location('contract',p);v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
        x=dict(mapped_ns=100,target_domain='synthetic',target_epoch='s',uncertainty_ns=0,
            evidence_sha256='a'*64,semantic_status='VERIFIED',semantic='CALIBRATED_CONING_INTEGRAL_MEAN')
        with self.assertRaises(ValueError):v.bracket_mapped_streams([dict(x,semantic='MID_EXPOSURE')],[x],[x],max_gap_ns=100,max_uncertainty_ns=0)
    def test_frame_cost_bounds(self):
        b=o.xrce_budget(60,200,460800)
        self.assertEqual(b['minimum_bytes_s'],15800)
        self.assertEqual(b['max_stuffed_bytes_s'],31400)
        self.assertLess(b['max_utilization'],1)
        self.assertGreater(o.xrce_budget(60,200,115200)['minimum_utilization'],1)
    def test_cdr_layout_not_memory_padding(self):
        base=Path(__file__).parents[1]
        self.assertEqual(o.cdr_size(base/'imu-input-20261010/msg__VehicleImu.msg'),60)
        self.assertEqual(o.cdr_size(base/'imu-transport-20261010/msg__SensorGyro.msg'),44)
        self.assertEqual(o.cdr_size(base/'imu-input-20261010/msg__SensorCombined.msg'),48)
    def test_unobservable_signal(self):
        # Two different instantaneous signals have the same interval integral.
        import math
        n=1000;dt=.005
        vals=[math.sin(2*math.pi*(i+.5)/n) for i in range(n)]
        self.assertAlmostEqual(sum(vals)*dt/n,0,places=12)
        self.assertGreater(max(vals),.99)
        self.assertEqual(o.diagnostic(dict(row(),delta_angle=[0,0,0]),units=o.UNITS,epoch='s')['gyro_mean_rad_s'],[0,0,0])

if __name__=='__main__':unittest.main()
