"""Synthetic protocol/timing tests, never hardware qualification."""
import importlib.util
from pathlib import Path
import unittest
from pymavlink.dialects.v20 import common as m
import imu_input as t

def frame(seq=0, stamp=1000000):
    enc=m.MAVLink(None,srcSystem=1,srcComponent=1); enc.seq=seq
    return enc.highres_imu_encode(stamp,1,2,-9.8,.1,.2,.3,0,0,0,0,0,0,20,63).pack(enc)

class InputTest(unittest.TestCase):
    def test_split_crc_source_and_timestamp(self):
        p=t.Decoder(); wire=frame(254)
        self.assertEqual(p.feed(wire[:5],100),[])
        r=p.feed(wire[5:],200)[0]
        self.assertEqual((r['sysid'],r['compid'],r['seq']),(1,1,254))
        self.assertEqual(r['fields']['time_usec'],1000000)
        self.assertEqual(r['receipt_mono_ns'],200)
        self.assertEqual(bytes.fromhex(r['wire_hex']),wire)
    def test_corruption_not_silently_valid(self):
        p=t.Decoder(); bad=bytearray(frame()); bad[-1]^=1
        rows=p.feed(bytes(bad)+frame(1),100)
        self.assertTrue(any(r['event']=='bad_data' for r in rows))
        self.assertEqual(sum(r['event']=='frame' for r in rows),1)
    def test_unknown_dialect_uses_wire_header_not_zero_defaults(self):
        # Actual captured opaque CURRENT_MODE frame; CRC cannot be checked by old dialect.
        row=t.Decoder().feed(bytes.fromhex('fd0200004201019b01004701edc0'),1)[0]
        self.assertEqual((row['sysid'],row['compid'],row['seq'],row['msgid']),(1,1,66,411))
        self.assertEqual(row['crc_status'],'UNKNOWN_DIALECT_UNVERIFIED')
        self.assertEqual(t.Decoder().feed(frame(),1)[0]['crc_status'],'VERIFIED')
    def test_sequence_wrap_and_cross_message_not_sensor_loss(self):
        p=t.Decoder(); rows=[]
        for i,seq in enumerate((254,255,0,2,2,1)):
            rows+=p.feed(frame(seq,1000000+i*10000),1000000000+i*10000000)
        report=t.summarize(rows)
        seq=report['sources']['1:1']['sequence']
        self.assertEqual(seq['forward_missing_estimate'],1)
        self.assertEqual(seq['duplicates'],1)
        self.assertEqual(seq['backward_or_reset'],1)
        self.assertIsNone(report['sensor_loss_count'])
    def test_intervals_and_disorder(self):
        stats=t.intervals([100,110,120,160,160,155],1e6)
        self.assertEqual(stats['duplicates'],1)
        self.assertEqual(stats['backwards'],1)
        self.assertEqual(stats['gap_events_gt_1_5_median'],1)
        self.assertEqual(stats['max_interval'],40)
        self.assertIsNone(stats['effective_hz'])
        self.assertIsNone(t.intervals([],1e6)['effective_hz'])
    def test_semantics_units_and_unqualified_clock(self):
        row=t.Decoder().feed(frame(),123)[0]
        x=t.imu_record(row,'session1')
        self.assertEqual(x['gyro_unit'],'rad/s')
        self.assertEqual(x['accel_unit'],'m/s^2')
        self.assertEqual(x['timestamp_semantic'],'GYRO_INTEGRATION_END')
        self.assertIsNone(x['mapped_ns'])
        self.assertIsNone(x['hardware_device_id'])
        self.assertFalse(x['vio_admissible'])
    def test_other_source_cannot_inherit_px4_semantics(self):
        row=t.Decoder().feed(frame(),123)[0]; row['sysid']=2
        with self.assertRaises(ValueError): t.imu_record(row,'s')
    def test_scaled_quantization_and_publication_time(self):
        e=m.MAVLink(None,srcSystem=1,srcComponent=1)
        msg=e.scaled_imu_encode(123,1000,0,-1000,100,-200,300,0,0,0)
        r=t.Decoder().feed(msg.pack(e),123)[0]
        x=t.imu_record(r,'s')
        self.assertEqual(x['timestamp_semantic'],'PUBLICATION')
        self.assertEqual(x['accel_si'],[9.80665,0,-9.80665])
        self.assertEqual(x['gyro_si'],[.1,-.2,.3])
        self.assertEqual(x['source_unit'],'ms')
    def test_processed_telemetry_rejected_by_existing_camera_interface(self):
        path=Path(__file__).parents[1]/'offline-vio-20261009/rm27/perception/vision/localization/experiments/vio_dataset.py'
        if not path.exists(): self.skipTest('Host-only existing dataset interface')
        spec=importlib.util.spec_from_file_location('existing_vio',path)
        v=importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
        raw=t.imu_record(t.Decoder().feed(frame(),1)[0],'s')
        with self.assertRaises(ValueError):
            v.map_timestamp(raw['source_timestamp'],{'evidence_status':'UNKNOWN'},source_epoch='s',source_domain='PX4_HRT',source_unit='us')
        row=dict(mapped_ns=10,target_domain='FC',target_epoch='s',uncertainty_ns=0,
                 evidence_sha256='a'*64,semantic_status='VERIFIED',semantic=raw['timestamp_semantic'])
        camera=dict(row,semantic='MID_EXPOSURE')
        with self.assertRaises(ValueError):
            v.bracket_mapped_streams([camera],[row],[row],max_gap_ns=10,max_uncertainty_ns=10)
    def test_bandwidth(self):
        self.assertAlmostEqual(t.wire_budget(62,100,115200)['utilization'],7400/11520)
        self.assertGreater(t.wire_budget(62,200,115200)['utilization'],1)
        self.assertEqual(t.wire_budget(24,1600,115200)['wire_bytes_per_second'],57600)

if __name__=='__main__': unittest.main()
