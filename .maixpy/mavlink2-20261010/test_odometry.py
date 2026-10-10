import unittest
from pymavlink.dialects.v20 import common as m
import mavlink_odometry_test as t

class TestOdometry(unittest.TestCase):
    def test_packet(self):
        enc=m.MAVLink(None,srcSystem=1,srcComponent=191)
        msg=t.odometry(enc,123456)
        wire=msg.pack(enc)
        self.assertEqual(wire[0],253)
        decoded=m.MAVLink(None).parse_char(wire)
        self.assertEqual(decoded.get_msgId(),331)
        self.assertEqual((decoded.get_srcSystem(),decoded.get_srcComponent()),(1,191))
        self.assertEqual(decoded.time_usec,123456)
        self.assertEqual((decoded.frame_id,decoded.child_frame_id),(m.MAV_FRAME_LOCAL_NED,m.MAV_FRAME_BODY_FRD))
        self.assertEqual((decoded.x,decoded.y,decoded.z),(1,2,-.5))
        self.assertEqual(decoded.q,[1,0,0,0])
        self.assertEqual((decoded.vx,decoded.vy,decoded.vz,decoded.rollspeed,decoded.pitchspeed,decoded.yawspeed),(0,)*6)
        self.assertEqual(decoded.estimator_type,m.MAV_ESTIMATOR_TYPE_VISION)
        self.assertEqual(decoded.quality,100)
        for cov in (decoded.pose_covariance,decoded.velocity_covariance):
            self.assertEqual(len(cov),21)
            for i,v in enumerate(cov):
                self.assertAlmostEqual(v,.01 if i in (0,6,11) else .0025 if i in (15,18,20) else 0)
    def test_safety(self):
        self.assertFalse(t.safe_heartbeat(None,10))
        self.assertFalse(t.safe_heartbeat((5,0),10))
        self.assertFalse(t.safe_heartbeat((9,128),10))
        self.assertTrue(t.safe_heartbeat((9,0),10))

if __name__=='__main__': unittest.main()
