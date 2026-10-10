import unittest
from pymavlink.dialects.v10 import common as mav
import heartbeat_only as h

class HeartbeatTest(unittest.TestCase):
    def test_v2_wire_and_fields(self):
        from pymavlink.dialects.v20 import common as v2
        for seq in (0,1,179):
            wire=h.frame_v2(seq)
            self.assertEqual(wire[0],253)
            msg=v2.MAVLink(None).parse_char(wire)
            self.assertEqual(msg.get_type(),'HEARTBEAT')
            self.assertEqual((msg.get_srcSystem(),msg.get_srcComponent()),(1,191))
            self.assertEqual((msg.type,msg.autopilot,msg.base_mode,msg.custom_mode),(18,8,0,0))
            self.assertEqual(msg.get_seq(),seq)
    def test_wire(self):
        for seq in (0,1,119,255):
            m=mav.MAVLink(None,srcSystem=1,srcComponent=191)
            m.seq=seq
            expected=m.heartbeat_encode(18,8,0,0,0,3).pack(m)
            self.assertEqual(h.frame(seq),expected)

if __name__=='__main__': unittest.main()
