import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest

@unittest.skipUnless(os.name=='posix','Linux pseudo-terminal safety test')
class ReceiveTest(unittest.TestCase):
    def run_case(self,armed):
        import pty
        import select
        import termios
        from pymavlink.dialects.v20 import common as m
        import receive_imu as r
        master,slave=pty.openpty(); path=os.ttyname(slave)
        old=termios.tcgetattr(slave)
        # Close slave so production ownership check is meaningful.
        os.close(slave)
        enc=m.MAVLink(None,srcSystem=1,srcComponent=1)
        wire=enc.heartbeat_encode(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,128 if armed else 0,0,0,3).pack(enc)
        output=None
        def source():
            # Wait for the capture's actual ready marker, not a scheduling guess.
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                if output is not None and output.exists() and '"event": "start"' in output.read_text():
                    os.write(master,wire); return
                time.sleep(.005)
        worker=threading.Thread(target=source); worker.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                output=Path(tmp)/'capture.jsonl'
                if armed:
                    with self.assertRaisesRegex(RuntimeError,'ARMED'): r.capture(path,output,.25,True)
                else: r.capture(path,output,.25,True)
                records=[json.loads(s) for s in output.read_text().splitlines()]
                self.assertEqual(records[-1]['event'],'closed')
                self.assertTrue(records[-1]['termios_restored'])
                self.assertEqual(termios.tcgetattr(master),old)
                # Slave writes/echo would become readable on master (EOF/EIO is not data).
                if select.select([master],[],[],0)[0]:
                    try: reply=os.read(master,4096)
                    except OSError: reply=b''
                    self.assertEqual(reply,b'')
        finally:
            worker.join(); os.close(master)
    def test_disarmed_receive_no_tx_and_restore(self): self.run_case(False)
    def test_armed_abort_and_restore(self): self.run_case(True)

if __name__=='__main__': unittest.main()
