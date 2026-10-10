"""Bounded PASSIVE UART capture. No MAVLink sender/requests or serial writes.

Reuse Stage A/B guard helpers and existing isolated pymavlink environment.
Changing temporary termios requires explicit --temporary-uart-approved.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import sys
import time
from imu_input import Decoder

# Host source layout and deployed layout both reuse the original probe helpers.
if (Path(__file__).parents[1]/'m3c-ping-20261009').exists():
    sys.path.insert(0,str(Path(__file__).parents[1]/'m3c-ping-20261009'))
from probe import owners, fresh_disarmed

def capture(path,output,seconds,temporary_approved=False):
    import fcntl
    import select
    import termios
    if not 0<seconds<=30: raise ValueError('Bounded capture <=30s')
    if owners(path): raise RuntimeError('UART occupied; no process killed')
    with open('/proc/consoles') as f:
        if any(row.split()[0]==os.path.basename(path) for row in f if row.strip()):
            raise RuntimeError('System console refused')
    # Refuse overwrite before opening hardware. Descriptor cannot write UART bytes.
    with Path(output).open('x',encoding='utf-8') as log:
        def emit(**row):
            log.write(json.dumps(row,allow_nan=False)+'\n'); log.flush()
        fd=os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        old=None; locked=False
        try:
            fcntl.ioctl(fd,termios.TIOCEXCL); locked=True
            old=termios.tcgetattr(fd)
            new=copy.deepcopy(old)
            new[0]=new[1]=new[3]=0
            mask=termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS|termios.CREAD|termios.CLOCAL
            new[2]=(new[2]&~mask)|termios.CS8|termios.CREAD|termios.CLOCAL
            new[4]=new[5]=termios.B115200
            new[6][termios.VMIN]=new[6][termios.VTIME]=0
            if new!=old and not temporary_approved: raise RuntimeError('Temporary UART settings not approved')
            termios.tcsetattr(fd,termios.TCSANOW,new)
            applied=termios.tcgetattr(fd)
            if any(applied[i]!=0 for i in (0,1,3)) or applied[4:6]!=[termios.B115200]*2 or applied[2]&mask != termios.CS8|termios.CREAD|termios.CLOCAL:
                raise RuntimeError('UART readback mismatch')
            # Discard pre-session stale/wrong-baud queued input; no evidence claim outside window.
            termios.tcflush(fd,termios.TCIFLUSH)
            start=time.monotonic_ns(); heartbeat=None; count=0
            emit(event='start',path=path,baud=115200,seconds=seconds,tx_enabled=False,
                 open_mode='O_RDONLY',monotonic_ns=start,pre_session_input_flushed=True,
                 temporary_setting_approved=temporary_approved,
                 old_termios=repr(old),applied_termios=repr(applied))
            decoder=Decoder()
            print(json.dumps(dict(event='receiving',seconds=seconds,tx_enabled=False)),flush=True)
            while (time.monotonic_ns()-start)/1e9<seconds:
                if select.select([fd],[],[],.05)[0]:
                    data=os.read(fd,8192); receipt=time.monotonic_ns()
                    if data:
                        count+=len(data)
                        emit(event='chunk',receipt_mono_ns=receipt,hex=data.hex())
                        decoded=decoder.feed(data,receipt)
                        # Preserve complete raw/decoded parity even if one frame triggers abort.
                        for row in decoded: emit(**row)
                        for row in decoded:
                            if row.get('type')=='HEARTBEAT' and (row['sysid'],row['compid'])==(1,1) and row['fields']['autopilot']==12:
                                heartbeat=(receipt/1e9,row['fields']['base_mode'])
                                if heartbeat[1]&128: raise RuntimeError('ARMED heartbeat; aborted')
                now=time.monotonic()
                if heartbeat is not None and not fresh_disarmed(heartbeat,now):
                    raise RuntimeError('Stale PX4 heartbeat; aborted')
                if heartbeat is None and now-start/1e9>5:
                    raise RuntimeError('No PX4 heartbeat within5s')
            if not fresh_disarmed(heartbeat,time.monotonic()): raise RuntimeError('No fresh Disarmed heartbeat')
            emit(event='finished',elapsed_s=(time.monotonic_ns()-start)/1e9,received_bytes=count,
                 last_heartbeat_base_mode=heartbeat[1],uart_bytes_written=0)
        except BaseException as exc:
            emit(event='aborted',error=type(exc).__name__,message=str(exc))
            raise
        finally:
            try:
                if old is not None:
                    termios.tcsetattr(fd,termios.TCSANOW,old)
                    restored=termios.tcgetattr(fd)==old
                    emit(event='closed',termios_restored=restored)
                    print(json.dumps(dict(event='closed',termios_restored=restored)),flush=True)
                    if not restored: raise RuntimeError('termios restoration mismatch')
            finally:
                try:
                    if locked: fcntl.ioctl(fd,termios.TIOCNXCL)
                finally: os.close(fd)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--seconds',type=int,choices=range(10,31),default=30)
    p.add_argument('--temporary-uart-approved',action='store_true')
    args=p.parse_args()
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    capture('/dev/ttyS2',args.output,args.seconds,args.temporary_uart_approved)

if __name__=='__main__': main()
