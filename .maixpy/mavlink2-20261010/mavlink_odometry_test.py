"""SYNTHETIC BENCH ONLY. No estimator, commands, setpoints or parameter writes.

Require externally read-back EKF2_EV_CTRL=0 and propellers removed before start.
M3C monotonic timestamps are transport-test timestamps, NOT synchronized VIO time.
"""
import os
os.environ['MAVLINK20']='1'
import argparse
import copy
import glob
import json
import signal
import time
from pymavlink.dialects.v20 import common as m

def emit(**kw):
    print(json.dumps(kw),flush=True)

def odometry(encoder,stamp):
    # Upper triangle x,y,z,roll,pitch,yaw: independent test uncertainties.
    cov=[0.0]*21
    for i in (0,6,11): cov[i]=0.01
    for i in (15,18,20): cov[i]=0.0025
    return encoder.odometry_encode(stamp,m.MAV_FRAME_LOCAL_NED,m.MAV_FRAME_BODY_FRD,
        1.0,2.0,-0.5,[1.0,0.0,0.0,0.0],0,0,0,0,0,0,cov,list(cov),
        reset_counter=0,estimator_type=m.MAV_ESTIMATOR_TYPE_VISION,quality=100)

def safe_heartbeat(hb,now):
    return hb is not None and 0 <= now-hb[0] <= 2.5 and not hb[1]&128

def main():
    import fcntl
    import select
    import termios
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=int,choices=range(10,181),default=45)
    p.add_argument('--bench-ev-disabled-confirmed',action='store_true',required=True)
    args=p.parse_args()
    path='/dev/ttyS2'
    if any(os.path.realpath(f)==path for f in glob.glob('/proc/[0-9]*/fd/*')):
        raise RuntimeError('Port owned; no process stopped')
    with open('/proc/consoles') as f:
        if any(line.split()[0]=='ttyS2' for line in f if line.strip()): raise RuntimeError('Console port')
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    old=None
    hb=None
    parser=m.MAVLink(None)
    parser.robust_parsing=True
    enc=m.MAVLink(None,srcSystem=1,srcComponent=191)
    counts={'HEARTBEAT':0,'ODOMETRY':0}
    def receive():
        nonlocal hb
        if select.select([fd],[],[],.01)[0]:
            for msg in parser.parse_buffer(os.read(fd,8192)) or []:
                if (msg.get_type()=='HEARTBEAT' and msg.get_srcSystem()==1 and
                        msg.get_srcComponent()==1 and msg.autopilot==m.MAV_AUTOPILOT_PX4):
                    hb=(time.monotonic(),msg.base_mode)
                    if msg.base_mode&m.MAV_MODE_FLAG_SAFETY_ARMED: raise RuntimeError('ARMED: abort')
    def send(msg):
        wire=msg.pack(enc,force_mavlink1=False)
        if wire[0]!=253: raise RuntimeError('Not MAVLink2')
        if os.write(fd,wire)!=len(wire): raise RuntimeError('Partial write; no retry')
        enc.seq=(enc.seq+1)&255
        counts[msg.get_type()]+=1
    try:
        fcntl.ioctl(fd,termios.TIOCEXCL)
        old=termios.tcgetattr(fd)
        new=copy.deepcopy(old)
        new[0]=new[1]=new[3]=0
        mask=termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS|termios.CREAD|termios.CLOCAL
        new[2]=(new[2]&~mask)|termios.CS8|termios.CREAD|termios.CLOCAL
        new[4]=new[5]=termios.B115200
        new[6][termios.VMIN]=new[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,new)
        applied=termios.tcgetattr(fd)
        if (any(applied[i]!=0 for i in (0,1,3)) or applied[4:6]!=[termios.B115200]*2 or
            applied[2]&mask != termios.CS8|termios.CREAD|termios.CLOCAL): raise RuntimeError('Settings mismatch')
        deadline=time.monotonic()+5
        while hb is None and time.monotonic()<deadline: receive()
        if not safe_heartbeat(hb,time.monotonic()): raise RuntimeError('No fresh disarmed PX4 heartbeat')
        start=time.monotonic()
        next_hb=next_odom=start
        emit(event='start',synthetic=True,device=path,baud=115200,mavlink=2,sysid=1,compid=191,
             heartbeat_hz=1,odometry_hz=10,frame_id=m.MAV_FRAME_LOCAL_NED,
             child_frame_id=m.MAV_FRAME_BODY_FRD,seconds=args.seconds)
        while time.monotonic()<start+args.seconds:
            receive()
            now=time.monotonic()
            if not safe_heartbeat(hb,now): raise RuntimeError('Stale/armed PX4 heartbeat')
            if now>=next_hb:
                send(enc.heartbeat_encode(m.MAV_TYPE_ONBOARD_CONTROLLER,m.MAV_AUTOPILOT_INVALID,0,0,m.MAV_STATE_UNINIT,3))
                next_hb=now+1
                emit(event='counts',elapsed=now-start,**counts)
            if now>=next_odom:
                send(odometry(enc,time.monotonic_ns()//1000))
                next_odom+=.1
                if next_odom<now: next_odom=now+.1
        emit(event='finished',elapsed=time.monotonic()-start,**counts)
    finally:
        try:
            if old is not None:
                termios.tcsetattr(fd,termios.TCSANOW,old)
                emit(event='restored',equal=termios.tcgetattr(fd)==old)
        finally:
            fcntl.ioctl(fd,termios.TIOCNXCL)
            os.close(fd)

if __name__=='__main__': main()
