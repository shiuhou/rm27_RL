"""UART2 only; 120 one-Hz HEARTBEATs, no other messages or pinmux changes."""
import copy
import os
import signal
import struct
import time
from probe import crc, owners, emit

def frame_v2(seq):
    os.environ['MAVLINK20']='1'
    from pymavlink.dialects.v20 import common as mav2
    encoder=mav2.MAVLink(None,srcSystem=1,srcComponent=191)
    encoder.seq=seq & 255
    packet=encoder.heartbeat_encode(mav2.MAV_TYPE_ONBOARD_CONTROLLER,
        mav2.MAV_AUTOPILOT_INVALID,0,0,mav2.MAV_STATE_UNINIT,3).pack(encoder,force_mavlink1=False)
    if packet[0]!=253: raise RuntimeError('MAVLink2 framing required')
    return packet

def frame(seq):
    payload=struct.pack('<IBBBBB',0,18,8,0,0,3)
    body=bytes([9,seq & 255,1,191,0])+payload
    return b'\xfe'+body+struct.pack('<H',crc(body+b'\x32'))

def main(seconds=120,mavlink2=False):
    if seconds not in (120,180): raise ValueError('Only bounded 120/180 second tests')
    version=None
    if mavlink2:
        os.environ['MAVLINK20']='1'
        import pymavlink
        version=pymavlink.__version__
        frame_v2(0)
    import termios
    import fcntl
    path='/dev/ttyS2'
    if owners(path): raise RuntimeError('UART2 busy; no process stopped')
    with open('/proc/consoles') as f:
        if any(line.split()[0]=='ttyS2' for line in f if line.strip()):
            raise RuntimeError('UART2 is console')
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    old=None
    try:
        fcntl.ioctl(fd,termios.TIOCEXCL)
        old=termios.tcgetattr(fd)
        new=copy.deepcopy(old)
        new[0]=new[1]=new[3]=0
        new[2]=(new[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS))|termios.CS8|termios.CREAD|termios.CLOCAL
        new[4]=new[5]=termios.B115200
        new[6][termios.VMIN]=new[6][termios.VTIME]=0
        termios.tcsetattr(fd,termios.TCSANOW,new)
        applied=termios.tcgetattr(fd)
        emit(event='termios_readback',iflag=applied[0],oflag=applied[1],cflag=applied[2],lflag=applied[3],speeds=applied[4:6])
        # Kernel normalizes baud bits in cflag; compare the requested semantics.
        mask=termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS|termios.CREAD|termios.CLOCAL
        if (applied[0]!=0 or applied[1]!=0 or applied[3]!=0 or
                applied[4:6]!=[termios.B115200]*2 or
                applied[2]&mask != (termios.CS8|termios.CREAD|termios.CLOCAL)):
            raise RuntimeError('UART settings readback mismatch')
        emit(event='start',port=path,baud=115200,seconds=seconds,sysid=1,compid=191,
             mavlink_version=2 if mavlink2 else 1,pymavlink_version=version,heartbeat_hz=1)
        start=time.monotonic()
        for seq in range(seconds):
            remaining=start+seq-time.monotonic()
            if remaining>0: time.sleep(remaining)
            data=frame_v2(seq) if mavlink2 else frame(seq)
            n=os.write(fd,data)
            if n!=len(data): raise RuntimeError('Partial write; no retry')
            emit(event='heartbeat_written',seq=seq,count=seq+1,bytes=n,wire=data.hex())
        time.sleep(max(0,start+seconds-time.monotonic()))
    finally:
        try:
            if old is not None:
                termios.tcsetattr(fd,termios.TCSANOW,old)
                emit(event='restored',termios_equal=termios.tcgetattr(fd)==old)
        finally:
            fcntl.ioctl(fd,termios.TIOCNXCL)
            os.close(fd)

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--mavlink2',action='store_true')
    parser.add_argument('--seconds',type=int,choices=(120,180),default=120)
    args=parser.parse_args()
    main(args.seconds,args.mavlink2)
