"""Isolated local PX4 process and HIL connection; context manager owns cleanup."""
import hashlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import socket
import subprocess
import time
import struct
from collections import deque
from pymavlink.dialects.v20 import common as mavlink
from .build import DEFAULT_SOURCE, verify
from .bridge import ActuatorDecoder, SensorProducer

class Writer:
    def __init__(self, sock, address=None): self.sock,self.address=sock,address
    def write(self, data):
        return self.sock.sendto(data,self.address) if self.address else self.sock.sendall(data)

class Runtime:
    def __init__(self, plant, output, instance=0, source=DEFAULT_SOURCE, profile='diagnostic'):
        if type(instance) is not int or not 0<=instance<=99: raise ValueError('instance must be 0..99')
        self.plant,self.output,self.instance=plant,Path(output),instance
        self.source=verify(source); self.profile=profile
        self.binary=self.source/'build/px4_sitl_default/bin/px4'
        self.process=None; self.server=self.sim=self.api=self.log=None
        self.decoder=ActuatorDecoder(); self.sensor=SensorProducer(plant)
        self.last_odom=-1.; self.telemetry={}; self.parameters={}; self.events=[]
        self.sim_port=4560+instance; self.api_port=18000+instance*2; self.host_port=self.api_port+1
        self.system=instance+1; self.started=None
        self.pending=deque()
        self.control_period=.004
        self.substeps=round(self.control_period/self.plant.model.opt.timestep)
        if abs(self.substeps*self.plant.model.opt.timestep-self.control_period)>1e-10:
            raise ValueError('Physics timestep must divide the 250 Hz HIL cycle')

    def __enter__(self):
        try:
            etc=self.output/'etc'; shutil.copytree(self.source/'build/px4_sitl_default/etc',etc,symlinks=False)
            params=json.loads((Path(__file__).resolve().parents[3]/'configs/px4'/f'{self.profile}.json').read_text())
            mass=float(self.plant.model.body_mass[self.plant.body])
            import numpy as np
            maximum=float(np.asarray(self.plant.maximum).max())
            params['MPC_THR_HOVER']=mass*9.81/(4*maximum)
            airframe=etc/'init.d-posix/airframes/4999_rm27'
            airframe.write_text('. ${R}etc/init.d/rc.mc_defaults\n'+''.join(f'param set {k} {v}\n' for k,v in params.items()))
            startup=etc/'init.d-posix/px4-rc.mavlink'
            startup.write_text(f'mavlink start -x -u {self.api_port} -o {self.host_port} -t 127.0.0.1 -r 4000000 -m onboard\n'+''.join(f'mavlink stream -u {self.api_port} -r {rate} -s {name}\n' for name,rate in [('HEARTBEAT',5),('LOCAL_POSITION_NED',50),('ATTITUDE_QUATERNION',100),('ESTIMATOR_STATUS',10),('EXTENDED_SYS_STATE',10),('DISTANCE_SENSOR',50)]))
            (etc/'init.d-posix/px4-rc.mavlinksim').write_text(f'simulator_mavlink start -t 127.0.0.1 {self.sim_port}\n')
            self.expected=params
            self.server=socket.socket(); self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            self.server.bind(('127.0.0.1',self.sim_port)); self.server.listen(1); self.server.settimeout(30)
            self.api=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.api.bind(('127.0.0.1',self.host_port)); self.api.setblocking(False)
            self.mav=mavlink.MAVLink(Writer(self.api,('127.0.0.1',self.api_port)),srcSystem=255,srcComponent=191)
            self.api_parser=mavlink.MAVLink(None); self.api_parser.robust_parsing=True
            self.sim_parser=mavlink.MAVLink(None); self.sim_parser.robust_parsing=True
            env={k:v for k,v in os.environ.items() if not k.startswith('PX4_')}
            env.update(PX4_SYS_AUTOSTART='4999',PX4_SIM_MODEL='rm27',PX4_SIMULATOR='mavlink',PX4_SIM_HOST_ADDR='127.0.0.1')
            command=[str(self.binary),'-i',str(self.instance),'-d','-s',str(etc/'init.d-posix/rcS')]
            self.log=(self.output/'sitl.log').open('w')
            self.process=subprocess.Popen(command,cwd=self.output,env=env,stdin=subprocess.DEVNULL,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            self.started=time.monotonic()
            self.sim,_=self.server.accept(); self.sim.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.hil=mavlink.MAVLink(Writer(self.sim),srcSystem=255,srcComponent=51)
            (self.output/'launch.json').write_text(json.dumps({'command':command,'parameters':params,'binary_sha256':hashlib.sha256(self.binary.read_bytes()).hexdigest(),'ports':{'sim':self.sim_port,'api':self.api_port,'host':self.host_port},'source':str(self.source),'profile':self.profile},indent=2))
            self.hil.send(self.sensor.hil_sensor())
            self.hil.send(self.sensor.distance_sensor())
            return self
        except BaseException:
            self.close(); raise

    def command(self, code, *values):
        self.mav.command_long_send(self.system,1,code,0,*(list(values)+[0.]*(7-len(values))))
    def mode(self, mode):
        if mode == 'LAND':
            self.land()
            return
        main,sub={'OFFBOARD':(6,0),'LAND':(4,6)}[mode]
        self.command(mavlink.MAV_CMD_DO_SET_MODE,1,main,sub)
    def land(self):
        """Request PX4's normal autonomous landing at the current position."""
        self.command(mavlink.MAV_CMD_NAV_LAND,0,0,0,float('nan'),float('nan'),float('nan'),float('nan'))
    def position(self, position, yaw=0., velocity=None):
        velocity=[0.,0.,0.] if velocity is None else velocity
        self.mav.set_position_target_local_ned_send(int(self.plant.data.time*1000),self.system,1,
            mavlink.MAV_FRAME_LOCAL_NED,0b100111000000,*position,*velocity,0,0,0,yaw,0)
    def heartbeat(self):
        self.mav.heartbeat_send(mavlink.MAV_TYPE_GCS,mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
    def receive(self):
        messages=[]; now=float(self.plant.data.time)
        while select.select([self.api],[],[],0)[0]:
            raw,address=self.api.recvfrom(65536)
            if address[0]!='127.0.0.1': continue
            for msg in self.api_parser.parse_buffer(raw) or []:
                if msg.get_srcSystem()!=self.system or msg.get_srcComponent()!=1:continue
                messages.append(msg)
                kind=msg.get_type()
                if kind=='HEARTBEAT':
                    main=(msg.custom_mode>>16)&255;sub=(msg.custom_mode>>24)&255
                    self.telemetry.update(armed=bool(msg.base_mode&128),mode='OFFBOARD' if main==6 else 'LAND' if main==4 and sub==6 else f'{main}:{sub}',heartbeat_time=now)
                elif kind=='LOCAL_POSITION_NED':
                    self.telemetry.update(local_position=[msg.x,msg.y,msg.z],local_velocity=[msg.vx,msg.vy,msg.vz],local_time=now)
                elif kind=='EXTENDED_SYS_STATE':self.telemetry.update(landed_state=msg.landed_state,landed_time=now)
                elif kind=='DISTANCE_SENSOR':self.telemetry.update(distance_m=float(msg.current_distance)*.01,distance_time=now)
                elif kind=='COMMAND_ACK':
                    self.telemetry.setdefault('command_ack',{})[msg.command]={'result':msg.result,'time':now}
                elif kind=='ESTIMATOR_STATUS':self.telemetry.update(estimator_flags=msg.flags,estimator_time=now)
                elif kind=='ATTITUDE_QUATERNION':self.telemetry.update(quaternion=[msg.q1,msg.q2,msg.q3,msg.q4],rates=[msg.rollspeed,msg.pitchspeed,msg.yawspeed],attitude_time=now)
                elif kind=='PARAM_VALUE':self.parameters[msg.param_id]=self.decode_param(msg)
        return messages

    @staticmethod
    def decode_param(msg):
        """Decode PX4's MAVLink parameter union, including integer bitcasts."""
        value = float(msg.param_value)
        typ = int(msg.param_type)
        if typ == mavlink.MAV_PARAM_TYPE_REAL32:
            return value
        raw = struct.pack('<f', value)
        if typ == mavlink.MAV_PARAM_TYPE_UINT8:return struct.unpack('<B', raw[:1])[0]
        if typ == mavlink.MAV_PARAM_TYPE_INT8:return struct.unpack('<b', raw[:1])[0]
        if typ == mavlink.MAV_PARAM_TYPE_UINT16:return struct.unpack('<H', raw[:2])[0]
        if typ == mavlink.MAV_PARAM_TYPE_INT16:return struct.unpack('<h', raw[:2])[0]
        if typ == mavlink.MAV_PARAM_TYPE_UINT32:return struct.unpack('<I', raw)[0]
        if typ == mavlink.MAV_PARAM_TYPE_INT32:return struct.unpack('<i', raw)[0]
        return value

    def step(self, timeout=5.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.process.poll() is not None:raise RuntimeError('PX4 exited; see sitl.log')
            now=float(self.plant.data.time)
            if now-self.last_odom>=.02-1e-8:
                self.hil.send(self.sensor.odometry()); self.last_odom=now
            if not self.pending and not select.select([self.sim],[],[],.05)[0]:
                # Initial module startup requires clock progression before output exists.
                if self.decoder.frames==0:
                    self.advance([0.,0.,0.,0.]);self.hil.send(self.sensor.hil_sensor())
                continue
            if not self.pending:
                raw=self.sim.recv(65536)
                if not raw:raise RuntimeError('Simulator connection closed')
                self.pending.extend(msg for msg in self.sim_parser.parse_buffer(raw) or [] if msg.get_type()=='HIL_ACTUATOR_CONTROLS')
            if self.pending:
                msg=self.pending.popleft()
                controls=self.decoder.decode(msg)
                self.advance(controls)
                self.hil.send(self.sensor.hil_sensor())
                self.hil.send(self.sensor.distance_sensor())
                return {'timestamp':float(self.plant.data.time),'actuator_time_usec':msg.time_usec,'controls':controls.tolist(),'state':self.plant.state(),'truth':self.plant.truth()}
        self.diagnostics()
        raise TimeoutError('PX4 actuator stream timed out')

    def diagnostics(self):
        with (self.output/'timeout-diagnostics.log').open('a') as log:
            for module,args in [('sensors',['status']),('ekf2',['status']),('mc_rate_control',['status']),
                                ('listener',['sensor_gyro','-n','1']),('listener',['sensor_combined','-n','1']),
                                ('listener',['actuator_motors','-n','1'])]:
                log.write('\n'+module+' '+str(args)+'\n');log.flush()
                try:subprocess.run([str(self.binary.parent/('px4-'+module)),'--instance',str(self.instance),*args],stdout=log,stderr=log,timeout=3)
                except subprocess.TimeoutExpired:log.write('diagnostic timeout\n')

    def advance(self, controls):
        # ContactPlant already advances two native 1 ms steps per invocation.
        steps=2 if hasattr(self.plant,'fixture') else self.substeps
        for _ in range(steps):self.plant.step_normalized(controls)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5)
        for sock in (self.sim,self.server,self.api):
            if sock is not None:sock.close()
        if self.log:self.log.close()
    def __exit__(self,*args):self.close()
