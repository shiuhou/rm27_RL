"""MAVLink telemetry -> timestamped policy observation. No plant state access.

AP LOCAL_POSITION_NED and ATTITUDE_QUATERNION share time_boot_ms. Receipt time
is logged separately. A fixed clock offset is calibrated during initialization;
that does not measure one-way transport latency exactly. World registration is
an explicit task input, never an ongoing truth-position correction.
"""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from ..model import VehicleEstimate
from ..distillation.deployment import VehicleObservation

B=np.diag([1.,-1.,-1.])


class TelemetryEstimator:
    def __init__(self,world_origin_nwu,velocity_std=.05,max_age_s=.1):
        self.origin=np.array(world_origin_nwu,float);self.velocity_std=velocity_std;self.max_age=max_age_s
        self.offset=None;self.offset_samples=[];self.locked=False;self.position=None;self.attitudes=deque(maxlen=100)
        self.last=None;self.last_pos_time=-np.inf;self.local_registration=np.zeros(3)
        self.rejected=0;self.receipts={};self.last_boot={}
    def lock_clock(self):
        if not self.offset_samples:raise ValueError('No telemetry clock samples')
        self.offset=float(min(self.offset_samples));self.locked=True
    def consume(self,message,receipt_s):
        kind=message.get('mavpackettype')
        if kind not in ('LOCAL_POSITION_NED','ATTITUDE_QUATERNION'):return
        t=float(message['time_boot_ms'])*.001
        # Explicitly reject reordering / reboot. A new process needs a new estimator.
        if not np.isfinite([t,receipt_s]).all() or t<=self.last_boot.get(kind,-np.inf):
            self.rejected+=1;return
        if kind=='LOCAL_POSITION_NED':
            p=np.array([message[x] for x in ('x','y','z')]);v=np.array([message[x] for x in ('vx','vy','vz')])
            if not np.isfinite(np.r_[p,v]).all():self.rejected+=1;return
            self.position=(t,p,v)
        else:
            q=np.array([message[x] for x in ('q2','q3','q4','q1')]);w=np.array([message[x] for x in ('rollspeed','pitchspeed','yawspeed')])
            if not np.isfinite(np.r_[q,w]).all() or abs(np.linalg.norm(q)-1)>.02:self.rejected+=1;return
            R=B@Rotation.from_quat(q).as_matrix()@B
            self.attitudes.append((t,R,B@w))
        # Malformed packets must not poison clock calibration or sequence state.
        self.last_boot[kind]=t
        if not self.locked:
            self.offset_samples.append(receipt_s-t);self.offset=min(self.offset_samples)
        self.receipts[kind]=receipt_s
    def observation(self,now):
        if self.position is None or not self.attitudes or self.offset is None:raise ValueError('Incomplete telemetry')
        tp,p,v=self.position
        # Publish each position/velocity acquisition once, with attitude aligned
        # from already-received samples. Do not stamp held velocity as fresh IMU.
        if tp>self.last_pos_time+1e-9:
            before=[x for x in self.attitudes if x[0]<=tp+1e-9]
            after=[x for x in self.attitudes if x[0]>tp+1e-9]
            if before:
                t,R,w=before[-1]
                if after:
                    t2,R2,w2=after[0];alpha=(tp-t)/(t2-t)
                    R=Slerp([t,t2],Rotation.from_matrix([R,R2]))([tp]).as_matrix()[0];w=(1-alpha)*w+alpha*w2
                elif tp-t<=.025:R=R@Rotation.from_rotvec(w*(tp-t)).as_matrix()
                else:raise ValueError('Attitude too old for position sample')
            else:
                t,R,w=self.attitudes[0]
                if t-tp>.025:raise ValueError('No aligned attitude')
                R=R@Rotation.from_rotvec(w*(tp-t)).as_matrix()
            self.last=VehicleEstimate(self.origin+B@(p-self.local_registration),B@v,R,w,tp+self.offset,'ARDUPILOT_EKF_TELEMETRY')
            self.last_pos_time=tp
        age=now-self.last.timestamp
        if age<-.005 or age>self.max_age:raise ValueError(f'Telemetry age outside contract: {age:.6f}')
        return VehicleObservation(self.last,np.eye(3)*self.velocity_std**2)
