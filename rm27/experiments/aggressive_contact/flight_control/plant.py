"""Current single research UAV/arena coupled to existing 500-Hz JSON protocol.

Two 1-ms MuJoCo steps per servo packet, preserving the research physics step.
An explicitly logged initialization fixture holds the airborne start ONLY during
EKF/arming/spool-up. Release is irreversible, no mission teleport or truth servo.
"""
import numpy as np
import mujoco
from rm27.autopilot.bridge.frames import to_ap,attitude_to_ap,specific_force


class ContactPlant:
    def __init__(self,bundle):
        self.model,self.data,self.info,self.xml=bundle
        self.body=self.model.body('uav').id
        self.origin=self.data.qpos[:3].copy();self.initial=self.data.qpos.copy()
        self.maximum=self.info['max_rotor_thrust_N_assumed'];self.fixture=True;self.released_at=None
        self.resets=0;self.model.opt.timestep=.001
    def release(self):
        if not self.fixture:raise ValueError('Already released')
        self.fixture=False;self.released_at=float(self.data.time)
    def reset(self):
        self.resets+=1
        raise RuntimeError('SITL reset during a flight is forbidden')
    def step(self,pwm):
        values=np.asarray(pwm[:4],float)
        return self.step_normalized(np.clip((values[[2,0,3,1]]-1000)/1000,0,1))
    def step_normalized(self,values):
        values=np.asarray(values,float)
        if values.shape!=(4,) or not np.isfinite(values).all() or np.any((values<0)|(values>1)):
            raise ValueError('Expected four finite force fractions in [0,1]')
        self.data.ctrl[:]=values*self.maximum
        for _ in range(2):
            mujoco.mj_step(self.model,self.data)
            if self.fixture:
                self.data.qpos[:]=self.initial;self.data.qvel[:]=0
            mujoco.mj_forward(self.model,self.data)
        return self.state()
    def state(self):
        d=self.data;R=d.xmat[self.body].reshape(3,3)
        acc=np.zeros(3) if self.fixture else d.qacc[:3]
        return dict(timestamp=float(d.time),imu=dict(gyro=to_ap(d.qvel[3:]).tolist(),accel_body=specific_force(R,acc,self.model.opt.gravity).tolist()),
                    position=to_ap(d.qpos[:3]-self.origin).tolist(),velocity=to_ap(d.qvel[:3]).tolist(),quaternion=attitude_to_ap(d.qpos[3:7]).tolist())
    def truth(self):
        return dict(position_nwu=self.data.qpos[:3].tolist(),velocity_nwu=self.data.qvel[:3].tolist(),quaternion_wxyz=self.data.qpos[3:7].tolist(),
                    body_rate_flu=self.data.qvel[3:].tolist(),motor_state_N=self.data.act.tolist(),motor_command_N=self.data.ctrl.tolist(),fixture=self.fixture)
