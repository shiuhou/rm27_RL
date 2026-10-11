"""Numerical downward optical-flow/range/compass sensors, without GPS or EV.

Physics state is confined to sensor production. This is an engineering sensor
model, not measured MTF-02P performance or an image-derived optical-flow claim.
The fixed magnetic field and seeded noise are explicit simulation assumptions.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from pymavlink.dialects.v20 import common as mavlink

from .bridge import SensorProducer


class FlowSensorProducer(SensorProducer):
    def __init__(self,plant,seed=170017):
        super().__init__(plant,seed)
        self.last_sample=None;self.interval=0.;self.flow_integral=np.zeros(2)
        self.gyro_integral=np.zeros(3);self.good=True
        self.mag_ned_gauss=np.array([.22,0.,.43])
        self.flow_noise_std_rad_s=.003
        # This numerical sensor can resolve the 2 cm dock start. Real MTF
        # close-range validity needs measurement and may need a distinct
        # attitude-controlled takeoff before flow becomes available.
        self.minimum_range_m=.01;self.maximum_range_m=4.
        self.maximum_tilt_deg=30.;self.maximum_flow_rad_s=4.

    def odometry(self):
        raise RuntimeError('External vision is forbidden in the flow-only profile')

    def hil_sensor(self):
        message=super().hil_sensor()
        q=np.asarray(self.plant.state()['quaternion'])
        rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        field=rotation.T@self.mag_ned_gauss+self.rng.normal(0,.0005,3)
        message.xmag,message.ymag,message.zmag=field
        message.fields_updated |= 0x1c0
        return message

    def optical_flow(self):
        s=self.plant.state();now=float(s['timestamp'])
        if self.last_sample is None:self.last_sample=now;return None
        dt=now-self.last_sample
        if dt<=0:raise ValueError('Nonmonotonic optical-flow sample time')
        self.last_sample=now
        q=np.asarray(s['quaternion']);r=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        v_body=r.T@np.asarray(s['velocity']);gyro=np.asarray(s['imu']['gyro'])
        height=self.distance_sensor().current_distance*.01
        slant=height/max(.01,r[2,2])
        valid=(self.minimum_range_m<=height<self.maximum_range_m
               and r[2,2]>=np.cos(np.deg2rad(self.maximum_tilt_deg)))
        # EKF2 uses negative sensor flow/gyro signs. After gyro compensation,
        # its LOS prediction is (v_body_y/range, -v_body_x/range).
        pixel_rate=np.array([-v_body[1],v_body[0]])/max(slant,.01)+gyro[:2]
        valid &= bool(np.linalg.norm(pixel_rate)<=self.maximum_flow_rad_s)
        self.flow_integral+=(pixel_rate+self.rng.normal(0,self.flow_noise_std_rad_s,2))*dt
        self.gyro_integral+=gyro*dt;self.interval+=dt;self.good &= valid
        if self.interval<.02-1e-8:return None
        message=mavlink.MAVLink_hil_optical_flow_message(
            max(1,round(now*1e6)),0,round(self.interval*1e6),*self.flow_integral,*self.gyro_integral,
            2000,255 if self.good else 0,0,float(slant) if valid else -1.)
        self.interval=0.;self.flow_integral[:]=0.;self.gyro_integral[:]=0.;self.good=True
        return message
