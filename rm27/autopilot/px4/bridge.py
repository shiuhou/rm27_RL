"""Strict HIL actuator decode and physical sensor production for PX4 SITL."""
import math
import numpy as np
from pymavlink.dialects.v20 import common as mavlink

# Custom allocator uses the SAME order as physical sites, unlike the AP bridge.
MOTOR_ORDER = (0, 1, 2, 3)

class ActuatorDecoder:
    def __init__(self):
        self.last_timestamp = None
        self.frames = 0
    def decode(self, message):
        if message.get_type() != 'HIL_ACTUATOR_CONTROLS':
            raise ValueError('Expected HIL_ACTUATOR_CONTROLS')
        stamp = int(message.time_usec)
        if stamp < 0 or (self.last_timestamp is not None and stamp <= self.last_timestamp):
            raise ValueError('Duplicate or reversed actuator timestamp')
        if not message.flags & 1:
            raise ValueError('PX4 lockstep flag missing')
        controls=np.asarray(message.controls[:4],float)
        if controls.shape!=(4,) or not np.isfinite(controls).all() or np.any((controls < -1e-6)|(controls>1+1e-6)):
            raise ValueError('Invalid motor control fraction')
        self.last_timestamp=stamp; self.frames+=1
        return np.clip(controls[list(MOTOR_ORDER)],0,1) if message.mode & 128 else np.zeros(4)

class SensorProducer:
    """Truth is confined to sensor generation. NED/FRD state supplied by plant."""
    def __init__(self, plant, seed=0):
        self.plant=plant
        self.rng=np.random.default_rng(seed)
        self.seed=seed
    def hil_sensor(self):
        s=self.plant.state(); z=-s['position'][2]
        # Pressure altitude relative to the local origin; temperature in Celsius.
        pressure=1013.25*max(.01,1-z/44330.)**5.255+self.rng.normal(0,.002)
        accel=np.asarray(s['imu']['accel_body'])+self.rng.normal(0,.003,3)
        gyro=np.asarray(s['imu']['gyro'])+self.rng.normal(0,.0003,3)
        return mavlink.MAVLink_hil_sensor_message(
            max(1,round(s['timestamp']*1e6)),*accel,*gyro,
            # PX4's simulator groups absolute pressure, pressure altitude and
            # temperature as 0x1a00; 0x1c3f omits absolute pressure.
            0.,0.,0.,pressure,0.,z,20.,0x1A3F,0)
    def odometry(self):
        s=self.plant.state()
        covariance=[0.]*21
        for i,v in zip((0,6,11,15,18,20),(.0001,.0001,.0001,.0001,.0001,.0001)):
            covariance[i]=v
        # LOCAL_FRD with fixed aligned NED axes is supported by PX4 external vision.
        # Velocity is transformed to body FRD; child_frame_id defines its basis.
        from scipy.spatial.transform import Rotation
        q=np.asarray(s['quaternion']); R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        velocity=R.T@np.asarray(s['velocity'])
        return mavlink.MAVLink_odometry_message(max(1,round(s['timestamp']*1e6)),
            mavlink.MAV_FRAME_LOCAL_NED,mavlink.MAV_FRAME_BODY_FRD,*s['position'],q,
            *velocity,*s['imu']['gyro'],covariance,covariance,0,mavlink.MAV_ESTIMATOR_TYPE_VISION,100)
    def distance_sensor(self):
        """Downward range in the MAVLink centimetre contract.

        The simulator converts this to metres and publishes a downward-facing
        range topic for EKF2 and the PX4 land detector.
        """
        z=max(0.01,float(-self.plant.state()['position'][2]))
        return mavlink.MAVLink_distance_sensor_message(
            max(1,round(float(self.plant.data.time)*1e3)),1,400,int(round(np.clip(z, .01, 4.)*100)),
            mavlink.MAV_DISTANCE_SENSOR_LASER,0,mavlink.MAV_SENSOR_ROTATION_PITCH_270,1,0.,0.,(0.,0.,0.,0.),100)
