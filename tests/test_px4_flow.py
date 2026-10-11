"""Optical-flow sensor wire convention and absence of external vision."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from rm27.autopilot.px4.flow import FlowSensorProducer


def sensor():
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
    <geom name="floor" type="plane" size="10 10 .1"/>
    <body name="drone_1" pos="0 0 1"><freejoint/>
    <geom type="sphere" size=".02" mass=".15" group="5"/>
    </body></worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    state=dict(timestamp=0.,position=[0,0,-1],quaternion=[1,0,0,0],velocity=[.2,.1,0],imu=dict(gyro=[.05,.06,0],accel_body=[0,0,-9.81]))
    plant=SimpleNamespace(model=model,data=data,body=model.body('drone_1').id,state=lambda:state)
    producer=FlowSensorProducer(plant);producer.rng=SimpleNamespace(normal=lambda mean,std,size=None:0. if size is None else np.zeros(size))
    return producer,state


def test_flow_sensor_sign_integration_and_gyro_compensation():
    flow,s=sensor()
    assert flow.optical_flow() is None
    for i in range(1,6):
        s['timestamp']=i*.004;packet=flow.optical_flow()
    assert packet.quality==255 and packet.integration_time_us==20000
    # EKF negates both sensor integrals. Its compensated LOS is (+vy,-vx).
    compensated=-np.array([packet.integrated_x,packet.integrated_y])/.02+np.array([.05,.06])
    np.testing.assert_allclose(compensated,[.1,-.2],atol=1e-9)
    assert packet.distance==pytest.approx(1.)
    with pytest.raises(ValueError,match='Nonmonotonic'):flow.optical_flow()
    with pytest.raises(RuntimeError,match='forbidden'):flow.odometry()


def test_flow_quality_rejects_missing_surface_and_close_range():
    flow,s=sensor();flow.optical_flow()
    flow.plant.data.qpos[2]=.02;mujoco.mj_forward(flow.plant.model,flow.plant.data)
    s['timestamp']=.02
    # Near-ground numerical flow exceeds the angular-rate envelope even
    # though this simulation model resolves the short range.
    assert flow.optical_flow().quality==0
    flow.plant.data.qpos[2]=5.;mujoco.mj_forward(flow.plant.model,flow.plant.data)
    s['timestamp']=.04
    packet=flow.optical_flow();assert packet.quality==0 and packet.distance==-1


def test_magnetic_field_is_rotated_to_body_and_enabled():
    flow,s=sensor()
    message=flow.hil_sensor()
    np.testing.assert_allclose([message.xmag,message.ymag,message.zmag],[.22,0,.43])
    s['quaternion']=[2**-.5,0,0,2**-.5]
    message=flow.hil_sensor()
    np.testing.assert_allclose([message.xmag,message.ymag,message.zmag],[0,-.22,.43],atol=1e-9)
    assert message.fields_updated & 0x1c0==0x1c0
