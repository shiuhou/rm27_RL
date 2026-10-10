"""Transport/estimator contracts, independent of a running firmware instance."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from rm27.experiments.aggressive_contact.flight_control.estimator import TelemetryEstimator,B
from rm27.experiments.aggressive_contact.flight_control.sitl import SpinMapping
from rm27.experiments.aggressive_contact.teacher_interface import attitude_ned_frd


def position(ms,**kw):
    return dict(mavpackettype='LOCAL_POSITION_NED',time_boot_ms=ms,x=1.,y=2.,z=-3.,vx=.1,vy=.2,vz=-.3,**kw)


def attitude(ms,angle=0.,omega=(0.,0.,0.)):
    q=Rotation.from_rotvec([0.,0.,angle]).as_quat()
    return dict(mavpackettype='ATTITUDE_QUATERNION',time_boot_ms=ms,q1=q[3],q2=q[0],q3=q[1],q4=q[2],
                rollspeed=omega[0],pitchspeed=omega[1],yawspeed=omega[2])


def initialized():
    e=TelemetryEstimator([10.,20.,30.]);e.consume(attitude(1000),1.007);e.consume(position(1000),1.009);e.lock_clock();return e


def test_telemetry_frame_and_registration():
    e=initialized();e.local_registration=np.array([.5,1.,-1.]);s=e.observation(1.02).estimate
    np.testing.assert_allclose(s.position,[10.5,19.,32.]);np.testing.assert_allclose(s.velocity,[.1,-.2,.3])
    np.testing.assert_allclose(s.rotation,np.eye(3));assert abs(s.timestamp-1.007)<1e-12
    R=Rotation.from_euler('xyz',[.3,-.2,.4]).as_matrix();q=attitude_ned_frd(R)
    m=attitude(1020,omega=(1.,2.,3.));m.update(q1=q[0],q2=q[1],q3=q[2],q4=q[3])
    e.consume(m,1.03);e.consume(position(1020),1.03);s=e.observation(1.03).estimate
    np.testing.assert_allclose(s.rotation,R,atol=1e-14);np.testing.assert_allclose(s.omega_body,[1.,-2.,-3.])


def test_async_attitude_interpolated_at_velocity_acquisition_time():
    e=initialized();e.consume(attitude(1020,.2,(0,0,1)),1.03);e.consume(position(1010),1.035)
    s=e.observation(1.035).estimate
    np.testing.assert_allclose(s.rotation,B@Rotation.from_rotvec([0,0,.1]).as_matrix()@B,atol=1e-14)
    assert abs(s.timestamp-1.017)<1e-12
    e.consume(attitude(1030,.3),1.04)
    assert e.observation(1.04).estimate is s # IMU receipt cannot freshen held velocity


def test_telemetry_dropouts_and_reordering_fail_closed():
    e=initialized();e.observation(1.02);e.consume(position(990),1.03);e.consume(position(1000),1.04)
    assert e.rejected==2 and e.position[0]==1.
    with pytest.raises(ValueError,match='age'):e.observation(1.2)
    e.consume(position(1100),1.11)
    with pytest.raises(ValueError,match='Attitude too old'):e.observation(1.12)


def test_invalid_telemetry_does_not_poison_clock_or_sequence():
    e=initialized();p=position(2000);p['vx']=float('nan');e.consume(p,1.02)
    a=attitude(2000);a['q1']=0.;e.consume(a,1.02)
    assert e.rejected==2 and e.last_boot['LOCAL_POSITION_NED']==1.
    e.consume(attitude(1020),1.03);e.consume(position(1020),1.03)
    assert abs(e.observation(1.03).estimate.timestamp-1.027)<1e-12


def test_gyro_extrapolation_respects_alignment_limit():
    e=TelemetryEstimator([0,0,0]);e.consume(attitude(1000,omega=(0,0,2)),1.01);e.lock_clock()
    e.consume(position(1020),1.04)
    np.testing.assert_allclose(e.observation(1.04).estimate.rotation,Rotation.from_rotvec([0,0,-.04]).as_matrix())
    e.consume(position(1030),1.05)
    with pytest.raises(ValueError,match='Attitude too old'):e.observation(1.05)


def test_spin_inverse_is_only_the_selected_nominal_force_mapping():
    f=4*1.103625;m=SpinMapping(f)
    for u in (0.,.2,.5,1.):assert abs(m(f*(.15+.8*u))-u)<1e-14
    assert m(0.)==0. # armed spin floor; not a zero-force or emergency-stop command
