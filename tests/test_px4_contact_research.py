from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27.experiments.aggressive_contact.flight_control.px4_sitl import ExternalNavigation, TeacherPolicy, run
from rm27.experiments.aggressive_contact.flight_control.estimator import TelemetryEstimator
from rm27.experiments.aggressive_contact.distillation.observation import ActuatorEstimate
from rm27.experiments.aggressive_contact.teacher_interface import RateThrust
from rm27.experiments.aggressive_contact.flight_control.batch import summarize


def test_px4_external_navigation_retains_delayed_source_and_body_velocity():
    data=SimpleNamespace(qpos=np.array([1.,2.,3.,1,0,0,0]),qvel=np.array([.1,.2,.3,.4,.5,.6]),
                         xmat=np.array([Rotation.from_euler('z',np.pi/2).as_matrix().ravel()]),time=1.)
    plant=SimpleNamespace(data=data,body=0,origin=np.zeros(3))
    sensor=ExternalNavigation(plant,dict(update_hz=50,delay_s=.03))
    packet=sensor.odometry()
    data.time=1.01
    assert sensor.odometry() is None  # A held acquisition is never restamped.
    data.time=1.04;sensor.odometry()
    data.time=1.08;packet=sensor.odometry()
    assert packet.time_usec==1_040_000
    np.testing.assert_allclose([packet.x,packet.y,packet.z],[1,-2,-3])
    np.testing.assert_allclose([packet.vx,packet.vy,packet.vz],[.2,.1,-.3],atol=1e-12)
    assert packet.child_frame_id==12 and packet.frame_id==1
    np.testing.assert_allclose([packet.rollspeed,packet.pitchspeed,packet.yawspeed],[.4,-.5,-.6])


def test_px4_research_rejects_ap_parameter_files_and_privileged_actor_mode(tmp_path):
    with pytest.raises(ValueError,match='AP .parm'):
        run(tmp_path/'flight',parameter_file='configs/contact_phase4/sitl.parm')
    with pytest.raises(ValueError,match='requires telemetry'):
        run(tmp_path/'flight',observation='truth')


def test_shared_estimator_preserves_explicit_px4_source():
    e=TelemetryEstimator([0,0,0],source_name='PX4_EKF2_TELEMETRY')
    e.consume(dict(mavpackettype='ATTITUDE_QUATERNION',time_boot_ms=1000,q1=1.,q2=0.,q3=0.,q4=0.,
                   rollspeed=0.,pitchspeed=0.,yawspeed=0.),1.01)
    e.consume(dict(mavpackettype='LOCAL_POSITION_NED',time_boot_ms=1000,x=0.,y=0.,z=-1.,vx=0.,vy=0.,vz=0.),1.01)
    e.lock_clock()
    assert e.observation(1.02).estimate.source=='PX4_EKF2_TELEMETRY'


def test_teacher_contact_feature_is_independent_of_airborne_initialization_clock(monkeypatch):
    module='rm27.experiments.aggressive_contact.flight_control.px4_sitl'
    monkeypatch.setattr(module+'.project',lambda *args:RateThrust(np.zeros(3),1.8))
    features=[]
    for offset in (0.,16.):
        s=SimpleNamespace(timestamp=offset+2.,position=np.zeros(3),velocity=np.zeros(3),
                          rotation=np.eye(3),omega_body=np.zeros(3))
        actor=SimpleNamespace(target=SimpleNamespace(position=np.zeros(3)),start_time=offset,
            p={'duration_s':6.},deployment={},inertia=np.eye(3),mixer=np.eye(4),
            cfg={'controller':{'attitude_damping':1.,'attitude_frequency_rad_s':1.}},
            command=lambda *args:(np.ones(4),None))
        pilot=TeacherPolicy.__new__(TeacherPolicy)
        pilot.actor=actor;pilot.mode=2;pilot.previous_action=np.r_[np.zeros(3),1.8];pilot.records=[]
        pilot.causal=SimpleNamespace(contact_time=offset+1.8,
            update=lambda *args:ActuatorEstimate(np.ones(4),1.))
        pilot.infer(SimpleNamespace(estimate=s),offset+2.)
        features.append(pilot.records[-1][2])
    np.testing.assert_allclose(features[0],features[1],atol=1e-12)
    assert features[1][-2]==pytest.approx(.2)


def test_research_matrix_does_not_promote_execution_integrity_or_abort_to_task_success():
    spec={'autopilot':'px4','acceptance':'task_success','cases':[{'name':'nominal'},{'name':'fault','expectation':'safe_abort'}]}
    rows=[{'name':'nominal','result':{'passed':False,'completed':True,
            'contact_evaluation':{'failure_modes':['EDGE_CONTACT']}}},
          {'name':'fault','result':{'passed':False,'safe_abort':True,
            'contact_evaluation':{'failure_modes':['TARGET_MISS']}}}]
    result=summarize(rows,spec)
    assert result['execution_passed'] and not result['passed']
    assert result['task_success_count']==0 and result['safe_abort_count']==1
    rows[0]['result']['passed']=True
    result=summarize(rows,spec)
    assert result['passed'] and result['task_success_count']==1 and result['total_attempts']==2
