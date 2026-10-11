from types import SimpleNamespace

import numpy as np
import pytest

from rm27.autopilot.px4.image_route import ImageRoute, BASIS


def route():
    audit=dict(passed=True,surface_route_clear=True,waypoints_world_m=[[1,0,1.6],[0,0,1.6],[-1,0,1.]])
    mission=dict(surveyed_armor_center_m=[-1,0,.4],surveyed_armor_normal=[0,0,1],physics_deadline_s=300,
        maximum_flight_speed_m_s=2,maximum_tilt_deg=45,horizontal_command_limit_m_s=.4,
        vertical_command_limit_m_s=.35,impact_acceleration_threshold_m_s2=2.,impact_rate_threshold_rad_s=.5,
        armor_geom='armor',contact_commitment_lease_s=.65)
    camera=dict(forward_body=[0,0,-1],up_body=[0,1,0],offset_body=[0,0,0])
    return ImageRoute(audit,mission,camera)


def test_task_rejects_physics_contact_input_and_does_not_mark_return_as_target_success():
    task=route()
    with pytest.raises(ValueError,match='forbidden'):
        task.target(1,np.zeros(3),0,[{'geoms':['armor','body']}],np.zeros(3))
    assert not task.result()['passed']
    task.aborted=True;task.impact={'time_s':1.}
    assert not task.result()['passed']


def test_impact_requires_a_fresh_strong_sensor_event_and_preserves_world_force_direction():
    task=route();task.navigation=dict(quaternion=[1.,0,0,0],rates=[0.,0.,0.],attitude_time=1.03)
    # A level FRD specific force of -9.81 is free hover, never an impact.
    message=SimpleNamespace(time_usec=1_000_000,xacc=0.,yacc=0.,zacc=-9.81)
    task.observe_imu(message)
    assert not task._impact(1.01,np.zeros(3),np.zeros(3))
    message.time_usec=1_020_000;message.zacc=-10.5
    task.observe_imu(message)
    assert not task._impact(1.03,np.zeros(3),np.zeros(3))
    assert not task._impact(1.031,np.zeros(3),np.zeros(3))  # Reusing a sample is insufficient.
    message.time_usec=1_040_000;message.zacc=-13.;task.observe_imu(message)
    assert task._impact(1.05,np.zeros(3),np.zeros(3))
    assert task.impact['source'].startswith('fresh IMU impulse')


def test_same_sample_gyro_detects_impact_before_cached_attitude_rates_update():
    task=route()
    task.navigation=dict(quaternion=[1.,0,0,0],rates=[0.,0.,0.],attitude_time=1.)
    task.observe_imu(SimpleNamespace(time_usec=1_020_000,xacc=0.,yacc=0.,zacc=-9.81,
        xgyro=.9,ygyro=0.,zgyro=0.,fields_updated=63))
    assert task._impact(1.024,np.zeros(3),np.zeros(3))
    assert task.impact['imu_samples'][0][2]==pytest.approx(.9)


def test_unupdated_or_old_gyro_cannot_admit_an_impact():
    task=route()
    task.navigation=dict(quaternion=[1.,0,0,0],rates=[.9,0.,0.],attitude_time=.5)
    task.observe_imu(SimpleNamespace(time_usec=1_020_000,xacc=0.,yacc=0.,zacc=-9.81,
        xgyro=.9,ygyro=0.,zgyro=0.,fields_updated=7))
    assert not task._impact(1.024,np.zeros(3),np.zeros(3))
    task.observe_imu(SimpleNamespace(time_usec=1_040_000,xacc=0.,yacc=0.,zacc=-9.81,
        xgyro=.9,ygyro=0.,zgyro=0.,fields_updated=63))
    assert not task._impact(1.15,np.zeros(3),np.zeros(3))
