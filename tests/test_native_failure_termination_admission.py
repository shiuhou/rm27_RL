from tools.px4.native_failure_termination import admission


def test_native_failure_requires_initial_footprint_and_fresh_valid_navigation():
    telemetry = dict(mode='OFFBOARD',armed=True,estimator_flags=10,
        local_time=10,attitude_time=10,estimator_time=10,heartbeat_time=10,distance_time=10,distance_m=.25,
        raw_local_position_ned=[0,0,-.25],local_velocity=[0,0,-1.],quaternion=[1,0,0,0])
    assert admission(telemetry,'TAKEOFF',10.1)
    assert not admission(telemetry,'ROUTE',10.1)
    assert not admission(telemetry,'TAKEOFF',10.31)
    assert not admission(dict(telemetry,estimator_flags=0),'TAKEOFF',10.1)
    assert not admission(dict(telemetry,raw_local_position_ned=[.03,0,-.25]),'TAKEOFF',10.1)
    assert not admission(dict(telemetry,local_velocity=[0,0,-2.]),'TAKEOFF',10.1)
