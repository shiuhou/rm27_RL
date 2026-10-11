from rm27.autopilot.px4.return_termination import landing_admission,handle_camera_failure


def test_cached_tag_landing_requires_fresh_valid_navigation_and_bounded_aligned_tag():
    config=dict(assigned_tag_id=21,maximum_cached_tag_age_s=20)
    tag=dict(valid=True,assigned_tag_id=21,source_time_s=1.,position_relative_slot_nwu_xy_m=[.001,0.])
    telemetry=dict(local_time=16.,attitude_time=16.,estimator_time=16.,heartbeat_time=16.,
        estimator_flags=8,local_position=[0,0,-.6],local_velocity=[0,0,0],
        quaternion=[1,0,0,0],armed=True,mode='OFFBOARD')
    assert landing_admission(telemetry,tag,16.1,config)
    for update in ({'estimator_flags':0},{'heartbeat_time':15.},{'local_time':17.},
        {'local_velocity':[.1,0,0]},{'local_position':[0,0,-2.]},{'quaternion':[2,0,0,0]}):
        assert not landing_admission(dict(telemetry,**update),tag,16.1,config)
    assert not landing_admission(telemetry,dict(tag,assigned_tag_id=22),16.1,config)
    assert not landing_admission(telemetry,dict(tag,source_time_s=-10.),16.1,config)
    near=dict(tag,position_relative_slot_nwu_xy_m=[.006,0.])
    assert not landing_admission(telemetry,near,16.1,config)  # Frozen V1 admission.
    revised=dict(config,maximum_cached_tag_xy_m=.007)
    assert landing_admission(telemetry,near,16.1,revised)
    assert not landing_admission(telemetry,dict(tag,position_relative_slot_nwu_xy_m=[.008,0.]),16.1,revised)


def test_camera_failure_continues_only_admitted_land_and_cannot_be_task_success(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import json
    import rm27.autopilot.px4.return_termination as termination
    settings=dict(assigned_tag_id=21,maximum_cached_tag_age_s=20,maximum_cached_tag_xy_m=.007,
        gate_version='TEST')
    tag=dict(valid=True,assigned_tag_id=21,source_time_s=1.,position_relative_slot_nwu_xy_m=[.001,0.])
    tel=dict(local_time=16.,attitude_time=16.,estimator_time=16.,heartbeat_time=16.,
        estimator_flags=8,local_position=[0,0,-.6],local_velocity=[0,0,0],
        quaternion=[1,0,0,0],armed=True,mode='OFFBOARD')
    runtime=SimpleNamespace(output=tmp_path,telemetry=tel,plant=SimpleNamespace(data=SimpleNamespace(time=16.1)))
    calls=[]
    def land(*args):
        calls.append(args);return dict(passed=True,task_success=False,checks={'motors_stopped':True})
    monkeypatch.setattr(termination,'continue_landing',land)
    result=handle_camera_failure(runtime,'AprilTag reacquisition timed out',tag,settings)
    assert result['passed'] and not result['task_success'] and len(calls)==1
    assert json.loads((tmp_path/'termination_summary.json').read_text())==result
    result=handle_camera_failure(runtime,'Heartbeat stale',tag,settings)
    assert not result['admitted'] and not result['passed'] and len(calls)==1
    runtime.telemetry=dict(tel,estimator_flags=0)
    result=handle_camera_failure(runtime,'AprilTag reacquisition timed out',tag,settings)
    assert not result['admitted'] and not result['passed'] and len(calls)==1
