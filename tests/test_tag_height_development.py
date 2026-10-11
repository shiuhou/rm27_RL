import numpy as np
import pytest
from types import SimpleNamespace

from rm27.experiments.tag_height_development import image_height,TagHeightRuntime


def test_return_height_comes_from_metric_tag_mount_and_attitude_with_explicit_plane_offset():
    row=dict(valid=True,assigned_tag_id=21,observations=[dict(tag_id=21,safe_for_control=True,
        translation_camera_m=[0.,0.,.0935])])
    assert image_height(row,[1,0,0,0],[0,0,-.025])==pytest.approx(.12)
    row['assigned_tag_id']=22
    with pytest.raises(ValueError,match='assigned'):image_height(row,[1,0,0,0],[0,0,-.025])


def test_return_registration_preserves_raw_height_and_does_not_accumulate_frame_shift():
    runtime=object.__new__(TagHeightRuntime)
    runtime.raw_local=np.array([1.,2.,-.20]);runtime.height_correction=.08
    runtime.height_source_time=1.;runtime.telemetry={}
    runtime._apply();runtime._apply()
    np.testing.assert_allclose(runtime.telemetry['local_position'],[1.,2.,-.12])
    np.testing.assert_allclose(runtime.telemetry['raw_local_position_ned'],[1.,2.,-.20])


def test_height_registration_rejects_old_images_wrong_slot_and_stale_navigation():
    runtime=object.__new__(TagHeightRuntime)
    runtime.raw_local=np.array([0.,0.,-.20]);runtime.height_correction=0.
    runtime.height_source_time=None;runtime.height_stream=None
    runtime.return_camera=SimpleNamespace(assigned_id=21,config={'position_body_flu_m':[0,0,-.025]})
    runtime.telemetry=dict(local_time=1.,attitude_time=1.,quaternion=[1,0,0,0],local_velocity=[0,0,.1])
    row=dict(valid=True,source_time_s=1.,assigned_tag_id=21,observations=[dict(tag_id=21,
        safe_for_control=True,translation_camera_m=[0,0,.0935])])
    runtime.observe_return_height(row,1.1)
    # Camera time is 1.0: using receive time 1.1 would cause a 1 cm bias.
    assert runtime.height_correction==pytest.approx(.08)
    for source,assigned,now,local_time,attitude_time in (
        (1.,21,1.1,1.,1.),(.99,21,1.1,1.,1.),
        (1.11,22,1.15,1.1,1.1),(1.4,21,1.15,1.1,1.1),
        (1.01,21,1.3,1.25,1.25),(1.28,21,1.3,1.,1.25),
        (1.28,21,1.3,1.25,1.)):
        changed=dict(row,source_time_s=source,assigned_tag_id=assigned)
        runtime.telemetry.update(local_time=local_time,attitude_time=attitude_time)
        runtime.observe_return_height(changed,now)
        assert runtime.height_source_time==1.
        assert runtime.height_correction==pytest.approx(.08)
