import numpy as np
import pytest

from rm27.perception.sitl.dock_tags import TagObservation
from rm27.perception.sitl.return_camera import slot_relative_position
from rm27.autopilot.px4.visual_return import VisualReturnGuard


def observation(*, valid=True):
    return TagObservation(21, 100, (), (-.01, .02, .5), tuple(map(tuple, np.eye(3))),
                          0., valid, 'valid' if valid else 'stale')


def test_return_position_uses_camera_mount_and_px4_attitude():
    # Level NED/FRD attitude; tag 10 mm left, 20 mm down in the image.
    np.testing.assert_allclose(slot_relative_position(observation(), [1, 0, 0, 0],
                              [0, 0, -.025]), [.01, .02], atol=1e-9)
    # PX4 90-degree NED yaw rotates the FLU displacement into NWU.
    yaw_ned_90=[np.sqrt(.5), 0, 0, np.sqrt(.5)]
    np.testing.assert_allclose(slot_relative_position(observation(), yaw_ned_90,
                              [0, 0, -.025]), [.02, -.01], atol=1e-9)


def test_return_position_rejects_unsafe_detection_and_attitude():
    with pytest.raises(ValueError, match='unsafe'):
        slot_relative_position(observation(valid=False), [1, 0, 0, 0], [0, 0, -.025])
    with pytest.raises(ValueError, match='attitude'):
        slot_relative_position(observation(), [0, 0, 0, 0], [0, 0, -.025])


def test_return_guard_rejects_other_slots_stale_data_and_cached_freshness_extension():
    guard=VisualReturnGuard(21)
    row=dict(valid=True,assigned_tag_id=21,source_time_s=1.,position_relative_slot_nwu_xy_m=[.001,.002])
    assert guard.observe(row,1.01,[.011,.002],telemetry_age_s=.01)
    np.testing.assert_allclose(guard.correction,[.01,0.])
    assert guard.aligned(1.1)
    assert not guard.observe(row,1.15,[.011,.002],telemetry_age_s=.01)
    assert not guard.fresh(1.21)
    assert not guard.observe(dict(row,source_time_s=1.3,assigned_tag_id=22),1.3,[0,0],telemetry_age_s=0)
    assert not guard.observe(dict(row,source_time_s=1.3),1.3,[0,0],telemetry_age_s=.21)
    assert not guard.observe(dict(row,source_time_s=2.),1.3,[0,0],telemetry_age_s=0)
    assert not guard.observe(dict(row,source_time_s=1.3),1.3,[float('nan'),0],telemetry_age_s=0)
    assert not guard.observe(dict(row,source_time_s=1.3),1.3,[.5,0],telemetry_age_s=0)
    assert guard.source_time==1.
