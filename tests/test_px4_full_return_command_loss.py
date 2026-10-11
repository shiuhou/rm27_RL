from tools.px4.full_return_command_loss import eligible
from rm27.autopilot.px4.runtime import mavlink


def observations(now):
    tag = dict(valid=True, assigned_tag_id=21, source_time_s=now,
               position_relative_slot_nwu_xy_m=[.001, 0.])
    telemetry = dict(local_time=now, attitude_time=now, estimator_time=now,
        heartbeat_time=now-.25, local_velocity=[0., 0., 0.], armed=True,
        mode='OFFBOARD', estimator_flags=mavlink.ESTIMATOR_POS_HORIZ_REL)
    return tag, telemetry


def test_heartbeat_reception_boundary_does_not_reset_stable_loss_admission():
    tag, telemetry = observations(10.)
    assert eligible(tag, telemetry, 'ALIGN', 10.)
    assert not eligible(tag, telemetry, 'ROUTE', 10.)
    telemetry['heartbeat_time'] = 8.
    assert not eligible(tag, telemetry, 'ALIGN', 10.)


def test_command_loss_requires_current_assigned_image_and_navigation():
    tag, telemetry = observations(10.)
    tag['assigned_tag_id'] = 22
    assert not eligible(tag, telemetry, 'ALIGN', 10.)
    tag['assigned_tag_id'] = 21
    tag['source_time_s'] = 9.7
    assert not eligible(tag, telemetry, 'ALIGN', 10.)
    tag['source_time_s'] = 10.
    telemetry['local_time'] = 9.7
    assert not eligible(tag, telemetry, 'ALIGN', 10.)
