import numpy as np
from scipy.spatial.transform import Rotation

from rm27.experiments.image_contact_development import GuardTipRoute


def test_contact_commitment_aligns_configured_guard_point_using_telemetry_attitude():
    audit=dict(passed=True,surface_route_clear=True,waypoints_world_m=[[0,0,2],[0,0,1.6]])
    mission=dict(surveyed_armor_center_m=[0,0,1],surveyed_armor_normal=[0,0,1],physics_deadline_s=400,
        maximum_flight_speed_m_s=2,maximum_tilt_deg=45,horizontal_command_limit_m_s=.4,vertical_command_limit_m_s=.35,
        contact_point_body_flu_m=[-.06,.06,-.018],tip_staging_distance_m=.1)
    task=GuardTipRoute(audit,mission,dict(forward_body=[0,0,-1],up_body=[0,1,0],offset_body=[0,0,0]))
    task.state='VISUAL_APPROACH';task.pose_time=1.;task.last_time=1.
    q=Rotation.from_euler('z',-45,degrees=True).as_quat()[[3,0,1,2]]
    task.navigation=dict(estimator_time=1.,estimator_flags=8,quaternion=q,rates=[0,0,0])
    offset=Rotation.from_euler('z',45,degrees=True).apply(task.contact_point)
    position=task.armor_center-offset+[0,0,.1];task.reference=position.copy()
    goal=task.target(1.01,position,0,velocity=np.zeros(3))
    assert task.state=='COMMIT'
    np.testing.assert_allclose(task.contact_bias,-offset)
    np.testing.assert_allclose(goal+offset,task.armor_center+[0,0,.1])
