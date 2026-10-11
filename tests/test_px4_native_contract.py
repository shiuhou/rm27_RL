import numpy as np
import pytest

from rm27.perception.sitl.native_contract import NativeContract
from rm27.perception.vision.localization.schema import TrackingState


def row(**changes):
    r=dict(sequence=0,epoch=0,state_time_s=1.,camera_time_s=1.,initialized=True,
        publish_monotonic_ns=9_010_000_000,camera_processing_started_monotonic_ns=9_000_000_000,
        camera_processing_completed_monotonic_ns=9_009_000_000,msckf_update_features=10,slam_features=0,
        covariance_p_q_v=(np.eye(9)*.001).ravel().tolist(),position=[1.,2.,3.],quaternion_xyzw=[0.,0.,0.,1.],velocity_world=[.1,0.,0.])
    r.update(changes);return r


def test_native_evidence_is_complete_but_not_automatically_qualified():
    e=NativeContract().estimate(row())
    assert e.valid and e.initialized and e.tracking_state==TrackingState.TRACKING
    assert not e.quality['flight_qualified']
    assert e.source_time.clock_domain!=e.publish_time.clock_domain
    assert e.quality['processing_duration_s']==pytest.approx(.009)
    assert e.orientation_wxyz==(1.,0.,0.,0.)


def test_native_tracking_loss_and_initialization_are_observed():
    assert NativeContract().estimate(row(msckf_update_features=0)).tracking_state==TrackingState.LOST
    assert NativeContract().estimate(row(initialized=False)).tracking_state==TrackingState.INITIALIZING
    assert not NativeContract().estimate(row(camera_time_s=.8)).valid


def test_native_covariance_order_and_publication_fail_closed():
    a=NativeContract();a.estimate(row())
    with pytest.raises(ValueError,match='publication order'):a.estimate(row())
    with pytest.raises(ValueError,match='state time'):a.estimate(row(sequence=1))
    with pytest.raises(ValueError,match='covariance'):NativeContract().estimate(row(covariance_p_q_v=(-np.eye(9)).ravel().tolist()))
    with pytest.raises(ValueError,match='publication times'):NativeContract().estimate(row(publish_monotonic_ns=1))


def test_jpl_local_orientation_covariance_rotates_into_parent_basis():
    # 90 degree body yaw maps body x uncertainty into world y. Position and
    # position/orientation cross covariance must retain the same mapping.
    from scipy.spatial.transform import Rotation
    c=np.diag([1.,2.,3.,4.,5.,6.,7.,8.,9.]);c[0,3]=c[3,0]=.2
    q=Rotation.from_euler('z',90,degrees=True).as_quat()
    e=NativeContract().estimate(row(quaternion_xyzw=q.tolist(),covariance_p_q_v=c.ravel().tolist()))
    pose=np.asarray(e.quality['pose_covariance'])
    assert np.diag(pose)==pytest.approx([1.,2.,3.,5.,4.,6.])
    assert pose[0,4]==pytest.approx(.2)
    assert abs(pose[0,3])<1e-12
