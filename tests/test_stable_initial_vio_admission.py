from dataclasses import replace
import numpy as np

from rm27.perception.vision.localization.schema import TimestampEvidence
from tools.px4.initial_vio_admission import StableInitialAdmission
from test_px4_odometry_clocks import estimate


def test_vertical_inconsistency_blocks_first_registration_and_requires_new_stability():
    guard=StableInitialAdmission()
    original=estimate()
    e=replace(original,velocity=(0,0,0),orientation_wxyz=(1,0,0,0),
              quality=dict(original.quality,pose_covariance=(np.eye(6)*.0025).tolist()))
    for i in range(5):
        source=1+i*.05
        sample=replace(e,source_time=TimestampEvidence(round(source*1e9),'ns','mujoco_simulation','NATIVE_IMU_STATE_TIME','VERIFIED'))
        history=[(source,[0,0,0],[1,0,0,0],[0,0,0])]
        accepted,_=guard(sample,history)
        assert accepted is (i==4)
    sample=replace(sample,velocity=(0,0,1))
    assert guard(sample,history)[0] is False
    next_sample=replace(e,source_time=TimestampEvidence(1_300_000_000,'ns','mujoco_simulation','NATIVE_IMU_STATE_TIME','VERIFIED'))
    assert guard(next_sample,[(1.3,[0,0,0],[1,0,0,0],[0,0,0])])[0] is False


def test_initial_stability_cannot_span_a_localization_epoch_reset():
    guard=StableInitialAdmission()
    original=estimate()
    e=replace(original,velocity=(0,0,0),orientation_wxyz=(1,0,0,0),
              quality=dict(original.quality,pose_covariance=(np.eye(6)*.0025).tolist()))
    results=[]
    for i in range(9):
        source=1+i*.05
        sample=replace(e,localization_epoch=0 if i<4 else 1,
            source_time=TimestampEvidence(round(source*1e9),'ns','mujoco_simulation','NATIVE_IMU_STATE_TIME','VERIFIED'))
        results.append(guard(sample,[(source,[0,0,0],[1,0,0,0],[0,0,0])])[0])
    assert results==[False]*8+[True]
