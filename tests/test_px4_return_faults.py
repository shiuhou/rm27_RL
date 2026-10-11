import cv2
import numpy as np

from rm27.autopilot.px4.return_faults import ImageFault, assess
from rm27.perception.sitl.dock_tags import DockTagDetector
from rm27.perception.sitl.mujoco_camera import CameraFrame


def frame(stamp=1_000_000):
    image=np.full((480,640,3),255,np.uint8)
    marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),21,160)
    image[160:320,240:400]=marker[:,:,None]
    return CameraFrame(image,stamp,640,480)


def fault(kind):
    return ImageFault(dict(kind=kind,phase='DESCEND',delay_s=.5,duration_s=1.,assigned_tag_id=21,replacement_tag_id=22))


def test_faults_operate_before_the_real_pixel_detector_and_preserve_source_age():
    detector=DockTagDetector({21:.08,22:.08})
    stale=fault('stale');initial=frame()
    stale.transform(initial,'DESCEND',1.)
    held=stale.transform(frame(1_600_000),'REACQUIRE',1.6)
    assert held.timestamp_us==1_000_000
    observations=detector.detect(held,now_us=1_600_000)
    assert observations and not observations[0].safe_for_control and observations[0].reason=='stale'
    wrong=fault('wrong_id');wrong.transform(initial,'DESCEND',1.)
    replaced=wrong.transform(frame(1_600_000),'REACQUIRE',1.6)
    assert [o.tag_id for o in detector.detect(replaced,now_us=1_600_000)]==[22]
    missing=fault('missing');missing.transform(initial,'DESCEND',1.)
    assert not detector.detect(missing.transform(frame(1_600_000),'REACQUIRE',1.6),now_us=1_600_000)


def test_fault_score_requires_new_identification_and_landing_after_the_fault():
    summary=dict(passed=True,events=[{'event':'REACQUIRE'}],landing_commitment={'timestamp_s':3.})
    injections=[dict(kind='missing',received_time_s=2.)]
    assert not assess(summary,[dict(valid=True,source_time_s=1.)],injections)['passed']
    assert assess(summary,[dict(valid=True,source_time_s=2.5)],injections)['passed']
    summary['landing_commitment']['timestamp_s']=1.5
    assert not assess(summary,[dict(valid=True,source_time_s=2.5)],injections)['passed']
