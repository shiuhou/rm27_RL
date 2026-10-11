"""Image pose and identity failures, without physics pose as detector input."""
import cv2
import numpy as np
import pytest

from rm27.perception.sitl.dock_tags import DockTagDetector
from rm27.perception.sitl.mujoco_camera import CameraFrame


def frame(ids=(21,),timestamp=100):
    image=np.ones((480,640),np.uint8)*255
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for i,tag in enumerate(ids):
        marker=cv2.aruco.generateImageMarker(dictionary,tag,100)
        x=270 if len(ids)==1 else 130+i*220
        image[190:290,x:x+100]=marker
    return CameraFrame(cv2.cvtColor(image,cv2.COLOR_GRAY2RGB),timestamp,640,480)


def test_pixel_tag_id_and_metric_depth_are_observed():
    d=DockTagDetector({21:.045});o=d.detect(frame(),now_us=100)
    assert len(o)==1 and o[0].safe_for_control and o[0].tag_id==21
    assert o[0].translation_camera_m[2]==pytest.approx(.045*415.6921938/99,rel=.02)
    assert np.linalg.norm(o[0].translation_camera_m[:2])<.001
    assert not d.detect(frame((22,)),now_us=100)


def test_duplicate_stale_and_future_tag_images_cannot_guide():
    d=DockTagDetector({21:.045})
    assert all(not o.safe_for_control for o in d.detect(frame((21,21)),now_us=100))
    assert not d.detect(frame(),now_us=300_101)[0].safe_for_control
    with pytest.raises(ValueError,match='future'):d.detect(frame(),now_us=99)
