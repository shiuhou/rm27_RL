import numpy as np
import pytest

from rm27.perception.sitl.mujoco_camera import CameraFrame
from rm27.perception.sitl.image_target_detector import ImageTargetDetector
from tools.px4.approach_target_fault import alter_target


@pytest.mark.parametrize('kind',['occlusion','wrong_colour'])
def test_target_pixel_fault_retains_background_clock_and_rejects_target(kind):
    pixels=np.full((160,240,3),40,dtype=np.uint8)
    pixels[65:80,110:130]=[10,230,10]
    pixels[90:110,105:135]=[20,40,240]
    frame=CameraFrame(pixels,151000,240,160)
    detector=ImageTargetDetector()
    assert detector.detect(frame,now_us=151000).safe_for_control
    changed,count=alter_target(frame,60.,kind)
    assert count==600 and changed.timestamp_us==frame.timestamp_us
    assert np.array_equal(changed.image_rgb[:90],pixels[:90])
    assert np.array_equal(frame.image_rgb,pixels)
    assert not detector.detect(changed,now_us=151000).safe_for_control
