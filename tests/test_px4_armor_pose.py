from dataclasses import asdict
import json
import cv2
import numpy as np
import pytest

from rm27.perception.sitl.armor_pose import ArmorPoseDetector
from rm27.perception.sitl.mujoco_camera import CameraFrame


def sample(*, marker=True, duplicate=False):
    image=np.full((480,640,3),20,np.uint8)
    image[209:271,269:371]=[5,115,242]
    if marker:cv2.circle(image,(320,160),15,(5,255,8),-1)
    if duplicate:
        image[209:271,99:201]=[5,115,242];cv2.circle(image,(150,160),15,(5,255,8),-1)
    return CameraFrame(image,1_000_000,640,480)


def test_metric_image_pose_requires_marker_identity_scale_and_source_age():
    detector=ArmorPoseDetector()
    observation=detector.detect(sample(),now_us=1_010_000)
    assert observation.valid and observation.reprojection_rmse_px<.4  # Rasterized aspect ratio is quantized.
    json.dumps(asdict(observation),allow_nan=False)
    assert observation.translation_camera_m[2]==pytest.approx(.37,abs=.012)
    assert np.linalg.norm(observation.normal_camera)==pytest.approx(1.)
    assert not detector.detect(sample(marker=False),now_us=1_010_000).valid
    assert not detector.detect(sample(duplicate=True),now_us=1_010_000).valid
    assert not detector.detect(sample(),now_us=1_210_000).valid
    with pytest.raises(ValueError,match='metric'):
        ArmorPoseDetector(width_m=0)
