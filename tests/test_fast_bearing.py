from dataclasses import asdict

import numpy as np

from rm27.experiments.fast_bearing import detect
from rm27.perception.sitl.image_target_detector import ImageTargetDetector,DetectorConfig
from rm27.perception.sitl.mujoco_camera import CameraFrame


def test_roi_bearing_preserves_original_outputs_across_noise_ambiguity_and_staleness():
    rng=np.random.default_rng(5937)
    for index in range(30):
        pixels=rng.integers(0,256,(120,160,3),np.uint8)
        if index%3:
            pixels[65:90,30:60]=(20,100,240);pixels[45:52,40:50]=(20,230,40)
        if index%5==0:
            pixels[65:90,110:140]=(20,100,240);pixels[45:52,120:130]=(20,230,40)
        frame=CameraFrame(pixels,1_000_000,160,120)
        detector=ImageTargetDetector(DetectorConfig(require_marker=index%4!=0))
        for now in [1_000_000,1_350_000]:
            assert asdict(detector.detect(frame,now_us=now))==asdict(detect(detector,frame,now_us=now))
