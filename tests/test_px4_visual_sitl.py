"""PX4 visual SITL camera and image-only target contract tests."""

import math

import cv2
import mujoco
import numpy as np
import pytest

from rm27.perception.sitl.image_target_detector import DetectorConfig, ImageTargetDetector
from rm27.perception.sitl.mujoco_camera import CameraFrame, MujocoCamera


def test_image_detector_uses_rgb_pixels_and_v2_angles():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    # A target to the right and above the optical center with a separate lamp.
    cv2.rectangle(image, (96, 35), (124, 63), (20, 100, 240), -1)
    cv2.circle(image, (110, 20), 5, (20, 230, 40), -1)
    frame = CameraFrame(image, 1_000_000, 160, 120)
    result = ImageTargetDetector(DetectorConfig(minimum_area_px=4)).detect(frame, now_us=1_020_000)
    assert result.valid and result.safe_for_control
    assert result.measurement_age_us == 20_000
    assert result.yaw_rad > 0.0 and result.pitch_rad > 0.0
    assert result.confidence > 0.15


def test_image_detector_marks_missing_and_stale_frames_unsafe():
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    detector = ImageTargetDetector()
    missing = detector.detect(CameraFrame(image, 10, 64, 48), now_us=10)
    assert not missing.valid and not missing.safe_for_control
    image[20:28, 30:38] = (20, 100, 240)
    image[10:14, 32:36] = (20, 230, 40)
    stale = detector.detect(CameraFrame(image, 10, 64, 48), now_us=300_011)
    assert stale.valid and not stale.safe_for_control
    assert stale.measurement_age_us == 300_001


@pytest.mark.skipif(not __import__("os").environ.get("MUJOCO_GL"), reason="renderer needs an explicit headless GL backend")
def test_mujoco_camera_renders_following_body():
    model = mujoco.MjModel.from_xml_string(
        """<mujoco><option timestep='.002'/><worldbody>
        <light pos='0 0 3' dir='0 0 -1'/>
        <body name='drone_1' pos='0 0 1'><freejoint/>
          <geom type='sphere' size='.05' rgba='0.2 0.2 0.2 1' mass='.15'/>
        </body></worldbody></mujoco>"""
    )
    data = mujoco.MjData(model)
    data.qpos[:3] = [1.0, -2.0, 1.5]
    data.qpos[3:7] = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    with MujocoCamera(model, width=96, height=64, body_name="drone_1") as camera:
        frame = camera.capture(data, 123_456)
        np.testing.assert_allclose(camera.renderer.scene.camera[0].pos, [1.0, -2.0, 1.52], atol=1e-6)
    assert frame.image_rgb.shape == (64, 96, 3)
    assert frame.timestamp_us == 123_456
    assert not hasattr(frame, 'camera_position_world')


def test_unassociated_blue_lines_and_ambiguous_targets_are_unsafe():
    detector=ImageTargetDetector()
    image=np.zeros((120,160,3),np.uint8)
    image[65:68,:]=(0,0,255)
    image[30:40,75:85]=(0,255,0)
    assert not detector.detect(CameraFrame(image,100,160,120),now_us=100).safe_for_control
    for x in [30,110]:
        image[65:80,x:x+20]=(20,100,240)
        image[45:52,x+7:x+14]=(20,230,40)
    assert not detector.detect(CameraFrame(image,100,160,120),now_us=100).safe_for_control


def test_camera_timestamp_cannot_launder_a_future_image():
    image=np.zeros((48,64,3),np.uint8)
    with pytest.raises(ValueError,match='future'):
        ImageTargetDetector().detect(CameraFrame(image,100,64,48),now_us=90)
    with pytest.raises(ValueError,match='integer'):
        CameraFrame(image,10.5,64,48)


def test_simulated_optical_extrinsics_and_first_pose_registration():
    from rm27.perception.sitl.dataset import camera_to_imu,intrinsics
    from rm27.perception.sitl.evaluate_vio import align_first_pose
    c=dict(forward_body=[1,0,0],up_body=[0,0,1],offset_body=[.01,0,.02])
    t=camera_to_imu(c)
    np.testing.assert_allclose(t[:3,0],[0,-1,0])
    np.testing.assert_allclose(t[:3,1],[0,0,-1])
    np.testing.assert_allclose(t[:3,2],[1,0,0])
    assert np.linalg.det(t[:3,:3])==pytest.approx(1.)
    assert intrinsics(640,480,60)[0]==pytest.approx(415.6921938)
    rotation,translation=align_first_pose(np.array([1,2,3]),np.eye(3),np.array([4,5,6]),np.eye(3))
    np.testing.assert_allclose(rotation@np.array([1,2,3])+translation,[4,5,6])


@pytest.mark.skipif(not __import__('os').environ.get('MUJOCO_GL'),reason='explicit GL backend required')
def test_rendered_projection_matches_the_exported_intrinsics():
    from rm27.perception.sitl.dataset import intrinsics
    xml='''<mujoco><visual><headlight ambient="1 1 1" diffuse="0 0 0"/></visual>
    <worldbody><body name="drone_1"><geom type="sphere" size=".001" mass=".15"/></body>
    <geom pos="1 -.2 .1" type="sphere" size=".01" rgba="0 1 0 1"/></worldbody></mujoco>'''
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
    with MujocoCamera(model,width=640,height=480,offset_body=(0,0,0)) as camera:
        frame=camera.capture(data,0)
    pixels=np.argwhere((frame.image_rgb[:,:,1]>100)&(frame.image_rgb[:,:,0]<20))
    v,u=pixels.mean(axis=0)
    fx,fy,cx,cy=intrinsics(640,480,60)
    assert u==pytest.approx(cx+.2*fx,abs=1.)
    assert v==pytest.approx(cy-.1*fy,abs=1.)


@pytest.mark.skipif(not __import__('os').environ.get('MUJOCO_GL'),reason='explicit GL backend required')
def test_named_down_camera_sees_close_pad_in_full_field_without_changing_projection():
    from rm27.perception.sitl.dataset import intrinsics
    xml='''<mujoco><statistic extent="20"/>
    <visual><headlight ambient="1 1 1" diffuse="0 0 0"/></visual>
    <worldbody><body name="drone_1" pos="0 0 .12">
      <geom type="sphere" size=".001" mass=".15"/>
      <camera name="down" fovy="70"/>
    </body><geom pos=".01 -.015 0" type="box" size=".004 .004 .0005" rgba="0 1 0 1"/>
    </worldbody></mujoco>'''
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
    assert model.vis.map.znear*model.stat.extent > .12
    with MujocoCamera(model,width=640,height=480,fovy_deg=70,camera_name='down') as camera:
        frame=camera.capture(data,0)
        assert camera.renderer.scene.camera[0].frustum_near <= .0021
    colors=frame.image_rgb.astype(float)
    pixels=np.argwhere((colors[:,:,1]-colors[:,:,0]>80)&(colors[:,:,1]-colors[:,:,2]>80))
    assert len(pixels)>50
    v,u=pixels.mean(axis=0);fx,fy,cx,cy=intrinsics(640,480,70)
    assert u==pytest.approx(cx+.01*fx/.1195,abs=1.)
    assert v==pytest.approx(cy+.015*fy/.1195,abs=1.)
