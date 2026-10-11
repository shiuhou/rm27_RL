from types import SimpleNamespace
import pytest

import numpy as np

from rm27.experiments.image_conditions import ConditionCamera,SensorCondition
from rm27.autopilot.px4.runtime import mavlink
from rm27.perception.sitl.mujoco_camera import CameraFrame
from rm27.perception.sitl.image_target_detector import ImageTargetDetector
from rm27.experiments.sensor_clock import InputClock


def test_delayed_camera_delivers_prior_pixels_with_their_acquisition_time():
    pixels=np.zeros((48,64,3),np.uint8)
    base=SimpleNamespace(capture=lambda data,t:CameraFrame(pixels.copy(),t,64,48))
    runtime=SimpleNamespace(return_phase='ROUTE')
    camera=ConditionCamera(base,runtime,{'primary_delay_frames':3},11,primary=True)
    assert camera.capture(SimpleNamespace(time=1.),1_000_000) is None
    assert camera.capture(SimpleNamespace(time=1.05),1_050_000) is None
    assert camera.capture(SimpleNamespace(time=1.10),1_100_000) is None
    delayed=camera.capture(SimpleNamespace(time=1.15),1_150_000)
    assert delayed.timestamp_us==1_000_000
    observation=ImageTargetDetector().detect(delayed,now_us=1_150_000)
    assert observation.measurement_age_us==150_000
    assert camera.records[-1]['source_time_us']==1_000_000


def test_actual_frame_loss_cannot_be_replaced_by_a_restamped_cached_image():
    base=SimpleNamespace(capture=lambda data,t:CameraFrame(np.zeros((48,64,3),np.uint8),t,64,48))
    runtime=SimpleNamespace(return_phase='ROUTE')
    settings=dict(primary_dropout={'phase':'ROUTE','delay_s':.05,'duration_s':.1})
    camera=ConditionCamera(base,runtime,settings,11,primary=True)
    assert camera.capture(SimpleNamespace(time=1.),1_000_000).timestamp_us==1_000_000
    assert camera.capture(SimpleNamespace(time=1.06),1_060_000) is None
    assert camera.capture(SimpleNamespace(time=1.11),1_110_000) is None
    fresh=camera.capture(SimpleNamespace(time=1.16),1_160_000)
    assert fresh.timestamp_us==1_160_000
    assert camera.records[-2]['source_time_us'] is None


def test_native_delivery_order_preserves_monotonic_acquisition_for_each_sensor():
    clock=InputClock()
    clock.observe('imu',1_300_000_000)
    clock.observe('camera',1_000_000_000)  #100ms or longer delay is valid.
    clock.observe('imu',1_304_000_000)
    clock.observe('camera',1_050_000_000)
    for kind,time in [('camera',1_050_000_000),('imu',1_303_000_000),('camera',-1),('camera',1.2)]:
        with pytest.raises(ValueError):clock.observe(kind,time)


def test_sensor_fault_changes_the_actual_packet_only_in_the_declared_window():
    runtime=SimpleNamespace(return_phase='HOVER',plant=SimpleNamespace(data=SimpleNamespace(time=1.)))
    c=SensorCondition(runtime,dict(kind='range_jump',phase='HOVER',delay_s=.5,duration_s=.5,offset_m=.2))
    message=mavlink.MAVLink_distance_sensor_message(1000,0,400,60,0,0,25,1)
    assert c.transform(message).current_distance==60
    runtime.plant.data.time=1.6
    assert c.transform(message).current_distance==80 and len(c.records)==1
    runtime.plant.data.time=2.1
    assert c.transform(message).current_distance==80 and len(c.records)==1
