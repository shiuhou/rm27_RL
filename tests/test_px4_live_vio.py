"""Live transport boundaries and simulation-only development admission."""
from dataclasses import replace
import struct

import numpy as np
import pytest

from rm27.perception.sitl.transport import PacketDecoder, encode_packet, MAX_PACKET_BYTES
from rm27.autopilot.px4.odometry import OdometryAdapter, Registration
from test_px4_odometry_clocks import estimate
from rm27.autopilot.px4.live_flight import VioRuntime
from rm27.perception.sitl.live import LiveVio
from rm27.perception.sitl.mujoco_camera import CameraFrame
from rm27.perception.sitl.image_target_detector import ImageTargetDetector


def test_live_runtime_rejects_implicit_diagnostic_profile():
    with pytest.raises(ValueError,match='explicit development profile'):
        VioRuntime(None,None,profile='diagnostic')


def test_fragmented_sensor_and_native_packets_preserve_source_clocks():
    rows=[dict(type='imu',source_time_ns=123,accel=[0,0,9.81],gyro=[0,0,0]),
          dict(type='native_state',row=dict(state_time_s=.123,publish_monotonic_ns=9000000))]
    wire=b''.join(encode_packet(row) for row in rows);decoder=PacketDecoder();received=[]
    for i in range(0,len(wire),7):received.extend(decoder.feed(wire[i:i+7]))
    assert received==rows and not decoder.buffer
    with pytest.raises(ValueError,match='length'):PacketDecoder().feed(struct.pack('!I',MAX_PACKET_BYTES+1))
    with pytest.raises(ValueError):encode_packet(dict(accel=float('nan')))


def test_unqualified_development_cannot_admit_hardware_or_hide_qualification():
    with pytest.raises(ValueError,match='restricted'):
        OdometryAdapter(Registration('vio','imu','real_sensor',np.eye(3),(0,0,0),np.eye(3)),allow_unqualified_simulation=True)
    e=estimate();e=replace(e,source_time=replace(e.source_time,clock_domain='mujoco_simulation'),
                          quality=dict(e.quality,flight_qualified=False))
    r=Registration('vio','imu','mujoco_simulation',np.eye(3),(0,0,0),np.eye(3),'native_monotonic')
    with pytest.raises(ValueError,match='qualification'):
        OdometryAdapter(r).packet(e,1_020_000_000,publication_now_ns=9_020_000_000)
    packet=OdometryAdapter(r,allow_unqualified_simulation=True).packet(e,1_020_000_000,publication_now_ns=9_020_000_000)
    assert packet.time_usec==1_000_000 and e.quality['flight_qualified'] is False


def test_live_camera_preserves_delayed_source_in_native_wire_and_saved_pixels(tmp_path):
    from collections import Counter
    from io import StringIO
    import json
    from types import SimpleNamespace

    wire=[]
    live=LiveVio.__new__(LiveVio)
    live.connection=SimpleNamespace(sendall=wire.append)
    live.detector=ImageTargetDetector();live.targets=StringIO();live.inputs=StringIO()
    live.counts=Counter();live.output=tmp_path;live.last_camera_source_us=-1
    (tmp_path/'images').mkdir()
    pixels=np.zeros((120,160,3),np.uint8)
    pixels[35:64,96:125]=(20,100,240);pixels[15:25,105:115]=(20,230,40)
    frame=CameraFrame(pixels,1_000_000,160,120)
    live.publish_camera(frame,1_300_000)
    assert live.last_target.valid and not live.last_target.safe_for_control
    assert live.last_target.measurement_age_us==300_000
    packet=PacketDecoder().feed(wire[0])[0]
    assert packet['source_time_ns']==1_000_000_000
    saved=json.loads(live.inputs.getvalue())
    assert saved['source_time_ns']==1_000_000_000
    assert saved['received_time_ns']==1_300_000_000
    assert (tmp_path/'images'/saved['filename']).exists()
    live.publish_camera(frame,1_400_000)
    live.publish_camera(replace(frame,timestamp_us=900_000),1_400_000)
    assert len(wire)==1 and live.counts['camera_non_increasing']==2
    live.publish_camera(replace(frame,timestamp_us=1_500_000),1_400_000)
    assert len(wire)==1 and live.counts['camera_future']==1 and live.last_frame is None
