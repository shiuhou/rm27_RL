"""Live transport boundaries and simulation-only development admission."""
from dataclasses import replace
import struct

import numpy as np
import pytest

from rm27.perception.sitl.transport import PacketDecoder, encode_packet, MAX_PACKET_BYTES
from rm27.autopilot.px4.odometry import OdometryAdapter, Registration
from test_px4_odometry_clocks import estimate
from rm27.autopilot.px4.live_flight import VioRuntime


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
