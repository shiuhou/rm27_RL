"""Finite logging session exercised over a fake wire, never a serial port."""
import importlib.util
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import common as mav

from rm27.telemetry.logging_session import LoggingSession

FIXTURES=Path(__file__).resolve().parents[1]/'.maixpy/imu-design-20261010/single-uart/reassembly/test_ulog_mavlink.py'
spec=importlib.util.spec_from_file_location('logging_wire_fixtures',FIXTURES)
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)


class Wire:
    def __init__(self):self.bytes=bytearray()
    def write(self,data):self.bytes.extend(data)


def ack(command,result=mav.MAV_RESULT_ACCEPTED):
    return mav.MAVLink_command_ack_message(command,result).pack(mav.MAVLink(None,srcSystem=1,srcComponent=1))


def messages(wire):
    return mav.MAVLink(None).parse_buffer(bytes(wire.bytes))


def streaming():
    wire=Wire();session=LoggingSession(wire,timeout_s=.1)
    session.start(0);session.receive(ack(mav.MAV_CMD_LOGGING_START),1)
    return session,wire


def test_logging_start_ack_retries_stop_drain_and_ended_session():
    session,wire=streaming();raw=fixtures.fixture();parts=fixtures.packets(raw,100)
    session.receive(parts[0],2);session.receive(parts[0],3)
    for part in parts[1:]:session.receive(part,4)
    assert session.reassembler.retransmissions==1
    assert [m.sequence for m in messages(wire) if m.get_type()=='LOGGING_ACK']==[0,0]
    with pytest.raises(ValueError,match='drain'):session.finish()
    session.stop(5);session.receive(ack(mav.MAV_CMD_LOGGING_STOP),6)
    assert session.state=='DRAINING' and session.poll(7) is None
    assert session.poll(100_000_006)==raw and session.state=='COMPLETE'
    assert session.summary()['upstream_source_complete']=='UNKNOWN'
    assert session.summary()['openvins_admissible'] is False
    with pytest.raises(ValueError,match='inactive'):session.receive(parts[0],100_000_007)


def test_logging_missing_packet_and_crc_failure_are_terminal():
    session,_=streaming();parts=fixtures.packets(fixtures.fixture(),100)
    session.receive(parts[0],2)
    with pytest.raises(ValueError,match='sequence'):session.receive(parts[2],3)
    assert session.state=='FAILED'
    session,_=streaming();bad=bytearray(parts[0]);bad[-1]^=1
    with pytest.raises(ValueError,match='CRC'):session.receive(bytes(bad),2)
    assert session.state=='FAILED'


@pytest.mark.parametrize('phase',['start','stream','stop'])
def test_logging_timeouts_never_return_partial_logs(phase):
    if phase=='start':session=LoggingSession(Wire(),timeout_s=.1);session.start(0)
    else:
        session,_=streaming()
        if phase=='stop':session.stop(2)
    with pytest.raises(ValueError,match='timeout'):session.poll(200_000_000)
    assert session.state=='FAILED'


def test_logging_rejected_command_and_reversed_receipt():
    session=LoggingSession(Wire());session.start(0)
    with pytest.raises(ValueError,match='rejected'):
        session.receive(ack(mav.MAV_CMD_LOGGING_START,mav.MAV_RESULT_DENIED),1)
    session,_=streaming()
    with pytest.raises(ValueError,match='clock reversed'):session.receive(b'',0)
