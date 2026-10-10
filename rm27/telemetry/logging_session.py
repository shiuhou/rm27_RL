"""Finite ULog/MAVLink session controller; transport must be explicitly injected.

This module opens no serial port. Source completeness is not established by
framing verification or a STOP command acknowledgement.
"""
import importlib.util
from pathlib import Path
from pymavlink.dialects.v20 import common as mavlink

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'.maixpy/imu-design-20261010/single-uart/reassembly/ulog_mavlink.py'
spec=importlib.util.spec_from_file_location('rm27_saved_ulog_reassembly',SOURCE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class LoggingSession:
    def __init__(self, transport, *, source=(1,1), target=(1,191), timeout_s=3., max_bytes=64*1024*1024):
        if timeout_s<=0:raise ValueError('timeout must be positive')
        self.transport,self.source,self.target=transport,source,target
        self.timeout_ns=round(timeout_s*1e9);self.state='IDLE';self.deadline=None;self.failure=None
        self.reassembler=module.Reassembler(source=source,target=target,max_bytes=max_bytes)
        self.decoder=module.decoder_module.Decoder()
        self.encoder=mavlink.MAVLink(transport,srcSystem=target[0],srcComponent=target[1])
        self.last_receipt=None
    def fail(self,reason):
        self.state='FAILED';self.failure=reason;raise ValueError(reason)
    def _command(self,command):
        self.encoder.command_long_send(*self.source,command,0,0,0,0,0,0,0,0)
    def start(self,now_ns):
        if self.state!='IDLE':raise ValueError('session already started')
        self.state='STARTING';self.deadline=now_ns+self.timeout_ns
        self._command(mavlink.MAV_CMD_LOGGING_START)
    def stop(self,now_ns):
        if self.state!='STREAMING':raise ValueError('cannot stop inactive session')
        self.state='STOPPING';self.deadline=now_ns+self.timeout_ns
        self._command(mavlink.MAV_CMD_LOGGING_STOP)
    def receive(self,wire,now_ns):
        if self.state in ('IDLE','FAILED','COMPLETE'):raise ValueError('session inactive')
        if self.last_receipt is not None and now_ns<self.last_receipt:self.fail('receipt clock reversed')
        self.last_receipt=now_ns
        for row in self.decoder.feed(wire,now_ns):
            if row.get('event')=='bad_data':self.fail('MAVLink framing or CRC failure')
            if (row.get('sysid'),row.get('compid'))!=self.source:continue
            f=row.get('fields',{})
            if row.get('type')=='COMMAND_ACK':
                command=f.get('command')
                expected=mavlink.MAV_CMD_LOGGING_START if self.state=='STARTING' else mavlink.MAV_CMD_LOGGING_STOP
                if self.state in ('STARTING','STOPPING') and command==expected:
                    if f.get('result')!=mavlink.MAV_RESULT_ACCEPTED:self.fail('logging command rejected')
                    self.state='STREAMING' if self.state=='STARTING' else 'DRAINING'
                    self.deadline=now_ns+self.timeout_ns
            elif row.get('type') in ('LOGGING_DATA','LOGGING_DATA_ACKED'):
                if self.state not in ('STREAMING','STOPPING','DRAINING'):self.fail('logging data before start acknowledgement')
                try:acks=self.reassembler.feed(row)
                except ValueError as exc:self.fail(str(exc))
                for sequence in acks:self.encoder.logging_ack_send(*self.source,sequence)
                self.deadline=now_ns+self.timeout_ns
    def poll(self,now_ns):
        if self.deadline is not None and now_ns>=self.deadline:
            if self.state=='DRAINING':return self.finish()
            if self.state in ('STARTING','STREAMING','STOPPING'):self.fail('logging session timeout')
        return None
    def finish(self):
        if self.state!='DRAINING':raise ValueError('STOP acknowledgement and drain required')
        try:raw=self.reassembler.finish()
        except ValueError as exc:self.fail(str(exc))
        self.state='COMPLETE';self.deadline=None;return raw
    def summary(self):
        return dict(self.reassembler.summary(),state=self.state,session_failure=self.failure,
                    upstream_source_complete='UNKNOWN',openvins_admissible=False)
