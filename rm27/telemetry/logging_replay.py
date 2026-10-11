"""Saved ULog -> synthetic MAVLink wire -> finite logging session, offline only."""
import argparse
from bisect import bisect_left
import hashlib
import json
from pathlib import Path
import time

from pymavlink.dialects.v20 import common as mavlink

from .logging_session import LoggingSession, module


class CommandSink:
    """Record commands/ACKs without a serial port, socket or device connection."""
    def __init__(self,path):self.stream=Path(path).open('xb');self.messages=0
    def write(self,wire):self.stream.write(wire);self.messages+=1
    def close(self):self.stream.close()


def packets(raw,starts,encoder,chunk=249):
    for number,begin in enumerate(range(0,len(raw),chunk)):
        data=raw[begin:begin+chunk];index=bisect_left(starts,begin)
        offset=starts[index]-begin if index<len(starts) and starts[index]<begin+len(data) else 255
        kind=mavlink.MAVLink_logging_data_acked_message if number==0 else mavlink.MAVLink_logging_data_message
        message=kind(1,191,number&65535,len(data),offset,list(data)+[0]*(249-len(data)))
        wire=message.pack(encoder);encoder.seq=(encoder.seq+1)&255
        yield number,wire


def run(source,output,*,drop_packet=None,retry_header=True):
    source,output=Path(source).resolve(),Path(output).resolve()
    original=source.read_bytes();source_hash=hashlib.sha256(original).hexdigest()
    output.mkdir(parents=True,exist_ok=False)
    summary=dict(passed=False,run_type='saved-input replay through synthetic MAVLink wire',
        gate_version='PX4_ULOG_FINITE_SAVED_WIRE_V1',source=str(source),source_sha256=source_hash,
        source_bytes=len(original),drop_packet=drop_packet,retry_header=retry_header,
        hardware_connected=False,upstream_source_complete='UNKNOWN',openvins_admissible=False)
    sink=CommandSink(output/'commands_and_acks.mavlink');session=LoggingSession(sink,timeout_s=.1)
    encoder=mavlink.MAVLink(None,srcSystem=1,srcComponent=1);now=0;wall=time.monotonic()
    def ack(command):return mavlink.MAVLink_command_ack_message(command,mavlink.MAV_RESULT_ACCEPTED).pack(encoder)
    try:
        starts,_=module.validate_ulog(original)
        session.start(now);now+=1;session.receive(ack(mavlink.MAV_CMD_LOGGING_START),now)
        with (output/'incoming.mavlink').open('xb') as wire_output:
            for number,wire in packets(original,starts,encoder):
                if number==drop_packet:continue
                now+=1_000_000;wire_output.write(wire);session.receive(wire,now)
                if number==0 and retry_header:
                    now+=1;wire_output.write(wire);session.receive(wire,now)
        session.stop(now);now+=1;session.receive(ack(mavlink.MAV_CMD_LOGGING_STOP),now)
        restored=session.poll(now+100_000_000)
        if restored!=original:raise ValueError('Reassembled log differs from saved source')
        (output/'reassembled.ulg').write_bytes(restored)
        summary.update(passed=drop_packet is None,byte_identical=True,
            reassembled_sha256=hashlib.sha256(restored).hexdigest(),expected_outcome='complete byte-identical finite session')
    except ValueError as exc:
        summary.update(passed=drop_packet is not None and session.state=='FAILED' and 'sequence' in str(exc),
            rejected=str(exc),byte_identical=False,
            expected_outcome='terminal sequence-loss rejection; no partial ULog returned' if drop_packet is not None else 'complete finite session')
    finally:
        sink.close();summary.update(session=session.summary(),elapsed_s=time.monotonic()-wall,
            source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==source_hash,
            runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        summary['passed']=summary['passed'] and summary['source_unchanged']
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--drop-packet',type=int)
    a=p.parse_args()
    if a.drop_packet is not None and a.drop_packet<1:p.error('drop a non-header packet (index >= 1)')
    result=run(a.source,a.output,drop_packet=a.drop_packet)
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
