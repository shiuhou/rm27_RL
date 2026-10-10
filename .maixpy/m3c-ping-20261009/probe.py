"""Bounded bench diagnostic: passive discovery, optional three PINGs only.

No pinmux changes, no heartbeat/control/parameter transmission. Stdlib only.
CRC checking here supports HEARTBEAT/PING only, not a general MAVLink parser.
"""
import argparse
import copy
import glob
import json
import os
import select
import signal
import struct
import time


def crc(data):
    value = 0xffff
    for byte in data:
        tmp = byte ^ (value & 255)
        tmp ^= (tmp << 4) & 255
        value = (value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return value


def ping_frame(stamp, sequence):
    payload = struct.pack('<QIBB', stamp, sequence, 0, 0)
    header = bytes([14, sequence, 246, 191, 4])
    return b'\xfe' + header + payload + struct.pack('<H', crc(header + payload + b'\xed'))


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 12:
            b = self.buffer
            if b[0] not in (253, 254):
                del b[0]
                continue
            v2 = b[0] == 253
            header = 10 if v2 else 6
            size = header + b[1] + 2
            signed = v2 and b[2] & 1
            wire_size = size + (13 if signed else 0)
            msgid = int.from_bytes(b[7:10], 'little') if v2 else b[5]
            extra = {0: 50, 4: 237}.get(msgid)
            if extra is None or (v2 and b[2] & ~1):
                del b[0]
                continue
            if len(b) < wire_size:
                break
            frame = bytes(b[:wire_size])
            expected = struct.unpack('<H', frame[size-2:size])[0]
            if crc(frame[1:size-2] + bytes([extra])) != expected:
                del b[0]
                continue
            del b[:wire_size]
            # Signed frames require authentication, which this probe cannot provide.
            if signed:
                continue
            sysid, compid = (frame[5], frame[6]) if v2 else (frame[3], frame[4])
            payload = frame[header:size-2]
            result.append((msgid, sysid, compid, payload, frame.hex()))
        return result


def fresh_disarmed(hb, now):
    return hb is not None and 0 <= now - hb[0] <= 2.5 and not (hb[1] & 128)


def emit(**data):
    print(json.dumps(data), flush=True)


def owners(path):
    return [entry for entry in glob.glob('/proc/[0-9]*/fd/*')
            if os.path.exists(entry) and os.path.realpath(entry) == path]


def run_port(path, transmit):
    import fcntl
    import termios
    if path not in ['/dev/ttyS' + str(i) for i in range(5)]:
        raise ValueError('Port outside external UART allowlist')
    with open('/proc/consoles') as consoles:
        console_names = [line.split()[0] for line in consoles if line.strip()]
    if os.path.basename(path) in console_names:
        raise RuntimeError('Active system console: do not open')
    if owners(path):
        emit(event='busy_skip', port=path)
        return False
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old = None
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        old = termios.tcgetattr(fd)
        new = copy.deepcopy(old)
        new[0] = new[1] = new[3] = 0
        new[2] = (new[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)) | termios.CS8 | termios.CREAD | termios.CLOCAL
        new[4] = new[5] = termios.B115200
        new[6][termios.VMIN] = new[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, new)
        applied = termios.tcgetattr(fd)
        assert applied[4:6] == [termios.B115200, termios.B115200]
        emit(event='opened', port=path, baud=115200, transmit_enabled=transmit)
        parser, hb, sent, replies = Parser(), None, {}, set()
        start = time.monotonic()
        deadline = start + (10 if transmit else 3.5)
        next_tx, received = start, 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if select.select([fd], [], [], 0.1)[0]:
                data = os.read(fd, 4096)
                received += len(data)
                for msgid, sysid, compid, payload, wire in parser.feed(data):
                    if (sysid, compid) != (1, 1):
                        continue
                    if msgid == 0:
                        payload = payload.ljust(9, b'\x00')
                        # Require PX4 autopilot (12), protocol version 3, disarmed.
                        if payload[5] != 12 or payload[8] != 3:
                            continue
                        hb = (time.monotonic(), payload[6])
                        emit(event='px4_heartbeat', port=path, base_mode=payload[6], frame=wire)
                        if payload[6] & 128:
                            raise RuntimeError('Armed heartbeat: abort')
                    elif msgid == 4:
                        stamp, seq, ts, tc = struct.unpack('<QIBB', payload.ljust(14, b'\x00'))
                        if (ts, tc) == (246, 191) and seq in sent and sent[seq][0] == stamp:
                            replies.add(seq)
                            emit(event='matched_ping_reply', port=path, sequence=seq,
                                 rtt_ms=(time.monotonic()-sent[seq][1])*1000, frame=wire)
            now = time.monotonic()
            if transmit and len(sent) < 3 and now >= next_tx and fresh_disarmed(hb, now):
                seq = len(sent)
                stamp = time.time_ns() // 1000
                frame = ping_frame(stamp, seq)
                written = os.write(fd, frame)
                if written != len(frame):
                    raise RuntimeError('Partial UART write; abort without retry')
                sent[seq] = (stamp, time.monotonic())
                next_tx = now + 1
                emit(event='ping_sent', port=path, sequence=seq, bytes=written, frame=frame.hex())
        emit(event='port_result', port=path, received_bytes=received,
             heartbeat_seen=hb is not None, sent=len(sent), matched_replies=len(replies))
        return hb is not None
    finally:
        try:
            if old is not None:
                termios.tcsetattr(fd, termios.TCSANOW, old)
                emit(event='restored', port=path, termios_equal=termios.tcgetattr(fd) == old)
        finally:
            fcntl.ioctl(fd, termios.TIOCNXCL)
            os.close(fd)


def main():
    args = argparse.ArgumentParser()
    group = args.add_mutually_exclusive_group()
    ports = ['/dev/ttyS' + str(i) for i in range(5)]
    group.add_argument('--ping-port', choices=ports)
    group.add_argument('--receive-port', choices=ports)
    opt = args.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    if opt.ping_port:
        run_port(opt.ping_port, True)
    elif opt.receive_port:
        run_port(opt.receive_port, False)
    else:
        # Never probe the system console ttyS0 or internal ttyS5. No TX in scan.
        for path in ['/dev/ttyS4', '/dev/ttyS2', '/dev/ttyS1', '/dev/ttyS3']:
            if run_port(path, False):
                break


if __name__ == '__main__':
    main()
