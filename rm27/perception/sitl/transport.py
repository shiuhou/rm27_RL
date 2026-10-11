"""Bounded length-prefixed JSON transport for local simulated sensors."""
import json
import struct

MAX_PACKET_BYTES = 2 * 1024 * 1024


def encode_packet(row):
    payload = json.dumps(row, allow_nan=False, separators=(',', ':')).encode()
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError('sensor packet exceeds limit')
    return struct.pack('!I', len(payload)) + payload


class PacketDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        rows = []
        while len(self.buffer) >= 4:
            length = struct.unpack('!I', self.buffer[:4])[0]
            if not 0 < length <= MAX_PACKET_BYTES:
                raise ValueError('invalid sensor packet length')
            if len(self.buffer) < length + 4:
                break
            rows.append(json.loads(self.buffer[4:4 + length]))
            del self.buffer[:4 + length]
        return rows
