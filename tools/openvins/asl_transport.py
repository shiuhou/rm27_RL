"""ASL ZIP to verified ROS2 cam0/IMU transport, never an estimator adapter.

Dataset integer timestamps are preserved as header and storage timestamps.
Equal-time events use IMU first; original per-sensor order is never altered.
Truth, cam1 and calibration are not published. Calibration is supplied separately.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import zipfile


def split_stamp(ns):
    if type(ns) is not int or ns < 0 or ns // 10**9 >= 2**31:
        raise ValueError('timestamp must be nonnegative integer ns in ROS Time range')
    return divmod(ns, 10**9)


def parse_rows(text, sensor):
    if sensor not in ('cam0', 'imu0'):
        raise ValueError('only cam0 and imu0 supported')
    rows = []
    previous = -1
    for row in csv.reader(line for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#')):
        if len(row) != (2 if sensor == 'cam0' else 7):
            raise ValueError('invalid CSV column count')
        ns = int(row[0])
        split_stamp(ns)
        if ns <= previous:
            raise ValueError('timestamps must be strictly increasing')
        previous = ns
        if sensor == 'cam0':
            if row[1] != f'{ns}.png':
                raise ValueError('camera filename must match timestamp')
            payload = row[1]
        else:
            payload = tuple(float(value) for value in row[1:])
            if not all(math.isfinite(value) for value in payload):
                raise ValueError('IMU values must be finite')
        rows.append((ns, payload))
    if not rows:
        raise ValueError('empty sensor sequence')
    return rows


def merge_events(cam, imu):
    return sorted([(ns, 'cam0', data) for ns, data in cam] +
                  [(ns, 'imu0', data) for ns, data in imu],
                  key=lambda entry: (entry[0], 0 if entry[1] == 'imu0' else 1))


def sha256_file(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def build_bag(archive, output):
    import cv2
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import serialize_message, deserialize_message
    from sensor_msgs.msg import Image, Imu

    archive, output = Path(archive), Path(output)
    if output.exists():
        raise FileExistsError(output)
    types = {'cam0': Image, 'imu0': Imu}
    topics = {'cam0': '/cam0/image_raw', 'imu0': '/imu0'}
    source_hash = sha256_file(archive)
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise ValueError('duplicate ZIP members')
        cam = parse_rows(z.read('mav0/cam0/data.csv').decode(), 'cam0')
        imu = parse_rows(z.read('mav0/imu0/data.csv').decode(), 'imu0')
        events = merge_events(cam, imu)
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for index, sensor in enumerate(types):
            writer.create_topic(rosbag2_py.TopicMetadata(id=index + 1, name=topics[sensor],
                                type='sensor_msgs/msg/' + types[sensor].__name__, serialization_format='cdr'))
        for ns, sensor, payload in events:
            message = types[sensor]()
            message.header.stamp.sec, message.header.stamp.nanosec = split_stamp(ns)
            message.header.frame_id = sensor
            if sensor == 'cam0':
                pixels = cv2.imdecode(np.frombuffer(z.read('mav0/cam0/data/' + payload), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
                    raise ValueError('expected uint8 mono image')
                message.height, message.width = pixels.shape
                message.encoding = 'mono8'
                message.is_bigendian = 0
                message.step = message.width
                message.data = pixels.tobytes()
            else:
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = payload[:3]
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = payload[3:]
                message.orientation_covariance[0] = -1.0  # orientation not measured
                # All-zero gyro/accel covariance means unknown, not noiseless.
            writer.write(topics[sensor], serialize_message(message), ns)
        del writer  # flush metadata and close SQLite before independent read-back

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        counts = {'cam0': 0, 'imu0': 0}
        for ns, sensor, payload in events:
            if not reader.has_next():
                raise ValueError('bag ended before source')
            topic, data, storage_ns = reader.read_next()
            message = deserialize_message(data, types[sensor])
            stamp = message.header.stamp
            if topic != topics[sensor] or storage_ns != ns or stamp.sec * 10**9 + stamp.nanosec != ns:
                raise ValueError('topic/order/timestamp changed')
            if sensor == 'cam0':
                expected = cv2.imdecode(np.frombuffer(z.read('mav0/cam0/data/' + payload), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                if (message.height, message.width) != expected.shape or message.encoding != 'mono8' or message.step != message.width or message.is_bigendian != 0 or bytes(message.data) != expected.tobytes():
                    raise ValueError('decoded image content changed')
            else:
                actual = (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                          message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z)
                if actual != payload or message.orientation_covariance[0] != -1.0:
                    raise ValueError('IMU values changed')
            counts[sensor] += 1
        if reader.has_next():
            raise ValueError('bag has unexpected extra samples')
        del reader
    if sha256_file(archive) != source_hash:
        raise ValueError('source archive changed during conversion')
    report = {'source': str(archive), 'source_sha256': source_hash,
              'verified_counts': counts, 'readback_exact': True,
              'timestamp_policy': 'integer ns unchanged; IMU first at equal timestamps',
              'truth_included': False, 'scope': 'transport only; no estimator or delivery qualification',
              'artifacts_sha256': {p.name: sha256_file(p) for p in sorted(output.iterdir()) if p.is_file()}}
    with (output / 'source_verification.json').open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(build_bag(args.archive, args.output), indent=2))
