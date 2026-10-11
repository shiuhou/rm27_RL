"""Receive current simulation images/IMU and return observed native VIO state.

This process mounts no scene or physical state. Native state crosses the UNIX
socket only after the pinned estimator has processed a camera measurement.
"""
import base64
import json
from pathlib import Path
import select
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from transport import PacketDecoder, encode_packet


def stamp(message, ns, frame):
    message.header.stamp.sec = ns // 10**9
    message.header.stamp.nanosec = ns % 10**9
    message.header.frame_id = frame


def main():
    native = node = log = connection = None
    counts = dict(camera=0, imu=0, native_state=0)
    result = dict(passed=False, run_type='live current-flight native VIO',
                  flight_qualified=False, truth_mounted=False)
    try:
        connection = socket.socket(socket.AF_UNIX)
        connection.connect('/ipc/native.sock')
        connection.settimeout(1.)
        rclpy.init()
        node = Node('rm27_live_simulation_sensors')
        cam_pub = node.create_publisher(Image, '/cam0/image_raw', QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        imu_pub = node.create_publisher(Imu, '/imu0', QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE))
        with Path('/out/native_state.jsonl').open('x', buffering=1) as states:
            def receive(message):
                row = json.loads(message.data)
                row['receipt_monotonic_ns'] = time.monotonic_ns()
                states.write(json.dumps(row, allow_nan=False) + '\n')
                connection.sendall(encode_packet(dict(type='native_state', row=row)))
                counts['native_state'] += 1
            subscription = node.create_subscription(String, '/rm27_native_state', receive, 1000)
            log = Path('/out/native.log').open('x')
            native = subprocess.Popen(['/ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf', '--ros-args',
                '-p', 'config_path:=/out/config/estimator_config.yaml', '-p', 'verbosity:=INFO'],
                stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while not (cam_pub.get_subscription_count() and imu_pub.get_subscription_count()):
                if native.poll() is not None:
                    raise RuntimeError('OpenVINS exited before subscribing')
                if time.monotonic() > deadline:
                    raise TimeoutError('sensor discovery timed out')
                rclpy.spin_once(node, timeout_sec=.05)
            connection.sendall(encode_packet(dict(type='ready')))
            decoder = PacketDecoder()
            stopping = False
            last_input = time.monotonic()
            previous_ns = -1
            while not stopping:
                if native.poll() is not None:
                    raise RuntimeError('native estimator exited during input')
                if time.monotonic() - last_input > 30:
                    raise TimeoutError('live input timeout')
                if select.select([connection], [], [], .001)[0]:
                    data = connection.recv(1024 * 1024)
                    if not data:
                        raise RuntimeError('sensor source disconnected')
                    for row in decoder.feed(data):
                        last_input = time.monotonic()
                        if row['type'] == 'stop':
                            stopping = True
                            break
                        ns = row['source_time_ns']
                        if type(ns) is not int or ns < previous_ns:
                            raise ValueError('sensor time reversed')
                        previous_ns = ns
                        if row['type'] == 'imu':
                            msg = Imu(); stamp(msg, ns, 'body_flu')
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = row['gyro']
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = row['accel']
                            imu_pub.publish(msg); counts['imu'] += 1
                        elif row['type'] == 'camera':
                            payload = base64.b64decode(row['gray_base64'], validate=True)
                            if len(payload) != row['width'] * row['height']:
                                raise ValueError('image length mismatch')
                            msg = Image(); stamp(msg, ns, 'camera_optical')
                            msg.width = row['width']; msg.height = row['height']; msg.step = msg.width
                            msg.encoding = 'mono8'; msg.data = payload
                            cam_pub.publish(msg); counts['camera'] += 1
                        else:
                            raise ValueError('unsupported live input')
                        rclpy.spin_once(node, timeout_sec=0)
                rclpy.spin_once(node, timeout_sec=0)
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            native.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 15
            while native.poll() is None:
                if time.monotonic() > deadline:
                    raise TimeoutError('native estimator shutdown timed out')
                rclpy.spin_once(node, timeout_sec=.01)
            result.update(native_exit_code=native.returncode,
                          passed=native.returncode == 0 and counts['native_state'] > 0)
            connection.sendall(encode_packet(dict(type='complete', result=result)))
    except Exception as exc:
        result['failure'] = str(exc)
    finally:
        if native and native.poll() is None:
            native.terminate()
            try:
                native.wait(timeout=5)
            except subprocess.TimeoutExpired:
                native.kill(); native.wait()
        if log:
            log.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if connection:
            connection.close()
        result['counts'] = counts
        Path('/out/runtime_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
