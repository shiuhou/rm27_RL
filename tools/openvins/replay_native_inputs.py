"""Replay saved image/IMU bytes through native VIO, without simulator inputs."""
import argparse
import base64
import json
import os
from pathlib import Path
import select
import shutil
import socket
import subprocess
import tempfile
import time
import uuid

import cv2

from rm27.perception.sitl.dataset import digest
from rm27.perception.sitl.transport import PacketDecoder, encode_packet

ROOT = Path(__file__).resolve().parents[2]


def load_inputs(recording):
    recording = Path(recording)
    rows = [json.loads(line) for line in (recording / 'inputs.jsonl').open()]
    previous, delivery = {}, -1
    images = {}
    for row in rows:
        role = row['type']
        if role not in ('camera', 'imu'):
            raise ValueError('Only recorded image and IMU input is allowed')
        ns = row['source_time_ns']
        if type(ns) is not int or ns <= previous.get(role, -1):
            raise ValueError('Non-increasing recorded sensor acquisition')
        previous[role] = ns
        arrival = row.get('received_time_ns', ns)
        if type(arrival) is not int or arrival < ns or arrival < delivery:
            raise ValueError('Invalid recorded delivery schedule')
        delivery = arrival
        if role == 'camera':
            name = row['filename']
            if Path(name).name != name:
                raise ValueError('Recorded image must stay within its input directory')
            image = recording / 'images' / name
            if digest(image) != row['sha256']:
                raise ValueError('Recorded image changed: ' + name)
            images[name] = row['sha256']
    if not rows:
        raise ValueError('Empty recording')
    return rows, images


def run(recording, installation, output, *, form='joseph', rate=.5):
    if form not in ('original', 'joseph') or not 0 < rate <= 1:
        raise ValueError('Explicit native covariance form and playback rate0..1 required')
    recording, installation, output = map(lambda p:Path(p).resolve(), (recording, installation, output))
    rows, image_hashes = load_inputs(recording)
    install = json.loads(installation.read_text())
    for p, expected in install['files_sha256'].items():
        if digest(p) != expected:
            raise ValueError('Native installation changed: ' + p)
    binary = Path(install['workspace']) / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    if digest(binary) != install['binary_sha256']:
        raise ValueError('Native binary changed')
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(recording / 'config', output / 'config')
    player = (ROOT / 'tools/openvins/live_simulation_ros2.py').read_text()
    old = """                        if type(ns) is not int or ns < previous_ns:
                            raise ValueError('sensor time reversed')
                        previous_ns = ns"""
    if old not in player:
        raise ValueError('Pinned publisher context changed')
    player = player.replace('from transport import PacketDecoder, encode_packet',
        'from transport import PacketDecoder, encode_packet\nfrom sensor_clock import InputClock')
    player = player.replace('previous_ns = -1', 'clocks = InputClock()').replace(old,
        "                        clocks.observe(row['type'], ns)")
    player_path = output / 'player.py'
    player_path.write_text(player)
    container = 'rm27-recorded-native-' + uuid.uuid4().hex
    counts = dict(imu=0, camera=0, native_state=0)
    result = dict(passed=False, flight_qualified=False, truth_mounted=False,
        run_type='recorded-input native VIO; separate from current-flight acceptance')
    with tempfile.TemporaryDirectory(prefix='rm27-native-replay-') as temp:
        ipc = Path(temp)
        server = socket.socket(socket.AF_UNIX)
        server.bind(str(ipc / 'native.sock'))
        server.listen(1)
        server.settimeout(45)
        command = ['docker', 'run', '--rm', '--name', container, '--network', 'none', '--cpus', '2',
            '--memory', '6g', '--user', f'{os.getuid()}:{os.getgid()}',
            '-e', 'ROS_DOMAIN_ID=119', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'OPENBLAS_NUM_THREADS=1',
            '-e', 'RM27_COVARIANCE_FORM=' + form,
            '-v', f'{player_path}:/player.py:ro',
            '-v', f'{ROOT/"rm27/perception/sitl/transport.py"}:/transport.py:ro',
            '-v', f'{ROOT/"rm27/experiments/sensor_clock.py"}:/sensor_clock.py:ro',
            '-v', f'{install["workspace"]}:/ws:ro', '-v', f'{output}:/out', '-v', f'{ipc}:/ipc',
            install['image'], 'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 /player.py']
        manifest = dict(command=command, recording=str(recording), inputs_sha256=digest(recording/'inputs.jsonl'),
            image_sha256=image_hashes, source_time_policy='original acquisition; pacing uses original receipt times',
            installation_sha256=digest(installation), binary_sha256=digest(binary), covariance_form=form,
            playback_rate=rate, player_sha256=digest(player_path), runner_sha256=digest(__file__),
            config_sha256={p.name:digest(p) for p in (output/'config').iterdir()},
            transport_sha256=digest(ROOT/'rm27/perception/sitl/transport.py'),
            sensor_clock_sha256=digest(ROOT/'rm27/experiments/sensor_clock.py'))
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        connection = process = None
        decoder, received = PacketDecoder(), []
        try:
            with (output/'container.log').open('x') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                connection, _ = server.accept()
                connection.settimeout(5)
                def receive(timeout=0):
                    if select.select([connection], [], [], timeout)[0]:
                        data = connection.recv(1024*1024)
                        if not data:
                            raise RuntimeError('Native replay transport disconnected')
                        received.extend(decoder.feed(data))
                deadline = time.monotonic()+40
                while not any(r['type']=='ready' for r in received):
                    receive(.05)
                    if time.monotonic()>deadline:
                        raise TimeoutError('Native replay startup timed out')
                started = time.monotonic()
                origin = rows[0].get('received_time_ns', rows[0]['source_time_ns'])
                for row in rows:
                    arrival = row.get('received_time_ns', row['source_time_ns'])
                    target = started+(arrival-origin)*1e-9/rate
                    while time.monotonic()<target:
                        receive(max(0., min(.01, target-time.monotonic())))
                    if row['type']=='camera':
                        pixels = cv2.imread(str(recording/'images'/row['filename']), cv2.IMREAD_GRAYSCALE)
                        if pixels is None:
                            raise ValueError('Cannot decode verified recorded image')
                        packet = dict(type='camera', source_time_ns=row['source_time_ns'],
                            width=pixels.shape[1], height=pixels.shape[0],
                            gray_base64=base64.b64encode(pixels.tobytes()).decode())
                    else:
                        packet = row
                    connection.sendall(encode_packet(packet))
                    counts[row['type']] += 1
                    receive()
                connection.sendall(encode_packet(dict(type='stop')))
                deadline = time.monotonic()+35
                while not any(r['type']=='complete' for r in received):
                    receive(.05)
                    if time.monotonic()>deadline:
                        raise TimeoutError('Native replay completion timed out')
                status = process.wait(timeout=15)
                runtime = next(r['result'] for r in received if r['type']=='complete')
                result.update(passed=runtime.get('passed') is True and status==0,
                    returncode=status, native_result=runtime)
        except Exception as error:
            result['failure'] = f'{type(error).__name__}: {error}'
        finally:
            if connection:
                connection.close()
            server.close()
            if process and process.poll() is None:
                subprocess.run(['docker', 'stop', '--time', '3', container], capture_output=True, timeout=15)
                process.wait(timeout=15)
            counts['native_state'] = sum(r['type']=='native_state' for r in received)
            result['counts'] = counts
    (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--installation', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--form', choices=['original', 'joseph'], default='joseph')
    parser.add_argument('--rate', type=float, default=.5)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.recording, args.installation, args.output, form=args.form, rate=args.rate)['passed'] else 1)
