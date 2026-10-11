"""Current MuJoCo sensors -> isolated native OpenVINS -> development ODOMETRY.

Registration uses one time-matched PX4 flow estimate. After registration this
path never reads a physical pose for estimator alignment or flight commands.
Qualification remains false until independent live acceptance passes.
"""
import base64
from collections import Counter, deque
from dataclasses import asdict
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
import numpy as np
from scipy.spatial.transform import Rotation

from .dataset import camera_to_imu, digest, intrinsics
from .mujoco_camera import MujocoCamera
from .image_target_detector import DetectorConfig, ImageTargetDetector
from .native_contract import NativeContract
from .openvins import ROOT, prepare_config
from .transport import PacketDecoder, encode_packet
from ...autopilot.px4.odometry import OdometryAdapter, Registration

DEFAULT_NATIVE = ROOT / 'artifacts/openvins-native-evidence-build-01/installation.json'


class LiveVio:
    def __init__(self, plant, output, *, installation=DEFAULT_NATIVE, camera_config=None,
                 player=None, player_mounts=()):
        self.plant = plant
        self.output = Path(output) / 'vio'
        self.installation = Path(installation)
        self.player=Path(player) if player else ROOT/'tools/openvins/live_simulation_ros2.py'
        self.player_mounts=tuple(player_mounts)
        self.camera_config_path=Path(camera_config) if camera_config else ROOT/'configs/px4_sim_camera.json'
        self.process = self.connection = self.server = self.camera = self.log = None
        self.ipc = None; self.decoder = PacketDecoder(); self.contract = NativeContract()
        self.adapter = None; self.registration = None
        self.history = deque(maxlen=500)
        self.counts = Counter(); self.states = Counter(); self.latencies = []; self.rejections = Counter()
        self.source_ages=[]; self.publication_ages=[]; self.last_target=None;self.last_frame=None
        self.next_image = 0.; self.last_sensor = -1.; self.closed = False
        self.last_camera_source_us=-1

    def __enter__(self):
        try:
            install = json.loads(self.installation.read_text())
            if install['commit'] != '69488123ed9362dd44b6f28e7f4680abbff1442b' or not install.get('instrumentation_sha256'):
                raise ValueError('pinned instrumented OpenVINS required')
            for name, expected in install['files_sha256'].items():
                if digest(name) != expected:
                    raise ValueError('native input hash changed: ' + name)
            binary = Path(install['workspace']) / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
            if digest(binary) != install['binary_sha256']:
                raise ValueError('native binary hash changed')
            self.output.mkdir(parents=True, exist_ok=False)
            self.config = json.loads(self.camera_config_path.read_text())
            settings = json.loads((ROOT / 'configs/px4_sim_vio_dev.json').read_text())
            config_dir = self.output / 'config'; config_dir.mkdir()
            c = self.config
            source = dict(configuration=c, imu_sampling_hz=250., calibration=dict(
                intrinsics=intrinsics(c['width'], c['height'], c['fovy_deg']),
                distortion=[0., 0., 0., 0.], T_imu_cam=camera_to_imu(c).tolist()))
            prepare_config(source, install, config_dir, stationary_init=True, settings=settings)
            self.rng = np.random.default_rng(c['seed'])
            self.detector=ImageTargetDetector(DetectorConfig(fovy_deg=c['fovy_deg']))
            self.camera = MujocoCamera(self.plant.model, **{k:c[k] for k in
                ['width', 'height', 'fovy_deg', 'body_name', 'offset_body', 'forward_body', 'up_body']})
            self.ipc = Path(tempfile.mkdtemp(prefix='rm27-vio-'))
            self.server = socket.socket(socket.AF_UNIX); self.server.bind(str(self.ipc / 'native.sock'))
            self.server.listen(1); self.server.settimeout(45)
            self.container = 'rm27-live-vio-' + uuid.uuid4().hex
            command = ['docker', 'run', '--rm', '--name', self.container, '--network', 'none',
                '--cpus', '4', '--memory', '12g', '--user', f'{os.getuid()}:{os.getgid()}',
                '-e', 'ROS_DOMAIN_ID=119', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'OPENBLAS_NUM_THREADS=1',
                '-v', f'{self.player}:/player.py:ro',
                '-v', f'{ROOT / "rm27/perception/sitl/transport.py"}:/transport.py:ro',
                '-v', f'{install["workspace"]}:/ws:ro', '-v', f'{self.output}:/out',
                '-v', f'{self.ipc}:/ipc', install['image'], 'bash', '-c',
                'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 /player.py']
            for source,destination in self.player_mounts:
                command[2:2]=['-v',f'{Path(source).resolve()}:{destination}:ro']
            self.log = (self.output / 'container.log').open('x')
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
            self.connection, _ = self.server.accept(); self.connection.settimeout(2.)
            deadline = time.monotonic() + 40
            ready = False
            while not ready:
                if time.monotonic() > deadline:
                    raise TimeoutError('live native readiness timed out')
                for row in self._receive(.05):
                    ready |= row['type'] == 'ready'
            self.manifest = dict(command=command, run_type='live current-flight native VIO',
                flight_qualified=False, truth_mounted=False, installation_sha256=digest(self.installation),
                binary_sha256=install['binary_sha256'], camera_configuration=c, estimator_settings=settings,
                runner_sha256=digest(__file__), player_sha256=digest(self.player),
                player_mount_sha256={str(p):digest(p) for p,_ in self.player_mounts},
                configuration_sha256={p.name:digest(p) for p in config_dir.iterdir()},
                registration_policy='one time-matched PX4 flow estimate; no ongoing truth alignment')
            (self.output / 'launch.json').write_text(json.dumps(self.manifest, indent=2) + '\n')
            self.estimates = (self.output / 'estimates.jsonl').open('x', buffering=1)
            self.inputs = (self.output / 'inputs.jsonl').open('x', buffering=1)
            self.targets = (self.output / 'targets.jsonl').open('x', buffering=1)
            (self.output / 'images').mkdir()
            return self
        except BaseException:
            self.close(); raise

    def _receive(self, timeout=0.):
        rows = []
        while select.select([self.connection], [], [], timeout)[0]:
            data = self.connection.recv(1024 * 1024)
            if not data:
                if self.process.poll() is None:
                    raise RuntimeError('native VIO transport disconnected')
                break
            rows.extend(self.decoder.feed(data)); timeout = 0.
        return rows

    def sensors(self):
        t = float(self.plant.data.time)
        if t <= self.last_sensor:
            return
        self.last_sensor = t; ns = round(t * 1e9)
        s = self.plant.state()['imu']
        gyro = np.asarray(s['gyro']) * [1, -1, -1] + self.rng.normal(0, self.config['gyroscope_noise_std_rad_s'], 3)
        accel = np.asarray(s['accel_body']) * [1, -1, -1] + self.rng.normal(0, self.config['accelerometer_noise_std_m_s2'], 3)
        row = dict(type='imu', source_time_ns=ns, gyro=gyro.tolist(), accel=accel.tolist())
        self.connection.sendall(encode_packet(row)); self.inputs.write(json.dumps(row) + '\n'); self.counts['imu'] += 1
        if t + 1e-9 < self.next_image:
            return
        self.next_image += 1. / self.config['camera_hz']
        if self.next_image<t:self.next_image=t+1./self.config['camera_hz']
        frame = self.camera.capture(self.plant.data, round(t * 1e6))
        if frame is None:
            self.last_frame=None;self.counts['camera_dropped']+=1
            return
        self.publish_camera(frame,round(t*1e6))

    def publish_camera(self,frame,received_us):
        """Preserve acquisition time through detection, storage and native IPC."""
        if frame.timestamp_us>received_us:
            self.last_frame=None;self.last_target=None
            self.counts['camera_future']+=1;return
        self.last_frame=frame
        self.last_target=self.detector.detect(frame,now_us=received_us)
        self.targets.write(json.dumps(asdict(self.last_target),allow_nan=False)+'\n')
        self.counts['safe_target']+=self.last_target.safe_for_control
        if frame.timestamp_us<=self.last_camera_source_us:
            self.counts['camera_non_increasing']+=1;return
        self.last_camera_source_us=frame.timestamp_us
        ns=frame.timestamp_us*1000
        gray = cv2.cvtColor(frame.image_rgb, cv2.COLOR_RGB2GRAY)
        name = str(ns) + '.png'; cv2.imwrite(str(self.output / 'images' / name), gray)
        row = dict(type='camera', source_time_ns=ns, width=frame.width, height=frame.height,
                   gray_base64=base64.b64encode(gray.tobytes()).decode())
        self.connection.sendall(encode_packet(row))
        self.inputs.write(json.dumps(dict(type='camera', source_time_ns=ns, filename=name,
            received_time_ns=received_us*1000,
            sha256=digest(self.output / 'images' / name))) + '\n'); self.counts['camera'] += 1

    def observe_telemetry(self, telemetry, now):
        required = ['local_position', 'quaternion', 'local_time', 'attitude_time', 'estimator_time']
        if not all(k in telemetry for k in required):
            return
        if max(now-telemetry[k] for k in ['local_time','attitude_time','estimator_time']) > .2:
            return
        flags = telemetry.get('estimator_flags', 0)
        if not flags & 8 or not flags & 2:  # relative XY position and horizontal velocity
            return
        self.history.append((now, np.array(telemetry['local_position']), np.array(telemetry['quaternion'])))

    def packets(self, now):
        packets = []
        for message in self._receive():
            if message['type'] != 'native_state':
                continue
            row = message['row']; self.counts['native_state'] += 1
            try:
                e = self.contract.estimate(row)
                self.states[e.tracking_state.value] += 1
                self.latencies.append(e.quality['processing_duration_s'])
                self.estimates.write(json.dumps(asdict(e), allow_nan=False) + '\n')
                if not e.valid:
                    self.rejections[e.tracking_state.value] += 1; continue
                source_s = e.source_time.raw_value * 1e-9
                self.source_ages.append(now-source_s)
                self.publication_ages.append((time.monotonic_ns()-e.publish_time.raw_value)*1e-9)
                if self.adapter is None:
                    if not self.history:
                        self.rejections['no initial valid PX4 estimate'] += 1; continue
                    t, p, q = min(self.history, key=lambda h: abs(h[0] - source_s))
                    if abs(t-source_s) > .05:
                        self.rejections['registration time mismatch'] += 1; continue
                    r_ekf = Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
                    r_native = Rotation.from_quat(np.array(e.orientation_wxyz)[[1,2,3,0]]).as_matrix()
                    child = np.diag([1., -1., -1.])
                    rotation = r_ekf @ child @ r_native.T
                    translation = p - rotation @ np.array(e.translation)
                    self.registration = dict(source_time_s=source_s, telemetry_time_s=t,
                        rotation_ned_parent=rotation.tolist(), translation_ned_parent=translation.tolist(),
                        evidence='PX4 optical-flow/range/IMU estimate, registered once')
                    (self.output/'registration.json').write_text(json.dumps(self.registration,indent=2)+'\n')
                    registration = Registration(e.parent_frame,e.child_frame,'mujoco_simulation',
                        rotation,translation,child,publication_clock_domain='native_monotonic')
                    self.adapter = OdometryAdapter(registration,max_age_s=.3,allow_unqualified_simulation=True)
                packets.append(self.adapter.packet(e, round(now*1e9), publication_now_ns=time.monotonic_ns()))
                self.counts['odometry'] += 1
            except ValueError as exc:
                self.rejections[str(exc)] += 1
        return packets

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.connection and self.process and self.process.poll() is None:
            try:
                self.connection.sendall(encode_packet(dict(type='stop')))
                deadline=time.monotonic()+20
                while self.process.poll() is None and time.monotonic()<deadline:
                    self._receive(.05)
            except (OSError,RuntimeError):
                pass
        if self.process and self.process.poll() is None:
            subprocess.run(['docker','stop','--time','5',self.container],capture_output=True,timeout=15)
            self.process.wait(timeout=15)
        for name in ['estimates','inputs','targets','log']:
            stream=getattr(self,name,None)
            if stream:stream.close()
        for sock in [self.connection,self.server]:
            if sock:sock.close()
        if self.camera:self.camera.close()
        if self.ipc:shutil.rmtree(self.ipc)
        if self.output.exists():
            summary=dict(passed=bool(self.process and self.process.returncode==0 and self.counts['odometry']>0),
                run_type='live current-flight native VIO development',flight_qualified=False,
                counts=dict(self.counts),tracking_states=dict(self.states),rejections=dict(self.rejections),
                registration=self.registration,truth_mounted=False,
                processing_p95_s=float(np.percentile(self.latencies,95)) if self.latencies else None,
                processing_maximum_s=max(self.latencies) if self.latencies else None,
                source_age_p95_s=float(np.percentile(self.source_ages,95)) if self.source_ages else None,
                publication_age_p95_s=float(np.percentile(self.publication_ages,95)) if self.publication_ages else None)
            (self.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

    def __exit__(self,*args):
        self.close()
