"""Pinned upstream CLI adapters. They launch only through the offline runner."""
import os
from pathlib import Path
import sys

from .common import checked_file, read_json, require, fixture_mode, write_json, number

PINS = {
    'stella_vslam': 'e445b5452535e781fdf6777ec33a06f5f8f5e416',
    'stella_vslam_examples': 'defc69eecc36e51cdda22885bb86954f08ad6887',
    'ORB-SLAM3': '4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4',
}


def camera_config(backend, calibration, provenance, nominal_fps, grayscale=False):
    """Exact pinhole OpenCV radtan5 conversion; unsupported models are rejected."""
    number(nominal_fps, 'nominal_fps', True)
    require(calibration.get('model') == 'pinhole' and calibration.get('distortion_model') == 'opencv_radtan5'
            and len(calibration['distortion_coefficients']) == 5,
            'BACKEND_CALIBRATION_UNSUPPORTED', 'supported: pinhole + opencv_radtan5 [k1,k2,p1,p2,k3] only')
    import json
    comment = f"# source_calibration_id: {json.dumps(calibration['calibration_id'])}\n# source_calibration_sha256: {provenance['sha256']}\n"
    k1, k2, p1, p2, k3 = calibration['distortion_coefficients']
    if backend == 'stella_vslam':
        fields = dict(name=calibration['calibration_id'], setup='monocular', model='perspective',
                      color_order='Gray' if grayscale else 'BGR', cols=calibration['image_width'],
                      rows=calibration['image_height'], fps=nominal_fps,
                      **{k:calibration[k] for k in ('fx','fy','cx','cy')}, k1=k1,k2=k2,p1=p1,p2=p2,k3=k3)
        return comment+'Camera:\n'+''.join(f'  {k}: {json.dumps(v)}\n' for k,v in fields.items())
    require(backend == 'ORB-SLAM3', 'BACKEND_UNSUPPORTED', backend)
    require(float(nominal_fps).is_integer(), 'BACKEND_RATE_UNSUPPORTED',
            'pinned ORB-SLAM3 Settings requires integer Camera.fps; do not round source cadence')
    fields = {'File.version':'1.0', 'Camera.type':'PinHole',
              **{f'Camera1.{k}':float(calibration[k]) for k in ('fx','fy','cx','cy')},
              'Camera1.k1':float(k1),'Camera1.k2':float(k2),'Camera1.p1':float(p1),'Camera1.p2':float(p2),'Camera1.k3':float(k3),
              'Camera.fps':int(nominal_fps), 'Camera.RGB':0, 'Camera.width':calibration['image_width'],
              'Camera.height':calibration['image_height'],
              'ORBextractor.nFeatures':1000,'ORBextractor.scaleFactor':1.2,'ORBextractor.nLevels':8,
              'ORBextractor.iniThFAST':20,'ORBextractor.minThFAST':7,
              'Viewer.KeyFrameSize':.05,'Viewer.KeyFrameLineWidth':1.0,'Viewer.GraphLineWidth':.9,
              'Viewer.PointSize':2.0,'Viewer.CameraSize':.08,'Viewer.CameraLineWidth':3.0,
              'Viewer.ViewpointX':0.,'Viewer.ViewpointY':-.7,'Viewer.ViewpointZ':-1.8,'Viewer.ViewpointF':500.}
    return '%YAML:1.0\n'+comment+''.join(f'{k}: {json.dumps(v)}\n' for k,v in fields.items())


class BackendAdapter:
    def __init__(self, backend, install_file=None):
        require(backend in ('stella_vslam', 'ORB-SLAM3'), 'BACKEND_UNSUPPORTED', backend)
        self.backend, self.install_file = backend, install_file

    def describe_version(self, *, _fixture_token=None):
        require(self.install_file is not None and Path(self.install_file).is_file(),
                'BACKEND_NOT_INSTALLED', f'{self.backend}: supply an external installation manifest')
        info = read_json(self.install_file)
        fixture_mode(_fixture_token, info)
        require(info.get('backend') == self.backend, 'BACKEND_INSTALL_INVALID', 'backend identity')
        binary = info.get('binary')
        require(binary and Path(binary).is_file() and os.access(binary, os.X_OK),
                'BACKEND_NOT_INSTALLED', str(binary))
        for key in ('binary_sha256', 'vocabulary', 'vocabulary_sha256', 'commit'):
            require(info.get(key), 'BACKEND_INSTALL_INVALID', key)
        checked_file(binary, info['binary_sha256'])
        checked_file(info['vocabulary'], info['vocabulary_sha256'])
        for path, digest in info.get('runtime_files_sha256', {}).items():
            checked_file(path, digest)
        if info.get('online_observation'):
            observer = info['online_observation']
            require(observer.get('format') == 'rm27_online_csv_v1', 'ONLINE_FORMAT_INVALID', 'installation format')
            require(observer.get('source_files_sha256'), 'BACKEND_INSTALL_INVALID', 'observer source hashes required')
            for path, digest in observer['source_files_sha256'].items():
                checked_file(path, digest)
        if info.get('source_patch'):
            checked_file(info['source_patch'], info['source_patch_sha256'])
        require(info['commit'] == PINS[self.backend], 'BACKEND_VERSION_UNSUPPORTED', 'adapter is verified against one pinned source revision')
        if self.backend == 'stella_vslam':
            require(info.get('examples_commit') == PINS['stella_vslam_examples'], 'BACKEND_VERSION_UNSUPPORTED', 'examples revision')
        return {**info, 'binary':str(Path(binary).resolve()), 'vocabulary':str(Path(info['vocabulary']).resolve()),
                'installation_manifest_sha256':checked_file(self.install_file)[1],
                'version_evidence':'operator build manifest + verified binary hash; build provenance not independently attested'}

    def prepare(self, root, frames, images, config_text, install, native_mask=None):
        require(native_mask is None, 'BACKEND_MASK_UNSUPPORTED', 'the pinned TUM CLI paths do not expose native feature masks')
        root = Path(root)
        sequence = root/'sequence'
        sequence.mkdir()
        # rgb.txt uses local generated names; no spaces or arbitrary shell quoting.
        image_dir = sequence/'rgb'
        image_dir.mkdir()
        lines = ['# RM27 offline monocular input\n', '# original source timestamps in seconds\n', '# timestamp image\n']
        for i,(frame,image) in enumerate(zip(frames,images)):
            link = image_dir/f'{i:08d}{image.suffix}'
            # Windows often blocks symlink creation unless Developer Mode or
            # SeCreateSymbolicLinkPrivilege is enabled. Copies preserve the
            # exact bytes and keep the offline experiment portable.
            try:
                link.symlink_to(image.resolve())
            except (OSError, NotImplementedError):
                import shutil
                shutil.copyfile(image, link)
            lines.append(f'{frame.encoded_pts} rgb/{link.name}\n')
        (sequence/'rgb.txt').write_text(''.join(lines))
        if self.backend == 'stella_vslam':
            # The pinned TUM loader requires a depth association even in monocular mode.
            # Identical times preserve PTS; its monocular branch NEVER reads depth pixels.
            (sequence/'depth.txt').write_text(''.join(lines))
            write_json(sequence/'association_note.json', {'depth_data_present':False,
                       'purpose':'unused association shim required by upstream TUM monocular loader; RGB aliases, not depth measurements'})
        config = root/'backend.yaml'
        config.write_text(config_text)
        raw = root/'raw'
        raw.mkdir()
        if self.backend == 'stella_vslam':
            command = [install['binary'], '--vocab', install['vocabulary'], '--data-dir', str(sequence),
                       '--config', str(config), '--frame-skip', '1', '--no-sleep', '--auto-term',
                       '--viewer', 'none', '--eval-log-dir', str(raw)]
        else:
            command = [install['binary'], install['vocabulary'], str(config), str(sequence)]
        # Test-only Python launchers are executable directly on POSIX, but
        # Windows needs its interpreter explicitly. Native backend binaries
        # retain their original command unchanged.
        if os.name == 'nt' and Path(install['binary']).suffix.lower() == '.py':
            command.insert(0, sys.executable)
        return command, raw

    def collect_outputs(self, raw):
        raw = Path(raw)
        filename = 'frame_trajectory.txt' if self.backend == 'stella_vslam' else 'KeyFrameTrajectory.txt'
        path = raw/filename
        require(path.is_file() and path.stat().st_size > 0, 'BACKEND_OUTPUT_MISSING', filename)
        times = raw/'track_times.txt'
        return dict(final_trajectory=path, online_trajectory=None,
                    trajectory_scope='frames' if self.backend == 'stella_vslam' else 'keyframes_only',
                    pose_direction='T_wc', quaternion_order='xyzw',
                    tracking_states=None, map_events=None,
                    timestamp_significant_digits=15 if self.backend == 'stella_vslam' else None,
                    processing_times=times if self.backend == 'stella_vslam' and times.is_file() else None)
