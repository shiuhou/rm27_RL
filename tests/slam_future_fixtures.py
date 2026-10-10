"""TEST_FIXTURE_ONLY. Artificial images/calibration/output; never camera evidence."""
import csv
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

from rm27.perception.vision.localization.experiments.common import file_hash, write_json
from rm27.perception.vision.localization.experiments.backends import PINS

LABEL='TEST_FIXTURE_ONLY'


def make_dataset(root, count=18, step=1):
    root=Path(root);root.mkdir(parents=True)
    images=root/'images';images.mkdir()
    video=root/'source.mp4';video.write_bytes(b'TEST_FIXTURE_ONLY not an actual video stream')
    calibration=dict(schema_version=1,purpose=LABEL,status='VALIDATED',calibration_id='TEST_FIXTURE_ONLY-cal',
        camera_module='TEST_FIXTURE_ONLY-camera',lens_id='TEST_FIXTURE_ONLY-lens',focus_setting='TEST_FIXTURE_ONLY-focus',
        camera_mode_id='TEST_FIXTURE_ONLY-mode', image_width=64,image_height=48,
        pixel_geometry={'crop':'NONE','resize':'NONE','pixel_format':'BGR8'},model='pinhole',distortion_model='opencv_radtan5',
        fx=57.,fy=55.,cx=31.5,cy=23.5,distortion_coefficients=[.01,-.001,.0001,.0002,.00001])
    report=dict(purpose=LABEL,passed=True,calibration_id=calibration['calibration_id'],criteria_id='TEST_FIXTURE_ONLY-criteria')
    write_json(root/'validation.json',report)
    calibration['validation']=dict(status='PASSED',criteria_id=report['criteria_id'],report_file='validation.json',
                                   report_sha256=file_hash(root/'validation.json'))
    write_json(root/'calibration.json',calibration)
    rows=[]
    for i, source_id in enumerate(range(0,count,step)):
        image=np.random.default_rng(source_id).integers(0,255,(48,64,3),dtype=np.uint8)
        path=images/f'{source_id}.png';assert cv2.imwrite(str(path),image)
        rows.append(dict(sample_index=i,source_frame_index=source_id,encoded_pts_s=format(source_id/180,'.17g'),
                    image='images/'+path.name,width=64,height=48,calibration_id=calibration['calibration_id'],
                    timestamp_unit='seconds',timestamp_clock_domain='UNKNOWN',timestamp_semantic='ENCODED_STREAM_PTS',
                    image_sha256=file_hash(path)))
    with (root/'frames.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    ids=[row['source_frame_index'] for row in rows]
    write_json(root/'subsets.json',{'all_decoded_sampled':dict(frame_count=len(ids),source_frame_indices=ids),
                                 'severe_blur':dict(frame_count=len(ids)//2,source_frame_indices=ids[:len(ids)//2])})
    write_json(root/'intervals.json',[dict(label='severe_blur',start_time_s=0,end_time_s=.03,reason=LABEL)])
    manifest=dict(schema_version=1,purpose=LABEL,dataset_id='TEST_FIXTURE_ONLY-dataset',status='READY_FOR_OFFLINE_RESEARCH',
        source=dict(artifact_directory='.',source_video='source.mp4',source_sha256=file_hash(video),producer_git_commit=LABEL),
        observed_geometry=dict(width=64,height=48,nominal_fps=180,**{k:calibration[k] for k in
                               ('camera_module','lens_id','focus_setting','camera_mode_id','pixel_geometry')}),
        calibration=dict(calibration_id=calibration['calibration_id'],status='VALIDATED',file='calibration.json',sha256=file_hash(root/'calibration.json')),
        frames_file='frames.csv',intervals_file='intervals.json',frame_count=len(rows))
    write_json(root/'dataset_manifest.json',manifest)
    return root/'dataset_manifest.json'


# The fixture stands in for a real backend binary, so it must be one host
# executable the runner can launch with no shell: a shebang script on POSIX, and
# a cmd/Python polyglot on Windows, which cannot exec a shebang script. Both stay
# a single file so binary_sha256 and the inline hooks in test_slam_online cover
# the exact artifact that is launched.
_FAKE_BACKEND_HEADER_LINES = 4
_FAKE_BACKEND_BODY = '''# TEST_FIXTURE_ONLY: emits predetermined poses, not SLAM.
import sys
from pathlib import Path
args=sys.argv[1:]
is_stella='--data-dir' in args
sequence=Path(args[args.index('--data-dir')+1]) if is_stella else Path(args[2])
raw=Path(args[args.index('--eval-log-dir')+1]) if is_stella else Path.cwd()
rows=[line.split() for line in (sequence/'rgb.txt').read_text().splitlines() if not line.startswith('#')]
poses=[f'{row[0]} {i*.1} {(i*.1)**2} 0 0 0 0 1' for i,row in enumerate(rows)]
(raw/('frame_trajectory.txt' if is_stella else 'KeyFrameTrajectory.txt')).write_text('\\n'.join(poses)+'\\n')
if is_stella: (raw/'track_times.txt').write_text('0.001\\n'*len(rows))
'''


def fake_binary(root):
    """Host-executable fake backend; arguments reach the body exactly as a binary's would."""
    if os.name != 'nt':
        binary = root/'fake_backend'
        binary.write_text('#!'+sys.executable+'\n'+_FAKE_BACKEND_BODY)
    else:
        # cmd runs the header, then exits before the Python body, which the
        # interpreter reads back out of this same file. Paths travel by
        # environment because python -c consumes argv[0], so argv[1:] must stay
        # the real arguments; %* forwards them unchanged.
        bootstrap = ("import os,pathlib;"
                     "src=pathlib.Path(os.environ['FAKE_BACKEND_SRC']).read_text();"
                     f"exec(compile(src.split(chr(10),{_FAKE_BACKEND_HEADER_LINES})"
                     f"[{_FAKE_BACKEND_HEADER_LINES}],'fake_backend','exec'))")
        header = ['@echo off',
                  'set "FAKE_BACKEND_SRC=%~f0"',
                  f'"{sys.executable}" -c "{bootstrap}" %*',
                  'exit /b %errorlevel%']
        assert len(header) == _FAKE_BACKEND_HEADER_LINES
        binary = root/'fake_backend.cmd'
        binary.write_text('\n'.join(header)+'\n'+_FAKE_BACKEND_BODY)
    binary.chmod(0o755)
    return binary


def fake_install(root, backend):
    root=Path(root);root.mkdir(parents=True)
    binary=fake_binary(root)
    vocab=root/'vocabulary';vocab.write_text(LABEL)
    manifest=dict(purpose=LABEL,backend=backend,binary=str(binary),binary_sha256=file_hash(binary),
                  vocabulary=str(vocab),vocabulary_sha256=file_hash(vocab),commit=PINS[backend])
    if backend=='stella_vslam':manifest['examples_commit']=PINS['stella_vslam_examples']
    write_json(root/'install.json',manifest)
    return root/'install.json'


def run_config(root,backend='stella_vslam'):
    dataset=make_dataset(root/'dataset')
    install=fake_install(root/'installation',backend)
    return dict(schema_version=1,purpose=LABEL,run_id='TEST_FIXTURE_ONLY-run',backend=backend,dataset=str(dataset),
                installation=str(install),output=str(root/'run'),subset='all_decoded_sampled',input_hz=60,
                transforms={},timeout_s=5)
