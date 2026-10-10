"""Clean pinhole-camera frames and synthetic camera truth for vision experiments."""

import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from .audit_cad import sha256
from .experiment import load_run, pose


def intrinsics(width, height, fovy):
    """Square pixels, zero-based pixel-center coordinates, vertical FOV in degrees."""
    f=height/(2*np.tan(np.deg2rad(fovy)/2))
    return np.array([[f,0,(width-1)/2],[0,f,(height-1)/2],[0,0,1.]])


def camera_truth(model, data):
    cid=model.camera('darts_fpv').id
    # OpenCV convention: right/down/forward; native MuJoCo: right/up/backward.
    rotation=data.cam_xmat[cid].reshape(3,3)@np.diag([1.,-1.,-1.])
    return dict(position_world_m=data.cam_xpos[cid].tolist(),
                R_world_from_camera=rotation.tolist())


def capture(run, output, fps=60., width=1280, height=720, fovy=60.):
    if not np.isfinite(fps) or not 0 < fps <= 1000:
        raise ValueError('Capture FPS must be finite and in (0,1000]')
    if any(not isinstance(n,int) or not 16 <= n <= 4096 for n in (width,height)):
        raise ValueError('Image dimensions must be integers in [16,4096]')
    run=Path(run).resolve()
    model,data,path,report=load_run(run,fovy)
    if not report['passed']:
        raise ValueError('Source run failed geometric validation; cannot export validation data')
    times=np.arange(int(np.ceil(path.duration*fps)),dtype=float)/fps
    times=times[times < path.duration]
    if len(times)>20000:
        raise ValueError('Capture exceeds 20000 frames')
    output=Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    (output/'frames').mkdir()
    provenance={name:sha256(run/name) for name in ('summary.json','scene.xml','dart_visual.obj')}
    (output/'asset_relocations.json').write_text(json.dumps(report['playback_asset_relocations'],indent=2)+'\n')
    model.vis.global_.offwidth=max(width,model.vis.global_.offwidth)
    model.vis.global_.offheight=max(height,model.vis.global_.offheight)
    option=mujoco.MjvOption()
    option.geomgroup[3]=0;option.geomgroup[4]=0;option.sitegroup[:]=0
    K=intrinsics(width,height,fovy)
    with mujoco.Renderer(model,height=height,width=width) as renderer, (output/'frames.jsonl').open('w') as log:
        for i,t in enumerate(times):
            pose(model,data,path,t)
            renderer.update_scene(data,camera='darts_fpv',scene_option=option)
            filename=f'frames/{i:06d}.png'
            Image.fromarray(renderer.render()).save(output/filename)
            record=dict(frame=i,file=filename,time_s=float(t),**camera_truth(model,data))
            log.write(json.dumps(record)+'\n')
    metadata=dict(
        source_run=str(run),source_sha256=provenance,
        source_code_sha256={name:sha256(Path(__file__).with_name(name))
                            for name in ('capture.py','experiment.py','trajectory.py')},
        mode='headless MuJoCo rendering of analytic trajectory; no SITL',
        width=width,height=height,fps=fps,frame_count=len(times),
        vertical_fov_degrees=fovy,K=K.tolist(),distortion_coefficients=[0.,0.,0.,0.,0.],
        camera_model='Ideal pinhole; assumed intrinsics, not real-camera calibration',
        camera_mount='Dart front-face center; optical axis +X, image up +Z in dart frame',
        pose_convention='p_world = R_world_from_camera @ p_camera + position_world_m; camera axes right/down/forward',
        pixel_convention='Top-left pixel center is (0,0); u right, v down',
        sampling='t=i/fps < impact time; no held launch/impact frames, no slow-motion duplication',
        flight_time_s=path.duration,near_clip_m=.001,
        limitations=[
            'Synthetic RGB and exact pose truth; no detection algorithm has been evaluated.',
            'Attitude follows trajectory velocity with world-up roll; no measured dart rotation or vibration.',
            'No lens distortion, exposure, motion blur, rolling shutter or sensor noise.',
            'Lamp and target are demonstration geometry/materials, not calibrated real optics.',
            'Sites and launch/path annotations hidden; target may leave view or become clipped near impact.',
        ])
    (output/'camera.json').write_text(json.dumps(metadata,indent=2)+'\n')
    files=list((output/'frames').glob('*.png'))
    summary=dict(passed=len(files)==len(times) and len(times)>0,
                 source_geometric_passed=report['passed'],frame_count=len(files),
                 mode=metadata['mode'],algorithm_accuracy_validated=False)
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return output
