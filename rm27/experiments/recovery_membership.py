"""Independent native-step recovery membership in a declared pose corridor."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .recovery_corridor import contains_pose


def run(recording,configuration,output):
    recording,configuration,output=map(lambda p:Path(p).resolve(),(recording,configuration,output))
    settings=json.loads(configuration.read_text());metadata=json.loads((recording/'metadata.json').read_text())
    scene=Path(metadata['scene'])
    if hashlib.sha256(scene.read_bytes()).hexdigest()!=metadata['scene_sha256']:raise ValueError('scene changed')
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    site=model.site(settings['armor_site']).id;centre=data.site_xpos[site].copy()
    plane=data.site_xmat[site].reshape(3,3).copy();point=np.asarray(settings['contact_point_body_flu_m'])
    rows=[];outside=[]
    source=recording/'physical_audit.jsonl'
    for line in source.open():
        row=json.loads(line);truth=row['truth']
        rotation=Rotation.from_quat(np.asarray(truth['quaternion_flu_to_nwu'])[[1,2,3,0]])
        tip=np.asarray(truth['position_nwu'])+rotation.apply(point)
        local=plane.T@(tip-centre);pose=np.r_[local[[2,0,1]],rotation.as_euler('xyz',degrees=True)]
        within=contains_pose(settings['pose_regions'],pose)
        event=dict(timestamp_s=row['timestamp'],pose=pose.tolist(),within=bool(within));rows.append(event)
        if not within:outside.append(event)
    times=np.array([r['timestamp_s'] for r in rows]);poses=np.array([r['pose'] for r in rows])
    step=float(metadata['physics_timestep_s'])
    continuous=bool(len(times)>1 and np.all(np.diff(times)>0) and np.diff(times).max()<=step+1e-8)
    output.mkdir(parents=True,exist_ok=False)
    result=dict(passed=bool(rows and not outside and continuous and times[-1]-times[0]>=3.),
        gate_version='PX4_NATIVE_RECOVERY_POSE_MEMBERSHIP_V1',run_type='independent recorded native-step recovery audit',
        source_run=str(recording),samples=len(rows),outside=outside,all_native_steps_present=continuous,
        duration_s=float(times[-1]-times[0]) if len(times) else 0,physics_step_s=step,
        pose_minimum=poses.min(axis=0).tolist() if len(poses) else [],
        pose_maximum=poses.max(axis=0).tolist() if len(poses) else [],
        configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
        recording_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        scene_sha256=metadata['scene_sha256'],
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations='Recorded 1 ms recovery states, not flight control input. The pose corridor requires its own passing continuous geometry certificate. Subsequent route poses remain separately audited.')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='outside'},indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();return 0 if run(a.recording,a.config,a.output)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
