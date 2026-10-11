"""Independent current-flight VIO scoring with one initial rigid registration.

Truth is available only to this scorer. There is no scale fit or trajectory-wide
alignment. A successful recorded-input score does not qualify live navigation.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .dataset import digest


def align_first_pose(estimate_position,estimate_rotation,truth_position,truth_rotation):
    rotation=truth_rotation@estimate_rotation.T
    translation=truth_position-rotation@estimate_position
    return rotation,translation


def evaluate(vio,dataset,output):
    vio,dataset,output=Path(vio).resolve(),Path(dataset).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    sensor=json.loads((dataset/'summary.json').read_text())
    run=Path(sensor['original_run'])
    if digest(run/'frames.jsonl')!=sensor['original_recording_sha256']:raise ValueError('flight recording changed')
    poses=[json.loads(line) for line in (vio/'pose.jsonl').open()]
    truth=[]
    with (run/'frames.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);t=row['timestamp']
            if t>sensor['end_time_s']+.01:break
            tr=row['truth'];truth.append([t,*tr['position_nwu'],*tr['quaternion_flu_to_nwu']])
    truth=np.asarray(truth,dtype=float)
    valid=[p for p in poses if truth[0,0]<=p['source_time_ns']/1e9<=truth[-1,0]]
    if not valid:
        result=dict(passed=False,run_type='recorded-input VIO scoring',failure='no associated current-flight poses')
    else:
        stamps=np.array([p['source_time_ns']/1e9 for p in valid])
        estimated=np.array([p['position'] for p in valid])
        observed=np.column_stack([np.interp(stamps,truth[:,0],truth[:,i]) for i in [1,2,3]])
        first_index=int(np.argmin(abs(truth[:,0]-stamps[0])))
        q=truth[first_index,4:8]
        r_truth=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        r_est=Rotation.from_quat(valid[0]['quaternion_xyzw']).as_matrix()
        rotation,translation=align_first_pose(estimated[0],r_est,observed[0],r_truth)
        registered=estimated@rotation.T+translation
        errors=registered-observed
        horizontal=np.linalg.norm(errors[:,:2],axis=1);vertical=np.abs(errors[:,2])
        duration=stamps[-1]-stamps[0]
        coverage=duration/max(1e-9,sensor['end_time_s']-sensor['start_time_s'])
        finite=bool(np.isfinite(errors).all() and np.isfinite(stamps).all())
        h95,z95=float(np.percentile(horizontal,95)),float(np.percentile(vertical,95))
        result=dict(passed=finite and coverage>=.8 and h95<=.2 and z95<=.1,
                    run_type='recorded-input VIO scoring',flight_qualified=False,
                    alignment=dict(type='one_initial_body_pose_SE3',scale=1.,rotation=rotation.tolist(),translation=translation.tolist()),
                    pose_count=len(valid),start_time_s=float(stamps[0]),end_time_s=float(stamps[-1]),coverage_fraction=float(coverage),
                    horizontal=dict(p95_m=h95,maximum_m=float(horizontal.max()),rmse_m=float(np.sqrt(np.mean(horizontal**2)))),
                    vertical=dict(p95_m=z95,maximum_m=float(vertical.max()),rmse_m=float(np.sqrt(np.mean(vertical**2)))),
                    thresholds=dict(horizontal_p95_m=.2,vertical_p95_m=.1,minimum_coverage_fraction=.8),
                    localization_contract_complete=False,
                    source_hashes=dict(sensor_manifest=digest(dataset/'summary.json'),poses=digest(vio/'pose.jsonl')))
        np.savez_compressed(output/'trajectory.npz',timestamp=stamps,estimate=registered,reference=observed,error=errors)
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('vio',type=Path);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=evaluate(a.vio,a.dataset,a.output);return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
