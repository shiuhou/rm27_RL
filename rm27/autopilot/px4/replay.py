"""Replay recorded physical motor fractions; no autopilot or navigation proof."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from ...simulation.micro_plant import MicroPlant


def replay(run, output):
    run=Path(run).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    meta=json.loads((run/'metadata.json').read_text());scene=Path(meta['scene'])
    if hashlib.sha256(scene.read_bytes()).hexdigest()!=meta['scene_sha256']:raise ValueError('scene changed')
    dynamics=run/'dynamics.json'
    plant=MicroPlant(scene,dynamics_profile=dynamics if dynamics.exists() else None)
    count=0; maximum=0.;failure=None
    try:
        with (run/'frames.jsonl').open() as handle:
            for line in handle:
                row=json.loads(line);target=row['timestamp']
                # Initial module startup may advance zero-input physics before the first row.
                while plant.data.time+plant.model.opt.timestep<target-1e-10:
                    plant.step_normalized([0.,0.,0.,0.] if count==0 else row['controls'])
                plant.step_normalized(row['controls'])
                if abs(plant.data.time-target)>1e-9:raise ValueError('recorded physics time is not on native step grid')
                for field,actual in [('all_qpos',plant.data.qpos),('all_qvel',plant.data.qvel),('all_act',plant.data.act),('all_motor_thrust_N',plant.data.actuator_force)]:
                    error=float(np.max(np.abs(np.asarray(row[field])-actual)));maximum=max(maximum,error)
                    if error!=0:raise ValueError(f'non-exact {field} at frame {count}: {error}')
                count+=1
    except Exception as exc:failure=str(exc)
    summary={'passed':failure is None and count>0,'failure':failure,'run_type':'recorded-input replay','frames':count,'maximum_absolute_error':maximum,'source_run':str(run)}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');return summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();r=replay(a.run,a.output);print(json.dumps(r,indent=2));return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
