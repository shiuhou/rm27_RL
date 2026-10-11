"""Contact force convergence on the current aircraft, scene and recorded input.

This is independent physics scoring. It reuses one pre-impact state and the
same 250 Hz recorded motor commands at finer physical steps. It does not run
new PX4 flights, classify untested collision cases, or certify the whole S1.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from rm27.experiments.contact_metrics import armor_contacts, point_approach_speed


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def measure(model,data,armor,normal,vehicle):
    records=armor_contacts(model,data,armor,normal)
    for row in records:
        point=np.asarray(row['position_world_m']);jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jp,jr,point,vehicle)
        velocity=jp@data.qvel
        row.update(point_velocity_world_m_s=velocity.tolist(),
            normal_approach_speed_m_s=point_approach_speed(velocity,np.zeros(3),np.zeros(3),np.zeros(3),normal),
            classification='front' if row['signed_normal_force_N']>0 else 'back')
    return records


def run(recording,output):
    recording,output=Path(recording).resolve(),Path(output).resolve()
    metadata=json.loads((recording/'metadata.json').read_text());summary=json.loads((recording/'summary.json').read_text())
    scene=Path(metadata['scene'])
    if digest(scene)!=metadata['scene_sha256']:raise ValueError('Original scene changed')
    impact=summary['route']['impact'];impact_time=impact['time_s']
    start=None;rows=[]
    with (recording/'frames.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);t=row['timestamp']
            if t<=impact_time-.08:start=row
            elif t<=impact_time+.5:rows.append(row)
            else:break
    if start is None or not rows:raise ValueError('Missing recorded contact interval')
    output.mkdir(parents=True,exist_ok=False)
    results=[]
    for dt in [.002,.001,.0005,.00025,.000125]:
        model=mujoco.MjModel.from_xml_path(str(scene));model.opt.timestep=dt
        data=mujoco.MjData(model);data.time=start['timestamp']
        data.qpos[:]=start['all_qpos'];data.qvel[:]=start['all_qvel'];data.act[:]=start['all_act']
        body=model.body('drone_1').id;armor=model.geom('rm27_base_armor').id
        site=model.site('rm27_base_armor_center').id
        motors=np.array([model.actuator(f'drone_1_motor_{i}').id for i in range(4)])
        mujoco.mj_forward(model,data);normal=data.site_xmat[site].reshape(3,3)[:,2]
        times=np.array([row['timestamp'] for row in rows]);controls=np.array([row['controls'] for row in rows])
        impulse=peak=duration=0.;classes=set();force_vector=np.zeros(3);n=0
        with (output/f'samples-{round(dt*1e6):04d}us.jsonl').open('w') as stream:
            while data.time<times[-1]-1e-9:
                # Each recorded command drove the interval ending at its row.
                index=min(int(np.searchsorted(times,data.time+dt/2)),len(times)-1)
                data.ctrl[:]=0;data.ctrl[motors]=controls[index]*model.actuator_ctrlrange[motors,1]
                mujoco.mj_forward(model,data)
                contacts=measure(model,data,armor,normal,body)
                force=sum(r['compressive_normal_force_N'] for r in contacts)
                impulse+=force*dt;peak=max(peak,force)
                if contacts:
                    duration+=dt;classes.update(r['classification'] for r in contacts)
                    force_vector+=sum((np.asarray(r['force_on_armor_world_N']) for r in contacts),np.zeros(3))*dt
                    stream.write(json.dumps(dict(timestamp_s=float(data.time),contacts=contacts),allow_nan=False)+'\n')
                mujoco.mj_step(model,data);n+=1
        result=dict(physics_timestep_s=dt,control_period_s=.004,normal_impulse_Ns=impulse,
            peak_normal_force_N=peak,contact_duration_s=duration,classification=sorted(classes),
            impulse_on_armor_world_Ns=force_vector.tolist(),samples=n)
        results.append(result)
    checks=[]
    for a,b in zip(results,results[1:]):
        impulse_difference=abs(a['normal_impulse_Ns']/b['normal_impulse_Ns']-1) if b['normal_impulse_Ns'] else None
        peak_difference=abs(a['peak_normal_force_N']/b['peak_normal_force_N']-1) if b['peak_normal_force_N'] else None
        passed=(impulse_difference is not None and peak_difference is not None and impulse_difference<=.05
                and peak_difference<=.1 and a['classification']==b['classification']==['front'])
        checks.append(dict(coarse_dt_s=a['physics_timestep_s'],fine_dt_s=b['physics_timestep_s'],
            impulse_relative_difference=impulse_difference,peak_relative_difference=peak_difference,passed=passed))
    native_step=float(metadata.get('physics_timestep_s',.002))
    native=next((r for r in checks if np.isclose(r['coarse_dt_s'],native_step,atol=1e-10,rtol=0)),None)
    result=dict(passed=bool(native and native['passed'] and checks[-1]['passed']),run_type='recorded-input current-aircraft contact convergence',
        s1_complete=False,source_run=str(recording),scene_sha256=digest(scene),recording_sha256=digest(recording/'frames.jsonl'),
        runner_sha256=digest(__file__),results=results,comparisons=checks,
        thresholds=dict(impulse_relative_difference=.05,peak_relative_difference=.1,classification='front'),
        native_timestep_s=native_step,native_step_converged=bool(native and native['passed']),
        limitations='One low-speed front impact only. Grazing, back, non-target contacts and surrounding forbidden geometry need separate qualification.')
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=run(a.recording,a.output);return 0 if r['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
