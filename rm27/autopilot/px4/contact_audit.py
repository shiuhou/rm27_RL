"""Independent current-aircraft contact classification, outside flight control.

The compiled vehicle/armor/dock geometry is exercised with motors off in zero
gravity to isolate force direction. This is physics evidence, not a new flight,
referee damage model or structural qualification.
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from ...experiments.contact_metrics import armor_contacts
from ...perception.sitl.dataset import digest


def classify(row, armor_rotation, armor_center, armor_size):
    if not row.get('target'):
        return 'non_target'
    point=armor_rotation.T@(np.asarray(row['position_world_m'])-armor_center)
    force=np.asarray(row['force_on_armor_world_N'])
    normal=armor_rotation[:,2]
    projection=-float(normal@force)
    magnitude=float(np.linalg.norm(force))
    # A side/edge contact does not establish a front normal impact. Tolerance
    # is geometric (0.5 mm) and fixed before this probe matrix.
    at_edge=bool(np.any(abs(point[:2])>=armor_size[:2]-.0005))
    if magnitude<1e-8:
        return 'inactive'
    if abs(projection)<.5*magnitude or at_edge:
        return 'grazing'
    return 'front' if projection>0 else 'back'


def probe(scene,dt,case,output):
    model=mujoco.MjModel.from_xml_path(str(scene));model.opt.timestep=dt;model.opt.gravity[:]=0
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    body=model.body('drone_1').id;armor=model.geom('rm27_base_armor').id
    qa=model.jnt_qposadr[model.body_jntadr[body]];va=model.jnt_dofadr[model.body_jntadr[body]]
    center=data.geom_xpos[armor].copy();rotation=data.geom_xmat[armor].reshape(3,3).copy()
    normal=rotation[:,2];tangent=rotation[:,0];size=model.geom_size[armor].copy()
    if case=='front':position=center+normal*.15;velocity=-normal*.3
    elif case=='back':position=center-normal*.15;velocity=normal*.3
    elif case=='grazing':position=center+tangent*.16+normal*.025;velocity=-tangent*.3
    else:
        bottom=model.geom('rm27_dock_collision_bottom').id
        position=data.geom_xpos[bottom]+[0,0,model.geom_size[bottom,2]+.15]
        # Shift into the surveyed slot to avoid the rim/divider geometry.
        position[:2]=data.xpos[body,:2];velocity=np.array([0.,0.,-.3])
    data.qpos[qa:qa+3]=position;data.qvel[va:va+3]=velocity
    data.qvel[va+3:va+6]=0;data.act[:]=0;data.ctrl[:]=0
    initial_velocity=velocity.copy();mass=model.body_mass[body]
    classes=set();impulse=np.zeros(3);peak=duration=0.;contacts_count=0
    with (output/f'{case}-{round(dt*1e6)}us.jsonl').open('x') as stream:
        for _ in range(round(.7/dt)):
            mujoco.mj_forward(model,data)
            target={tuple(row['geoms']):row for row in armor_contacts(model,data,armor,normal)}
            active=False
            for i in range(data.ncon):
                c=data.contact[i];g=list(map(int,c.geom))
                if body not in model.geom_bodyid[g]:continue
                wrench=np.zeros(6);mujoco.mj_contactForce(model,data,i,wrench)
                force_second=c.frame.reshape(3,3).T@wrench[:3]
                on_vehicle=force_second if model.geom_bodyid[g[1]]==body else -force_second
                magnitude=np.linalg.norm(on_vehicle)
                if magnitude<1e-8:continue
                names=tuple(model.geom(k).name for k in g)
                row=target.get(names,dict(geoms=names,position_world_m=c.pos.tolist()))
                row=dict(row,target=armor in g)
                category=classify(row,rotation,center,size);classes.add(category)
                row.update(classification=category,force_on_vehicle_world_N=on_vehicle.tolist())
                stream.write(json.dumps(dict(timestamp_s=float(data.time),**row))+'\n')
                impulse+=on_vehicle*dt;peak=max(peak,float(magnitude));contacts_count+=1;active=True
            if active:duration+=dt
            mujoco.mj_step(model,data)
    momentum=mass*(data.qvel[va:va+3]-initial_velocity)
    error=np.linalg.norm(momentum-impulse)/max(np.linalg.norm(momentum),1e-12)
    result=dict(case=case,timestep_s=dt,classes=sorted(classes),contact_samples=contacts_count,
        impulse_on_vehicle_world_Ns=impulse.tolist(),momentum_change_world_Ns=momentum.tolist(),
        momentum_relative_error=float(error),peak_contact_force_N=peak,contact_duration_s=duration,
        passed=bool(case in classes and error<=.05))
    return result


def run(scene,output):
    scene=Path(scene).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    results=[probe(scene,dt,case,output) for case in ['front','back','grazing','non_target']
             for dt in [.001,.0005,.00025,.000125]]
    result=dict(passed=all(r['passed'] for r in results if r['timestep_s']==.000125),
        run_type='independent compiled-aircraft contact physics',s1_complete=False,
        scene_sha256=digest(scene),runner_sha256=digest(__file__),results=results,
        limitations='Motors off and zero gravity; local target/dock probes, not closed-loop flight or full surrounding forbidden-region audit')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=run(a.scene,a.output);return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
