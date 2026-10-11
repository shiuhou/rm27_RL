"""Contact-step convergence using the actual recorded physical uncertainty."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from rm27.autopilot.px4 import contact_convergence
from .physical_fixture import configure_model


def loaded_classification(contacts):
    classes=set();unloaded=0
    for row in contacts:
        force=np.asarray(row['force_on_armor_world_N'],float)
        signed=row['signed_normal_force_N']
        if force.shape!=(3,) or not np.isfinite(np.r_[force,signed]).all():
            raise ValueError('Finite three-dimensional contact force required')
        if not np.any(force!=0.):
            unloaded+=1
        else:
            classes.add('front' if signed>0 else 'back' if signed<0 else 'grazing')
    return classes,unloaded


def run(recording,output):
    metadata=json.loads((Path(recording)/'metadata.json').read_text())
    settings=metadata.get('physical_fixture',{}).get('settings')
    if settings is None:raise ValueError('Recorded physical fixture required')
    original=contact_convergence.mujoco
    def load(path):
        model=original.MjModel.from_xml_path(path)
        motors=[model.actuator(f'drone_1_motor_{i}').id for i in range(4)]
        configure_model(model,motors,settings)
        return model
    namespace=SimpleNamespace(**vars(original))
    namespace.MjModel=SimpleNamespace(from_xml_path=load)
    contact_convergence.mujoco=namespace
    try:
        result=contact_convergence.run(recording,output)
        (Path(output)/'geometric_candidate_summary.json').write_text(json.dumps(result,indent=2)+'\n')
        for trial in result['results']:
            classes=set();unloaded=0
            samples=Path(output)/f'samples-{round(trial["physics_timestep_s"]*1e6):04d}us.jsonl'
            for line in samples.open():
                row=json.loads(line)
                active,zero=loaded_classification(row['contacts']);classes.update(active);unloaded+=zero
            trial['geometric_candidate_classification']=trial['classification']
            trial['classification']=sorted(classes)
            trial['exact_zero_force_candidates']=unloaded
        for comparison,a,b in zip(result['comparisons'],result['results'],result['results'][1:]):
            impulse= comparison['impulse_relative_difference'];peak=comparison['peak_relative_difference']
            comparison['passed']=bool(impulse is not None and peak is not None and impulse<=.05 and peak<=.1 and
                a['classification']==b['classification']==['front'])
        native=next(c for c in result['comparisons'] if c['coarse_dt_s']==result['native_timestep_s'])
        result['passed']=bool(native['passed'] and result['comparisons'][-1]['passed'])
        result['native_step_converged']=native['passed']
        result['classification_policy']='Loaded force direction; exact zero-force geometric candidates retained separately. Nonzero negative/grazing loads are never ignored.'
        result['gate_version']='PX4_FIXTURE_LOADED_CONTACT_CONVERGENCE_V2'
        result.update(physical_fixture=settings,fixture_source_sha256=hashlib.sha256(
            Path(__file__).with_name('physical_fixture.py').read_bytes()).hexdigest())
        (Path(output)/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        return result
    finally:contact_convergence.mujoco=original


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();raise SystemExit(0 if run(args.recording,args.output)['passed'] else 1)
