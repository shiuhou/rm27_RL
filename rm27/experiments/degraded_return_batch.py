"""Frozen permanent image-loss termination trials; full task success stays false."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from ..autopilot.px4.return_fault_batch import frozen_inputs,ROOT,digest


def trial(job):
    command,destination,hashes=job
    changed=[p for p,h in hashes.items() if digest(p)!=h]
    if changed:return dict(passed=False,failure='frozen inputs changed: '+str(changed))
    with destination.with_suffix('.log').open('x') as stream:
        process=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    path=destination/'termination_summary.json'
    result=json.loads(path.read_text()) if path.exists() else dict(passed=False,failure='missing termination summary')
    return dict(passed=process.returncode==0 and result.get('passed') is True and
        result.get('task_success') is False and result.get('original_mission_passed') is False,
        returncode=process.returncode,command=command,path=str(destination),termination=result)


def run(scene,configuration,output,*,workers=2,instance=46):
    if not 1<=workers<=2 or not 0<=instance<=99-workers:raise ValueError('one or two distinct local instances required')
    scene,configuration,output=map(lambda p:Path(p).resolve(),(scene,configuration,output))
    spec=json.loads(configuration.read_text());output.mkdir(parents=True,exist_ok=False)
    base=ROOT/spec['policy_configuration'];hashes=frozen_inputs(scene,configuration)
    for p in (base,Path(__file__),Path(__file__).with_name('degraded_return.py')):hashes[str(p)]=digest(p)
    camera=ROOT/spec['camera_configuration'] if spec.get('camera_configuration') else None
    if camera:hashes[str(camera)]=digest(camera)
    (output/'manifest.json').write_text(json.dumps(dict(configuration=spec,input_sha256=hashes,
        scene=str(scene),git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        run_type='headless closed-loop SITL independent launches; continued physical termination'),indent=2)+'\n')
    (output/'policy_runner.py').write_bytes(Path(__file__).with_name('degraded_return.py').read_bytes())
    (output/'batch_runner.py').write_bytes(Path(__file__).read_bytes())
    attempts=[]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            cases=spec['cases'][offset:offset+workers];jobs=[]
            for i,case in enumerate(cases):
                settings=json.loads(base.read_text());settings['fault_delay_s']=case['fault_delay_s']
                if camera:
                    camera_settings=json.loads(camera.read_text());camera_settings['seed']=case['camera_seed']
                    camera_path=output/(case['name']+'-camera.json')
                    camera_path.write_text(json.dumps(camera_settings,indent=2)+'\n')
                    settings['camera_configuration']=str(camera_path)
                path=output/(case['name']+'-configuration.json');path.write_text(json.dumps(settings,indent=2)+'\n')
                destination=output/case['name']
                command=[sys.executable,'-m','rm27.experiments.degraded_return',str(scene),
                    '--config',str(path),'--output',str(destination),'--instance',str(instance+i),'--speedup',str(spec['speedup'])]
                case_hashes=dict(hashes,**{str(path):digest(path)})
                if camera:case_hashes[str(camera_path)]=digest(camera_path)
                jobs.append((command,destination,case_hashes))
            for case,result in zip(cases,pool.map(trial,jobs)):
                attempts.append(dict(case=case,**result))
                errors=[a.get('termination',{}).get('horizontal_rest_error_m') for a in attempts]
                errors=[e for e in errors if e is not None]
                summary=dict(passed=len(attempts)==len(spec['cases']) and all(a['passed'] for a in attempts),
                    passed_count=sum(a['passed'] for a in attempts),total_attempts=len(attempts),required_count=len(spec['cases']),
                    gate_version=spec['gate_version'],task_success_count=0,
                    horizontal_rest_error_m=dict(maximum=max(errors),p95=float(np.percentile(errors,95))) if errors else None,
                    attempts=attempts,scope=spec['scope'],autopilot='px4',model_mass_kg=.15,
                    run_type='headless closed-loop SITL independent launches; fault termination')
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=result['passed'],termination=result.get('termination'))),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=2);p.add_argument('--instance',type=int,default=46)
    a=p.parse_args();return 0 if run(a.scene,a.config,a.output,workers=a.workers,instance=a.instance)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
