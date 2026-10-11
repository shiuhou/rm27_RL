"""Reserved target-image faults during approach, including safe pre-contact return."""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from rm27.autopilot.px4.return_fault_batch import ROOT,digest,frozen_inputs
from .archive_closed_auxiliary import run as archive_auxiliary
from .camera_storage import archive as archive_camera
from .score_approach_fault import run as score


def trial(job):
    command,destination,hashes,spec=job;destination=Path(destination)
    changed=[p for p,h in hashes.items() if not Path(p).is_file() or digest(p)!=h]
    if changed:return dict(passed=False,failure='Frozen inputs changed: '+str(changed))
    with destination.with_suffix('.log').open('x') as stream:
        process=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    qualified=dict(passed=False)
    try:
        qualified=score(destination,ROOT/spec['surface_configuration'],
            ROOT/spec['recovery_configuration'],destination/'independent_approach_fault_score')
    except Exception as error:
        qualified['failure']=f'{type(error).__name__}: {error}'
    result=dict(passed=process.returncode==0 and qualified['passed'],returncode=process.returncode,
        task_success=qualified.get('task_success',False),safe_termination=qualified.get('safe_termination',False),
        qualification=qualified,command=command,path=str(destination))
    if (destination/'summary.json').is_file() and (destination/'metadata.json').is_file():
        result['summary']=json.loads((destination/'summary.json').read_text())
        try:
            for name in ['vio','return_camera']:
                camera=destination/name
                if any((camera/'images').glob('*.png')):archive_camera(camera)
            auxiliary=archive_auxiliary(destination,destination/'auxiliary_archive')
            result['auxiliary_storage']=dict(passed=auxiliary['passed'],saved_bytes=auxiliary['saved_bytes'])
        except Exception as error:result['storage_failure']=f'{type(error).__name__}: {error}'
    return result


def report(attempts,spec):
    tasks=sum(a.get('task_success',False) and a['passed'] for a in attempts)
    ends=sum(a.get('safe_termination',False) and a['passed'] for a in attempts)
    errors=[a['qualification'].get('resting_horizontal_error_m') for a in attempts if a.get('qualification')]
    errors=[e for e in errors if e is not None]
    groups=Counter(a['category'] for a in attempts)
    passes=Counter(a['category'] for a in attempts if a['passed'])
    return dict(passed=len(attempts)==len(spec['cases']) and tasks==spec['required_tasks'] and
        ends==spec['required_terminations'] and all(a['passed'] for a in attempts),
        total_attempts=len(attempts),required_count=len(spec['cases']),task_successes=tasks,safe_terminations=ends,
        unhandled_failures=sum(not a['passed'] for a in attempts),autopilot='px4',model_mass_kg=.15,
        run_type='headless closed-loop SITL independent launches; actual target pixels altered during VISUAL_APPROACH',
        gate_version=spec['gate_version'],scope=spec['scope'],attempts=attempts,
        per_fault={k:dict(handled=passes[k],total=v) for k,v in groups.items()},
        resting_horizontal_error_m=dict(maximum=max(errors),p95=float(np.percentile(errors,95))) if errors else None)


def run(scene,audit,configuration,output,*,workers=2,instance=72):
    if not 1<=workers<=3 or not 0<=instance<=99-workers:raise ValueError('Distinct valid SITL instances required')
    scene,audit,configuration,output=map(lambda p:Path(p).resolve(),(scene,audit,configuration,output))
    spec=json.loads(configuration.read_text())
    mission=ROOT/spec['mission_configuration'];camera=ROOT/spec['camera_configuration']
    recovery=ROOT/spec['recovery_configuration'];certificate=ROOT/spec['continuous_certificate']
    proof=json.loads(certificate.read_text())
    if proof.get('passed') is not True or proof.get('scene_sha256')!=digest(scene) or proof.get('configuration_sha256')!=digest(recovery):
        raise ValueError('Matching passed continuous recovery certificate required')
    files=[audit,mission,camera,recovery,certificate,ROOT/spec['surface_configuration'],Path(__file__),
        *[Path(__file__).with_name(n+'.py') for n in ['approach_target_fault','score_approach_fault',
            'ground_rest','causal_flight','initial_vio_admission','camera_storage','archive_closed_auxiliary']],
        *[ROOT/'rm27/experiments'/f'{n}.py' for n in ['image_conditions','sensor_clock','fast_bearing',
            'vio_initial_admission','target_region_audit','recovery_corridor','recovery_membership']],
        ROOT/'rm27/telemetry/ulog.py',ROOT/'rm27/telemetry/recording.py']
    profile=json.loads(mission.read_text())
    files.extend([ROOT/'configs/px4'/f'{profile["estimation_profile"]}.json',ROOT/profile['return_termination_configuration']])
    video_report=ROOT/'artifacts/px4-lossless-video-runtime-01/summary.json';video=json.loads(video_report.read_text())
    if video.get('passed') is not True or digest(video['binary'])!=video['sha256']:raise ValueError('Verified video runtime required')
    files.extend([video_report,Path(video['binary'])])
    hashes=frozen_inputs(scene,configuration);hashes.update({str(p.resolve()):digest(p) for p in files})
    output.mkdir(parents=True,exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(dict(configuration=spec,input_sha256=hashes,
        command=sys.argv,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_snapshots={str(p):p.read_text() for p in files if p.suffix=='.py'}),indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            for index,case in enumerate(group):
                camera_path=output/(case['name']+'-camera.json')
                camera_path.write_text(json.dumps(dict(json.loads(camera.read_text()),seed=case['camera_seed']),indent=2)+'\n')
                conditions=output/(case['name']+'-conditions.json')
                conditions.write_text(json.dumps(dict(initial_vio_admission=True,efficient_bearing=True),indent=2)+'\n')
                destination=output/case['name']
                command=[sys.executable,'-m','tools.px4.approach_target_fault',str(scene),'--audit',str(audit),
                    '--config',str(mission),'--camera-config',str(camera_path),'--conditions',str(conditions),
                    '--kind',case['kind'],'--delay',str(case['delay_s']),'--duration',str(case['duration_s']),
                    '--instance',str(instance+index),'--output',str(destination)]
                jobs.append((command,str(destination),dict(hashes,**{str(p):digest(p) for p in [camera_path,conditions]}),spec))
            for case,result in zip(group,pool.map(trial,jobs)):
                attempts.append(dict(name=case['name'],category=case['category'],camera_seed=case['camera_seed'],**result))
                summary=report(attempts,spec)
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=result['passed'],task_success=result.get('task_success'),
                    safe_termination=result.get('safe_termination'),failure=result.get('failure') or result.get('qualification',{}).get('failure'))),flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('scene',type=Path)
    for flag in ['audit','config','output']:parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--workers',type=int,default=2);parser.add_argument('--instance',type=int,default=72)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.scene,args.audit,args.config,args.output,workers=args.workers,instance=args.instance)['passed'] else 1)
