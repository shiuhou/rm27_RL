"""Frozen independent complete missions across declared simulation conditions."""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from ..autopilot.px4.return_fault_batch import frozen_inputs,ROOT,digest
from ..autopilot.px4.image_batch import assess
from ..telemetry.ulog import analyze
from ..telemetry.recording import compress_recording
from .recovery_membership import run as membership
from .target_region_audit import audit_recording


def trial(job):
    command,destination,hashes,spec=job;destination=Path(destination)
    changed=[p for p,h in hashes.items() if not Path(p).exists() or digest(p)!=h]
    if changed:return dict(passed=False,task_success=False,failure='Frozen inputs changed: '+str(changed),command=command)
    with destination.with_suffix('.log').open('x') as stream:
        process=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    def read(name):
        path=destination/name
        return json.loads(path.read_text()) if path.exists() else dict(passed=False,failure=name+' missing')
    summary=read('summary.json');condition=read('condition_summary.json');vio=read('vio/summary.json')
    logs=list((destination/'log').rglob('*.ulg'))
    log=analyze(logs[0],destination/'ulog_analysis') if len(logs)==1 else dict(passed=False,failure='exactly one ULog required')
    try:
        surfaces=audit_recording(destination,ROOT/spec['surface_configuration'],destination/'local_surface_audit')
        poses=membership(destination,ROOT/spec['recovery_configuration'],destination/'native_recovery_membership')
    except Exception as error:
        surfaces=poses=dict(passed=False,failure=f'{type(error).__name__}: {error}')
    observer=read('observer_rate.json')
    qualification=assess(summary,vio,log,surfaces=surfaces,require_surfaces=True,
        observer=observer,required_observer_interval_us=4000)
    qualification['checks']['native_recovery_membership']=poses.get('passed') is True
    qualification['passed']=all(qualification['checks'].values())
    task=bool(condition.get('task_success') is True and condition.get('passed') is True and qualification['passed'])
    required_termination=('physical_hover','physical_recovery','image_target','native_vio',
        'ulog_integrity','actual_ekf2_ev_fusion','no_unexpected_failsafe','recorded_local_surfaces',
        'measured_imu_observer','native_recovery_membership')
    termination=bool(condition.get('task_success') is False and condition.get('passed') is True and
        condition.get('safe_termination',{}).get('passed') is True and
        all(qualification['checks'][k] for k in required_termination))
    archives=[]
    if (destination/'frames.jsonl').exists():archives.append(compress_recording(destination/'frames.jsonl'))
    return dict(passed=process.returncode==0 and (task or termination),task_success=task,
        safe_termination=termination,returncode=process.returncode,command=command,path=str(destination),
        summary=summary,condition_summary=condition,qualification=qualification,recording_archives=archives,
        native_recovery_membership_path=str(destination/'native_recovery_membership/summary.json'))


def report(attempts,spec):
    successes=sum(a.get('task_success',False) for a in attempts)
    terminations=sum(a.get('safe_termination',False) for a in attempts)
    complete=len(attempts)==len(spec['cases'])
    counts=Counter(a['category'] for a in attempts)
    task_counts=Counter(a['category'] for a in attempts if a.get('task_success'))
    good_counts=Counter(a['category'] for a in attempts if a['passed'])
    values=[a.get('summary',{}).get('independent_resting_score',{}).get('horizontal_error_m') for a in attempts]
    values=[v for v in values if v is not None]
    return dict(passed=bool(complete and successes>=spec['minimum_task_successes'] and all(a['passed'] for a in attempts)),
        total_attempts=len(attempts),required_count=len(spec['cases']),task_successes=successes,
        safe_terminations=terminations,unhandled_failures=sum(not a['passed'] for a in attempts),
        gate_version=spec['gate_version'],autopilot='px4',model_mass_kg=.15,
        run_type='headless closed-loop SITL independent launches',scope=spec['scope'],
        per_condition={k:dict(total=v,task_successes=task_counts[k],handled=good_counts[k]) for k,v in counts.items()},
        resting_horizontal_m=dict(maximum=max(values),p95=float(np.percentile(values,95))) if values else None,
        attempts=attempts)


def run(scene,audit,configuration,output,*,workers=2,instance=55):
    if type(workers) is not int or not 1<=workers<=3 or not 0<=instance<=99-workers:
        raise ValueError('1..3 separate valid SITL instances required')
    scene,audit,configuration,output=map(lambda p:Path(p).resolve(),(scene,audit,configuration,output))
    spec=json.loads(configuration.read_text());output.mkdir(parents=True,exist_ok=False)
    hashes=frozen_inputs(scene,configuration)
    files=[audit,ROOT/spec['mission_configuration'],ROOT/spec['camera_configuration'],
        ROOT/spec['surface_configuration'],ROOT/spec['recovery_configuration'],
        Path(__file__),Path(__file__).with_name('image_conditions.py'),Path(__file__).with_name('sensor_clock.py'),
        Path(__file__).with_name('recovery_membership.py'),Path(__file__).with_name('recovery_corridor.py'),
        Path(__file__).with_name('target_region_audit.py'),ROOT/'rm27/telemetry/recording.py']
    mission=json.loads((ROOT/spec['mission_configuration']).read_text())
    files.extend([ROOT/'configs/px4'/f'{mission["estimation_profile"]}.json',ROOT/mission['return_termination_configuration']])
    certificate=ROOT/spec['continuous_certificate'];proof=json.loads(certificate.read_text())
    if proof.get('passed') is not True or proof.get('scene_sha256')!=digest(scene):
        raise ValueError('matching passed continuous geometry certificate required')
    if proof.get('configuration_sha256')!=digest(ROOT/spec['recovery_configuration']):
        raise ValueError('certificate/recorded membership configuration mismatch')
    files.append(certificate)
    hashes.update({str(p):digest(p) for p in files})
    manifest=dict(configuration=spec,input_sha256=hashes,scene=str(scene),audit=str(audit),
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),command=sys.argv)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            for i,c in enumerate(group):
                dest=output/c['name'];camera=json.loads((ROOT/spec['camera_configuration']).read_text());camera['seed']=c['camera_seed']
                cam_path=output/(c['name']+'-camera.json');cam_path.write_text(json.dumps(camera,indent=2)+'\n')
                conditions_path=output/(c['name']+'-conditions.json');conditions_path.write_text(json.dumps(c['conditions'],indent=2)+'\n')
                command=[sys.executable,'-m','rm27.experiments.image_conditions',str(scene),'--audit',str(audit),
                    '--config',str(ROOT/spec['mission_configuration']),'--camera-config',str(cam_path),
                    '--conditions',str(conditions_path),'--output',str(dest),'--instance',str(instance+i)]
                saved=dict(hashes,**{str(p):digest(p) for p in [cam_path,conditions_path]})
                jobs.append((command,str(dest),saved,spec))
            for case,result in zip(group,pool.map(trial,jobs)):
                row=dict(name=case['name'],category=case['category'],camera_seed=case['camera_seed'],**result);attempts.append(row)
                summary=report(attempts,spec);(output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],task_success=row.get('task_success'),safe_termination=row.get('safe_termination'),failure=row.get('failure') or row.get('summary',{}).get('failure'))),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--instance',type=int,default=55)
    a=p.parse_args();return 0 if run(a.scene,a.audit,a.config,a.output,workers=a.workers,instance=a.instance)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
