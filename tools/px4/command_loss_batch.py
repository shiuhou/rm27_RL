"""Reserved complete-route command interruptions with independent scoring."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from rm27.autopilot.px4.return_fault_batch import ROOT,digest,frozen_inputs
from rm27.telemetry.recording import compress_recording
from .archive_closed_auxiliary import run as archive_auxiliary
from .camera_storage import archive as archive_camera
from .score_command_loss import run as score


def trial(job):
    command,destination,hashes,spec=job
    destination=Path(destination)
    changed=[p for p,h in hashes.items() if not Path(p).is_file() or digest(p)!=h]
    if changed:
        return dict(passed=False,failure='Frozen inputs changed: '+str(changed),command=command)
    with destination.with_suffix('.log').open('x') as log:
        process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    def read(name):
        path=destination/name
        return json.loads(path.read_text()) if path.is_file() else dict(passed=False,failure=name+' missing')
    summary=read('summary.json')
    command_loss=read('command_loss_summary.json')
    qualification=dict(passed=False)
    try:
        qualification=score(destination,ROOT/spec['surface_configuration'],
            ROOT/spec['recovery_configuration'],destination/'independent_command_loss_score')
    except Exception as error:
        qualification['failure']=f'{type(error).__name__}: {error}'
    checks=dict(command_interruption=command_loss.get('passed') is True,
        independent_audits=qualification.get('passed') is True,
        failed_task=summary.get('passed') is False,
        causal_actor=summary.get('actor_input_policy')=='PX4 telemetry and current image observations; physical monitors score after actor completion',
        independent_physical_monitor=summary.get('independent_physical_monitor',{}).get('passed') is True)
    result=dict(passed=process.returncode==0 and all(checks.values()),task_success=False,
        returncode=process.returncode,checks=checks,qualification=qualification,
        command_loss_summary=command_loss,summary=summary,command=command,path=str(destination))
    if (destination/'metadata.json').is_file() and (destination/'summary.json').is_file():
        try:
            if (destination/'frames.jsonl').is_file():
                result['recording_archive']=compress_recording(destination/'frames.jsonl')
            cameras=[]
            for name in ['vio','return_camera']:
                camera=destination/name
                if any((camera/'images').glob('*.png')):
                    stored=archive_camera(camera)
                    cameras.append(dict(camera=name,result={k:v for k,v in stored.items() if k!='records'}))
            result['camera_storage']=cameras
            auxiliary=archive_auxiliary(destination,destination/'auxiliary_archive')
            result['auxiliary_storage']=dict(passed=auxiliary['passed'],saved_bytes=auxiliary['saved_bytes'])
        except Exception as error:
            result['storage_failure']=f'{type(error).__name__}: {error}'
    return result


def report(attempts,spec):
    errors=[a.get('command_loss_summary',{}).get('termination',{}).get('resting_horizontal_error_m') for a in attempts]
    errors=[e for e in errors if e is not None]
    return dict(passed=len(attempts)==len(spec['cases']) and all(a['passed'] for a in attempts),
        total_attempts=len(attempts),required_count=len(spec['cases']),
        task_successes=0,safe_terminations=sum(a['passed'] for a in attempts),
        unhandled_failures=sum(not a['passed'] for a in attempts),
        autopilot='px4',model_mass_kg=.15,
        estimation_source='current-flight native OpenVINS and numerical flow/range',
        run_type='headless closed-loop SITL independent launches; upstream commands stop during image-aligned dock return',
        gate_version=spec['gate_version'],scope=spec['scope'],attempts=attempts,
        resting_horizontal_error_m=dict(maximum=max(errors),p95=float(np.percentile(errors,95))) if errors else None)


def run(scene,audit,configuration,output,*,workers=2,instance=66):
    if not 1<=workers<=3 or not 0<=instance<=99-workers:
        raise ValueError('Distinct valid SITL instances required')
    scene,audit,configuration,output=map(lambda p:Path(p).resolve(),(scene,audit,configuration,output))
    spec=json.loads(configuration.read_text())
    mission=ROOT/spec['mission_configuration']
    camera=ROOT/spec['camera_configuration']
    recovery=ROOT/spec['recovery_configuration']
    certificate=ROOT/spec['continuous_certificate']
    proof=json.loads(certificate.read_text())
    if (proof.get('passed') is not True or proof.get('scene_sha256')!=digest(scene)
            or proof.get('configuration_sha256')!=digest(recovery)):
        raise ValueError('Matching passed continuous recovery certificate required')
    paths=[audit,mission,camera,recovery,certificate,ROOT/spec['surface_configuration'],
        ROOT/'rm27/experiments/image_conditions.py',ROOT/'rm27/experiments/sensor_clock.py',
        ROOT/'rm27/experiments/vio_initial_admission.py',ROOT/'rm27/experiments/fast_bearing.py',
        ROOT/'rm27/experiments/recovery_membership.py',ROOT/'rm27/experiments/recovery_corridor.py',
        ROOT/'rm27/experiments/target_region_audit.py',ROOT/'rm27/telemetry/recording.py',
        ROOT/'rm27/telemetry/ulog.py',Path(__file__),
        *[Path(__file__).with_name(n+'.py') for n in ['full_return_command_loss','score_command_loss',
            'initial_vio_admission','ground_rest','causal_flight','camera_storage','archive_closed_auxiliary']]]
    mission_spec=json.loads(mission.read_text())
    paths.extend([ROOT/'configs/px4'/f'{mission_spec["estimation_profile"]}.json',
        ROOT/mission_spec['return_termination_configuration']])
    video_report=ROOT/'artifacts/px4-lossless-video-runtime-01/summary.json'
    video=json.loads(video_report.read_text())
    if video.get('passed') is not True or digest(video['binary'])!=video['sha256']:
        raise ValueError('Verified lossless camera runtime required')
    paths.extend([video_report,Path(video['binary'])])
    hashes=frozen_inputs(scene,configuration)
    hashes.update({str(p.resolve()):digest(p) for p in paths})
    output.mkdir(parents=True,exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(dict(configuration=spec,input_sha256=hashes,
        command=sys.argv,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_snapshots={str(p):p.read_text() for p in paths if p.suffix=='.py'}),indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers]
            jobs=[]
            for index,case in enumerate(group):
                conditions=output/(case['name']+'-conditions.json')
                conditions.write_text(json.dumps(dict(initial_vio_admission=True,efficient_bearing=True),indent=2)+'\n')
                camera_path=output/(case['name']+'-camera.json')
                camera_path.write_text(json.dumps(dict(json.loads(camera.read_text()),seed=case['camera_seed']),indent=2)+'\n')
                destination=output/case['name']
                command=[sys.executable,'-m','tools.px4.full_return_command_loss',str(scene),
                    '--audit',str(audit),'--config',str(mission),'--camera-config',str(camera_path),
                    '--conditions',str(conditions),'--causal-mission-inputs',
                    '--instance',str(instance+index),'--output',str(destination)]
                saved=dict(hashes,**{str(p):digest(p) for p in [conditions,camera_path]})
                jobs.append((command,str(destination),saved,spec))
            for case,result in zip(group,pool.map(trial,jobs)):
                attempts.append(dict(name=case['name'],camera_seed=case['camera_seed'],**result))
                summary=report(attempts,spec)
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=result['passed'],checks=result.get('checks'))),flush=True)
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--workers',type=int,default=2)
    parser.add_argument('--instance',type=int,default=66)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.scene,args.audit,args.config,args.output,
        workers=args.workers,instance=args.instance)['passed'] else 1)
