"""Freeze fresh complete-task cases for the explicit stable landing runner.

The existing failed reserved batches remain untouched. No completed attempt
is retried. Every flight retains its source copy, original and enriched result,
native VIO, physical score, actual EV fusion and recorded surface checks.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import subprocess
import sys

from rm27.autopilot.px4.return_fault_batch import ROOT, digest, frozen_inputs
from rm27.experiments.image_condition_batch import trial, report
from .camera_storage import archive as archive_camera
from .archive_closed_auxiliary import run as archive_auxiliary


def stored_trial(job):
    result=trial(job)
    destination=Path(job[1])
    archives=[]
    if (destination/'metadata.json').is_file() and (destination/'summary.json').is_file():
        try:
            for name in ['vio','return_camera']:
                camera=destination/name
                if any((camera/'images').glob('*.png')):
                    stored=archive_camera(camera)
                    archives.append(dict(camera=name,
                        manifest=str(camera/'images.archive.json') if stored.get('storage_changed') else None,
                        result={k:v for k,v in stored.items() if k!='records'}))
            auxiliary=archive_auxiliary(destination,destination/'auxiliary_archive')
            result['auxiliary_storage_summary']=dict(passed=auxiliary['passed'],saved_bytes=auxiliary['saved_bytes'])
        except Exception as error:
            # Storage failure does not rewrite a flight. Original data remain
            # intact; the archive verifies exact bytes before removing files.
            result['storage_failure']=f'{type(error).__name__}: {error}'
    result['camera_storage']=archives
    return result


def run(scene, audit, configuration, output, *, workers=2, instance=60):
    if not 1 <= workers <= 3 or not 0 <= instance <= 99-workers:
        raise ValueError('Distinct valid SITL instances required')
    scene, audit, configuration, output = map(
        lambda p: Path(p).resolve(), (scene, audit, configuration, output))
    spec = json.loads(configuration.read_text())
    if spec.get('runner') != 'stable_initial_and_landing_v1':
        raise ValueError('Explicit stable-runner gate required')
    certificate = ROOT/spec['continuous_certificate']
    proof = json.loads(certificate.read_text())
    recovery = ROOT/spec['recovery_configuration']
    if (proof.get('passed') is not True or proof.get('scene_sha256') != digest(scene) or
            proof.get('configuration_sha256') != digest(recovery)):
        raise ValueError('Matching passed continuous recovery certificate required')
    mission, camera = [ROOT/spec[k] for k in ['mission_configuration','camera_configuration']]
    profile = json.loads(mission.read_text())['estimation_profile']
    hashes = frozen_inputs(scene, configuration)
    paths = [audit, mission, camera, recovery, certificate, ROOT/spec['surface_configuration'],
        ROOT/'configs/px4'/f'{profile}.json',
        ROOT/'rm27/telemetry/recording.py', ROOT/'rm27/telemetry/ulog.py',
        ROOT/'rm27/experiments/image_condition_batch.py', ROOT/'rm27/experiments/image_conditions.py',
        ROOT/'rm27/experiments/sensor_clock.py', ROOT/'rm27/experiments/fast_bearing.py',
        ROOT/'rm27/experiments/vio_initial_admission.py', ROOT/'rm27/experiments/recovery_membership.py',
        ROOT/'rm27/experiments/recovery_corridor.py', ROOT/'rm27/experiments/target_region_audit.py',
        Path(__file__), *[Path(__file__).with_name(n+'.py') for n in
            ['run_initial_admission','initial_vio_admission','ground_rest','landing_alignment_pulse','causal_flight','physical_fixture',
             'camera_storage','archive_closed_auxiliary']]]
    video_report=ROOT/'artifacts/px4-lossless-video-runtime-01/summary.json'
    video=json.loads(video_report.read_text())
    if video.get('passed') is not True or digest(video['binary'])!=video['sha256']:
        raise ValueError('Verified exact-byte camera storage runtime required')
    paths.extend([video_report,Path(video['binary'])])
    termination = json.loads(mission.read_text()).get('return_termination_configuration')
    if termination:
        paths.append(ROOT/termination)
    hashes.update({str(p.resolve()):digest(p) for p in paths})
    output.mkdir(parents=True, exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(dict(configuration=spec,input_sha256=hashes,
        scene=str(scene),audit=str(audit),command=sys.argv,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_snapshots={str(p):p.read_text() for p in paths if p.suffix=='.py'}),indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0, len(spec['cases']), workers):
            group = spec['cases'][offset:offset+workers]
            jobs = []
            for i, case in enumerate(group):
                destination = output/case['name']
                if case['conditions'].get('initial_vio_admission') is not True:
                    raise ValueError('Every reserved case requires explicit initial admission')
                camera_path = output/(case['name']+'-camera.json')
                camera_path.write_text(json.dumps(dict(json.loads(camera.read_text()),seed=case['camera_seed']),indent=2)+'\n')
                conditions = output/(case['name']+'-conditions.json')
                conditions.write_text(json.dumps(case['conditions'],indent=2)+'\n')
                command = [sys.executable,'-m','tools.px4.run_initial_admission',str(scene),
                    '--audit',str(audit),'--config',str(mission),'--camera-config',str(camera_path),
                    '--conditions',str(conditions),'--output',str(destination),'--instance',str(instance+i),
                    '--ground-rest-confirmation','--retry-final-alignment','--causal-mission-inputs']
                saved = dict(hashes,**{str(p):digest(p) for p in [camera_path,conditions]})
                jobs.append((command,str(destination),saved,spec))
            for case, result in zip(group,pool.map(stored_trial,jobs)):
                summary=result.get('summary',{})
                actor=(summary.get('actor_input_policy')=='PX4 telemetry and current image observations; physical monitors score after actor completion')
                monitor=summary.get('independent_physical_monitor',{}).get('passed') is True
                if 'qualification' in result:
                    result['qualification']['checks'].update(causal_actor_inputs=actor,independent_physical_monitor=monitor)
                    result['qualification']['passed'] &= actor and monitor
                result['task_success']=bool(result.get('task_success') and actor and monitor)
                result['safe_termination']=bool(result.get('safe_termination') and actor and monitor)
                result['passed']=bool(result['passed'] and actor and monitor)
                attempts.append(dict(name=case['name'],category=case['category'],
                    camera_seed=case['camera_seed'],**result))
                summary = report(attempts,spec)
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=result['passed'],
                    task_success=result.get('task_success'),safe_termination=result.get('safe_termination'),
                    failure=result.get('failure') or result.get('summary',{}).get('failure'))),flush=True)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--workers',type=int,default=2)
    parser.add_argument('--instance',type=int,default=60)
    args = parser.parse_args()
    raise SystemExit(0 if run(args.scene,args.audit,args.config,args.output,
        workers=args.workers,instance=args.instance)['passed'] else 1)
