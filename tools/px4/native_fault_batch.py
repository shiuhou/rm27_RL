"""Reserved independent takeoff native-disconnect termination trials."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from rm27.autopilot.px4.return_fault_batch import frozen_inputs,ROOT,digest
from rm27.telemetry.recording import compress_recording
from rm27.telemetry.ulog import analyze
from .resume_image_batch import verify_frozen


def run(scene,audit,configuration,output,*,instance=46):
    scene,audit,configuration,output = map(lambda p:Path(p).resolve(),(scene,audit,configuration,output))
    spec = json.loads(configuration.read_text())
    hashes = frozen_inputs(scene,configuration)
    files = [audit,ROOT/spec['mission_configuration'],ROOT/spec['camera_configuration'],
        ROOT/spec['conditions_configuration'],ROOT/spec['installation'],Path(__file__),
        Path(__file__).with_name('run_native_variant.py'),Path(__file__).with_name('native_failure_termination.py'),
        Path(__file__).with_name('resume_image_batch.py'),ROOT/'rm27/experiments/image_conditions.py',
        ROOT/'rm27/experiments/fast_bearing.py',ROOT/'rm27/experiments/sensor_clock.py',
        ROOT/'rm27/experiments/vio_initial_admission.py']
    mission=json.loads((ROOT/spec['mission_configuration']).read_text())
    files.append(ROOT/'configs/px4'/f'{mission["estimation_profile"]}.json')
    hashes.update({str(p):digest(p) for p in files})
    output.mkdir(parents=True,exist_ok=False)
    (output/'manifest.json').write_text(json.dumps(dict(configuration=spec,input_sha256=hashes,
        scene=str(scene),audit=str(audit),command=sys.argv,instance=instance,
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()),indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    for case in spec['cases']:
        verify_frozen(hashes)
        camera = json.loads((ROOT/spec['camera_configuration']).read_text())
        camera['seed']=case['camera_seed']
        cam_path=output/(case['name']+'-camera.json')
        cam_path.write_text(json.dumps(camera,indent=2)+'\n')
        destination=output/case['name']
        command=[sys.executable,'-m','tools.px4.run_native_variant',str(scene),'--audit',str(audit),
            '--config',str(ROOT/spec['mission_configuration']),'--camera-config',str(cam_path),
            '--conditions',str(ROOT/spec['conditions_configuration']),'--installation',str(ROOT/spec['installation']),
            '--terminate-native-failure','--inject-native-failure-delay',str(case['delay_s']),
            '--instance',str(instance),'--output',str(destination)]
        with destination.with_suffix('.log').open('x') as log:
            process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        def read(name):
            p=destination/name
            return json.loads(p.read_text()) if p.is_file() else {}
        variant,summary,injection=read('native_variant_summary.json'),read('summary.json'),read('native_fault_injection.json')
        logs=list((destination/'log').rglob('*.ulg'))
        ulog=analyze(logs[0],destination/'ulog_analysis') if len(logs)==1 else {'passed':False}
        range_fused=any(flags.get('cs_rng_hgt',{}).get('ever') is True for flags in ulog.get('fusion',{}).values())
        checks=dict(native_fault_injected=injection.get('returncode')==0,
            declared_delay=injection.get('delay_s')==case['delay_s'],
            failed_task=summary.get('passed') is False and variant.get('task_success') is False,
            actual_termination=variant.get('safe_termination') is True,
            ulog_integrity=ulog.get('passed') is True,actual_range_fusion=range_fused,
            no_unexpected_failsafe=not any('failsafe' in str(e).lower() for e in variant.get('termination',{}).get('events',[])))
        row=dict(**case,passed=process.returncode==0 and all(checks.values()),checks=checks,
            returncode=process.returncode,command=command,path=str(destination),
            terminal=variant.get('termination'),camera_sha256=digest(cam_path))
        if (destination/'frames.jsonl').is_file():row['recording_archive']=compress_recording(destination/'frames.jsonl')
        attempts.append(row)
        values=[r['terminal']['horizontal_rest_error_m'] for r in attempts if r.get('terminal') and 'horizontal_rest_error_m' in r['terminal']]
        result=dict(passed=len(attempts)==len(spec['cases']) and all(r['passed'] for r in attempts),
            passed_count=sum(r['passed'] for r in attempts),total_attempts=len(attempts),required_count=len(spec['cases']),
            task_successes=0,gate_version=spec['gate_version'],run_type='headless closed-loop SITL independent launches',
            resting_horizontal_m=dict(maximum=max(values),p95=float(np.percentile(values,95))) if values else None,
            scope=spec['scope'],attempts=attempts)
        (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(name=case['name'],passed=row['passed'],checks=checks)),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path)
    for flag in ['audit','config','output']:
        parser.add_argument('--'+flag,type=Path,required=True)
    parser.add_argument('--instance',type=int,default=46)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.scene,args.audit,args.config,args.output,instance=args.instance)['passed'] else 1)
