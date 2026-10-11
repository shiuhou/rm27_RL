"""Freeze inputs and retain every independent MTF or live-VIO return attempt."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(scene, output, *, dynamics, profile, count=20, instance=33, speedup=None):
    if count<=0 or profile not in ('mtf','vio'):
        raise ValueError('positive count and explicit mtf/vio profile required')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    scene=Path(scene).resolve();dynamics=Path(dynamics).resolve()
    model=mujoco.MjModel.from_xml_path(str(scene))
    mass=float(model.body_mass[model.body('drone_1').id])
    files=[scene,dynamics,ROOT/f'configs/px4/{profile}.json']
    xml=ET.parse(scene).getroot()
    for element in xml.findall('.//asset/*'):
        if element.get('file'):
            files.append((scene.parent/element.get('file')).resolve())
    files.append(ROOT/'artifacts/deps/PX4-Autopilot/build/px4_sitl_default/bin/px4')
    files+=list((ROOT/'rm27/autopilot/px4').glob('*.py'))
    if profile=='vio':
        files+=list((ROOT/'rm27/perception/sitl').glob('*.py'))
        files+=list((ROOT/'configs').glob('px4_sim_*.json'))
    hashes={str(p):digest(p) for p in files}
    speedup=(2. if profile=='vio' else 5.) if speedup is None else speedup
    manifest=dict(schema_version=1,profile=profile,required_count=count,instance=instance,
        seed_policy='fixed sensor seeds, separate PX4 process/parameters/log and native VIO session per attempt',
        gate_version='PX4_NAVIGATION_RETURN_V1',scene=str(scene),dynamics=str(dynamics),speedup=speedup,
        input_sha256=hashes,model_mass_kg=mass,git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        required_gates=['existing physical hover/rest thresholds','normal LAND and automatic disarm',
                        'no unexpected PX4 failsafe'],
        scope='short sensor/VIO return; no target approach or contact, not S4/S5 acceptance')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    attempts=[]
    for n in range(count):
        dest=output/f'flight-{n+1:02d}'
        if profile=='vio':
            command=[sys.executable,'-m','rm27.autopilot.px4.live_flight',str(scene)]
        else:
            command=[sys.executable,'-m','rm27','micro-flight',str(scene),'--autopilot','px4','--profile',profile]
        command+=['--output',str(dest),'--dynamics',str(dynamics),'--instance',str(instance),'--speedup',str(speedup)]
        changed=[p for p,h in hashes.items() if digest(p)!=h]
        if changed:
            result=dict(passed=False,failure='Frozen inputs changed: '+str(changed))
            code=None
        else:
            with (output/f'flight-{n+1:02d}.log').open('x') as log:
                process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            code=process.returncode
            summary=dest/'summary.json'
            result=json.loads(summary.read_text()) if summary.exists() else dict(passed=False,failure='summary missing')
        failsafes=[e for e in result.get('events',[]) if e.get('event')=='STATUSTEXT' and 'failsafe' in e.get('text','').lower()]
        passed=bool(code==0 and result.get('passed') is True and not failsafes)
        attempts.append(dict(attempt=n+1,command=command,returncode=code,path=str(dest/'summary.json'),
            passed=passed,summary=result,unexpected_failsafes=failsafes))
        failures=Counter(a['summary'].get('failure') or 'unexpected failsafe' for a in attempts if not a['passed'])
        scores={}
        for key in ['horizontal_error_m','height_error_m']:
            values=[a['summary'].get('independent_resting_score',{}).get(key) for a in attempts]
            values=[v for v in values if v is not None]
            scores[key]=dict(maximum=max(values),p95=float(np.percentile(values,95))) if values else None
        report=dict(passed=len(attempts)==count and all(a['passed'] for a in attempts),
            passed_count=sum(a['passed'] for a in attempts),total_attempts=len(attempts),required_count=count,
            autopilot='px4',model_mass_kg=mass,profile=profile,run_type='headless closed-loop SITL independent launches',
            gate_version=manifest['gate_version'],estimation_source='current-flight OpenVINS and numerical flow/range' if profile=='vio' else 'numerical flow/range/compass/IMU',
            failure_categories=dict(failures),resting_error=scores,attempts=attempts,
            scope=manifest['scope'])
        (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(attempt=n+1,passed=passed,failure=result.get('failure'),failsafes=len(failsafes))),flush=True)
        if changed:
            break
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dynamics',type=Path,default=ROOT/'configs/dynamics_150g.json')
    p.add_argument('--profile',choices=('mtf','vio'),required=True)
    p.add_argument('--count',type=int,default=20);p.add_argument('--instance',type=int,default=33)
    p.add_argument('--speedup',type=float)
    a=p.parse_args();r=run(a.scene,a.output,dynamics=a.dynamics,profile=a.profile,
                         count=a.count,instance=a.instance,speedup=a.speedup)
    return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
