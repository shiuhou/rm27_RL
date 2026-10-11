"""Frozen current-flight VIO/downward-camera fault matrix; retain all attempts."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

from .navigation_batch import ROOT, digest
from ...perception.sitl.live import DEFAULT_NATIVE


def frozen_inputs(scene, configuration):
    scene=Path(scene).resolve()
    files=[scene,Path(configuration).resolve(),ROOT/'configs/dynamics_150g.json',ROOT/'configs/px4/vio.json',
        ROOT/'rm27/simulation/micro_plant.py',ROOT/'rm27/autopilot/bridge/frames.py',
        ROOT/'tools/openvins/live_simulation_ros2.py',DEFAULT_NATIVE,
        ROOT/'artifacts/deps/PX4-Autopilot/build/px4_sitl_default/bin/px4']
    files+=list((ROOT/'rm27/autopilot/px4').glob('*.py'))
    files+=list((ROOT/'rm27/perception/sitl').glob('*.py'))
    files+=list((ROOT/'configs').glob('px4_sim_*.json'))
    for item in ET.parse(scene).getroot().findall('.//asset/*'):
        if item.get('file'):files.append((scene.parent/item.get('file')).resolve())
    install=json.loads(DEFAULT_NATIVE.read_text())
    files+=[Path(p) for p in install['files_sha256']]
    files.append(Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf')
    return {str(p):digest(p) for p in files}


def trial(job):
    command,destination,hashes=job
    changed=[p for p,h in hashes.items() if not Path(p).exists() or digest(p)!=h]
    if changed:return dict(passed=False,failure='Frozen inputs changed: '+str(changed),command=command)
    with destination.with_suffix('.log').open('x') as log:
        process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    def read(name):
        path=destination/name
        return json.loads(path.read_text()) if path.exists() else dict(passed=False,failure=name+' missing')
    summary=read('summary.json');fault=read('fault_summary.json')
    return dict(passed=bool(process.returncode==0 and summary.get('passed') is True and fault.get('passed') is True),
        returncode=process.returncode,command=command,summary=summary,fault_summary=fault,path=str(destination))


def report(attempts, spec):
    counts=Counter(a['kind'] for a in attempts)
    passes=Counter(a['kind'] for a in attempts if a['passed'])
    errors=[a.get('summary',{}).get('independent_resting_score',{}).get('horizontal_error_m') for a in attempts]
    errors=[e for e in errors if e is not None]
    failures=Counter(a.get('failure') or a.get('summary',{}).get('failure') or 'fault handling gate failed'
        for a in attempts if not a['passed'])
    return dict(passed=len(attempts)==len(spec['cases']) and all(a['passed'] for a in attempts),
        passed_count=sum(a['passed'] for a in attempts),total_attempts=len(attempts),required_count=len(spec['cases']),
        autopilot='px4',model_mass_kg=.15,estimation_source='current-flight native OpenVINS plus numerical flow/range',
        run_type='headless closed-loop SITL independent launches',gate_version=spec['gate_version'],
        per_fault={kind:dict(passed=passes[kind],total=count) for kind,count in counts.items()},
        resting_horizontal_error_m=dict(maximum=max(errors),p95=float(np.percentile(errors,95))) if errors else None,
        failure_categories=dict(failures),scope=spec['scope'],attempts=attempts)


def run(scene, configuration, output, *, workers=2, instance=37):
    if type(workers) is not int or not 1<=workers<=3 or not 0<=instance<=99-workers:
        raise ValueError('1..3 workers with distinct valid PX4 instances required')
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    spec=json.loads(Path(configuration).read_text())
    hashes=frozen_inputs(scene,configuration)
    manifest=dict(configuration=spec,input_sha256=hashes,scene=str(Path(scene).resolve()),
        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        command=sys.argv,workers=workers,first_instance=instance)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'source_diff.patch').write_bytes(subprocess.check_output(['git','diff','--binary','HEAD'],cwd=ROOT))
    attempts=[]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            for i,c in enumerate(group):
                dest=output/c['name']
                cmd=[sys.executable,'-m','rm27.autopilot.px4.return_faults',str(Path(scene).resolve()),
                    '--output',str(dest),'--kind',c['kind'],'--phase',c['phase'],'--delay',str(c['delay_s']),
                    '--duration',str(c['duration_s']),'--instance',str(instance+i),'--speedup',str(spec['speedup'])]
                jobs.append((cmd,dest,hashes))
            for case,result in zip(group,pool.map(trial,jobs)):
                row=dict(name=case['name'],kind=case['kind'],**result);attempts.append(row)
                summary=report(attempts,spec)
                (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
                print(json.dumps(dict(name=case['name'],passed=row['passed'],
                    failure=row.get('failure') or row.get('summary',{}).get('failure'))),flush=True)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('scene',type=Path)
    p.add_argument('--config',type=Path,default=ROOT/'configs/px4_return_faults_v1.json')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=2)
    p.add_argument('--instance',type=int,default=37)
    a=p.parse_args();return 0 if run(a.scene,a.config,a.output,workers=a.workers,instance=a.instance)['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
