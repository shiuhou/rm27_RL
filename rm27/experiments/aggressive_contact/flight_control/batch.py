"""Frozen local SITL matrix; distinct ports and artifact directories per worker."""
import argparse,contextlib,hashlib,json
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from ..evidence import snapshot,write_json,save_failure
from ..model import load_config


def summarize(rows, spec):
    execution=all('evaluation_failure' not in x['result'] and 'failure' not in x['result'] and
                  'exception' not in x['result'] for x in rows)
    expected={c['name']:c.get('expectation','task_success') for c in spec['cases']}
    successes=sum(x['result'].get('passed') is True for x in rows)
    aborts=sum(x['result'].get('safe_abort') is True for x in rows)
    accepted=all(x['result'].get('safe_abort' if expected[x['name']]=='safe_abort' else 'passed') is True for x in rows)
    complete=len(rows)==len(spec['cases'])
    acceptance=spec.get('acceptance','execution_integrity')
    failures=Counter(reason for x in rows if not x['result'].get('passed')
        for reason in x['result'].get('contact_evaluation',{}).get('failure_modes',
        [x['result'].get('failure',x['result'].get('exception','unclassified'))]))
    return dict(passed=complete and execution and (accepted if acceptance=='task_success' else True),
        scope=acceptance,execution_passed=complete and execution,task_expectations_passed=complete and accepted,
        task_success_count=successes,safe_abort_count=aborts,total_attempts=len(rows),required_count=len(spec['cases']),
        autopilot=spec.get('autopilot','ardupilot'),run_type='headless closed-loop SITL independent launches',
        gate_version='existing Phase 4/5 contact/recovery evaluator',failure_categories=dict(failures),results=rows)


def trial(job):
    from .sitl import run
    dest=Path(job.pop('output'));log=dest.parent/(dest.name+'.log')
    with log.open('w') as f,contextlib.redirect_stdout(f),contextlib.redirect_stderr(f):
        try:return run(str(dest),**job)
        except Exception as exc:return save_failure(dest,exc)


def run(config,output,workers=3):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);spec=json.loads(Path(config).read_text())
    snapshot(output,load_config());write_json(output/'manifest.json',spec);rows=[]
    hashes=json.loads((output/'provenance.json').read_text())['source_sha256']
    from ..model import ROOT
    frozen={str(ROOT/path):value for path,value in hashes.items()}
    # The selected Student lives in ignored artifacts. Freeze its actual
    # bundle too, so source stability cannot hide changed fitted coefficients.
    frozen[str(Path(config).resolve())]=hashlib.sha256(Path(config).read_bytes()).hexdigest()
    for case in spec['cases']:
        paths=[]
        bundle=case.get('study',{}).get('student_bundle')
        if bundle:paths.extend((ROOT/bundle).rglob('*'))
        if case.get('parameters'):paths.append(ROOT/case['parameters'])
        for path in paths:
            if path.is_file():frozen[str(path.resolve())]=hashlib.sha256(path.read_bytes()).hexdigest()
    backends={case.get('autopilot',spec.get('autopilot','ardupilot')) for case in spec['cases']}
    for backend in backends:
        binary=ROOT/('artifacts/deps/PX4-Autopilot/build/px4_sitl_default/bin/px4' if backend=='px4'
                    else 'rm27/autopilot/vendor/ardupilot/build/sitl/bin/arducopter')
        frozen[str(binary)]=hashlib.sha256(binary.read_bytes()).hexdigest()
    write_json(output/'frozen_inputs.json',frozen)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            changed=[p for p,h in frozen.items() if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
            if changed:
                rows.extend(dict(name=case['name'],result=dict(passed=False,failure='Frozen inputs changed: '+str(changed))) for case in group)
                write_json(output/'summary.json',summarize(rows,spec))
                continue
            for i,case in enumerate(group):
                autopilot=case.get('autopilot',spec.get('autopilot','ardupilot'))
                study=output/(case['name']+'.json');write_json(study,case.get('study',{}))
                jobs.append(dict(output=str(output/case['name']),mode='maneuver',observation=case.get('observation','telemetry'),
                    action_layer=case.get('action_layer','attitude'),instance=45+i,duration=case.get('duration',9.),speedup=2.,
                    parameter_file=case.get('parameters','configs/px4/response.json' if autopilot=='px4' else 'configs/contact_phase4/attitude_unshaped.parm'),
                    autopilot=autopilot,policy=case.get('policy','student'),
                    skill_mode=case.get('mode','balanced'),study_file=str(study)))
            for case,result in zip(group,pool.map(trial,jobs)):
                rows.append(dict(name=case['name'],result=result));write_json(output/'partial.json',rows)
                print(case['name'],result['passed'],result.get('safe_abort'),flush=True)
                write_json(output/'summary.json',summarize(rows,spec))
    r=summarize(rows,spec)
    write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--output',required=True);p.add_argument('--workers',type=int,default=3);a=p.parse_args();r=run(a.config,a.output,a.workers);raise SystemExit(0 if r['passed'] else 1)
