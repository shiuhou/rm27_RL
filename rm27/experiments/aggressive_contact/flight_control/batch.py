"""Frozen local SITL matrix; distinct ports and artifact directories per worker."""
import argparse,contextlib,json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from ..evidence import snapshot,write_json,save_failure
from ..model import load_config


def trial(job):
    from .sitl import run
    dest=Path(job.pop('output'));log=dest.parent/(dest.name+'.log')
    with log.open('w') as f,contextlib.redirect_stdout(f),contextlib.redirect_stderr(f):
        try:return run(str(dest),**job)
        except Exception as exc:return save_failure(dest,exc)


def run(config,output,workers=3):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);spec=json.loads(Path(config).read_text())
    snapshot(output,load_config());write_json(output/'manifest.json',spec);rows=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for offset in range(0,len(spec['cases']),workers):
            group=spec['cases'][offset:offset+workers];jobs=[]
            for i,case in enumerate(group):
                study=output/(case['name']+'.json');write_json(study,case.get('study',{}))
                jobs.append(dict(output=str(output/case['name']),mode='maneuver',observation=case.get('observation','telemetry'),
                    action_layer='attitude',instance=45+i,duration=case.get('duration',9.),speedup=2.,
                    parameter_file=case.get('parameters','configs/contact_phase4/attitude_unshaped.parm'),
                    skill_mode=case.get('mode','balanced'),study_file=str(study)))
            for case,result in zip(group,pool.map(trial,jobs)):
                rows.append(dict(name=case['name'],result=result));write_json(output/'partial.json',rows)
                print(case['name'],result['passed'],result.get('safe_abort'),flush=True)
    r=dict(passed=all('evaluation_failure' not in x['result'] and 'failure' not in x['result'] and 'exception' not in x['result'] for x in rows),scope='execution integrity, not flight success rate',results=rows)
    write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--output',required=True);p.add_argument('--workers',type=int,default=3);a=p.parse_args();r=run(a.config,a.output,a.workers);raise SystemExit(0 if r['passed'] else 1)
