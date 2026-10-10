"""Deterministic comparison of evidence-matched offline runs; no automatic ranking."""
import argparse
from pathlib import Path

from .common import require, read_json, write_json, canonical, object_hash


def compare_runs(runs):
    require(len(runs)>=2,'COMPARISON_INVALID','at least two runs')
    first=runs[0].get('comparison_identity')
    require(first is not None,'COMPARISON_UNAVAILABLE','run stopped before inputs were qualified')
    purpose=runs[0]['purpose']
    for run in runs:
        require(run.get('comparison_identity')==first and run.get('comparison_sha256')==object_hash(first),
                'COMPARISON_INPUT_MISMATCH','dataset, selection, calibration, timing or transforms differ')
        require(run['purpose']==purpose,'COMPARISON_INPUT_MISMATCH','cannot mix fixture and research results')
    rows=[]
    for run in sorted(runs,key=lambda r:(r['backend'],r['run_id'])):
        rows.append({k:run.get(k) for k in ('run_id','backend','backend_version','status','failure_reason',
                                           'online_trajectory','final_optimized_trajectory','benchmark','evaluation')})
    return dict(schema_version=1,purpose=purpose,comparison_identity=first,runs=rows,
                interpretation='Compare observable tradeoffs; absent states/resources/reference remain unavailable. Final optimized poses are not online outputs.',
                qualification='NO_STAGE_PASS')


def markdown(report):
    lines=['# Offline localization comparison','',f"Evidence: {report['purpose']}",'',
           '| Backend / run | Process status | Tracking coverage | Online | Final | Ground truth |',
           '|---|---|---|---|---|---|']
    def cell(value):
        return str(value).replace('|','\\|').replace('\n',' ') if value is not None else 'UNAVAILABLE'
    for run in report['runs']:
        evaluation=run.get('evaluation') or {}
        lines.append('| '+' | '.join(cell(v) for v in (run['backend']+' / '+run['run_id'],run['status'],
            evaluation.get('tracking_coverage'),run.get('online_trajectory'),run.get('final_optimized_trajectory'),
            evaluation.get('ground_truth_status','UNAVAILABLE')))+' |')
    lines += ['',report['interpretation'],'','Full version, behavior, timing, resources and diagnostic intervals: companion JSON.','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',nargs='+',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--reference-evaluations',nargs='*',type=Path,default=[])
    args=parser.parse_args()
    runs=[read_json(p) for p in args.runs]
    for path in args.reference_evaluations:
        reference=read_json(path)
        matches=[r for r in runs if r['run_id']==reference['run_id'] and r['config_sha256']==reference['run_config_sha256']]
        require(len(matches)==1,'COMPARISON_INPUT_MISMATCH','reference evaluation run identity')
        require(reference['purpose']==matches[0]['purpose'],'COMPARISON_INPUT_MISMATCH','reference purpose')
        matches[0].setdefault('evaluation',{})['reference_evaluation']=reference
    report=compare_runs(runs)
    args.out.mkdir(parents=True,exist_ok=False)
    write_json(args.out/'comparison.json',report)
    (args.out/'comparison.md').write_text(markdown(report))


if __name__=='__main__':
    main()
