"""Read-only native-epoch termination and actual estimator fusion audit."""
import argparse
import json
from pathlib import Path

from rm27.autopilot.px4.return_fault_batch import digest
from rm27.telemetry.ulog import analyze


def run(recording,output):
    recording,output=Path(recording).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    def read(name):return json.loads((recording/name).read_text())
    logs=list((recording/'log').rglob('*.ulg'))
    if len(logs)!=1:raise ValueError('Exactly one original ULog required')
    files=[logs[0],*[recording/n for n in ['summary.json','epoch_boundary_summary.json',
        'native_fallback_parameters.json','vio/summary.json','vio/runtime_summary.json']],Path(__file__)]
    (output/'manifest.json').write_text(json.dumps(dict(input_sha256={str(p):digest(p) for p in files},
        source=Path(__file__).read_text()),indent=2)+'\n')
    report=analyze(logs[0],output/'ulog_analysis')
    epoch=read('epoch_boundary_summary.json');summary=read('summary.json')
    fallback=read('native_fallback_parameters.json');vio=read('vio/summary.json')
    fused=any(all(v.get(k,{}).get('ever') is True for k in ['cs_ev_pos','cs_ev_vel','cs_ev_hgt'])
        for v in report.get('fusion',{}).values())
    range_fused=any(v.get('cs_rng_hgt',{}).get('ever') is True for v in report.get('fusion',{}).values())
    events=epoch.get('epoch_events',[])
    checks=dict(declared_epoch_handled=epoch['passed'] is True,failed_task=summary['passed'] is False,
        physical_termination=epoch['termination']['passed'] is True,readback_fallback=fallback['verified'] is True,
        native_vio=vio['passed'] is True,native_session_closed=read('vio/runtime_summary.json')['passed'] is True,
        ulog_integrity=report['passed'] is True,actual_ev_fused_before_fault=fused,
        actual_range_height_fused=range_fused,physical_monitor=summary['independent_physical_monitor']['passed'] is True,
        epoch_event=bool(events) and events[0]['observed_epoch']==events[0]['registered_epoch']+1)
    result=dict(passed=all(checks.values()),checks=checks,task_success=False,
        scope='Known initial dock hover, declared native-publication frame reset, blocked new epoch and continued normal LAND; no10s hover, full-route or actual native restart qualification',
        resting_horizontal_error_m=epoch['termination']['horizontal_rest_error_m'],
        run_type='independent retained epoch flight and actual ULog fusion audit')
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.recording,args.output)['passed'] else 1)
