"""Read-only ULog intake. python -m rm27.telemetry.ulog INPUT --output NEW_DIR."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def analyze(path, output):
    path=Path(path).resolve(); output=Path(output).resolve()
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    output.mkdir(parents=True,exist_ok=False)
    summary={'passed':False,'input':str(path),'sha256':digest,'run_type':'read-only ULog analysis','fusion':{},'topics':{},'failure':None}
    try:
        log=ULog(str(path));summary['file_corruption']=bool(log.file_corruption)
        summary['dropouts']=[{'duration_ms':int(d.duration),'timestamp':int(d.timestamp)} for d in log.dropouts]
        summary['initial_parameters']=log.initial_parameters
        summary['metadata']=log.msg_info_dict
        for dataset in log.data_list:
            key=f'{dataset.name}:{dataset.multi_id}';data=dataset.data;times=np.asarray(data['timestamp'],dtype=np.int64)
            delta=np.diff(times);duration=(times[-1]-times[0])/1e6 if len(times)>1 else 0.
            summary['topics'][key]={'samples':len(times),'duration_s':float(duration),'rate_hz':float((len(times)-1)/duration) if duration>0 else None,'max_gap_ms':float(delta.max()/1000) if len(delta) else None,'non_increasing_timestamps':int(np.sum(delta<=0)),'fields':list(data)}
            if dataset.name=='estimator_status_flags':
                summary['fusion'][str(dataset.multi_id)]={k:{'ever':bool(np.any(v)),'last':bool(v[-1]),'samples_active':int(np.count_nonzero(v))} for k,v in data.items() if k.startswith('cs_')}
            if dataset.name=='vehicle_status':
                summary['vehicle_status']={'armed_samples':int(np.sum(data['arming_state']==2)),'last_arming_state':int(data['arming_state'][-1]),'last_nav_state':int(data['nav_state'][-1])}
        summary['passed']=not summary['file_corruption']
    except Exception as exc:
        summary['failure']=f'{type(exc).__name__}: {exc}'
    (output/'summary.json').write_text(json.dumps(summary,indent=2,default=lambda x:x.item() if hasattr(x,'item') else str(x))+'\n')
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('input',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze(a.input,a.output);print(json.dumps({'passed':r['passed'],'sha256':r['sha256'],'fusion':r['fusion']},indent=2));return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
