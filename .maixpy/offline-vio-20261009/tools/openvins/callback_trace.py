"""Read-only parser for isolated RM27 OpenVINS callback diagnostic prints.

Log order is not an internal IMU-buffer trace: another worker can run between
feed_imu returning and its print. Never infer exact integration bounds from this.
"""
from decimal import Decimal,InvalidOperation
import hashlib
import json
from pathlib import Path
import re


def parse_trace(text):
    imu=[];dispatch=[];pending=None
    def stamp(v):
        try: d=Decimal(v)
        except InvalidOperation: raise ValueError('invalid trace timestamp')
        if not d.is_finite() or d<0: raise ValueError('invalid trace timestamp')
        return d
    for line in text.splitlines():
        if '[RM27_' not in line: continue
        if '[RM27_CONTROL]' in line and line.count('[RM27_') == 1: continue
        match=re.search(r'\[RM27_(IMU_FED|DISPATCH|DONE)\]\s+([^\x1b]+)',line)
        if not match or line.count('[RM27_') != 1:
            raise ValueError('malformed trace marker')
        kind=match[1];values=match[2].split()
        if len(values) != {'IMU_FED':1,'DISPATCH':3,'DONE':2}[kind]:
            raise ValueError('wrong trace field count')
        if kind != 'IMU_FED' and (not values[-1].isdigit() or int(values[-1]) >= 2**64):
            raise ValueError('invalid uint64 RNG state')
        now=stamp(values[0])
        if kind=='IMU_FED':
            if imu and now<=stamp(imu[-1]): raise ValueError('IMU trace not strictly ordered')
            imu.append(values[0])
        elif kind=='DISPATCH':
            if pending is not None or (dispatch and now<=stamp(dispatch[-1]['camera'])):
                raise ValueError('overlapping/unordered camera dispatch')
            stamp(values[1])
            pending=dict(camera=values[0],imu_trigger=values[1],rng_before=int(values[2]),
                imu_fed_log_count=len(imu),last_imu_fed_log=imu[-1] if imu else None)
        else:
            if pending is None or stamp(pending['camera'])!=now:
                raise ValueError('camera completion mismatch')
            pending['rng_after']=int(values[1]);dispatch.append(pending);pending=None
    if pending is not None: raise ValueError('unfinished camera dispatch')
    return dict(imu_times=imu,dispatch=dispatch,
        limits='callback print ordering, not exact internal integration/scheduling timing; no hardware evidence')


def analyze(logs,output):
    output=Path(output)
    if output.exists(): raise FileExistsError(output)
    if not logs: raise ValueError('at least one trace log required')
    traces=[parse_trace(Path(p).read_text(errors='replace')) for p in logs]
    if any(not t['imu_times'] or not t['dispatch'] for t in traces):
        raise ValueError('IMU and completed camera traces required')
    result=dict(inputs_sha256={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in logs},
        runs=[dict(imu_fed=len(t['imu_times']),camera_dispatched=len(t['dispatch']),
                   rng_before_values=sorted({r['rng_before'] for r in t['dispatch']}),
                   rng_after_values=sorted({r['rng_after'] for r in t['dispatch']})) for t in traces],
        comparisons=[],limitations=traces[0]['limits'])
    for i in range(len(traces)):
        for j in range(i+1,len(traces)):
            a,b=traces[i],traces[j]
            ai={r['camera']:r for r in a['dispatch']};bi={r['camera']:r for r in b['dispatch']}
            common=sorted(ai.keys()&bi.keys(),key=Decimal)
            def differing(key): return [k for k in common if ai[k][key]!=bi[k][key]]
            horizon=differing('imu_trigger');rng=differing('rng_before');fed=differing('imu_fed_log_count')
            result['comparisons'].append(dict(a=i,b=j,identical_imu_fed_sequence=a['imu_times']==b['imu_times'],
                identical_camera_dispatch_sequence=[r['camera'] for r in a['dispatch']]==[r['camera'] for r in b['dispatch']],
                common_cameras=len(common),different_trigger_count=len(horizon),
                first_different_trigger_camera=horizon[0] if horizon else None,
                different_rng_before_count=len(rng),different_fed_log_count=len(fed),
                first_different_fed_log_camera=fed[0] if fed else None))
    with output.open('x') as stream: json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--logs',nargs='+',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();analyze(args.logs,args.output)
