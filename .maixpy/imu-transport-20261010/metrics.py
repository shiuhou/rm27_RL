"""Offline comparison only; receipt jitter is not synchronized latency."""
import json
import math
from pathlib import Path
import re
import statistics

HERE=Path(__file__).resolve().parent

def percentile(values,q):
    if not values: return None
    v=sorted(values); at=(len(v)-1)*q; lo=math.floor(at); hi=math.ceil(at)
    return v[lo]+(v[hi]-v[lo])*(at-lo)

def timing(values,to_ms):
    d=[(b-a)*to_ms for a,b in zip(values,values[1:])]
    return dict(intervals=len(d),zero=sum(x==0 for x in d),backwards=sum(x<0 for x in d),
        min_ms=min(d) if d else None,p50_ms=percentile(d,.5),p95_ms=percentile(d,.95),
        p99_ms=percentile(d,.99),max_ms=max(d) if d else None,
        stdev_ms=statistics.pstdev(d) if d else None)

def capture(path,summary):
    rows=[json.loads(x) for x in path.read_text().splitlines()]
    s=json.loads(summary.read_text())
    fs=[r for r in rows if r['event']=='frame']
    im=[r for r in fs if r.get('type')=='HIGHRES_IMU']
    bad=[r for r in rows if r['event']=='bad_data']
    start=next(r['monotonic_ns'] for r in rows if r['event']=='start')
    end=next(r for r in rows if r['event']=='finished')
    first=fs[0]['receipt_mono_ns']
    return dict(file=path.name,elapsed_s=end['elapsed_s'],bytes=end['received_bytes'],
        all_bytes_per_s=end['received_bytes']/end['elapsed_s'],
        valid_frame_bytes_per_s=sum(len(r['wire_hex'])//2 for r in fs)/end['elapsed_s'],
        source=timing([r['fields']['time_usec'] for r in im],.001),
        receipt=timing([r['receipt_mono_ns'] for r in im],1e-6),
        highres=s['messages']['1:1:HIGHRES_IMU'],sources=s['sources'],
        bad_data=[dict(bytes=len(r['wire_hex'])//2,reason=r['reason'],
            since_start_ms=(r['receipt_mono_ns']-start)/1e6,
            before_first_valid_frame=r['receipt_mono_ns']<first) for r in bad],
        heartbeat_base_modes=sorted({r['fields']['base_mode'] for r in fs if r.get('type')=='HEARTBEAT'}),
        restored=next(r['termios_restored'] for r in rows if r['event']=='closed'))

def main():
    result={}
    for rate in (50,100):
        result[str(rate)]=capture(HERE/f'imu-controlled-{rate}-01.jsonl',HERE/f'analysis-{rate}/summary.json')
    records=[json.loads(x) for x in (HERE/'control-session.jsonl').read_text().splitlines()]
    loads=[]
    for r in records:
        if r.get('command')=='listener cpuload -n 10':
            values=[float(x) for x in re.findall(r'\n\s+load: ([\d.]+)',r['output'])]
            loads.append(dict(n=len(values),mean=statistics.mean(values),min=min(values),max=max(values)))
    result['px4_cpu_baseline_50_100']=loads
    result['stream_restore']=[r for r in records if 'configured_rates_restored' in r]
    with (HERE/'comparison.json').open('x',encoding='utf-8') as f: json.dump(result,f,indent=2)
    for rate in ('50','100'):
        r=result[rate]
        print(rate,json.dumps({k:r[k] for k in ('all_bytes_per_s','valid_frame_bytes_per_s','source','receipt','bad_data','heartbeat_base_modes','restored')}))
        print('HIGHRES',r['highres']['count'],r['highres']['source_timestamp']['effective_hz'],r['sources'])
    print('CPU',loads,'RESTORE',result['stream_restore'])

if __name__=='__main__': main()
