"""Export all trial outcomes, including aborts and failures, and static plots."""
import argparse,csv,json,hashlib
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ..evidence import write_json


def run(inputs,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);rows=[];hashes={}
    for source in map(Path,inputs):
        path=source/'summary.json';raw=path.read_bytes();hashes[str(path)]=hashlib.sha256(raw).hexdigest();r=json.loads(raw)
        for x in r.get('results',[dict(name=source.name,result=r)]):
            s=x['result'];c=s.get('contact_evaluation',s);f=c.get('first_contact') or {}
            row=dict(source=source.name,case=x.get('case',x.get('name')),mode=x.get('mode',s.get('selected_skill_mode','balanced')),variant=x.get('variant','sitl'),
                passed=s['passed'],safe_abort=s.get('safe_abort',False),aborted=s.get('aborted',False),
                contact_error_mm=1000*f['position_error_m'] if f else None,margin_mm=1000*f['region_margin_m'] if f else None,
                recovery_s=c.get('recovery_time_s'),confirmed_recovery_s=c.get('time_to_confirmed_recovery_s'),contact_time_s=f.get('time_s'),
                contact_rate_rad_s=f.get('body_rate_rad_s'),post_rate_rad_s=c.get('max_post_body_rate_rad_s'),altitude_loss_m=c.get('altitude_loss_m'),
                saturation=c.get('saturation_fraction'),failures=';'.join(c.get('failure_modes',[s.get('failure','UNKNOWN')])) if not s['passed'] else '')
            rows.append(row)
    with (output/'trials.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    groups=defaultdict(list)
    for r in rows:groups[(r['source'],r['mode'],r['variant'])].append(r)
    summary=[]
    for key,group in groups.items():
        summary.append(dict(source=key[0],mode=key[1],variant=key[2],trials=len(group),joint_success=sum(x['passed'] for x in group),
            safe_abort=sum(x['safe_abort'] for x in group),failures=dict(Counter(f for x in group for f in x['failures'].split(';') if f))))
    fig,axes=plt.subplots(1,3,figsize=(14,4.5))
    for ax,prefix,unit,label in zip(axes,['p','v','att'],[100,1,1],['Position bias [cm]','Velocity bias [m/s]','Attitude bias [deg]']):
        for axis,color in zip(range(3),['tab:blue','tab:orange','tab:green']):
            a=[x for x in rows if x['source']=='phase5-sensitivity-01' and x['case'].startswith(f'{prefix}{axis}_')]
            a.sort(key=lambda x:float(x['case'].split('_')[1]));xx=[float(x['case'].split('_')[1])*unit for x in a]
            yy=[x['margin_mm'] if x['margin_mm'] is not None else np.nan for x in a]
            ax.plot(xx,yy,color=color,label=('roll','pitch','yaw')[axis] if prefix=='att' else ('x','y','z')[axis])
            for x,y,r in zip(xx,yy,a):ax.scatter(x,y,marker='o' if r['passed'] else 'x',color=color)
        ax.axhline(8,ls='--',color='black',lw=1,label='8 mm reserve');ax.axhline(0,color='gray',lw=.7);ax.grid(alpha=.2);ax.set_xlabel(label);ax.set_ylabel('First-contact margin [mm]');ax.legend(fontsize=8)
    fig.suptitle('Balanced full-field hybrid / rate proxy; circles=joint PASS, crosses=FAIL; no-contact trials in CSV')
    fig.tight_layout();fig.savefig(output/'sensitivity.png',dpi=160);plt.close(fig)
    guard=[x for x in rows if x['source']=='phase5-guard-holdout-01'];fig,axes=plt.subplots(1,4,figsize=(14,4),sharey=True)
    for tier,ax in enumerate(axes):
        for j,variant in enumerate(['fixed','guard','scheduled']):
            a=[x for x in guard if x['case'].startswith(f'tier{tier}_') and x['variant']==variant]
            success=sum(x['passed'] for x in a);abort=sum(x['safe_abort'] for x in a)
            ax.bar(j,success,color='tab:green');ax.bar(j,abort,bottom=success,color='tab:blue');ax.bar(j,len(a)-success-abort,bottom=success+abort,color='tab:red')
        ax.set_xticks(range(3),['fixed','guard','schedule']);ax.set_title(['low uncertainty','reported bias','50 ms delay','hidden bias'][tier]);ax.set_ylim(0,6);ax.set_ylabel('Trials / 6')
    fig.suptitle('Green: contact+recovery; blue: safe abort (NOT task success); red: failure')
    fig.tight_layout();fig.savefig(output/'commitment.png',dpi=160);plt.close(fig)
    r=dict(passed=True,scope='aggregation complete, not flight success',trials=len(rows),groups=summary,input_sha256=hashes)
    write_json(output/'summary.json',r);return r

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',nargs='+',required=True);p.add_argument('--output',required=True);a=p.parse_args();run(a.input,a.output)
