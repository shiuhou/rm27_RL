import argparse,json
from pathlib import Path
from .dataset import generate,settings,arrays
from .network import train
from ..evidence import write_json


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['generate','train','evaluate','spline','benchmark','plot','mission','confirm'])
    p.add_argument('--output',type=Path,required=True);p.add_argument('--input',type=Path);p.add_argument('--policy',type=Path)
    p.add_argument('--history',action='store_true');p.add_argument('--no-observer',action='store_true');p.add_argument('--extra',type=Path)
    p.add_argument('--epochs',type=int);p.add_argument('--seed',type=int,default=92317);p.add_argument('--hybrid',action='store_true')
    p.add_argument('--profile',default='perfect');p.add_argument('--split',default='validation');p.add_argument('--oracle',action='store_true')
    p.add_argument('--mix',type=float,default=0.);p.add_argument('--width',type=int);p.add_argument('--baseline',action='store_true')
    a=p.parse_args()
    if a.output.exists():p.error('Use a new output directory')
    try:
        if a.command=='generate':r=generate(a.output)
        elif a.command=='train':
            lags=settings()['history_lags'] if a.history else [0]
            from .evaluation import PROFILES
            X,Y,VX,VY,sources=arrays(a.input,lags,not a.no_observer,a.extra,PROFILES[a.profile]);s=settings()
            s['training_observation_profile']=a.profile
            if a.epochs:s['epochs']=a.epochs
            if a.width:s['hidden']=[a.width,a.width]
            r=train(X,Y,VX,VY,a.output,s,lags,a.seed);write_json(a.output/'sources.json',sources)
        elif a.command=='spline':
            from .spline import fit
            r=fit(a.input,a.output)
        elif a.command=='evaluate':
            from .evaluation import run
            r=run(a.policy,a.output,a.split,a.profile,a.hybrid,a.oracle,not a.no_observer,a.mix,a.baseline)
        elif a.command=='confirm':
            from .campaign import confirm
            r=confirm(a.output,a.input,a.policy)
        elif a.command=='mission':
            from .mission import run
            r=run(a.policy,a.output,profile=a.profile)
        elif a.command=='benchmark':
            from .evaluation import benchmark
            r=benchmark(a.policy,a.output,a.hybrid)
        else:
            from .analysis import plot
            r=plot(a.input,a.output)
    except Exception as exc:
        from ..evidence import save_failure
        save_failure(a.output,exc)
        write_json(a.output/'requested_settings.json',settings())
        raise
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)

if __name__=='__main__':main()
