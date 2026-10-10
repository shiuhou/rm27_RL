"""Matched observation ablations and matched-width corrective imitation.

Only existing episode-level train/validation sources are loaded. Phase-3 closed
loop validation/holdout are not used for gradients or normalization.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from .dataset import arrays
from .network import train,MLP
from ..evidence import write_json,snapshot
from ..model import load_config


def run(output,epochs=160):
    output=Path(output);output.mkdir(parents=True,exist_ok=False);snapshot(output,load_config())
    X,Y,VX,VY,sources=arrays('artifacts/distill-data-01',[0,2,6])
    np.savez_compressed(output/'training_arrays.npz',X=X,Y=Y,VX=VX,VY=VY)
    write_json(output/'sources.json',sources)
    base=np.zeros(32,bool);base[5:20]=True
    masks={'state':base.copy()}
    for name,indices in [('phase',range(5)),('actuator',range(20,25)),('contact',range(29,31))]:
        masks['state_'+name]=base.copy();masks['state_'+name][list(indices)]=True
    masks['state_history']=np.tile(base,3);masks['all']=np.ones(32,bool)
    summary={}
    for name,mask in masks.items():
        h=[0,2,6] if len(mask)>32 else [0];x=X[:,:len(mask)].copy();vx=VX[:,:len(mask)].copy()
        x[:,~mask]=0;vx[:,~mask]=0
        cfg=dict(hidden=[64,64],epochs=epochs,batch_size=512,learning_rate=.001)
        dest=output/name;r=train(x,Y,vx,VY,dest,cfg,h,seed=92323)
        m=MLP.load(dest/'policy.npz');m.W[0][~mask]=0;m.save(dest/'policy.npz')
        write_json(dest/'features.json',dict(mask=mask.tolist(),meaning='omitted features have exactly zero first-layer weights'))
        summary[name]=r;write_json(output/'partial.json',summary)
    # All-state small/large and history, same data/seed/epochs as all above.
    for name,width,lags in [('all_small',16,[0]),('all_large',128,[0]),('all_history',64,[0,2,6])]:
        n=32*len(lags);cfg=dict(hidden=[width,width],epochs=epochs,batch_size=512,learning_rate=.001)
        summary[name]=train(X[:,:n],Y,VX[:,:n],VY,output/name,cfg,lags,seed=92323)
        write_json(output/'partial.json',summary)
    for name,extra in [('dagger','artifacts/distill-dagger-data-val-01'),('corrective','artifacts/distill-interventions-01')]:
        dx,dy,dvx,dvy,src=arrays('artifacts/distill-data-01',[0,2,6],extra=extra)
        cfg=dict(hidden=[64,64],epochs=epochs,batch_size=512,learning_rate=.001)
        summary[name]=train(dx,dy,dvx,dvy,output/name,cfg,[0,2,6],seed=92323)
        write_json(output/name/'sources.json',src);write_json(output/'partial.json',summary)
    write_json(output/'summary.json',dict(passed=True,models=summary,split='whole prior train/validation scenarios; no Phase-3 holdout'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--epochs',type=int,default=160);a=p.parse_args();run(a.output,a.epochs)
