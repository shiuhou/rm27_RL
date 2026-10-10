"""Small tanh MLP + Adam, NumPy-only deterministic training and inference."""
import json,time
from pathlib import Path
import numpy as np
from ..evidence import write_json


class MLP:
    def __init__(self,W,b,xmean,xscale,ymean,yscale,lags):
        self.W,self.b=W,b;self.xmean,self.xscale=xmean,xscale;self.ymean,self.yscale=ymean,yscale;self.lags=lags
    def __call__(self,x):
        a=(np.asarray(x)-self.xmean)/self.xscale
        for W,b in zip(self.W[:-1],self.b[:-1]):a=np.tanh(a@W+b)
        return (a@self.W[-1]+self.b[-1])*self.yscale+self.ymean
    def save(self,path):
        np.savez_compressed(path,**{f'W{i}':v for i,v in enumerate(self.W)},**{f'b{i}':v for i,v in enumerate(self.b)},
                            xmean=self.xmean,xscale=self.xscale,ymean=self.ymean,yscale=self.yscale,lags=self.lags)
    @classmethod
    def load(cls,path):
        a=np.load(path);n=len([k for k in a if k.startswith('W')]);return cls([a[f'W{i}'] for i in range(n)],[a[f'b{i}'] for i in range(n)],a['xmean'],a['xscale'],a['ymean'],a['yscale'],a['lags'].tolist())


def train(X,Y,VX,VY,output,settings,lags,seed=0,initial=None):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    rng=np.random.default_rng(seed);xm=X.mean(0);xs=np.maximum(X.std(0),.03);ym=Y.mean(0);ys=np.maximum(Y.std(0),[.15,.15,.15,.2])
    x=((X-xm)/xs).astype(np.float32);y=((Y-ym)/ys).astype(np.float32)
    vx=((VX-xm)/xs).astype(np.float32);vy=((VY-ym)/ys).astype(np.float32)
    widths=[X.shape[1],*settings['hidden'],4]
    W=[(rng.normal(size=(i,j))*np.sqrt(2/(i+j))).astype(np.float32) for i,j in zip(widths[:-1],widths[1:])]
    b=[np.zeros(j,dtype=np.float32) for j in widths[1:]]
    params=W+b;m=[np.zeros_like(p) for p in params];v=[np.zeros_like(p) for p in params]
    step=0;best=np.inf;records=[];begin=time.perf_counter()
    with (output/'progress.jsonl').open('w',buffering=1) as log:
        for epoch in range(settings['epochs']):
            indices=rng.permutation(len(x))
            for start in range(0,len(x),settings['batch_size']):
                ids=indices[start:start+settings['batch_size']];acts=[x[ids]]
                for wi,bi in zip(W[:-1],b[:-1]):acts.append(np.tanh(acts[-1]@wi+bi))
                pred=acts[-1]@W[-1]+b[-1];g=2*(pred-y[ids])/(len(ids)*4);gw=[];gb=[]
                for i in range(len(W)-1,-1,-1):
                    gw.insert(0,acts[i].T@g+1e-6*W[i]);gb.insert(0,g.sum(0))
                    if i:g=(g@W[i].T)*(1-acts[i]**2)
                step+=1
                for p,mi,vi,grad in zip(params,m,v,gw+gb):
                    np.clip(grad,-5,5,out=grad);mi*=.9;mi+=.1*grad;vi*=.999;vi+=.001*grad*grad
                    p-=settings['learning_rate']*(mi/(1-.9**step))/(np.sqrt(vi/(1-.999**step))+1e-8)
            z=vx
            for wi,bi in zip(W[:-1],b[:-1]):z=np.tanh(z@wi+bi)
            pred=z@W[-1]+b[-1];loss=float(np.mean((pred-vy)**2))
            record=dict(epoch=epoch,validation_normalized_mse=loss);records.append(record)
            if loss<best:
                best=loss;MLP(W,b,xm,xs,ym,ys,lags).save(output/'policy.npz');best_epoch=epoch
            if epoch%10==0:log.write(json.dumps(record)+'\n')
    model=MLP.load(output/'policy.npz');err=model(VX)-VY
    result=dict(passed=True,seed=seed,train_samples=len(X),validation_samples=len(VX),best_epoch=best_epoch,
                best_validation_normalized_mse=best,action_rmse=np.sqrt(np.mean(err**2,axis=0)).tolist(),
                parameter_count=sum(p.size for p in params),checkpoint_bytes=(output/'policy.npz').stat().st_size,
                wall_s=time.perf_counter()-begin,settings=settings,lags=lags,history=records)
    write_json(output/'summary.json',result);return result
