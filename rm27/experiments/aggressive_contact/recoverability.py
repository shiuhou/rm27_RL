"""Privileged local kNN recovery predictor with explicit support/abstention.

Trained at 120 ms after first contact. It predicts the *existing controller's*
recovery, not a viability kernel. Labels do not require valid first contact.
"""
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree

from .evidence import write_json
from .teacher_data import features,read_trial


def samples(roots,actuators=True):
    rows=[]
    for root in roots:
        for path in sorted(Path(root).glob('**/summary.json')):
            # Whole-maneuver roots contain a duplicate raw summary under flight/.
            if path.parent.name=='flight':continue
            flight=path.parent/'flight' if (path.parent/'flight').exists() else path.parent
            if not (flight/'physical.json').exists():continue
            f=features(path.parent,actuators)
            if f is None:continue
            _,s,c,p,a=read_trial(path.parent)
            rec=s.get('recovery_time_s')
            recovered=(rec is not None and rec<=c['gates']['recovery_deadline_s']
                       and (s.get('altitude_loss_m') or 0)<=c['gates']['max_altitude_loss_m']
                       and s['max_post_body_rate_rad_s']<=c['gates']['max_body_rate_rad_s'])
            time_remaining=4. if rec is None else max(0.,rec+c['gates']['recovery_hold_s']-.12)
            rows.append((f,[float(recovered),time_remaining,s.get('altitude_loss_m') or 0.],str(path.parent)))
    return rows


class RecoverabilityModel:
    def __init__(self,X,Y,actuators=True):
        self.X,self.Y=np.asarray(X),np.asarray(Y)
        self.mean=self.X.mean(axis=0);self.std=np.maximum(self.X.std(axis=0),.01)
        self.Z=(self.X-self.mean)/self.std
        self.tree=cKDTree(self.Z)
        distances,_=self.tree.query(self.Z,k=2)
        self.support_radius=max(.1,2.5*float(np.quantile(distances[:,1],.95)))
        self.actuators=actuators

    def predict(self,x):
        distances,ids=self.tree.query((np.asarray(x)-self.mean)/self.std,k=min(15,len(self.X)))
        distances,ids=np.atleast_1d(distances),np.atleast_1d(ids)
        w=1/(distances**2+.01);w/=w.sum()
        y=w@self.Y[ids]
        return dict(recovery_score=float(y[0]),remaining_time_s=float(y[1]),altitude_loss_m=float(y[2]),
                    supported=bool(distances[0]<=self.support_radius),nearest_distance=float(distances[0]))

    def save(self,path):np.savez_compressed(path,X=self.X,Y=self.Y,actuators=self.actuators)

    @classmethod
    def load(cls,path):
        d=np.load(path);return cls(d['X'],d['Y'],bool(d['actuators']))


def study(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    train=['artifacts/contact-confirmatory-20260923/training','artifacts/contact-v2-confirmatory-20260924/training',
           'artifacts/maneuver-confirmatory-01/training']
    validation=['artifacts/maneuver-confirmatory-01/evaluation','artifacts/maneuver-early-pose-01/evaluation']
    # These are historical data, explicitly validation, not the new teacher holdout.
    results={}
    for use_act in (False,True):
        rows=samples(train,use_act);val=samples(validation,use_act)
        X=np.array([r[0] for r in rows]);Y=np.array([r[1] for r in rows])
        model=RecoverabilityModel(X,Y,use_act)
        name='actuator_state' if use_act else 'kinematic'
        model.save(output/f'{name}.npz')
        begin=time.perf_counter();pred=[model.predict(r[0]) for r in val];query_s=time.perf_counter()-begin
        truth=np.array([r[1] for r in val]);prob=np.array([r['recovery_score'] for r in pred]);time_pred=np.array([r['remaining_time_s'] for r in pred])
        results[name]=dict(train_trials=len(rows),validation_trials=len(val),train_recovery_rate=float(Y[:,0].mean()),
                           validation_recovery_rate=float(truth[:,0].mean()),brier=float(np.mean((prob-truth[:,0])**2)),
                           constant_baseline_brier=float(np.mean((Y[:,0].mean()-truth[:,0])**2)),
                           time_mae_s=float(np.mean(abs(time_pred-truth[:,1]))),
                           constant_time_mae_s=float(np.mean(abs(Y[:,1].mean()-truth[:,1]))),
                           supported=sum(p['supported'] for p in pred),query_mean_ms=1000*query_s/len(val))
        write_json(output/f'{name}-validation.json',[dict(source=r[2],truth=r[1],prediction=p) for r,p in zip(val,pred)])
        write_json(output/f'{name}-training-sources.json',[r[2] for r in rows])
    result=dict(passed=True,scope='historical training and validation only; scores not calibrated probabilities; map labels not pooled',results=results,
                excluded_recovery_map='99 free-air trials use different anchor/no separation gate: incompatible outcome definition; retain as separate basin evidence')
    write_json(output/'summary.json',result);return result


def failure_audit(source,output):
    """Frozen models on historical v1 evaluation; includes recovery failures."""
    output=Path(output);output.mkdir(parents=True,exist_ok=False);results={}
    for name,act in [('kinematic',False),('actuator_state',True)]:
        model=RecoverabilityModel.load(Path(source)/(name+'.npz'))
        rows=samples(['artifacts/contact-confirmatory-20260923/evaluation'],act)
        preds=[model.predict(r[0]) for r in rows];truth=np.array([r[1] for r in rows]);prob=np.array([r['recovery_score'] for r in preds]);pt=np.array([r['remaining_time_s'] for r in preds])
        predicted=prob>=.8;positive=truth[:,0]>.5
        results[name]=dict(trials=len(rows),successes=int(positive.sum()),brier=float(np.mean((prob-truth[:,0])**2)),
          time_mae_s=float(np.mean(abs(pt-truth[:,1]))),constant_brier=float(np.mean((model.Y[:,0].mean()-truth[:,0])**2)),
          constant_time_mae_s=float(np.mean(abs(model.Y[:,1].mean()-truth[:,1]))),
          true_positive=int(sum(predicted&positive)),false_positive=int(sum(predicted&~positive)),
          true_negative=int(sum(~predicted&~positive)),false_negative=int(sum(~predicted&positive)),supported=sum(p['supported'] for p in preds))
        write_json(output/(name+'-predictions.json'),[dict(source=r[2],truth=r[1],prediction=p) for r,p in zip(rows,preds)])
    result=dict(passed=True,scope='historical unseen local v1 evaluation; no retraining; labels independent of first-contact validity',results=results)
    write_json(output/'summary.json',result);return result
