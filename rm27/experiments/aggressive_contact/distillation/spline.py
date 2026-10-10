"""Hybrid imitation: supervised compact reference + retained geometric feedback.

This is an explicit alternative to direct action BC, not a pure neural policy.
Only recorded training reference labels are fit. No trajectory optimizer is
invoked by the fitted reference. Knot geometry is uniform, not teacher knots.
"""
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from ..evidence import write_json
from .dataset import settings


class Reference:
    def __init__(self,knots,coeff,duration):
        self.spline=BSpline(knots,coeff,5);self.duration=duration
        self.derivatives=[self.spline.derivative(i) for i in range(4)]
    def sample(self,t):
        elapsed=min(max(t,0),self.duration)
        p,v,a,j=[f(elapsed) for f in self.derivatives]
        if t>self.duration:
            h=min(.25,t-self.duration);p=p+v*h+.5*a*h*h;v=v+a*h
        return p,v,a,j


class SplinePolicy:
    def __init__(self,references):self.references=references
    def reference(self,mode):return self.references[mode]
    @classmethod
    def load(cls,path):
        d=np.load(path);return cls([Reference(d[f'knots{i}'],d[f'coeff{i}'],float(d[f'duration{i}'])) for i in range(3)])


def fit(source,output,intervals=64):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    entries=json.loads((Path(source)/'index.json').read_text());arrays={};details={};sources=[]
    for mode,name in enumerate(settings()['modes']):
        times=[];refs=[]
        for entry in entries:
            if entry['split']!='train' or entry['teacher']!=name or not entry['passed']:continue
            p=Path(entry['path']);d=np.load(p/'demo.npz');m=json.loads((p/'metadata.json').read_text())
            tc=m['outcome']['first_contact']['time_s'];duration=m['outcome']['parameters']['duration_s']
            use=(d['time_s']<tc)&(np.arange(len(d['time_s']))%4==0)
            times.extend(d['time_s'][use]);refs.extend(d['reference'][use,:9]);sources.append(str(p))
        t=np.array(times);y=np.array(refs)
        knots=np.r_[[0.]*6,np.linspace(0,duration,intervals+1)[1:-1],[duration]*6];n=len(knots)-6
        b=BSpline(knots,np.eye(n),5)
        # Derivative supervision improves consistency over independent p/v/a fits.
        A=np.vstack([20*b(t),b.derivative(1)(t),.15*b.derivative(2)(t)])
        Y=np.vstack([20*y[:,:3],y[:,3:6],.15*y[:,6:9]])
        # Small jerk regularizer, solely numerical conditioning of unobserved tail.
        grid=np.linspace(0,duration,250);reg=b.derivative(3)(grid)*1e-6
        coeff=np.linalg.lstsq(np.vstack([A,reg]),np.vstack([Y,np.zeros((len(grid),3))]),rcond=1e-10)[0]
        arrays.update({f'knots{mode}':knots,f'coeff{mode}':coeff,f'duration{mode}':duration})
        residual=b(t)@coeff-y[:,:3]
        details[name]=dict(duration_s=duration,position_fit_rmse_m=float(np.sqrt(np.mean(residual**2))),coefficients=int(coeff.size))
    np.savez_compressed(output/'policy.npz',**arrays);write_json(output/'sources.json',sources)
    result=dict(passed=True,methods=details,checkpoint_bytes=(output/'policy.npz').stat().st_size,
                scope='learned reference ONLY; model-based observer, geometric feedback and estimated-contact recovery retained')
    write_json(output/'summary.json',result);return result
