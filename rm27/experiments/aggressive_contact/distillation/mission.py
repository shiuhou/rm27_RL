"""Learned hybrid outbound/recovery, then explicit model-based return handoff."""
import json
from pathlib import Path
from ..model import load_config
from ..maneuver import episode
from ..teacher_return import continue_return
from ..evidence import write_json
from .dataset import designs,settings
from .spline import SplinePolicy
from .policy import StudentSkill
from .evaluation import PROFILES


def run(checkpoint,output,mode='balanced',seed=None,profile='noise_filtered',deployment=None,estimator_override=None):
    output=Path(output);cfg=load_config();model=SplinePolicy.load(checkpoint);holder=[]
    index=settings()['modes'].index(mode)
    def factory(teacher,initial,target,info):
        skill=StudentSkill(teacher,initial,target,info,model,index,dict(PROFILES[profile] if estimator_override is None else estimator_override,seed=0 if seed is None else seed+1000000),hybrid=True,detector_window_s=.02,deployment=deployment)
        holder.append(skill);return skill
    def return_after(c,bundle,skill,slot,path):
        # Explicit handoff after evaluator's held-recovery gate. No hidden teacher
        # action is used during student approach/contact/recovery.
        skill.save(path/'student.npz')
        return continue_return(c,bundle,skill.feedback,slot,path)
    r=episode(cfg,designs()[mode],output,seed,1.,finish_on_recovery=True,skill_factory=factory,after_recovery=return_after)
    if holder:holder[0].save(output/'student.npz')
    r['maneuver_passed']=r['passed'];r['passed']=bool(r['passed'] and r.get('return',{}).get('passed'))
    r['policy_checkpoint']=str(checkpoint);r['observation_profile']=profile
    if deployment is not None:r['deployment_options']=deployment
    if estimator_override is not None:r['estimator_options']=estimator_override
    r['scope']='student learned reference + model feedback through recovery; explicit model-based return; truth-assisted synthetic ego estimates; no takeoff/landing'
    if r.get('return'):r['mission_duration_s']=r['return']['completed_time_s']
    write_json(output/'summary.json',r);return r
