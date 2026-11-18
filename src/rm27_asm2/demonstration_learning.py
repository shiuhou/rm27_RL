"""R8 archived command alignment and bounded source-grouped behavior cloning."""
from pathlib import Path
import copy
import json
import numpy as np
from .asset import ROOT, read_json, digest, write_result
from .flight_suite import rotation_matrix

CONFIG = ROOT/'configs/rm27_asm2_r8.json'


def observations(states, goal):
    return np.column_stack((np.tile(goal-states[:, :3], (1,10)), states[:,7:10],
        rotation_matrix(states[:,3:7]).transpose(0,2,1).reshape(-1,9), states[:,10:13])).astype(np.float32)


def invert_commands(ctbr, hover, epsilon):
    squashed = np.column_stack((ctbr[:,1:]/2., 2*(ctbr[:,0]/hover-1)))
    if np.max(np.abs(squashed)) > 1+1e-7:
        raise ValueError('Archived command outside actor interface')
    clipped = np.clip(squashed, -1+epsilon, 1-epsilon)
    return np.arctanh(clipped).astype(np.float32), squashed, np.abs(squashed-clipped)>0


def build():
    from .r7_experiment import configuration
    from .terminal_workflow import legacy_inputs
    from .terminal_native_eval import load_controller
    from .flight import load_profile, allocate_ctbr
    from .contact_task import load_contact_spec
    exp=read_json(CONFIG); out=ROOT/exp['output'];out.mkdir(exist_ok=True)
    cfg,_,_,_=configuration();_,task=legacy_inputs(cfg)
    profile,spec=load_profile(),load_contact_spec()
    goal=spec.target_center+(-spec.patch_face_z_m-task['goal_inset_m'])*spec.target_normal
    command,_=load_controller((ROOT/cfg['parent']).with_suffix('.npz'),digest(ROOT/cfg['parent']),profile,goal)
    pieces=[]; records=[]; pins={}; max_alloc=0.; max_actor=0.; max_motor=0.
    for phase in ('terminal-set','r5','r5-1'):
        folder=out.parent/phase
        manifest=folder/'manifest.json'
        saved=read_json(manifest)
        for p,h in saved['pins'].items():
            if digest(ROOT/p)!=h:
                correction=read_json(folder/'ledger-correction.json') if phase=='terminal-set' else {}
                if not (p==str((folder/'experiment.py').relative_to(ROOT)) and h==correction.get('original_sha256') and digest(ROOT/p)==correction.get('corrected_sha256')):
                    raise ValueError(f'Changed evidence: {p}')
                pins[str((folder/'ledger-correction.json').relative_to(ROOT))]=digest(folder/'ledger-correction.json')
        pins[str(manifest.relative_to(ROOT))]=digest(manifest)
        for path in sorted((folder/'cases').glob('*/outcome.json')):
            result=read_json(path)
            if not (result['qualified'] if phase=='terminal-set' else result['result']['qualified']):continue
            data=dict(np.load(path.parent/'physics.npz',allow_pickle=False))
            times=data['control_times_s'];idx=np.rint(times/float(data['dt'])).astype(int)
            np.testing.assert_allclose(times,np.arange(len(times))*.005,atol=1e-12,rtol=0)
            np.testing.assert_allclose(data['sample_times_s'][idx],times,atol=1e-12,rtol=0)
            states=data['states'][idx];cmd=data['ctbr_N_rad_s']
            raw,squashed,clipped=invert_commands(cmd,profile.hover_thrust_N,exp['bc']['inverse_tanh_epsilon'])
            alloc,_=allocate_ctbr(profile,cmd,states[:,10:13])
            err=float(np.max(np.abs(alloc-data['allocated_motor_target_N'])))
            if err>1e-12:raise ValueError('Motor allocation misalignment')
            max_alloc=max(max_alloc,err)
            np.testing.assert_allclose(data['motor_target_N'][idx+1],alloc,atol=1e-15,rtol=0)
            # Complete archived filter history, without any new integration.
            expected=data['motor_target_N'][1:]+(data['motor_thrust_N'][:-1]-data['motor_target_N'][1:])*np.exp(-float(data['dt'])/profile.motor_time_constant_s)
            motor_error=float(np.max(np.abs(expected-data['motor_thrust_N'][1:])))
            if motor_error>1e-12:raise ValueError('Motor history inconsistent')
            max_motor=max(max_motor,motor_error)
            prefix=(idx<int(data['switch_tick'])) if phase=='terminal-set' else np.zeros(len(idx),bool)
            if phase=='terminal-set':
                np.testing.assert_array_equal(data['transfer_integration_before'],data['transfer_integration_after'])
            for s,c in zip(states[~prefix],cmd[~prefix]):
                e=float(np.max(np.abs(command(s)-c)));max_actor=max(max_actor,e)
                if e>1e-12:raise ValueError('Frozen suffix action misalignment')
            source=result['source'] if phase=='terminal-set' else result['case']
            identity=f'{phase}/{path.parent.name}'
            pieces.append(dict(observations=observations(states,goal),raw_actions=raw,ctbr=cmd,
                states=states,motors=data['motor_thrust_N'][idx],held_targets=data['motor_target_N'][idx],
                source=np.full(len(idx),source),trajectory=np.full(len(idx),identity),
                prefix=prefix,demonstration=np.full(len(idx),phase=='terminal-set'),
                control_times_s=times,clipped=clipped))
            records.append(dict(identity=identity,source=source,samples=len(idx),prefix_samples=int(prefix.sum()),
                path=str(path.parent.relative_to(ROOT)),demonstration=phase=='terminal-set'))
            for p in (path,path.parent/'physics.npz'):
                pins[str(p.relative_to(ROOT))]=digest(p)
    if sum(r['demonstration'] for r in records)!=7:raise ValueError('Expected exactly seven qualified demonstrations')
    data={k:np.concatenate([p[k] for p in pieces]) for k in pieces[0]}
    with (out/'demonstrations.npz').open('xb') as f:np.savez_compressed(f,**data)
    write_result(out/'demonstrations.json',dict(records=records,pins=pins,samples=len(data['source']),
        demonstration_samples=int(data['demonstration'].sum()),clipped_components=int(data['clipped'].sum()),
        max_allocation_error_N=max_alloc,max_fixed_a_command_error=max_actor,max_motor_filter_error_N=max_motor,
        observation='Exact native float32 45D target delta x10, world velocity, R transpose, body rates.',
        target='p,q,r,T raw=atanh(normalized executed CTBR); endpoints censored at 1-1e-6; fixed-variance Gaussian mean squared error.',
        coverage='Seven correlated connections / three saved seeds / two original sources. 25 successful autonomous retention trajectories / five sources. All development data.'))
    print(json.dumps(dict(samples=len(data['source']),records=len(records),clipped=int(data['clipped'].sum()),allocation=max_alloc,actor=max_actor)))


def weights(data, mask):
    """Half demonstrations / half retention, equal sources then trajectories."""
    w=np.zeros(len(mask),np.float64)
    for demo in (False,True):
        group=mask & (data['demonstration']==demo)
        sources=np.unique(data['source'][group])
        for source in sources:
            same=group & (data['source']==source)
            trajectories=np.unique(data['trajectory'][same])
            for trajectory in trajectories:
                rows=same & (data['trajectory']==trajectory)
                w[rows]=.5/len(sources)/len(trajectories)/rows.sum()
    return w/w.sum()


def fit():
    import torch
    from simpleflight_lab.ppo import Actor
    from .r7_experiment import configuration
    torch.set_num_threads(4)
    exp=read_json(CONFIG);bc=exp['bc'];out=ROOT/exp['output']
    if (out/'bc.pt').exists():raise FileExistsError('BC already fitted')
    cfg,_,_,_=configuration();parent=torch.load(ROOT/cfg['parent'],map_location='cpu',weights_only=True)
    data=dict(np.load(out/'demonstrations.npz',allow_pickle=False))
    x=torch.from_numpy(data['observations']);y=torch.from_numpy(data['raw_actions'])
    def run(train,valid,epochs,early):
        torch.manual_seed(bc['seed']); actor=Actor(45);actor.load_state_dict(parent['actor'])
        actor.log_std.requires_grad_(False)
        opt=torch.optim.Adam(actor.mean.parameters(),lr=bc['learning_rate'])
        tw=torch.tensor(weights(data,train),dtype=torch.float32)
        vw=torch.tensor(weights(data,valid),dtype=torch.float32)
        history=[];best=float('inf');best_epoch=0;best_state=None
        def loss_all():
            with torch.no_grad():
                error=((actor(x)-y)*torch.exp(-actor.log_std)).square().mean(-1)
                return float((error*vw).sum()),float((error*tw).sum())
        val,tr=loss_all();history.append(dict(epoch=0,validation=val,training=tr));best=val;best_state=copy.deepcopy(actor.state_dict())
        for epoch in range(1,epochs+1):
            for _ in range(bc['steps_per_epoch']):
                rows=torch.multinomial(tw,bc['batch_size'],replacement=True)
                loss=.5*((actor(x[rows])-y[rows])*torch.exp(-actor.log_std)).square().mean()
                if not torch.isfinite(loss):raise FloatingPointError('Nonfinite BC loss')
                opt.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(actor.mean.parameters(),1.);opt.step()
            val,tr=loss_all();history.append(dict(epoch=epoch,validation=val,training=tr))
            if val<best-1e-6:best=val;best_epoch=epoch;best_state=copy.deepcopy(actor.state_dict())
            if early and epoch-best_epoch>=bc['patience']:break
        # Cross-validation selects epoch count; final fit uses that exact count.
        return actor,history,best_epoch
    folds=[]
    for source in ('stationary_0','stationary_2'):
        valid=data['source']==source
        actor,history,best_epoch=run(~valid,valid,bc['max_epochs'],True)
        folds.append(dict(source=source,best_epoch=best_epoch,history=history,
            train_sources=np.unique(data['source'][~valid]).tolist(),validation_sources=[source]))
    epochs=min(f['best_epoch'] for f in folds)
    actor,history,_=run(np.ones(len(x),bool),np.ones(len(x),bool),epochs,False)
    payload=copy.deepcopy(parent);payload['actor']={k:v.detach().cpu() for k,v in actor.state_dict().items()}
    payload['r8_bc']=dict(epochs=epochs,dataset_sha256=digest(out/'demonstrations.npz'),config_sha256=digest(CONFIG))
    torch.save(payload,out/'bc.pt')
    with (out/'bc.npz').open('xb') as f:np.savez_compressed(f,**{k:v.numpy() for k,v in payload['actor'].items()},checkpoint_sha256=digest(out/'bc.pt'))
    write_result(out/'bc-results.json',dict(folds=folds,selected_epochs=epochs,final_history=history,
        checkpoint_sha256=digest(out/'bc.pt'),parent_sha256=digest(ROOT/cfg['parent']),
        rule='Minimum best epoch across two source-excluded folds, bounded80 patience10; one final refit to all sources.',
        unchanged='Critic, value normalizer, both parent PPO optimizer states, log_std. BC has separate Adam; discarded before PPO.'))
    print(json.dumps(dict(selected_epochs=epochs,folds=[dict(source=f['source'],best_epoch=f['best_epoch'],first=f['history'][0],last=f['history'][-1]) for f in folds])))

if __name__=='__main__':
    import sys
    {'build':build,'fit':fit}[sys.argv[1]]()
