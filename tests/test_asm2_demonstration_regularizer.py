import unittest
import copy
import io
import tempfile
from pathlib import Path
import numpy as np
import torch
from simpleflight_lab import ppo
from rm27_asm2.demonstration_regularizer import install_regularizer, coefficient

class RegularizerTests(unittest.TestCase):
    def test_schedule(self):
        self.assertEqual(coefficient(0),.01)
        self.assertEqual(coefficient(196608),.005)
        self.assertEqual(coefficient(393216),0)
        with self.assertRaises(ValueError):coefficient(-1)

    def test_actual_ppo_update_and_disabled_equivalence(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'data.npz'
            np.savez(path,observations=np.ones((2,45),np.float32),raw_actions=np.ones((2,4),np.float32),
                demonstration=np.array([True,False]),source=np.array(['s','s']),trajectory=np.array(['a','b']))
            def run(enabled):
                torch.manual_seed(17)
                with install_regularizer(path,initial=enabled):
                    obj=ppo.PPO({'policy':torch.zeros(1,45),'critic':torch.zeros(1,45)},1.)
                for key in obj.rollout:obj.rollout[key].zero_()
                obj.rollout['advantage']=torch.zeros(64,1)
                obj.rollout['return']=torch.zeros(64,1,1)
                before=copy.deepcopy(obj.actor.state_dict())
                obj.update()
                return obj,before
            a,before=run(0);b,_=run(.01)
            for k,v in before.items():self.assertTrue(torch.equal(v,a.actor.state_dict()[k]))
            self.assertFalse(torch.equal(a.actor.mean[-1].weight,b.actor.mean[-1].weight))
            self.assertFalse(torch.equal(a.actor.log_std,b.actor.log_std))
            for k,v in a.critic.state_dict().items():self.assertTrue(torch.equal(v,b.critic.state_dict()[k]))
            self.assertEqual(b.demo_transitions,64)


class R9PreflightTests(unittest.TestCase):
    def test_gaussian_direction_and_tanh_constant(self):
        mean=torch.tensor([[.2,-.3,.4,-.1]],requires_grad=True)
        logstd=torch.tensor([-.2,.1,-.4,.3],requires_grad=True)
        target=torch.tensor([[.7,.5,-.2,-.5]])
        lp=ppo.log_prob(target,mean,logstd)
        expected=torch.distributions.Normal(mean,logstd.exp()).log_prob(target).sum(-1)
        torch.testing.assert_close(lp,expected)
        loss=-lp.sum();gm,gs=torch.autograd.grad(loss,(mean,logstd),retain_graph=True)
        torch.testing.assert_close(gm,(mean-target)*torch.exp(-2*logstd))
        torch.testing.assert_close(gs,(1-(target-mean).square()*torch.exp(-2*logstd)).sum(0))
        corrected=loss+torch.log1p(-torch.tanh(target).square()).sum()
        cm,cs=torch.autograd.grad(corrected,(mean,logstd))
        self.assertTrue(torch.equal(cm,gm)); self.assertTrue(torch.equal(cs,gs))
        self.assertLess(float(-ppo.log_prob(target,mean-.001*gm,logstd-.001*gs)),float(loss))
        for invalid in (float('nan'),float('inf'),-.1,.5):
            with self.assertRaises(ValueError):coefficient(invalid)

    def test_workflow_checkpoint_metadata_and_counter_guard(self):
        root=Path(__file__).resolve().parents[1]
        dataset=root/'.agents/runs/asm2-terminal/r8/demonstrations.npz'
        if not dataset.exists():self.skipTest('Archived dataset required')
        with install_regularizer(dataset):
            obj=ppo.PPO({k:torch.zeros(1,45) for k in ('policy','critic')},1.)
            payload=dict(task='asm2_resumable_contact',completed_transitions=0,actor=obj.actor.state_dict())
            stream=io.BytesIO();torch.save(payload,stream);stream.seek(0)
            restored=torch.load(stream,weights_only=True)
            self.assertEqual(restored['r9_regularizer'],obj.demonstration_state_dict())
            self.assertNotIn('r9_regularizer',payload)
            with self.assertRaises(ValueError):torch.save(dict(payload,completed_transitions=1024),io.BytesIO())
        with install_regularizer(dataset,resume_state=restored['r9_regularizer']):
            recovered=ppo.PPO({k:torch.zeros(1,45) for k in ('policy','critic')},1.)
        self.assertEqual(recovered.demo_transitions,0)
        with self.assertRaises(ValueError):
            with install_regularizer(dataset,completed_transitions=1024):pass

    def test_real_checkpoint_zero_equivalence_and_resume(self):
        root=Path(__file__).resolve().parents[1]
        checkpoint=root/'.agents/runs/asm2-terminal/r8/bc.pt'
        if not checkpoint.exists():self.skipTest('Archived R8 checkpoint required for preflight')
        saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
        dataset=root/'.agents/runs/asm2-terminal/r8/demonstrations.npz'
        data=np.load(dataset)
        recorded=[]
        def make(scale=None):
            torch.manual_seed(42722)
            obs={k:torch.zeros(1,45) for k in ('policy','critic')}
            if scale is None: obj=ppo.PPO(obs,1.)
            else:
                with install_regularizer(dataset,initial=scale,diagnostics_callback=recorded.append):obj=ppo.PPO(obs,1.)
            for key,target in [('actor',obj.actor),('critic',obj.critic),('value_normalizer',obj.normalizer),
                               ('actor_optimizer',obj.actor_opt),('critic_optimizer',obj.critic_opt)]:
                target.load_state_dict(copy.deepcopy(saved[key]))
                self.assert_nested(saved[key],target.state_dict())
            # CPU arithmetic test only: GPU fused Adam is disabled identically in both arms.
            for opt in (obj.actor_opt,obj.critic_opt):
                for group in opt.param_groups:group['fused']=False
            # Actual archived observations; synthetic nonzero learner fixture, NOT an archived on-policy rollout.
            x=torch.from_numpy(data['observations'][np.linspace(0,len(data['observations'])-1,64,dtype=int)])[:,None,:]
            with torch.no_grad():
                mean=obj.actor(x); act=mean+torch.linspace(-1.,1.,256).reshape(64,1,4)*obj.actor.log_std.exp()
                obj.rollout={k:x.clone() for k in ('policy','critic')}
                obj.rollout.update(action=act,value=obj.critic(x),log_prob=ppo.log_prob(act,mean,obj.actor.log_std),
                    reward=torch.linspace(-1,1,64).reshape(64,1,1),done=torch.zeros(64,1,1,dtype=torch.bool),
                    advantage=torch.linspace(-2,2,64).reshape(64,1),**{'return':torch.linspace(-1,1,64).reshape(64,1,1)})
            return obj
        a=make();zero=make(0);b=make(.01)
        torch.manual_seed(315);a.update()
        torch.manual_seed(315);zero.update()
        for field in ('actor','critic','normalizer','actor_opt','critic_opt'):
            self.assert_nested(getattr(a,field).state_dict(),getattr(zero,field).state_dict())
        torch.manual_seed(315);b.update()
        self.assert_nested(a.critic.state_dict(),b.critic.state_dict())
        self.assert_nested(a.critic_opt.state_dict(),b.critic_opt.state_dict())
        self.assertFalse(torch.equal(a.actor.log_std,b.actor.log_std))
        self.assertEqual(b.last_demo_metrics['actor_minibatches'],64)
        print('NONZERO_SYNTHETIC_FIXTURE_DIAGNOSTICS',b.last_demo_metrics)
        resumed=make(.01)
        for field in ('actor','critic','normalizer','actor_opt','critic_opt'):
            getattr(resumed,field).load_state_dict(copy.deepcopy(getattr(b,field).state_dict()))
        resumed.load_demonstration_state_dict(b.demonstration_state_dict())
        torch.manual_seed(316);b.update()
        torch.manual_seed(316);resumed.update()
        for field in ('actor','critic','normalizer','actor_opt','critic_opt'):
            self.assert_nested(getattr(b,field).state_dict(),getattr(resumed,field).state_dict())
        self.assertEqual(recorded[-1]['completed_transitions'],128)
        self.assertEqual(b.demo_transitions,128)
        self.assertEqual(b.demonstration_state_dict(),resumed.demonstration_state_dict())
        bad=b.demonstration_state_dict();bad['dataset_sha256']='changed'
        with self.assertRaises(ValueError):resumed.load_demonstration_state_dict(bad)

    def assert_nested(self,a,b):
        if isinstance(a,torch.Tensor):self.assertTrue(torch.equal(a,b))
        elif isinstance(a,dict):
            self.assertEqual(set(a),set(b))
            for k in a:self.assert_nested(a[k],b[k])
        elif isinstance(a,(list,tuple)):
            self.assertEqual(len(a),len(b))
            for x,y in zip(a,b):self.assert_nested(x,y)
        else:self.assertEqual(a,b)

if __name__=='__main__':unittest.main()
