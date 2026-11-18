import tempfile
import unittest
from pathlib import Path
import numpy as np
import torch
from simpleflight_lab import ppo
from rm27_asm2.continuation_auxiliary import install_auxiliary


class TestAuxiliary(unittest.TestCase):
    def test_supervised_gradient_and_resume(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'labels.npz'
            np.savez(path, observations=np.random.default_rng(1).normal(size=(12,45)).astype('float32'),
                     labels=np.arange(12)%2)
            obs = {'policy':torch.randn(1,45), 'critic':torch.randn(1,45)}
            with install_auxiliary(path):
                learner=ppo.PPO(obs,.16)
                before={k:v.detach().clone() for k,v in learner.actor.state_dict().items()}
                for key,value in learner.rollout.items():
                    value.zero_()
                learner.rollout['policy'].normal_()
                learner.rollout['critic'].normal_()
                learner.rollout['advantage']=torch.zeros(64,1)
                learner.rollout['return']=torch.zeros(64,1,1)
                learner.update()
                self.assertFalse(torch.equal(before['mean.1.weight'],learner.actor.mean[1].weight))
                self.assertTrue(torch.equal(before['mean.9.weight'],learner.actor.mean[9].weight))
                self.assertTrue(torch.equal(before['log_std'],learner.actor.log_std))
                saved=learner.actor_opt.state_dict()
                restored=ppo.PPO(obs,.16)
                restored.actor_opt.load_state_dict(saved)
                self.assertEqual(restored.auxiliary_updates,64)
                for key,value in learner.success_head.state_dict().items():
                    torch.testing.assert_close(value,restored.success_head.state_dict()[key])

if __name__=='__main__':unittest.main()
