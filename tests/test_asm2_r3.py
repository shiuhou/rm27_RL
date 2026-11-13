"""Exercise the actual PPO sampling and resume boundary of the one R3 change."""
import copy
import unittest

import torch

from rm27_asm2.r3_experiment import exploration_initialization
from simpleflight_lab import ppo


class ExplorationTests(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1)
        torch.manual_seed(17)
        self.obs = dict(policy=torch.randn(16,45), critic=torch.randn(16,45))
        self.actor = ppo.Actor(45, .16)

    def model(self, enabled, resume):
        actor = copy.deepcopy(self.actor)
        with exploration_initialization(enabled, resume, .64):
            return ppo.PPO(self.obs, .16, actor=actor)

    def test_only_fresh_B_scale_changes_and_density_matches(self):
        model = self.model(True, False)
        torch.testing.assert_close(model.actor(self.obs['policy']), self.actor(self.obs['policy']), rtol=0, atol=0)
        torch.testing.assert_close(model.actor.log_std.exp(), torch.full((4,),.64))
        action, density = model.actor.sample(self.obs['policy'])
        expected = torch.distributions.Normal(model.actor(self.obs['policy']), model.actor.log_std.exp()).log_prob(action).sum(-1)
        torch.testing.assert_close(density, expected)
        loss = -density.mean()
        loss.backward()
        self.assertTrue(torch.isfinite(model.actor.log_std.grad).all())
        self.assertGreater(model.actor.log_std.grad.abs().sum().item(), 0)

    def test_control_and_resume_preserve_source_actor(self):
        for enabled, resume in [(False,False),(False,True),(True,True)]:
            model = self.model(enabled,resume)
            for key, value in self.actor.state_dict().items():
                torch.testing.assert_close(model.actor.state_dict()[key],value,rtol=0,atol=0)


if __name__ == '__main__':
    unittest.main()
