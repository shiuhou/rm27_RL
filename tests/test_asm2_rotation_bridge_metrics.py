import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from rm27_asm2.rotation_bridge_metrics import install_metrics
from rm27_asm2.terminal_curriculum import BANDS

try:
    import torch
except ModuleNotFoundError:
    torch = None


@unittest.skipIf(torch is None, 'PPO tensor tests require PyTorch')
class RotationBridgeMetricsTests(unittest.TestCase):
    def test_completed_rollout_matches_original_learner(self):
        from simpleflight_lab import ppo

        class AutoResetEnv:
            def __init__(self):
                self.active_band = torch.tensor([0, 1, 2])
                self.step_error_m = torch.zeros(3)
                self.tick = 0
                self.pre_step_bands = []

            def observation(self):
                obs = torch.eye(3) + self.tick * .01
                return dict(policy=obs, critic=obs)

            def step(self, action):
                self.pre_step_bands.append(self.active_band.clone())
                self.tick += 1
                done = (self.tick + torch.arange(3)) % 3 == 0
                self.active_band[done] = (self.active_band[done] + 1) % len(BANDS)
                reward = action.tanh().sum(dim=1)
                self.step_error_m = reward.abs()
                return self.observation(), reward, done, torch.zeros_like(done), {}

        original_ppo = ppo.PPO
        original_gae = ppo.compute_gae
        worlds = 3
        frames = ppo.ROLLOUT_STEPS * worlds
        with TemporaryDirectory() as tmp, torch.random.fork_rng():
            output = Path(tmp)
            env = AutoResetEnv()
            torch.manual_seed(81)
            baseline = ppo.PPO(env.observation(), .3)
            with torch.no_grad():
                actor_mean = baseline.actor(env.observation()['policy'])
                actor_std = baseline.actor.log_std.exp()
                mean, variance = baseline.normalizer.running_mean_var()
            obs, stats = baseline.collect(env, env.observation())
            rng = torch.get_rng_state()

            torch.manual_seed(81)
            instrumented_env = AutoResetEnv()
            with install_metrics(output, completed_transitions=frames, updates=1):
                candidate = ppo.PPO(instrumented_env.observation(), .3)
                candidate_obs, candidate_stats = candidate.collect(instrumented_env, instrumented_env.observation())
                self.assertFalse((output / 'regional-rollouts').exists())
                (output / 'learning.jsonl').write_text(json.dumps(
                    dict(transitions=3 * frames, completed_transitions=2 * frames, updates=2)) + '\n')

            self.assertIs(ppo.PPO, original_ppo)
            self.assertIs(ppo.compute_gae, original_gae)
            torch.testing.assert_close(torch.get_rng_state(), rng, rtol=0, atol=0)
            torch.testing.assert_close(candidate_stats, stats, rtol=0, atol=0)
            for key, tensor in obs.items():
                torch.testing.assert_close(candidate_obs[key], tensor, rtol=0, atol=0)
            for key, tensor in baseline.rollout.items():
                torch.testing.assert_close(candidate.rollout[key], tensor, rtol=0, atol=0)
            self.assertEqual(baseline.rollout.keys(), candidate.rollout.keys())

            batch = baseline.rollout
            values = batch['value'] * variance.sqrt() + mean
            with torch.no_grad():
                next_value = baseline.critic(obs['critic']) * variance.sqrt() + mean
            raw_gae, _ = original_gae(batch['reward'], batch['done'], values, next_value,
                                      gamma=baseline.gamma, lmbda=baseline.gae_lambda)
            first_z = ((batch['action'][0] - actor_mean) / actor_std).detach().norm(dim=1).numpy()
            path = output / 'regional-rollouts' / f'rollout-{2 * frames:06d}.npz'
            with np.load(path, allow_pickle=False) as record:
                expected_regions = torch.stack(env.pre_step_bands).numpy()
                np.testing.assert_array_equal(record['region'], expected_regions)
                np.testing.assert_array_equal(record['reward'], batch['reward'][..., 0])
                np.testing.assert_array_equal(record['critic_stored'], batch['value'][..., 0])
                np.testing.assert_array_equal(record['critic_denormalized'], values[..., 0])
                np.testing.assert_array_equal(record['gae_raw'], raw_gae[..., 0])
                np.testing.assert_array_equal(record['gae_normalized'], batch['advantage'])
                np.testing.assert_allclose(record['exploration_z_l2'][0], first_z, atol=1e-6)
                summary = json.loads(str(record['summary_json']))
                self.assertEqual(summary['completed_transitions'], 2 * frames)
                self.assertEqual(summary['charged_transitions'], 3 * frames)
                self.assertEqual(summary['updates'], 2)
                for index, name in enumerate(BANDS):
                    selected = expected_regions == index
                    self.assertEqual(summary['regions'][name]['steps'], int(selected.sum()))
                    self.assertEqual(summary['regions'][name]['ends'], int(record['done'][selected].sum()))

            with install_metrics(output, completed_transitions=2 * frames, updates=2):
                failed_env = AutoResetEnv()
                failed = ppo.PPO(failed_env.observation(), .3)
                failed.collect(failed_env, failed_env.observation())
            self.assertEqual(list(path.parent.iterdir()), [path])


if __name__ == '__main__':
    unittest.main()
