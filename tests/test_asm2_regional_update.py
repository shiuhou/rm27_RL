import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from rm27_asm2.regional_update import regional_weights, weighted_update, install_regional_update
from rm27_asm2.terminal_curriculum import BANDS

try:
    import torch
except ModuleNotFoundError:
    torch = None

TARGET = dict(terminal=.1, braking=.45, rotation_late=.2, rotation_bridge=.25)


class RegionalWeightTests(unittest.TestCase):
    def test_long_episodes_do_not_dominate_weighted_mass(self):
        regions = np.repeat([BANDS.index(b) for b in TARGET], [5, 20, 15, 60])
        weights = regional_weights(regions, TARGET)
        self.assertAlmostEqual(weights.mean(), 1.)
        for band, mass in TARGET.items():
            self.assertAlmostEqual(weights[regions == BANDS.index(band)].sum()/len(regions), mass)

    def test_missing_region_renormalizes_without_fabrication(self):
        regions = np.array([[BANDS.index('braking')]*3, [BANDS.index('rotation_bridge')]*3])
        weights = regional_weights(regions, TARGET)
        self.assertAlmostEqual(weights.mean(), 1.)
        self.assertAlmostEqual(weights[0].sum()/weights.sum(), .45/.70)
        for bad in ({**TARGET, 'handover': .1}, {'braking': float('nan')}, {'braking': -1.}):
            with self.assertRaises(ValueError):
                regional_weights(regions, bad)
        with self.assertRaises(ValueError):
            regional_weights(np.array([BANDS.index('handover')]), TARGET)


@unittest.skipIf(torch is None, 'Requires actual PyTorch PPO')
class RegionalUpdateTests(unittest.TestCase):
    def test_original_and_all_one_weight_updates_match(self):
        from simpleflight_lab import ppo
        torch.set_num_threads(1)
        with torch.random.fork_rng():
            torch.manual_seed(7)
            obs = dict(policy=torch.randn(16, 5), critic=torch.randn(16, 7))
            original = ppo.PPO(obs, .2)
            candidate = ppo.PPO(obs, .2)
            for name in ('actor', 'critic', 'normalizer', 'actor_opt', 'critic_opt'):
                getattr(candidate, name).load_state_dict(getattr(original, name).state_dict())
            batch = original.rollout
            for key in ('policy', 'critic', 'action', 'value', 'log_prob', 'reward'):
                batch[key].normal_(0, .1)
            batch['done'].zero_()
            batch['advantage'] = torch.randn(64, 16)
            batch['return'] = torch.randn(64, 16, 1)
            with torch.no_grad():
                batch['log_prob'].copy_(ppo.log_prob(batch['action'], original.actor(batch['policy']), original.actor.log_std))
            candidate.rollout = {k: v.clone() for k, v in batch.items()}
            candidate.rollout['update_weight'] = torch.ones(64, 16)
            state = torch.get_rng_state()
            expected = original.update()
            final_state = torch.get_rng_state()
            torch.set_rng_state(state)
            actual = weighted_update(candidate)
            torch.testing.assert_close(actual, expected, rtol=0, atol=0)
            torch.testing.assert_close(torch.get_rng_state(), final_state, rtol=0, atol=0)
            for name in ('actor', 'critic'):
                for k, v in getattr(original, name).state_dict().items():
                    torch.testing.assert_close(getattr(candidate, name).state_dict()[k], v, rtol=0, atol=0)

    def test_control_collection_and_update_parity_and_weighted_effect(self):
        from simpleflight_lab import ppo
        from rm27_asm2.rotation_bridge_metrics import install_metrics
        torch.set_num_threads(1)

        class Env:
            def __init__(self):
                self.active_band = torch.tensor([BANDS.index('braking')]*4 + [BANDS.index('rotation_bridge')]*12)
                self.step_error_m = torch.zeros(16)
                self.tick = 0

            def observation(self):
                obs = torch.arange(80).reshape(16, 5).float()/80 + self.tick*.001
                return dict(policy=obs, critic=obs)

            def step(self, action):
                self.tick += 1
                done = torch.full((16,), self.tick % 17 == 0)
                return self.observation(), action.tanh().sum(-1), done, torch.zeros_like(done), {}

        with TemporaryDirectory() as tmp, torch.random.fork_rng():
            def execute(mode):
                torch.manual_seed(19)
                env = Env()
                model = ppo.PPO(env.observation(), .2)
                model.collect(env, env.observation())
                metrics = model.update()
                if mode is not None:
                    (Path(tmp)/'learning.jsonl').write_text(json.dumps(
                        dict(transitions=1024, completed_transitions=1024, updates=1))+'\n')
                return model, metrics, torch.get_rng_state()

            original, expected, rng = execute(None)
            with install_metrics(Path(tmp)), install_regional_update(Path(tmp), TARGET, balanced=False):
                control, observed, control_rng = execute(False)
            torch.testing.assert_close(observed, expected, rtol=0, atol=0)
            torch.testing.assert_close(control_rng, rng, rtol=0, atol=0)
            for name in ('actor', 'critic'):
                for k, v in getattr(original, name).state_dict().items():
                    torch.testing.assert_close(getattr(control, name).state_dict()[k], v, rtol=0, atol=0)
            with install_regional_update(Path(tmp), TARGET, balanced=True):
                candidate, changed, candidate_rng = execute(True)
            for key in original.rollout:
                torch.testing.assert_close(candidate.rollout[key], original.rollout[key], rtol=0, atol=0)
            torch.testing.assert_close(candidate_rng, rng, rtol=0, atol=0)
            self.assertFalse(torch.equal(changed, expected))
            self.assertTrue(any(not torch.equal(v, candidate.actor.state_dict()[k]) for k, v in original.actor.state_dict().items()))
            rows = [json.loads(s) for s in (Path(tmp)/'update-weights.jsonl').read_text().splitlines()]
            self.assertAlmostEqual(rows[-1]['regions']['braking']['loss_mass'], .45/.70)
            self.assertEqual(rows[-1]['completed_transitions'], 1024)


if __name__ == '__main__':
    unittest.main()
