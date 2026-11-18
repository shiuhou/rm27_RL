"""Auxiliary binary supervision of the existing Actor's shared features."""
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np


@contextmanager
def install_auxiliary(dataset, coefficient=.01):
    import torch
    from simpleflight_lab import ppo
    original = ppo.PPO

    class AuxiliaryPPO(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            device = next(self.actor.parameters()).device
            # Separate RNG initialization preserves the PPO sampling stream.
            with torch.random.fork_rng(devices=[device.index] if device.type == 'cuda' else []):
                torch.manual_seed(713)
                self.success_head = torch.nn.Linear(256, 1).to(device)
            self.success_optimizer = torch.optim.Adam(self.success_head.parameters(), lr=.0005)
            with np.load(dataset) as archive:
                self.success_observations = torch.tensor(archive['observations'], device=device)
                self.success_labels = torch.tensor(archive['labels'], dtype=torch.float32, device=device)
            self.auxiliary_updates = 0
            self.last_auxiliary_loss = None
            def save_hook(optimizer, state):
                state['continuation_auxiliary'] = dict(head=self.success_head.state_dict(),
                    optimizer=self.success_optimizer.state_dict(), updates=self.auxiliary_updates)
                return state
            def load_hook(optimizer, state):
                state = dict(state)
                saved = state.pop('continuation_auxiliary', None)
                if saved is not None:
                    self.success_head.load_state_dict(saved['head'])
                    self.success_optimizer.load_state_dict(saved['optimizer'])
                    self.auxiliary_updates = saved['updates']
                return state
            self.actor_opt.register_state_dict_post_hook(save_hook)
            self.actor_opt.register_load_state_dict_pre_hook(load_hook)

        def update(self, before_minibatch=None):
            original_clip = torch.nn.utils.clip_grad_norm_
            actor_parameters = list(self.actor.parameters())
            def clip(parameters, max_norm, *args, **kwargs):
                parameters = list(parameters)
                if parameters and parameters[0] is actor_parameters[0]:
                    self.success_optimizer.zero_grad(set_to_none=True)
                    features = self.actor.mean[:-1](self.success_observations)
                    logits = self.success_head(features).squeeze(-1)
                    loss = torch.nn.functional.binary_cross_entropy_with_logits(logits, self.success_labels)
                    (coefficient * loss).backward()
                    self.last_auxiliary_loss = float(loss.detach())
                    original_clip(self.success_head.parameters(), max_norm)
                    self.success_optimizer.step()
                    self.auxiliary_updates += 1
                return original_clip(parameters, max_norm, *args, **kwargs)
            # Standard PPO computes its original surrogate and critic loss.
            # Auxiliary gradients are added before clipping and the Actor step.
            with patch.object(torch.nn.utils, 'clip_grad_norm_', clip):
                return super().update(before_minibatch)
    with patch.object(ppo, 'PPO', AuxiliaryPPO):
        yield
