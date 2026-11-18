"""Isolated R9 candidate. No launcher, simulator or training authorization."""
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np
import math
import hashlib
from pathlib import Path


def coefficient(transitions, initial=.01, decay_transitions=393216):
    if (not all(math.isfinite(x) for x in (transitions, initial, decay_transitions))
            or transitions < 0 or int(transitions) != transitions or initial < 0 or decay_transitions <= 0):
        raise ValueError('Invalid schedule')
    return initial * max(0., 1.-transitions/decay_transitions)


@contextmanager
def install_regularizer(dataset, *, completed_transitions=0, initial=.01, decay_transitions=393216, resume_state=None, diagnostics_callback=None):
    import torch
    from simpleflight_lab import ppo
    from .demonstration_learning import weights
    coefficient(completed_transitions, initial, decay_transitions)
    dataset_sha256 = hashlib.sha256(Path(dataset).read_bytes()).hexdigest()
    if completed_transitions and resume_state is None:
        raise ValueError('R9 resume requires saved regularizer metadata')
    original = ppo.PPO
    original_save = torch.save
    live = None

    class RegularizedPPO(original):
        def __init__(self, *args, **kwargs):
            nonlocal live
            super().__init__(*args, **kwargs)
            data = dict(np.load(dataset, allow_pickle=False))
            device = next(self.actor.parameters()).device
            self.demo_x = torch.as_tensor(data['observations'], device=device)
            self.demo_y = torch.as_tensor(data['raw_actions'], device=device)
            self.demo_w = torch.as_tensor(weights(data, np.ones(len(self.demo_x), bool)), dtype=torch.float32, device=device)
            if (self.demo_x.ndim != 2 or self.demo_x.shape[1] != 45
                    or self.demo_y.shape != (len(self.demo_x), 4)
                    or not torch.isfinite(self.demo_x).all() or not torch.isfinite(self.demo_y).all()
                    or set(data['demonstration'].tolist()) != {False, True}):
                raise ValueError('Invalid demonstration dataset or missing retention class')
            self.demo_transitions = completed_transitions
            self.last_demo_loss = None
            self.last_demo_metrics = None
            if resume_state is not None:
                self.load_demonstration_state_dict(resume_state)
                if self.demo_transitions != completed_transitions:
                    raise ValueError('Workflow and regularizer resume counters differ')
            live = self

        def demonstration_state_dict(self):
            return dict(completed_transitions=self.demo_transitions, initial=initial,
                        decay_transitions=decay_transitions, dataset_sha256=dataset_sha256)

        def load_demonstration_state_dict(self, state):
            expected = self.demonstration_state_dict()
            if any(state[k] != expected[k] for k in ('initial', 'decay_transitions', 'dataset_sha256')):
                raise ValueError('Changed demonstration protocol on resume')
            coefficient(state['completed_transitions'], initial, decay_transitions)
            self.demo_transitions = state['completed_transitions']

        def update(self, before_minibatch=None):
            # Count completed rollouts, not minibatches; caller restores count from checkpoint.
            scale = coefficient(self.demo_transitions, initial, decay_transitions)
            original_clip = torch.nn.utils.clip_grad_norm_
            actor_parameters = list(self.actor.parameters())
            diagnostics = []
            def clip(parameters, max_norm, *args, **kwargs):
                parameters = list(parameters)
                if scale and len(parameters) == len(actor_parameters) and all(a is b for a,b in zip(parameters, actor_parameters)):
                    # Full current Gaussian likelihood: mean AND log_std receive gradients.
                    # tanh Jacobian is constant w.r.t. parameters for archived fixed actions.
                    loss = -(self.demo_w * ppo.log_prob(self.demo_y, self.actor(self.demo_x), self.actor.log_std)).sum()
                    if not torch.isfinite(loss):raise FloatingPointError('Nonfinite demonstration loss')
                    before = [p.grad.detach().clone() for p in parameters]
                    (scale*loss).backward()
                    diagnostics.append(torch.stack((loss.detach(),
                        torch.stack([g.square().sum() for g in before]).sum().sqrt(),
                        torch.stack([(p.grad-g).square().sum() for p,g in zip(parameters,before)]).sum().sqrt())))
                    self.last_demo_loss = float(loss.detach())
                return original_clip(parameters, max_norm, *args, **kwargs)
            with patch.object(torch.nn.utils, 'clip_grad_norm_', clip):
                result = super().update(before_minibatch)
            self.last_demo_metrics = dict(coefficient=scale, actor_minibatches=len(diagnostics),
                nll_ppo_norm_aux_norm=(torch.stack(diagnostics).mean(0).tolist() if diagnostics else None))
            self.demo_transitions += self.rollout['reward'].shape[0]*self.rollout['reward'].shape[1]
            if diagnostics_callback is not None:
                diagnostics_callback(dict(completed_transitions=self.demo_transitions, **self.last_demo_metrics))
            return result
    def save(payload, *args, **kwargs):
        if isinstance(payload, dict) and payload.get('task') == 'asm2_resumable_contact':
            if live is None or payload['completed_transitions'] != live.demo_transitions:
                raise ValueError('Workflow and regularizer checkpoint counters differ')
            payload = dict(payload, r9_regularizer=live.demonstration_state_dict(),
                           r9_diagnostics=live.last_demo_metrics)
        return original_save(payload, *args, **kwargs)
    with patch.object(ppo, 'PPO', RegularizedPPO), patch.object(torch, 'save', save):
        yield
