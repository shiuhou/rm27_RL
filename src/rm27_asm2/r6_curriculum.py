"""Competence-gated predecessor resets; no actions or reward targets are supplied."""
from __future__ import annotations
import copy
import numpy as np
from .terminal_curriculum import ResetSampler

class FrontierSampler(ResetSampler):
    def __init__(self, bands, mode, seed, settings, metadata):
        self.meta = metadata
        self.frontier = [0]*len(metadata['chains'])
        self.recent = {}
        self.promotions = []
        super().__init__(bands, mode, seed, settings)

    def weights(self):
        weights = super().weights()
        weights[self.bands == 'rotation_bridge'] = 0.
        chains = self.meta['chains']
        # Equal source mass. Half frontier practice, half successful suffix replay.
        for i, chain in enumerate(chains):
            weights[chain[self.frontier[i]]] += .125/len(chains)
            replay = self.meta['support'][i]
            weights[replay] += .125/len(chains)/len(replay)
        if not np.isclose(weights.sum(), 1.):
            raise ValueError('R6 must preserve total reset mass')
        return weights

    def record(self, rows, successes):
        for row, success in zip(rows, successes, strict=True):
            key = str(int(row))
            history = self.recent.setdefault(key, [])
            history.append(int(success)); del history[:-8]
        for i, chain in enumerate(self.meta['chains']):
            current = chain[self.frontier[i]]
            history = self.recent.get(str(current), [])
            if len(history) == 8 and sum(history) >= 4 and self.frontier[i]+1 < len(chain):
                self.promotions.append(dict(source=i, old_row=current, successes=sum(history), trials=8))
                self.frontier[i] += 1
                # Previously collected evidence cannot promote an unpractised predecessor.
                self.recent.pop(str(chain[self.frontier[i]]), None)

    def state_dict(self):
        return dict(super().state_dict(), r6=dict(frontier=self.frontier, recent=self.recent,
                    promotions=self.promotions, metadata=self.meta))

    def load_state_dict(self, state):
        r6 = copy.deepcopy(state['r6'])
        if r6['metadata'] != self.meta: raise ValueError('R6 reset provenance changed')
        self.frontier = list(r6['frontier']); self.recent = dict(r6['recent'])
        self.promotions = list(r6['promotions'])
        super().load_state_dict(state)
