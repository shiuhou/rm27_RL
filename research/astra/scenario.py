"""Private simulation target generator; trajectories never reach autonomy."""
import numpy as np


class Scenario:
    def __init__(self, n, seed, kind='mixed'):
        rng = np.random.default_rng(seed)
        self.kind = np.arange(n) % 4 if kind == 'mixed' else np.zeros(n, dtype=int)
        if kind not in ('mixed', 'static'):
            raise ValueError(kind)
        self.start = np.column_stack([rng.uniform(2, 8, n), rng.uniform(-.2, .2, n), rng.uniform(1.9, 2.1, n)])
        speed = rng.uniform(.2, 5., n)
        angle = rng.uniform(-np.pi, np.pi, n)
        self.velocity = np.column_stack([speed*np.cos(angle), speed*np.sin(angle), np.zeros(n)])
        self.velocity[self.kind == 0] = 0
        self.velocity[self.kind == 1, 0] = 0  # transverse crossing
        self.velocity[self.kind == 1, 1] = speed[self.kind == 1]*rng.choice([-1., 1.], (self.kind == 1).sum())
        self.velocity[self.kind == 2, 0] = speed[self.kind == 2]  # receding
        self.velocity[self.kind == 2, 1] = 0
        self.amplitude = rng.uniform(.1, .8, n)*(self.kind == 3)
        self.omega = rng.uniform(.6, 1.8, n)
        self.phase = rng.uniform(-np.pi, np.pi, n)
        self.noise_seed = seed+100000
        self.performance = rng.uniform(-1, 1, n)
        self.drift_direction = rng.normal(size=(n, 3))
        self.drift_direction[:, 2] = 0
        self.drift_direction /= np.maximum(np.linalg.norm(self.drift_direction, axis=1, keepdims=True), 1e-12)
        self.occlusion_start = rng.uniform(.5, 2., n)
        self.mass_error = rng.uniform(-1, 1, n)

    def target(self, t):
        position = self.start + t*self.velocity
        phase = self.phase+self.omega*t
        position[:, 1] += self.amplitude*(np.sin(phase)-np.sin(self.phase))
        position[:, 2] += .25*self.amplitude*(np.cos(phase)-np.cos(self.phase))
        velocity = self.velocity.copy()
        velocity[:, 1] += self.amplitude*self.omega*np.cos(phase)
        velocity[:, 2] -= .25*self.amplitude*self.omega*np.sin(phase)
        return position, velocity
