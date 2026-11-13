"""R4 diagnostic candidate; empirical support is not a reachable-set certificate.

No baseline learner or scorer imports this module. Targets are preserved with
the observations, but at a policy boundary CTBR immediately overwrites them.
Actual filtered rotor thrust is included through lag-impulse distance blocks.
"""
from __future__ import annotations

import numpy as np

from .flight_suite import rotation_matrix


def shaping_difference(before, after, terminated, truncated, gamma=.995):
    """Episodic timeouts are failures here; collector chunk ends are neither flag.

    Call with the physical successor before auto-reset. For either absorbing
    boundary the successor potential is zero, never the next reset's potential.
    """
    done = np.asarray(terminated) | np.asarray(truncated)
    return gamma * np.where(done, 0., after) - before


def rms_pair_scale(values):
    values = np.asarray(values)
    # Mean squared distance between all ordered pairs, including the diagonal.
    scale = np.sqrt(2 * np.sum(np.var(values, axis=0)))
    if not np.isfinite(scale) or scale <= 1e-12:
        raise ValueError("Empirical support has a degenerate feature block")
    return float(scale)


class BrakingPotential:
    """One bounded kernel, with scales fixed by qualified nominal states only.

    The seven blocks are position, velocity, attitude chord, body rates,
    translational/angular motor relaxation impulses, and body-plane clearance.
    Impulses are *distance features*, not predictions of the next command.
    The body envelope contains every detailed hull; positive plane separation
    certifies separation from the finite registered target but is not its exact
    Euclidean distance. A nonpositive bound is conservative exclusion only.
    """

    def __init__(self, states, motors, end_s, profile, spec, vertices, target_support,
                 limits, gamma=.995, bound=.5):
        self.profile, self.spec, self.limits = profile, spec, limits
        self.vertices, self.target_support = vertices, target_support
        self.gamma, self.bound = gamma, bound
        states, motors = np.asarray(states), np.asarray(motors)
        r = rotation_matrix(states[:, 3:7])
        self.scales = dict(position=rms_pair_scale(states[:, :3]),
                           velocity=rms_pair_scale(states[:, 7:10]),
                           attitude=rms_pair_scale(r.reshape(-1, 9)/np.sqrt(2)),
                           rates=rms_pair_scale(states[:, 10:13]))
        self.nodes = self.features(states, motors)
        self.weights = gamma ** (np.asarray(end_s)/profile.policy_dt_s)

    def clearance(self, states):
        states = np.atleast_2d(states)
        body_normal = np.einsum('nji,j->ni', rotation_matrix(states[:, 3:7]),
                                self.spec.target_normal)
        return (states[:, :3] @ self.spec.target_normal
                + np.min(body_normal @ self.vertices.T, axis=1) - self.target_support)

    def features(self, states, motors):
        states, motors = np.atleast_2d(states), np.atleast_2d(motors)
        r, p, s = rotation_matrix(states[:, 3:7]), self.profile, self.scales
        df = motors - p.hover_motor_thrust_N
        linear = np.einsum('nij,nj->ni', r, df @ p.axes) * p.motor_time_constant_s / p.mass_kg
        angular = (df @ p.mixer[1:].T) @ np.linalg.inv(p.inertia_kg_m2).T * p.motor_time_constant_s
        return [states[:, :3]/s['position'], states[:, 7:10]/s['velocity'],
                r.reshape(-1, 9)/(np.sqrt(2)*s['attitude']), states[:, 10:13]/s['rates'],
                linear/s['velocity'], angular/s['rates'],
                self.clearance(states)[:, None]/s['position']]

    def valid(self, states, motors):
        states, motors = np.atleast_2d(states), np.atleast_2d(motors)
        local = (states[:, :3]-self.spec.target_center) @ self.spec.target_rotation
        bounds = self.spec.field_geometry['selection']
        tilt = np.rad2deg(np.arccos(np.clip(rotation_matrix(states[:, 3:7])[:, 2, 2], -1., 1.)))
        return (np.isfinite(states).all(1) & np.isfinite(motors).all(1)
                & (motors >= 0).all(1) & (motors <= self.profile.max_rotor_thrust_N).all(1)
                & (local >= bounds['safety_lower_target_frame_m']).all(1)
                & (local <= bounds['safety_upper_target_frame_m']).all(1)
                & (np.linalg.norm(states[:, 7:10], axis=1) <= self.limits['max_com_speed_m_s'])
                & (np.linalg.norm(states[:, 10:13], axis=1) <= self.limits['max_body_rate_rad_s'])
                & (tilt <= self.limits['max_tilt_deg']) & (self.clearance(states) > 0))

    def evaluate(self, states, motors):
        features = self.features(states, motors)
        blocks = np.stack([np.sum((x[:, None]-y[None])**2, axis=2)
                           for x, y in zip(features, self.nodes)], axis=-1)
        distance = np.sqrt(blocks.mean(-1))
        score = self.bound * self.weights[None] * np.exp(-distance)
        best = np.argmax(score, axis=1)
        phi = score[np.arange(len(score)), best]
        phi = np.where(self.valid(states, motors), phi, 0.)
        visible_score = self.bound * self.weights[None] * np.exp(-np.sqrt(
            (blocks.sum(-1)-blocks[:, :, 4:6].sum(-1))/7))
        visible = visible_score.max(1)
        # Fraction of -log(Phi/bound) due to hidden actuator distance,
        # recomputing the nearest node after the actuator blocks are removed.
        contribution = np.log(np.maximum(visible, 1e-300)/np.maximum(phi, 1e-300))
        hidden = contribution / np.maximum(-np.log(np.maximum(phi/self.bound, 1e-300)), 1e-12)
        hidden = np.where(phi > 0, hidden, 0.)
        return phi, hidden, best
