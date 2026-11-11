"""ASM2 terminal plant and signed first-contact feedback for the bounded pilot."""

from __future__ import annotations

import math
import numpy as np

from .asset import digest, read_json
from .contact_isaac import EVIDENCE, PROXY_MANIFEST
from .isaac import ASM2Sim
from .proxy import load_proxy


def rotation(q):
    """Body-to-world matrix for batched wxyz quaternions."""
    import torch

    w, x, y, z = q.unbind(dim=-1)
    return torch.stack((
        1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y),
    ), dim=-1).reshape(-1, 3, 3)


class TerminalSim(ASM2Sim):
    """Keep pose, rates, filtered thrust and held targets on the PhysX device."""

    def __init__(self, worlds, layout, profile, spec, *, diagnostic=False, physics_dt=None,
                 contact_offset_m=None, detailed=False, asset_override=None):
        if (detailed or asset_override is not None) and not diagnostic:
            raise ValueError("Alternative collision assets require matched diagnostic mode")
        if detailed and asset_override is not None:
            raise ValueError("Select a single alternative collision asset")
        if not detailed and asset_override is None:
            baseline = read_json(EVIDENCE / "isaac-proxy-field.json")
            manifest = digest(PROXY_MANIFEST)
            asset = EVIDENCE / f"proxy-{manifest[:12]}.usda"
            if (baseline["proxy"]["manifest_sha256"] != manifest
                    or baseline["contact_config_sha256"] != spec.contract_sha256
                    or baseline["asset_sha256"] != digest(asset)):
                raise ValueError("Phase 2 proxy or contact contract changed")
            asset_override = (asset, load_proxy()["entries"])
        self.signed = {}
        self.callback_active = False
        self.diagnostic = diagnostic
        dt = spec.config["comparison_dt_s"] if physics_dt is None else float(physics_dt)
        if (not 0. < dt <= spec.config["comparison_dt_s"]
                or not math.isclose(profile.policy_dt_s/dt, round(profile.policy_dt_s/dt), abs_tol=1e-10, rel_tol=0.)
                or not math.isclose(1./dt, round(1./dt), abs_tol=1e-7, rel_tol=0.)):
            raise ValueError("Contact physics must exactly divide the control interval and use integer Hz")
        super().__init__(worlds, "candidate", layout, field=True, export_recipe=False,
                         asset_override=asset_override, physics_dt=dt,
                         contact_offset_m=contact_offset_m)
        if self.field_registered != spec.legal_targets or not math.isclose(self.sim.get_physics_dt(), dt, abs_tol=1e-10):
            raise ValueError("Live Isaac Field or physics clock differs from the experiment")
        self.profile, self.spec = profile, spec
        self.substeps = round(profile.policy_dt_s/self.dt)
        self.device = self.torch.device("cuda:0")
        self.order_ids = self.torch.as_tensor(self.order, device=self.view_device, dtype=self.torch.long)
        self.to_view_order = self.order_ids.argsort().to(self.device)
        self.force_ids = self.indices.int()
        self.grid = self.tensor(self.origins)
        self.axes = self.tensor(profile.axes)
        self.mixer = self.tensor(profile.mixer)
        self.inverse_mixer = self.tensor(profile.inverse_mixer)
        self.inertia = self.tensor(profile.inertia_kg_m2)
        self.gravity_force = self.tensor([0., 0., -profile.hover_thrust_N])
        self.thrust = self.torch.empty((worlds, 4), device=self.device)
        self.targets = self.torch.empty_like(self.thrust)
        self.alpha = -math.expm1(-self.dt/profile.motor_time_constant_s)
        self.pending = {}
        self.closed = set()
        self.events = {}
        self.event_speed = {}
        self.details = {}
        self.limits = spec.config["thresholds"]
        self.cos_align = math.cos(math.radians(self.limits["alignment_max_deg"]))

    def tensor(self, array):
        return self.torch.as_tensor(array, dtype=self.torch.float32, device=self.device)

    def observe(self):
        transform = self.view.get_transforms()[self.order_ids].to(self.device)
        velocity = self.view.get_velocities()[self.order_ids].to(self.device)
        q = transform[:, (6, 3, 4, 5)]
        R = rotation(q)
        rates = (R.transpose(1, 2) @ velocity[:, 3:, None]).squeeze(-1)
        return self.torch.cat((transform[:, :3]-self.grid, q, velocity[:, :3], rates), dim=1), R

    def restore(self, ids, state, thrust, targets):
        """Restore at a control boundary before the first new CTBR allocation."""
        poses, velocity = self.view.get_transforms(), self.view.get_velocities()
        indices = self.order_ids[ids.to(self.view_device)]
        pose = self.torch.cat((state[:, :3]+self.grid[ids], state[:, (4, 5, 6, 3)]), dim=1)
        omega_world = (rotation(state[:, 3:7]) @ state[:, 10:13, None]).squeeze(-1)
        poses[indices] = pose.to(self.view_device)
        velocity[indices] = self.torch.cat((state[:, 7:10], omega_world), dim=1).to(self.view_device)
        self.view.set_transforms(poses, indices.int())
        self.view.set_velocities(velocity, indices.int())
        self.thrust[ids], self.targets[ids] = thrust, targets
        if self.pending or self.closed:
            for i in ids.tolist():
                self.pending.pop(i, None)
                self.closed.discard(i)
                self.events.pop(i, None)
                self.event_speed.pop(i, None)
                self.details.pop(i, None)

    def set_commands(self, commands, body_rates):
        profile = self.profile
        collective = commands[:, :1].clamp(0., 4.*profile.max_rotor_thrust_N)
        requested = commands[:, 1:].clamp(-profile.native_rate_limit_rad_s, profile.native_rate_limit_rad_s)
        momentum = body_rates @ self.inertia.T
        torque = (profile.rate_gain_per_s*(requested-body_rates)) @ self.inertia.T
        torque = torque + self.torch.linalg.cross(body_rates, momentum)
        raw = self.torch.cat((collective, torque), dim=1) @ self.inverse_mixer.T
        self.targets.copy_(raw.clamp(0., profile.max_rotor_thrust_N))
        return ((raw < 0.) | (raw > profile.max_rotor_thrust_N)).any(dim=1)

    def _contacts(self, headers, points):
        if not self.callback_active:
            return
        try:
            for header in headers:
                if not header.num_contact_data:
                    continue
                a, b = self.decode(header.collider0), self.decode(header.collider1)
                if a[0] != b[0] or (a[2] is None) == (b[2] is None):
                    raise ValueError(f"Unexpected contact pair {a}/{b}")
                air, field, sign = (a, b, 1.) if a[2] is not None else (b, a, -1.)
                if air[0] in self.closed:
                    continue
                for i in range(header.contact_data_offset, header.contact_data_offset+header.num_contact_data):
                    p = points[i]
                    separation = float(p.separation)
                    if separation > 0.:
                        continue
                    normal = tuple(sign*c for c in p.normal)
                    self.signed.setdefault(air[0], set()).add((air[1], field[1], air[2],
                                                              separation, tuple(p.position), normal))
        except Exception as error:
            self.callback_error = error

    def physics_step(self, state, R):
        force = (R @ (self.thrust @ self.axes)[:, :, None]).squeeze(-1) + self.gravity_force
        torque = (R @ (self.thrust @ self.mixer[1:].T)[:, :, None]).squeeze(-1)
        self.view.apply_forces_and_torques_at_position(force_data=force[self.to_view_order].to(self.view_device),
                                                       torque_data=torque[self.to_view_order].to(self.view_device), position_data=None,
                                                       indices=self.force_ids, is_global=True)
        self.thrust.add_(self.alpha*(self.targets-self.thrust))
        self.signed.clear()
        self.tick += 1
        self.callback_active = True
        try:
            self.sim.step(render=False)
        finally:
            self.callback_active = False
        if self.callback_error:
            raise RuntimeError("Could not decode PhysX contact pairs") from self.callback_error
        after, after_R = self.observe()
        if self.signed or self.pending or self.diagnostic:
            # Net contact momentum from the independent state update, with the
            # applied external force removed. PhysX point impulse is never read.
            dp = self.profile.mass_kg*(after[:, 7:10]-state[:, 7:10]) - self.dt*force
            if self.diagnostic:
                self.latest_normal_dp = dp @ self.tensor(self.spec.target_normal)
            if self.signed or self.pending:
                worlds = sorted(set(self.signed) | set(self.pending))
                self._finish_tick(worlds, state[worlds].cpu().numpy(), R[worlds].cpu().numpy(), dp[worlds].cpu().numpy())
        return after, after_R

    def _finish_tick(self, worlds, states, rotations, momentum):
        limits = self.limits
        for world, state, R, dp in zip(worlds, states, rotations, momentum):
            rows = sorted(self.signed.get(world, ()))
            pairs = {(a, b) for a, b, *_ in rows}
            previous = self.pending.get(world)
            reason = None
            point_speed, tangent, alignment = None, None, None
            if any(group == "body" for _, _, group, *_ in rows):
                reason = "FORBIDDEN_AIRFRAME_FIRST_TOUCH"
            elif any(target not in self.field_registered for _, target, *_ in rows):
                reason = "FORBIDDEN_TARGET_FIRST_TOUCH"
            elif rows:
                positions = np.array([r[4] for r in rows])-self.origins[world]
                normals = np.array([r[5] for r in rows])
                relative = state[7:10]+np.cross(R @ state[10:13], positions-state[:3])
                point_speed = -np.einsum("ij,ij->i", normals, relative)
                tangent = np.linalg.norm(relative+point_speed[:, None]*normals, axis=1)
                target = (positions-self.spec.target_center) @ self.spec.target_rotation
                alignment = min(np.min(normals @ self.spec.target_normal), np.min(normals @ R[:, 2]),
                                self.spec.target_normal @ R[:, 2])
                if previous is None:
                    if np.any(np.abs(target[:, :2]) > self.spec.region_halfsize_m) or np.any(
                            np.abs(target[:, 2]) > limits["target_plane_error_max_m"]):
                        reason = "OUTSIDE_REGISTERED_FOOTPRINT"
                    elif np.any((point_speed < limits["normal_speed_min_m_s"])
                                | (point_speed > limits["normal_speed_max_m_s"])
                                | (tangent > limits["tangent_speed_max_m_s"])):
                        reason = "CONTACT_POINT_SPEED"
                    elif alignment < self.cos_align:
                        reason = "CONTACT_ALIGNMENT"
                    elif np.linalg.norm(state[10:13]) > limits["body_rate_max_rad_s"]:
                        reason = "CONTACT_BODY_RATE"
                if reason is None and -min(r[3] for r in rows) > limits["penetration_max_m"]:
                    reason = "CONTACT_PENETRATION"
            if previous is None and rows:
                previous = dict(tick=self.tick, pairs=pairs)
                if point_speed is not None:
                    previous["max_normal_speed"] = float(point_speed.max())
                if self.diagnostic:
                    previous.update(groups=sorted({r[2] for r in rows}), first_pairs=sorted(pairs),
                                    signed_points=len(rows), separation_m=min(r[3] for r in rows),
                                    normal_speed_m_s=None if point_speed is None else [float(point_speed.min()), float(point_speed.max())],
                                    tangent_speed_max_m_s=None if tangent is None else float(tangent.max()),
                                    alignment_deg=None if alignment is None else float(np.degrees(np.arccos(np.clip(alignment, -1., 1.)))))
                self.pending[world] = previous
            if previous is None:
                continue
            normal_dp = float(dp @ self.spec.target_normal)
            if reason is None and pairs & previous["pairs"] and normal_dp >= limits["normal_impulse_min_N_s"]:
                reason = "DESIGNATED_LOADED"
            if reason is None and (self.tick-previous["tick"])*self.dt >= limits["load_delay_max_s"]:
                reason = "UNLOADED_FIRST_TOUCH"
            if reason is not None:
                self.events[world] = reason
                if reason == "DESIGNATED_LOADED":
                    self.event_speed[world] = previous["max_normal_speed"]
                self.closed.add(world)
                del self.pending[world]
                if self.diagnostic:
                    self.details[world] = {key: value for key, value in previous.items() if key != "pairs"}
                    self.details[world].update(reason=reason, end_tick=self.tick, measured_normal_momentum_Ns=normal_dp)
