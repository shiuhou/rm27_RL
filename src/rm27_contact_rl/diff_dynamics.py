"""Differentiable, contact-free I4 proxy; MuJoCo remains the authority.

State (..., 17): world position, body-to-world wxyz quaternion, world velocity,
body angular velocity, four motor activations in nominal newtons. Actions are
physical CTBR (N, rad/s), not SB3 normalized actions. No reward or termination
is approximated here. Callers must stop at contact/safety/task events.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
import torch
from torch import Tensor


def _skew(x: Tensor) -> Tensor:
    a, b, c = x.unbind(-1)
    z = torch.zeros_like(a)
    return torch.stack((z, -c, b, c, z, -a, -b, a, z), -1).reshape(*x.shape[:-1], 3, 3)


def _multiply(q: Tensor, r: Tensor) -> Tensor:
    scalar = q[..., :1] * r[..., :1] - (q[..., 1:] * r[..., 1:]).sum(-1, keepdim=True)
    vector = (q[..., :1] * r[..., 1:] + r[..., :1] * q[..., 1:]
              + torch.linalg.cross(q[..., 1:], r[..., 1:]))
    return torch.cat((scalar, vector), -1)


def _rotate(q: Tensor, v: Tensor) -> Tensor:
    t = 2 * torch.linalg.cross(q[..., 1:], v)
    return v + q[..., :1] * t + torch.linalg.cross(q[..., 1:], t)


def _advance_quaternion(q: Tensor, omega_dt: Tensor) -> Tensor:
    half = omega_dt / 2
    square = (half * half).sum(-1, keepdim=True)
    angle = square.clamp_min(1e-12).sqrt()
    # Finite first AND second derivatives at zero angular velocity.
    scalar = torch.where(square < 1e-8, 1 - square / 2 + square.square() / 24, angle.cos())
    scale = torch.where(square < 1e-8, 1 - square / 6 + square.square() / 120, angle.sin() / angle)
    result = _multiply(q, torch.cat((scalar, half * scale), -1))
    return result / torch.linalg.vector_norm(result, dim=-1, keepdim=True)


@dataclass(frozen=True)
class DynamicsParameters:
    """Snapshot of compiled physical parameters, separate from controller references.

    Re-extract after changing domain-randomization scales. Parameters are fixed
    tensors, but state/action computations retain the autograd graph.
    """

    dt: float
    substeps: int
    mass: Tensor
    inertia: Tensor
    controller_inertia: Tensor
    inverse_mixer: Tensor
    rate_gain: float
    maximum: float
    force_map: Tensor
    torque_map: Tensor
    motor_alpha: Tensor
    gravity: Tensor
    action_low: Tensor
    action_high: Tensor

    @classmethod
    def from_env(cls, env, *, dtype=torch.float64, device="cpu"):
        """Accept only the audited MuJoCo 3.12.0 standalone I4 free-body model.

        Geometry stays in the authoritative model; it is deliberately not copied
        into this free-flight surrogate. Unsupported dynamics fail explicitly.
        """
        p, m = env.plant, env.plant.model
        uid = m.body("uav").id
        sites = m.actuator_trnid[:, 0]
        # Runtime mass edits without mj_setConst leave dof_M0 stale for this
        # simple free body. Do not silently replace that inconsistent reference
        # with physically corrected dynamics or mutate the authoritative model.
        if (not np.allclose(m.dof_M0[:3], m.body_mass[uid], rtol=1e-12, atol=0)
                or not np.allclose(m.dof_M0[3:], m.body_inertia[uid], rtol=1e-12, atol=0)):
            raise ValueError("Compiled mass cache differs from body mass/inertia; unsupported runtime scaling")
        checks = [
            mujoco.__version__ == "3.12.0",
            (m.nq, m.nv, m.nu, m.na, m.njnt) == (7, 6, 4, 4, 1),
            m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE,
            m.jnt_bodyid[0] == uid, m.body_parentid[uid] == 0,
            not np.any(m.body_parentid == uid),
            np.allclose(m.body_ipos[uid], 0), np.allclose(m.jnt_pos[0], 0),
            np.allclose(m.body_iquat[uid], [1, 0, 0, 0]),
            m.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
            (env.physics_substeps == 5 and np.isclose(m.opt.timestep, .001)
             and np.isclose(env.policy_dt_s, .005)),
            not np.any(m.dof_damping), not np.any(m.dof_armature),
            not np.any(m.dof_frictionloss), not np.any(m.jnt_stiffness),
            not np.any(m.body_gravcomp),
            m.neq == 0, m.ntendon == 0, m.nflex == 0,
            not m.opt.viscosity, not m.opt.density, not np.any(m.opt.wind),
            not m.opt.disableflags, not m.opt.enableflags,
            np.all(m.actuator_trntype == mujoco.mjtTrn.mjTRN_SITE),
            np.all(m.actuator_trnid[:, 1] == -1),
            np.all(m.site_bodyid[sites] == uid), np.allclose(m.site_quat[sites], [1, 0, 0, 0]),
            np.all(m.actuator_dyntype == mujoco.mjtDyn.mjDYN_FILTEREXACT),
            np.all(m.actuator_gaintype == mujoco.mjtGain.mjGAIN_FIXED),
            np.all(m.actuator_biastype == mujoco.mjtBias.mjBIAS_NONE),
            not np.any(m.actuator_actearly), not np.any(m.actuator_actlimited),
            not np.any(m.actuator_forcelimited), np.all(m.actuator_ctrllimited),
            np.allclose(m.actuator_ctrlrange, [0, p.maximum]),
            np.array_equal(m.actuator_actadr, np.arange(4)),
            np.all(m.actuator_actnum == 1), np.all(m.actuator_dynprm[:, 0] > 0),
            not np.any(p.data.qfrc_applied), not np.any(p.data.xfrc_applied),
        ]
        if not all(checks):
            raise ValueError("Unsupported dynamics for the audited I4 differentiable proxy")
        force = m.actuator_gear[:, :3] * m.actuator_gainprm[:, :1]
        torque = (np.cross(m.site_pos[sites], force)
                  + m.actuator_gear[:, 3:] * m.actuator_gainprm[:, :1])
        def tensor(value):
            return torch.tensor(np.array(value, copy=True), dtype=dtype, device=device)
        return cls(float(m.opt.timestep), env.physics_substeps, tensor(m.body_mass[uid]),
                   tensor(m.body_inertia[uid]), tensor(p.inertia), tensor(p.inverse_mixer),
                   p.rate_gain, p.maximum, tensor(force.T), tensor(torque.T),
                   tensor(-np.expm1(-m.opt.timestep / m.actuator_dynprm[:, 0])),
                   tensor(m.opt.gravity), tensor(env._action_low), tensor(env._action_high))


@dataclass(frozen=True)
class ControlStep:
    state: Tensor
    endpoints: Tensor  # (..., 5, 17), including each physical interval endpoint
    motor_commands: Tensor
    raw_motor_commands: Tensor
    clipped_action: Tensor


def step_control(state: Tensor, action: Tensor, params: DynamicsParameters) -> ControlStep:
    """One 5 ms CTBR hold, preserving all five 1 ms endpoints for later event checks.

    Inputs must share batch shape, floating dtype and device with the parameters;
    quaternion must be unit length. Clamps are exact, piecewise differentiable.
    Autograd's selected subgradient at a clamp boundary is NOT a unique derivative.
    """
    if (state.shape[-1:] != (17,) or action.shape != (*state.shape[:-1], 4)
            or state.dtype != params.mass.dtype or action.dtype != state.dtype
            or state.device != params.mass.device or action.device != state.device):
        raise ValueError("Expected matching (...,17) state and (...,4) physical CTBR tensors")
    if not bool(torch.isfinite(state).all() & torch.isfinite(action).all()):
        raise ValueError("Nonfinite dynamics input")
    norm = torch.linalg.vector_norm(state[..., 3:7], dim=-1)
    if not bool(torch.allclose(norm, torch.ones_like(norm), atol=1e-6, rtol=0)):
        raise ValueError("State quaternion must be normalized")
    clipped = torch.clamp(action, params.action_low, params.action_high)
    omega = state[..., 10:13]
    torque = (params.controller_inertia * params.rate_gain * (clipped[..., 1:] - omega)
              + torch.linalg.cross(omega, params.controller_inertia * omega))
    raw = torch.cat((clipped[..., :1], torque), -1) @ params.inverse_mixer.T
    command = raw.clamp(0, params.maximum)
    endpoints = []
    for _ in range(params.substeps):
        position, q, velocity, omega, motors = torch.split(state, (3, 4, 3, 3, 4), -1)
        force = motors @ params.force_map.T
        torque = motors @ params.torque_map.T
        velocity = velocity + params.dt * (_rotate(q, force) / params.mass + params.gravity)
        angular_momentum = params.inertia * omega
        # MuJoCo 3.12 implicitfast's local free-body solve includes the gyro
        # derivative: d(w x Iw)/dw = [w]x I - [Iw]x.
        gyro_jacobian = _skew(omega) * params.inertia - _skew(angular_momentum)
        effective_inertia = torch.diag(params.inertia) + params.dt * gyro_jacobian
        acceleration = torch.linalg.solve(
            effective_inertia, (torque - torch.linalg.cross(omega, angular_momentum)).unsqueeze(-1)
        ).squeeze(-1)
        omega = omega + params.dt * acceleration
        position = position + params.dt * velocity
        q = _advance_quaternion(q, params.dt * omega)
        # actearly=false: forces above used OLD activation, not this new value.
        motors = motors + params.motor_alpha * (command - motors)
        state = torch.cat((position, q, velocity, omega, motors), -1)
        endpoints.append(state)
    return ControlStep(state, torch.stack(endpoints, -2), command, raw, clipped)
