"""Regression checks for the moving handover reference's physical boundary."""

import unittest

import numpy as np

from rm27_asm2.contact_task import load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.flight_suite import rotation_matrix
from rm27_asm2.handover import MovingReference, attitude_derivatives


class HandoverReferenceTest(unittest.TestCase):
    def test_pitch_reference_rate_and_acceleration(self):
        pitch, speed, acceleration = .31, -.43, .64
        z = np.array([np.sin(pitch), 0., np.cos(pitch)])
        dz_dpitch = np.array([np.cos(pitch), 0., -np.sin(pitch)])
        rate, angular_accel = attitude_derivatives(9.81*z, 9.81*speed*dz_dpitch,
                                                   9.81*(acceleration*dz_dpitch-speed**2*z))
        np.testing.assert_allclose(rate, [0., speed, 0.], atol=1e-12)
        np.testing.assert_allclose(angular_accel, [0., acceleration, 0.], atol=1e-12)

    def test_moving_boundary_keeps_live_acceleration_and_velocity(self):
        profile, spec = load_profile(), load_contact_spec()
        q = np.array([np.cos(.07), np.sin(.07), 0., 0.])
        state = np.r_[.32*spec.target_normal, q, [.08, .03, -.14], [.1, -.06, .02]]
        motors = profile.hover_motor_thrust_N*np.array([.95, .98, 1.04, 1.05])
        held_targets = motors + np.array([.01, -.005, 0., .002])
        reference = MovingReference(profile, spec, state, motors, held_targets, 1.6, .00002, .04)
        position, velocity, acceleration, jerk, _ = reference.kinematics(0.)
        np.testing.assert_array_equal(position, state[:3])
        np.testing.assert_allclose(velocity, state[7:10], atol=1e-12)
        world = rotation_matrix(q)
        force = motors @ profile.axes/profile.mass_kg
        np.testing.assert_allclose(acceleration, world @ force-[0., 0., profile.gravity_m_s2], atol=1e-12)
        force_rate = (held_targets-motors) @ profile.axes/(profile.mass_kg*profile.motor_time_constant_s)
        np.testing.assert_allclose(jerk, world @ (np.cross(state[10:13], force)+force_rate), atol=1e-12)
        _, final_v, final_a, final_j, _ = reference.kinematics(1.6)
        np.testing.assert_allclose(final_v, -.04*spec.target_normal, atol=1e-10)
        thrust = final_a+[0., 0., profile.gravity_m_s2]
        np.testing.assert_allclose(thrust/np.linalg.norm(thrust), spec.target_normal, atol=1e-10)
        np.testing.assert_allclose(final_j, 0., atol=1e-9)


if __name__ == "__main__":
    unittest.main()
