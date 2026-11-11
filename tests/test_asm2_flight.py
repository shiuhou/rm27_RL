import unittest

import numpy as np

from rm27_asm2.flight import Actuator, allocate_ctbr, load_profile


class ASM2FlightActuationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_profile()

    def test_cad_lever_arms_and_hover_trim(self):
        profile = self.profile
        np.testing.assert_allclose(profile.mixer, [
            [1.0, 1.0, 1.0, 1.0],
            [0.041, -0.041, -0.041, 0.041],
            [-0.0275, -0.0275, 0.0275, 0.0275],
            [0.018, -0.018, 0.018, -0.018],
        ], atol=1e-12, rtol=0)
        command = np.array([[0.142 * 9.81, 0.0, 0.0, 0.0]])
        motors, saturated = allocate_ctbr(profile, command, np.zeros((1, 3)))
        self.assertFalse(saturated[0])
        np.testing.assert_allclose(motors, np.full((1, 4), 0.142 * 9.81 / 4))
        np.testing.assert_allclose(profile.mixer @ motors[0], command[0], atol=1e-12)

    def test_rate_command_achieves_requested_angular_acceleration(self):
        profile = self.profile
        rates = np.array([[0.52, -0.39, 0.25], [-0.26, 0.32, -0.49]])
        target = np.array([[0.61, -0.38, 0.30], [-0.33, 0.36, -0.42]])
        command = np.column_stack((np.full(len(rates), profile.hover_thrust_N), target))
        motors, saturated = allocate_ctbr(profile, command, rates)
        self.assertFalse(saturated.any())
        inertia = profile.inertia_kg_m2
        realized_torque = (motors @ profile.mixer.T)[:, 1:]
        angular_accel = np.linalg.solve(
            inertia, (realized_torque - np.cross(rates, rates @ inertia.T)).T
        ).T
        np.testing.assert_allclose(angular_accel, 17.0 * (target - rates), atol=1e-12)

    def test_mixer_clips_only_the_saturated_environment(self):
        profile = self.profile
        command = np.array([[profile.hover_thrust_N, 0.0, 0.0, 100.0],
                            [profile.hover_thrust_N, 0.0, 0.0, 0.0]])
        motors, saturated = allocate_ctbr(profile, command, np.zeros((2, 3)))
        np.testing.assert_array_equal(saturated, [True, False])
        np.testing.assert_allclose(motors[0], [1.103625, 0.0, 1.103625, 0.0])
        np.testing.assert_allclose(motors[1], profile.hover_motor_thrust_N)

    def test_motor_applies_previous_thrust_then_filters_with_exact_lag(self):
        profile = self.profile
        actuator = Actuator(profile, 1)
        hover = profile.hover_thrust_N
        target = 1.3 * hover
        actuator.set_ctbr(np.array([[target, 0.0, 0.0, 0.0]]), np.zeros((1, 3)))
        force, torque = actuator.step_wrench()
        np.testing.assert_allclose(force, [[0.0, 0.0, hover]], atol=1e-12)
        np.testing.assert_allclose(torque, 0.0, atol=1e-12)
        for _ in range(39):
            actuator.step_wrench()
        force, _ = actuator.step_wrench()
        expected = hover + (target - hover) * (1.0 - np.exp(-1.0))
        np.testing.assert_allclose(force, [[0.0, 0.0, expected]], atol=1e-12)
        actuator.reset()
        np.testing.assert_allclose(actuator.motor_thrust_N, profile.hover_motor_thrust_N[None])
        np.testing.assert_allclose(actuator.motor_command_N, profile.hover_motor_thrust_N[None])
        self.assertFalse(actuator.mixer_saturated.any())


if __name__ == "__main__":
    unittest.main()
