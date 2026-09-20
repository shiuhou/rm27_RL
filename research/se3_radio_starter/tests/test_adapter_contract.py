"""Interface-contract tests using fake modules, NOT physics integration tests."""
from types import SimpleNamespace
import unittest

import numpy as np

from upstream_adapter import UpstreamSimulation


class FakeState:
    def __init__(self, pos, vel, quat, omega):
        self.position, self.velocity, self.quaternion, self.omega = pos, vel, quat, omega


class FakeController:
    def control_update(self, current, desired, dt, heading):
        self.received = (current, desired, dt, heading)
        return SimpleNamespace(thrust=1.0, angular=np.array([1.0,2.0,3.0]))


class FakeMixer:
    def calculate(self, thrust, tx, ty, tz):
        self.received = (thrust, tx, ty, tz)
        return np.array([10.,12.,14.,16.])


class AdapterContractTests(unittest.TestCase):
    def make_adapter(self):
        sim = UpstreamSimulation.__new__(UpstreamSimulation)
        sim.qadr = sim.vadr = 0
        sim.data = SimpleNamespace(qpos=np.array([0.,0.,0.3,1.,0.,0.,0.]),
                                   qvel=np.array([0.1,0.2,0.3,0.4,0.5,0.6]), ctrl=np.zeros(4))
        sim.model = object()
        sim.motor_ids = np.arange(4)
        sim.State = FakeState
        sim.controller = FakeController()
        sim.mixer = FakeMixer()
        sim.mj = SimpleNamespace(mj_step=lambda model,data: None)
        return sim

    def test_units_quaternion_velocity_and_position_contract(self):
        sim = self.make_adapter()
        original_qpos = sim.data.qpos.copy()
        sim.step(np.array([0.2,0.3,0.5]), np.array([0.,1.,0.]))
        current, desired, dt, heading = sim.controller.received
        np.testing.assert_array_equal(current.quaternion, [0,0,0,1])
        np.testing.assert_array_equal(current.omega, [0.4,0.5,0.6])
        np.testing.assert_array_equal(desired.velocity, [0,0,0])
        np.testing.assert_array_equal(desired.omega, [0,0,0])
        self.assertEqual(dt, 0.001)
        self.assertAlmostEqual(sim.mixer.received[0], 0.033*9.8066)
        np.testing.assert_allclose(sim.mixer.received[1:], [0.001,0.002,0.003])
        np.testing.assert_array_equal(sim.data.qpos, original_qpos)  # no teleporting in control code

    def test_controls_are_normalized_and_clipped(self):
        sim = self.make_adapter()
        sim.mixer.calculate = lambda *args: np.array([-1.,10.,22.,100.])
        sim.step(np.array([0,0,0.3]), np.array([0.,1.,0.]))
        self.assertTrue(np.all(sim.data.ctrl >= 0))
        self.assertTrue(np.all(sim.data.ctrl <= 1))
        self.assertEqual(sim.data.ctrl[0], 0)

    def test_nonfinite_mixer_output_rejected(self):
        sim = self.make_adapter()
        sim.mixer.calculate = lambda *args: np.array([np.nan,0,0,0])
        with self.assertRaises(RuntimeError):
            sim.step(np.array([0,0,0.3]), np.array([0.,1.,0.]))

    def test_nonfinite_state_rejected(self):
        sim = self.make_adapter()
        sim.data.qpos[0] = np.nan
        with self.assertRaises(RuntimeError):
            sim.step(np.array([0,0,0.3]), np.array([0.,1.,0.]))


if __name__ == '__main__':
    unittest.main()
