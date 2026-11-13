"""Bounded pre-outcome selection and lossless observational recording."""
from types import SimpleNamespace
import unittest
import numpy as np
from rm27_asm2.backward_frontier import select_frames,Capture


class BackwardFrontierTests(unittest.TestCase):
    def test_selection_is_fixed_and_excludes_all_known_frames(self):
        self.assertEqual(select_frames(100,{98,96}),(99,94))
        with self.assertRaises(ValueError):select_frames(100,set(range(90,100)))

    def test_recorder_copies_live_memory_and_restores_methods(self):
        class Plant:
            def __init__(self):
                self.x=np.zeros(13);self.motor_thrust_N=np.zeros(4)
                self.data=SimpleNamespace(ctrl=np.zeros(4));self.tick=0;self.dt=.0001
            def state(self):return self.x
            def set_ctbr(self,cmd):self.data.ctrl[:]=cmd;return False
            def step_physics(self):
                self.tick+=1;self.x+=1;self.motor_thrust_N+=.1
                return []
        plant=Plant();step=plant.step_physics;control=plant.set_ctbr
        sample=dict(state=plant.x,motor_thrust_N=plant.motor_thrust_N,motor_target_N=plant.data.ctrl)
        with Capture(plant,sample) as c:
            plant.set_ctbr(np.ones(4));plant.step_physics();plant.step_physics()
        np.testing.assert_array_equal(c.states[0],0)
        np.testing.assert_array_equal(c.states[1],1)
        np.testing.assert_allclose(c.motors[1],.1)
        self.assertEqual(plant.step_physics,step);self.assertEqual(plant.set_ctbr,control)
        self.assertEqual(c.times,[0.,.0001,.0002])


if __name__=='__main__':unittest.main()
