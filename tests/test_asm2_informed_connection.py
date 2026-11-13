import unittest
import numpy as np
from rm27_asm2.informed_connection import decode,encode,terminal_residual,match_report
class TestConnection(unittest.TestCase):
    def test_convex_control_interpolation_preserves_actor_bounds(self):
        knots=np.random.default_rng(6).uniform(-1,1,(8,4));commands=decode(knots,171,1.39302)
        self.assertTrue(np.all(np.abs(commands[:,1:])<=2))
        self.assertTrue(np.all((commands[:,0]>=.5*1.39302)&(commands[:,0]<=1.5*1.39302)))
        np.testing.assert_allclose(encode(commands[[0,-1]],1.39302),knots[[0,-1]],atol=1e-15)
    def setup_state(self):
        x=np.zeros(13);x[3]=1;f=np.full(4,.35)
        target=dict(state=x.tolist(),motor_thrust_N=f.tolist(),motor_target_N=f.tolist())
        tol=dict(position_m=.0005,velocity_m_s=.005,attitude_rad=.01,body_rate_rad_s=.05,rotor_thrust_N=.005,held_target_N=.005)
        return x,f,target,tol
    def test_terminal_matching_includes_hidden_actuator_state(self):
        x,f,target,tol=self.setup_state();self.assertTrue(match_report(x,f,f,target,tol)['passed'])
        for motors,held in [(f+.006,f),(f,f+.006)]:self.assertFalse(match_report(x,motors,held,target,tol)['passed'])
        x[7]=.006;self.assertFalse(match_report(x,f,f,target,tol)['passed'])
    def test_quaternion_double_cover_and_rotation_error(self):
        x,f,target,tol=self.setup_state();x[3]=-1
        np.testing.assert_allclose(terminal_residual(x,f,f,target,tol),0,atol=1e-15)
        x[3:7]=[np.cos(.02),np.sin(.02),0,0]
        self.assertFalse(match_report(x,f,f,target,tol)['passed'])
if __name__=='__main__':unittest.main()
