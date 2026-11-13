"""Potential reward boundaries and full-state features, without new rollouts."""
from types import SimpleNamespace
import unittest
import numpy as np

from rm27_asm2.braking_potential import BrakingPotential, shaping_difference
from rm27_asm2.flight import load_profile


class PotentialTests(unittest.TestCase):
    def test_discounted_telescoping_and_timeout_reset_isolation(self):
        p=np.array([.1,.4,.3,.49]); gamma=.995
        for terminated, truncated in [(True,False),(False,True)]:
            reward=shaping_difference(p[:-1],p[1:], [False,False,terminated], [False,False,truncated],gamma)
            self.assertAlmostEqual(float(reward @ gamma**np.arange(3)),-p[0],places=14)
            # Even an auto-reset state's nonzero potential must never leak in.
            self.assertEqual(reward[-1],-p[-2])
        continuing=shaping_difference(p[:-1],p[1:],False,False,gamma)
        self.assertAlmostEqual(float(continuing @ gamma**np.arange(3)),gamma**3*p[-1]-p[0],places=14)
        self.assertLess(float(shaping_difference(.3,.3,False,False,gamma)),0)

    def candidate(self):
        profile=load_profile()
        spec=SimpleNamespace(target_normal=np.array([0.,0.,1.]), target_center=np.zeros(3),
            target_rotation=np.eye(3),field_geometry={'selection':dict(
                safety_lower_target_frame_m=[-1,-1,-1],safety_upper_target_frame_m=[1,1,1])})
        x=np.zeros((3,13)); x[:,3]=1; x[:,:3]=[[0,0,.1],[.02,0,.12],[-.01,.01,.15]]
        x[:,7]=[0,.1,.2]; x[:,11]=[0,.1,.2]; x[1,3:7]=[np.cos(.05),0,np.sin(.05),0]
        f=np.tile(profile.hover_motor_thrust_N,(3,1))
        p=BrakingPotential(x,f,[.1,.2,.3],profile,spec,np.array([[0,0,-.02],[.1,0,0]]),0,
             dict(max_com_speed_m_s=2,max_body_rate_rad_s=6,max_tilt_deg=60))
        return p,x,f

    def test_full_state_and_conservative_clearance(self):
        p,x,f=self.candidate(); value=p.evaluate(x,f)[0]
        self.assertTrue(np.all((value>0)&(value<=.5)))
        faster=x[:1].copy();faster[0,7]=1.5
        self.assertLess(p.evaluate(faster,f[:1])[0][0],value[0])
        motor=f[:1].copy();motor[0,0]+=.2
        self.assertLess(p.evaluate(x[:1],motor)[0][0],value[0])
        x[0,2]=.01
        self.assertAlmostEqual(p.clearance(x[:1])[0],-.01)
        self.assertEqual(p.evaluate(x[:1],f[:1])[0][0],0)
        x[1,7]=2.1
        self.assertEqual(p.evaluate(x[1:2],f[1:2])[0][0],0)

    def test_quaternion_sign_and_targets_policy_boundary(self):
        p,x,f=self.candidate(); a=p.evaluate(x,f)[0]
        x[:,3:7]*=-1
        np.testing.assert_array_equal(p.evaluate(x,f)[0],a)
        # Existing allocator depends on the next CTBR and body rates, not old
        # held targets. The actual motor state remains in the potential above.
        from rm27_asm2.flight import allocate_ctbr
        command=np.tile([p.profile.hover_thrust_N,0,0,0],(3,1))
        target,sat=allocate_ctbr(p.profile,command,x[:,10:13])
        self.assertEqual(target.shape,(3,4))
        self.assertFalse(sat.any())


if __name__=='__main__':
    unittest.main()
