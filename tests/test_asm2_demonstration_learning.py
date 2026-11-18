import unittest
import numpy as np
from rm27_asm2.demonstration_learning import observations,invert_commands,weights

class DemonstrationLearningTests(unittest.TestCase):
    def test_actor_action_order_and_round_trip(self):
        from rm27_asm2.flight import load_profile
        p=load_profile();cmd=np.array([[p.hover_thrust_N*1.3,.3,-.9,1.8]])
        raw,normalized,clipped=invert_commands(cmd,p.hover_thrust_N,1e-6)
        np.testing.assert_allclose(np.tanh(raw),normalized,atol=1e-7)
        self.assertFalse(clipped.any())
        np.testing.assert_allclose(normalized,[[.15,-.45,.9,.6]])
        with self.assertRaises(ValueError):invert_commands(np.array([[p.hover_thrust_N,2.1,0,0]]),p.hover_thrust_N,1e-6)

    def test_native_observation_and_actor_parity(self):
        from rm27_asm2.asset import ROOT,read_json,digest
        from rm27_asm2.flight import load_profile
        from rm27_asm2.contact_task import load_contact_spec
        from rm27_asm2.terminal_native_eval import load_controller
        from simpleflight_lab.stage1_native_eval import NativeActor
        p=load_profile();s=load_contact_spec()
        task=read_json(ROOT/'configs/rm27_asm2_terminal_recovery.json')
        goal=s.target_center+(-s.patch_face_z_m-task['goal_inset_m'])*s.target_normal
        cp=ROOT/'.agents/runs/asm2-terminal/resumable-comparison/A/checkpoint-196608.pt'
        actor,_=load_controller(cp.with_suffix('.npz'),digest(cp),p,goal)
        data=np.load(ROOT/'.agents/runs/asm2-terminal/r8/demonstrations.npz')
        states=data['states'][::101]
        with np.load(cp.with_suffix('.npz')) as a:layers=NativeActor.layers(a,45)
        raw=NativeActor.mean(observations(states,goal),layers)
        cmd=np.column_stack((p.hover_thrust_N*(1+.5*np.tanh(raw[:,3])),2*np.tanh(raw[:,:3])))
        np.testing.assert_allclose(cmd,np.array([actor(v) for v in states]),atol=1e-6,rtol=0)

    def test_source_exclusion_and_trajectory_weight(self):
        from rm27_asm2.asset import ROOT
        data=dict(np.load(ROOT/'.agents/runs/asm2-terminal/r8/demonstrations.npz'))
        mask=data['source']!='stationary_2';w=weights(data,mask)
        self.assertEqual(w[~mask].sum(),0)
        self.assertAlmostEqual(w[data['demonstration']].sum(),.5)
        self.assertAlmostEqual(w[~data['demonstration']].sum(),.5)

if __name__=='__main__':unittest.main()
