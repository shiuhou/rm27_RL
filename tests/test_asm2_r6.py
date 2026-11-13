import unittest
import numpy as np
from rm27_asm2.r6_curriculum import FrontierSampler
class TestR6(unittest.TestCase):
    def sampler(self):
        return FrontierSampler(np.array(['terminal','braking','rotation_late']+['rotation_bridge']*5),'fixed',7,
            {'fixed_mix':dict(terminal=.1,braking=.45,rotation_late=.2,rotation_bridge=.25)},
            dict(chains=[[3,4,5]],support=[[6,7]]))
    def test_gating_and_retention(self):
        s=self.sampler();np.testing.assert_allclose(s.weights(),[.1,.45,.2,.125,0,0,.0625,.0625])
        s.record([3]*7,[True]*7);self.assertEqual(s.frontier,[0])
        s.record([3],[False]);self.assertEqual(s.frontier,[1]);self.assertEqual(s.weights()[3],0)
        s.record([4]*8,[False]*8);self.assertEqual(s.frontier,[1])
        s.record([4]*4,[True]*4);self.assertEqual(s.frontier,[2])
        self.assertAlmostEqual(s.weights().sum(),1)
    def test_resume_same_draws_and_history(self):
        a=self.sampler();a.sample(13);a.record([3]*4,[True]*4)
        b=self.sampler();b.load_state_dict(a.state_dict())
        np.testing.assert_array_equal(a.sample(90),b.sample(90))
        a.record([3]*4,[False]*4);b.record([3]*4,[False]*4)
        self.assertEqual(a.state_dict(),b.state_dict())
    def test_failure_does_not_unlock_or_change_reward(self):
        s=self.sampler();s.record([3]*50,[False]*50)
        self.assertEqual(s.frontier,[0]);self.assertEqual(s.weights()[4],0)
if __name__=='__main__':unittest.main()
