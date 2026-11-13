"""G3.1 diagnostic coordinate and timing semantics, without changing the plant."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

PATH = Path(__file__).resolve().parents[1] / '.agents/runs/asm2-g31/contact_audit.py'
spec = importlib.util.spec_from_file_location('contact_audit', PATH)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class ContactAuditTest(unittest.TestCase):
    def test_normal_translation_changes_support_not_order(self):
        geometry = object.__new__(audit.Geometry)
        geometry.plant = SimpleNamespace(spec=SimpleNamespace(target_center=np.zeros(3), target_normal=np.array([0.,0.,1.])))
        geometry.vertices = np.array([[0,0,-.020], [0,0,-.019], [0,0,-.020+13e-6]])
        geometry.names = np.array(['front','rear','body'])
        geometry.groups = np.array(['contact_front','contact_rear','body'])
        state = np.r_[[0.,0.,.021], [1.,0.,0.,0.], np.zeros(6)]
        before = geometry.support(state)
        shifted = audit.perturb(state, SimpleNamespace(target_rotation=np.eye(3)), 'translation', 2, -10e-6)
        after = geometry.support(shifted)
        self.assertAlmostEqual(after['body_minus_front_m'], 13e-6, places=12)
        self.assertAlmostEqual(after['contact_front']['plane_clearance_m']-before['contact_front']['plane_clearance_m'], -10e-6, places=12)
        np.testing.assert_array_equal(shifted[3:],state[3:])

    def test_attitude_is_body_frame_and_preserves_com(self):
        state = np.r_[[1.,2.,3.], [np.sqrt(.5),0.,0.,np.sqrt(.5)], np.zeros(6)]
        result = audit.perturb(state, None, 'attitude', 0, np.pi/2)
        # Body X rotation after world Z rotation sends body Z to world X.
        np.testing.assert_allclose(audit.rotation_matrix(result[3:7])[:,2],[1,0,0],atol=1e-14)
        np.testing.assert_array_equal(result[:3],state[:3])
        np.testing.assert_array_equal(result[7:],state[7:])

    def test_only_final_command_switch_moves(self):
        commands = np.arange(42*4).reshape(42,4)
        events = audit.schedule(commands,2000,-4)
        self.assertEqual(set(events),{0,46})
        np.testing.assert_array_equal(events[0],commands[40])
        np.testing.assert_array_equal(events[46],commands[41])
        self.assertEqual(set(audit.schedule(commands,2000,4)),{0,54})

    def test_budget_and_axes(self):
        cases = audit.cases()
        self.assertEqual(len(cases),78)
        self.assertEqual(len({c['id'] for c in cases}),len(cases))
        self.assertLessEqual(len(cases)+2,300)
        self.assertEqual({c['axis'] for c in cases if c['kind']=='attitude'},{0,1,2})


if __name__ == '__main__':
    unittest.main()
