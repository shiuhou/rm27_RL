"""Regression checks for event ordering and contact qualification gates."""

from dataclasses import replace
import unittest

import numpy as np

from rm27_asm2.contact_task import ContactScorer, load_contact_spec
from rm27_asm2.contacts import contact_record


class ContactTaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = load_contact_spec()
        cls.spec = replace(spec, target_rotation=np.eye(3), legal_targets=frozenset({"target"}))
        cls.state = np.r_[0., 0., -spec.patch_face_z_m, 1., np.zeros(9)]
        cls.state[9] = -.1

    def sample(self, tick, *, group="contact_front", target="target", distance=-1e-6, load=1e-5):
        return contact_record(tick=tick, dt=self.spec.dt, group=group, geom=group + "_000", other_geom=target,
                              point=[0., 0., 0.], normal=[0., 0., 1.], impulse=[0., 0., load], separation=distance,
                              pre_position=self.state[:3], pre_linear=self.state[7:10], pre_angular_world=[0., 0., 0.])

    def test_positive_separation_and_later_different_pair_do_not_qualify(self):
        scorer = ContactScorer(self.spec)
        self.assertIsNone(scorer.observe([self.sample(1, distance=1e-6)], self.state, 1))
        self.assertIsNone(scorer.observe([self.sample(2, load=0.)], self.state, 2))
        self.assertIsNone(scorer.observe([self.sample(3, group="contact_rear")], self.state, 3))
        result = scorer.observe([self.sample(4)], self.state, 4)
        self.assertTrue(result["qualified"])
        self.assertEqual(result["first_touch"]["tick"], 2)
        self.assertEqual(result["first_loaded"]["tick"], 4)
        self.assertEqual(result["positive_separation_compressive_records"], 1)

    def test_simultaneous_unloaded_body_touch_cannot_be_rescued(self):
        scorer = ContactScorer(self.spec)
        result = scorer.observe([self.sample(1), self.sample(1, group="body", load=0.)], self.state, 1)
        self.assertEqual(result["reason"], "FORBIDDEN_AIRFRAME_FIRST_TOUCH")
        self.assertEqual(scorer.observe([self.sample(2)], self.state, 2), result)

    def test_preimpact_attitude_controls_alignment_gate(self):
        scorer = ContactScorer(self.spec)
        state = self.state.copy()
        half_angle = np.deg2rad(9.) / 2.
        state[3:7] = [np.cos(half_angle), 0., np.sin(half_angle), 0.]
        result = scorer.observe([self.sample(1)], state, 1)
        self.assertEqual(result["reason"], "CONTACT_ALIGNMENT")
        self.assertAlmostEqual(result["first_touch"]["records"][0]["body_to_target_alignment_deg"], 9.)


if __name__ == "__main__":
    unittest.main()
