"""ASM2 physical-evidence contracts, without simulator startup or task qualification."""
import unittest

import numpy as np

from rm27_asm2.contacts import ContactLedger, contact_record, point_velocity, scorer_input, summarize
from rm27_asm2.native import nearest_surface_distance


def sample(**overrides):
    fields = dict(tick=1, dt=.001, group="contact_front", geom="contact_front_000", other_geom="target",
                  point=[1., 0., 0.], normal=[0., 0., 1.], impulse=[0., 0., .003], separation=-1e-5,
                  pre_position=[0., 0., 0.], pre_linear=[.1, 0., -.2], pre_angular_world=[0., 0., .4],
                  collider_paths=["/Drone/Collision/contact_front_000", "/Field/target"])
    return contact_record(**(fields | overrides))


class ContactEvidenceTest(unittest.TestCase):
    def test_point_velocity_uses_lever_arm_and_world_angular_rate(self):
        value = point_velocity(np.array([2., 3., 4.]), [.1, .2, .3], [0., 0., 2.], [3., 3., 4.])
        np.testing.assert_allclose(value, [.1, 2.2, .3])
        record = sample()
        np.testing.assert_allclose(record["relative_velocity_world_m_s"], [.1, .4, -.2])
        self.assertAlmostEqual(record["normal_approach_speed_m_s"], .2)

    def test_impulse_is_force_integrated_over_the_recorded_interval(self):
        record = sample(tick=7)
        np.testing.assert_allclose(record["force_on_drone_world_N"], [0., 0., 3.])
        self.assertAlmostEqual(record["normal_force_N"], 3.)
        self.assertAlmostEqual(record["interval_start_s"], .006)
        self.assertAlmostEqual(record["interval_end_s"], .007)
        # A ceiling pushes down: scalar compression stays positive and an
        # upward-moving aircraft still has positive inward approach speed.
        ceiling = sample(normal=[0., 0., -1.], impulse=[0., 0., -.003], pre_linear=[0., 0., .2])
        self.assertAlmostEqual(ceiling["normal_force_N"], 3.)
        self.assertAlmostEqual(ceiling["normal_approach_speed_m_s"], .2)

    def test_each_semantic_group_and_target_membership_reaches_scorer_input(self):
        for group, geom, designated in (("contact_front", "contact_front_000", True),
                                         ("contact_rear", "contact_rear_000", True),
                                         ("body", "collision_11_3135", False)):
            with self.subTest(group=group):
                record = sample(group=group, geom=geom)
                data = scorer_input(record, {"target"})
                self.assertEqual(data["contact_group"], group)
                self.assertEqual(data["geoms"], [geom, "target"])
                self.assertEqual(data["designated"], designated)
                self.assertNotIn("accepted", data)
                self.assertNotIn("passed", data)
                self.assertEqual(data["task_qualification"], "NOT_DEFINED_FOR_ASM2")
                self.assertFalse(scorer_input(record, {"another_target"})["designated"])

    def test_zero_force_and_positive_separation_are_retained_as_evidence(self):
        record = sample(impulse=[0., 0., 0.], separation=2e-6)
        ledger = ContactLedger()
        ledger.begin(1)
        self.assertTrue(ledger.add(record))
        data = summarize(ledger.records)
        self.assertEqual(data["reported_groups"], ["contact_front"])
        self.assertEqual(data["touching_groups"], [])
        self.assertEqual(data["touching_points"], 0)
        self.assertEqual(data["positive_separation_points"], 1)
        self.assertEqual(data["zero_impulse_points"], 1)
        self.assertEqual(record["qualification"], "NOT_DEFINED_FOR_ASM2")

    def test_exact_duplicate_does_not_double_integrate_impulse(self):
        ledger = ContactLedger()
        ledger.begin(1)
        self.assertTrue(ledger.add(sample()))
        self.assertFalse(ledger.add(sample()))
        self.assertEqual(ledger.duplicates, 1)
        np.testing.assert_allclose(summarize(ledger.records)["impulse_on_drone_world_Ns"], [0., 0., .003])
        ledger.begin(2)
        self.assertTrue(ledger.add(sample(tick=2)))
        np.testing.assert_allclose(summarize(ledger.records)["impulse_on_drone_world_Ns"], [0., 0., .003])

    def test_equal_points_on_different_colliders_and_multiple_points_are_distinct(self):
        ledger = ContactLedger()
        ledger.begin(1)
        for record in (sample(), sample(group="contact_rear", geom="contact_rear_000"),
                       sample(point=[1., .001, 0.])):
            self.assertTrue(ledger.add(record))
        self.assertEqual(ledger.duplicates, 0)
        data = summarize(ledger.records)
        self.assertEqual(data["touching_groups"], ["contact_front", "contact_rear"])
        self.assertEqual(data["touching_points"], 3)
        np.testing.assert_allclose(data["impulse_on_drone_world_Ns"], [0., 0., .009])

    def test_ledger_rejects_wrong_or_repeated_tick(self):
        ledger = ContactLedger()
        ledger.begin(3)
        for tick in (0, 2, 3):
            with self.subTest(tick=tick), self.assertRaises(ValueError):
                ledger.begin(tick)
        with self.assertRaises(ValueError):
            ledger.add(sample(tick=2))

    def test_invalid_normal_nonfinite_contact_and_unknown_group_are_rejected(self):
        for change in (dict(normal=[0., 0., 0.]), dict(point=[np.nan, 0., 0.]),
                       dict(impulse=[0., np.inf, 0.]), dict(separation=np.nan),
                       dict(group="unknown"), dict(dt=0.)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                sample(**change)


class SurfaceDistanceTest(unittest.TestCase):
    def test_face_edge_and_vertex_distances(self):
        triangles = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
        for point, expected in (([.2, .2, .3], .3), ([.8, .8, .2], np.sqrt(.22)),
                                ([1.5, .1, .2], np.sqrt(.30))):
            with self.subTest(point=point):
                self.assertAlmostEqual(nearest_surface_distance(np.asarray(point), triangles), expected, places=12)

    def test_thin_triangle_distance_does_not_suffer_dot_product_cancellation(self):
        triangles = np.array([[[0., 0., 0.], [1., 0., 0.], [1., 1e-8, 0.]]])
        self.assertAlmostEqual(nearest_surface_distance(np.array([.75, .5e-8, 2e-6]), triangles), 2e-6, places=12)
        self.assertAlmostEqual(nearest_surface_distance(np.array([.5, -1e-8, 2e-6]), triangles),
                               np.hypot(1e-8, 2e-6), places=12)


if __name__ == "__main__":
    unittest.main()
