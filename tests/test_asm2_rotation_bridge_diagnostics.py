from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from rm27_asm2 import rotation_bridge_diagnostics as diagnostics
from rm27_asm2.asset import digest
from rm27_asm2.rotation_bridge_metrics import SUMMARY_FIELDS
from rm27_asm2.terminal_curriculum import BANDS


class RotationBridgeDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.spec = SimpleNamespace(target_center=np.zeros(3), target_rotation=np.eye(3),
                                    target_normal=np.array([0., 0., 1.]))
        self.region = dict(columns=list(diagnostics.COLUMNS), min=[-1., -.5, .1, .2, 0.],
                           max=[-.5, .5, 1., .8, 20.])
        self.states = np.zeros((3, 13))
        self.states[:, :3] = [[-1.1, 0., .5], [-.8, 0., .5], [-.7, 0., .5]]
        self.states[:, 3] = 1.
        self.states[:, 7:10] = [.5, 0., -.5]
        self.times = np.array([0., .005, .01])
        self.outcome = dict(safety_failure=None, end_s=.01, contact={"first_touch": None})

    def evaluate_trace(self):
        return diagnostics.transition_diagnostic(self.states, self.times, self.spec, self.region, self.outcome)

    def test_new_entry_excludes_starts_already_inside_and_outward_motion(self):
        result = self.evaluate_trace()
        self.assertTrue(result["initially_outside"])
        self.assertTrue(result["newly_entered"])
        self.assertEqual(result["first_entry_s"], .005)
        self.states[0, 0] = -.9
        result = self.evaluate_trace()
        self.assertTrue(result["initially_inside"])
        self.assertFalse(result["newly_entered"])
        self.states[:, 7] = -.5
        result = self.evaluate_trace()
        self.assertTrue(result["initially_outside"])
        self.assertFalse(result["newly_entered"])

    def test_no_entry_on_or_after_first_touch_or_safety_exit(self):
        self.outcome["contact"]["first_touch"] = {"interval_end_s": .005}
        self.assertIsNone(self.evaluate_trace()["first_entry_s"])
        self.outcome["contact"]["first_touch"]["interval_end_s"] = .0051
        self.assertEqual(self.evaluate_trace()["first_entry_s"], .005)
        self.outcome.update(safety_failure="SPEED_LIMIT", end_s=.005)
        self.assertIsNone(self.evaluate_trace()["first_entry_s"])

    def test_wrapper_observes_one_original_rollout_and_requires_pinned_protocol(self):
        plant = SimpleNamespace(spec=self.spec, metadata={"native": True})
        limits = {"max_com_speed_m_s": 2.}
        trace = self.states, self.times, np.empty((2, 4))
        with TemporaryDirectory() as directory:
            path = Path(directory) / "protocol.json"
            path.write_text(json.dumps(dict(model=plant.metadata, limits=limits, region=self.region)))
            with patch.object(diagnostics.native, "rollout", return_value=(self.outcome, trace)) as frozen:
                with self.assertRaisesRegex(ValueError, "protocol changed"):
                    with diagnostics.native_transition_diagnostics(path, "wrong-hash"):
                        self.fail("Hash validation failed")
                with diagnostics.native_transition_diagnostics(path, digest(path)):
                    result, original_trace = diagnostics.native.rollout(plant, None, None, None, None, .01, limits)
                self.assertIs(original_trace, trace)
                self.assertIs(result, self.outcome)
                self.assertEqual(result["transition_region"]["first_entry_s"], .005)
                self.assertIs(diagnostics.native.rollout, frozen)
                frozen.assert_called_once()

    def report(self, qualifying=()):
        rows = []
        for frame in range(12):
            for variant in sorted(diagnostics.VARIANTS):
                qualified = (frame, variant) in qualifying
                touch = (dict(sample_time_s=.0099, groups=["contact_front"],
                              aircraft_colliders=["front"], target_colliders=["armor"])
                         if qualified else None)
                reason = "QUALIFIED" if qualified else "NO_SIGNED_TOUCH"
                rows.append(dict(case="test", phase="rotation", frame=frame, source_frame=frame, variant=variant,
                                 source_contact_lead_s=1., horizon_s=2., transition_region=self.evaluate_trace(),
                                 qualified=qualified, reason=reason, safety_failure=None, saturation_controls=0,
                                 contact=dict(qualified=qualified, reason=reason, first_touch=touch)))
        return dict(bridge=dict(rows=rows), development=dict(cohorts=dict(
            braking=dict(episodes=36, qualified=32, safety_failures=0),
            terminal=dict(episodes=30, qualified=27, safety_failures=0),
        )))

    def test_paired_learning_signal_requires_net_gain_multiple_anchors_and_retention(self):
        control = self.report({(0, "plus")})
        intervention = self.report({(0, "minus"), (1, "minus"), (2, "nominal")})
        result = diagnostics.paired_bridge_readout(control, intervention)
        self.assertTrue(result["scientific_learning_signal"])
        self.assertEqual(result["net_qualification_delta"], 2)
        self.assertEqual(len(result["gain_trials"]), 3)
        self.assertEqual(len(result["loss_trials"]), 1)
        self.assertEqual([a["frame"] for a in result["positive_net_anchors"]], [1, 2])
        self.assertEqual(diagnostics.summarize_bridge_transition(intervention["bridge"]["rows"])["newly_entered"], 36)
        copy = deepcopy(intervention)
        copy["development"]["cohorts"]["terminal"]["safety_failures"] = 1
        self.assertFalse(diagnostics.paired_bridge_readout(control, copy)["scientific_learning_signal"])
        single = self.report({(1, "minus"), (1, "nominal"), (1, "plus")})
        self.assertFalse(diagnostics.paired_bridge_readout(control, single)["scientific_learning_signal"])
        copy["bridge"]["rows"][0]["horizon_s"] = 1.
        with self.assertRaisesRegex(ValueError, "metadata differs"):
            diagnostics.paired_bridge_readout(control, copy)

    def test_finalize_requires_equal_endpoint_and_measures_recorded_band_steps(self):
        target = diagnostics.SCIENTIFIC_RULE["transitions_per_arm"]
        with TemporaryDirectory() as directory:
            root = Path(directory)
            protocol = root / "protocol.json"
            protocol.write_text(json.dumps({"region": self.region}))
            cfg = dict(budget=dict(per_arm_transitions=target), diagnostics=dict(
                transition_protocol="protocol.json", transition_protocol_sha256=digest(protocol),
            ))
            arms, usage = {}, {}
            for arm, qualifications in (("A", ()), ("B", ((0, "nominal"), (1, "minus")))):
                actor = arm * 64
                report = self.report(qualifications) | {"checkpoint_sha256": actor}
                regional = root / arm / "regional-rollouts"
                regional.mkdir(parents=True)
                path = regional.parent / "evaluation.json"
                path.write_text(json.dumps(report))
                arms[arm] = dict(evaluation=str(path.relative_to(root)), evaluation_sha256=digest(path),
                                 checkpoint=f"{arm}/checkpoint.pt", checkpoint_sha256=actor,
                                 charged_transitions=target, trained_transitions=target)
                usage[arm] = dict(accounting=dict(updates=1, pending=None, transitions=target, learning_s=100.),
                                  evaluation_wall_s=5.)
                if arm == "A":
                    regions = {band: {"steps": 0} for band in BANDS}
                    regions["braking"] = dict(steps=target, **dict.fromkeys(SUMMARY_FIELDS, {}))
                    summary = dict(updates=1, completed_transitions=target, regions=regions)
                    np.savez(regional / "rollout-098304.npz", summary_json=json.dumps(summary),
                             **dict.fromkeys(SUMMARY_FIELDS, np.zeros(1)))
            comparison = dict(equal_budget_endpoint=dict(arms=arms), usage=usage, promotion={"decision": "NO_GO"})
            (root / "bridge-comparison.json").write_text(json.dumps(comparison))
            with patch.object(diagnostics, "ROOT", root):
                result = diagnostics.finalize_diagnostics(cfg, root, comparison)
                self.assertEqual(result, diagnostics.finalize_diagnostics(cfg, root, comparison))
                arms["B"]["trained_transitions"] -= 1
                with self.assertRaisesRegex(ValueError, "has not completed exactly"):
                    diagnostics.finalize_diagnostics(cfg, root, comparison)
            self.assertTrue(result["readout"]["scientific_learning_signal"])
            self.assertEqual(result["frozen_promotion"]["decision"], "NO_GO")
            self.assertEqual(result["usage"]["A"]["learning_transitions_per_second"], target / 100.)
            self.assertTrue(result["usage"]["A"]["regional_metrics"]["complete"])
            self.assertEqual(result["usage"]["A"]["regional_metrics"]["per_band_transitions"]["braking"], target)
            self.assertEqual(result["usage"]["B"]["regional_metrics"]["missing_updates"], [1])
