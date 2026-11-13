import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from rm27_asm2.asset import ROOT, digest, write_result
from rm27_asm2.contact_task import load_contact_spec
from rm27_asm2.flight import load_profile
from rm27_asm2.handover_curriculum import NAV_CONFIG, RUN_CONFIG, build_dataset


class ASM2ResetDatasetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = load_contact_spec()
        cls.profile = load_profile()

    def _case(self, root, name, split, kind, *, qualified, nav_passed=True):
        spec, profile = self.spec, self.profile
        model = {
            "backend": "MuJoCo", "version": "3.12.0", "physics": {"timestep_s": spec.dt},
            "detailed_aircraft_colliders": len(spec.entries), "profile_sha256": profile.profile_sha256,
            "contract_sha256": spec.contract_sha256,
            "field": {
                "geometry_json_sha256": spec.config["field_geometry"]["sha256"],
                "geometry_npz_sha256": digest((ROOT / spec.config["field_geometry"]["path"]).with_suffix(".npz")),
                "native_scene_sha256": spec.field_geometry["model_sha256"],
                "registration_sha256": spec.config["registration"]["sha256"],
            },
        }
        times = np.array([0., .005, .010, .015, .020, .0201])
        states = np.zeros((len(times), 13))
        states[:, :3] = spec.target_center + np.array([.32, .31, .28, .26, .020, .019])[:, None] * spec.target_normal
        states[:, 3] = 1.
        states[:, 7:10] = -np.array([.02, .03, .21, .13, .05, .0])[:, None] * spec.target_normal
        thrust = np.tile(profile.hover_motor_thrust_N, (len(times), 1))
        targets = thrust + np.arange(len(times))[:, None] * .001
        trace = root / f"{name}.npz"
        np.savez_compressed(trace, states=states[:, None], motor_thrust_N=thrust[:, None],
                            motor_target_N=targets[:, None], sample_times_s=times,
                            handover_frame=1, dt=profile.policy_dt_s,
                            reference=states[:, None, :3], ctbr_N_rad_s=np.ones((5, 1, 4)),
                            names=np.array([name]))
        contact = dict(tick=201, sample_time_s=.020, groups=["contact_front"],
                       target_colliders=[sorted(spec.legal_targets)[0]], compressive_normal_impulse_N_s=1e-5)
        contact_reason = "QUALIFIED" if qualified else "NO_SIGNED_TOUCH"
        result = dict(
            qualified=qualified, reason=contact_reason, contract_sha256=spec.contract_sha256,
            physics_dt_s=spec.dt, first_touch=contact if qualified else None,
            first_loaded=contact if qualified else None, positive_separation_compressive_records=0,
        )
        report = dict(case=name, split=split, kind=kind, model=model, result=result,
                      qualified=qualified, reason=contact_reason if nav_passed else "NAVIGATION_GATE",
                      navigation={"passed": nav_passed}, safety={"passed": qualified},
                      handover_frame=1, handover_state=states[1].tolist(), handover_time_s=.005,
                      trace_sha256=digest(trace))
        write_result(root / f"{name}.json", report)
        return report

    def _summary(self, root, cases, unrun=()):
        write_result(root / "summary.json", dict(
            schema="rm27_asm2_handover_results_v1", contact_contract_sha256=self.spec.contract_sha256,
            profile_sha256=self.profile.profile_sha256, config_sha256=digest(RUN_CONFIG),
            navigation_config_sha256=digest(NAV_CONFIG), training_transitions=0,
            evaluated=len(cases), qualified=sum(case["qualified"] for case in cases),
            cases=[{key: case[key] for key in ("case", "split", "kind", "qualified", "reason")} for case in cases],
            unrun=list(unrun),
        ))

    def test_phase_resets_keep_exact_pre_contact_state_and_split(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            cases = [
                self._case(root, "dev_center", "development", "nominal", qualified=True),
                self._case(root, "dev_crossflow", "development", "boundary", qualified=False, nav_passed=False),
                self._case(root, "held_center_mid", "heldout", "nominal", qualified=True),
                self._case(root, "held_diagonal", "heldout", "nominal", qualified=False, nav_passed=False),
                self._case(root, "held_fast_offset", "heldout", "boundary", qualified=False),
            ]
            self._summary(root, cases, unrun=["held_outer_slow"])
            manifest = build_dataset(root, spec=self.spec, profile=self.profile)
            bundle = root / "reset-dataset"
            with np.load(bundle / "development.npz", allow_pickle=False) as dev, np.load(
                root / "dev_center.npz", allow_pickle=False
            ) as original:
                frames = dev["source_frame"]
                np.testing.assert_array_equal(np.sort(frames), [1, 2, 3])
                self.assertEqual(set(dev["case_ids"]), {"dev_center"})
                self.assertTrue(np.all(dev["sample_times_s"] < .020))
                for key in ("states", "motor_thrust_N", "motor_target_N"):
                    np.testing.assert_array_equal(dev[key], original[key][frames, 0])
                self.assertFalse({"reference", "ctbr_N_rad_s"} & set(dev.files))
            with np.load(bundle / "heldout.npz", allow_pickle=False) as phases, np.load(
                bundle / "heldout-handover.npz", allow_pickle=False
            ) as handovers:
                self.assertEqual(set(phases["case_ids"]), {"held_center_mid"})
                self.assertEqual(dict(zip(handovers["case_ids"], handovers["witness_qualified"])),
                                 {"held_center_mid": True, "held_fast_offset": False})
            heldout = manifest["heldout_handover"]
            self.assertEqual(heldout["frozen_recipe_denominator"], 4)
            self.assertEqual(heldout["cases"]["held_diagonal"]["status"], "NAVIGATION_REJECTED")
            self.assertEqual(heldout["cases"]["held_outer_slow"]["status"], "UNRUN")
            self.assertEqual(json.loads((bundle / "manifest.json").read_text()), manifest)

    def test_trace_change_prevents_publication(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            case = self._case(root, "dev_center", "development", "nominal", qualified=True)
            self._summary(root, [case])
            with (root / "dev_center.npz").open("ab") as trace:
                trace.write(b"changed")
            with self.assertRaisesRegex(ValueError, "Trace SHA-256 mismatch"):
                build_dataset(root, spec=self.spec, profile=self.profile)
            self.assertFalse((root / "reset-dataset").exists())



class ResumableCurriculumTests(unittest.TestCase):
    def test_sampler_refresh_and_rng_restore(self):
        from rm27_asm2.terminal_curriculum import ResetSampler
        bands = np.array(['terminal', 'braking', 'rotation_late', 'rotation_middle', 'rotation_early', 'handover'])
        settings = dict(boundary_mix=.55, boundary_floor=.05, discovery_mix=.1,
                        retention={'terminal':.1, 'braking':.25})
        a = ResetSampler(bands, 'adaptive', 12, settings)
        a.refresh([[1,1,1], [1,1,1], [0,1,1], [0,0,0], [0,0,0], [0,0,0]])
        self.assertGreater(a.weights()[2], a.weights()[3])
        self.assertGreaterEqual(a.weights()[0], .1)
        self.assertGreaterEqual(a.weights()[1], .25)
        a.sample(19)
        b = ResetSampler(bands, 'adaptive', 99, settings)
        b.load_state_dict(a.state_dict())
        np.testing.assert_array_equal(a.sample(100), b.sample(100))
        a.refresh(np.zeros((6,3), dtype=int))
        self.assertTrue(np.all(a.probability == 0))  # replaces stale actor evidence

    def test_fixed_control_mixture(self):
        from rm27_asm2.terminal_curriculum import ResetSampler, BANDS, MIX
        sampler = ResetSampler(BANDS, 'fixed', 3, {})
        for stage in range(3):
            sampler.stage = stage
            np.testing.assert_allclose(sampler.weights(), MIX[stage])

    def test_bridge_mixture_redistributes_only_late_rotation_and_resumes_exactly(self):
        from rm27_asm2.terminal_curriculum import ResetSampler

        bands = np.array(['terminal', 'braking', 'rotation_late', 'rotation_late',
                          'rotation_middle', 'rotation_bridge', 'rotation_bridge'])
        control = dict(terminal=.10, braking=.45, rotation_late=.45)
        bridge = dict(control, rotation_late=.20, rotation_bridge=.25)
        a = ResetSampler(bands, 'fixed', 11, {'fixed_mix': control})
        b = ResetSampler(bands, 'fixed', 11, {'fixed_mix': bridge})
        legacy = ResetSampler(bands[:-2], 'fixed', 11, {})
        np.testing.assert_array_equal(a.weights()[:-2], legacy.weights())
        np.testing.assert_array_equal(a.sample(100), legacy.sample(100))
        self.assertEqual(a.draws[-2:].sum(), 0)
        np.testing.assert_allclose(b.weights(), [.10, .45, .10, .10, 0., .125, .125])
        b.stage = 2
        np.testing.assert_allclose(b.weights(), [.10, .45, .10, .10, 0., .125, .125])
        b.sample(100)
        self.assertGreater(b.draws[-2:].sum(), 0)
        resumed = ResetSampler(bands, 'fixed', 99, {'fixed_mix': bridge})
        resumed.load_state_dict(b.state_dict())
        np.testing.assert_array_equal(b.sample(100), resumed.sample(100))
        np.testing.assert_array_equal(b.draws, resumed.draws)
        with self.assertRaisesRegex(ValueError, 'Sampler configuration changed'):
            a.load_state_dict(b.state_dict())

    def test_static_mixture_rejects_invalid_mass_and_missing_active_band(self):
        from rm27_asm2.terminal_curriculum import ResetSampler

        for mix in ({'invalid': 1.}, {'terminal': float('nan')}, {'terminal': .9},
                    {'terminal': 1.1, 'braking': -.1}, {'rotation_bridge': 1.}):
            with self.subTest(mix=mix), self.assertRaises(ValueError):
                ResetSampler(['terminal', 'braking'], 'fixed', 11, {'fixed_mix': mix})

    def test_budget_restart_and_aggregate(self):
        from rm27_asm2.terminal_curriculum import CampaignLedger
        from simpleflight_lab.stage1_training import LearningLimit
        import tempfile
        caps = dict(per_arm_transitions=20, total_transitions=30,
                    per_arm_learning_s=30, total_learning_s=40, update_reservation_s=10)
        with tempfile.TemporaryDirectory() as directory:
            a = CampaignLedger(directory, 'config', caps)
            a.reserve('A', 10)
            a.settle('A', 3., 'checkpoint', True)
            a.reserve('A', 10)  # crash before settlement stays fully charged
            a.close()
            b = CampaignLedger(directory, 'config', caps)
            self.assertEqual(b.data['arms']['A']['transitions'], 20)
            self.assertEqual(b.data['arms']['A']['learning_s'], 13.)
            self.assertEqual(len(b.data['arms']['A']['abandoned']), 1)
            with self.assertRaises(LearningLimit):
                b.reserve('A', 1)
            b.reserve('B', 10)
            b.settle('B', 4., 'checkpoint-b', True)
            with self.assertRaises(LearningLimit):
                b.reserve('B', 1)
            b.close()
            with self.assertRaises(ValueError):
                CampaignLedger(directory, 'changed-config', caps)


class PolicyFrontierTests(unittest.TestCase):
    def test_anchor_ordinals_follow_source_frames_in_each_witness(self):
        from rm27_asm2.policy_frontier import select_anchors

        bank = dict(
            case_ids=np.array(["dev_b", "dev_a", "dev_b", "dev_a", "dev_b", "dev_a"]),
            phase=np.array(["rotation", "rotation", "rotation", "handover", "handover", "rotation"]),
            source_frame=np.array([80, 25, 60, 10, 50, 15]),
        )
        anchors = [("gate", "handover", 0), ("early", "rotation", 0), ("late", "rotation", -1)]
        self.assertEqual(
            select_anchors(bank, ["dev_b", "dev_a"], anchors),
            [("dev_b", "gate", 4), ("dev_b", "early", 2), ("dev_b", "late", 0),
             ("dev_a", "gate", 3), ("dev_a", "early", 5), ("dev_a", "late", 1)],
        )
        with self.assertRaises(ValueError):
            select_anchors(bank, ["dev_a"], [("last", "rotation", -1), ("same", "rotation", 1)])

    def test_perturbations_are_matched_and_preserve_attitude_and_body_rates(self):
        from rm27_asm2.policy_frontier import perturbed_state

        state = np.array([.3, .2, .4, 1., 0., 0., 0., .2, -.5, .1, .6, .7, .8])
        tangent = np.array([.6, 0., .8])
        nominal = perturbed_state(state, tangent, 0, .001, .01)
        np.testing.assert_array_equal(nominal, state)
        self.assertFalse(np.shares_memory(nominal, state))
        for sign in (-1, 1):
            changed = perturbed_state(state, tangent, sign, .001, .01)
            np.testing.assert_allclose(changed[:3] - nominal[:3], sign * .001 * tangent, atol=1e-15)
            np.testing.assert_allclose(changed[7:10] - nominal[7:10], sign * .01 * tangent, atol=1e-15)
            np.testing.assert_array_equal(changed[3:7], nominal[3:7])
            np.testing.assert_array_equal(changed[10:], nominal[10:])
        np.testing.assert_array_equal(state, nominal)


class ActuatorMemoryTests(unittest.TestCase):
    def test_actor_features_match_frozen_fixed_a_inference(self):
        from rm27_asm2 import terminal_native_eval as native
        from rm27_asm2.actuator_memory import actor_features

        manifest_path = ROOT / ".agents/runs/asm2-frontier/phase-r/manifest.json"
        if not manifest_path.is_file():
            self.skipTest("Frozen Phase R manifest is unavailable")
        manifest = json.loads(manifest_path.read_text())
        actor = manifest["controllers"]["fixed_a"]
        checkpoint = ROOT / actor["checkpoint"]
        weights = checkpoint.with_suffix(".npz")
        if not checkpoint.is_file() or not weights.is_file():
            self.skipTest("Frozen Fixed A checkpoint or portable actor is unavailable")
        profile = load_profile()
        goal = np.asarray(manifest["goal"])
        state = np.asarray(next(s["state"] for s in manifest["samples"]
                                if s["phase"] == "rotation" and s["variant"] == "nominal"))
        command, pin = native.load_controller(weights, digest(checkpoint), profile, goal)
        self.assertEqual(pin, {k: actor[k] for k in pin})
        features = actor_features(state, goal)
        self.assertEqual(features.shape, (45,))
        self.assertEqual(features.dtype, np.float32)
        np.testing.assert_array_equal(features[:30].reshape(10, 3),
                                      np.repeat((goal - state[:3]).astype(np.float32)[None], 10, axis=0))
        with np.load(weights, allow_pickle=False) as archive:
            layers = native.NativeActor.layers(archive, 45)
        action = np.tanh(native.NativeActor.mean(features[None], layers)[0])
        np.testing.assert_array_equal(command(state),
                                      np.r_[profile.hover_thrust_N * (1. + .5 * action[3]), 2. * action[:3]])

    def test_visible_errors_use_physical_norms_and_quaternion_sign_invariance(self):
        from rm27_asm2.actuator_memory import visible_errors

        state = np.zeros(13)
        state[3] = 1.
        equivalent = state.copy()
        equivalent[3:7] *= -1.
        changed = state.copy()
        changed[:3] = [.003, .004, 0.]
        half_angle = np.deg2rad(1.5)
        changed[3:7] = -np.array([np.cos(half_angle), 0., 0., np.sin(half_angle)])
        changed[7:10] = [0., 0., -.02]
        changed[10:13] = [.3, .4, 0.]
        np.testing.assert_allclose(visible_errors(state, [equivalent, changed]),
                                   [[0., 0., 0., 0.], [.005, .02, 3., .5]], rtol=0., atol=1e-10)

    def test_pulse_panel_omits_commands_outside_actor_limits(self):
        from rm27_asm2.actuator_memory import pulse_panel

        hover = 1.4
        center = np.array([1.5 * hover, -2., 2., 0.])
        original = center.copy()
        panel = pulse_panel(center, hover, dict(collective_fraction_hover=.02, rate_rad_s=.1))
        self.assertEqual(set(panel), {"center", "thrust_minus", "roll_plus", "pitch_minus",
                                      "yaw_minus", "yaw_plus"})
        np.testing.assert_allclose(panel["thrust_minus"] - center, [-.028, 0., 0., 0.], atol=1e-15)
        np.testing.assert_allclose(panel["roll_plus"] - center, [0., .1, 0., 0.], atol=1e-15)
        np.testing.assert_allclose(panel["pitch_minus"] - center, [0., 0., -.1, 0.], atol=1e-15)
        panel["center"][0] = 0.
        np.testing.assert_array_equal(center, original)
        self.assertGreater(panel["yaw_plus"][0], 0.)

    def test_local_ranking_separates_viability_and_strict_reversals(self):
        from rm27_asm2.actuator_memory import compare_panel

        left = {"pitch_plus": 2, "pitch_minus": 1, "center": 1, "roll_plus": 0}
        right = {"pitch_plus": 1, "pitch_minus": 2, "center": 1, "roll_plus": 1}
        result = compare_panel(left, right)
        self.assertEqual(result["changed_viable_actions"], ["roll_plus"])
        self.assertEqual(result["strict_preference_reversals"], [("pitch_plus", "pitch_minus")])
        with self.assertRaisesRegex(ValueError, "same actions"):
            compare_panel(left, {"center": 1})

    def test_pulse_counts_control_decisions_then_delegates_live_state(self):
        from rm27_asm2.actuator_memory import command_with_pulse

        applied_pulse = np.array([1.4, .05, -.1, 0.])
        calls = []

        def actor(state):
            calls.append(state)
            return np.array([1.4, float(state[0]), 0., 0.])

        action = command_with_pulse(actor, applied_pulse, 4)
        states = [np.array([tick / 10]) for tick in range(6)]
        commands = [action(state) for state in states]
        for command in commands[:4]:
            np.testing.assert_array_equal(command, applied_pulse)
        self.assertEqual(len(calls), 2)
        self.assertIs(calls[0], states[4])
        self.assertIs(calls[1], states[5])
        np.testing.assert_array_equal(commands[4], [1.4, .4, 0., 0.])
        np.testing.assert_array_equal(commands[5], [1.4, .5, 0., 0.])
        for pulse, ticks in ((None, 4), (applied_pulse, 0)):
            np.testing.assert_array_equal(command_with_pulse(actor, pulse, ticks)(states[1]),
                                          [1.4, .1, 0., 0.])

    def test_collective_branches_are_symmetric_and_keep_body_rates(self):
        from rm27_asm2.actuator_memory_followup import branch_commands

        hover = 1.25
        center = np.array([hover, .4, -.3, .2])
        original = center.copy()
        branches = branch_commands(center, hover)
        self.assertEqual(set(branches), {"minus", "plus"})
        for name, sign in (("minus", -1.), ("plus", 1.)):
            np.testing.assert_allclose(branches[name][0], center[0] + sign * .14 * hover,
                                       rtol=0., atol=1e-12)
            np.testing.assert_array_equal(branches[name][1:], center[1:])
        np.testing.assert_array_equal(center, original)
        self.assertFalse(np.shares_memory(branches["minus"], center))
        self.assertFalse(np.shares_memory(branches["minus"], branches["plus"]))
        for outside in ([.63 * hover, 0., 0., 0.], [1.37 * hover, 0., 0., 0.],
                        [hover, 0., -2.01, 0.]):
            with self.subTest(outside=outside), self.assertRaises(ValueError):
                branch_commands(np.array(outside), hover)

    def test_followup_matches_require_visible_actor_and_activation_agreement(self):
        from rm27_asm2.actuator_memory import CONFIG
        from rm27_asm2.actuator_memory_followup import matched_pair

        cfg = json.loads(CONFIG.read_text())
        hover = 1.25
        actor = cfg["actor_limits"]
        residual = {k: .5 * v for k, v in cfg["visible_limits"].items()}
        center = np.array([hover, 0., 0., 0.])
        close = center + .5 * np.array([hover * actor["collective_fraction_hover"],
                                      0., actor["rate_rad_s"], 0.])
        pair = dict(visible_residual=residual, ctbr=[center.tolist(), close.tolist()],
                    activation_delta_N=[0., -cfg["min_memory_delta_N"], 0., 0.],
                    previous_target_delta_N=[0.] * 4)
        self.assertTrue(matched_pair(pair, cfg, hover))
        target_only = dict(pair, activation_delta_N=[0.] * 4, previous_target_delta_N=[.15] * 4)
        self.assertFalse(matched_pair(target_only, cfg, hover))
        for field, limit in cfg["visible_limits"].items():
            with self.subTest(field=field):
                outside = dict(residual, **{field: 1.01 * limit})
                self.assertFalse(matched_pair(dict(pair, visible_residual=outside), cfg, hover))
        for command in ([hover * (1. + 1.01 * actor["collective_fraction_hover"]), 0., 0., 0.],
                        [hover, 0., -1.01 * actor["rate_rad_s"], 0.]):
            with self.subTest(command=command):
                outside = dict(pair, ctbr=[center.tolist(), command])
                self.assertFalse(matched_pair(outside, cfg, hover))


class TransitionMatchedBudgetTests(unittest.TestCase):
    def test_measures_learning_time_without_capping_matched_transitions(self):
        from tempfile import TemporaryDirectory
        from pathlib import Path
        from rm27_asm2.rotation_bridge_experiment import TransitionLedger
        from simpleflight_lab.stage1_training import LearningLimit

        with TemporaryDirectory() as temp:
            path = Path(temp)
            caps = dict(per_arm_transitions=2048, total_transitions=4096)
            ledger = TransitionLedger(path, "test", caps)
            self.assertEqual(ledger.reserve("A", 1024), 0.)
            ledger.settle("A", 1400., "resume.pt", True)
            ledger.close()
            ledger = TransitionLedger(path, "test", caps)
            self.assertEqual(ledger.remaining("A"), (1024, float("inf")))
            for arm in ("A", "B", "B"):
                ledger.reserve(arm, 1024)
                ledger.settle(arm, 1400., "resume.pt", True)
            for arm in ("A", "B"):
                self.assertEqual(ledger.data["arms"][arm]["transitions"], 2048)
                self.assertEqual(ledger.data["arms"][arm]["learning_s"], 2800.)
                with self.assertRaisesRegex(LearningLimit, "TRANSITION_LIMIT"):
                    ledger.reserve(arm, 1024)
            ledger.close()

    def test_update_still_enforces_frame_cap_and_explicit_interrupt(self):
        from rm27_asm2.rotation_bridge_experiment import TransitionBudget
        from simpleflight_lab.stage1_training import LearningLimit

        budget = TransitionBudget(16, 0.)
        budget.before_step(16)
        with self.assertRaisesRegex(LearningLimit, "TRANSITION_LIMIT"):
            budget.before_step(1)
        budget.stop_reason = "INTERRUPTED"
        with self.assertRaisesRegex(LearningLimit, "INTERRUPTED"):
            budget.check()


if __name__ == "__main__":
    unittest.main()
