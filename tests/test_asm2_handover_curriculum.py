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


if __name__ == "__main__":
    unittest.main()
