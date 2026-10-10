"""End-to-end gates for the independent truth-guided interception scenario."""
import json
from pathlib import Path

import numpy as np
import mujoco
import pytest

from research.Darts.experiment import pose as original_dart_pose
from research.Darts.trajectory import BallisticPath
from rm27.experiments.dart_interception.experiment import DEFAULT_CONFIG, ROOT, simulate, prescribed_dart_state


def test_prescribed_state_matches_original_replay_and_angular_rate():
    # No CAD needed: exercise the original replay pose routine on a mocap body.
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="darts_projectile" mocap="true"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    path = BallisticPath.aimed([12., -6., 1.05], [-11.6, .02, 1.26], 1.77)
    for t in [0., .4, path.velocity[2]/9.81, 1.2, 1.77]:
        original_dart_pose(model, data, path, t)
        q, v = prescribed_dart_state(path, t)
        np.testing.assert_allclose(q[:3], data.mocap_pos[0], atol=1e-12)
        assert abs(np.dot(q[3:], data.mocap_quat[0])) == pytest.approx(1., abs=1e-12)
        np.testing.assert_allclose(v[:3], path.state(t)[1], atol=1e-12)
        expected_pitch_rate = 9.81*np.linalg.norm(path.velocity[:2])/np.dot(v[:3], v[:3])
        np.testing.assert_allclose(v[3:], [0., expected_pitch_rate, 0.], atol=1e-8)


@pytest.mark.skipif(not (ROOT / "artifacts/isolated-blue-base-red-dart-20260927-v1/scene.xml").exists(),
                    reason="Local CAD-derived scene artifact unavailable")
def test_active_intercept_changes_dart_outcome(tmp_path):
    active = simulate(tmp_path / "active")
    passive = simulate(tmp_path / "passive", active=False)
    cfg = json.loads(Path(DEFAULT_CONFIG).read_text())
    assert active["passed"] and active["interception_succeeded"]
    assert passive["passed"] and not passive["interception_succeeded"]
    assert active["base_proxy_contact"] is None
    assert passive["base_proxy_contact"] is not None
    start = active["inputs"]["config"]["start_center_m"]
    assert active["takeoff_center_height_at_launch_m"] > start[2] + .3
    assert active["max_prelaunch_horizontal_motion_m"] < .005
    assert active["inputs"]["restored_hangar_geoms"]
    assert np.allclose(np.load(tmp_path / "active/trace.npz")["qpos"][0, :3],
                       [.651751442, 1.0824279387749842, 1.762])
    assert active["first_post_launch_lateral_move_s"] > active["launch_time_s"]
    assert active["intercept"]["time_s"] < passive["base_proxy_contact"]["time_s"]
    a = np.load(tmp_path / "active/trace.npz")
    b = np.load(tmp_path / "passive/trace.npz")
    i = np.argmin(abs(a["time"] - passive["base_proxy_contact"]["time_s"]))
    k = np.argmin(abs(b["time"]-a["time"][i]))
    assert np.linalg.norm(a["qpos"][i, -7:-4] - b["qpos"][k, -7:-4]) > .15
    original = json.loads((ROOT / cfg["source_dart_run"] / "summary.json").read_text())
    original_model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="darts_projectile" mocap="true"/></worldbody></mujoco>')
    original_data = mujoco.MjData(original_model)
    path = BallisticPath.aimed(original["launch_m"], original["target_m"], original["flight_time_s"],
                               original["config"]["gravity_m_s2"])
    for report, trace in [(active, a), (passive, b)]:
        release = report["dart_release"]
        elapsed = trace["time"] - report["launch_time_s"]
        before = (elapsed >= 0.) & (trace["time"] <= release["time_s"])
        for index in np.flatnonzero(before):
            original_dart_pose(original_model, original_data, path, elapsed[index])
            np.testing.assert_allclose(trace["qpos"][index, -7:-4], original_data.mocap_pos[0], atol=1e-10)
            assert abs(np.dot(trace["qpos"][index, -4:], original_data.mocap_quat[0])) == pytest.approx(1., abs=1e-12)
        event = np.flatnonzero(trace["time"] == release["time_s"])[0]
        np.testing.assert_allclose(trace["qpos"][event, -7:], release["qpos"], atol=0.)
        np.testing.assert_allclose(trace["qvel"][event, -6:], release["qvel"], atol=0.)
        assert np.linalg.norm(release["qvel"][3:]) > .1
        assert np.all(trace["dart_mode"][event:] == "dynamic")
        assert np.all(trace["dart_mode"][(elapsed >= 0.) & (trace["time"] < release["time_s"])] == "prescribed")
        # After release both pose and velocity must depart from the forced path.
        p, v = path.state(elapsed[-1])
        assert np.linalg.norm(trace["qpos"][-1, -7:-4]-p) > .1
        assert np.linalg.norm(trace["qvel"][-1, -6:-3]-v) > .1
    assert active["dart_release"]["time_s"] == active["intercept"]["time_s"]
    assert passive["dart_release"]["time_s"] == passive["base_proxy_contact"]["time_s"]
    assert active["uav_dart_normal_impulse_Ns"] > 0.
    assert passive["base_dart_normal_impulse_Ns"] > 0.
    assert np.all(np.isfinite(a["qpos"]))
