"""Check the selected UAV's SITL interface in a shared physical dock scene."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest


def test_live_view_isolates_ui_and_copies_all_vehicles(scene_path, monkeypatch):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import mujoco.viewer
    import rm27.simulation.live_view as live_view
    from rm27.simulation.live_view import FlightView

    plant = MicroPlant(scene_path)
    calls = []
    handle = SimpleNamespace(
        cam=mujoco.MjvCamera(),
        opt=mujoco.MjvOption(),
        lock=nullcontext,
        is_running=lambda: True,
        set_texts=lambda text: calls.append(text),
        sync=lambda **kwargs: None,
        close=lambda: None,
    )
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda *a, **kw: handle)
    viewer = FlightView(plant)
    viewer.model.opt.gravity[:] = 100
    viewer.data.ctrl[:] = 99
    viewer.data.qpos[:] = 0
    assert np.array_equal(plant.model.opt.gravity, [0, 0, -9.81])
    assert np.all(plant.data.ctrl == 0)
    plant.step([1400] * 16)
    before = plant.data.qpos.copy()
    clock = iter([0.0, 0.05, 0.051, 0.1, 0.15])
    monkeypatch.setattr(
        live_view, "time", SimpleNamespace(monotonic=lambda: next(clock))
    )
    viewer.update("HOVER", {"mode": "GUIDED", "armed": True}, force=True)
    assert np.array_equal(plant.data.qpos, before)
    assert np.array_equal(viewer.data.qpos, plant.data.qpos)
    assert np.array_equal(viewer.data.qvel, plant.data.qvel)
    assert np.array_equal(viewer.data.actuator_force, plant.data.actuator_force)
    assert viewer.data.time == plant.data.time
    assert "HOVER" in calls[-1][3]
    # A slow display sync must not cause every subsequent physics step to render.
    viewer.update("RETURN", {"mode": "GUIDED", "armed": True})
    assert viewer.updates == 1
    viewer.update("ROUTE", {"mode": "GUIDED", "armed": True}, force=True)
    np.testing.assert_allclose(handle.cam.lookat, plant.qpos[:3])
    np.testing.assert_array_equal(plant.data.qpos, before)
    viewer.close()


from rm27.integration import dock, drone
from rm27.simulation.micro_plant import MicroPlant


@pytest.fixture
def scene_path(tmp_path):
    root = ET.fromstring(
        '<mujoco><option timestep=".002" integrator="RK4"/>'
        "<asset/><worldbody/></mujoco>"
    )
    translation = np.array([2.3, -1.7, 0.8])
    yaw = np.pi / 4
    quaternion = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    rotation = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    body = ET.SubElement(
        root.find("worldbody"),
        "body",
        name="dock",
        pos=" ".join(map(str, translation)),
        quat=" ".join(map(str, quaternion)),
    )
    for name, center, size in dock.boxes():
        ET.SubElement(
            body,
            "geom",
            name=f"dock_{name}",
            type="box",
            pos=" ".join(map(str, center)),
            size=" ".join(map(str, size)),
        )
    cfg = json.loads(
        (Path(__file__).resolve().parents[1] / "configs/drone.json").read_text()
    )
    for i, (name, center) in enumerate(dock.slot_centers().items(), start=1):
        ET.SubElement(body, "site", name=name, pos=" ".join(map(str, center)))
        spawn = translation + rotation @ (np.array(center) + [0, 0, 0.021])
        drone.add(root, f"drone_{i}", spawn, quaternion, cfg)
    path = tmp_path / "scene.xml"
    ET.ElementTree(root).write(path, encoding="unicode")
    return path


@pytest.mark.parametrize(
    "pwm, expected",
    [
        ([2000, 1000, 1000, 1000], [0, 1, 0, 0]),
        ([1000, 2000, 1000, 1000], [0, 0, 0, 1]),
        ([1000, 1000, 2000, 1000], [1, 0, 0, 0]),
        ([1000, 1000, 1000, 2000], [0, 0, 1, 0]),
        ([900, 2100, 1250, 1750], [0.25, 0, 0.75, 1]),
    ],
)
def test_vehicle_two_motor_mapping_and_scale(scene_path, pwm, expected):
    plant = MicroPlant(scene_path, vehicle=2)
    assert plant.qa == 7 and plant.va == 6
    assert plant.motors.tolist() == [4, 5, 6, 7]
    # Old or unrelated commands must not leak into the inactive vehicles.
    plant.data.ctrl[:] = 0.5
    plant.step(pwm)
    np.testing.assert_allclose(
        plant.data.ctrl[4:8], np.array(expected) * 1.103625, rtol=1e-12, atol=1e-15
    )
    np.testing.assert_array_equal(plant.data.ctrl[:4], 0)
    np.testing.assert_array_equal(plant.data.ctrl[8:], 0)
    assert plant.data.time == pytest.approx(0.002)
    np.testing.assert_allclose(plant.truth()["command_thrust_n"], plant.data.ctrl[4:8])


def test_selected_slot_origin_and_ned_state(scene_path):
    plant = MicroPlant(scene_path, vehicle=2)
    np.testing.assert_allclose(
        plant.origin, [2.3 - np.sqrt(2) * 0.07375, -1.7, 0.96], atol=1e-12
    )
    np.testing.assert_allclose(plant.state()["position"], [0, 0, -0.021], atol=1e-12)
    # Distinct first-vehicle state catches accidental use of qpos[:3]/qvel[:3].
    plant.data.qpos[:3] = [20, 30, 40]
    plant.data.qvel[:6] = [11, 12, 13, 14, 15, 16]
    plant.qpos[:3] = plant.origin + [0.11, -0.07, 0.24]
    plant.qvel[:] = [0.3, -0.4, 0.5, 1, 2, 3]
    # A 90-degree FLU yaw must become a negative 90-degree FRD/NED yaw.
    plant.qpos[3:7] = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    mujoco.mj_forward(plant.model, plant.data)
    plant.data.qacc[plant.va : plant.va + 3] = [0, 0, 0]
    state = plant.state()
    np.testing.assert_allclose(state["position"], [0.11, 0.07, -0.24], atol=1e-12)
    np.testing.assert_allclose(state["velocity"], [0.3, 0.4, -0.5], atol=1e-12)
    np.testing.assert_allclose(state["imu"]["gyro"], [1, -2, -3], atol=1e-12)
    np.testing.assert_allclose(state["imu"]["accel_body"], [0, 0, -9.81], atol=1e-12)
    np.testing.assert_allclose(
        state["quaternion"], [np.sqrt(0.5), 0, 0, -np.sqrt(0.5)], atol=1e-12
    )
    np.testing.assert_allclose(plant.truth()["position_nwu"], plant.qpos[:3])
    np.testing.assert_allclose(plant.truth()["body_rates_flu"], [1, 2, 3])


def test_inactive_vehicles_settle_independently_of_selected_thrust(scene_path):
    active = MicroPlant(scene_path, vehicle=2)
    reference = MicroPlant(scene_path, vehicle=2)
    for _ in range(500):
        active.step([1700] * 4)
        reference.step([1000] * 4)
        np.testing.assert_array_equal(active.data.ctrl[:4], 0)
        np.testing.assert_array_equal(active.data.ctrl[8:], 0)
    assert active.qpos[2] > active.origin[2] + 0.3
    for qa, va, initial in active.inactive:
        np.testing.assert_allclose(
            active.data.qpos[qa : qa + 7], reference.data.qpos[qa : qa + 7], atol=1e-12
        )
        np.testing.assert_allclose(
            active.data.qvel[va : va + 6], reference.data.qvel[va : va + 6], atol=1e-12
        )
        np.testing.assert_allclose(
            active.data.qpos[qa : qa + 2], initial[:2], atol=1e-8
        )
        assert active.data.qpos[qa + 2] == pytest.approx(0.98, abs=0.002)
        assert np.max(np.abs(active.data.qvel[va : va + 6])) < 0.01


@pytest.mark.parametrize("pwm", [[1000] * 3, [1000, np.nan, 1000, 1000]])
def test_invalid_pwm_cannot_advance_physics(scene_path, pwm):
    plant = MicroPlant(scene_path, vehicle=2)
    initial = plant.data.qpos.copy()
    with pytest.raises(ValueError, match="four finite PWM"):
        plant.step(pwm)
    assert plant.data.time == 0
    np.testing.assert_array_equal(plant.data.qpos, initial)
    np.testing.assert_array_equal(plant.data.ctrl, 0)
