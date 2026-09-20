"""Physical scene checks; root tests intentionally do not run the subrepo suites."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest
from rm27.integration import dock, drone


@pytest.fixture
def scene():
    root = ET.fromstring(
        '<mujoco><option timestep=".002" integrator="RK4"/><asset/><worldbody/></mujoco>'
    )
    body = ET.SubElement(root.find("worldbody"), "body", name="dock")
    for name, p, size in dock.boxes():
        ET.SubElement(
            body,
            "geom",
            name=name,
            type="box",
            pos=" ".join(map(str, p)),
            size=" ".join(map(str, size)),
        )
    cfg = json.loads(Path("configs/drone.json").read_text())
    for i, p in enumerate(dock.slot_centers().values()):
        drone.add(root, f"drone_{i+1}", np.array(p) + [0, 0, 0.021], [1, 0, 0, 0], cfg)
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def test_slot_floor_and_walls(scene):
    m, d = scene
    # Disable aircraft only for ray geometry queries, leaving dock shape unchanged.
    for g in range(m.ngeom):
        if m.geom_bodyid[g] > 1:
            m.geom_group[g] = 5
    groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
    for center in dock.slot_centers().values():
        hit = np.array([-1], dtype=np.int32)
        p = np.array(center) + [0, 0, 0.03]
        length = mujoco.mj_ray(m, d, p, np.array([0.0, 0.0, -1.0]), groups, 1, -1, hit)
        assert length == pytest.approx(0.03, abs=1e-9)
        for direction in (
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ):
            length = mujoco.mj_ray(m, d, p, np.array(direction), groups, 1, -1, hit)
            assert length == pytest.approx(0.07125, abs=1e-9)


def test_four_vehicles_settle_in_assigned_slots(scene):
    m, d = scene
    assert m.nq == 28 and m.nv == 24 and m.nu == 16
    assert d.ncon == 0
    mujoco.mj_step(m, d, nstep=2000)
    mujoco.mj_forward(m, d)
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert np.max(np.abs(d.qvel)) < 0.01
    assert np.all(d.ctrl == 0)
    for i, p in enumerate(dock.slot_centers().values()):
        assert np.allclose(d.qpos[i * 7 : i * 7 + 2], p[:2], atol=0.001)
        assert d.qpos[i * 7 + 2] == pytest.approx(0.18, abs=0.002)
    assert d.ncon > 0


def test_vehicle_envelope_and_propeller_clearance(scene):
    cfg = json.loads(Path("configs/drone.json").read_text())
    assert cfg["outer_width_m"] == 0.12
    assert cfg["mass_kg"] == 0.18
    assert cfg["rotor_center_offset_m"] + cfg["propeller_diameter_m"] / 2 < 0.056
    assert (0.1425 - cfg["outer_width_m"]) / 2 == pytest.approx(0.01125)


def test_old_large_propeller_geometry_rejected():
    cfg = json.loads(Path("configs/drone.json").read_text())
    cfg["propeller_diameter_m"] = 0.062
    root = ET.fromstring("<mujoco><worldbody/></mujoco>")
    with pytest.raises(ValueError, match="Propellers overlap"):
        drone.add(root, "bad", [0, 0, 0], [1, 0, 0, 0], cfg)
