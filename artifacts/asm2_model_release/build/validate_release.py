"""Headless regression of the authoritative aircraft and its portable actuated models."""

from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from load_aircraft import REFERENCE, load_detailed
from rm27_asm2.asset import ROOT, digest, load_inputs, read_json
from rm27_asm2.flight import load_profile
from rm27_asm2.flight_native import _verify_compiled
from rm27_asm2.flight_suite import rotation_matrix
from rm27_asm2.proxy import load_proxy


def main():
    _, _, geometry, mass = load_inputs()
    profile = load_profile()
    plant = read_json(ROOT / "models/plant.json")
    original = mujoco.MjModel.from_xml_path(str(REFERENCE))
    detailed = load_detailed()
    flight = mujoco.MjModel.from_xml_path(str(ROOT / "models/free_flight.xml"))
    entries = geometry["entries"]
    assert len(entries) == 11874
    assert (original.ngeom, original.nmesh, original.nu) == (11888, 11888, 0)
    assert (detailed.ngeom, detailed.nmesh, detailed.nu, detailed.na) == (11888, 11888, 4, 4)
    assert np.all(detailed.actuator_dyntype == mujoco.mjtDyn.mjDYN_FILTEREXACT)
    assert not np.any(detailed.actuator_actearly)
    np.testing.assert_array_equal(detailed.actuator_dynprm[:, 0], profile.motor_time_constant_s)
    for model in (original, detailed):
        body = model.body("drone")
        axes = rotation_matrix(body.iquat)
        np.testing.assert_allclose(body.mass, profile.mass_kg, rtol=0, atol=1e-12)
        np.testing.assert_allclose(body.ipos, plant["com_body_m"], rtol=0, atol=1e-12)
        np.testing.assert_allclose(axes @ np.diag(body.inertia) @ axes.T, profile.inertia_kg_m2,
                                   rtol=0, atol=1e-10 if model is original else 1e-12)
        assert len({model.geom(entry["name"]).id for entry in entries}) == len(entries)
    np.testing.assert_array_equal(plant["inertia_body_kg_m2"], profile.inertia_kg_m2)
    np.testing.assert_array_equal(plant["rotor_positions_body_m"], profile.hubs_m)
    np.testing.assert_array_equal(plant["rotor_axes_body"], profile.axes)
    for i in range(4):
        np.testing.assert_array_equal(detailed.site(f"rotor_{i}").pos, profile.hubs_m[i])
        np.testing.assert_array_equal(detailed.actuator_gear[i, :3], profile.axes[i])
        np.testing.assert_array_equal(detailed.actuator_gear[i, 3:],
                                      profile.yaw_torque_per_thrust_m[i]*profile.axes[i])
    probes = _verify_compiled(flight, profile)

    transform = np.asarray(mass["cad_to_body"])
    center = np.asarray(mass["cad_com_mm"])*.001
    total, weighted_com, inertia = mass["extra_mass_kg"], np.zeros(3), np.zeros((3, 3))
    for part in mass["parts"]:
        position = transform @ (np.asarray(part["center_m"])-center)
        np.testing.assert_allclose(position, part["center_body_m"], atol=1e-12, rtol=0)
        total += part["mass_kg"]
        weighted_com += part["mass_kg"]*position
        inertia += transform @ np.asarray(part["inertia_kg_m2"]) @ transform.T
        inertia += part["mass_kg"]*((position @ position)*np.eye(3)-np.outer(position, position))
    np.testing.assert_allclose(total, profile.mass_kg, atol=1e-12, rtol=0)
    np.testing.assert_allclose(weighted_com/total, 0., atol=1e-12, rtol=0)
    np.testing.assert_allclose(inertia, profile.inertia_kg_m2, atol=1e-12, rtol=0)

    patches = [entry for entry in entries if entry["group"] != "body"]
    assert patches == plant["contact_patches"]
    assert Counter(row["group"] for row in patches) == {"contact_front": 14, "contact_rear": 1}
    proxy = load_proxy()
    assert len(proxy["entries"]) == 3521
    assert all(row["name"] != "collision_0_570" for row in proxy["entries"])
    assert [row["group"] for row in entries if row["name"] == "collision_0_570"] == ["body"]
    mjcf = ET.parse(REFERENCE)
    assets = {element.attrib["name"]: element.attrib["file"] for element in mjcf.findall("./asset/mesh")}
    geoms = {element.attrib["name"]: element.attrib["mesh"]
             for element in mjcf.findall("./worldbody/body/geom")}
    for patch in patches:
        assert assets[geoms[patch["name"]]] == patch["mesh"]
    receipt = read_json(ROOT / "evidence/phase4r2/backend-receipt.json")
    overlay = ROOT / ".agents/runs/asm2-terminal/phase4r2/proxy-with-reference-body.usda"
    assert digest(overlay) == receipt["asset_sha256"]
    print(f"MuJoCo {mujoco.__version__}, NumPy {np.__version__}")
    print(f"Original and actuated detail: {len(entries)} colliders, {original.ngeom-len(entries)} visuals")
    print(f"Patch IDs verified: {', '.join(row['name'] for row in patches)}")
    print(f"CAD tensor reconstruction max error: {np.max(np.abs(inertia-profile.inertia_kg_m2)):.6g} kg m^2")
    print(probes)


if __name__ == "__main__":
    main()
