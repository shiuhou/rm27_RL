"""Load the detailed ASM2 airframe with provisional motor dynamics and no fixture."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from rm27_asm2.asset import ROOT

REFERENCE = ROOT / "assets/asm2/source/mujoco/model.xml"


def detailed_xml(mesh_base: Path | None = None) -> str:
    """Use absolute mesh paths for in-memory loading or paths relative to mesh_base."""
    root = ET.parse(REFERENCE).getroot()
    flight = ET.parse(ROOT / "models/free_flight.xml").getroot()
    for mesh in root.findall("./asset/mesh"):
        source = REFERENCE.parent / mesh.attrib["file"]
        mesh.set("file", str(source) if mesh_base is None else os.path.relpath(source, mesh_base))
    option = flight.find("option")
    option.set("timestep", "0.0001")
    root.insert(1, option)
    body = root.find("./worldbody/body[@name='drone']")
    flight_body = flight.find("./worldbody/body[@name='drone']")
    body.remove(body.find("inertial"))
    body.insert(0, flight_body.find("inertial"))
    body.extend(flight_body.findall("site"))
    root.append(flight.find("actuator"))
    return ET.tostring(root, encoding="unicode")


def load_detailed() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(detailed_xml())
