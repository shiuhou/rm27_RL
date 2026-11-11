"""Immutable geometry contract for the isolated ASM2 collision proxy."""

from pathlib import Path

from .asset import ROOT, digest, read_json


SOURCE = ROOT / "assets/asm2/source"
PROXY = ROOT / "assets/asm2/proxy/manifest.json"
GEOMETRY = SOURCE / "metadata/collision_geometry.json"
SCHEMA = "rm27_asm2_collision_proxy_v1"


def load_proxy(path=PROXY):
    """Resolve and verify the bundle before creating simulator shapes."""
    path = Path(path).resolve()
    manifest = read_json(path)
    if manifest["schema"] != SCHEMA or manifest["training_allowed"] is not False:
        raise ValueError("Expected an ASM2 engineering-only collision proxy")
    for relative, expected in manifest["source_pins"].items():
        source = (SOURCE / relative).resolve()
        if not source.is_relative_to(SOURCE) or digest(source) != expected:
            raise ValueError(f"ASM2 proxy source changed: {relative}")
    original = {
        entry["name"]: entry for entry in read_json(GEOMETRY)["entries"]
        if entry["group"] != "body"
    }
    if len(original) != 15 or manifest["source_pins"].get(
        "metadata/collision_geometry.json"
    ) != digest(GEOMETRY):
        raise ValueError("Missing source contact-patch identity pin")
    seen = set()
    for entry in manifest["entries"]:
        name = entry["name"]
        group = entry["group"]
        mesh = (path.parent / entry["mesh"]).resolve()
        if name in seen or not mesh.is_relative_to(path.parent) or digest(mesh) != entry["sha256"]:
            raise ValueError(f"Invalid or modified proxy mesh: {name}")
        seen.add(name)
        if name in original:
            frozen = original.pop(name)
            if (group != frozen["group"] or entry["source_part"] != frozen["source_part"]
                    or entry["sha256"] != digest(SOURCE / "mujoco" / frozen["mesh"])):
                raise ValueError(f"Source contact patch was altered: {name}")
        elif group != "body" or not name.startswith("collision_proxy_"):
            raise ValueError(f"Unexpected ASM2 proxy collider: {name}")
    if original or not any(entry["group"] == "body" for entry in manifest["entries"]):
        raise ValueError("Incomplete ASM2 proxy geometry")
    return manifest
