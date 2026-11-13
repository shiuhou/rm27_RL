def _verify_frozen_geometry(*, contact_parameters=None):
    """Load the standalone snapshot exported after comparison with the pinned MJB."""
    from .asset import ROOT

    evidence = ROOT / "evidence"
    field = read_json(ROOT / ".agents/runs/rm27-stage1/contact-geometry.json")
    receipt = read_json(evidence / "field-materials.json")
    archive = evidence / "field-hulls.npz"
    provenance = receipt["provenance"]
    if (digest(archive) != receipt["hulls_sha256"]
            or digest(ROOT / ".agents/runs/rm27-stage1/contact-geometry.json") != provenance["geometry_json_sha256"]):
        raise ValueError("Portable Field snapshot changed")
    names = {shape["name"] for shape in field["static"]}
    if names != receipt["contact_parameters"].keys() or len(names) != 1822:
        raise ValueError("Incomplete Field contact material snapshot")
    keys = {shape["mesh"] for shape in field["static"] if shape["mesh"] is not None}
    with np.load(archive, allow_pickle=False) as arrays:
        meshes = {key: (arrays[f"{key}_vertices"], arrays[f"{key}_faces"]) for key in sorted(keys)}
    if contact_parameters is not None:
        contact_parameters.update(receipt["contact_parameters"])
    return field, meshes, provenance
