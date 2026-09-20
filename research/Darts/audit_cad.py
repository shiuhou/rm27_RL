"""Read-only STEP audit and tessellation; outputs stay below research/Darts/artifacts."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def export(source, output):
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    import trimesh

    source, output = Path(source).resolve(), Path(output).resolve()
    digest = sha256(source)
    if b'SI_UNIT ( .MILLI., .METRE. )' not in source.read_bytes():
        raise ValueError('Expected the audited millimetre STEP source')
    output.mkdir(parents=True, exist_ok=False)
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone or not reader.TransferRoots():
        raise ValueError('STEP import failed')
    shape = reader.OneShape()
    bound = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, bound)
    bounds = np.array([bound.CornerMin().Coord(), bound.CornerMax().Coord()])
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    solids = []
    while explorer.More():
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(explorer.Current(), box)
        solids.append([box.CornerMin().Coord(), box.CornerMax().Coord()])
        explorer.Next()
    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.25, False).Perform()
    if not StlAPI_Writer().Write(shape, str(output / 'cad_mm.stl')):
        raise RuntimeError('STEP tessellation failed')
    mesh = trimesh.load(output / 'cad_mm.stl', force='mesh')
    mesh.vertices *= .001
    mesh.export(output / 'cad_m.obj')
    report = dict(source=str(source), source_sha256=digest, units='mm',
                  bounds_mm=bounds.tolist(), dimensions_mm=np.diff(bounds,axis=0)[0].tolist(),
                  solid_occurrences=len(solids), solid_bounds_mm=solids,
                  triangles=len(mesh.faces), tessellation_linear_tolerance_mm=.15,
                  interpretation='Full placed assembly; no density, inertia or collision inferred')
    (output / 'audit.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).with_name('mech_darts.STEP'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = export(args.source, args.output)
    print(json.dumps({k:v for k,v in report.items() if k != 'solid_bounds_mm'}, indent=2))
