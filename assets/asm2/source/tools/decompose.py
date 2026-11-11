"""Cache convex collision pieces for each current body-frame visual mesh."""
from pathlib import Path
import hashlib
import json
import numpy as np
import trimesh
import coacd

ROOT = (Path(__file__).resolve().parents[1] / 'mujoco')
ELECTRONICS_ENVELOPES = {1, 4, 5, 12, 13}

def decompose():
    coacd.set_log_level('warn')
    records = json.loads((ROOT / '../metadata/mass_properties.json').read_text())['parts']
    folder = ROOT / 'collision_meshes'
    folder.mkdir(exist_ok=True)
    manifest = []
    for index, part in enumerate(records):
        source = ROOT / part['mesh']
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        cache = folder / f'part_{index}.json'
        if cache.exists():
            entries = json.loads(cache.read_text())
            if all(e.get('source_mesh_sha256') == digest for e in entries):
                manifest.extend(entries)
                continue
        mesh = trimesh.load(source)
        print('Decomposing', index, part['name'], len(mesh.faces), flush=True)
        if index in ELECTRONICS_ENVELOPES or mesh.is_convex:
            hull = mesh.convex_hull
            hulls = [(hull.vertices, hull.faces)]
        else:
            hulls = coacd.run_coacd(
                coacd.Mesh(mesh.vertices.astype(np.float64), mesh.faces.astype(np.int32)),
                threshold=(.0015 if index in {9, 10, 11} else .003 if index == 0 else .01),
                preprocess_resolution=300 if index in {9, 10, 11} else 100, resolution=3000,
                mcts_iterations=80 if index == 0 else 20,
                mcts_nodes=15 if index == 0 else 10, mcts_max_depth=3,
                merge=True, seed=42)
        entries = []
        for hull_index, (vertices, faces) in enumerate(hulls):
            hull = trimesh.Trimesh(vertices, faces, process=True)
            path = folder / f'part_{index}_hull_{hull_index}.obj'
            hull.export(path)
            entries.append(dict(name=f'collision_{index}_{hull_index}',
                source_part=index, group='body', mesh=str(path.relative_to(ROOT)),
                source_mesh_sha256=digest))
        cache.write_text(json.dumps(entries, indent=2))
        manifest.extend(entries)
        print('Finished', index, len(entries), flush=True)
    (ROOT / '../metadata/collision_manifest.json').write_text(json.dumps(manifest, indent=2))

if __name__ == '__main__':
    decompose()
