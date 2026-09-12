# Base STEP preliminary inventory

Source: `rm_uav_lab_m600/cad_raw/base2.STEP`, SHA-256
`1a63c97fe67e7e0c140c7a1016a1d6659e53e2b976f1a9e6e2324dd902656539`.
The original CAD is unchanged. No whole-field STEP was used.

OpenCASCADE imports one root containing 236 placed solid occurrences. Splitting
by the sign of each solid bounding-box center's X coordinate gives two groups
of 118. No solid crosses X=0. Both groups have the same bounding dimensions,
1609.005 × 1880.918 × 1371.119 mm, in the imported CAD axes. These are geometric
groups, not yet verified red/blue team labels or a verified geographic frame.

| Group | Minimum XYZ (mm) | Maximum XYZ (mm) |
|---|---|---|
| Negative X | (-12196.458, -949.996, -692.031) | (-10587.453, 930.921, 679.088) |
| Positive X | (10885.177, -949.997, -692.031) | (12494.182, 930.921, 679.088) |

The complete root spans 24690.640 mm in X and reaches Z=692.026 mm, beyond
the solid-only maximum. The face traversal excluding solid descendants counts
21,918 face occurrences outside solids. Occurrence counts are not counts of
unique physical parts. A solid-only export would therefore omit source
surfaces. The file must not be uniformly shrunk to make the entire assembly
look like a single base.

This inventory does not yet identify the photograph's green lamp or blue armor,
establish upward direction, provide collision geometry, or certify a flight
corridor. The separate assembly preview described below includes the tessellated
non-solid surfaces. No base was inserted into the integrated field at this stage.

Reproduce from the project root, using a new report filename:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.audit_base rm_uav_lab_m600/cad_raw/base2.STEP artifacts/base-audit/new-inventory.json
```

Current machine-readable evidence: `artifacts/base-audit/complete-inventory.json`.

## Static visual preview

`rm27.preview_base` tessellates the complete imported shape, including surfaces
outside solids, with 0.5 mm linear tolerance. Meshes use metres and preserve
the CAD axes and relative placements. The X=0 split is accepted only when no
triangle crosses it. Local mesh centering is compensated by body translation.
The two gray geoms are static and have collision disabled. CAD colors and
semantic labels are not recovered.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.preview_base rm_uav_lab_m600/cad_raw/base2.STEP artifacts/new-base-preview
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.preview_base artifacts/new-base-preview/scene.xml --render-only
/home/shiuhou/venvs/mujoco/bin/python -m rm27 view artifacts/new-base-preview/scene.xml
```

The first export exceeded MuJoCo's 200,000-face limit for an individual STL.
The preview therefore uses OBJ for the final meshes. The intermediate STL is
read by trimesh, which does not impose that MuJoCo decoder limit. OpenCASCADE
also reported 68 faces skipped for null triangulation. The preview cannot be
claimed surface-complete, even though its input includes the whole assembly.
Neither source CAD nor the dock/field scene is modified by this command.

Verified output: `artifacts/base-preview-v2/scene.xml` compiles under MuJoCo
3.12.0 with two static mesh geoms, zero collision geoms and no generalized
coordinates. Each group contains 1,267,269 triangles (2,534,538 total). Four
EGL-rendered views in `artifacts/base-preview-v2/views.png` were inspected.
Both groups display a tower on a broad plinth in the retained CAD orientation;
this visual observation does not establish geographic axes or team identity.
The source has overlapping/detail surfaces visible as mottling in places.
The final meshes are large (approximately 324 MiB combined OBJ); this is a CAD
inspection asset, with no real-time field rendering benchmark established.
