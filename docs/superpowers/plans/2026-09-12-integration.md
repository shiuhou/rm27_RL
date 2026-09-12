# RM27 dock and unified entry implementation plan

**Goal:** A root-owned, reproducible dock scene with replaceable assets and four
parked provisional vehicles; subsequent mission stages stay explicitly separate.

**Architecture:** Preserve subrepositories, compose field and CAD through the
root CLI, record coordinate/geometry assumptions in configuration and reports.

**Tech Stack:** Python, OpenCASCADE, MuJoCo, NumPy, trimesh, pytest.

**Spec:** `docs/INTEGRATION.md` (user accepted root entry + subrepositories and
one complete round trip before the next vehicle).

## Global constraints

- Native field coordinates, metres, +Z up.
- No edits to frozen G1 vehicle or controller.
- Dock CAD geometry is authoritative; fail on an unreviewed changed shape.
- Four physical proxies do not imply working multi-ArduPilot flight.

## Task 1: CAD and placement

- [x] Read dock units, solid validity, bounds and 26 planar faces.
- [x] Implement `rm27/dock.py` eight-box partition with CAD volume and 6,000
  deterministic occupancy comparisons before emitting meshes.
- [x] Implement `python -m rm27 build-dock --output <new-directory>` using
  `configs/scene.json`, named platform placement and absolute asset resolution.
- [x] Validate four cavity floors and walls with independent MuJoCo rays.
- [x] Render actual composed model and inspect platform/slot placement.

## Task 2: Provisional aircraft and stability

- [x] Record user motor, propeller, mass and 120 mm outer envelope separately
  from unmeasured inertia/thrust/lag assumptions in `configs/drone.json`.
- [x] Add four uniquely named free bodies and actuators in `rm27/drone.py`.
- [x] Simulate unpowered settling, assert finite state, slot occupancy,
  contact support and unchanged zero actuator inputs.
- [x] Check camera views and rotor clearance; report no flight validation.

## Task 3: Root handoff

- [x] Update root README with ownership, reproducible commands and status.
- [x] Retain compact verification evidence outside ignored raw artifacts.
- [x] Review diff for accidental source edits and unlicensed asset vendoring.

Next implementation plans depend on inspected base armor geometry and a
validated micro-UAV flight plant. They must implement the successive gates in
`docs/INTEGRATION.md`, not mark them complete from this parked-scene test.
