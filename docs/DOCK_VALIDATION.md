# Dock integration evidence — 2026-09-12

Source: `rm_uav_lab_m600/cad_raw/dock.STEP`, SHA-256
`1df1750e198d7f7012ba3489cc094cba63bbb179142dca64d2f1968c354ea72f`.
OpenCASCADE 8.0.1 via cadquery-ocp 8.0.1.0.0; MuJoCo 3.12.0.

CAD import succeeded, solid validity passed, 26 planar faces inspected. CAD
bounds: X [-150,150], Y [0,200], Z [-150,150] mm. Up is CAD +Y.
The dock is a solid lower block with four shallow top cavities, not an empty
box containing modeled electronics or doors. External shape: 300 × 300 × 200 mm.
Four cavities: 142.5 × 142.5 mm, 40 mm deep, 5 mm perimeter/divider walls.

Visual mesh: CAD tessellation, 76 triangles. Collision: eight disjoint boxes.
CAD volume 0.014750999999999995 m³; collision partition volume
0.014751000000000002 m³. All 6,000 seeded occupancy samples agreed. Finite
sampling is not a universal geometric equivalence proof; it complements the
planar-face audit, analytic partition and volume check. Source hash prevents
unreviewed reuse of this partition after a CAD update.

Development field `67c1327`: dock bottom center is
(0.651751442,1.186726189,1.58) m, aligned to the red H platform at yaw 45°.
Platform collision top is 400 × 400 mm. The dock has 50 mm margin on each side.
Four slot floor centers at Z=1.74 m:

| Slot | World X | World Y |
|---|---:|---:|
| 1 | 0.651751442 | 1.082427939 |
| 2 | 0.547453192 | 1.186726189 |
| 3 | 0.756049692 | 1.186726189 |
| 4 | 0.651751442 | 1.291024439 |

Four separate 180 g rigid bodies, 28 qpos, 24 qvel, 16 motor actuators. Combined
scene: 885 geoms. No controller attached; all motor commands remain zero.
120 mm guard envelope fits with 11.25 mm nominal side clearance. Propeller
centers ±30 mm, disk radius 25.4 mm, 0.6 mm nominal horizontal clearance to
the inner guard rails; manufacturing, flex and aerodynamic margins are untested.

Full-field unpowered test: 2,500 RK4 steps at 2 ms (5 s), initial 0 contacts,
final 48 contacts. All positions/velocities finite; maximum absolute generalized
velocity 5.07e-13; zero measured XY drift for all four vehicles. Final body
height 1.75998483 m. Maximum final single-contact force 0.14715 N.
The reproducible `check_scene` gate also measures the entire settling transient:
maximum single-contact force 0.373272 N and maximum penetration 0.768522 mm.
Evidence is saved in `artifacts/dock-final/settling.json`.
This is resting support validation, not a docking/landing or motor-on test.

Focused tests: four passed, covering independent floor/wall rays for all slots,
four-body settling, geometry clearances and rejection of oversized propellers.
Default environment auto-loads an unrelated ROS pytest plugin requiring yaml;
tests intentionally disable third-party plugin autoload.

Reproduction from root:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27 build-dock --output artifacts/new-dock
/home/shiuhou/venvs/mujoco/bin/python -m rm27.check_scene artifacts/new-dock/scene.xml
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.render artifacts/new-dock/scene.xml
```

Inspected render: `artifacts/dock-v2/views.png` (close view, top view, H platform,
whole field). Failed pre-output audit attempt encountered a non-UTF8 STEP
product label; reading ASCII unit tokens from bytes fixed that without changing
CAD. Initial 32 mm rotor offset intersected the guard envelope and was rejected;
the explicit provisional offset is now 30 mm. Original sources are untouched.

Subsequent evidence is recorded separately: [base visual audit](BASE_AUDIT.md)
and [micro-UAV SITL takeoff, hover and return-to-slot flight](MICRO_FLIGHT.md).
Armor/marker coordinates and mission-corridor validation remain unresolved.
The older G1 flight evidence is unchanged and does not validate this new
vehicle or scene.
