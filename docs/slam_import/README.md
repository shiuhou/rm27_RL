# RM27 autonomous micro-UAV development

日常啟動與查看指令：[START_HERE.md](START_HERE.md)（繁體中文速查表）。

RoboMaster 2027 simulation and research workspace. The current priority is one
vehicle repeatedly completing dock → blue base armor impact → recovery → assigned
dock slot round trips. Quantity expands only after single-vehicle acceptance;
see [the milestone gates](docs/MILESTONES.md). The green light above
the armor is the 520 nm dart-guidance lamp used as a localization marker. The
research mission includes stabilizing after a potentially greater-than-10 m/s
armor-normal impact component and returning to dock; this value comes from the
RMUC 2026 42 mm detection rule, while the final RM2027 aerial-robot damage
threshold remains unknown. Interception is a later milestone.

The root project owns integration and experiment configuration; existing
simulator and upstream repositories remain independently versioned. Read the
[integration contract and status](docs/INTEGRATION.md) first. The full directory
ownership and migration map is in [docs/REPOSITORY_MAP.md](docs/REPOSITORY_MAP.md).
Evidence: [dock validation](docs/DOCK_VALIDATION.md) and
[preliminary base CAD inventory](docs/BASE_AUDIT.md).
Competition constraints and source hashes are tracked in the
[rulebook baseline](docs/RULEBOOK_BASELINE.md).
The base audit also includes commands for a separate static CAD preview.
Current real-aircraft parts and pending measurements are tracked in the
[real-hardware baseline](docs/REAL_HARDWARE.md). `docs/archive/chatgptre.md` is retained as
historical conversation context only and is not a current specification.

The published SLAM/VIO research integration is maintained on the local review
branch `codex/integrate-slam-3ea1e16` and is indexed in `HANDOFF.md`. It remains
offline research code: its reports do not qualify M3C calibration, live VIO,
PX4 fusion or flight readiness.

## Current integration

- `configs/scene.json`: field revision, named H platform, dock placement and
  sequential mission contract.
- `configs/drone.json`: user-reported 1103 9600KV, 2-inch props, approximately
  180 g; provisional 120 × 120 mm envelope. Unmeasured dynamics are labeled.
- `configs/real_hardware.json`: current MicoAir flight controller and sensor
  inventory, with unresolved installation/log fields left explicit.
- `rm27/`: CAD audit, dock conversion, scene composition, preview and single-vehicle flight gate.
- `tests/`: dock geometry, four-vehicle settling, isolated flight-interface and
  live-view state-isolation, calibration, contact and route checks (31 passing).
- `artifacts/`: generated models, renders and local evidence (ignored).

The dock is placed on the red H platform by default. Four separate physical
vehicle proxies occupy its slots. A separate [single-vehicle flight gate](docs/MICRO_FLIGHT.md)
uses real ArduPilot SITL for takeoff, hover, return and landing, with simulation
truth assisting precision return. The scene builder itself is a static preview.
Base-contact missions and four ArduPilot instances are not implemented.
A separate [provisional base target](docs/BASE_TARGET.md) adds a green marker
and collision test patch to the existing field base without replacing it.
The [S0 batch report](docs/S0_REPORT.md) records ten independent passing dock
flights. [Calibration workflow](docs/CALIBRATION_WORKFLOW.md) provides explicit
simulation dynamics profiles, read-only DataFlash intake and contact probes.
The [staging-route experiment](docs/CORRIDOR_REPORT.md) now reaches a point
in front of the base and returns to the dock using truth-assisted SITL guidance.
It does not include armor impact or visual navigation.

```bash
python -m pip install -r requirements-integration.txt
python -m rm27 build-dock --output artifacts/my-dock-scene
python -m rm27 view artifacts/my-dock-scene/scene.xml
python -m rm27.integration.check_scene artifacts/my-dock-scene/scene.xml
MUJOCO_GL=egl python -m rm27.simulation.render artifacts/my-dock-scene/scene.xml
python -m rm27 micro-flight artifacts/my-dock-scene/scene.xml --output artifacts/my-flight --instance 31
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

Add `--view` to `micro-flight` for a live MuJoCo window with flight status.
Viewing defaults to requested 1× speed; headless runs default to 5×.
Use `--speedup` to override. Closing the live window cancels the test.

Use a new build output directory. This machine uses
`/home/shiuhou/venvs/mujoco/bin/python`. CAD source expected at
`references/cad_raw/dock.STEP`; raw CAD is local and excluded from Git.
Generated XML references sibling field assets by absolute path and must be
rebuilt after moving the workspace. The converter deliberately rejects
unreviewed geometry changes rather than silently reusing obsolete colliders.

## Components and baselines

| Path | Role |
|---|---|
| `rm27/autopilot/baselines/mujoco_uav_ardupilot` | Verified G1 ArduPilot–MuJoCo baseline, commit `2369d7d` |
| `rm27/simulation/field/rmuc2026_battlefield_mjcf` | Upstream field; development scene pins `67c1327` |
| `rm27/simulation/legacy/mujoco_uav` | Standalone M0–M4 vehicle/control experiments, source-only |
| `research/astra` | Independent observation-only terminal-guidance research |
| `rm27/perception/` | Unified sensing layer: visual SLAM/localization, camera calibration, target recognition, guidance and simulator producers |
| `rm27/perception/vision/` | M3C/OS04A10 camera/SLAM work and device-side MaixCAM2 green-light and armor recognition |
| `docs/MIGRATION_PROVENANCE.md` | Source commits and migration records for code now under `rm27/` |
| `rm27/experiments/legacy_sitl` | Historical standalone SITL mission scripts; current root missions live under `rm27/` |

G1's historical field was `dcf1202`; its results do not certify the updated
field or the new 180 g vehicle. Each asset/controller version requires its own
regression evidence. The canonical ArduPilot dependency is under
`rm27/autopilot/vendor/`; archived PX4 and historical workspaces live outside
the repository root in `../rm27_drones_legacy/`.

For a new checkout, initialize needed components with `git submodule update --init`
and install the integration dependencies above. Ensure the field checkout is
at the revision pinned by `configs/scene.json`; builds fail on a mismatch.
Raw CAD and the rules,
photo and video supplied locally are separate source inputs; do not assume a
clone alone includes them. No hardware actuation is provided by the root CLI.

Existing research: [Astra](research/astra/README.md), [research journal](research/astra/docs/research_log.md),
[hardware evidence](research/astra/docs/hardware.md), [real experiment protocol](research/astra/docs/real_experiments.md).
