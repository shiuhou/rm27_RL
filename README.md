# RM27 autonomous micro-UAV development

日常啟動與查看指令：[START_HERE.md](START_HERE.md)（繁體中文速查表）。

RoboMaster 2027 simulation and research workspace. The near-term mission is
four vehicles completing sequential dock → blue base armor contact → assigned
dock slot round trips, with only one vehicle airborne. The green light above
the armor is the localization marker. Interception is a later milestone.

The root project owns integration and experiment configuration; existing
simulator and upstream repositories remain independently versioned. Read the
[integration contract and status](docs/INTEGRATION.md) first.
Evidence: [dock validation](docs/DOCK_VALIDATION.md) and
[preliminary base CAD inventory](docs/BASE_AUDIT.md).
The base audit also includes commands for a separate static CAD preview.
Current real-aircraft parts and pending measurements are tracked in the
[real-hardware baseline](docs/REAL_HARDWARE.md). `chatgptre.md` is retained as
historical conversation context only and is not a current specification.

## Current integration

- `configs/scene.json`: field revision, named H platform, dock placement and
  sequential mission contract.
- `configs/drone.json`: user-reported 1103 9600KV, 2-inch props, approximately
  180 g; provisional 120 × 120 mm envelope. Unmeasured dynamics are labeled.
- `configs/real_hardware.json`: current MicoAir flight controller and sensor
  inventory, with unresolved installation/log fields left explicit.
- `rm27/`: CAD audit, dock conversion, scene composition, preview and single-vehicle flight gate.
- `tests/`: dock geometry, four-vehicle settling, isolated flight-interface and
  live-view state-isolation checks (14 currently passing).
- `artifacts/`: generated models, renders and local evidence (ignored).

The dock is placed on the red H platform by default. Four separate physical
vehicle proxies occupy its slots. A separate [single-vehicle flight gate](docs/MICRO_FLIGHT.md)
uses real ArduPilot SITL for takeoff, hover, return and landing, with simulation
truth assisting precision return. The scene builder itself is a static preview.
Base-contact missions and four ArduPilot instances are not implemented.
A separate [provisional base target](docs/BASE_TARGET.md) adds a green marker
and collision test patch to the existing field base without replacing it.

```bash
python -m pip install -r requirements-integration.txt
python -m rm27 build-dock --output artifacts/my-dock-scene
python -m rm27 view artifacts/my-dock-scene/scene.xml
python -m rm27.check_scene artifacts/my-dock-scene/scene.xml
MUJOCO_GL=egl python -m rm27.render artifacts/my-dock-scene/scene.xml
python -m rm27 micro-flight artifacts/my-dock-scene/scene.xml --output artifacts/my-flight --instance 31
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

Add `--view` to `micro-flight` for a live MuJoCo window with flight status.
Viewing defaults to requested 1× speed; headless runs default to 5×.
Use `--speedup` to override. Closing the live window cancels the test.

Use a new build output directory. This machine uses
`/home/shiuhou/venvs/mujoco/bin/python`. CAD source expected at
`rm_uav_lab_m600/cad_raw/dock.STEP`; raw CAD is local and excluded from Git.
Generated XML references sibling field assets by absolute path and must be
rebuilt after moving the workspace. The converter deliberately rejects
unreviewed geometry changes rather than silently reusing obsolete colliders.

## Components and baselines

| Path | Role |
|---|---|
| `rm_uav_lab_m600/mujoco_uav_ardupilot` | Verified G1 ArduPilot–MuJoCo baseline, commit `2369d7d` |
| `rm_uav_lab_m600/rmuc2026_battlefield_mjcf` | Upstream field; development scene pins `67c1327` |
| `rm_uav_lab_m600/mujoco_uav` | Standalone M0–M4 vehicle/control experiments |
| `astra` | Independent observation-only terminal-guidance research |
| `rm_uav_lab_m600/maixcam2_dart` | Camera detection/tracking reference |
| `rm_uav_lab_m600/visual_guidance` | Existing guidance-interface experiments |

G1's historical field was `dcf1202`; its results do not certify the updated
field or the new 180 g vehicle. Each asset/controller version requires its own
regression evidence. The local `ardupilot/` and `rm_uav_lab/` reference workspaces
remain excluded from root commits.

For a new checkout, initialize needed components with `git submodule update --init`
and install the integration dependencies above. Ensure the field checkout is
at the revision pinned by `configs/scene.json`; builds fail on a mismatch.
Raw CAD and the rules,
photo and video supplied locally are separate source inputs; do not assume a
clone alone includes them. No hardware actuation is provided by the root CLI.

Existing research: [Astra](astra/README.md), [research journal](astra/docs/research_log.md),
[hardware evidence](astra/docs/hardware.md), [real experiment protocol](astra/docs/real_experiments.md).
