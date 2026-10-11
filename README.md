# RM27 autonomous micro-UAV development

日常啟動與查看指令：[START_HERE.md](START_HERE.md)（繁體中文速查表）。

PX4 v1.17.0 migration implementation, actual simulation results and remaining
gates: [PX4_MIGRATION_STATUS.md](docs/PX4_MIGRATION_STATUS.md). The independent
PX4 worktree contains the 150 g target model, HIL bridge, estimator-guided
micro-flight, contact recovery, rendered image inputs and native OpenVINS
evaluation. Full visual mission qualification remains in progress.

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

## Current integration

- `configs/scene.json`: field revision, named H platform, dock placement and
  sequential mission contract.
- `configs/drone.json`: user-reported 1103 9600KV, 2-inch props, approximately
  180 g; provisional 120 × 120 mm envelope. Unmeasured dynamics are labeled.
- `configs/real_hardware.json`: current MicoAir flight controller, selected PX4 v1.17.0
  firmware and sensor inventory, with unresolved installation/log fields left explicit.
- `rm27/`: CAD audit, dock conversion, scene composition, preview and single-vehicle flight gate.
- `tests/`: dock geometry, four-vehicle settling, isolated flight-interface,
  live-view state-isolation, calibration, contact, route and policy checks (104 passing).
- `artifacts/`: generated models, renders and local evidence (ignored).

The dock is placed on the red H platform by default. Four separate physical
vehicle proxies occupy its slots. A separate [single-vehicle flight gate](docs/MICRO_FLIGHT.md)
uses real ArduPilot SITL for takeoff, hover, return and landing, with simulation
truth assisting precision return. The scene builder itself is a static preview.
A separate Phase 4 research experiment now reaches physical base contact and recovery through actual ArduPilot SITL with truth-assisted ego state; EKF-only contact still fails. This does not complete the accepted dock-to-dock mission or four-instance integration.
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
# PX4 v1.17.0 closed-loop SITL (explicit backend)
python -m rm27 micro-flight artifacts/px4-dock-150g-01/scene.xml \
  --autopilot px4 --dynamics configs/dynamics_150g.json \
  --output artifacts/my-px4-flight --instance 0
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

The [whole-maneuver study](docs/WHOLE_MANEUVER_REPORT.md) jointly optimizes the
airborne dock-to-contact trajectory and recovery parameters, including early
thrust-direction preparation. It compares total time to held recovery against
the staged mission and transit-only time compression on the same RMUC plant.

The separate [aggressive contact/recovery laboratory](rm27/experiments/aggressive_contact/README.md)
executes continuous trajectories, physical bottom-bumper contact and recovery on
the root MuJoCo vehicle model, with paired parameter-search benchmarks. Its
[arena mission and v2 report](docs/AGGRESSIVE_CONTACT_V2_REPORT.md) reuse the
existing RMUC field and dock: airborne departure, blue-base contact, recovery,
and return near the dock, with a MuJoCo viewer. It uses truth state and direct
motor control; it does not advance the ArduPilot S2–S5 or real-flight acceptance gates.

当前研究新增 [Whole-Maneuver Teacher 报告](docs/WHOLE_MANEUVER_TEACHER_REPORT.md)：
完整场地三档连续进场／接触／恢复、同植物返航、240 次新留出比较、rate/thrust 代理与
320 条可审计 demonstrations。balanced 在本次 small/stress 各 24/24 通过，约 6.1 秒恢复确认；
仍为 truth-assisted MuJoCo 研究，未验证 SITL／视觉／真机。观看命令见 [START_HERE](START_HERE.md)。

[Whole-Maneuver Distillation](docs/WHOLE_MANEUVER_DISTILLATION_REPORT.md) 已实现并训练 BC、history、
纠偏数据和混合 student；纯动作网络在本轮未成功，learned reference + model feedback 在独立仿真测试中
保持主要 teacher 性能。包含可回放完整任务、checkpoint、数据划分、延迟／噪声 ablation 与 host runtime。

### Hybrid student Phase 3

[Architecture, ablations and deployment report](docs/HYBRID_STUDENT_DEPLOYMENT_REPORT.md):
1,698 new maneuver evaluations and a full-field round trip. On 72 fresh paired
trials, noisy 20 ms ego delay improves from 16/72 to 68/72 through causal state
propagation, analytic reference rates and model feedback. Small-noise success
remains 66/72; no claim of universal improvement. All 11 newly trained direct BC,
history and corrective MLP variants still fail their closed-loop validation.
The deployment API accepts timestamped estimates/covariance and emits body
rates + physical thrust; the MAVLink adapter is an offline prototype, with no
ArduPilot firmware or hardware validation. Root tests: 91 passed.

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3 mission --config configs/contact_phase3/deployment_selected.json --output artifacts/phase3-mission-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase3-mission-01 --speed 1 --loop
```

The second command plays the actual recorded full-field mission. It does not
start a new closed-loop flight. [EGL motion GIF](artifacts/phase3-render-01/maneuver.gif).

### Hybrid policy Phase 4: actual firmware integration

[ArduPilot integration and estimator report](docs/HYBRID_POLICY_SITL_REPORT.md):
actual Guided/attitude/PID/mixer control on the existing full-field MuJoCo plant.
One nominal balanced maneuver passes with privileged ego state after disabling
firmware input shaping (7.47 mm contact error, 23.03 mm margin); the paired EKF
telemetry run hits the rim (82.72 mm error). This separates interface shaping
from the remaining localization gap. SITL uses synthetic GPS, which is not the
intended onboard navigation source. Six new interface tests pass; a final root
rerun and further SITL validation were blocked by an execution-environment fault.
No hardware validation or RL. Recorded flight viewing commands are in START_HERE.

### Estimator-aware maneuver Phase 5

[Estimator requirements, commitment and non-GPS EKF report](docs/ESTIMATOR_AWARE_HYBRID_MANEUVER_REPORT.md):
254 main MuJoCo episodes and 32 actual SITL flights. A synthetic external-navigation
source now feeds real ArduPilot EKF with GPS disabled; the independent sensor-seed
holdout passes 9/12 contact/recovery tasks. This is not a camera/VIO implementation.
A paired commitment guard avoids 12 uncertain attempts but sacrifices task
availability and cannot detect an unreported localization bias. Relative frame
alignment and persistent velocity/tilt errors matter more than position RMSE alone.
The combined-error validation passes 16/16 under its explicit simulation inputs.
Root tests: 104 passed. [Recorded non-GPS SITL maneuver](artifacts/phase5-render-01/maneuver.gif).
