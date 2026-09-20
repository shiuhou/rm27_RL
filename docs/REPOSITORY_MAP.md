# Repository map

This repository is one RM27 research project. The root integration package is
the main runtime entry point; the other directories provide research, external
components, source evidence, or generated results.

| Path | Responsibility | Owner | Independent Git repository | Movable? |
|---|---|---|---|---|
| `rm27/` | Unified root integration package: `integration/`, `simulation/`, `autopilot/`, `perception/` and `experiments/` | RM27 root | No | Yes, only with package/import updates |
| `configs/` | Reviewed scene, vehicle, target, hardware and experiment configuration | RM27 root | No | Yes, with config path updates |
| `tests/` | Root unit and integration tests | RM27 root | No | Yes, with test/config path updates |
| `docs/` | Current contracts, reports, milestones and historical archive | RM27 root | No | Yes |
| `rm27/perception/` | Complete RM27 sensing layer: vision (SLAM, camera calibration and target recognition), target estimates, guidance handoff and simulator producers | RM27 root | No | Yes, with import updates |
| `references/cad_raw/` | Raw STEP/SLDASM inputs used by reviewed integration configs | RM27 root | No | Yes, preserve hashes and update config paths |
| `flight_logs/data/` | Read-only historical and real-flight logs, grouped by source | RM27 root | No | Yes, preserve original files and hashes |
| `../rm27_drones_legacy/` | External PX4, old RM27 and standalone ArduPilot reference workspaces | Historical/reference owner | Contains independent repositories | Keep outside this repository |
| `research/astra/` | Observation-only terminal-guidance research | Astra research | No | Yes, as a whole research package |
| `research/Darts/` | Simplified dart trajectory and CAD collision-path experiment | Darts research | No | Yes, as a whole research package |
| `research/se3_radio_starter/` | SE3 controller, MuJoCo and radio-input reproduction package | SE3 research | No | Yes, as a whole research package |
| `rm27/perception/vision/` | Camera-based perception: M3C/OS04A10 calibration, monocular localization/SLAM, green-light and armor/target recognition | RM27 perception | Contains the `maixcam2_dart_vision` Git submodule | Move only as one perception unit; preserve nested Git history |
| `rm27/autopilot/vendor/` | External ArduPilot and Gazebo plugin checkouts used by the root launchers | RM27 autopilot | Yes, Git submodules | Move only with `.gitmodules` and launcher path updates |
| `rm27/autopilot/baselines/` | Frozen G1 ArduPilot–MuJoCo baseline and its historical evidence | RM27 autopilot | Yes, Git submodule | Keep pinned; do not mix with the current vehicle model |
| `rm27/simulation/field/` | Pinned upstream battlefield model used by `configs/scene.json` | RM27 simulation | Yes, Git submodule | Move only with scene and asset path updates |
| `rm27/simulation/legacy/` | Source-only M0–M4 simulator and older controller experiments | Historical/reference owner | Yes, Git submodule | Keep isolated from current simulation interfaces |
| `rm27/experiments/legacy_sitl/` | Historical direct pymavlink mission scripts moved from the M600 lab | RM27 experiments | No | Yes, as an archive; do not use as current mission entry |
| `references/` | Local rulebooks, photographs, CAD sources and external reference material | RM27 root | No | Yes, preserve hashes and update docs |
| `flight_logs/` | Read-only real-flight intake plus grouped historical SITL logs | RM27 root | No | Yes, preserve raw files and hashes |
| `artifacts/` | Ignored generated models, renders, trajectories and reports | RM27 root | No | Yes, after output paths are made configurable |
| `patches/` | Local patches for independently versioned components | RM27 root | No | Yes, with patch paths updated |

## Ownership rule

Current RM27 behavior belongs in `rm27/`, `configs/`, `tests/` and `docs/`.
Research packages remain separate when they are independent studies. The
M3C/OS04A10, camera-calibration, SLAM and target-recognition work is part of
`rm27/perception/vision/` because it defines the project's camera sensing path.
`rm27/perception/camera/` documents the producer boundary; it is not a separate
research project. Third-party source and
historical results keep their original version boundaries.

## Safe reorganization order

1. Update this map and the directory READMEs.
2. Make output paths configurable and index existing artifacts.
3. Migrate reusable code into `rm27/` with tests and provenance records.
4. Move or archive old workspaces only after commands and submodule paths pass.

The detailed code migration sequence is maintained in
[`MIGRATION_PLAN.md`](MIGRATION_PLAN.md).

Do not infer a clean repository from this map. Check root and nested Git status
separately before moving or deleting anything.
