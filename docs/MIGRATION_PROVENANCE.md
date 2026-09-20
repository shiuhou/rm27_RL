# Migration provenance and Git history

The product is unified in the root repository, but source history remains
traceable through a migration record. A directory is considered migrated only
when root code imports it from `rm27/`, its focused tests pass, and its source
commit is recorded here or in the component's migration note.

## History policy

Use a temporary clone for history operations. Never rewrite a dirty component
checkout in place.

1. Record the component URL, source commit, local diff state and file hashes.
2. Extract only the selected paths into a temporary branch with
   `git filter-repo --path ...` (or an equivalent filtered export).
3. Merge that branch into the root repository with
   `--allow-unrelated-histories`, keeping the imported commits under the final
   `rm27/` path.
4. Add the migration commit and test result to this file.
5. Keep the original checkout until the root version has passed review and one
   reproducible run. Then remove the old runtime path; retain raw evidence and
   source commit references.

`git subtree` is also acceptable when it is available, but the imported prefix
must be the final product path. Git submodules are appropriate for third-party
source that is still built in place; they are not the final product structure
for code already owned by the root integration package.

## Recorded source boundaries

| Destination | Source | Source commit | Current state |
|---|---|---|---|
| `rm27/experiments/route_experiment.py` | root `rm27/route_experiment.py` | root working tree | migrated; 35 root tests pass |
| `rm27/experiments/analyze_route.py` | root `rm27/analyze_route.py` | root working tree | migrated; 35 root tests pass |
| `rm27/integration/{dock,drone,base_target}.py` | root `rm27/` modules | root working tree | migrated; 35 root tests pass |
| `rm27/integration/{cad_drone,isolated_scene,check_scene,audit_base,preview_base}.py` | root `rm27/` modules | root working tree | migrated; 35 root tests pass |
| `rm27/simulation/*`, `rm27/experiments/*`, `rm27/autopilot/micro_flight.py` | root `rm27/` modules | root working tree | migrated; 35 root tests pass |
| `rm27/autopilot/bridge/` and `rm27/autopilot/models/quadrotor_m1.xml` | `rm27/autopilot/baselines/mujoco_uav_ardupilot/ardupilot_bridge/` and `models/quadrotor_m1.xml` | `2369d7d207818160e2d140b16118a53f99491b66` | selected bridge and plant copied; root tests pass; SITL equivalence still pending |
| `rm27/perception/{guidance,sitl,gazebo}/` | `rm_uav_lab_m600/visual_guidance/{uav_guidance,v1_sitl,v2_gazebo}/` | `994727e480ec578efff24fbc422088d6c68a7701` | visual guidance interface copied; root tests pass; live Gazebo/SITL run pending |
| `rm27/perception/vision/` | canonical `maixcam2_dart_vision` submodule (formerly `research/vision` plus `rm_uav_lab_m600/maixcam2_dart/`) | `c7d47b20a5a948db1e55f99fb128ee7616d02e1c` (original source `f4c153f`) | SLAM plan and device-side C++ now live under perception; old duplicate checkout removed; board build remains external |

The former source checkouts for visual guidance and MaixCAM2 were removed only
after the migrated imports and root tests passed. Their source commits remain
recorded above. Moving files does not transfer the source claims: G1, PX4 and
old field results retain their original evidence scope.
