# ASM2 collision integration — 2026-11-10

**Verdict: runnable collision candidate; NOT ACCEPTED FOR TRAINING. Zero PPO transitions.**

The supplied 142 g aircraft loads in Isaac Lab 2.1 / Isaac Sim 4.5. A separate candidate fixes a demonstrated small-mesh PhysX cooking defect while retaining all 11,874 collider identities. All ten geometry fixtures pass in two fresh eight-world layouts. Acceptance remains blocked by the reproduced guard false positive, first-contact sampling sensitivity, unvalidated PhysX impulse accounting, and the absence of an ASM2 task qualification contract. Loading and fixture classification are not flight or task qualification.

## Inputs and preserved baseline

| Item | Existing Ver2 baseline | New ASM2 candidate |
|---|---|---|
| Mass | 150 g nominal | 142 g CAD estimate |
| Aircraft geometry | 247 shapes, including designated sphere | 11,859 body + 14 front + 1 rear convex shapes |
| Contact semantics | Single underside sphere | Individual front/rear patches; all other aircraft shapes are body |
| Scene hierarchy | `/Stage1Scene/{Field,Drone}` | `/World/ASM2Envs/env_N/{Field,Drone}`; standalone asset root `/Drone` |
| Target scene | 1,822 fixed shapes; 25 registered legal colliders | Same frozen Field component, translated rigidly near the origin for these tests |
| Task scorer | Frozen `Stage1Evaluator`, sphere-specific | Unchanged; ASM2 qualification explicitly undefined |

At the time of this integration audit, the project root had no Git repository. The four `upstream/` repositories were clean and detached at their pinned revisions. Existing `simpleflight_lab.rm27_contact_geometry`, `rm27_contact_scene`, `stage1_sim`, native scorer, configurations and 35 checkpoints are preserved. `Stage1Sim` already decodes per-collider PhysX paths; aggregate body force was never the required semantic identifier.

Source: `assets/asm2/source/`, extracted from the supplied ZIP. ZIP SHA-256:
`96450a99f393c4f8649bbdfbc7052124a662873f8603e0d6a02a0e6e1c71b682`.
All 15,470 manifest entries match. The package USDC is used; the earlier standalone USDC differs and was not substituted. The two supplied MJCF versions match. Package tools were inspected without executing their generation/install instructions.

Candidate: `assets/asm2/candidate/drone.usda`, SHA-256:
`c86f70ac9380670e78907c7d4a0dc605dadd49b6ab6562393941d7bc0555fd50`.
Its relative reference resolves to the unchanged package USDC. Configuration and source pins are in `configs/rm27_asm2_collision.json`; `training_allowed=false`.

## Asset and cooking validation

**CONFIRMED:** metres, Z-up, identity root transform, one rigid body, COM at origin, no second battery transform. Body axes are consistent with the supplied CAD transform and FLU interpretation (+X camera/forward); hardware frame calibration is not established. The supplied battery CAD translation `[0.0414462979379, 0, -2.14273390648]` mm is already baked into the meshes.

Recomputed part masses, parallel-axis terms and full inertia agree with `mass_properties.json`. Body inertia in kg m² is:

```text
 1.1150257838e-4   1.0350512713e-7  -4.8322420247e-6
 1.0350512713e-7   1.0455706507e-4   1.1561894898e-7
-4.8322420247e-6   1.1561894898e-7   1.8709338834e-4
```

PhysX reads mass 0.14200000465 kg and the same full body-frame inertia within floating-point precision. The rotated principal COM frame is not an extra body rotation. These are **CAD_ESTIMATE** properties: RK3576 uses a surface-mass approximation due to defective STEP volume; 4.5 g is represented at the COM. No calibrated actuator, motor or thrust model was supplied or invented.

Source USD points agree with OBJ coordinates within 3.73 nm. Nevertheless, source meshes fail 5/10 PhysX fixture classifications. Thin 50 µm contact patches cook into tetrahedra; sampled bounds expand by approximately 0.7 mm. Source offsets are 10 µm on 3,217 shapes and 100 µm on 8,657, with zero rest offset. Merely using PGS / 1 µm offsets, zero minimum hull thickness and a 255-vertex limit still passes only 5/10.

The effective repair normalizes each collision mesh's local coordinates by 1,000 and applies inverse local scale 0.001. World dimensions, topology, collider names, normals, mass properties and visual geometry remain unchanged; maximum authored coordinate error is 7.25 nm. No collision pieces were removed. Candidate settings are isolated: PGS, 1 ms, 1 µm contact offsets, zero rest offset, minimum hull thickness zero and hull vertex limit 255. Existing production settings are unchanged.

This measured scale sensitivity is consistent with QuickHull tolerances operating in mesh-local coordinates; PhysX documents a default `planeTolerance` of 0.0007. The installed schema separately reports default minimum thickness 0.001 and vertex limit 64. The exact plugin-internal tolerance was not instrumented. [PhysX cooking parameters](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/_build/physx/latest/struct_px_cooking_params.html), [PhysX schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.2/dev_guide/schemas/physxschema.html).

Cooking is still an approximation. Across 19 inspected cooked hulls, maximum axis-support error is 10.73 µm. Across 288 guard rays, maximum native-versus-PhysX hit-distance error improves from 2.223 mm to 55.57 µm. These sampled checks are not a whole-surface Hausdorff bound.

## Contact classifications and simulator comparison

Fixtures are frozen in `.agents/runs/asm2-collision/fixtures.json`. Front/rear probes and simultaneous plane contact intentionally start with 10 µm overlap; they are geometry diagnostics, not valid flight resets. Gravity is zero. Approach probe speed is 0.1 m/s. All failed checks remain in the counts.

| Fixture | Native MuJoCo, 1 ms | Candidate PhysX layout 0 | Candidate PhysX layout 1 |
|---|---:|---:|---:|
| Front only | PASS | 8/8 | 8/8 |
| Rear only | PASS | 8/8 | 8/8 |
| Simultaneous front/rear | PASS | 8/8 | 8/8 |
| Four side/body orientations | 4/4 | 32/32 | 32/32 |
| Clear by 1 mm / 20 µm | 2/2 | 16/16 | 16/16 |
| Descending from 1 mm gap | FAIL: body also reported | 8/8 front/rear | 8/8 front/rear |
| Total fixture classifications | 9/10 | 80/80 | 80/80 |

**Sampling matters:** MuJoCo's first touching sample is at 11 ms (interval ends 12 ms) at 1 ms resolution, with front/rear/body contacts. At 0.1 ms, first touch is sampled at 10.1 ms (interval ends 10.2 ms), front/rear only; body overlap appears at 10.6 ms with zero body force. The 50 µm patch/carrier separation can be crossed within one 1 ms step. No body records are discarded to make a pass. Native refinement is diagnostic only; candidate timestep remains 1 ms. PhysX first touch occurs in local interval 10–11 ms, around zero separation; the JSON uses global physics tick 20 after nine earlier fixtures.

**Matched Field component:** the ASM2 replaces only the aircraft in an in-memory native model, using static hulls verified against the frozen full-field MJB. All 1,822 static identities and 25 registered surfaces remain. Recompiled maximum world-vertex error is 9.93 nm. Both engines report 112 front/rear points, all 15 patches against the same four registered surfaces, with no body/illegal pair. Minimum separations are -30.88 µm native and -30.89 µm PhysX. This one-step overlap fixture is not a continuous approach or full-field episode qualification. The original 398 MB MJB and evaluator remain untouched.

**Event implementation:** `src/rm27_asm2/contacts.py` and `isaac.py` decode each collider path, preserve patch identity, orient normals from obstacle toward aircraft, calculate pre-impact point velocity `v + ω × r`, and retain signed separation, zero impulses and positive-separation reports. Normal and point-velocity sign/lever-arm tests pass. Static obstacles have zero velocity in this suite. Force is explicitly the reported impulse divided by the physics interval. Exact duplicate samples are suppressed only within the same pair/tick; distinct patches and points remain. Actual duplicate counts were 1,814 and 1,605 across the two batches. [PhysX contact-report API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html).

`scorer_input()` is exercised on real fixture/Field events and supplies existing event fields plus `contact_group` and registered-target membership. It deliberately supplies no accepted/pass decision. The frozen sphere scorer cannot qualify the new shell by relabeling events. An ASM2 evaluator/contract must be separately defined before task acceptance.

**Force/impulse limitation — BLOCKED:** the vector from PhysX `p.impulse` is report evidence, not a validated total contact force. Front/rear probe normal impulses match normal momentum changes, but tangential friction is absent from those vectors; some vector residual is therefore missing friction. A separate normal overcount remains: in the Field fixture, the normal projection of reported impulse is 0.044633 N s versus normal momentum change 0.014877 N s (approximately 3×). The full-vector residual is 0.02976 N s with no external force. Maximum point normal force is 10.13 N versus native 0.144 N. PhysX has 73/112 zero-impulse points versus native 11/112. In descending batches, worlds 0–2 show approximately 0.0142 N s residual (approximately 2× normal reporting), while other worlds are near 1.5e-7 N s. Repeated geometric points with different impulses occur in descending; Field's 112 points have no repeated exact geometric keys. Saved evidence lacks notification/header identifiers needed to isolate report aggregation versus solver behavior. Exact deduplication is insufficient, and dropping points or dividing totals by two/three is unjustified. Contact categories repeat, but force aggregation is not validated. No scaling correction, zero-force filtering or force equivalence is asserted.

## Guard opening defect — FAIL, preserved

All 288 supplied guard probes are reproduced. Of 86 rays with no visual-surface hit, one still hits collision geometry in both engines. The ray is index 198 at x=0.0275, y=-0.019 m, travelling upward from z=-0.06 m; source collider `collision_11_3135` is hit near z=0.01600423 m. The measured hit lies 59.77 µm from the closest supplied visual triangle. The source report gives 65.28 µm using its own calculation; its `passed=false` remains unchanged.

This is a convex-proxy intrusion into an open visual path, reproduced from the supplied native model and present after the PhysX cooking repair. No opening was filled deliberately or marked passed. No arbitrary box replacement or geometry decimation was used. Ray tests do not certify clearance for every finite-size object; targeted local mesh refinement and a bounded geometric-error/physical-clearance check remain necessary before acceptance.

## Performance and stability

Measured on the RTX 4090 using existing image `xtrainer-astra:isaac-2.1.0`, image ID `sha256:6deeed4a37c67b8a737c8de9e0ecc6c049dbf60ed93b83c24e9dbc2c83a1ebad`. No image or driver changes. PGS GPU pipeline, 1 ms, 50 warmup + 200 timed steps per case, common state readback, isolated from the legacy viewer. No Field replication in these microbenchmarks.

| Worlds / aircraft shapes | Free world-steps/s | Contact, reports off | Headers only | Full Python point records | Peak process RSS MiB | Device memory MiB |
|---|---:|---:|---:|---:|---:|---:|
| 1 / 11,874 | 789 | 552 | 615 | 170 | 3,031 | 4,834 |
| 8 / 94,992 | 2,685 | 2,494 | 2,240 | 211 | 3,310 | 5,138 |
| 32 / 379,968 | 2,936 | 2,788 | 2,760 | 207 | 4,194 | 6,242 |

Device memory includes an approximately 2,822 MiB unrelated/display baseline; compute-process snapshots report approximately 1,955 / 2,259 / 3,363 MiB (duplicate `nvidia-smi` PID rows must not be added). Geometry authoring/load takes about 21–23 s. All measured end states are finite. Sustained-contact final speed stays below 3e-6 m/s and final displacement is approximately 10 µm over the 250-step segment. Stored speed/displacement fields are final spatial maxima across worlds, not temporal maxima. These short samples have no confidence intervals; the one-world headers result illustrates timing noise.

At 32 worlds collision processing adds about 5% wall time over free simulation; full Python point collection adds approximately 13.5× over reports-off contact and consumes about 28.3/30.9 s inside the callback. Throughput saturates by eight worlds for full reports. Scaling was therefore stopped at 32. At 4,096 worlds this geometry would create 48,635,904 hull instances; practicality is **not demonstrated**, and the current detailed Python collector is unsuitable for large GPU PPO batches. A compiled/batched collider collector should be measured before considering a separately validated collision proxy. The GPU pipeline setting does not prove every convex contact runs on GPU; larger hulls can require CPU handling. [PhysX convex GPU limits](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/_api_build/classPxConvexMeshDesc.html).

Native one-world microbenchmarks: 5,553 free / 3,832 contact steps/s, approximately 1,898 MiB peak RSS, 13.9 s compile. This uses no Python per-point collection and is not an equal-work backend speed comparison.

## Run and evidence

From `/home/dsa/project/ceo_pj/RM27_Agile_Flight_RL_Project`:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p test_asm2_contacts.py -v
./run_asm2_collision.sh --mode candidate --suite fixtures --worlds 8 --layout 0 \
  --output .agents/runs/asm2-collision/my-fixtures.json
./run_asm2_collision.sh --mode candidate --suite field \
  --output .agents/runs/asm2-collision/my-field.json
./run_asm2_collision.sh --mode candidate --suite bench --worlds 8 --steps 200 \
  --output .agents/runs/asm2-collision/my-benchmark.json
PYTHONPATH=src .agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.native \
  --output .agents/runs/asm2-collision/my-native.json
PYTHONPATH=src .agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.native_resolution \
  --output .agents/runs/asm2-collision/my-native-resolution.json
```

Output paths must be unused; existing evidence is never overwritten. Run one Isaac launcher at a time. It has a 600 s timeout, 48 GiB RAM/16 CPU bounds and no network or learner. Host asset audit/export requires OpenUSD; native tests use the existing MuJoCo 3.12 environment. The candidate USD is portable inside this project through its source-relative reference.

Evidence under `.agents/runs/asm2-collision/`: `asset-audit.json`, `candidate-export.json`, source/cooked/normalized fixture comparisons, `candidate-layout{0,1}.json`, `native.json`, `native-resolution.json`, `isaac-field.json`, `guard-rays.json`, `bench-{1,8,32}.json`, and integrity snapshots. Ten host/native evidence-contract tests pass. Final integrity verification covers package contents, all historical checkpoint bytes, protected source/config files and upstream state.

**Remaining order:** resolve event impulse accounting; refine and recheck the specific guard proxy; establish sampling-robust ASM2 first-contact semantics and a new independently validated evaluator; then optimize collection/scaling. Flight/actuator adaptation and PPO remain outside this authorization. No hardware, commits or pushes.
