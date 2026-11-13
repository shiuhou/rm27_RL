# ASM2 G3.1.1 — continuous contact order and competition mapping

Completed 2026-11-12 America/New_York. Existing evidence/manifests retain their original 2026-11-13 Asia/Shanghai timestamps. Scope follows the user's narrowed completion request. No new rollouts, controller, training, static alignment or angular-impulse experiment was run during completion. Frozen geometry, contact contract, scorer and physics remain unchanged.

Cross-workspace integration completed 2026-11-13 Asia/Shanghai: actual 5060 source/artifacts inspected over read-only SSH; no remote execution of research code and no additional native rollouts. The original 27-case dataset and completion receipts remain immutable.

## Decision and scope

**SUPPORTED, conditional on the pinned saved-motion reconstruction:** designated `contact_front_006` intersects before any forbidden body intersection in both G2 row-51 histories and all 25 available label-flipping suffixes. No continuous body-first counterexample was found. In seven cases that the native scorer rejects for body contact, the independent signed roots place front contact **64.485–80.983 µs earlier** than body contact. The remaining 20 cases have no signed body intersection before the saved trace ends.

This supports continuous geometric ordering; it does not turn a native rejection into qualification. The frozen scorer rejects any forbidden body point in the sampled manifold, including unloaded points. Its behavior remains correct under the established engineering contract. A real continuous physical trajectory, hardware tolerances and official impact detection remain unverified.

The original 27-case results, two FCL pilot results and every original manifest were already complete at interruption. Completion reused them, added missing signed-root/uncertainty checks, finite-prefix exclusion, tests and figures. It did not restart the completed audit. [Machine-readable completion](../runs/asm2-g311/contact-order-uncertainty.json) · [All 27 cases](../runs/asm2-g311/contact-order.csv) · [Preservation manifest](../runs/asm2-g311/completion-manifest.json).

## Three representations

| Representation | What it represents | What the available evidence establishes |
| --- | --- | --- |
| Official RoboMaster V1.0.0 §5.1.1, Table 5-1 | Armor impact-event detector: any position above **α N** starts an event; all positions below α end it; active regions may be disconnected; duration and surface cumulative quantity **∬ F dt dS** enter detection | **CONFIRMED** from the retrieved official English manual, printed pp. 65–66, and Chinese p. 64. Numerical α and drone duration/impulse ranges are unpublished/TBD. Sensor spatial discretization and the dimensional interpretation/calibration of local F are unspecified. |
| Pinned ASM2 aircraft CAD/collision package | 11,874 convex collision meshes: 14 `contact_front_*`, one `contact_rear_*`, 11,859 `body` | **CONFIRMED** mapping below. Existing designated front/rear semantics remain legal; other-body contact remains forbidden. |
| Frozen MuJoCo engineering scorer | First signed sampled manifold, designated membership, registered finite target, central **90 × 100 mm** engineering region, alignment, point velocity, body rate, penetration and compressive-load gates; forbidden co-contact rejects | **CONFIRMED** from `contact_task.py`, the pinned contract and saved native records. This is an engineering acceptance definition. It implements no distributed force-area-time detector. |

The original base region is verified in both workspaces as target half-sizes **45 × 50 mm**. Read-only inspection now verifies that the separate R2 fixture is **72 × 72 cm**, inclined 18°, with no original-base field geometry. It was not substituted for the base or merged into this analysis.

Official sources already retrieved, with no new web requests during completion: [official document listing](https://bbs.robomaster.com/wiki/20204847/1969149), [local English V1.0.0 PDF](../runs/asm2-g311/references/RM2027-V1.0.0-en.pdf), [Chinese PDF](../runs/asm2-g311/references/RM2027-V1.0.0-zh.pdf), [API document links](../runs/asm2-g311/references/rules-api.json), [download hash](../runs/asm2-g311/references/official-source.json). The English MD5 matches the listing's `8e70df73c034b0157f2568c01a63af63`.

## Actual CAD surface mapping

**CONFIRMED from already saved geometry measurements and the source generator** `assets/asm2/source/tools/contact_geometry.py`:

- The generator selects all main-body (`0_rk3576_body.obj`) triangles whose three vertices lie within 0.2 µm of its minimum body-frame Z, **−19.86282 mm**. It partitions them by front/rear X, merges only convex adjacent footprints and extrudes them upward by **50 µm**. This is source construction metadata, not a measured manufacturing tolerance.
- The 15 projected patches have total union area **228.094 mm²** and exactly match the selected **150 lowest coplanar CAD triangles**: measured symmetric difference is zero. They retain holes and bends in that selected face set.
- The main-body mesh has 206,219 triangles. Its projected silhouette and union of down-facing projections are each **5,952.208 mm²**; the patch union is **3.832%** of that projection. The denominator includes surfaces at different heights. It is neither a simultaneous contact area nor a declaration of legal surface membership.
- Thus the strips are an exact representation of the selected lowest CAD face set. They are not geometrically identical to every down-facing surface or the entire silhouette. **The 3.83% figure does not establish that the legal underside is incorrectly represented.** No additional CAD face has been reclassified as legal.

[Saved geometry values](../runs/asm2-g311/geometry.json) · [Existing CAD projection figure](../runs/asm2-g311/underside.png). Source surface mapping is confirmed; material compliance, manufacturing dimensions, and whether higher faces participate in physical distributed contact remain unknown.

## Independent continuous method

The completed main audit checked all 11,874 aircraft source hulls against all 25 finite registered target hulls, with conservative broadphase pruning, then independent convex separating-axis (SAT) queries. Narrowphase uses face normals and edge cross products, rather than the unchanged MuJoCo contact algorithm. Pose reconstruction is linear COM translation and shortest-arc quaternion SLERP between saved **0.1 ms pre-integration states**, beginning at 200 ms. The source hulls, finite target transforms, native model and control/checkpoint inputs are pinned.

The interval procedure excludes an interval only when a positive separation projection exceeds its maximum rigid-motion displacement bound. Remaining intervals subdivide to **48.828 ns**. Those original proximity brackets indicate possible contact within a numerical band; they were **not** exact signed-intersection certificates. Completion found actual signed SAT zero crossings, retained positive/negative samples on either side, and calculated symmetric **±0.1 µm separation-band** intervals. A separate **±1 µm stress band** measures sensitivity; it is not a physical tolerance or an official threshold.

The computation is double precision and has analytic motion bounds, but does not use outward-rounded interval arithmetic. Numerical bands are diagnostic allowances, not a formally certified bound on all floating-point errors or arbitrary geometric perturbations. Any unresolved/censored band is retained explicitly.

**Earlier G2 prefix:** the saved original histories have 5 ms states before 200 ms. A conservative infinite-plane exclusion failed, even after analytic rotation support minimization. That failure did not prove collision: the finite registered target has thickness and spatial extent. A subsequent missing finite-hull check evaluated the same 11,874 aircraft hulls against the actual 25 finite targets, using 164 nominal / 158 stale broadphase candidate pairs. It excluded all prefix intersections under the saved 5 ms linear/SLERP reconstruction with a 1 µm diagnostic band. [Finite-prefix result](../runs/asm2-g311/early-finite-results.json) · [Retained failed plane bound](../runs/asm2-g311/early-sweep-exclusion.json). Suffixes are conditional perturbation experiments beginning at 200 ms; they are not concatenated through their reset jumps into hypothetical full flight trajectories.

**FCL and Tight-Inclusion:** two already completed FCL convex-distance pilots provide an independent query cross-check. Earliest front/body proximity intervals match SAT for the pilots; secondary target-tile proximity brackets differ by at most **0.293 µs**, without changing earliest identity/order. This used FCL distance queries inside the audit's interval search, not a claim that FCL's default continuous solver is a certified rotational CCD implementation. The retrieved FCL 0.7.0 header exposes translation, linear, screw and spline motion choices; the Python distance wrapper exposes no distance-tolerance setting. Tight-Inclusion's retrieved README describes conservative vertex-face/edge-edge queries, numerical filters and minimum separation. Tight-Inclusion was **NOT_RUN**; its full implementation/version was not pinned here. No Tight-Inclusion certification is claimed. SLERP preserves rigid rotation explicitly; replacing it with straight vertex chords would require accounting for chord error.

## Continuous results versus native results

Times below are milliseconds from the original G2 reset. Signed roots are numerical reconstructions, not measurements of physical time at nanosecond accuracy.

| Case | First designated root (ms) | First body root (ms) | Front lead (µs) | First native sample (ms) | Native verdict | Loaded body points at sample |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| nominal-pilot | 206.029033 | Not observed | — | 206.100 | Qualified | 0 |
| stale-pilot | 206.405229 | 206.486212 | 80.983 | 206.500 | Forbidden co-contact | 0 |
| nominal-005 | 205.902512 | 205.975195 | 72.683 | 206.000 | Forbidden co-contact | 2 |
| nominal-017 | 205.803092 | 205.876497 | 73.405 | 205.900 | Forbidden co-contact | 1 |
| nominal-026 | 205.729584 | 205.794069 | 64.485 | 205.800 | Forbidden co-contact | 0 |
| nominal-027 | 206.026950 | 206.099426 | 72.476 | 206.100 | Forbidden co-contact | 0 |
| nominal-029 | 206.018619 | 206.091144 | 72.526 | 206.100 | Forbidden co-contact | 1 |
| nominal-031 | 206.008211 | 206.080798 | 72.587 | 206.100 | Forbidden co-contact | 1 |

The 19 stale label flips qualify under the unchanged native scorer, have designated-first roots, and have no signed body intersection within their saved windows. These are correlated cases selected because their labels changed; no population success rate or generalization estimate follows.

For the G2 pair, continuous front roots differ by **376.196 µs**, compared with the native samples' **400 µs** difference. Stale's front interval under the 0.1 µm separation band is **206.404541–206.405865 ms** and its body interval **206.485547–206.486846 ms**: the ordering intervals do not overlap. No simultaneous first intersection is resolved for this pair. The subsequent native co-contact is fully compatible with distinct continuous contacts occurring within one 100 µs sample interval.

![Continuous projection gaps](../runs/asm2-g311/continuous-contact-gaps.png)

![Seven native co-contact cases](../runs/asm2-g311/contact-order.png)

[Timing for all 27 cases](../runs/asm2-g311/all-case-timing.png). Censored body intervals are drawn with an arrow; their endpoints are not invented.

## Geometric uncertainty and limits

- **Numerical representation — CONFIRMED:** source/compiled vertex Hausdorff discrepancies for the only aircraft candidate meshes are **2.002 nm** (`contact_front_006`) and **2.050 nm** (`collision_0_550`). Previously verified maximum target recompilation discrepancy is **9.934 nm**. Replacing aircraft source vertices by compiled vertices moves signed roots by at most **3.501 ns** across the tested events.
- **Rotation and interpolation — CONFIRMED sensitivity:** SLERP vertex-to-straight-chord deviation is at most **0.0168 nm** over the near-contact sample intervals. A separate endpoint-velocity cubic COM/SO(3) Bézier reconstruction changes roots by at most **16.308 ns**; sampled pose-induced vertex discrepancy is at most **6.714 nm**. At the 66 observed pair crossings, these reconstructions agree on order. This is a consistency check on saved states, not a bound on unknown real-world motion or proof that every possible interpolation agrees.
- **0.1 µm band:** all 27 cases preserve designated-first ordering or designated-only contact before trace end. For `nominal-027` (−0.01° body yaw), nominal geometry has a body root, but the final body projection overlap is only **0.0909 µm**. The opposite-side band crossing is not observed before termination. Body-intersection existence is therefore unresolved under that band even though designated-before-any-possible-body ordering remains supported.
- **1 µm stress band:** ordering remains designated-first. The unmodified nominal pilot and `stale-071` admit a possible body event within the expanded band even though their actual signed body roots are absent. Body occurrence is unresolved under that stress band. Do not report these two cases as observed body collisions. `nominal-026` also lacks a −1 µm body-overlap witness before termination.
- **Physical uncertainty — UNKNOWN:** no measured manufacturing tolerance, shell compliance, armor deformation, COM error or pose-estimation covariance is available for these traces. Neither diagnostic band estimates a real physical distribution. Earlier 5 ms prefix exclusion and all 0.1 ms roots remain conditional on the stated reconstructed motions.

The previously reported approximately 12–13 µm infinite-plane support difference remains G3.1 background. Finite target hulls and interpolation are what establish the current event order; a support difference alone does not establish either scorer qualification or impulse transfer.

## Intersection, load and official impact qualification

**Continuous intersection** is a signed hull crossing under the stated reconstruction. **Physically loaded contact in this simulation** is a nonzero resolved native wrench; it is observed over a discrete solver interval. **Native qualification** is the frozen engineering scorer's decision. **Official impact qualification** requires the unpublished distributed detector conditions and is **UNKNOWN**.

At first native sample, nominal/stale total compressive normal impulse is **0.430920 / 0.428349 mN·s** over their recorded 0.1 ms solver intervals. Stale retains four signed body points with exactly zero force/impulse. Four nominal label-flipping cases (`005`, `017`, `029`, `031`) do have nonzero body force at their first sampled manifolds; the other two and stale do not. Thus geometric intersection cannot universally be treated as loaded contact, and body co-contact cannot universally be dismissed as unloaded numerical reporting. Rejected cases can carry nonzero designated and/or body forces even when `first_loaded` is unset because the scorer rejected before reaching its load-acceptance branch.

The existing `point_region_alpha_proxy` arrays and `alpha-proxy.png` are retained as historical **point-threshold diagnostics only**. They use `sum(F_normal,i * dt)` over point records above a symbolic α. A changing number of solver points is not a set of calibrated armor detection regions. Their thresholds do not reproduce official α, and their force sum is not the official surface integral. Point forces are already integrated forces; multiplying them by an invented physical area would not establish the detector's quantity. Actual active area, force density, complete impact duration and surface cumulative detector quantity are unavailable. The saved records stop at native decision; even the whole event duration is censored. No exact competition hit is inferred.

## Cross-workspace evidence: 5060 original base versus 4090 G2

**CONFIRMED inspection; historical simulation results REPORTED, not rerun.** SSH `ceo5060` exposed `/home/shiuhou/Projects/rm27_drones/components/rm_uav_mujoco/`. Ancestor/component instructions, engineering, SPEC/DESIGN/ROADMAP and relevant plan handoffs were read. Component HEAD was `f414b4c7517db7eae79ffee112d60bc45f2ef302`, branch `feat/moving-base-target`, with **96 dirty/untracked status entries**; HEAD alone does not identify the inspected implementation. A bounded [evidence extraction](../runs/asm2-g311/cross-workspace-evidence.json) retains absolute/resolved source paths, whole-file SHA-256 values for 68 selected files, source excerpts, artifact fields and start/end Git snapshots. The snapshots agree. No repository, scene binary or trajectory was copied; only selected metadata, excerpts and small target vertex arrays were transferred. [Collector](../runs/asm2-g311/cross_workspace_evidence.py) uses standard-library reads/hash/parsing remotely, with bytecode disabled and Git optional locks disabled.

| Item | Compatibility established | Limit on reuse |
| --- | --- | --- |
| Legal aircraft contact surface | All **15 patch OBJ hashes match byte for byte** between remote model, local source CAD collision meshes and local proxy export. Aircraft FLU frame, 142 g mass, COM-centered inertia and provisional actuator scalars agree. Proxy XML SHA-256 is identical: `2356bc3f…05ae4`. | 5060 experiments use **3,522** aircraft hulls (3,507 forbidden); local G2/CCD uses **11,874** (11,859 forbidden). The shared proxy is a local export, not the model used for the current G2 roots. Body clearances, wrenches and outcomes cannot be pooled. |
| Original-base registration | Same center `[11.4561,0,1.079] m`, normal `[-0.4617148005,0,0.8870284342]`, central **90 × 100 mm** engineering region. All **25 finite target hulls** present in the remote cropped scene match local saved compiled world vertices to **5.604 nm** maximum vertex Hausdorff discrepancy. | Remote slope axis has the opposite sign to the local slope axis; across/normal agree. Local native composition translates the base center to zero. Frame conversion is explicit in the comparison. |
| Target membership and scene identity | Remote crop retains `35363…35379` plus backing `36286…36293`, the same 25 local targets. | Remote mapping also lists `39516…39531`, absent from this crop and discarded by its scorer. Whole scenes/field sources differ: remote compiled scene `b3793d6f…4ff8f`, remote field model `6d8608c2…74896`; local field extraction model `8023b223…156b9a`. Matching selected target hulls does not establish identical surrounding geometry or dynamics. |
| Native time and contact material | MuJoCo 3.12.0, 0.1 ms contact step, implicitfast, 5 ms control; original-base target solref `.004 1` and friction `.8 .01 .001` agree with saved local target metadata. | Remote XML/defaults and local frozen material setup are retained separately; no cross-model force equivalence is assumed. Remote event timestamps are solver-interval **ends**; local first-touch sample timestamps identify **pre-states**. |
| Scorer semantics | Both identify the 15 legal patches and registered engineering region. | Remote `ContactScorer.active` skips unresolved constraints and normal force **≤1e−8 N**, has no explicit signed-distance filter, and uses a full SO(3) attitude gate. Local scorer uses signed distance ≤0 and rejects forbidden points even with zero load. Remote/local attitude gates are **18°/8°**; normal speed **0–0.40/0.02–0.35 m/s**; plane error **5/0.6 mm**; penetration **5/0.25 mm**. Local rate/impulse gates also differ. Remote qualified touch cannot establish local qualification. |
| R2 fixture and state continuity | Source verifies one neutral box **0.72 × 0.72 × 0.04 m**, 18° incline, friction `.8 .005 .0001`, no original-base/neighbor geometry. | R2 mechanics and estimator timing are separate evidence, not original-base outcomes. R2.2/R2.2-B provide timestamp/impact-epoch lessons; they do not validate these G2 contacts or a new recovery controller. |

**Static original-base evidence.** Remote `artifacts/asm2_contact_recovery/surface_alignment_native_geometry_01/summary.json` (SHA-256 `ffa6ab29…f7b49`) and its matching source `native_geometry.py` (`24c447a9…2a9`) confirm an `mj_forward` calculation at zero velocity/motors, not a flown equilibrium. At its centered pose, front/rear normal loads are **0.541653 / 0.858380 N** and their projected areas **111.748 / 116.346 mm²**, fully within the engineering rectangle. The nearest forbidden proxy hull is `collision_proxy_00_170`, **+29.096 µm** from target `35363`. The scanned clean dual-loading offsets are **−29…+18 µm**, in 1 µm increments. Source lines 147–149 require positive load in both groups, **no forbidden candidate manifold**, and no scorer rejection. This is stronger than merely filtering unloaded forbidden forces, but remains specific to the proxy geometry, static pose and scan; it is not a manufacturing tolerance or sustained dynamic demonstration.

**Dynamic evidence already available; no repetition needed.** Seven inspected metrics files link to trajectories whose actual SHA-256 hashes match the recorded hashes and to the same remote scene. Their analysis source hash also matches. Each window starts one native interval before the recorded first-contact endpoint and ends 20 ms after it (201 × 0.1 ms). All seven have **zero simultaneous front/rear loaded duration**. Representative results:

| Remote archived case | Positive-force duration in window (ms) | Contact angular impulse norm (µN·m·s) | Total COM angular momentum change norm (µN·m·s) | Archived recovery |
| --- | ---: | ---: | ---: | --- |
| A, baseline | 9.6 | 107.92 | 101.64 | Joint success under remote protocol |
| B, centered terminal | **8.3** | 86.33 | 107.83 | Joint success under remote protocol |
| C, full-surface lead | 12.1 | 91.45 | 95.48 | Forbidden contact; recovery deadline |

Source/provenance discrepancy: remote `plan_impact_recovery_01.md` §0 reports B as **10.0 ms**; the hashed `surface_alignment_b_metrics_01/summary.json` records **8.3 ms**, solely rear loading in its matched window. This report uses the artifact value and does not silently reconcile different summaries. Both the numerical near-plane area (±50 µm band) and infinite-plane support clearance are geometric diagnostics, not loaded physical area or finite-hull collision certificates.

**Robustness and angular impulse.** The inspected R1 screening diagnostics contain 48 matched A/B pairs, with only **19** eligible for the reported joint-qualified/complete-window contact metrics. In that selected subset, median B−A is **−60.399 µN·m·s** contact angular impulse norm, **−17.911 µN·m·s** total COM angular momentum change norm and **+14.262 µm** sampled minimum forbidden clearance. These are conditional descriptive statistics. Inputs identify different planner/driver baselines; the B config enables `surface_alignment`. This is not an isolated causal intervention on center alignment and does not establish a globally superior strategy. The source sums contact forces and their moments about COM separately from total angular momentum; neither quantity supplies calibrated distributed pressure/active area.

The existing `r1a_physics_01` archive covers **12 saved seeds × 13 branches**. In its native branch, 11/12 first contacts qualify and 6/12 later have forbidden contact; 0.05 ms steps retain labels, while 0.2 ms changes one. Half/double solref scale yields 2/12 versus 9/12 later forbidden outcomes; friction half/1.5× yields 4/12 versus 8/12. This already demonstrates solver/material sensitivity in that model. Modified-physics and patch-offset branches remain historical sensitivity evidence, never additions to the frozen 4090 dataset or measured material/tolerance estimates.

**Reusable continuity lesson, with limits.** R2's saved fixed-material-point diagnosis follows `collision_proxy_00_407`, vertex 30: at absolute **123.6 ms** (108.5 ms after first contact), COM normal velocity is **+0.043077 m/s**, rotation contributes **−0.043193 m/s**, and the point moves inward. COM separation alone therefore need not guarantee body separation. Its plane-support calculation is distinct from finite geometry. R2.2 explicitly rejects treating delayed pre-impact estimates as current post-impact state; R2.2-B's handoff still had final validation outstanding at inspection. Neither is new G2 evidence, and loaded contact without a contact-free interval must not be relabelled recontact.

Coaxial, centered, opposing-normal contact remains a preferred nominal hypothesis. Verified static overlap and bilateral load do not establish sustained bilateral dynamics or global optimality. The existing local geometry/force/centroid/impulse calculations remain unchanged, and no legal surface has been reclassified. The new remote evidence neither contradicts nor extends the 27 local continuous-order cases; it prevents redundant alignment and proxy force-tail experiments.

## Verification, reproducibility and source gaps

The immutable original run used **zero new rollouts**; completion also used **zero**. Ten focused tests pass, covering thin convex solids, translation contact between clear endpoints, SLERP, signed roots, censored bands, overlapping intervals remaining unresolved, constant-motion cubic reconstruction and analytic rotational support with an interior minimum. Production scorer/physics tests were not rerun because their code and inputs are unchanged.

Original `verification.json` confirms the 27-case run completed in 131.035 s with unchanged pins. Missing uncertainty calculations completed in 127.647 s; finite-prefix check took 12.062 s. The completion's first figure attempt encountered a `None` endpoint for the intentionally censored `nominal-027` band after numerical results were written. The final figure was rendered from saved JSON with a censoring arrow; no calculations/rollouts were restarted. Both failed infinite-plane bounds remain preserved as unsuccessful exclusion evidence.

Scripts and results live in [asm2-g311](../runs/asm2-g311/). Existing `audit.py`, manifests, results and figures retain their hashes. `finish_contact_order.py` exclusively creates additional evidence and should not be rerun over completed output. The final report/figure command reads saved results; the finite-prefix script is already complete.

```bash
# From the project root; isolated diagnostic environment, no production dependency changes.
export PYTHONPATH=src:. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/tmp/rm27-g311-env/bin/python -m unittest discover -s tests -p test_asm2_g311_geometry.py -v
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g311/report.py
```

[Dependencies](../runs/asm2-g311/requirements-offline.txt) · [Final hash/test receipt](../runs/asm2-g311/completion-verification.json). Phase 4 `checkpoint-186368.pt`, Fixed A and experimental checkpoints remain protected. Goal Hybrid archives were read only. No commits/pushes, hardware work or new web requests occurred during completion.

The [integration verification](../runs/asm2-g311/cross-workspace-verification.json) checks preserved numerical results, source/checkpoint pins and remote-to-local compatibility. Historical receipts retain their original document hashes; only this report, the active plan and roadmap are intentionally updated. Existing tests were not repeated for this documentation/provenance integration.

Remaining gaps are measured tolerances/compliance, sensor-area/force calibration and unpublished official detector thresholds, true motion between saved samples, and exact loaded tails for the detailed-model G2 pair. Remote archives are inspected, not independently rerun; the large compiled binary was identified by saved provenance rather than transferred or rehashed. Target vertex matching establishes selected source-hull compatibility, not whole-scene or force equivalence. These limits are not filled with invented values.

## Exactly one recommended next experiment

**A bounded matched contact-tail capture for the G2 nominal/stale pair, using the same frozen plant and contact contract.** Question: after the first signed sampled manifold, does forbidden body contact become loaded, and how do front/rear loads evolve?

Restore each archived final pre-contact state with its saved motor state and actuator targets; retain those final targets for a strictly bounded **10 ms** observation suffix at the unchanged 0.1 ms physics step. Record every signed manifold, per-pair wrench and impulse, body-load onset, and post-contact motion. Keep the scorer latched to its original verdict and save the diagnostic tail separately. This is two conditional continuations with held final targets, not evidence about a newly designed feedback policy. Freeze the two-case manifest before execution; do not tune geometry, offsets or gates.

Estimated implementation cost: **half an engineer-day**, reusing the existing native recorder; simulation cost is two 100-step suffixes. Research value: distinguish the stale pair's unloaded forbidden touch from subsequent mechanical loading, and assess the missing event tail before deriving a new planner terminal set. It will still not measure physical active area or establish official qualification. **Recommended only; NOT_RUN and not authorized automatically by this report.**

This remains justified after inspecting the 5060's existing 20 ms force windows: those use different proxy body hulls, approach states, actuator histories and scorer semantics. They cannot answer when the **archived G2 detailed-model** body touch becomes loaded. Reuse their wrench/time conventions; do not repeat their alignment experiments or broaden this proposed capture into planner development.
