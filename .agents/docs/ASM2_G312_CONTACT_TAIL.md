# ASM2 G3.1.2 — bounded contact-tail validation

Completed 2026-11-13 Asia/Shanghai. **Two experimental cases, exactly 100 × 0.1 ms each (200 native steps total), no extra native replays.** Frozen detailed ASM2 plant, geometry, native scorer and actuator model unchanged. No PPO, new controller, remote operations, commits or pushes.

## Conclusion

**Forbidden body contact becomes mechanically loaded in both archived G2 row-51 tails.** The first positive body force occurs in the **second solver interval, [0.1, 0.2] ms**, relative to each restored first-contact pre-state. The stale first interval contains signed but zero-force body points; nominal has no signed body point in its first interval. At the second interval, body normal force is **3.793 / 4.005 N** (nominal/stale). The original nominal `QUALIFIED` and stale `FORBIDDEN_AIRFRAME_FIRST_TOUCH` verdicts remain latched. First-touch qualification alone therefore does not establish body-safe follow-through under these held targets.

Both tails load the front for a summed **6.4 ms**, never load the rear, and have **zero simultaneous front/rear loading**. This is intermittent front contact, not a 6.4 ms continuous bilateral event. Native signed contacts disappear for the final **1.9 / 2.0 ms**, with small outward COM velocity. Ten milliseconds does not establish sustained separation or recovery.

The research priority now moves from the resolved contact-order question to **actuator-feasible pre-contact terminal/contact-exit dynamics**. Continuous front-first order remains supported by G3.1.1; the new loads neither contradict that result nor revise the frozen scorer. They rule out interpreting the stale pair's initial unloaded body points as the whole mechanical event.

## Exact restoration and protocol

Inputs are the unmodified G3.1 `nominal-pilot-trace.npz`, `stale-pilot-trace.npz` and `pilot-results.json`, which reproduced the original G2 row-51 histories exactly. Restore the last saved 13-state vector, all four actual rotor thrust states and final four motor targets. These start at original **206.1 / 206.5 ms** and end at **216.1 / 216.5 ms**. Each 10 ms window includes its archived first-contact solver interval; it is not an additional 10 ms after that interval, nor aligned to a new continuous-force root.

The immediately preceding saved state has no native contacts. Its unconstrained acceleration is recomputed by `mj_forward` without integration and used for the solver warm start. Both restored first steps reproduce the archived post-state **bit for bit**, and all compared first signed manifold positions, distances, normals, forces and impulses agree exactly. This establishes the archived contact interval was restored correctly, including the 40 ms rotor filter memory. Targets remain fixed thereafter; there is no CTBR reallocation or feedback update.

The [manifest](../runs/asm2-g312/manifest.json) was frozen before execution. It pins source, tests, archives and all existing G3.1.1 evidence, and references the existing protected source/checkpoint manifests. Runtime model metadata matches the archived 11,874-hull aircraft and 1,822 static-geometry scene exactly. MuJoCo 3.12.0, Newton, implicitfast, 0.1 ms, 100 iterations and 1e−8 solver tolerance remain unchanged.

Native `ContactScorer.result` is initialized from the original archived verdict; unchanged `observe` returns it at every step. Diagnostic contacts are stored separately. There is no new qualification gate or retrospective relabelling.

## Matched mechanical results

“Normal” here sums positive pair-normal resolved solver forces/impulses, including friction in the separately recorded world resultant and angular impulse. It is not an armor sensor quantity. Group force/impulse includes all field contacts; every observed target happens to belong to the registered 25-hull set.

| Metric | Nominal | Stale |
| --- | ---: | ---: |
| Latched native verdict | QUALIFIED | FORBIDDEN_AIRFRAME_FIRST_TOUCH |
| Front loaded duration, summed (ms) | 6.4 | 6.4 |
| Body loaded duration, summed (ms) | 1.4 | 1.1 |
| Front / body normal impulse (mN·s) | 20.640 / 2.958 | 19.749 / 2.871 |
| Peak front / body normal force (N) | 11.186 / 4.117 | 9.311 / 4.110 |
| Rear / front–rear simultaneous loading (ms) | 0 / 0 | 0 / 0 |
| Total contact angular impulse norm (µN·m·s) | 212.944 | 211.986 |
| Body contact angular impulse norm (µN·m·s) | 124.290 | 120.329 |
| Body rate: initial → final (rad/s) | 0.490 → 1.529 | 0.496 → 1.511 |
| Alignment at 10 ms (deg) | 2.210 | 2.113 |
| Final outward COM normal speed (mm/s) | 9.399 | 8.482 |
| Net COM normal displacement (mm) | -0.273 | -0.271 |

![Matched group forces](../runs/asm2-g312/matched-forces.png)

The first loaded-body intervals correspond to original **206.2–206.3 / 206.6–206.7 ms**. They are discrete solver intervals, not continuous-time onset certificates. Native geometric body contact lasts a summed **7.6 ms** in each tail, much longer than its positive-force duration. Geometric intersection and mechanical loading therefore remain separate observations.

Body loading spans nominal **[0.1,1.3] and [4.5,4.7] ms**, stale **[0.1,1.2] ms**. The later nominal load occurs while signed body contact persists; it is not a proven recontact after geometric separation. Front loading spans nominal **[0,0.2], [0.3,0.8], [1.3,7.0] ms**, stale **[0,0.2], [0.3,0.8], [1.2,6.9] ms**. Empty rear histories mean no rear intersection or load was reported in this window, not global infeasibility of rear loading.

![Signed contacts, forces and post-contact motion](../runs/asm2-g312/contact-timelines.png)

## Centroid, angular impulse and separation

All force-producing body hulls are `collision_0_550`, `collision_0_551` and `collision_0_552`. Body normal impulse contributes **12.53% / 12.69%** of summed pair-normal impulse. Contact angular impulse includes each resolved force lever arm about the pre-state COM and any intrinsic contact torque. Group angular-impulse vectors sum; their norms do not. Total COM angular-momentum change is recorded separately, with rotor torque and discretization residual, rather than attributed entirely to contact.

The impulse-weighted, positive-normal-force point centroid in target `[slope, across, normal]` coordinates is **[26.919, 9.928, -0.019] / [26.903, 8.938, -0.019] mm**. Instantaneous centroid motion and angular impulse are shown below. This is a centroid of solver point loads, not measured physical active area or a unique equivalent point for a full frictional wrench. It supplies no causal test of centered/coaxial alignment.

![Contact mechanics and centroid](../runs/asm2-g312/contact-mechanics.png)

COM moves inward by about 0.27 mm over the window despite ending with outward velocity. Attitude changes by **0.719 / 0.715°**, and body rates rise to about **1.53 / 1.51 rad/s**. The final contact-free samples support short observed separation only. No continuous collision-order audit was repeated and behavior beyond 10 ms remains unknown.

## Verification and reproducibility limits

Three focused synthetic tests pass: unloaded forbidden intersections remain distinct from load; a known force couple plus intrinsic torque produces the expected COM angular impulse; positive-separation force remains visible separately. No additional physics trials are hidden in these tests.

[Verification](../runs/asm2-g312/verification.json) passes for both cases: exact archived first endpoint and manifold, unchanged latched result, exactly 100 intervals, finite full integration snapshots and constant targets. Maximum analytic motor-filter discrepancy is **3.89e−16 N**; force/generalized-constraint mapping error **1.78e−15 N**; torque mapping error **3.61e−16 N·m**; linear impulse–momentum residual **1.96e−18 N·s**. These are numerical accounting checks, not physical parameter calibration.

To honor exactly two continuations, **the full 10 ms tails were not independently rerun**. Reproducibility evidence consists of bitwise archived first-interval reproduction, frozen inputs/runtime, all 101 serialized full MuJoCo integration states per case, and per-step independent actuator/wrench/linear-momentum checks. No full-tail repeatability claim is made. The [final receipt](../runs/asm2-g312/final-verification.json) records protected input/evidence preservation, tests, report links and output hashes.

Raw point records, pre/post states, rotor thrust and targets are in `nominal-tail.json` / `stale-tail.json`. The NPZ files contain states, activations, held targets and `mjSTATE_INTEGRATION` vectors. [CSV timelines](../runs/asm2-g312/timelines.csv) contain one row per solver interval; [mechanics summaries](../runs/asm2-g312/mechanics-summary.json) and [summary](../runs/asm2-g312/summary.json) retain derived values. The [runner](../runs/asm2-g312/contact_tail.py) refuses repeated study execution or overwriting evidence; [report generation](../runs/asm2-g312/report.py) reads saved data only.

```bash
# From the project root. Already completed; do not rerun over these outputs.
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g312/contact_tail.py freeze
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g312/contact_tail.py run
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g312/contact_tail.py analyze
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g312/contact_tail.py verify
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-g312/report.py
```

The 5060's source-inspected static bilateral loading and seven dynamic windows remain separate [G3.1.1 cross-workspace evidence](ASM2_G311_CONTACT_ORDER.md#cross-workspace-evidence-5060-original-base-versus-4090-g2). Its 3,522-hull proxy, states and loaded-contact scorer differ. No remote audit or experiment was repeated and no remote result enters this two-case denominator. No physical force-area-time or official impact-score qualification is inferred. Actuator values remain inherited provisional assumptions.

## Exactly one recommended next experiment

**One preregistered pre-contact terminal-release feasibility pilot using the existing feedback controller and frozen detailed plant.** Start from the saved nominal **160 ms** pose/velocity/rotor/target snapshot, about 46 ms before its archived first touch. Select **one** actuator-feasible terminal/release reference using a bounded reachability calculation; freeze it before native execution. Compare its first-contact verdict and 10 ms signed-body/load tail against the already saved G2/G3.1.2 baseline; reuse that control evidence. Run only the single candidate, with no rollout search or new controller, and treat absence of forbidden follow-through as a diagnostic feasibility outcome separate from the frozen native first-touch verdict.

Why this priority: body loading appears by the second 0.1 ms interval, earlier than the original loop's next 5 ms update. The current 40 ms actuator filter has only a 22.1% step response after 10 ms; this favors examining the pre-contact state and rotor history before tuning a reaction after touch. It does **not** prove that all post-contact feedback is ineffective, that a candidate release is feasible, or that the provisional actuator model matches hardware. The 160 ms start gives roughly one modeled actuator time constant of lead rather than treating an instantaneous thrust change as available.

Expected cost: **1–2 engineer-days** to reuse existing feedback, constrain one terminal reference and instrument the same tail metrics; one bounded candidate simulation. Research value: test whether qualified contact can connect to a body-safe exit before adding continuous rotation or RL complexity. **Recommended only, NOT_RUN; requires separate authorization.** No candidate reference or controller has been implemented in this phase.
