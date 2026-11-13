# R4 — dynamics-validated braking-entry potential

Completed 2026-11-13 Asia/Shanghai. **Step 1 No-Go for the tested potential; PPO comparison NOT_RUN; no baseline promotion.** The archive supports an empirical Fixed A success frontier, but the tested full-state similarity potential does not establish useful directional guidance into it. Zero new native continuations and zero PPO transitions were used. Phase 4 remains the verified learned baseline; Fixed A remains the continuation reference.

The useful distinction is **recognizing a successful state versus guiding a dynamically feasible path toward it**. A diagnostic score without the conservative clearance mask discriminates saved Fixed A successes well (source-excluded AUC 0.984), yet the potential has long decreases along saved successful rotation/recentering motions. Its cheap clearance mask also rejects known clear states. Neither removing that mask nor accepting high classification accuracy would establish the requested dynamics-grounded learning signal.

## Scope and evidence

The completed [R3 report](ASM2_R3_SKILL_BRIDGING.md), [G3.1.2 report](ASM2_G312_CONTACT_TAIL.md), prior margin-potential and temporal-credit findings were reused. No exploration, reset-ratio, regional-weighting, temporal-credit, contact-tail, geometry-contract or remote 5060 campaign was repeated.

R3's endpoint evidence remains earlier contacts 0/36 for both arms, with B reducing envelope exits from 33/36 to 0/36 but producing timeouts/footprint misses. Both retained braking 35/36 and terminal 30/30. Those are **historical R3 results, not new R4 results**. G3.1.2's loaded forbidden-body follow-through remains a separate post-contact limitation. The frozen first-contact scorer stays the RL benchmark; no official impact qualification is inferred.

R4 matched complete initial states in the saved Phase R [manifest](../runs/asm2-frontier/phase-r/manifest.json) to frozen Fixed A outcomes in [results](../runs/asm2-frontier/phase-r/results.json), using `(case, frame, variant)` identities. Only `dev_center`, `dev_fast`, `dev_near_slow`, `stationary_0`, and `stationary_2` were used. Reserved selection sources and all held-out sources were excluded from fitting and this analysis.

| Archived Fixed A start phase | Native qualified | Failed |
| --- | ---: | ---: |
| Handover | 0 | 15 |
| Early rotation | 0 | 15 |
| Mid-rotation | 0 | 15 |
| Transition before braking | 6 | 9 |
| Transition after braking | 8 | 7 |
| Mid-braking | 14 | 1 |
| Late braking | 15 | 0 |
| Near contact | 15 | 0 |
| **Total** | **58** | **62** |

All 120 states contain position, quaternion, linear velocity, body rates, actual rotor thrust and held motor targets. Native restoration found no signed field contacts at these initial states. The **19 qualified nominal states** form the kernel support; the original ± variants contribute validation evidence without duplicating support nodes. Frames and variants within a source are correlated; no independent-sample significance claim is made.

These are successful **sampled states under Fixed A**, not certified reachable sets, a validated surrounding neighborhood, or proof that a failed Fixed A state is unreachable. No earlier autonomous success trajectory with complete motor histories was recovered from R3's three stochastic training successes; those remain outside this dataset.

## One candidate and prospective decision rule

[Configuration](../../configs/rm27_asm2_r4.json), [frozen manifest](../runs/asm2-terminal/r4/freeze.json), [candidate implementation](../../src/rm27_asm2/braking_potential.py), [archive diagnostic](../../src/rm27_asm2/r4_potential_audit.py).

For a state `s`, the candidate is

`Phi(s) = 0.5 * max_i [gamma^(T_i / 0.005) * exp(-d(s, s_i))]`,

with `gamma = 0.995`. `T_i` is the **actual archived Fixed A continuation time to qualified contact**, not the witness phase time. The unordered maximum imposes no trajectory index, reversal time, action, phase switch or tracking reference.

Distance is the root mean square of seven block distances: position, velocity, attitude chord, body rates, motor relaxation linear impulse divided by mass, motor relaxation angular impulse multiplied by inverse full inertia, and finite-body separating-plane clearance. Motor features use the frozen 40 ms lag, mass, rotor allocation geometry and inertia; they are local distance features, **not predictions of future feedback or a reachability model**. Block scales are RMS pairwise dispersion of qualified nominal states, recomputed on each source-excluded fit. Full-fit scales are 0.216 m position, 0.730 m/s velocity, 0.260 attitude chord, and 1.427 rad/s body rates. No bandwidth, weight or reward sweep was performed.

The candidate is bounded by 0.5, half the minimum qualified-contact event reward. It is zero outside the existing flight envelope, outside valid motor rails, or when its full-aircraft separating-plane bound cannot establish positive clearance. That bound includes every detailed collider but is **not exact finite-target distance**; its false exclusions are measured below.

Held targets are preserved in all empirical-state records. At the evaluated policy boundary the existing native rollout immediately replaces them through `set_ctbr` before integration. They therefore are not an extra distance block. Actual filtered motor thrust persists and is included. This does not resolve the previous actuator-observability question.

Before running the analysis, the following diagnostic criteria were frozen. They are research usefulness thresholds, not aircraft limits or official rules:

| Gate | Required | Observed | Result |
| --- | ---: | ---: | --- |
| Leave-one-training-source-out AUC | ≥0.80 | 0.726 | FAIL |
| Transition-only source-excluded AUC | ≥0.75 | 0.866 | PASS |
| Sources with positive median above failed median | ≥4/5 | 5/5 | PASS |
| Useful early direction on saved qualified motion | ≥4/5 | 2/5 | FAIL |
| Hidden-motor contribution to negative log score, p90 | ≤25% | 1.59% | PASS |
| Successful starts admitted by clearance bound | 58/58 | 43/58 | FAIL |

“Useful early direction” required net increase, at least 60% positive consecutive potential changes, and median absolute change ≥0.0001 per 5 ms. The check runs from saved handover to the earliest nominal Fixed A-qualified entry. Saved witness states are offline validation probes only; they are never an actor reference or kernel support merely because a witness visits them. Their action-limit caveat is material and reported below.

The conditional native confirmation cap was 20 new continuations, permitted only after these gates passed. The PPO budget was 98,304 transitions per arm, extendable to 196,608 only for B gains in **entry followed by native qualification** at two distinct earlier anchors, at least one robust 3/3 anchor, and braking/terminal retention. Both conditional stages remain **NOT_RUN**. No criteria were relaxed after seeing outcomes.

## Why classification did not establish useful guidance

The [mechanism analysis](../runs/asm2-terminal/r4/mechanism-analysis.json) separates the mask problem from the directional problem. Removing the mask **for attribution only** raises source-excluded AUC from 0.726 to **0.984**. The low aggregate AUC is therefore substantially a false-exclusion artifact, not evidence that complete state cannot discriminate Fixed A outcomes. Transition-only AUC already passes.

Every early trace has positive potential throughout, so the early decreases are independent of the clearance mask. Rotation and outward repositioning first move away from the successful braking-state cloud. The kernel penalizes that motion before rewarding the later return.

| Saved motion | Longest decreasing potential interval | Maximum drop | Steps with positive discounted shaping |
| --- | ---: | ---: | ---: |
| dev_center | 0.450 s | 0.151 | 39.0% |
| dev_fast | 0.730 s | 0.202 | 31.5% |
| dev_near_slow | 0.435 s | 0.137 | 36.1% |
| stationary_0 | 0.715 s | 0.168 | 31.4% |
| stationary_2 | 0.635 s | 0.042 | 37.4% |

For example, the center trace moves along the target slope from −0.023 m to −0.270 m while alignment error rises from 12.1° to 52.9° during the potential drop, then eventually reaches a Fixed A-qualified entry. The candidate has no dynamics-to-entry cost that recognizes why such intermediate motion can be useful.

**Provenance limit:** four original Phase 3 witness traces exceed the current actor's ±2 rad/s command limit (peaks 2.17–3.29 rad/s). They demonstrate possible motion of the pinned plant, not an actor-compatible path certificate. `stationary_2` stays within the rate and collective interface on the analyzed prefix (peak 1.985 rad/s; 0.898–1.207 hover collective), and still has a 0.635 s decrease. Existing R2 capped-feasibility outcomes are retained, but this task did not rerun them or pretend their success labels supply missing full trajectories. These checks do not prove every feasible route must cross a potential valley or that PPO with this potential must fail; they fail to supply the predeclared evidence for **useful** directional guidance.

![Candidate scores and saved-motion directional check](../runs/asm2-terminal/r4/potential-validation.png)

The hidden-actuator statistic is specific to this metric: omitting its two motor blocks changes negative log score by only 1.59% at p90 across evaluated samples. It does not establish actor observability, physical parameter accuracy or motor-memory irrelevance. Earlier observability experiments were not repeated.

## Finite-body clearance and terminal boundaries

All 15 false exclusions are the three near-contact variants from each training source. The conservative plane bound is −0.208 to −0.043 mm for nominal states. Its target support plane is at +2.220 mm along the target normal and includes protruding parts of finite registered geometry; extending this support plane everywhere excludes some poses that are clear of the actual target.

Targeted static queries cover all **11,874 aircraft hulls × 25 registered target hulls** per excluded state, using containing-sphere lower bounds to prune native convex-distance queries. The numerical finite-geometry minimum is **1.992–2.156 mm**, and all 15 restored states have zero signed field contacts. This confirms a conservative approximation failure, not a collision defect, manufacturing tolerance result, need to change geometry, or scorer error. These are static distance queries with zero physics steps, not new rollouts or a repeated continuous-contact audit. A separate analytic box fixture checks the pruning result against both known distance and exhaustive pair queries.

![Directional shaping and finite-distance attribution](../runs/asm2-terminal/r4/failure-mechanisms.png)

The shaping helper implements `gamma * Phi(next) - Phi(current)`, with **zero successor potential for both termination and this task's episodic timeout**. An ordinary collector chunk boundary retains the successor potential. Tests check discounted telescoping, isolation from auto-reset potential, continuing-boundary bootstrap semantics, negative shaping for stationary dwelling, state/motor sensitivity, quaternion-sign invariance and conservative exclusion.

Thus completed discounted shaping return is `-Phi(initial)` and cannot add an episode-return bonus for dwelling, leave/return behavior or timing out. This algebra preserves the original objective under correct integration; it does **not** guarantee improved finite-budget PPO learning. The helper was not installed into the learner because Step 1 failed. No claim of end-to-end Isaac/PPO integration is made.

## Verification, preservation and reproduction

**CONFIRMED:** three focused unit tests and an independent analytic distance check pass. Saved values reproduce exactly: all 120 full-fit scores, 120 source-excluded scores, and 1,481 saved-motion potential samples. Frozen source/checkpoint pins and model metadata match. No fresh held-out outcomes were opened, no checkpoint was selected, and no R4 learning curve or capability gain exists.

The initial geometry-extraction invocation attempted an unnecessary contact solve at the target origin and failed before producing results or advancing physics. It was corrected to kinematics-only frame extraction; the original [freeze](../runs/asm2-terminal/r4/freeze.json) and additive [correction record](../runs/asm2-terminal/r4/freeze-correction.json) remain. Formula, criteria, plant and data were unchanged. Failed-candidate code is isolated and retained solely to reproduce this negative result; it is not a baseline feature.

Fixed A checkpoint SHA-256 remains `eba759e7eeea93cd9fe06e0f8653d8f12bafc6cfd05a64bcbc11a7a78086fa3f`; Phase 4 remains `85831eb55973c575b09283e276c17e7004ed059f7b64c9e25f98814ff8307fbd`. Frozen MuJoCo 3.12.0, 0.1 ms physics, 5 ms CTBR, 45 actor inputs, collision assets and native scorer are unchanged. No controller, recovery training, hardware, remote modification, commit or push occurred.

- [Empirical complete-state support and clearance](../runs/asm2-terminal/r4/empirical-support.json).
- [Candidate results and failed gates](../runs/asm2-terminal/r4/potential-audit.json).
- [Mechanism implementation](../../src/rm27_asm2/r4_potential_report.py) and [verification receipt](../runs/asm2-terminal/r4/verification.json).
- [Tests](../../tests/test_asm2_braking_potential.py). PNG/PDF figures are stored beside the results.

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
# Recompute saved numerical results and tests without advancing native physics:
/tmp/rm27-g311-env/bin/python .agents/runs/asm2-terminal/r4/verify.py
# Recreate figures from saved evidence:
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r4_potential_audit plots
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r4_potential_report figures
```

The complete archive/geometry calculations can also be reproduced with `r4_potential_audit audit` and `r4_potential_report mechanisms`; they reuse the freeze and reject changed existing result files. Routine verification above avoids rebuilding the detailed native model. The matched PPO experiment, midpoint/extension assessment, fresh earlier-state suites, retention evaluations and reached-handover evaluation are **NOT_RUN because Step 1 failed**, not reported as zero-success policies.

## Exactly one next research action — not launched

**One bounded native-only backward expansion of the Fixed A entry frontier, capped at 20 previously unevaluated starts across the five training sources.** Select control-boundary frames just before each earliest robust qualified entry, restore complete motor state, and let frozen Fixed A act autonomously. Record full state/motor histories, exact finite-geometry clearance and final native qualification. Do not replay witness controls or fit another reward during that experiment.

This addresses the concrete missing evidence: the present archive labels isolated entry states, while the successful **autonomous transition into those states** is not represented densely enough to validate a dynamics-to-entry signal. It can reveal whether a backward extension exists under the current actor interface, or whether the frontier is locally sharp. It does not promise to reach the original early anchors. Implementation cost is a small extension of the existing native recorder and a bounded CPU run, with no PPO or architecture build. It has greater immediate value than training this rejected similarity signal or retuning its scales. **Recommendation only; no additional cases are authorized or launched by this report.**
