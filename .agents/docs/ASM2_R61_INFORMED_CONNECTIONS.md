# ASM2 R6.1 — Dynamics-constrained informed-reset connection feasibility

**Status: COMPLETE — decision C; no matched connection found in the bounded study; zero PPO.** The roadmap places this study immediately after R6 and reserves no new integer phase. R6.1 follows the existing decimal convention for a bounded follow-up. All prior R5/R5.1/R6 evidence, Fixed A, Phase 4, physics and native scoring remain protected.

## Question and experimental unit

Can bounded physical CTBR prefixes connect outward-moving earlier rotation states to complete states on empirically successful Fixed A trajectories, after which Fixed A autonomously qualifies contact? A planned prefix is feasibility evidence, never an autonomous RL success. No prefix actions are teacher labels, policy references or future actor inputs.

Five development families are used, with no held-out sources. Initial states are the existing nominal Phase R `rotation` samples. Their target-slope velocities are outward, approximately −0.715 to −1.113 m/s. Four prescribed starts per family use the existing ±0.5 mm position and ±5 mm/s velocity offsets, separately along registered target slope and across tangents. All other state and actuator components remain identical. Twenty cases are correlated sensitivity samples from **five source families**, not twenty independent task distributions or a certified region.

Goals come from the complete native R5 nominal trajectories beginning at center:725, fast:816, near_slow:702, stationary_0:330 and stationary_2:346. Each original trajectory ends in frozen-native qualified contact. Select the earliest 5 ms boundary at least 5 ms into the autonomous trajectory with each body-rate component within ±1.8 rad/s and at least 40 ms remaining before contact. This rate margin is a proposal heuristic; actual rates and commanded rates are distinct, and it is not a proof that other states are unreachable. Goals retain exact position, velocity, quaternion, body rates, actual rotor thrust and held motor targets. The observed future suffix is empirical evidence; arbitrary neighboring states are not certified successful.

| Source | Earlier source frame | Goal sample into successful autonomous trajectory | Fixed prefix horizon | Recorded successful suffix remaining |
| --- | ---: | ---: | ---: | ---: |
| dev_center | 595 | 5 ms | 885 ms | 94.9 ms |
| dev_fast | 666 | 95 ms | 1,145 ms | 340.6 ms |
| dev_near_slow | 594 | 110 ms | 880 ms | 106.8 ms |
| stationary_0 | 181 | 100 ms | 1,145 ms | 336.3 ms |
| stationary_2 | 172 | 50 ms | 1,245 ms | 265.8 ms |

Horizon is 1.35 times the archived source-to-goal elapsed time, rounded up to a 5 ms control boundary. It is a fixed, preregistered engineering choice, not an optimized minimum time or a necessary physical limit.

## Frozen protocol and disclosed correction

The [original manifest](../runs/asm2-terminal/r6-1/manifest.json) was frozen before any optimization or candidate validation. It pins all starts/goals, controls, source/checkpoint hashes, registered geometry, native model, matching thresholds and the twenty-case cap.

**One pre-validation amendment:** the original protocol shared each nominal optimized prefix across its four perturbations. Before any candidate validation or refinement, analysis of free-flight translational dynamics identified a confound: a ±5 mm/s initial velocity change accumulates several millimetres of endpoint displacement over these horizons, exceeding the frozen 0.5 mm matching tolerance even if a nominal prefix were perfect. The [amendment](../runs/asm2-terminal/r6-1/refinement-amendment.json) preserves the original manifest and implementation, all twenty start/goal pairs, thresholds, physics and validation count. It adds bounded per-candidate refinement of the nominal warm start. No native validation outcome was used to change the design.

Consequently this experiment measures **reoptimized feasibility across the specified offsets**. It does not measure whether one open-loop control sequence works unchanged for neighboring starts. No tolerance is widened to obtain success, and no replacement case or adaptive extension is allowed.

## Direct Shooting and hard validation

Eight linearly interpolated physical CTBR knots give 32 decision variables. Every control is held for 5 ms. Collective is constrained to `[0.5,1.5] × mg`, each body-rate command to ±2 rad/s, exactly the existing actor map. The frozen allocator and four actual motor filters evolve inside the original detailed MuJoCo plant at 0.1 ms; there is no point-mass, instantaneous-actuator or collider-free optimization surrogate.

The initial guess resamples and clips archived controls from the source prefix followed by the observed autonomous segment. This is offline optimizer initialization only: no trajectory-tracking cost, policy imitation, actor update or scripted action inside the Fixed A continuation. The solver may change every knot.

Bounded SciPy least squares minimizes full terminal state error, path-envelope violations, signed-contact/unfinished-prefix penalties, allocation saturation and a small knot-difference regularizer. Quaternion error uses the shortest rotation vector, invariant to quaternion sign. Every optimization forward evaluation includes actual detailed-body contact detection. A signed contact truncates that trial. This creates a nonsmooth objective and may leave local solvers stuck near first-contact boundaries; successful optimization is **not** inferred from a low residual or a solver termination flag. Final acceptance imposes hard prefix, clearance and matching gates.

Matching requires all six groups below to pass simultaneously:

| State group | Maximum error |
| --- | ---: |
| Position, Euclidean norm | 0.5 mm |
| Velocity, Euclidean norm | 5 mm/s |
| Attitude, geodesic angle | 0.5° |
| Body-rate error, Euclidean norm | 0.05 rad/s |
| Actual rotor thrust, each rotor | 5 mN |
| Held target thrust, each rotor | 5 mN |

Position/velocity reuse the existing numerical sensitivity scales. The remaining thresholds are declared engineering state-match tolerances, **not measured physical uncertainty, official rules or a validated success-neighborhood certificate**. Actual continuation contact is evaluated independently.

Compute budget: five nominal solves, each at most 384 forward evaluations and a 900 s initiation-time cap; twenty candidate refinements, each at most 128 evaluations and a 360 s initiation-time cap. Caps count finite-difference evaluations and failed/early-truncated trajectories. Wall checks occur before each evaluation, so one in-flight integration can finish beyond the initiation cap. Total maximum is **4,480 optimizer prefix integrations**, in five parallel workers. No Fixed A continuation is queried inside either optimization stage. These forward integrations are explicitly separate from the **twenty final native candidate validations**; they are not PPO transitions.

All optimized controls are sealed before validation outcomes are opened. Each validation restores the archived complete initial state once, executes its optimized prefix, and checks the full physical envelope at every 0.1 ms physics step, allocator limits and any signed contact. Exact numerical distance between all detailed aircraft hulls and finite registered target hulls is measured on saved 10 ms samples and the endpoint, using separate query data so the live solver is unaffected. Native full-scene collisions are checked at every physics step. This is discrete native feasibility, not continuous-time CCD certification.

If the prefix is safe, control transfers in the **same MuJoCo instance**, retaining position, velocity, quaternion, rates, actual rotor thrust, held targets, solver state and time. Fixed A supplies every subsequent action for at most 2.3 s; there is no teleport to the intended goal or scripted suffix. Safe but off-target prefixes may continue for diagnosis, but their contacts cannot pass the strict connection gate. An unsafe/contacting prefix is stopped without an invalid handover.

The unchanged native scorer remains authoritative. Classification:

- **A:** safe prefix, full-state match, and subsequent native-qualified Fixed A contact.
- **B:** safe prefix and full-state match, but Fixed A continuation rejects or fails.
- **C:** no accepted matched connection for that candidate. Off-target autonomous contact, if any, is reported separately.

Three focused tests cover convex CTBR bounds, quaternion double-cover handling, and rejection of hidden-actuator mismatch. Final saved-data verification passed checks of initial-state/sign provenance, optimizer/native prefix reproducibility, physical motor evolution, continuity at transfer, Fixed A action reproduction, scorer replay and replay-file validity.


## Results — CONFIRMED, 2026-11-13

**C: no feasible full-state connection was found under the tested parameterization, horizons, action limits and compute budget.** A=0/20, B=0/20, C=20/20. This is not evidence of global unreachability. It establishes that this bounded single-shooting search did not produce a validated bridge into the empirically successful set.

Exactly **20 native candidate validations** ran, with no replacements or extensions. Five nominal warm-start solves plus twenty candidate refinements used **3,211 detailed-plant forward integrations** (maximum 4,480), taking 7,127.4 summed worker-seconds; summed worker time is not elapsed wall time. No PPO transitions ran. All cases are development data.

| Source | Candidates | Safe complete prefixes | Full-state matches | Fixed A continuations | Qualified suffix contacts |
| --- | ---: | ---: | ---: | ---: | ---: |
| dev_center | 4 | 0 | 0 | 0 | 0 |
| dev_fast | 4 | 0 | 0 | 0 | 0 |
| dev_near_slow | 4 | 0 | 0 | 0 | 0 |
| stationary_0 | 4 | 2 | 0 | 2 | 0 |
| stationary_2 | 4 | 1 | 0 | 1 | 0 |

Seventeen prefixes encountered **forbidden-airframe first touch** before their horizon; five of those also exceeded the speed envelope. Fixed A was **NOT_RUN** for these seventeen cases. Three prefixes passed native prefix safety/clearance and reached the horizon, but none matched the intended state. Their autonomous suffix failures are diagnostic, not category B, because B requires a successful full-state connection first.

| Safe but off-target case | Prefix / autonomous duration | Autonomous outcome | Minimum sampled finite target gap during prefix |
| --- | --- | --- | ---: |
| stationary_0-slope_minus | 1145.0 / 450.0 ms | SPEED_LIMIT | 184.29 mm |
| stationary_0-slope_plus | 1145.0 / 256.4 ms | CONTACT_POINT_SPEED | 195.24 mm |
| stationary_2-across_plus | 1245.0 / 141.7 ms | FORBIDDEN_AIRFRAME_FIRST_TOUCH | 150.69 mm |

The stationary_0 positive-slope suffix touched a designated front patch but failed contact-point speed (approximately 0.592 m/s tangential speed at first contact). The negative-slope suffix exceeded the speed envelope without signed touch. The stationary_2 positive-across suffix touched forbidden body geometry first. No successful state-transfer neighborhood was demonstrated.

The table below gives **actual endpoint errors**, not distances to a descriptor box. For interrupted prefixes the endpoint is the first signed-contact stop, before the requested horizon; these are not terminal states reached at the planned transfer time. Actual/held motor columns are maximum absolute per-rotor errors. All twenty fail matching.

| Candidate | Prefix end (ms) | Position (mm) | Velocity (m/s) | Attitude (°) | Rate (rad/s) | Actual / held motor (mN) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| dev_center-slope_minus | 791.0 | 22.40 | 0.124 | 7.46 | 0.984 | 24.81 / 29.26 |
| dev_center-slope_plus | 821.1 | 26.74 | 0.255 | 4.19 | 0.649 | 43.04 / 25.28 |
| dev_center-across_minus | 789.2 | 16.52 | 0.083 | 7.83 | 0.615 | 20.48 / 11.75 |
| dev_center-across_plus | 777.8 | 15.71 | 0.080 | 8.52 | 0.881 | 30.46 / 17.79 |
| dev_fast-slope_minus | 853.1 | 154.14 | 0.678 | 18.70 | 2.169 | 26.54 / 33.86 |
| dev_fast-slope_plus | 879.9 | 159.52 | 0.469 | 13.31 | 1.577 | 29.75 / 39.07 |
| dev_fast-across_minus | 851.9 | 152.49 | 0.673 | 18.83 | 2.166 | 26.42 / 33.85 |
| dev_fast-across_plus | 851.6 | 153.01 | 0.678 | 18.88 | 2.184 | 26.38 / 33.85 |
| dev_near_slow-slope_minus | 748.2 | 25.13 | 0.251 | 6.08 | 1.206 | 63.15 / 86.37 |
| dev_near_slow-slope_plus | 718.9 | 26.83 | 0.222 | 9.17 | 0.594 | 17.69 / 13.66 |
| dev_near_slow-across_minus | 551.7 | 50.47 | 0.717 | 34.01 | 0.876 | 19.61 / 39.09 |
| dev_near_slow-across_plus | 785.1 | 34.92 | 0.275 | 1.96 | 0.471 | 21.84 / 14.63 |
| stationary_0-slope_minus | 1145.0 | 24.10 | 1.063 | 26.70 | 0.636 | 50.84 / 29.63 |
| stationary_0-slope_plus | 1145.0 | 11.20 | 0.080 | 4.34 | 0.319 | 31.55 / 20.83 |
| stationary_0-across_minus | 917.8 | 192.00 | 0.598 | 11.98 | 0.554 | 98.27 / 163.22 |
| stationary_0-across_plus | 914.9 | 191.96 | 0.617 | 12.47 | 0.604 | 93.57 / 155.91 |
| stationary_2-slope_minus | 1059.5 | 158.47 | 0.159 | 16.42 | 2.412 | 23.78 / 26.52 |
| stationary_2-slope_plus | 1069.8 | 159.79 | 0.217 | 17.51 | 2.397 | 24.16 / 26.11 |
| stationary_2-across_minus | 1073.0 | 159.16 | 0.247 | 18.40 | 2.368 | 24.09 / 25.74 |
| stationary_2-across_plus | 1245.0 | 30.29 | 0.256 | 9.62 | 1.979 | 9.49 / 25.75 |

## Interpretation and limits

**CONFIRMED:** no source family supplies a validated earlier-to-success connection in this study. None of the three safe prefixes misses only an invisible actuator component: position, velocity, attitude and rate errors also exceed their gates. The most promising safe positive-slope stationary_0 endpoint is still 11.20 mm, 0.080 m/s, 4.34° and 0.319 rad/s away, with 31.56 mN actual-rotor and 20.86 mN held-target error. Its front-patch contact is not qualification.

All executed optimized-prefix controls remained within the original CTBR map, with zero allocator saturation. Planned commands for dev_fast and dev_near_slow approached the ±2 rad/s rate-command bounds; other families did not require persistent command-limit operation. Recorded actual rotor thrust stayed at least 0.219 N from either rail across these runs (including suffixes). The `control_effort` command maxima/fractions in outcome JSON refer to the full planned prefix, including unexecuted commands after an early stop; recorded executed commands are in `physics.npz`. These findings do not show that stronger actuation is necessary or sufficient: the solver never found a feasible matched connection, so no minimum-effort connection was measured.

**INFERENCE:** optimization reliability is a significant unresolved confound. Seventeen selected prefixes terminate at contact; the objective changes discontinuously with the stopping step, and terminal residuals then describe a premature endpoint. The local least-squares solver can terminate on small parameter changes without satisfying constraints. Single shooting, eight knots, one clipped/resampled initialization per source and a fixed horizon also restrict the search. Envelope penalties were sampled at control boundaries inside optimization, whereas final validation checked every physics step. Hard native acceptance was not relaxed to accommodate those approximations.

The initial guesses were clipped and stretched historical commands, not established collision-free solutions under the actor's bounds. Presence of these earlier starts in Phase R/R3 records is not a certificate of a feasible CTBR connection. The experiment cannot distinguish an inaccessible target under the selected horizon from an inadequate local optimizer/initialization. No global unreachability or actuator-insufficiency conclusion follows.

**Sensitivity:** per-candidate reoptimization found safe prefixes for both slope signs at stationary_0 but neither across sign, and only the positive-across case at stationary_2. Every case still failed the full connection test. Different local optimization outcomes confound physical sensitivity, and no nominal final validation was added. This provides neither common-control robustness nor a successful multidimensional transition region. The offsets are numerical probes, not measured manufacturing uncertainty.

**Informed resets:** no new state is validated as a successful bridge reset by this study. The three safe trajectories can be retained as failed/off-target physical transitions, preserving family and trajectory identity; they should not be relabeled successful informed resets. Previous R5/R5.1 successful trajectories remain empirical autonomous evidence. Planned feasibility, autonomous qualification and future RL learnability remain separate. No teacher actions entered any actor.

G3.1.2 post-contact body loading remains separate from this unchanged first-contact benchmark. No official impact-score or force-area-time claim is made. No 5060 outcomes were pooled with these runs.

## Reproducibility and artifacts

The implementation is `src/rm27_asm2/informed_connection.py`; the frozen config is `configs/rm27_asm2_r61.json`. Original input/geometry/checkpoint hashes and complete initial/goal states are in [manifest.json](../runs/asm2-terminal/r6-1/manifest.json). The disclosed per-candidate refinement is pinned in [refinement-amendment.json](../runs/asm2-terminal/r6-1/refinement-amendment.json); [selection.json](../runs/asm2-terminal/r6-1/selection.json) sealed all twenty optimized result hashes before validation.

Every optimization retains evaluation cost, stopping reason, best controls and state samples. Every final case retains `physics.npz` (0.1 ms full state, actual thrust, held targets; 5 ms CTBR/allocation), `contacts.json`, `outcome.json` and `replay.npz`. All **188,376 physics intervals** are preserved. This is the final validation record count, distinct from optimizer integrations. [results.json](../runs/asm2-terminal/r6-1/results.json) contains all twenty outcomes; [analysis.json](../runs/asm2-terminal/r6-1/analysis.json) contains the saved-data verification and per-source counts.

Three focused tests PASS. Offline verification PASS: exact initial state and perturbation transforms; native 0.1 ms motor-filter evolution; actuator allocation and CTBR bounds; bitwise agreement between all saved optimizer prefix states and final validation samples; unchanged scorer replay on all twenty cases; all 171 autonomous suffix commands reproduced from frozen Fixed A; transfer state/actuator sample continuity; all twenty archives accepted by the existing replay loader. No additional physics rollout was used for reproducibility verification.

Saved-data verification/figure reproduction (does not rerun optimization or native cases):

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:. \
/tmp/rm27-g311-env/bin/python -m rm27_asm2.informed_connection_report
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src:. \
/tmp/rm27-g311-env/bin/python -m unittest discover -s tests -p test_asm2_informed_connection.py
```

Representative recorded-state replays: [center collision](../runs/asm2-terminal/r6-1/cases/dev_center-slope_minus/replay.npz), [stationary_0 safe transfer then speed rejection](../runs/asm2-terminal/r6-1/cases/stationary_0-slope_plus/replay.npz), [stationary_2 safe transfer then forbidden touch](../runs/asm2-terminal/r6-1/cases/stationary_2-across_plus/replay.npz). These use the existing viewer format; they are saved states, not rerun controllers.

![Optimization and full-state matching](../runs/asm2-terminal/r6-1/matching-and-optimization.png)

![Native connection trajectories](../runs/asm2-terminal/r6-1/connections-3d.png)

![Representative actuator and state histories](../runs/asm2-terminal/r6-1/actuator-and-state-timelines.png)

Timeline vertical markers denote the prefix endpoint, including interrupted prefixes; they indicate an actual handover only where `fixed_a_run=true`.

## Decision and one next experiment

**C / No-Go for a new PPO budget based on these connections.** No matched bridge exists in this finite evidence, and no policy promotion is justified. Phase 4 remains the learned baseline and Fixed A the continuation reference. R5.2 remains pending and was not launched.

**Exactly one recommendation — NOT_LAUNCHED:** a bounded clearance-aware multiple-shooting feasibility-restoration study initialized from the three saved safe prefixes, retaining the same CTBR bounds, full-state/actuator matching gates and native Fixed A suffix test. This directly tests whether local single-shooting sensitivity and contact-truncated residuals explain the negative result. It requires moderate offline optimizer work (segment continuity and clearance constraints), without a plant or policy change. Its research value is to resolve the numerical-feasibility confound before spending more PPO transitions; successful optimizer output would still require native matching plus autonomous qualification. It is not a recommendation to increase action limits or weaken contact criteria.
