# R9.1 — Corrective-demonstration pilot proposal

**Status: PROPOSAL / NOT_RUN.** R9 remains Scientific / Promotion No-Go. This is a bounded follow-up to R9, not a new PPO campaign. Current authorization covers this document and existing-evidence inspection only. No new fitting, optimizer simulations, native rollouts or replays are authorized. Execution requires separate explicit approval of this entire budget and a pinned implementation manifest. No commits/pushes.

## Question and evidential basis

Can adding qualified corrections at states actually visited by the frozen R9 policy improve autonomous first-contact precision, beyond an equal amount of refitting on the old data?

[Completed R9](ASM2_R9_DEMONSTRATION_REGULARIZED_PPO.md): final earlier0/36, demonstrated starts0/3, braking25/36; weighted action error improved relative to R8 B without qualified bridging. The three demonstrated starts comprise only two source families. The [terminal-set feasibility study](ASM2_TERMINAL_SET_FEASIBILITY.md) found7/12 planned-prefix/Fixed A qualifications, proving some archived demonstrations feasible, not proving recoverability from the new policy-visited states. Distribution drift and precision are hypotheses, not established exclusive causes.

A correction is an optimized prefix followed by a genuinely qualified Fixed A suffix. Neither an optimizer cost nor membership in a descriptor neighborhood qualifies a label. This is one offline correction/refit round, not iterative DAgger, PPO, recovery deployment or a claim of certified reachability.

## Frozen data selection rules

Use only final R9 demonstrated-start archives, chosen before generating any corrections. Do not select an intermediate R9 checkpoint or a favorable native anchor after seeing outcomes.

| Identity | R9 native-record directory | Sampling times after original start | Fixed prefix end time |
|---|---|---|---|
|stationary_0-slope_minus|r9/native-records/0604|0.285,0.570,0.855 s|1.145 s|
|stationary_0-slope_plus|r9/native-records/0605|0.285,0.570,0.855 s|1.145 s|
|stationary_2-across_plus|r9/native-records/0606|0.310,0.620,0.930 s|1.245 s|

Paths above are under `.agents/runs/asm2-terminal/`. Times are the quarter/half/three-quarter fractions of the original terminal-set prefix horizon rounded down to5ms. They cover early/middle/late policy drift without choosing frames by prospective correction success. Exactly9 candidate states; no replacements. The corresponding remaining prefix durations are0.860/0.575/0.290 s and0.935/0.625/0.315 s. This schedule is an experimental convention, not a required flight phase or actor input.

At each time extract the state immediately BEFORE the next CTBR update, including qpos/qvel, actual rotor thrust, previous held targets, native integration state where available, target registration and elapsed clock. Use recorded arrays, not interpolation between frames or regenerated witness states. The control boundary and held-target indexing must be verified offline against the capture implementation. Actor observation must reproduce the archived45-D semantics exactly. Native allocator and Actor are stateless here; do not invent a history buffer. Complete restoration must preserve the original remaining horizon `2.3 − t_start` seconds; no fresh2.3s allocation to the suffix.

Before freezing executable manifest, check all9 states for finite values, finite-body separation, original envelope and actuator limits, and absence of prior signed contact. An invalid/missing state consumes its manifest identity and is reported; do not move it to an easier time. The9 frame rows, dtype, source hashes and physical/actuator vectors must be saved before any optimization. No held-out/fresh source or R9 favorable earlier anchor enters correction selection.

## Correction generation and acceptance

Reuse `.agents/runs/asm2-terminal/terminal-set/experiment.py` and `src/rm27_asm2/informed_connection.py` numerical/state machinery in an isolated adapter; historical files remain untouched. Use the original terminal-set support samples, full-state residual scales, path penalties, target-frame registration and fixed detailed plant. Eight linearly interpolated CTBR knots, original collective[0.5,1.5]mg and commanded rates±2rad/s. Match position, velocity, attitude, body rate, actual rotor thrust AND held targets to the empirical support union; no exact-single-state requirement and no position-only cost.

For each identity, warm-start from the same seed's earliest lexicographic native-qualified terminal-set candidate, crop at the declared elapsed time and resample to8 knots. This is initialization only, not a tracking objective. Use one least-squares solve with inherited tolerances, at most128 actual forward-integration calls including finite-difference probes. No multi-start, horizon sweep, adaptive support expansion or policy-continuation calls inside optimization. Reserve every call before integration; crashes/nonfinite/aborted calls count. Cap checks include Jacobian probes. A hard call cap supersedes solver-internal iteration limits; no wall-clock deadline.

Select one vector per state before any native validation: completed contact-free/envelope-valid prefixes first, then lowest inherited scalar objective, then earliest call index. If none safe, retain the lowest-cost attempted vector and explicitly mark it. Seal all9 controls and costs before opening validation outcomes. No candidate refinement after validation.

Each of the9 native validations restores the sampled state once, executes its fixed optimized prefix, and, only if safe, transfers to frozen Fixed A in the SAME instance without state/motor/time reset. Use remaining original horizon. Record all failures, full0.1ms state/actuator/contact histories and5ms actions. Qualification requires prefix safety and actual unchanged native-qualified first contact after autonomous transfer; neighborhood distance is reported separately. Unsafe prefixes stop, and Fixed A is NOT_RUN, not a policy failure. No official impact-score claim or post-contact tail.

**Label gate:** at least6/9 qualified correction trajectories, with at least one from EACH of the three start identities, covering both source families. Otherwise STOP before fitting: correction-supervision feasibility not established under this search budget. No feasible correction found does not imply physical unreachability.

Only entire validated prefix+Fixed A trajectories provide new action labels. A failed prefix, failed suffix or truncated trajectory supplies no positive corrective-action labels. Exclude terminal/padded controls without an actual executed interval. Keep source/seed/sample-time/prefix-versus-suffix provenance. Correlated correction frames are never independent examples of capability.

## One matched supervised refit

Both arms start from the same final R9 checkpoint393216 (SHA256 `091d9c8f00e30d202b3ee59656c5c5b04ad2bfd759cf53624b83658267a41562`). This is the policy that generated the chosen states, selected for distribution consistency, not retention quality. Its25/36 braking is NOT an acceptable relaxed baseline: the final gate remains32/36.

- **A — equal-compute control:** old R8 data only; total weight0.5 original demonstrations +0.5 original autonomous retention.
- **B — corrective data:**0.25 original demonstrations +0.25 accepted corrections +0.5 original autonomous retention.

Within each category, equal original-source mass, then equal seed identity (for corrections), trajectory and frame mass. Original-category weighting follows the existing R8 source/trajectory scheme. B's correction share replaces old-demonstration mass; it does not increase total loss scale or reduce retention mass. All accepted corrections are included; no outcome-ranked subset.

Use original `demonstration_learning.py` action conversion: raw target `atanh(clip(a,−1+1e−6,1−1e−6))`, normalized Actor order[p,q,r,T], physical order[T,p,q,r]. Loss is `0.5 E_w[mean_d((mu_theta(x)_d − y_d)^2 exp(−2 log_std_d))]`. Freeze log_std to the common starting checkpoint. Update Actor mean network only; critic, ValueNorm, original PPO optimizer states, reward, native scoring and45-D inputs unchanged. No expert action, phase, future state or motor privileged input at evaluation. New supervised Adam is discarded after fitting.

Fixed settings inherited from the successful execution of R8 fitting code: Adam lr1e−4, batch128,16 updates/epoch, gradient clip1. **18 epochs**,288 updates per arm, seed82714, fresh supervised Adam for each arm; same RNG reset and architecture.18 is the historical R8 selected dose, not selected from this pilot's outcomes. Save epoch0/6/12/18 for offline loss diagnostics; evaluate ONLY epoch18. No early-stop/checkpoint selection on native results, hyperparameter sweep, refit or repeated correction round. A/B consume equal36,864 sampled labels each, not environment transitions.

Before fitting, tensor tests must confirm action inversion/order, weights summing to1, gradient only into Actor mean, unchanged protected tensors, and equal initialization. Detect duplicate/near-identical Actor observations with inconsistent action labels and report associated hidden motor-state differences; do not discard conflicts based on desired answers. Nonfinite data, observation mismatch or restoration failure blocks execution. Numerical failures stop; no replacement training seed.

Loss and per-source diagnostics are IN-SOURCE, including the sampled policy states. No new out-of-source predictive claim: there are only two correction families. Do not split neighboring frames randomly or call original-data performance a fresh validation set. The independent behavioral test below determines usefulness.

## Native evaluation, fixed selection and gates

Seal BOTH epoch18 checkpoint hashes before any new policy evaluation. All policy evaluations start at the ORIGINAL three early starts, never at the corrected intermediate states. Run autonomous deterministic Actor throughout, without optimized prefixes or Fixed A switching.

For each start, use nominal plus four variants: target-slope−/+, target-across−/+. Each variant applies signed0.5mm position and5mm/s velocity along the SAME registered tangent; preserve attitude, rates, actual/held motors and every other component. These are numerical sensitivity probes, not manufacturing tolerances. Variants are displacements relative to the existing named start, which itself may already be perturbed. Freeze the15 explicit state vectors and identify any duplicates across starts; do not replace duplicates, and report unique identities separately.

Both arms receive15 local cases plus original36 braking and30 terminal cases:81 per arm. No fresh/held-out suite and no generalization claims outside the tested development families. Run the unmodified R9 final Actor on the12 nonnominal local cases for matched baseline context; reuse archived3 nominal and36/30 retention results. Baseline results cannot tune labels, fit or selection.

**Local mechanism GO requires ALL:**

1. B qualifies at least2/3 nominal starts, covering BOTH stationary_0 and stationary_2; at least two more qualified nominal starts than A.
2. Each of those two successful nominal starts qualifies at least4/5 of its local cases, including nominal; B totals at least four more qualifications than A across15 cases. Report source-family correlations; two stationary_0 variants do not constitute two independent families.
3. B braking≥32/36, terminal≥27/30, zero flight-envelope exits in both retention cohorts. No gate is lowered to R9's25/36. These absolute gates apply irrespective of A's performance.
4. Two qualifying B nominal traces reproduce with identical native outcomes and full deterministic physics histories; no privileged input or evaluation-time expert command.

If label gate passes but the refit fails these gates, scientific No-Go for this fixed corrective-data mechanism/dose. If both A/B improve similarly, extra refitting rather than correction specificity remains sufficient explanation. Passing is only GO for local correction learning and a separately proposed next study, NOT baseline promotion or earlier multi-source skill bridging. No persistence or global robustness claim from one final checkpoint.

Report each contact reason separately: forbidden airframe/target and simultaneous groups, no signed touch, footprint, flight-volume exit, flight speed limit, contact-point speed; also first-touch location/time/normals, state/action deviation, qualified source/anchor counts and actuator evolution. Keep signed intersection, loaded contact and official impact qualification distinct. Loss improvement alone cannot pass any gate.

## Hard proposed budget — no current execution authorization

| Item | Maximum new work |
|---|---:|
|PPO transitions|**0**|
|Correction optimizer native forward integrations|9×128 = **1,152**|
|Final correction validations incl. Fixed A suffix|**9**|
|Supervised fitting|2×288 = **576** optimizer updates;73,728 label draws|
|A/B local+retention native evaluations|2×(15+36+30) = **162**|
|Unmodified R9 local perturbation baseline|**12**|
|Deterministic replay reproductions|**6**|
|Total native validation/evaluation/replay episodes, excluding declared optimizer integrations|**189**|
|Total simulator call reservations including optimizer|**1,341**|

Optimizer calls are real native physics work, NOT free/offline dataset processing. Their maximum individual duration is0.935s; the aggregate maximum requested prefix time is681.60 simulator-seconds (128×sum of the9 prefix durations), before early termination. Each final validation/evaluation/replay has at most the original2.3s episode horizon, with correction states receiving only remaining time. Log optimizer physics steps and evaluation physics steps separately. No hidden optimizer rollout budget or arbitrary deadline.

Replay priority: first two lexicographic B qualified nominal starts spanning the two families, same-case A comparisons, then two B failures by unrepresented reason/lexicographic order. If successful identities do not exist, fill remaining slots with distinct B cases by reason then identity; such replays cannot satisfy the success gate. Maximum6, no replacements. Viewer files generated from existing records consume no new rollout; any new physical reproduction consumes one of6.

All reservations precede execution; interrupted/failed calls consume budget. No retries, extension, alternative solver, additional seed or unused-budget reassignment. Stop after failed label gate or material contract defect. The unused allowance expires. Preserve all attempted controls, failures and source pins. No R8/R9 training reruns or historical artifact edits.

## Required freeze and deliverables before any future launch

Implementation is NOT_RUN. Proposal is concrete, but not an executable freeze: no new code/optimizer test has run in this documentation-only task. Future authorized preflight must pin the9 actual sampled states,15 evaluation states, warm-start controls/support, remaining-horizon semantics, source/runtime/model/scorer hashes, R9 final/Fixed A/Phase4 checkpoints, original data, new adapter/tests and this specification. Missing provenance or infeasible state restore blocks dependent work; never fabricate compatibility. The archived R9 completion receipt records the former roadmap/plan hashes; subsequent proposal-only edits to those living documents are expected, not changes to R9 evidence.

Deliver correction ledger and all9 outcomes, accepted-label provenance/class coverage, exact A/B sampling/loss/update records, frozen final weights, matched native table with source grouping, complete physical/motor/action archives,6 replay results and one local Go/No-Go decision. Record newly produced labels as optimized controls or genuine Fixed A actions, never as autonomous actions from the learned candidate.

Only isolated adapters/tests are proposed; no default learner, architecture, observation, reward, controller, plant or scorer replacement. R9 historical documents and all evidence remain immutable. This pilot requires a new explicit authorization; R9's previous393216/831 allowance cannot be reused.
