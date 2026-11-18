# ASM2 — Independent terminal-set feasibility

**COMPLETE · CONFIRMED · 2026-11-13 America/New_York · GO for the bounded feasibility hypothesis.** Seven of twelve optimized-prefix/Fixed A connections achieved original native-qualified contact. Success covers all three saved R6.1 seed identities and both original source families. All twelve prefixes passed the native safety gates. These are planned-prefix feasibility witnesses, **not autonomous RL successes**, a certified viability basin, or a robustness estimate. Zero PPO transitions.

| Saved R6.1 seed | Original source | Feasible prefixes | Fixed A qualifications |
| --- | --- | ---: | ---: |
| stationary_0-slope_minus | stationary_0 | 4/4 | 3/4 |
| stationary_0-slope_plus | stationary_0 | 4/4 | 3/4 |
| stationary_2-across_plus | stationary_2 | 4/4 | 1/4 |

The requested distinct-saved-seed condition passes, as does the more conservative condition requiring distinct original sources. The twelve candidates are correlated development samples, not twelve independent families.

## Inputs and bounded method

Read [R5](ASM2_R5_BACKWARD_FRONTIER.md), [R5.1](ASM2_R51_CROSS_TRACK.md), [R6](ASM2_R6_SKILL_BRIDGING.md), [R6.1](ASM2_R61_INFORMED_CONNECTIONS.md) and the active roadmap. Implementation and raw outputs are isolated in [terminal-set](../runs/asm2-terminal/terminal-set/); no Auxiliary PPO code, outputs, process or result is a dependency.

The terminal collection contains **125 actual states from all 25 successful R5/R5.1 autonomous continuations**, covering five source families. Five control-boundary samples per trajectory span its start through the latest boundary with at least 40 ms of observed successful future. Each sample preserves position, quaternion, velocity, body rates, actual rotor thrust and held motor targets, with original trajectory path, sample index, elapsed time, successful-future duration and file hashes. These are observed successful suffix states; their neighbors are not presumed successful.

The [manifest](../runs/asm2-terminal/terminal-set/manifest.json) freezes support, starts, seed controls, protocol, actor and source pins. The three earlier starts and eight-knot controls come directly from the three saved safe R6.1 prefixes. Horizons remain 1.145 s for stationary_0 and 1.245 s for stationary_2. All knots are optimized with existing bounded SciPy least squares and the detailed native R6.1 shooting infrastructure. CTBR remains collective `[0.5,1.5] mg`, rates ±2 rad/s; native physics is MuJoCo 3.12.0 at 0.1 ms, with 5 ms control and the original allocator/motor filters.

The terminal residual selects the nearest **complete-state sample in the union**, rather than matching one prescribed archived state. Search scales are 10 mm position, 0.1 m/s velocity, 0.1 rad attitude, 0.5 rad/s body rate and 50 mN per actual/held rotor value. These are disclosed optimization/neighborhood conventions, not contact thresholds or certified tolerance radii. All six error groups must lie within those scales for the reported neighborhood flag; **actual native Fixed A qualification is additionally required**. No physical continuity tolerance is introduced: transfer state is unchanged exactly.

Best safe parameter vectors are retained at each 100 charged evaluations and at termination, yielding four unique final candidates per seed. All twelve were sealed before opening any new native outcomes. No policy continuation was queried inside optimization. The [selection](../runs/asm2-terminal/terminal-set/selection.json) retains controls, evaluation identities and complete cost histories. No multiple shooting was needed: all 1,197 completed optimizer integrations stayed contact-free and passed the optimizer's control-boundary envelope checks, while residuals continued decreasing. Native acceptance checks the envelope at every physics step.

## Actual optimizer costs and convergence

| Seed | Detailed integrations | Charged evaluations | Initial → final terminal residual norm | Stop |
| --- | ---: | ---: | ---: | --- |
| stationary_0-slope_minus | 399 | 400 | 11.2994 → 0.04082 | Evaluation cap |
| stationary_0-slope_plus | 399 | 400 | 2.0785 → 0.05127 | Evaluation cap |
| stationary_2-across_plus | 399 | 400 | 6.0211 → 0.03312 | Evaluation cap |

**1,197 actual detailed-plant integrations**, including numerical differentiation, plus **three failed pre-integration objective calls**, conservatively charged as **1,200 evaluations**. No interrupted or contact-truncated integration occurred. No solver convergence certificate is claimed: all three stopped at the cap while still improving.

A ledger-only error stopped the first run after one integration per worker: the existing exclusive-create JSON writer rejected the next ledger update. Its traceback, initial ledgers and [correction record](../runs/asm2-terminal/terminal-set/ledger-correction.json) remain preserved. The corrected run uses append-only reservation/completion records. Those three integrations, failed calls and worker time were retained in the original budget; no validation was repeated and no scientific criterion changed.

The corrected workers measured **2,692.055 summed seconds**. The initial failed workers' recorded clocks bound their summed duration at **69.174–77.103 seconds**; their final timers were not captured before the exception. Thus actual total worker time is bounded at **2,761.229–2,769.158 seconds (46.02–46.15 minutes)**, not falsely reported as an exact measurement. Conservatively charged total is **2,782.055 seconds (46.37 minutes)**. This includes worker setup and remains below 7,200 seconds. Three seed families, twelve candidates and exactly twelve new native validations exhaust the allocated evaluation/candidate budget. Optimization stopped before validation; there was no outcome-driven refinement or extension.

## Native outcomes

Candidate suffixes 1–4 denote successive sealed optimization checkpoints. All prefixes were feasible. Every candidate received one same-instance autonomous continuation.

| Seed / candidate | Terminal residual norm | Neighborhood flag | Fixed A outcome | Autonomous duration (ms) |
| --- | ---: | --- | --- | ---: |
| stationary_0-slope_minus / 1 | 2.08192 | Outside | Forbidden airframe first touch | 233.5 |
| stationary_0-slope_minus / 2 | 0.42449 | Inside | Qualified | 235.0 |
| stationary_0-slope_minus / 3 | 0.11208 | Inside | Qualified | 236.1 |
| stationary_0-slope_minus / 4 | 0.04082 | Inside | Qualified | 235.8 |
| stationary_0-slope_plus / 1 | 0.55585 | Inside | Outside registered footprint | 348.5 |
| stationary_0-slope_plus / 2 | 0.30537 | Inside | Qualified | 339.9 |
| stationary_0-slope_plus / 3 | 0.07028 | Inside | Qualified | 341.5 |
| stationary_0-slope_plus / 4 | 0.05127 | Inside | Qualified | 341.1 |
| stationary_2-across_plus / 1 | 1.00332 | Inside | Forbidden airframe first touch | 248.6 |
| stationary_2-across_plus / 2 | 0.30664 | Inside | Contact-point speed | 246.0 |
| stationary_2-across_plus / 3 | 0.09236 | Inside | Qualified | 250.6 |
| stationary_2-across_plus / 4 | 0.03312 | Inside | Forbidden airframe first touch | 245.9 |

The norm aggregates all residual components; a norm slightly above one can still have each individual error group within its search scale. Neither flag nor norm awards contact success.

Selected terminal provenance:

- All stationary_0 negative-slope candidates target **R5 stationary_2-349-nominal, sample 650 (65 ms)**: the union permits a different source's successful suffix.
- stationary_0 positive-slope candidate 1 targets **R5 stationary_0-330-nominal, sample 950 (95 ms)**; candidates 2–4 target **R5.1 stationary_0-330-across_plus at the same elapsed time**.
- All stationary_2 candidates target **R5 stationary_2-346-minus, sample 650 (65 ms)**.

The stationary_2 fourth candidate is closer than its successful third candidate yet fails. Its errors to the observed successful sample are only approximately **2.03 µm position, 0.107 mm/s velocity, 0.0201° attitude, 0.00352 rad/s rate, 0.830 mN actual and 0.536 mN held rotor thrust**. Closeness—even within the old R6.1 numerical matching scales—does not guarantee the frozen first-contact label. The rejection is retained; no loaded-contact substitution or scoring relaxation was applied. Across all failures: three forbidden-airframe first touches, one footprint rejection, one contact-point-speed rejection; no envelope exit, timeout or allocator saturation.

## Continuity, verification and preserved evidence

Each validation restores the archived early physical/motor state once, executes the optimized prefix, then continues with Fixed A in the **same MuJoCo instance**. No state replacement, motor reset, controller initialization or clock restart occurs at transfer. The complete MuJoCo integration-state vector is saved immediately before and after the transfer boundary and is bitwise identical. The native rate allocator and Fixed A are stateless; the 45 actor inputs contain current relative position repeated ten times, current velocity, attitude and rates—not a recurrent or temporal observation buffer. Motor filter state is retained in `data.act`, and held targets in `data.ctrl`.

The total episode limit remains **2.3 seconds from the early reset**. Transfer leaves 1.155 or 1.055 seconds, respectively, rather than granting a fresh continuation episode. Every trial finishes inside that remaining time.

Full detailed-scene signed contacts and envelope constraints are checked at every 0.1 ms step. Separate read-only finite-body distance queries at 10 ms samples and transfer are positive for every prefix; the minimum across validations is **117.563 mm**. This is sampled native feasibility, not continuous-time collision certification. Actuator limits, CTBR limits and allocation are unchanged.

[Offline verification](../runs/asm2-terminal/terminal-set/verification.json) **PASS**: all 75 input pins verified; **174,425 physical intervals** preserved; exact early state/motor restoration and exact optimizer/native terminal agreement; complete motor holds and analytic filter evolution (maximum residual **5.55×10⁻¹⁷ N**); all **667 autonomous actor commands** reproduced exactly; zero full-history envelope violations; original scorer results reproduced exactly; saved replays match recorded states and load through the existing viewer. Target-metric checks also passed for quaternion sign invariance and sensitivity to hidden actuator mismatch. Verification performed zero additional integrations or candidate rollouts.

Every [case](../runs/asm2-terminal/terminal-set/cases/) contains `physics.npz`, `contacts.json`, `outcome.json`, and `replay.npz`. Successful examples: [stationary_0](../runs/asm2-terminal/terminal-set/cases/stationary_0-slope_minus-4/replay.npz), [stationary_2](../runs/asm2-terminal/terminal-set/cases/stationary_2-across_plus-3/replay.npz). The [closer but rejected stationary_2 case](../runs/asm2-terminal/terminal-set/cases/stationary_2-across_plus-4/replay.npz) is also preserved. [Raw results](../runs/asm2-terminal/terminal-set/results.json).

![Optimizer convergence and recorded connections](../runs/asm2-terminal/terminal-set/feasibility.png)

## Interpretation and one next recommendation

**CONFIRMED:** the bounded search found actuator-feasible, native collision-safe connections from earlier outward-rotation states into sampled-success neighborhoods, followed by autonomous native-qualified Fixed A contact in two original source families. This resolves the existence question for these starts under the provisional frozen plant and original scorer. It does not prove broader reachability, a viability basin, robust neighborhoods, learned early-rotation competence or post-contact safety.

**INFERENCE:** safe-prefix reuse and terminal-set targeting overcame limitations of the prior failed search without requiring multiple shooting. The study changes terminal sample selection and residual scaling and spends additional optimization compute; it does not isolate terminal-set targeting as the sole causal improvement over R6.1. R6.1's negative result remains valid for its tested search. The present nonmonotonic continuation outcomes show why state similarity cannot replace native validation.

**One recommendation, NOT LAUNCHED:** a twelve-validation early-state robustness study using one saved successful control sequence from each original source (`stationary_0-slope_minus-4` and `stationary_2-across_plus-3`), with six preregistered early-state perturbations per sequence and no reoptimization. Preserve full actuator/controller continuity and the original native gates. This would test whether the demonstrated connections tolerate initial-state variation before drawing any learning or neighborhood-robustness conclusion.

Phase 4, Fixed A, physics, detailed geometry and scorer remain unchanged. No reward/actor changes, teacher training, imitation dataset, policy trajectory reference, PPO transitions, 5060 scorer, hardware, PX4, Sim-to-Real, commit or push was used.
