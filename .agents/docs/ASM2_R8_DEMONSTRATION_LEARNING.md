# ASM2 R8 — Demonstration-assisted autonomous policy learning

**COMPLETE — scientific No-Go and baseline promotion No-Go.** Both arms completed exactly 393,216 additional PPO transitions (786,432 total). BC learned partial recentering/contact behavior but qualified none of the three demonstrated starts. B briefly qualified one earlier anchor, then lost it. No multi-anchor or held-out earlier improvement was established.

R8 explicitly permits action demonstrations. This changes the learning premise from earlier pure PPO studies while preserving the 45-D Actor, Direct-CTBR interface, plant, reward and native first-contact scorer. Planned connections are evidence of feasible actions, never counted as autonomous policy successes. G3.1.2 post-contact loading remains separate.

## Dataset and alignment

Seven complete native-qualified terminal-set connections cover three saved seeds in two original families (`stationary_0`, `stationary_2`). Twenty-five successful R5/R5.1 autonomous trajectories across five development families supply braking/terminal retention. No new optimization or demonstration was generated. Complete raw trajectories remain in their original archives; the compact dataset preserves source paths/hashes, control-boundary physical state and rotor/held targets for verification only.

The dataset contains 3,439 control pairs. Observation reconstructs the native float32 input: current target-relative position repeated ten times, world velocity, transposed rotation matrix and body rates. Labels invert executed `[T,p,q,r]` into raw Actor `[p,q,r,T]` using the existing hover scaling and inverse tanh. No command required clipping at the declared inverse-tanh epsilon 1e-6. Every archived Fixed A suffix command reproduced exactly; batched allocator error was at most 1.11e-16N. Complete first-order motor histories and transfer-state equality passed. Three focused tests verify action order, native observation parity and source exclusion/weighting. The terminal-set archive's documented ledger-only source correction was checked explicitly; original manifest and correction remain unchanged.

## BC method

Initialize the preserved Fixed A Actor. Freeze its Gaussian log standard deviation; optimize only Actor mean parameters using a fixed-variance Gaussian squared-error loss on pre-tanh targets. BC uses a separate Adam at 1e-4, batch 128, 16 steps per epoch, at most 80 epochs, patience 10 and seed 82714. Half the sampling probability belongs to demonstrations and half to retention; within each category, sources and then trajectories receive equal mass. This avoids weighting long or densely sampled trajectories as independent evidence.

Two source-excluded validation folds hold out all `stationary_0` or all `stationary_2` records, including retention records from that source. There is no adjacent-frame split. Their best epochs were 34 and 18; the predeclared conservative minimum selected 18 epochs for a final refit to all seven demonstrations and retention trajectories. Fold validation measures action prediction, not closed-loop control or broad generalization. The final refit has seen both demonstration families.

`bc.pt` preserves the original critic, value normalizer, parent PPO optimizer states and exploration log standard deviation. BC's optimizer is discarded. Both PPO arms start with identical parent Adam state; only B's Actor mean weights differ. This preserves unchanged control A continuation, with the inherited Adam moments disclosed as a design choice.

## Matched PPO and evaluation protocol

A starts Fixed A; B starts the BC Actor. Ordinary PPO uses no imitation term, auxiliary classifier or modified reward. Both retain seed 42722 and the same reset distribution: 10% terminal, 45% braking, 20% late rotation, 25% earlier bridge. Each is authorized 393,216 additional transitions, with checkpoints/native suites every 98,304 and one charged save/restart. There is no wall-clock deadline or early-zero-success futility gate.

Development checkpoint selection is frozen in `configs/rm27_asm2_r8.json`: require braking ≥32/36, terminal ≥27/30 and no retention envelope exit; rank by distinct earlier successful anchors, robust 3/3 anchors, earlier contacts, braking contacts, then latest endpoint. If no checkpoint passes retention, retain the final endpoint for analysis without promotion. Seal selected hashes before fresh evaluation. Also evaluate the final checkpoints to expose loss of intermediate gains.

Three unique demonstrated starts are evaluated separately. Every evaluated action, from time zero onward, comes from the learned Actor; no optimized prefix, trajectory tracking, Fixed A switch or phase input executes. Fresh earlier offsets belong to development sources; separate rotation/braking/terminal/reached-handover cohorts test genuinely separate original sources. A descriptor entry is never qualification.

## BC results (before PPO)

The two source-excluded validation errors fall from 1.8953 to 0.4027 (stationary_0) and 1.5722 to 0.1983 (stationary_2), measured at each fold's selected epoch. Final all-source weighted action error falls from 1.7337 to 0.1666 in 18 epochs / 288 gradient updates. These are supervised action errors, not success probabilities.

Autonomous BC from the three unique demonstrated seeds achieves 0/3 qualified: all three reach forbidden-airframe first touch at 1.3490, 1.3620 and 1.3656s. Frozen Fixed A also qualifies 0/3, exiting the flight volume at 0.450,0.455 and0.440s. BC therefore changes the maneuver substantially but has not completed qualified contact even at the demonstrated starts. The optimized reference traces succeed only when their original prefix commands execute; those outcomes remain separate.

![Demonstrated-start autonomous traces](../runs/asm2-terminal/r8/bc-demonstrated-starts.png)

## Matched PPO and native results

| Additional transitions | A earlier /36 | B earlier /36 | A robust /12 | B robust /12 | A braking /36 | B braking /36 | A terminal /30 | B terminal /30 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 | 0 | 35 | 33 | 30 | 30 |
| 98,304 | 0 | 0 | 0 | 0 | 34 | 29 | 30 | 30 |
| 196,608 | 1 | 0 | 0 | 0 | 26 | 19 | 30 | 30 |
| 294,912 | 0 | 3 | 0 | 1 | 21 | 23 | 30 | 30 |
| 393,216 | 0 | 0 | 0 | 0 | 23 | 23 | 30 | 30 |

Zero-transition B is the separately saved BC Actor. Both arms use 384 PPO updates and 393,216 transitions. Each restart is charged; exact counts and finite rollout tensors are checked in `checkpoint-verification.json`.

Selected checkpoints: A `checkpoint-098304.pt`, B `checkpoint-393216.pt`. Selection uses only the frozen development rule. Final endpoints are also retained below.

| Autonomous policy | Demonstrated starts qualified /3 | Failure reasons |
|---|---:|---|
| fixed_a | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| BC | 0 | {'FORBIDDEN_AIRFRAME_FIRST_TOUCH': 3} |
| A-98304 | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| B-98304 | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| A-196608 | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| B-196608 | 0 | {'SPEED_LIMIT': 3} |
| A-294912 | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| B-294912 | 0 | {'FORBIDDEN_AIRFRAME_FIRST_TOUCH': 2, 'NO_SIGNED_TOUCH': 1} |
| A-393216 | 0 | {'FLIGHT_VOLUME_EXIT': 3} |
| B-393216 | 0 | {'NO_SIGNED_TOUCH': 2, 'FLIGHT_VOLUME_EXIT': 1} |

Demonstrated starts cover three saved seeds, only two original source families. These counts never enter the new-anchor selection or Go gate.

| Fresh cohort | Fixed A | BC | Selected A | Selected B | Final A | Final B |
|---|---:|---:|---:|---:|---:|---:|
| earlier_fresh | 0/36 | 0/36 | 0/36 | 0/36 | 0/36 | 0/36 |
| rotation | 1/18 | 1/18 | 1/18 | 1/18 | 0/18 | 1/18 |
| braking | 16/18 | 16/18 | 16/18 | 11/18 | 11/18 | 11/18 |
| terminal | 18/18 | 18/18 | 18/18 | 18/18 | 18/18 | 18/18 |
| reached_handover | 0/4 | 0/4 | 0/4 | 0/4 | 0/4 | 0/4 |

Fresh earlier tests perturb development-source anchors and do not establish unseen-source generalization. Rotation, braking, terminal and reached-handover use distinct historical source identities excluded from demonstrations and training. All actor hashes were sealed before these outcomes were opened.

| Endpoint policy | Earlier envelope exits /36 | Earlier failures |
|---|---:|---|
| A | 24 | {'FLIGHT_VOLUME_EXIT': 24, 'NO_SIGNED_TOUCH': 9, 'CONTACT_POINT_SPEED': 3} |
| B | 0 | {'NO_SIGNED_TOUCH': 33, 'CONTACT_POINT_SPEED': 3} |

![Learning and native comparisons](../runs/asm2-terminal/r8/learning-curves.png)

The archived native evaluators and replay generators execute only each candidate Actor. No demonstration prefix, clock-indexed command or Fixed A switch is used. Intermediate descriptor entry is recorded but is not labeled membership of a verified success region or completed skill bridging.

## Decision and next action

**No-Go.** Keep Phase 4 as the verified learned baseline and Fixed A as the continuation reference. Neither B nor BC is promoted. A is development-selected at 98,304; no B PPO checkpoint passes the retention gate, so B 393,216 is the prespecified analysis fallback, not an acceptable deployment candidate. Matched-budget results are reported at every common checkpoint above; the selected A/B comparison is explicitly not a matched-dose comparison.

**CONFIRMED:** B's transient 3/36 at 294,912 is one robust 3/3 anchor, `dev_lateral_positive:730`, already recentering at reset and historically favorable in earlier experiments. A's single midpoint success was the positive variant of that same anchor. B gains no second anchor, loses all three qualifications at the final checkpoint and retains only 23/36 braking. Final B replaces envelope exits with 33/36 no-touch timeouts and 3/36 contact-speed rejections; lower exit count does not complete the maneuver. Fresh earlier remains 0/36 for every evaluated Actor. The one held-out rotation qualification is the same `held_diagonal:813` state already passed by Fixed A and BC, not a new held-out capability.

**CONFIRMED:** BC's original three demonstrated-start contacts are all forbidden. At B 98,304 all three instead exit the flight volume; later checkpoints never qualify them. Thus plain PPO does not preserve even the demonstrated local contact-reaching behavior consistently. This is not destruction of a previously qualified BC skill—BC never had one. Two A and three B earlier training successes remain training-only outcomes. All final terminal scores remain 30/30 development and 18/18 fresh; braking, rather than terminal-only contact, degrades substantially.

**INFERENCE / limits:** action supervision can change the earlier maneuver, but low source-excluded action error does not establish closed-loop terminal precision. The strongest observed bottleneck is maintaining an accurate end-to-end rotation/recentering-to-contact behavior during ordinary on-policy updates. Sparse two-family demonstrations and off-demonstration state drift remain plausible contributors; this single-seed comparison does not isolate them, prove an observation limitation or establish a globally optimal training method. It supplies no certified reachable region or official impact-score claim.

**Exactly one next experiment, NOT LAUNCHED:** a bounded demonstration-regularized PPO comparison from the saved BC checkpoint, retaining a single preregistered, decaying action-likelihood term from the same existing demonstrations/retention traces during fine-tuning. Keep the original reward and native benchmark, and require new multi-anchor qualification plus braking retention. This tests whether retaining the demonstrated transition actions allows PPO to refine contact without losing recentering. It would explicitly change the next experiment's learning objective; no such term was used here. No new optimization, demonstration campaign or automatic training extension is launched.

## Verification, cost and reproducibility

Three focused dataset/interface tests passed. Actual checkpoint verification confirms parent/BC Actor initialization, identical inherited critic/normalizer/Adam states, unchanged Gaussian log standard deviation at initialization, one charged restart per arm and 384 finite saved PPO rollouts per arm. Portable Actor exports match Torch inference. The offline replay check passes 43 archives and 9,293 commands, each exactly reproducing its own Actor. One previously scored transient successful episode was reproduced with an identical native outcome for its saved replay; it is not an additional success sample. Frozen source pins and all previously recorded R7 artifact hashes pass; Phase 4/FixedA digests are unchanged. Viewer data loaders pass; no live 3D rendering or hardware test is claimed.

Measured learning time is 4,871.66s for A and 4,865.21s for B; development evaluation pauses are 363.73s and 448.08s. Both stop at `TRANSITION_LIMIT`, with no abandoned updates or budget extension. The scheduled development comparisons contain 1,380 evaluations; fresh evaluation contains 470 across five distinct Actor selections (B final aliases selected B); demonstrated-start evaluation contains 30. Replay reproductions are separate from those denominators. No numerical instability occurred. Native braking deteriorated, but terminal competence remained intact; the authorized full budget was completed rather than applying the earlier 98,304-transition futility gate.

[Configuration](../../configs/rm27_asm2_r8.json) · [Dataset/BC implementation](../../src/rm27_asm2/demonstration_learning.py) · [PPO adapter](../../src/rm27_asm2/r8_experiment.py) · [Native report implementation](../../src/rm27_asm2/r8_report.py) · [Tests](../../tests/test_asm2_demonstration_learning.py).

Run artifacts: [frozen inputs](../runs/asm2-terminal/r8/freeze.json), [dataset provenance](../runs/asm2-terminal/r8/demonstrations.json), [BC checkpoint](../runs/asm2-terminal/r8/bc.pt), [BC validation](../runs/asm2-terminal/r8/bc-results.json), [development selection](../runs/asm2-terminal/r8/final-selection.json), [fresh native results](../runs/asm2-terminal/r8/heldout-results.json), [checkpoint verification](../runs/asm2-terminal/r8/checkpoint-verification.json), [replay/provenance verification](../runs/asm2-terminal/r8/evidence-verification.json).

Representative autonomous replays: [BC contact rejection](../runs/asm2-terminal/r8/demonstrated-starts/BC-stationary_0-slope_minus.npz), [transient B qualification](../runs/asm2-terminal/r8/transient-qualified.npz), [final B demonstrated-start timeout](../runs/asm2-terminal/r8/demonstrated-starts/B-393216-stationary_0-slope_minus.npz). All checkpoints and unsuccessful outcomes remain available.

![Original PPO learning metrics](../runs/asm2-terminal/r8/optimization-curves.png)

Reproduction uses the existing native environment with `PYTHONPATH=src:. /tmp/rm27-g311-env/bin/python`. Saved evaluation commands `-m rm27_asm2.r8_report fresh` and `demo` reuse completed outcomes; `plots` regenerates the figures. `r8/verify_evidence.py` checks saved data without integration. Torch checkpoint verification uses the frozen Isaac image and `r8/verify_checkpoints.py`. Do not rerun `demonstration_learning build` or `fit`, or restart completed PPO; all allocated results are preserved. No commits, pushes, remote modifications, physics/geometry/scorer changes or hardware work occurred.
