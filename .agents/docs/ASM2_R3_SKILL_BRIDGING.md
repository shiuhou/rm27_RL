# R3 — autonomous skill bridging

Completed 2026-11-13 Asia/Shanghai. **R3 skill bridging: No-Go; no baseline promotion.** Both arms used exactly 196,608 transitions, 393,216 total. Larger exploration produced rare successful training episodes and fewer native envelope exits, but no final qualified earlier-state contacts or new held-out capability. Phase 4 remains the verified baseline; Fixed A remains the continuation reference. This authorization is closed. G3.1.2 establishes loaded forbidden follow-through in two held-target tails, but no material defect in the frozen first-contact contract. Its proposed non-RL follow-up is not executed here.

## Bottleneck and one intervention

**CONFIRMED:** regional-update A collected 55,352 earlier-state transitions and 300 completed episodes with zero qualified training outcomes. B's regional loss weighting also produced zero earlier training qualifications. Native outcomes remain concentrated on the already-recentering anchor. These completed studies are not repeated.

A small read-only analysis of saved A rollouts excludes the first update's subsequent restart and right-censored returns. On 53,956 earlier-state samples, raw GAE std is 0.0557 versus braking 0.3198; value-minus-realized-discounted-return bias is +0.0149, RMSE 0.1516, explained variance 0.6596. Braking value RMSE is 0.6823 and late-rotation bias +0.4462. These realized returns come from changing behavior policies; they are not independent true-value targets. Good agreement with bootstrapped targets alone would be circular evidence. Broad critic failure is not established as the primary earlier-state bottleneck.

**HYPOTHESIS:** independent 5 ms exploration around the learned maneuver is too narrow after actuator filtering to discover useful intermediate behavior. Fixed A's raw Gaussian std is 0.156–0.167. A linear first-order 40 ms filter attenuates independent sample std by sqrt((1-exp(-dt/tau))/(1+exp(-dt/tau))) = 0.2498. At existing earlier training states, offline tanh sampling gives filtered command std about 0.0355–0.0386. This is a filter approximation, not native dynamic feasibility evidence.

**One modification:** fresh B initializes all four `actor.log_std` entries to log(0.64); original PPO then optimizes them freely. This approximately compensates the linear filter attenuation of the original 0.16 scale. Nonlinearity gives about 3.2×, rather than 4×, filtered command variation; predicted tanh saturation fractions remain 0.4–1.4%. A retains its exact source actor. Mean networks, critic, optimizers, 45 actor inputs, rewards, gamma/GAE, reset mixture, 5 ms Direct-CTBR interface, plant and scorer stay matched. No entropy schedule, noise correlation, new controller, reference path, teacher action or phase timing is introduced. Sampling densities remain those of the actual Gaussian used by PPO.

Increasing Gaussian std also changes the expected tanh-transformed action and outcome variance. The comparison tests this complete exploration-scale intervention; it cannot uniquely attribute a result to motor filtering. The intervention tests exploration availability, not whether arbitrary larger noise guarantees dynamically viable states. Action/state occupancy and qualified outcomes must determine its value. The unchanged sparse feedback, observation aliasing and control restrictions remain possible limitations; no completed diagnostic is repeated.

The alternatives remain bounded hypotheses. Credit assignment is difficult over the early-to-contact horizon, but the completed temporal-credit experiment did not establish a remedy. Late-rotation critic bias exists; the saved earlier-state returns give weaker evidence for critic bias as the primary bottleneck. Previous capped-controller certificates establish feasibility at the reset states within the frozen action interface, without proving that the actor can learn it. Actuator-memory observability remains inconclusive. These findings motivated one exploration test, not a claim that other mechanisms were disproved.

## Frozen protocol

[Configuration](../../configs/rm27_asm2_r3.json), [implementation](../../src/rm27_asm2/r3_experiment.py), [offline diagnosis](../runs/asm2-terminal/r3/diagnosis.json), [manifest](../runs/asm2-terminal/r3/freeze.json).

- Shared initializer: Fixed A `resumable-comparison/A/checkpoint-196608.pt`, SHA `eba759e7eeea93cd9fe06e0f8653d8f12bafc6cfd05a64bcbc11a7a78086fa3f`. Preserve Phase 4 checkpoint-186368.pt.
- Both arms: fixed reset probabilities terminal 0.10, braking 0.45, late rotation 0.20, earlier bridge 0.25; identical certified reset archive and seed 42722. Realized episode/reset counts can diverge with policy behavior.
- Budget: maximum 196,608 additional transitions per arm, matched assessment at 98,304. Both execute a charged one-update resume check, then stop at the intermediate checkpoint. Resumption uses valid fresh resets, not bitwise PhysX solver-cache restoration.
- Intermediate futility: stop both only if B has zero earlier training qualifications, no native new-entry or qualification gain over A, and no newly qualified anchor. Technical failures also permit stopping. No automatic extension or method sweep.
- Primary comparison and held-out actor: last common positive endpoint, no best-checkpoint search. Seal actor hashes before held-out outcomes are opened.
- Scientific progress requires gains across at least two distinct earlier anchors, at least one new robust 3/3 anchor, braking >=32/36 and terminal >=27/30 without retention envelope exits, corroborated by fresh cases. Reaching a descriptor without contact is insufficient; descriptors are not viability certificates.
- Fresh cases: 36 independent position/velocity perturbations on the 12 reserved earlier anchors; 18 each rotation, braking and terminal from three historically examined held-out sources; four reached handovers. 94 total. Position ±0.5 mm and velocity ±5 mm/s along the existing target tangent, independently seeded. This is the existing numerical sensitivity range, not manufacturing uncertainty. The 36 earlier cases share selection source trajectories; the other sources are disjoint from training/selection. No claim of wholly novel trajectory sources.

Fresh starts undergo only native restore/clearance and envelope validation; no new feasibility-controller campaign. Actual learned evaluations use frozen detailed MuJoCo and the unchanged native scorer. First-contact qualification remains separate from post-contact stability, official impact scoring and hardware capability. Existing 5060 results remain separate; no remote work is needed.

## Verification and execution

Two real-PyTorch tests passed in the existing immutable Isaac image: fresh B changes only Gaussian scale with correct log density; A and resume preserve actor tensors. Existing learner, collector and evaluator are reused unchanged. The frozen manifest validates all 94 fresh resets and pins 42 source/reference files. All 17 saved A initial actor tensors exactly match Fixed A. Both training arms and the sealed final native evaluation are complete. See the final results and verification below.

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_experiment freeze
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_experiment run --stage mid
# Only after the preregistered intermediate assessment permits continuation:
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_experiment run --stage full
```

## Intermediate result — 98,304 transitions per arm

CONFIRMED native A/B: earlier contacts 0/36 versus 3/36; robust anchors 0/12 versus 1/12; envelope exits 33/36 versus 0/36; new sampled descriptor entries 3/36 versus 12/36. B's three qualifications all come from the historically favorable `dev_lateral_positive:730` recentering anchor. Its remaining 33 cases end without signed touch. Braking is 34/36 versus 30/36; terminal 30/30 each. Thus B trades envelope failures for non-completion and fails the distinct-anchor and braking-retention requirements. **Scientific Go remains false.**

Earlier Isaac training outcomes are A 0/300 and B 2/213 completed episodes. B first qualified in updates ending 55,296 and 62,464 transitions, at distinct training anchors `stationary_0:243` and `stationary_2:224`. These are stochastic training events, not native generalization results. Actual initial checkpoints verify that B changes only log_std, and A's 98,304-transition actor matches the historical uniform-control actor bit for bit.

The frozen futility rule is not met: B has successful earlier training episodes, more native descriptor entries, more qualifications and a newly qualified anchor relative to the shared initializer. That anchor was already favorable in historical experiments, so it is not a new research frontier. Both arms continue to the originally authorized 196,608 transitions; no new method or budget is added. Held-out outcomes remain unopened.

## Final matched native outcome — 196,608 transitions per arm

| Metric | A: unchanged PPO | B: larger exploration |
| --- | ---: | ---: |
| Earlier qualified contacts | 0/36 | 0/36 |
| Robust earlier anchors, 3/3 variants | 0/12 | 0/12 |
| Earlier envelope exits | 33/36 | 0/36 |
| New sampled braking-descriptor entries | 3/36 | 6/36 |
| Entered descriptor and qualified | 0 | 0 |
| Braking qualified | 35/36 | 35/36 |
| Terminal qualified | 30/30 | 30/30 |

A's other three earlier failures are two forbidden first touches and one contact outside the registered footprint. B has 33 `NO_SIGNED_TOUCH` timeouts and three `OUTSIDE_REGISTERED_FOOTPRINT` failures, all at the familiar `dev_lateral_positive:730` anchor. No new robust anchor emerges. B's transient midpoint 3/36 qualification and 1/12 robust-anchor result disappears at the final endpoint. Both final policies meet the braking/terminal retention floors, but **neither demonstrates autonomous skill bridging**.

The descriptor records target-frame position, inward speed and alignment at native control boundaries. It is **not a dynamically validated viability set**: it does not certify future contact for the complete attitude/rate/actuator state. More descriptor entries cannot substitute for qualified completion. Zero exits applies only to this 2.3 s earlier-state suite; it is not global stabilization or recovery evidence.

![Learning and native evidence](../runs/asm2-terminal/r3/learning-evidence.png)

## Fresh held-out comparison

Both final checkpoint hashes were sealed in [final-selection.json](../runs/asm2-terminal/r3/final-selection.json) before any current held-out outcomes were opened. R3 uses the last common endpoint; the reused workflow's legacy `result.json.selected` field is not the R3 selection authority. There was no best-checkpoint search or reselection after seeing fresh results.

| Fresh population | A | B | Frozen Fixed A |
| --- | ---: | ---: | ---: |
| Earlier-anchor perturbations | 0/36 | 0/36 | 0/36 |
| Robust earlier anchors | 0/12 | 0/12 | 0/12 |
| Separate-source rotations | 1/18 | 1/18 | 1/18 |
| Braking | 15/18 | 16/18 | 16/18 |
| Terminal | 18/18 | 18/18 | 18/18 |
| Reached handovers | 0/4 | 0/4 | 0/4 |

The single rotation qualification is the same `held_diagonal:813` state for all three actors. It establishes no new capability. Fresh earlier A/Fixed A failures are 33 envelope exits and three forbidden contacts each. B has 33 no-touch timeouts, one forbidden contact and two footprint misses; its six descriptor entries produce zero qualifications. Separate-source rotation volume exits fall from A's 12 to B's six, with six B timeouts; every reached-handover trial still fails a flight limit. Fresh braking B matches Fixed A, and all terminal cases qualify.

These are **94 cases per actor, 282 scored fresh episodes**, reported by population rather than pooled. The 36 earlier cases share historical development anchors; the remaining cases use historical source trajectories disjoint from current training and selection. Offsets are fresh and were not outcome-selected, but source-level novelty is limited. Variants and frames from the same trajectory are correlated; no independent-sample statistical significance is claimed.

![Fresh qualification](../runs/asm2-terminal/r3/heldout-qualification.png)

## What changed, and why bridging remains unresolved

**CONFIRMED exploration effect:** A collected 111,002 earlier-state transitions (56.46% of its budget), with 0/576 completed training qualifications. B collected 124,902 (63.53%), with 3/400. B's successes were `stationary_0:243` (1/16), `stationary_2:224` (1/20) and `stationary_2:220` (1/11): three nearby anchors across two stationary training sources. These are stochastic Isaac events, not independently rescored native trajectories. The archived rollout data does not contain full training state/actuator histories, so no physical replay of these three training successes is claimed.

B's earlier raw advantage standard deviation was 0.1146 versus A's 0.04794. Its value-minus-realized-return bias was +0.1461 versus A's +0.01191. These are descriptive estimates under changing training policies, not true-value or causal-error measurements. The larger noise exposed successful behavior, but successful feedback remained rare and was not consolidated into a policy that reliably finishes the maneuver. Same reset probabilities did not imply the same realized state distribution: B's longer episodes consumed more of the transition budget, leaving 32,683 braking transitions versus A's 40,561. That is a consequence of the intervention, not an additional sampler change.

**CONFIRMED credit pattern, limited inference:** the three successful episodes lasted 0.960, 0.890 and 1.005 s. Actual normalized GAE averaged as follows:

| Successful update / cumulative transitions | First half of episode | Last 100 ms |
| --- | ---: | ---: |
| 54 / 55,296 | -0.109 | +2.426 |
| 61 / 62,464 | +0.200 | +1.880 |
| 187 / 191,488 | +0.253 | +0.369 |

The first two have much stronger near-contact credit; the third has more distributed positive credit. Thus it would be wrong to claim that every successful prefix received negative credit, or that this proves a unique temporal-credit defect. The evidence supports a remaining problem with sparse successful feedback and coordinated completion. It does not justify repeating the failed GAE-lambda study or declaring critic/observation limits resolved. [Reproducible credit arrays](../runs/asm2-terminal/r3/successful-episode-credit-196608.json).

**CONFIRMED autonomous motion:** the matched fresh `dev_far:678` replays start at the same full saved state. A exits at 0.885 s, with target-tangent position -0.803 m. B begins recentering at 0.230 s versus A's 0.320 s and reaches a minimum absolute tangent offset of 0.191 m, but finishes at 2.3 s with target-frame COM coordinates approximately [-0.280, +0.240, +0.205] m and across-target velocity +0.759 m/s. It remains misregistered and never touches. This is improved partial maneuvering, not hover stabilization, completed braking, or qualified contact. Neither replay uses teacher actions, references or phase switches.

![Matched saved motion](../runs/asm2-terminal/r3/representative-trajectories.png)

**INFERENCE:** simply widening independent Gaussian exploration is insufficient. It changes failure from early departure to incomplete interception on this suite, while the learned policy still lacks a reliable connection into complete braking/contact. A goal-directed intermediate learning signal grounded in full-state dynamic viability is now a stronger candidate than another increase in unstructured noise. The experiment does not isolate actuator filtering, value approximation, delayed credit or partial observability as a unique root cause.

## Reproducibility, cost and preserved evidence

A/B used 2,439.785 / 2,413.987 learning seconds (4,853.771 s total), 192 updates each, and exactly 393,216 combined transitions. Both stopped at `TRANSITION_LIMIT`; no charged update remains pending. Development-evaluation pauses were 204.421 / 239.735 s. Both performed the charged first-update restart and the preregistered midpoint restart. These resume from fresh valid episode states; no bitwise PhysX active-episode continuation is claimed.

[Verification](../runs/asm2-terminal/r3/verification.json) confirms all 384 saved rollout/update pairs are finite, exact transition counts, unchanged 42 pinned source/reference files, and unchanged Phase 4 / Fixed A hashes. Actual saved initial critics, normalizers and both optimizer states match the shared source. A's initial actor is exact; B's mean tensors are exact and only its Gaussian scale differs. The exploration state at each final-half resume exactly matches the preceding midpoint checkpoint, so B was not reset to 0.64 again. The two pre-freeze real-PyTorch tests passed; saved midpoint analysis was independently recomputed without rewriting its result.

The native evaluator saved **12 representative replays** and independently reproduced each selected episode's verdict, end time and contact records before writing it. They are recorded-state replay archives compatible with the existing viewer; a new Isaac viewer rendering was not run. All failed actors, checkpoints, rollout arrays and midpoint results remain preserved. The experimental adapter remains isolated from the frozen learner and is retained for reproducing this negative result; it is not promoted into the baseline workflow.

- [Final assessment](../runs/asm2-terminal/r3/endpoint-assessment.json) and [fresh results](../runs/asm2-terminal/r3/heldout-results.json).
- [Final training statistics](../runs/asm2-terminal/r3/learning-summary-196608.json) and [checkpoint verification](../runs/asm2-terminal/r3/checkpoint-verification.json).
- [A endpoint](../runs/asm2-terminal/r3/A/checkpoint-196608.pt), [B endpoint](../runs/asm2-terminal/r3/B/checkpoint-196608.pt), [replays](../runs/asm2-terminal/r3/heldout-replays/).
- [Analysis/figures implementation](../../src/rm27_asm2/r3_report.py); PNG and PDF figures are saved beside the results.

```bash
# Analysis only; both training stages are already complete. Do not relaunch training.
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_report assess --endpoint 196608
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_report final --endpoint 196608  # validates and reuses saved results
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r3_report plots --endpoint 196608
```

Frozen physics, scorer, collision geometry, CTBR interface and actor observations were unchanged. No reference trajectory, teacher action, predefined reversal, controller development, G3.1.2 rerun, 5060 repeat, hardware work, commit or push occurred. First-contact engineering qualification does not establish safe follow-through, official impact qualification or hardware performance.

## Exactly one recommended next experiment — not launched

**One matched potential-based shaping experiment toward a native-validated braking-entry set.** Define the set by successful frozen Fixed A continuations from complete physical states, including translational velocity, orientation, body rates, geometric clearance, realized rotor thrust and held actuator targets. The current marginal descriptor box is not sufficient. Freeze a bounded potential over validated support, use the original gamma and correct terminal/truncation handling for the shaping difference, and retain autonomous actor actions, the original success criterion, matched resets and held-out gates. No trajectory tracking, teacher actions or scripted switch should enter the actor.

Why this experiment: R3 established that larger noise can expose occasional early-to-contact behavior and improve recentering, but did not make the successful connection persistent or generalizable. A full-state validated potential tests whether useful intermediate feedback can connect those partial maneuvers to the already-learned braking skill. It directly distinguishes this from the failed geometric margin reward; it must not reward proximity to an unvalidated box. Approximate implementation cost is 1–2 engineer-days for bounded native set validation, potential construction and invariance checks using the existing plant/evaluator, followed by one separately authorized matched training budget. Expected value is a causal test of missing intermediate guidance, not another noise sweep. **Recommendation only; no set construction, shaping implementation or follow-up experiment has been started.**
