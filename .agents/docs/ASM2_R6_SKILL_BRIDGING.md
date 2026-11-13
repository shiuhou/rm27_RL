# ASM2 R6 — Literature-grounded autonomous skill bridging

**COMPLETE — skill-bridging No-Go; no extension or baseline promotion. Local development gains are confirmed below.** First-contact qualification remains the benchmark. G3.1.2 post-contact body loading does not redefine it. Phase 4 is the protected baseline; Fixed A is the continuation reference. No R5.2 evidence was found: its three-case amplitude study remains **NOT_RUN**, not a dependency.

## Evidence and causal hypothesis

Phase R and Fixed A establish braking/terminal competence, but early rotation does not reliably reach qualified contact. R2's fixed earlier-state bank and adaptive reset weighting, regional PPO balancing, and temporal/reward changes failed to establish bridging. R3's increased exploration reduced final early envelope exits from 33/36 to 0/36, but both final actors had 0/36 earlier contacts. Its midpoint successes at one favorable anchor disappeared. R4's full-state similarity potential recognized successful states but declined along successful approaches and was rejected before training; it remains rejected.

R5/R5.1 add **30 full autonomous trajectories, 25 qualified and five failed**, all from five development source families. They are recentering/braking examples, not outward-rotation solutions. The near-slow sequence 702 pass → 706 fail → 707 pass disproves a simple temporal success threshold. The across-negative dev_fast:816 failure retains the frozen scorer's forbidden-manifold rejection even though the body point was unloaded. Continuous intersection, loaded contact, native first-contact qualification and official competition scoring remain distinct.

**HYPOTHESIS:** PPO lacks a repeatable succession of trainable predecessor starts leading into its learned skill. Practice should expand only after actual current-policy terminal successes, with successful suffix replay to preserve the connection. This is a testable mechanism, not an established diagnosis of PPO or proof of reachable predecessor states.

## Actual literature and source inspection

Downloaded papers, text, retrieval metadata and pinned source excerpts are retained in `r6/literature/` under the run directory. No external code was imported into the runtime.

| Work and verified primary source | Actual method/evidence | Transfer to ASM2 and limitations |
| --- | --- | --- |
| Wu et al., *Precise Aggressive Aerial Maneuvers with Sensorimotor Policies*, [DOI](https://doi.org/10.1126/scirobotics.aeb0180), [author preprint](https://arxiv.org/abs/2604.05828) | Read Informed Reset, supplement S7 and Algorithm S1. Offline differentiable flatness/SE(3) trajectory optimization supplies position, velocity and attitude resets; probability 0.5 vs random takeoff/task starts. Policy generates actions. Real gap-traversal and HIL evidence, including >100 real-world trials, addresses gap traversal rather than contact. | Reset information without action imitation is directly relevant. ASM2 already used feasible witness resets: copying the 0.5 mixture would repeat prior work. Paper's reset description does not establish ASM2-equivalent rotor/motor-memory restoration. Publisher returned 403; inspected author preprint with supplements, not publisher PDF. Data availability cites Zenodo 18005929; its inspected API record lists `data_plots.zip` (48,358,476 bytes), CC-BY-4.0, described as recorded experiment data and plotting scripts. It was not downloaded as a training-code release. GCOPTER is a cited planner, not verified release of the complete learner. |
| Florensa et al., [Reverse Curriculum Generation, CoRL 2017](https://arxiv.org/abs/1707.05300) | Read assumptions, algorithms and source. Brownian **forward action rollouts** generate nearby starts; intermediate success probability selects starts; old starts are replayed. Requires reset access and a communicating/reversible-enough system. TRPO experiments in simulated navigation/manipulation. | Competence-gated expansion and replay transfer. Brownian motion from a terminating contact goal does not provide valid backward quadrotor states. R6 uses full-state archived predecessors as proposals and online evidence, not reverse dynamics or a monotonic boundary. |
| Florensa et al., [Automatic Goal Generation, ICML 2018](https://proceedings.mlr.press/v80/florensa18a.html) | Read paper, supplemental A/B and goal-GAN source. Goal-conditioned policy, intermediate difficulty bounds 0.1/0.9, modified least-squares GAN; supplement uses two-thirds new goals, one-third replay and distance-separated insertion. Simulated ant/maze/point-mass evidence. | Replay and competence labels are useful principles. A dynamic intermediate goal contract and goal-conditioned actor are missing for ASM2; coordinate arrival is not qualified contact. No GAN, arbitrary subgoal or goal-interface change is introduced. |
| [PerchRL, arXiv 2606.03441](https://arxiv.org/abs/2606.03441), inspected v3 | State PPO then vision fine-tuning; temporal platform observation encoder; asymmetric critic with 15 future platform observations; CTBR. Simulation and hardware: state-policy Crazyflie/static 90° surface, vision/moving platform with different attachment/contact hardware. Text says code “will be released”; no corresponding repository verified. | State pretraining and CTBR are relevant, already present here. Moving-platform memory/future-state privilege is not evidence that fixed-target ASM2 needs that observation change. Do not transfer perching thresholds, vision, magnetic/Velcro assumptions or deployment claims. |

Reverse Curriculum and Goal GAN official project pages link [rllab-curriculum](https://github.com/florensacc/rllab-curriculum), inspected at **f55b50224fcf5a9a5c064542eb0850a966cab223**. Its LICENSE is MIT, copyright rllab contributors. Read README, maze Brownian and goal-GAN algorithms, state generator/evaluator. Source confirms environment-generated starts, TRPO rollout success labels, replay and GAN labels. It depends on the older rllab stack; only algorithmic ideas are adapted with independent small code, no dependency port. License and excerpt SHA-256 values are in `retrieval-pinned.json`. Lack of verified code availability for the other two papers is a provenance limitation, not a claim that no code exists.

## Candidate comparison and selection

| Candidate | Difference from earlier failures | Preservation/interface and decision |
| --- | --- | --- |
| Informed resets from autonomous successful transitions | Supplies observed positive transition suffixes rather than increasing exposure to fixed unsuccessful early resets. Avoids R4 extrapolated similarity rewards. Alone it may only rehearse existing skill. | Complete-state resets are privileged only at training initialization; unchanged 45-D actor and CTBR. Used as seeds/replay within the selected curriculum. |
| **Competence-gated predecessor reset expansion** | Changes the support of the reset distribution based on actual terminal success, rather than R2's weights over a static bank. R3's isolated success must repeat before expansion; old skills receive replay. No potential or extra reward. | **Selected primary method.** A per-source expansion rule, not an action phase switch. Full state and actuator history restored; earlier candidates remain unvalidated until evaluated. |
| Goal-conditioned/intermediate-goal learning | Could learn reusable transitions, but goal arrival can reproduce descriptor-only gains. Needs a dynamic goal contract, new input/termination semantics and additional data. | Not selected; would confound the fixed-interface experiment and does not currently solve contact qualification. |
| Dynamics-aware value/model learning | Could propagate downstream returns and account for actuator state. Current complete data cover only late recentering; early model/value targets would extrapolate beyond successful support, with contact discontinuities. Existing R3 does not establish critic failure as the primary cause. | Not selected. Privileged training critic is possible in principle, but a justified model/target and independent early calibration are missing. Learned planning or hierarchical termination contracts would add greater scope and failure modes. |

No candidate is claimed guaranteed to outperform R2. The controlled test isolates whether competence-gated support expansion supplies useful positive learning experience and transfers to distinct **earlier** native anchors. Braking retention or increased training success alone cannot pass.

## Dataset and implementation

`dataset.json` is a hash-addressed index into all 30 complete `physics.npz`, `contacts.json`, `outcome.json` and native replay files, preserving source, reset frame, perturbation and result. Full physical vectors, actual motor thrust, held targets and CTBR commands remain at 0.1 ms/5 ms original resolution. Five source clusters, not 89,724 independent transitions, define the independent provenance units. Native rewards were **not recorded**; historical R3 reward/action/advantage/value archives lack complete physical histories and are explicitly kept separate. No invented observed rewards or action supervision.

R6 constructs 159 successful suffix reset rows, sampled every 40 ms with at least approximately 40 ms left before the first-contact decision, plus 85 predecessor proposals (17 per source). The latter begin at R5's five nominal successful anchors and move back eight 5 ms source frames at a time, at most 640 ms. Forty milliseconds is the provisional plant motor time constant, used as an interpretable proposal scale, **not** a proven feasible step or measured physical uncertainty. Original source commands are not loaded into the training target. There is no reversed physics integration. Every new reset passes exact native pose/velocity/actuator restoration, collision/loaded-contact absence, finite dynamics, envelope and rotor limits. These checks establish valid starts, not qualified futures.

B retains the original terminal/braking/late masses 0.10/0.45/0.20. The remaining 0.25 is split equally between current predecessor practice and successful-suffix replay, uniformly across the five sources, then within each source's support. At least four successes among the latest eight completed episodes at a current source row unlock its next predecessor. Failure never unlocks a row. This operational training criterion is not a robustness confidence bound; feedback is training-backend first-contact success, independent MuJoCo remains authoritative. Proposal order does not imply that all intervening states succeed. The curriculum can stall at a nonmonotonic failure; this is a known limitation, not permission to skip failed rows after inspecting outcomes.

The sampler persists row histories, frontier, promotions and RNG in the existing checkpoint. A restart test exposed shallow history aliasing and was fixed before freeze. Three focused tests check gating, mass/retention and exact resumed sampling. Training records additionally preserve control-boundary full state/motor/action/reward/outcome arrays, including terminal states before automatic reset, alongside existing regional PPO arrays. This closes the historical reward/physical-history gap for the new comparison.

A uses the unchanged original Fixed A continuation bank and sampler. Both arms preserve actor/critic architecture, source optimizers, initial Gaussian scale, gamma/GAE, reward, horizons, plant and scorer. No extra reward magnitude or timeout rule supplies the intervention. Shorter realized episodes are an expected reset-distribution consequence and will be reported; they are not evidence of bridging. Existing positive trajectories justify an experiment, not a success claim.

## Frozen experimental decision

Initial budget **98,304 per arm**, matched seed, original Fixed A checkpoint. One charged update/restart verifies resumability in both arms. Use last common endpoint; no best-checkpoint selection. Fresh valid offsets have a new seed and are generated before training; endpoint actor hashes are sealed before opening their outcomes. Shared-source fresh perturbations and disjoint-source cases must be reported separately.

Extend only if B gains contacts over A at **at least two distinct earlier anchors**, gains **at least one robust 3/3 anchor**, retains braking **≥32/36**, terminal **≥27/30**, and no retention envelope exits. An extension, if warranted, is limited to the user's maximum 196,608 per arm; no automatic method sweep. Final Go additionally requires corroboration on fresh evaluation and persistence at the final endpoint. Single-anchor contact, descriptor entry, reduced exits, curriculum promotions or training-only success are insufficient.

Both arms completed the bounded comparison; the extension gate failed. Its first 1,024 transitions exactly match the historical unchanged control in raw actions, rewards, done flags, raw GAE and critic values. The physical capture matches PPO reward/action/done arrays and continuing state/actuator transitions.

Saved-path annotation with the existing frozen descriptor finds 24/30 initially inside and **0/30 newly entering** it. This further limits claims about these late trajectories: none provides a newly observed descriptor-entry event. The successful physical suffixes still supply actual autonomous contact outcomes; descriptor membership is not used for reward, competence gating or reachability certification. `transition-annotations.json` records this negative result alongside full terminal actuator state. The dynamics figure distinguishes successful and rejected paths without attributing co-varying velocity, rate and actuator differences to a single cause.

![Transition dataset](../runs/asm2-terminal/r6/transition-dataset.png)

## Matched learning and independent endpoint result

Both arms completed **98,304 transitions / 96 PPO updates**, 196,608 total, with one charged restart each and no abandoned update. Learning wall time A/B was **1,225.65 / 1,250.59 s**; evaluation is excluded. Both initial actors, critics, value normalizers and both optimizer states exactly match the frozen Fixed A parent. Final actor tensors are finite. All 192 control-boundary physical rollout archives agree exactly with PPO action/reward/done arrays, and continuing state/motor transitions are consistent. Checkpoint sampler promotions exactly match reconstruction from recorded episodes.

| Native endpoint metric | A unchanged | B R6 |
| --- | ---: | ---: |
| Earlier qualified contacts | 0/36 | 0/36 |
| Robust earlier anchors | 0/12 | 0/12 |
| Earlier envelope exits | 33/36 | 33/36 |
| Newly entering the old descriptor | 3/36 | 3/36 |
| Descriptor entry followed by qualified contact | 0/36 | 0/36 |
| Braking qualified contacts | 34/36 | 31/36 |
| Terminal qualified contacts | 30/30 | 30/30 |

Neither arm had an earlier timeout. A's three non-envelope failures were two footprint misses and one forbidden first contact. B's were one contact-point-speed rejection and two forbidden first contacts. B misses the frozen braking-retention threshold of 32/36 as well as every earlier-learning criterion. **Do not extend to 196,608 per arm.** No best-checkpoint selection was performed: the stopped 98,304 endpoint is final, so there is no claimed later persistence or withheld better checkpoint.

| Fresh evaluation, kept separate | Fixed A | A | B |
| --- | ---: | ---: | ---: |
| Earlier perturbations on reserved development anchors | 0/36 | 0/36 | 0/36 |
| Rotation from sources disjoint from training | 1/18 | 1/18 | 0/18 |
| Braking | 16/18 | 16/18 | 13/18 |
| Terminal | 18/18 | 18/18 | 18/18 |
| Reached handovers | 0/4 | 0/4 | 0/4 |

These are 94 cases per policy, **not a pooled 282-trial success estimate**. Fresh seed is 627113; earlier perturbations share historical anchor sources and are not independent new navigation recipes. Actor hashes were sealed before these outcomes. Fresh earlier exits remain 33/36 for every policy. B's three fresh earlier contacts all reject (two forbidden, one contact-point speed); its fresh braking failures include three forbidden contacts, one speed rejection and one alignment rejection. No official impact scoring or post-contact recovery claim is made.

![Learning and native outcomes](../runs/asm2-terminal/r6/learning-evidence.png)

## What the intervention actually learned

B produced **310/499 positive bridge-band training episodes**, versus A **0/300**. Separate the B denominator: **214/251 successful-suffix episodes** and **96/248 frontier episodes**. Mean completed duration was 0.182 s for suffixes and 0.366 s for frontier starts, versus A's 0.899 s early-bank episodes. Thus higher contact-event density is partly a change in reset difficulty and realized episode duration; the task horizon and reward formula were unchanged. The event count is not a causal estimate of skill acquisition.

The online gates unlocked 200, 80, 80, 40 and 120 ms-earlier proposals for center, fast, near-slow, stationary_0 and stationary_2 respectively. Final proposed frames were 685, 800, 686, 322 and 322. Current-row last-eight feedback is mostly negative: fast and near-slow 0/8, stationary_0 1/8, stationary_2 3/8; center has seven completed failures at its final row. These advances are **proposal exposure**, not native-certified backward extension. Center also had a historical nominal success at frame 708, so some early curriculum movement merely traversed already-known competence.

Replay in this R6 implementation contains the original autonomous successful suffixes; it does **not** automatically archive newly gate-passed predecessor rows. Expansion is one-way along each proposal chain, without periodic native revalidation of old gates. This is a limited adaptation of Reverse Curriculum, not a full reproduction of its evolving old-start replay. Repeated training success therefore did not guarantee deterministic endpoint competence or prevent forgetting. Native/Isaac contact sensitivity, stochastic versus mean actions, and policy drift are not causally separated by this experiment.

The defensible conclusion is narrower than “curricula cannot work”: **this complete-state, success-seeded predecessor curriculum produced repeatable positive training experience, but did not solve earlier-start contact or retain braking sufficiently.** More Gaussian exploration, reset-ratio changes, update weighting or reviving R4 are not justified by these results.

![Curriculum exposure](../runs/asm2-terminal/r6/curriculum-progress.png)

![Representative native replays](../runs/asm2-terminal/r6/representative-3d.png)

## Secondary development checks: real local gains, substantial losses

These checks use the sealed final policies and are **not held-out evidence or continuation criteria**. The support protocol includes all 30 original R5/R5.1 starts, including both available perturbation directions; its case list was fixed before B's first training update. Fixed A outcomes are reused from the original archives. The adaptive-frontier check selects only each source's final current row and previous gate-passed row (ten unique starts), under A/B/Fixed A. Its selection rule was recorded during B training, before native endpoint/frontier outcomes. It is exploratory and training-selected.

| R5/R5.1 source support, six cases each | Archived Fixed A | A | B |
| --- | ---: | ---: | ---: |
| dev_center | 6 | 6 | 6 |
| dev_fast | 5 | 2 | 0 |
| dev_near_slow | 5 | 6 | 4 |
| stationary_0 | 3 | 1 | 5 |
| stationary_2 | 6 | 5 | 6 |
| Total | **25/30** | **20/30** | **21/30** |

B newly qualifies all three `stationary_0:327` slope variants that Fixed A rejected. However, it loses seven previously qualified support cases, all through forbidden first contact. Across-only totals are Fixed A **9/10**, A **8/10**, B **7/10**. B passes both across signs at center, near-slow and stationary_2, neither at fast, and only positive at stationary_0. These sampled results do not define transverse widths or a connected region.

| Adaptive training-frontier start | Fixed A | A | B |
| --- | --- | --- | --- |
| center:693 | Qualified | Speed | Speed |
| center:685 | Forbidden | Forbidden | Forbidden |
| fast:808 | Forbidden | Footprint | **Qualified** |
| fast:800 | Forbidden | Forbidden | Forbidden |
| near_slow:694 | Qualified | Speed | Speed |
| near_slow:686 | Forbidden | Forbidden | Forbidden |
| stationary_0:330 | Qualified | Footprint | Qualified |
| stationary_0:322 | Forbidden | Forbidden | Forbidden |
| stationary_2:330 | Speed | Speed | **Qualified** |
| stationary_2:322 | Speed | Footprint | **Qualified** |
| Total | **3/10** | **0/10** | **4/10** |

Here Speed means native contact-point-speed rejection; Forbidden means forbidden airframe first touch. All ten starts are full-state-valid development proposals. These rollouts preserve complete 0.1 ms physical/motor/target histories, 5 ms autonomous commands, contacts, outcomes and 3D replay files for all three policies. Recorded actuator histories satisfy the frozen first-order motor update, and every replay loads in the existing viewer.

The positive finding is **real local policy learning** at fast:808 and stationary_2:330/322, plus the stationary_0:327 slope triplet. The negative finding is equally material: B fails the later fast:816 starts while succeeding at 808, loses native successes at center:693 and near_slow:694, and does not qualify any reserved earlier case. Fixed A's successes at 693/694 also demonstrate that some apparent backward curriculum advancement was previously unmeasured baseline competence. A time-indexed expansion rule cannot establish a monotonic success frontier.

These results favor the interpretation that the intervention reshaped a narrow recentering/contact success set rather than learning the missing outward-rotation-to-braking skill. They do not isolate whether the remaining gap is dominated by exploration, value generalization or hidden actuator state. All three co-vary with the changed reset distribution. One seed and a finite set of correlated source trajectories limit the inference.

![Matched local native transitions](../runs/asm2-terminal/r6/frontier-transitions.png)

Representative complete replays: [B fast:808](../runs/asm2-terminal/r6/frontier-trajectories/B-391/replay.npz) versus [Fixed A fast:808](../runs/asm2-terminal/r6/frontier-trajectories/fixed_a-391/replay.npz); [B stationary_2:322](../runs/asm2-terminal/r6/frontier-trajectories/B-444/replay.npz) versus [Fixed A stationary_2:322](../runs/asm2-terminal/r6/frontier-trajectories/fixed_a-444/replay.npz). Selection is illustrative; all 30 frontier outcomes remain reported.

## Artifacts, validation and decision

- Literature references, supplements, code/license pins: [retrieval directory](../runs/asm2-terminal/r6/literature/).
- [Frozen experiment](../runs/asm2-terminal/r6/freeze.json), [transition dataset](../runs/asm2-terminal/r6/dataset.json), [reset-row identity](../runs/asm2-terminal/r6/reset-row-provenance.json), and [120 historical Fixed A labels, 58 qualified](../runs/asm2-terminal/r6/historical-fixed-a-labels.json).
- [Endpoint assessment](../runs/asm2-terminal/r6/intermediate-assessment.json), [sealed selection](../runs/asm2-terminal/r6/final-selection.json), [fresh native results](../runs/asm2-terminal/r6/heldout-results.json), [support comparison](../runs/asm2-terminal/r6/support-evaluation.json), [adaptive-frontier comparison](../runs/asm2-terminal/r6/frontier-evaluation.json), and [mechanism accounting](../runs/asm2-terminal/r6/mechanism.json).
- [A endpoint](../runs/asm2-terminal/r6/A/checkpoint-098304.pt), SHA-256 `0ee12ede398ffa4d89202eee630b329a465880fc5fa04fc53c01e1baec3671b2`; [B endpoint](../runs/asm2-terminal/r6/B/checkpoint-098304.pt), SHA-256 `53c5e3ff49fa7dcc6382a66572b49d72c1ccdb35f12c5549cd18ef617d4f3167`.
- [Implementation](../../src/rm27_asm2/r6_curriculum.py), [workflow adapter](../../src/rm27_asm2/r6_experiment.py), [dataset builder](../../src/rm27_asm2/r6_dataset.py), [analysis/evaluation](../../src/rm27_asm2/r6_report.py), and [three focused tests](../../tests/test_asm2_r6.py).
- [Checkpoint/optimizer/resume verification](../runs/asm2-terminal/r6/checkpoint-verification.json), [196,608-transition record verification](../runs/asm2-terminal/r6/physical-record-verification.json), [native frontier actuator verification](../runs/asm2-terminal/r6/frontier-actuator-verification.json), [final preservation receipt](../runs/asm2-terminal/r6/completion-verification.json).

Saved-data reproduction (no training or new physics rollouts):

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
/tmp/rm27-g311-env/bin/python -m unittest discover -s tests -p test_asm2_r6.py
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r6_report dataset
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r6_report verify
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r6_report mechanism
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r6_report curves
/tmp/rm27-g311-env/bin/python -m rm27_asm2.r6_report frontier-plots
```

**Scientific skill-bridging No-Go; baseline promotion No-Go.** Preserve Phase 4 as the verified baseline, Fixed A as the continuation reference, and all R6 checkpoints and negative evidence. No budget extension, additional method, reward/potential, controller, physics/scorer change, hardware, remote modification, commit or push was performed. R5.2 remains pending.

## Exactly one next research experiment — not launched

**A bounded, dynamics-constrained informed-reset connection feasibility study into frozen Fixed A's empirically verified success set.** Use only the five development sources and at most 20 predeclared native candidate validations. Optimize offline CTBR prefixes from earlier rotation states into complete successful handover states, including actual rotor thrust, held-target memory, attitude/rates and finite-body clearance. Validate the planned prefix and then autonomous Fixed A continuation to the unchanged native-qualified contact. Save every failure as well as every successful full trajectory.

This targets the gap exposed by R6: sparse source-time predecessors create local gains and losses but do not supply a verified connection from early rotation. It uses the transferable part of Informed Reset—physically feasible reset states—with the motor-state and native-contact checks absent from a simple geometric planner. Implementation cost is moderate: bounded CTBR direct shooting and existing native restoration/scoring, with no new actor, observation, reward or full G3 architecture. Expected value is a concrete answer about whether useful connecting reset trajectories exist before spending another PPO budget. Planned-prefix success would be **feasibility evidence only**, never autonomous RL success or teacher actions; any later policy would generate all its own actions. No such optimization, rollout or training was launched in R6.
