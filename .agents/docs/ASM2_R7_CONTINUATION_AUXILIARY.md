# ASM2 R7 — Continuation-success auxiliary representation pilot

**COMPLETE. Scientific No-Go and baseline promotion No-Go.** Both arms completed exactly 98,304 transitions and 96 PPO updates. B produced zero earlier native-qualified contacts and reduced braking retention. The conditional extension gate failed. No further training was launched.

## Supervision and source coverage

The supervised target is frozen Fixed A empirical contact success with a 2.3 s remaining episode horizon. The dataset contains 120 Phase R initial continuations and 30 R5/R5.1 initial continuations, with 83 positive and 67 negative labels. Five original development source families contribute 30 records each. Neighboring frames and numerical perturbations share a source group. Each label corresponds to an actual Fixed A autonomous rollout from an archived complete initial physical state and actual/held rotor thrust. Source hashes and outcomes are indexed in `supervision.json`. Only initial states were selected, avoiding additional suffix-horizon assumptions. No held-out source was used to construct labels or fit predictors.

The 45 actor inputs contain repeated target-relative position, velocity, attitude axes and body rates. They contain no history buffer. Control updates occur every 5 ms with 0.1 ms native integration. Full-state comparison adds actual rotor thrust and held targets, representing the current first-order actuator memory. Label creation checks original pinned evidence, and all 150 reconstructed observations reproduce the native Fixed A CTBR outputs within 7.16e-7.

A fixed ridge logistic regression (coefficient .01, no search) was fitted in five leave-one-source-out folds. Equal source sizes give equal total source weight. Pooled out-of-source actor-visible AUC is 0.98238 and Brier score 0.05354. Full physical state gives AUC 0.98202 and Brier 0.04952. Rotor memory improves this predictor's calibration modestly in this sample. This secondary comparison does not isolate actuator causality. Each fold has only one independent validation source, and perturbations remain correlated.

| Validation source | Positive / total | Visible AUC | Visible Brier | Physical AUC | Physical Brier |
| --- | ---: | ---: | ---: | ---: | ---: |
| dev_center | 15/30 | 0.9867 | 0.0684 | 0.9867 | 0.0641 |
| dev_fast | 20/30 | 0.9800 | 0.0335 | 0.9800 | 0.0296 |
| dev_near_slow | 15/30 | 0.9644 | 0.0660 | 0.9644 | 0.0637 |
| stationary_0 | 18/30 | 1.0000 | 0.0839 | 1.0000 | 0.0785 |
| stationary_2 | 15/30 | 1.0000 | 0.0159 | 1.0000 | 0.0117 |

Coverage extends from failed early/rotation starts to successful braking/contact starts, with only sparse samples at the contact-label boundary. The R5 near-slow pass–fail–pass and cross-track contact flips limit any smooth classification interpretation. These data do not provide action-conditioned intermediate transition targets. Fixed A failure establishes that policy's observed outcome at the given horizon, with physical reachability remaining unresolved.

## Auxiliary mechanism and matched experiment

The original Actor has a 45-input MLP and a 256-dimensional penultimate representation, consumed by its four-action linear output layer. B adds a separate linear binary head directly to those same 256 features. The complete supervised batch of 150 initial observations is used once per PPO minibatch, with fixed BCE coefficient **0.01**. All sources contribute equally. Binary head parameters use Adam at 0.0005. The Actor's original Adam retains its learning rate and restored state.

The gradient pathway is `observations → Actor mean[:-1] → binary head → BCE`. BCE backpropagation adds gradients to the Actor feature extractor after the original PPO surrogate backward call and before Actor gradient clipping and optimizer step. The action head and Gaussian log standard deviation receive only PPO gradients. The binary head has its own clipped gradient and optimizer step. Its tensors, optimizer and update counter are included in the Actor optimizer checkpoint via state hooks, without adding inference parameters to the actor export. The classifier supplies no action, reward, value target or phase switch.

Both arms retain the same Fixed A actor, critic, value normalizer, optimizer initialization, seed 42722, reset probabilities (terminal .10, braking .45, late rotation .20, earlier .25), 45-D input, CTBR mapping, reward, GAE, return targets, PPO objective and frozen plant/scorer. A uses the original PPO path. Both passed one charged one-update stop/resume. Full-batch auxiliary data and a separately initialized head preserve the PPO random stream at construction. Realized trajectories and reset counts diverge with behavior and simulator execution.

The fixed auxiliary coefficient, source split, head placement and full-batch scheduling were implementation choices declared before B evaluation. The matched experiment uses one seed. The predeclared extension criterion also requires one robust 3/3 earlier anchor in addition to gains at two distinct earlier anchors and braking ≥32/36, terminal ≥27/30 without retention exits. Actual results fail both contact improvement and braking retention.

## Learning and native outcomes

| Metric | Control A | Auxiliary B |
| --- | ---: | ---: |
| PPO transitions | 98,304 | 98,304 |
| PPO updates | 96 | 96 |
| Native earlier qualified | 0/36 | 0/36 |
| Distinct successful earlier anchors | 0/12 | 0/12 |
| Earlier envelope exits | 33/36 | 36/36 |
| Earlier footprint failures | 2/36 | 0/36 |
| Earlier forbidden contact | 1/36 | 0/36 |
| Earlier timeouts | 0/36 | 0/36 |
| Braking qualified | 32/36 | 16/36 |
| Terminal qualified | 30/30 | 30/30 |

A completed 295 earlier training episodes with 0 training successes.

B completed 289 earlier training episodes with 0 training successes.

B executed 6,144 auxiliary updates. Its final auxiliary head attains 94.67% accuracy, BCE 0.13454 and Brier 0.03965 on its supervised training labels. These are training-fit metrics. The out-of-source numbers above belong to the separate fixed logistic comparison. The learned B head has seen all five source families, so its training fit cannot be reported as out-of-source generalization.

Initial A/B native earlier scores were both 0/36. Endpoint scores remain 0/36 and there were no earlier training successes. No persistent earlier improvement was observed. Intermediate native checkpoints were not evaluated, so transient native gains between endpoints are unmeasured. Initial braking was 35/36 in both arms.

Final checkpoint selection was sealed before fresh native evaluation. Fresh earlier offsets retain historically examined source families. Fresh rotation/braking/terminal and reached-handover sources are separate from supervision and training.

| Fresh native cohort | Fixed A | Control A | Auxiliary B |
| --- | ---: | ---: | ---: |
| earlier_fresh | 0/36 | 0/36 | 0/36 |
| rotation | 1/18 | 0/18 | 0/18 |
| braking | 15/18 | 15/18 | 9/18 |
| terminal | 18/18 | 18/18 | 18/18 |
| reached_handover | 0/4 | 0/4 | 0/4 |

No earlier qualification completes an autonomous entry-to-contact chain. Existing descriptor-entry diagnostics remain recorded in the evaluation JSON, but no descriptor or classifier threshold is interpreted as entry into an empirically verified success region. The study does not independently label arbitrary states reached by the new policies as successful Fixed A handovers. G3.1.2 loaded-body follow-through remains a separate post-contact finding.

![Learning curves](../runs/asm2-terminal/r7/learning-curves.png)

![Fresh native qualification](../runs/asm2-terminal/r7/native-comparison.png)

## Verification, artifacts and decision

The focused Torch test verifies that with zero PPO advantages, supervised gradients update Actor features while leaving its action output layer and log standard deviation unchanged. Auxiliary state round-trip passes. Real checkpoints verify exact equality to Fixed A for both initial Actor/critic/normalizer and optimizers, and restore the B auxiliary counter through the charged restart. Both final checkpoints report 98,304 transitions, 96 updates and one restart. All frozen source/data hashes pass. Existing native replay loading passes for every saved representative replay.

Implementation is `src/rm27_asm2/continuation_auxiliary.py`, `continuation_supervision.py`, `r7_experiment.py` and `r7_report.py`, with config `configs/rm27_asm2_r7.json`. Reproducible records are under `.agents/runs/asm2-terminal/r7/`. `freeze.json` pins source/config/reset data, `supervision.json` records all labels and source folds, `ledger.json` accounts transitions, and `checkpoint-verification.json` records learner-state checks. A/B checkpoint-098304.pt and portable .npz exports are saved alongside per-update PPO diagnostics and native evaluations. `heldout-results.json` contains the fresh outcomes and `heldout-replays/` contains recorded 3D trajectories.

Representative replays include [B braking failure](../runs/asm2-terminal/r7/heldout-replays/B-braking-failed.npz), [B braking success](../runs/asm2-terminal/r7/heldout-replays/B-braking-qualified.npz) and [A rotation failure](../runs/asm2-terminal/r7/heldout-replays/A-rotation-failed.npz). Replays use the existing recorded-state viewer format.

**No-Go.** The auxiliary objective learns to classify the supplied continuations while earlier autonomous skill bridging stays at zero and braking retention deteriorates. The strongest remaining evidence points to difficulty learning action-dependent transitions into contact-capable states, plus interference with the existing braking representation under this auxiliary update. Gradient conflict itself remains a hypothesis because this pilot did not measure per-objective gradient alignment. Classification performance alone failed to provide the required control-learning benefit. One seed and a sparse empirical boundary limit causal generalization.

**Exactly one next research action, NOT_LAUNCHED:** an offline gradient-interference diagnostic using the saved PPO batches, labels and checkpoints, measuring auxiliary-versus-PPO feature-gradient alignment and braking-action displacement. This would test whether the observed retention loss comes from conflicting shared-feature updates before another training budget is considered. It requires no new native rollouts or PPO transitions.
