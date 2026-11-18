# R9 preflight report

**PASS for the authorized CPU/offline preflight; WAITING FOR USER APPROVAL.** New environment transitions0, new native rollouts0. Scientific efficacy NOT_RUN. No full PPO launcher or simulator experiment was executed. Full execution still requires `APPROVE_R9_393216`; this report is not authorization.

![Frozen decay schedule and offline gradient scales](../runs/asm2-terminal/r9-preflight/schedule-and-scale.png)

## Evidence and checks

[Offline results](../runs/asm2-terminal/r9-preflight/offline-results.json), [frozen-runtime tests](../runs/asm2-terminal/r9-preflight/tests-frozen-runtime.log), and [frozen specification](R9_EXPERIMENT_SPEC.md) are the evidence entry points. Rechecked74 R8 completion artifacts,106 source pins and68 dataset provenance pins. Protected R8 A/B and BC archives were not changed. Current branch `codex/asm2-research-update`, HEAD `534f91a829fcc223ecdd236523a7030528ec394a`; preexisting dirty docs, untracked R7/R8 work and tracked deletions remain untouched. No commits/pushes.

| Check | Result / scope |
|---|---|
|Actor observations|All3439 observations exactly reconstruct from saved physical states and registered target;45-element interface unchanged.|
|Action order, scaling, atanh|All source commands reconstruct saved raw labels exactly using original source dtypes; zero censored components.|
|Weighting|.5 demonstration/.5 retention; equal sources then trajectories then samples; seven category/source masses verified.|
|Gaussian mathematics|Analytic mean/log_std derivatives match independent Torch Normal; both signs and a small descent step tested. Fixed-action tanh Jacobian leaves gradient identical.|
|Original PPO objective|Source review confirms old log probabilities remain the sampled-action denominator; regularizer is a separate current Gaussian likelihood. No PPO ratio/advantage/value-target edits.|
|Actor gradient path|Actual PPO.update receives extra mean/log_std gradients before original clipping/Adam. Same-input critic tensors and critic Adam states remain bitwise equal.|
|True zero coefficient|Original PPO versus λ=0 subclass: nonzero learner fixture, real saved BC/Adam, exact Actor/Critic/ValueNorm/optimizer state equality after update.|
|Full checkpoint restoration|All five state dictionaries load exactly from real BC checkpoint. Inherited PPO moments preserved. CPU tests disable GPU fused Adam in both arms only for execution.|
|Resume|Two sequential tensor updates equal save-state/restore continuation, including Actor/Critic/optimizer state. Schedule count retained; dataset/coefficient/schedule metadata mismatch rejected. Actual workflow-shaped binary serialization includes metadata; counter mismatch and absent resume metadata rejected.|
|Schedule|Boundary coefficients and invalid/nonfinite inputs tested; one increment per completed rollout, no minibatch decay.|
|Scale|BC full weighted NLL−3.279386, raw auxiliary norm2.144135; λ=.01 norm.021441. Absolute scale gate PASS; no λ search.|
|Runtime|Five focused tests pass in the exact R8 image SHA9c5b00…; no GPU exposed, network disabled, workspace read-only during tests.|

The nonzero fixture uses real archived observations with synthetic nonzero advantages/returns and generated raw actions. Original regional records lack observations and normalized returns, so no exact historical on-policy gradient replay is claimed. Mean PPO/aux norms in that fixture5.314621/.114030 are engineering diagnostics, not a causal estimate of R8 training dynamics. ValueNorm is loaded and unchanged by update as in the original workflow; original collect/GAE normalization is not rewritten or executed here.

An initial aggregate CTBR inversion check found a maximum1.49e-7 discrepancy. The combined dataset promotes heterogeneous float32/float64 command arrays; inversion in original per-trajectory dtype restores exact equality. Dataset and labels were not changed and no acceptance tolerance was relaxed. An unclosed-file warning in the initial adapter was corrected before final tests.

## Changes actually made

Only isolated `src/rm27_asm2/demonstration_regularizer.py`, its existing test file, proposal config, these three R9 documents and current plan/roadmap were changed for R9. Added finite schedule/data guards; exact Actor parameter identity check; per-update coefficient/NLL/PPO/aux gradient diagnostics with a callback for durable recording; dataset-bound schedule state; and metadata injection/counter checks for original workflow checkpoint saves. The wrapper is context scoped and removable. Added real-checkpoint/nonzero-update parity, analytical gradient and resume/serialization tests. Offline source reconstruction and scale evidence live under the R9 preflight run directory.

Original PPO source, R8 code/config/report, all frozen physical and scoring code, source datasets, BC weights, reward, resets, observations and CTBR mapping are unchanged. No environment reconstruction, BC retraining, native replay, hardware or remote action took place. Tensor parameter updates exist only in CPU test memory; they are not new environment transitions or learned research checkpoints.

## Evidence limits and concrete failure mechanisms

- Source support is narrow: demonstrations have three seeds/two families;3439 rows are correlated data, not3439 independent skills.
- BC already produces0/3 qualified demonstrated starts, all forbidden. Likelihood retention may stabilize an insufficient first-contact maneuver rather than improve precision.
- Initial auxiliary log_std gradients are [.985409,.567792,.864107,.916273]; descent initially contracts exploration. Later residual growth could encourage variance inflation. Both can improve likelihood without improving deterministic mean control.
- PPO and demonstration gradients can conflict; a small initial norm does not bound Adam's cumulative displacement, particularly with inherited moments and a full batch at every minibatch.
- Decay could permit late loss of behavior; selected and final comparison is mandatory.
- Fixed archived states do not cover all autonomous policy deviations. Action accuracy does not guarantee closed-loop stability, contact qualification or new-source generalization.

## Readiness decision

No blocker remains in the authorized tensor/offline scope. The mechanism is ready for the proposed controlled experiment **after explicit approval**, source/runtime pin revalidation and thin driver wiring to the existing workflow. Wiring must pass saved `r9_regularizer` and completed count on resume, persist every `diagnostics_callback` row, use separate R9 output/ledger and enforce shared native budget. This is integration of the frozen mechanism, not permission to redesign it. Actual GPU/environment execution and full training-driver integration remain NOT_RUN, intentionally outside this preflight; CPU parity is not a GPU or native success claim.

R8 remains No-Go and is not reopened. No capability claim is supported yet. The next action is solely the frozen single-arm R9 comparison, pending `APPROVE_R9_393216`; no other algorithm or experiment is proposed.
