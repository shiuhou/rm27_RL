# R9 — Controlled Demonstration-Regularized PPO

Status: **PREFLIGHT COMPLETE / AWAITING APPROVAL**. This document preregisters one candidate, not a training result. R8 remains Scientific No-Go. Training and native evaluation are NOT_RUN. Only a subsequent explicit user message `APPROVE_R9_393216`, with passing current preflight pins, authorizes execution. No launcher is enabled by this specification.

## Question and falsification

From the same saved R8 BC initialization, does a decaying action-likelihood regularizer produce persistent autonomous qualified earlier contacts at multiple distinct anchors while retaining braking/terminal competence? Action prediction, demonstration-region behavior, qualified first contact, and generalization are four separate outcomes. Lower imitation loss, more contact-reaching, or a repeated forbidden touch cannot pass the scientific gate.

## Frozen intervention

Let y_i = atanh(a_i) be archived normalized executed actions in Actor order [p,q,r,T], x_i the archived native float32 45-element observation. The current policy is a diagonal Gaussian over raw actions, with mean μθ(x), trainable log standard deviations sθ. Physical execution remains tanh followed by CTBR scaling: rates ±2 rad/s; collective [0.5,1.5] mg, physical order [T,p,q,r]. Inverse-tanh epsilon is the existing 1e-6; zero source components were censored.

L_demo(θ) = −Σ_i w_i log N(y_i; μθ(x_i), diag(exp(2sθ))).

L_actor(θ) = −4 mean_j[min(r_j A_j, clip(r_j,0.9,1.1) A_j)] + λ(c) L_demo(θ),

r_j = exp(log πθ(u_j|x_j) − log π_old(u_j|x_j)); λ(c)=0.01 max(0,1−c/393216).

Here u_j are the actual sampled PPO raw actions, never substituted with demonstration targets. Original globally normalized advantages and clipped value/Huber loss are unchanged. The tanh Jacobian of each fixed demonstration action is constant with respect to θ and is omitted from the regularizer; likelihood diagnostics explicitly report raw Gaussian density, not a probability of contact. Negative NLL is valid for continuous densities.

c counts **completed new R9 transitions before each rollout update**, not inherited Fixed A transitions, minibatches, epochs or wall time. Each 64×16 rollout advances c by1024 after its full update. Coefficients at completed checkpoints0/98304/196608/294912/393216 are .01/.0075/.005/.0025/0; the final update begins at392192 and uses .0000260416667. There is no extra zero-coefficient update. Apply the full-dataset weighted term at every original Actor minibatch (4 epochs×16), then original joint gradient clipping10 and original Adam. Critic gets no demonstration gradient. No new network, action switch, reward, reset curriculum, teacher execution or actor input is introduced.

### Coefficient rationale, fixed before candidate evaluation

Retain handoff's .01, not a sweep. Before measuring it, the offline scale sanity gate was: λ times the real BC demonstration-gradient norm must be ≤1 (one tenth of original clipping10); otherwise block for review, not tune using success. Measured2.144135 unscaled, .021441 weighted: PASS. This constrains initial magnitude, not cumulative optimizer influence. Nonzero synthetic learner fixture has mean PPO/auxiliary norms5.314621/.114030 during one update. This fixture is NOT an original on-policy rollout and does not estimate the actual training gradient ratio. Runtime diagnostics must report that ratio without adapting λ.

Initial log_std gradients are positive in all four dimensions, so this regularizer initially encourages variance contraction. Later mismatch can reverse the sign. Both contraction and inflation must be reported; neither can be silently fixed by changing the method.

## Data and independence

Use only existing `r8/demonstrations.npz`:3439 pairs,2023 demonstration and1416 retention. Seven correlated qualified planned-prefix/autonomous-suffix traces cover three demonstrated seeds in two source families. Twenty-five autonomous retention trajectories cover five families. Half total mass belongs to each category; within category sources receive equal mass, then trajectories, then frames. Thus demonstration stationary_0/stationary_2 each .25; retention center/fast/near_slow/stationary_0/stationary_2 each .10. Full-batch evaluation consumes no extra sampling RNG. Existing source-grouped R8 validation is retained; no new fold or held-out result is used to choose λ. Frames and perturbations are not independent trajectories or independent skills.

Complete physical/actuator history is provenance only. Actor remains current goal delta repeated10×, world velocity, R-transpose entries and body rates. No phase, future state, motor/history or trajectory reference input. Per-source command inversion must use original archive dtypes before concatenation; recomputing from promoted combined CTBR arrays introduces float32 rounding discrepancies.

## Initialization, runtime and resumption

BC SHA256 `d49d089a091098b760dce0703f9b9e1cc710436df9fcc3649b5c04a338526284`; dataset SHA256 `c90d50de9fc7a2aba10e92176f014ec6148b83f8b4b1745af5e8ca1016faa774`. Restore Actor, Critic, ValueNorm and both inherited PPO Adam states; do not refit BC or reset Adam. No observation normalizer is added. Learning rate comes from preserved R8 workflow/optimizer (.00012); gamma .995, GAE .95; all remaining settings from unchanged R8 config chain.

Seed42722; original16 worlds, reset terminal/braking/late/earlier mixture10/45/20/25; same initial RNG order. Runtime `xtrainer-astra:isaac-2.1.0-repaired`, image `sha256:9c5b00fb775c65913f78f22eadb56912853c1f9f1c38828e53fb99369e29c49e`. Frozen source drift blocks launch. CPU tests disable fused Adam identically for comparison; they do not claim GPU arithmetic or simulator-resume equivalence.

Future authorized driver must wrap the existing workflow in `install_regularizer`, initialize c=0, and on resume load `r9_regularizer` from the saved checkpoint alongside its `completed_transitions`. The context injects metadata and diagnostics into original workflow checkpoint saves and rejects counter mismatch. Metadata binds dataset SHA, λ and decay budget. Restore original RNG after constructor creation. Original workflow restores fresh valid resets, not simulator solver cache. One charged save/restart after the first1024 transitions matches R8's restart cadence; maintain original ledger accounting and no discarded transitions. No new general training framework is needed. The tensor/save adapter is tested; a simulator launch is intentionally not part of preflight.

## Dose, selection and decision gates

Single new arm, maximum393216 transitions, checkpoints at98304/196608/294912/393216; zero endpoint reused BC. Archived R8 B is the control; no A/B retraining. No multi-seed, extension or coefficient change. See comparison plan for the831 native upper bound.

Retain original R8 development selection exactly: among endpoints with braking≥32/36, terminal≥27/30 and zero retention envelope exits, maximize distinct earlier qualified anchors, robust3/3 anchors, earlier contacts, braking contacts, then latest endpoint. If none eligible, final is analysis-only. Seal selected checkpoint hash before any fresh evaluation. BC is never substituted for a trained candidate.

Scientific adoption requires improvement over R8 B at matched dose at **at least two distinct earlier anchors**, plus **at least one additional robust3/3 anchor**, all original retention gates, and confirmation in deterministic replay. Demonstrated starts do not count toward this gate. Require gains at at least two of the same earlier anchors to persist at final under retention gates; selected-only gains are transient, not adoption. Report fresh multi-anchor results separately; absent fresh earlier improvement, generalization remains No-Go even if local development gates pass. A single source/anchor cluster cannot establish broad skill bridging. This is one-seed evidence, not statistical proof of universal advantage.

Stop on nonfinite update, changed frozen contract, ledger/budget violation, or invalid checkpoint/metadata. Severe collapse is operationalized as zero development braking AND zero terminal qualification at two consecutive scheduled assessments; stop without spending remaining allowance. Zero earlier success alone is not a futility stop; no arbitrary wall deadline. No-Go at budget end if gates fail. No automatic follow-up campaign.

## Frozen artifacts

Config `configs/rm27_asm2_r9_proposal.json`; isolated mechanism and tests; three R9 documents; run evidence under `.agents/runs/asm2-terminal/r9-preflight/`. `freeze.json` binds these plus R8 manifests and protected baselines. Changes after sealing require a disclosed amendment before any execution, never silent outcome-driven tuning.
