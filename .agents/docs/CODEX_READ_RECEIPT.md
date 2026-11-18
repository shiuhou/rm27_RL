# RM27 Research-to-Code Handoff v2 — read receipt

Status: ingestion, isolated candidate implementation and CPU synthetic learner tests COMPLETE. R9 remains PROPOSAL / NOT AUTHORIZED TO TRAIN. No new PPO environment transitions, native rollouts, BC reconstruction, commits or pushes.

## Actual source locations and reading

The requested `~/rm27_research_inputs/handoff_v2` does not exist under dsa-cli. Actual supplied bundle is `/home/dsa/rm27_research_inputs/handoff_v2/`; it was read without changing it. `sources/Paper_To_Code` resolves in this bundle to `sources/RM27_Paper_To_Code_Transfer`. Read README_FIRST, CURRENT_DECISION, CODEX_4090_V2; the historical host label does not change RL scope.

Read Round1 `sources/REPORT.md` (task events, source comparisons, A/B/C and limits), `RM27_Independent_Research_Round1/architecture_study/learning/LEARNING_EVIDENCE.md` (SimpleFlight and Flightning), and `independent_architecture/rl/RL_CONTACT_REVIEW.md` (PerchRL, Pi, moving landing). Read Paper-to-Code `RL_INCLINED_TRANSFER.md` and `TRANSFER_DECISION.md`; Round2 `RM27_ROUND2_ARCHITECTURE_DECISION.md`; `sources/ASM2_R8_USER_REPORT.md`. Round2's R8/D continuation recommendations are superseded. No claim of current 5060 inspection or of independently reproducing Dot's runs.

### Official source actually inspected

InclinedDroneLander official raw files fetched at commit `a301375eecc7687e4d6547c5e1bbfc2087d50389`; read-only research copies in `/home/dsa-cli/rm27_research_inputs/handoff_v2_source_cache/`. No upstream code incorporated into RM27 implementation.

- `main.py:20–27,43–53`: 50 Hz, PWM amplitude16000, requested3M, fixed gamma.97. This is SB3, not our tensor PPO.
- `Environments/crazyflie.py:54–71,81–159`, `Crazyflie_2d_inclined.step/reset`: five visible state values; hidden thrust persists across reset; full visible-state clipping; PWM/pitch action; goal tolerance, position radius, tilt and platform change on reset. No success-adaptive schedule. Collision heuristic zeros horizontal velocity after reward.
- `Reward/rewardfuncs.py:8–41`, `sparse_reward2d`: additive goal/boundary/polygon penalties; goal box terminates without requiring physical contact. Three points are not a full hull or impulse model.
- `EOM/eom.py:25–49`, `eom2d_crazyflie_closedloop`: identified thrust state plus first-order pitch dynamics, not RM27 nonlinear 3D CTBR plant.
- Source hashes (main/environment/reward/EOM): `d1dd50309891fec517c2cc00b3c8dadcdbca6d24477430bb46a67a59f4c6f48f`, `fd314ad4cb0786083e586a7d89716669761306ad44f0ad86d6c63b38ef5b5664`, `b60a38ff369e39c9b3adeb7f8e988cb0b4b00b7618cb898fa7fe5d90e6845351`, `cc9fe4b2dd055789cb0049ca3eab448d5d44952ad988891e1fa4fc5407f5d877`.
- Local official SimpleFlight snapshot: `upstream/SimpleFlight/cfg/algo/mappo.yaml:1–69` and `omni_drones/envs/single/track.py:383–405`; pinned project revision `f5ae8fc3689edc7c982e07cbfd587eeeab72f279`. Actual future target positions differ from ASM2's repeated current goal delta. Retain existing PPO/action abstraction, not its tracking references.

Transfer decision: use separation of learning objective, reset state and independent physical outcome; do not port Crazyflie constants, hidden-state reset omissions, weaker goal-event scorer or SB3 clipping convention. Dot's 99/100 is reported goal-event evidence, not ASM2 contact qualification. Its curriculum ablation is a proposal, not demonstrated causal superiority, and would repeat a direction already tested locally. Flightning's surrogate gradients and PerchRL's temporal/privileged interfaces require new assumptions; not selected. Fast-Perching terminal freedom is relevant to existing feasible demonstrations, not evidence for another RL optimizer or full recovery. No literature source here establishes demonstration-regularized PPO success for ASM2.

## Local R8 evidence, CONFIRMED from saved artifacts

Working tree `/home/dsa-cli/ceo_pj/RM27_Agile_Flight_RL_Project`, HEAD `534f91a829fcc223ecdd236523a7030528ec394a`. Preexisting dirty docs, R7/R8 additions, `.gitmodules` and experiments deletions preserved. Root instructions and active plan govern the isolated additions.

Rechecked R8 config and `demonstration_learning.py` action inversion/weights/fit, `r8_experiment.py:60–72` initial Actor replacement, `r8_report.py:14–46` retention and selection. Native inference/restoration remains `terminal_native_eval.py:78–136`; actual policy loss and clipping are `simpleflight_lab/ppo.py:130–160`. No frozen file edited.

[Local evidence JSON](../runs/asm2-terminal/handoff-v2/local-evidence.json) checks 74 completion-pinned artifacts including actual checkpoint files, 106 frozen source pins, both 384-row training logs, native checkpoint outcomes, BC failures, fresh results and saved state arrays from three BC replays plus the transient successful replay. Existing checkpoint/evidence verification receipts are preserved; this turn does not claim a new full tensor/deserialization audit or new physical reproduction.

- Each arm393,216 transitions; B294,912 qualifications are exactly nominal/minus/plus of `dev_lateral_positive:730`; B final0/36, both final braking23/36. Fresh earlier0/36 all; B fresh braking11/18 vs Fixed A16/18.
- BC three demonstrated starts: forbidden first touch at1.3490/1.3620/1.3656s. Saved end physical states retained in the evidence JSON. Failure is already present before PPO: qualified BC skill never existed. B98,304 changes these outcomes to exits, consistent with loss of contact-reaching behavior, not proof of a unique forgetting cause.
- Correction to CURRENT_DECISION: final24/36 exits belongs to A, not a previous B endpoint. B itself goes33,36,0,0 exits across its four PPO checkpoints. Final B's zero exits are33 timeouts and3 contact-speed rejections.
- No proof of global unreachability, observation insufficiency, impact recovery, official impact scoring or causal attribution to motor memory.

## Concrete candidate and research decision

[Independent implementation](../../src/rm27_asm2/demonstration_regularizer.py), [proposal config](../../configs/rm27_asm2_r9_proposal.json), [tests](../../tests/test_asm2_demonstration_regularizer.py). No training CLI is wired or enabled. Context-managed subclass adds `lambda(t) * -sum(w*log pi(raw_expert|obs))` after original PPO actor backward and before the original gradient clipping/Adam step. No critic, advantage, reward or inference change. Full current Gaussian likelihood updates mean and log_std; tanh Jacobian for fixed archived actions is parameter-independent. This deliberately differs from R8 BC's frozen-variance fit; variance inflation is a risk to report, not an extra exploration intervention.

Weights reuse R8's half demonstration/half retention, source/trajectory balancing, exact45D and raw CTBR labels. Full-batch evaluation consumes no sampling RNG. Proposed lambda starts.01, linearly decays to zero at393,216 completed additional transitions; restore count from checkpoint on resume, never wall time or minibatch count. The coefficient is an unvalidated fixed proposal, not tuned on B outcomes.

Two real-Torch CPU tests PASS (tests.log): actual PPO.update on synthetic tensors shows enabled mean/log_std gradients, exactly identical critic tensors, zero-coefficient unchanged Actor with zero surrogate advantage, and transition schedule boundaries. No environment stepped; no learned checkpoint saved. This is a learner-path test, not training readiness or a native success claim. Before future launch, wire an isolated driver and verify real checkpoint resume parity; current module requires caller to provide saved completed_transitions.

Decision: conditional GO to consider one controlled experiment, NOT a capability Go or automatic training authorization. BC failure does not prove action regularization cannot help subsequent PPO, but it can preserve incorrect contact and may worsen adaptation. The falsifiable target is multiple earlier anchors AND retention, not demonstration loss or reaching contact.

## One proposed experiment, awaiting separate authorization

Use archived R8 B as control at identical transition checkpoints; do not rerun it. One new arm from saved BC, same seed42722, reset10/45/20/25, parent critic/Adam and frozen plant/scorer. Only the decaying demonstration likelihood changes. Requested maximum393,216 new PPO transitions; native allowance831 includes564 checkpoint/demo evaluations,188 selected/final fresh episodes, up to79 replay reproductions; unused allowance is not new cases. Checkpoints every98,304. Initial BC baseline reused. No sweeps or automatic extension. Stop technical invalidity; intermediate zero early qualification alone is not futility.

Require >=2 independent earlier anchor identities, dev braking>=32/36, terminal>=27/30 and no retention envelope exit; selected hashes sealed before fresh evaluation and final persistence reported. Reuse R8 development selection and archived control at the same dose. Same-seed historical control is not a contemporaneous randomized replication; software/runtime pin drift blocks launch, and one seed cannot support broad statistical generalization. Failed retention rejects promotion even with earlier gains. This proposal tests retention-guidance during PPO, not a new curriculum or full contact-to-recovery architecture.
