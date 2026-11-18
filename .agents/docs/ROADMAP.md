# RM27 Agile Flight RL — ROADMAP

**Version:** 1.1

**Date:** 2026-11-18

**Status:** N1/R9 plan alignment complete; R9-A offline consumer PASS (55/55 CPU tests) against published G0 9ea80518259173c15882ae8a3ab6059eb2643336, acquired locally. Published scenarios are diagnostic-only with no admitted recorded cases; overall R9-A, formal RL evaluation and R9-B remain BLOCKED. Phase 4 baseline and Fixed A retained; historical R8/R9 closed No-Go. No new training/native authorization.

---

## 1. Roadmap Principles

- Milestone describes a capability/result, not a list of coding tasks.
- Each milestone has an explicit deliverable and evidence-based exit gate.
- A milestone is not PASS merely because code exists or training completed.
- Training does not start until the new airframe, contact task and evaluator are sufficiently defined.
- Old-model results remain historical evidence and are not silently transferred to the new model.
- Independent work may proceed in parallel only when dependencies allow it.

## 2. Overall Roadmap

| ID | Milestone | Status | Main dependency |
|---|---|---|---|
| M0 | New Airframe CAD & Design Freeze Candidate | COLLISION/MASS REVIEWED; design freeze pending | Contact-region intent and physical characterization |
| M1 | New Airframe Physical Model / Frozen Plant v0 | PROVISIONAL_FREE_FLIGHT_PASS; plant not frozen | Actuator/control characterization |
| M2 | Full-Underside Contact Contract & Conformance | Native and proxy fixtures pass | Impulse accounting and contact response conformance |
| M3 | Deterministic Feasibility & Validated Handover Set | Sampled simulated handovers verified | Navigation estimation, latency and broader reachability |
| M4 | Terminal Agile Contact Skill | Braking promotion gate passed | Broader approach coverage |
| M5 | Full Handover-Distribution Agile Policy | R8 and historical R9 No-Go; N1/R9-A CPU consumer/protocol in verification | Admitted published scenarios, G0 consumer/model conformance, multi-source capability and controlled learning evidence |
| M6 | Vision-Based Adaptation | NOT_STARTED | M5, perception/VIO interfaces |
| M7 | Sim-to-Real Flight / Contact Readiness | NOT_STARTED | M1–M6, safety authorization |
| M8 | Agile Contact + Recovery Integration | Full-task requirements aligned; implementation NOT_STARTED | G0 event semantics, fixed-handover contact evidence, continuous separation/recovery |

### 2.1 N1/R9 研究路線（2026-11-18）

本路線整合「正常帶速導航 → 接管 → 敏捷接觸 → 分離 → 穩定恢復」的共同研究目標。以 [SPEC §4.2](SPEC.md#42-n1r9-共用研究邊界2026-11-18) 為需求、[DESIGN §6.5–6.6](DESIGN.md#65-n1r9-a-shared-scenario-consumer--fixed-handover-protocol) 為架構。以下為 gates，沒有日期承諾或自動預算。

| Gate / 工作包 | 最新狀態 | 交付與通過條件 | 未通過時 |
|---|---|---|---|
| G0 正式共用資料發布與 4090 身份驗收 | 發布 commit `9ea80518259173c15882ae8a3ab6059eb2643336`、`0.1.0-draft / PROPOSED`、80 檔本地 pin；正式 dataset BLOCKED | acquisition hashes／原始 producer receipts 可追溯；發布 3 diagnostic records、單一 dev family、0 admitted recorded scenarios；未發布 closure 分開 | 不自建替代契約，不啟動正式共用比較；接收方 inventory 不冒充 producer release manifest |
| **R9-A Shared Scenario Consumer / Fixed-handover Protocol** | **離線 Consumer PASS（33 官方 + 22 本地 tests）；整體／dataset BLOCKED** | lossless replay／45D synthetic-only mapping／provenance／GT isolation／source split／具名 slices 與 ledger 已驗；結果見 `.agents/runs/n1-r9-a/results.json` | 缺失 runtime 資料不可用 GT 補齊；不能稱 G1-R PASS |
| G1-R RL protocol readiness | BLOCKED；fixed-handover protocol 僅離線準備 | admitted published snapshots、Actor observation/normalizer/history、模型／scorer／時間相容性，以及 Direct-CTBR authority 語義需就緒；任何 native smoke 另批 | 不啟動 PPO；published REFERENCE_TRAJECTORY conformance 不替代 RL runtime readiness |
| **R9-B Controlled Learning Experiment** | **BLOCKED / UNSELECTED / NOT_AUTHORIZED** | G1-R 就緒後，一個有證據的機制、可比 control、凍結訓練／評估規格、測試、獨立批准與明確 transition/native/replay 上限 | 不重跑歷史 regularizer，不 sweep，不沿用剩餘 allowance |
| G2 Common Fixed-handover Capability | NOT_PASSED | 共用 scenario 上的分項 contact／separation／recovery／安全與 timing 結果；RL 新 anchor、跨來源與 retention 證據 | No-Go 也形成結果；不能由完成訓練宣稱能力成立 |
| G3 Full Navigation / Autonomous Handover | DEFERRED | 固定接管能力具覆蓋後，才研究各自接管選擇；所有合法 navigation attempts 作分母 | 不先訓練 discrete selector 迴避難例 |
| G4 Robustness / Hardware readiness | NOT_AUTHORIZED | 有依據的估測／時延／模型擾動、重複任務與尾部計算時延；硬體另有安全批准 | 不作真機／泛化聲稱 |

5060 的 N1-A/B/C 是獨立 Non-RL 路線與授權；4090 不執行其規劃器、native 預算或 remote 修改。共享的是正式資料和評分語義，RL observation、action、PPO 和 training pipeline 獨立。非 RL 的七條可行連接、幾何相似或同一來源相鄰 frames 不構成 RL 泛化。

#### 歷史證據與命名界線

| 歷史項目 | 已完成證據／判定 | 本路線用途 |
|---|---|---|
| Phase 4 | checkpoint-186368.pt，verified learned baseline | 保護與 regression reference，非完整接觸恢復能力 |
| Fixed A | checkpoint-196608.pt，continuation reference | 既有自主末段接觸對照，不當作一般導航入口 |
| R8 BC → PPO | A/B 各 393,216 transitions；BC 0/3，均 forbidden；B 中途 earlier 3/36 僅一 anchor，final/fresh earlier 0/36；Scientific No-Go | 保留全部正負結果，不重跑 |
| **歷史 R9 demo-regularized PPO** | 393,216 transitions，726 native/replay；final earlier 0/36，braking 25/36，terminal 30/30；No-Go | 證明較好 imitation 未形成 bridging；不是待做方法 |
| R9.1 corrective generator | 0/9 qualified corrections；1152 optimizer integrations，9 validations；refit/PPO 未跑 | 拒絕此有界 label generator，不推論全局不可達 |

歷史報告與凍結 `R9_EXPERIMENT_SPEC.md`、`R9_COMPARISON_PLAN.md`、`r9/` 等身份保持不變。為避免同名混淆，新工作一律寫 **N1/R9-A consumer**、**N1/R9-B controlled learning**；新實驗 ID 與機制尚未決定，不能覆用歷史 R9 config／輸出目錄。

附件將 regularizer 描述為「僅 CPU 實作、未訓練」與本機較新完成證據衝突，採本機原始 artifacts／最終報告為準。因此 R9-B 不預選 regularizer，需在 R9-A 後提出新的具體假設並獨立批准。完整恢復 gate 的來源與本地尚未驗證的邊界見 SPEC；本輪僅執行已授權的 R9-A CPU consumer／protocol 準備。

## 3. Milestone Details

The [ASM2 Aircraft Model Release v1.0.0](../../artifacts/asm2_model_release/README.md) packages the provisional MuJoCo plant and the Phase 4R.2 Isaac proxy. Its sampled first-contact gate reproduced 64/64 labels on RTX 4090.

A separate [MuJoCo collision proxy v0.1.0](../../artifacts/asm2_mujoco_proxy_v0.1.0/README.md) exports the 3,522 Phase 4R.2 hulls to actuated MJCF. Its first-contact classification matched the detailed native reference on the ten frozen controls.

The [repaired Isaac runtime](DESIGN.md#34-isaac-runtime-on-the-rtx-4090) passed
GPU kernel/image I/O, deterministic flight and both sampled first-contact layouts
on 2026-11-12. The main launchers use this separate image; research milestone and
training authorization status are unchanged.

### M0 — New Airframe CAD & Design Freeze Candidate

**Status:** `PARTIALLY_REVIEWED` — ASM2 STEP / USD / MJCF integrity, mass provenance and collision semantics reviewed. Full M0 gate not yet passed.

Objective:
- Receive and review the complete new drone assembly.
- Identify all parts, coordinate frames, rotor locations and underside geometry.
- Clarify the intended full-bottom-shell contact region.
- Record CAD-derived mass properties and their assumptions.

Deliverables:
- CAD assembly inventory.
- Coordinate-frame convention.
- Part/role map.
- Candidate underside contact region.
- CAD mass-property report.
- Unknowns / measurement plan.
- New-airframe reference version.

Exit gate:
- Complete assembly is available and interpretable.
- Relevant component geometry and coordinate frames are identified.
- Contact-region intent is confirmed by the user.
- CAD estimates are distinguished from measured physical properties.

No RL training in M0.

### M1 — New Airframe Physical Model / Frozen Plant v0

**Status:** `PROVISIONAL_FREE_FLIGHT_PASS` — matched Isaac/MuJoCo Hover, Shift and moving-reference suites passed with deterministic CTBR. The flight plant remains provisional pending actuator/control characterization. See [flight design and launch commands](DESIGN.md#33-asm2-provisional-free-flight-baseline).

Objective:
Build and validate the new rigid-body and actuator model.

Work:
- Convert CAD to visual, collision-proxy and evaluation geometry.
- Establish mass, COM and inertia.
- Establish rotor positions, axes and thrust model.
- Characterize motor/ESC response and rate-controller response where possible.
- Implement the new plant in Isaac and MuJoCo.
- Validate free-flight response against analytical checks and available measurements.

Deliverable:
`RM27 New-Airframe Frozen Plant v0`

Exit gate:
- Physical parameter provenance documented.
- Hover trim and actuator mapping are coherent.
- Basic free-flight and angular response tests pass.
- Model limitations and uncertain parameters are documented.

### M2 — Full-Underside Contact Contract & Conformance

The Phase 2 evaluator distinguishes qualifying patch contact from forbidden contact on the verified 2027 Field crop. The 3,521-hull proxy passes the sampled geometry and Isaac fixture checks. Fine-step native qualification and matched coarse-step diagnostics expose distinct sampling, solver and reporting effects. [Phase 2 contract and commands](DESIGN.md#94-asm2-phase-2-contact-contract)

Objective:
Replace the legacy sphere-contact definition with a CAD-grounded underside contact model.

Work:
- Define allowed shell/contact patch.
- Define local normal and surface-frame representation.
- Define overlap/contact-centroid metrics where geometrically meaningful.
- Define first-contact and forbidden-contact classification.
- Derive thresholds from official rules, geometry and physical evidence.
- Build MuJoCo scorer.
- Build matched Isaac/PhysX contact conformance fixtures.

Deliverable:
`RM27 Full-Underside Contact Contract v1`

Exit gate:
- Contact geometry is tied to the new CAD.
- Scorer distinguishes valid underside contact from forbidden first contact.
- Positive and negative fixtures are reproducible.
- Isaac training contact events are sufficiently consistent with the independent evaluation reference for the tested domain.

### M3 — Deterministic Feasibility & Validated Handover Set

Phase 3 qualified eight moving handovers on the detailed native model. The existing navigation controller reached nine of twelve proposed gates. A frozen held-out case reached handover and then failed on body-first contact. Three stationary stress references qualified. These results establish a sampled set for the provisional plant and ground-truth simulated navigation.

Objective:
Prove physical feasibility and define a handover distribution that the actual navigation system can reach.

Work:
- Implement deterministic planner/witness or other feasible controller.
- Identify contact-feasible states and approach trajectories.
- Generate candidate reset states.
- Measure the conventional navigation controller's achievable handover distribution.
- Include state-estimation error, delay and safety boundaries.
- Compare feasible states with navigation-achievable states.

Deliverable:
`RM27 Handover & Feasibility Benchmark v1`

Exit gate:
- At least one reproducible nominal feasibility witness exists, or infeasibility is explicitly characterized.
- Handover set is supported by navigation reachability evidence.
- Training and held-out evaluation distributions are frozen before training.
- Handover set is not arbitrarily narrowed to inflate policy success.

### M4 — Terminal Agile Contact Skill

**Status:** The Phase 4 continuation with the Phase 4R.2 overlay and action map reached 30/36 development braking qualifications with zero envelope exits. It qualified 28/36 on the old braking regression and 17/18 on fresh reset frames, ahead of Frozen Track and constant commands on both. Rotation and navigation-reached handover coverage remain limited. [Phase 4 inputs](DESIGN.md#73-phase-4-terminal-inputs)

Objective:
Learn the final approach/braking/contact portion from physically valid terminal-neighbourhood states.

Candidate methods:
- PPO baseline.
- TD3 challenger.
- Direct-CTBR first candidate.
- Target-relative observation with history if justified.
- Witness-derived reset states, without teacher-action replay.

Deliverable:
A reloadable terminal-contact policy and independent evaluation report.

Exit gate:
- Held-out contact states qualify under the frozen scorer.
- Baselines are evaluated on matched initial states.
- Contact kinematics, geometry, rejection reasons and saturation are reported.
- No success is counted from deterministic witness execution.

### M5 — Full Handover-Distribution Agile Policy

**Independent terminal-set feasibility complete (2026-11-13): bounded GO.** Reusing the three saved R6.1 safe seeds, twelve native validations produced twelve feasible prefixes and seven qualified Fixed A continuations across all three seed identities and both original source families. Terminal support used 125 actual states from 25 successful R5/R5.1 continuations; proximity alone failed on four in-neighborhood candidates. Cost: 1,197 detailed optimizer integrations plus three failed pre-integration calls, conservatively charged as 1,200 evaluations; total summed worker time bounded at 46.02–46.15 minutes (46.37 conservatively charged). Exact same-instance physical/actuator transfer and remaining episode time verified; zero PPO. This is optimized-prefix feasibility, not autonomous RL success or a certified basin. No Auxiliary PPO dependency. One unlaunched recommendation: twelve early-state robustness validations of two successful saved control sequences, without reoptimization. [Evidence, native outcomes and limits](ASM2_TERMINAL_SET_FEASIBILITY.md).

**R7 complete — No-Go.** Actual Fixed A continuation-success supervision influenced Actor features through fixed .01 BCE. A/B each used 98,304 transitions. Earlier native 0/36 each, braking 32/36 versus 16/36, terminal 30/30 each. Fresh earlier 0/36 each, braking 15/18 versus 9/18. No extension. [Report](ASM2_R7_CONTINUATION_AUXILIARY.md). One unlaunched next action is an offline auxiliary/PPO feature-gradient interference analysis.

**R6.1 complete — decision C: no matched connection found.** Exactly twenty native development validations across five families, after 3,211 bounded optimizer prefix integrations. Three safe but off-target prefixes reached Fixed A; none qualified. Seventeen prefixes stopped at forbidden contact; none matched the full-state/actuator target. Zero PPO, unchanged plant/scorer/checkpoints. This is not proof of global unreachability. [Report](ASM2_R61_INFORMED_CONNECTIONS.md). One next recommendation, not launched: bounded clearance-aware multiple-shooting feasibility restoration from the three saved safe prefixes, with unchanged native gates. R5.2 remains pending; R6 closed results remain below.

**R6 complete (2026-11-13): local learning confirmed; skill-bridging and promotion No-Go.** Actual literature/supplements and pinned curriculum source informed one competence-gated predecessor reset experiment. A/B each completed 98,304 transitions; endpoint earlier qualification 0/36 each, robust anchors 0/12 each, braking 34/36 versus 31/36, terminal 30/30 each. Fresh earlier and reached-handovers remain unsuccessful. B learns narrow native development gains at fast:808, stationary_2:330/322 and the stationary_0:327 slope triplet, with substantial losses elsewhere and nonmonotonic support; no certified region or generalization claim. Complete physical/actuator trajectories, tests, checkpoint/resume verification and replays preserved. No extension; preserve Phase 4 and Fixed A. R5.2 remains pending. One unlaunched next study: bounded CTBR/actuator-constrained informed-reset connection feasibility into verified Fixed A success states before further PPO. [Evidence and limits](ASM2_R6_SKILL_BRIDGING.md).

**R5.1 complete (2026-11-13): mixed transverse sensitivity.** Exactly ten prescribed across-tangent variants yield 9/10 native qualifications, with five R5 nominal controls reused. Four anchors pass both signs; dev_fast:816 negative has an unloaded forbidden body point in the first signed manifold and remains rejected by the frozen scorer. Three anchors have successful slope/across samples at the same frame; stationary_0:330 lacks same-frame slope trials. These are useful multidirectional samples, not a continuous reachable region or early-rotation bridge. Complete 0.1 ms state/actuator records, matched plots, two tests and saved-data scorer/actor verification pass; all 148 pins preserved, zero PPO and no held-out use. One three-case negative-cross amplitude-bracketing study at dev_fast:816 is recommended, not launched. [Evidence and limits](ASM2_R51_CROSS_TRACK.md).

**R5 complete (2026-11-13): local Fixed A backward extension observed, full early-rotation bridging unproved.** Exactly 20 new native continuations yield 16 qualifications, two body-first-manifold failures and two footprint failures. Sampled 3/3 slope-sensitive entries extend 20–25 ms in four training sources; stationary_0 gains a single nominal pass 5 ms earlier but fails its earlier triplet. near_slow 702 pass / 706 fail / archived 707 pass excludes a single monotonic temporal threshold. All new starts are already recentering; no certified continuous region is claimed. Complete 0.1 ms state/actuator histories, commands, 20 replays, source manifest and offline scorer/actor verification are preserved; zero PPO and no additional verification rollout. Phase 4/Fixed A unchanged. The recommended cross-track sensitivity study was subsequently authorized and completed as R5.1 above, without repeating R5. [Results, timing, mechanics and limits](ASM2_R5_BACKWARD_FRONTIER.md).

**R4 complete (2026-11-13), Step 1 No-Go; conditional PPO NOT_RUN.** One full-state potential was tested against 120 archived Fixed A continuations (58 qualifications). It recognizes success-like states but lacks validated early directional guidance: only 2/5 saved-motion checks pass, with 0.435–0.730 s potential decreases; historical witness action limits are explicitly separated. Its plane-clearance mask falsely excludes 15 known successful starts, whose finite detailed-hull gaps are 1.992–2.156 mm. Ungated source-excluded AUC is 0.984 versus 0.726 with the mask. Three unit tests, an analytic distance check and exact saved-score reproduction pass. Zero new rollouts or PPO transitions; no held-out evaluation or new checkpoint. Preserve Phase 4/Fixed A and frozen first-contact scoring. Its recommended bounded native-only backward expansion was subsequently authorized and completed as R5 above. R4 remains rejected, with no PPO. [Evidence, limits and reproducible implementation](ASM2_R4_BRAKING_POTENTIAL.md).

**R3 complete (2026-11-13), skill-bridging No-Go.** One exploration-scale-only matched PPO study completed 196,608 transitions per arm. Final native earlier contacts 0/36 each, robust 0/12; B reduced exits 33→0 but produced timeouts/footprint misses. Both retained 35/36 braking and 30/30 terminal. Fresh A/B/Fixed A: earlier 0/36 each, separate-source rotation 1/18 each, braking 15/18 / 16/18 / 16/18, terminal 18/18 each, reached handovers 0/4 each. Three rare B training successes did not consolidate into autonomous bridging. Preserve Phase 4 and Fixed A. Budget closed; its recommended potential follow-up was subsequently authorized and completed at the R4 Step-1 gate above, without PPO. [Evidence and analysis](ASM2_R3_SKILL_BRIDGING.md).

**Phase G3.1.2 complete (2026-11-13 Asia/Shanghai): both exact G2 contact tails develop mechanically loaded forbidden body contact.** Two 10 ms cases / 200 native steps; archived first contact reproduced bit for bit. Body load appears in the second [0.1,0.2] ms solver interval, with 2.958/2.871 mN·s body normal impulse. Front loading is 6.4 ms each; rear/bilateral loading zero. Original native verdicts and G3.1.1 contact order stand. Final 1.9/2.0 ms contact-free spans are short separation observations, not recovery qualification. Three focused tests and saved-data accounting pass; no new controller, training, physics/scorer/geometry changes, remote work or Git publication. **One proposed next experiment:** one preregistered actuator-feasible pre-contact terminal-release candidate with existing feedback, compared against saved baseline; NOT_RUN. Phase 4 / Fixed A and prior evidence remain protected. [Report and reproducible evidence](ASM2_G312_CONTACT_TAIL.md).

**Phase G3.1.1 complete (2026-11-12 America/New_York): continuous designated-first order SUPPORTED for saved-motion reconstruction.** All 27 cases retain designated-first ordering; seven native co-contact rejections have a 64.485–80.983 µs front lead and remain rejected under the unchanged engineering contract. Finite-hull checks exclude earlier intersections in both saved G2 prefixes. Censored body-occurrence intervals and physical/sensor uncertainty remain explicit. Ten focused tests pass; zero new rollouts, training or repeated alignment/impulse experiments. Original evidence and 15,597 protected pins are unchanged. The 3.83% projection statistic does not establish incorrect legal geometry; official impact qualification remains unknown. Its proposed two-case 10 ms contact-tail study was subsequently authorized and completed as G3.1.2 above. Phase 4 remains the learned baseline; Fixed A the continuation reference. [Results, provenance and limits](ASM2_G311_CONTACT_ORDER.md). Prior G3.1 literature/actuator audit remains [archived evidence](ASM2_G31_AUDIT.md); read-only 5060 integration (2026-11-13 Asia/Shanghai) now verifies historical source/artifacts: all 15 legal patch hashes match and 25 finite base hulls agree within 5.604 nm. Remote proxy body hulls and loaded-contact scorer differ, so no results are pooled. Seven existing force windows and R1 robustness findings are reused; no completed experiments are repeated. The exact G2 detailed-model tails are now documented in G3.1.2; their outcomes remain separate from remote proxy results. [Cross-workspace provenance](../runs/asm2-g311/cross-workspace-evidence.json).


The regional PPO-update weighting pilot completed 98,304 transitions per arm. B qualified 2/36 earlier trials versus A 0/36, both from one already-recentering anchor; both retained 0/12 fully robust anchors and 33/36 earlier exits. Braking was A/B 34/36 versus 32/36, terminal 30/30 each. Scientific and baseline promotion decisions are No-Go. Keep Phase 4 and Fixed A; no larger campaign is authorized. [Results and limits](../plans/plan_new_airframe_phase0_01.md#21-regional-ppo-update-balancing)

The instrumented Phase R2 pilot completed exactly 98,304 transitions per arm. A and B qualified 0/36 reserved earlier trials each. B entered the transition/braking descriptor on 3/36, exited on 36/36, and retained 19/36 braking and 30/30 terminal. Both scientific and baseline promotion decisions are No-Go. Earlier resets consumed 56.52% of B's transitions with zero qualified training episodes. Keep Phase 4 and Fixed A. [Matched results and regional learning evidence](../plans/plan_new_airframe_phase0_01.md#20-instrumented-phase-r2-earlier-reset-pilot)

The actuator-memory diagnostic is closed with Decision B. A second development anchor showed a qualification split under a filtered-thrust contrast, with the same locally viable actions and no preference reversals. The tested controls leave decision-relevant aliasing unresolved. [Final observability probe](../plans/plan_new_airframe_phase0_01.md#19-phase-r15-m2-final-observability-probe)

The earlier archived Phase R2 comparison completed at 98,304 transitions per arm. Its intervention qualified 0/12 robust bridge anchors, tied the control, and exited the flight envelope in 33/36 reserved trials. Braking and terminal retention passed in that archived run. Its evidence remains distinct from the instrumented pilot.

Phase R localized the sampled PPO boundary to rotation before the transition into braking. Both learned controllers failed all 72 development handover, early and mid-rotation trials by envelope exit. The capped deterministic reference qualified 37 of the same 72. Fixed A improved the transition, and the full-handover gate remains unmet.

The fixed/adaptive curriculum comparison completed at 299,008 transitions per arm. On final perturbations, fixed and adaptive qualified 1/18 and 0/18 rotation starts, and each qualified 0/4 reached handovers. Fixed improved historical braking to 33/36 from Phase 4’s 28/36. Phase 4 remains the verified reference, and the full-handover gate remains unmet. [Resumable workflow](DESIGN.md#74-resumable-fixed-and-adaptive-comparison)

**Status:** The Phase 5 outcome-driven temporal-credit trial used 294,912 transitions and 61.51 learning minutes. Its development-selected actor qualified 0/36 held-out rotations and 0/4 reached handovers, with 29/36 old and 15/18 separate braking regression. Phase 4 remains the verified baseline. One candidate completed; no second candidate or architecture change was launched. [Outcome-driven evidence](../plans/plan_new_airframe_phase0_01.md#15-outcome-driven-learning-campaign)

**Goal Hybrid closure.** Keep Phase 4 Direct-CTBR PPO `checkpoint-186368.pt` as the verified learned baseline. Goal Hybrid showed local tracking and some qualified braking. The tested transitions did not establish a reliable path from rotation to qualified contact. Preserve matched resets, including realized rotor thrust and previous motor targets, and use the frozen native MuJoCo scorer for signed first touch when comparing approaches.

The passive checker passed its held-command gate in 402 comparisons, with 84 correct warnings, 190 correct safe labels, 128 unknowns, and zero false-safe or false-unsafe labels. The separate replay comparison with changing commands produced seven false-safe labels at the 200 ms horizon. The checker exceeded the 5 ms control period, with 26.1 ms p95 even at the 50 ms horizon on the development host. Its supported use is offline diagnosis. Live shielding and readiness for Goal-conditioned RL remain unverified, and the results do not establish inherent PPO superiority.

Objective:
Expand the learned terminal skill backward through rotation/translation states to the validated navigation handover set.

Work:
- Witness-based reverse curriculum.
- Overlapping reset distributions.
- Held-out boundary and unseen handover tests.
- Retention checks on mastered regions.
- Stationary-hover extension as a separate stress/generalization evaluation.

Deliverable:
A policy that autonomously controls the complete agile manoeuvre from the validated handover distribution.

Exit gate:
- Independent full-handover evaluation passes pre-registered criteria.
- No teacher action or full reference trajectory is used by the deployed actor.
- Navigation can demonstrably deliver the tested handover distribution.
- Basic-flight and contact capabilities are reported separately.

### M6 — Vision-Based Adaptation

**Status:** `NOT_STARTED`

Objective:
Replace privileged ground-truth state with realistic perception/localization outputs.

Work:
- State-policy to estimated-state adaptation.
- Observation noise, age, latency and dropout.
- Target visibility / camera FOV loss.
- Temporal encoding or recurrent policy if needed.
- VIO confidence and validity handling.

Deliverable:
Vision/estimated-state agile policy and robustness evaluation.

Exit gate:
- Frozen task remains unchanged.
- Policy passes independent evaluation using realistic observation pipeline.
- Failure under degraded perception is classified and reported.

### M7 — Sim-to-Real Flight / Contact Readiness

**Status:** `NOT_STARTED`

Objective:
Establish controlled deployment readiness on the new aircraft.

Work:
- System identification and selective domain randomization.
- PX4 CTBR interface validation.
- Policy inference timing.
- Safety supervisor and failsafe.
- SITL/HITL.
- Controlled free-flight tests before any contact experiment.

Deliverable:
Sim-to-real readiness dossier and separately authorized test procedure.

Exit gate:
- No automatic flight authorization.
- Each real-hardware test requires explicit user approval and safety review.
- Free-flight validation precedes any physical contact testing.

### M8 — Agile Contact + Recovery Integration

**Status:** `NOT_STARTED`

Objective:
Connect contact event handling to a separate recovery controller/policy.

Work:
- Impact detection.
- Post-contact state capture.
- Attitude/rate stabilization.
- VIO reinitialization / estimator confidence.
- Safe return to PX4 normal control.
- Mission-level reacquisition.

Deliverable:
Integrated contact-and-recovery state machine and evidence.

Exit gate:
- State continuity across contact and recovery.
- Recovery success measured separately from contact success.
- Safe control handover demonstrated in simulation before hardware tests.

## 4. Parallel Recovery Research Branch

The following may be researched independently of M4–M6, but integration is deferred:

- Classical quaternion-based recovery.
- Reachability-based recovery switching.
- Proprioceptive recurrent reflex policies (e.g. REFLEX-inspired).
- Impact-resilient mechanical design.
- VIO reinitialization and PX4 handover.

Do not block the main contact research on completion of this branch.

## 5. Current Active Priority

**2026-11-18：研究計劃對齊及 N1/R9-A 授權的 CPU-only Consumer／Protocol 工作已完成；整體 readiness 仍 BLOCKED。** 繼續使用現有 `plan_new_airframe_phase0_01.md` 的 §0／§34，不另開 active plan。該計劃為本地工作入口；正式需求、設計與 gates 已自足記錄於 SPEC／DESIGN／本 ROADMAP，不依賴個人附件或未跟蹤 PLAN 才能理解。若收到指向 5060 Non-RL component 的工作要求，先停止並提醒使用者，不跨 host 執行。

**本輪交付：釘選發布子集，完成最小 consumer、離線 replay 與 protocol tests，明列剩餘 blockers 後停止。** 80 個發布檔案已置於 `upstream/rm27_g0/9ea8051/`，原始 producer receipts、逐檔接收方 SHA-256 與來源差異存於 `.agents/runs/n1-r9-a/acquisition.json`；發布 commit 不包含 standalone checksum/test receipt release 檔，不以接收方產物補稱 producer 正式發布。最終本地測試與評估準備狀態以同目錄 `results.json`、`replay.jsonl` 和 `fixed_handover_protocol.json` 為準。

**下一項依賴：正式發布可 admission 的完整 G0 recorded scenario package 並完成 RL 兼容性核對。** 當前發布只有 3 筆 diagnostic records、單一 dev family、0 admitted cases；另有 receipt 描述的 3 筆 `RECORDED_SIM_ESTIMATED` closure 為 `published=false`，不匯入本地 dataset、不當作已發布成果。需補足可追溯的 runtime observation/history、模型／scorer／時序相容性及 Direct-CTBR authority 決策；單一 dev family 不構成獨立 held-out distribution。本輪不為此重做導航、幾何或 actuator-memory 實驗，不修改 frozen contract。R9-B 保持 BLOCKED／未選方法／無訓練預算。

**後續本地差分已完成（PLAN §34.6）：** 沿A分支保留原發布包與55項receipt，沒有新code／tests／simulation。已核對G0 D1 XML在本地artifact的byte identity、本地detailed模型／actuator／target axes／scorer與時間身份，以及七項scenario資格。精確差分見同批 `compatibility.json`；不把XML相同推為full-plant相容，不把3筆BLOCKED寫成任務FAIL。下一個最小工作是取得經批准且正式發布的一筆完整dev estimated snapshot／profile及registration，先做限定離線projection資格核對；獨立train/held-out是後續泛化／學習的另一項依賴，不能由單family拆出。本輪不輪詢或自取新包，R9-B維持UNSELECTED／NOT_AUTHORIZED。

以下 R8 完成狀態為歷史證據；其「下一建議」已被後續歷史 R9/R9.1 結果及 §2.1 取代，不是現行執行任務。

**R8 COMPLETE — scientific/promotion No-Go.** Demonstration BC plus ordinary PPO and matched control each completed393,216 transitions. Final earlier0/36 and braking23/36 both, terminal30/30 both. B's transient3/36 belongs to one previously favorable anchor and disappears. BC and PPO qualify0/3 demonstrated starts; fresh earlier0/36 all. Preserve Phase4 and Fixed A; no additional training is authorized. [Evidence, checkpoints and one unlaunched recommendation](ASM2_R8_DEMONSTRATION_LEARNING.md).

**The Phase 4 braking promotion gate is verified.** Rotation and moving handover remain open. Actuator characterization remains a dependency of plant freeze.

No new contact PPO/TD3 training until:
- New airframe model is defined.
- Full-underside contact geometry is defined.
- Independent contact evaluator is validated.
- Training and evaluation distributions are specified.

## 6. Historical Work — Retained, Not Reopened by Default

| Historical asset/result | Disposition |
|---|---|
| SimpleFlight Direct-CTBR flight baseline | Retain as reference |
| RM27 old Ver2 Hover/Track checkpoints | Retain; validate before transfer |
| Old sphere-contact PPO checkpoints | Retain as legacy research |
| 20/20 moving-frontier terminal contact | Retain as terminal-skill evidence for old task |
| 0/19 stationary contact and 0/57 handover | Retain as failure evidence for old task |
| Deterministic old-model witness | Retain as old-model feasibility evidence |
| Old MuJoCo scene/scorer | Freeze and preserve |
| Old Isaac PGS + 1 µm contact tests | Preserve; not assumed valid for new shell |

## 7. Roadmap Update Rules

When a milestone changes:
- Update status and date.
- Link evidence and artifacts.
- Record unmet exit criteria.
- Record changes to dependencies.
- Do not mark PASS based solely on implementation completion.
- If scope/acceptance changes, update SPEC first and record the reason.

### Research-to-Code Handoff v2

**Historical snapshot — superseded by the completed R9/R9.1 results and current §2.1.** 下列提案、awaiting-approval、running 字樣僅記錄當時進度，均不代表有效授權。

Completed source ingestion and R8 saved-evidence verification. R9 decaying demonstration likelihood is an isolated, CPU-tested candidate only; no new PPO/native budget authorized. See [read receipt and concrete proposal](CODEX_READ_RECEIPT.md). R8 remains closed Scientific No-Go.

### Historical R9 controlled demonstration regularization — preflight complete

CPU/offline preflight PASS; zero new PPO environment transitions or native rollouts. The fixed .01 linear-decay mechanism retains archived R8 B as control, original selection/retention gates and a maximum393216/831 proposed dose. Training awaits explicit `APPROVE_R9_393216`; R8 remains Scientific No-Go. [Specification](R9_EXPERIMENT_SPEC.md), [preflight evidence and limits](R9_PREFLIGHT_REPORT.md), [comparison plan](R9_COMPARISON_PLAN.md). Actual simulator integration and efficacy remain NOT_RUN.

Historical execution authorization received: `APPROVE_R9_393216`. The isolated single arm was active within393216 transitions /831 native cap at that time; it is now complete and closed as recorded below. Preflight specification remains immutable; execution approval and wiring pins are in the R9 run directory. R8 remains archived No-Go. No extension or commit/push authorization.


### R9 completed — Scientific / Promotion No-Go

The approved run completed393216 transitions and726/831 native rollouts (including68 exact reproductions); archived R8 B was not rerun. Final earlier0/36, braking25/36, terminal30/30; fresh earlier0/36 and independent rotation0/18. The two transient qualifications at196608 came from the same existing favorable anchor and did not persist. Improved late action imitation did not produce skill bridging; no checkpoint passes the braking gate. Preserve Phase4 and Fixed A. All full histories, policy commands, checkpoint states and original/amended pins verify. [Final report](ASM2_R9_DEMONSTRATION_REGULARIZED_PPO.md). Prior preflight/running entries above are historical and superseded. Authorization is closed; no automatic use of the remaining105 native allowance.

One next proposal only, NOT_LAUNCHED: a bounded corrective-demonstration pilot using policy-visited states at the three existing demonstrated starts, evaluated for autonomous qualified contact and braking retention before another PPO campaign. Separate manifest and approval required.


### Historical R9.1 corrective-demonstration pilot — proposal snapshot

[Concrete specification](R9_1_CORRECTIVE_DEMONSTRATION_PILOT.md) defines9 policy-visited correction states, qualified-label acceptance, matched equal-compute supervised A/B refits and unchanged retention thresholds. Proposed ceiling: zero PPO transitions,1152 optimizer forward integrations,576 supervised updates and189 native validation/evaluation/replay episodes. Current authorization is documentation only: implementation, fitting and simulator calls NOT_RUN. No broad skill-bridging or promotion claim from this local two-source pilot. R9 is complete No-Go; no prior unused budget carries forward.


### R9.1 completed — corrective-label gate No-Go

[Results](ASM2_R9_1_CORRECTIVE_DEMONSTRATION_RESULTS.md):0/9 qualified corrections,6 forbidden-body prefix contacts and3 safe transfers followed by Fixed A forbidden-body touch.1152 optimizer integrations and9 native validations; no supervised fitting/PPO or subsequent evaluations. Original label gate failed, so conditional training/evaluation allocation is closed. Saved-data verification passes; historical baselines intact. This rejects the bounded label generator, not learnability or global reachability. One unlaunched recommendation: a separately bounded optimizer-convergence test, changing only the call allowance on the same9 states. Prior proposal status above is superseded.
