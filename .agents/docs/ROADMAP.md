# RM27 Agile Flight RL — ROADMAP

**Version:** 1.0  
**Date:** 2026-11-10  
**Status:** ASM2 Phase 3 nominal simulated handovers verified. Terminal simulator conformance remains open.

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
| M4 | Terminal Agile Contact Skill | Phase 4R.2 pilot missed braking gate | Verify learned advantage on braking starts |
| M5 | Full Handover-Distribution Agile Policy | NOT_STARTED | M4 |
| M6 | Vision-Based Adaptation | NOT_STARTED | M5, perception/VIO interfaces |
| M7 | Sim-to-Real Flight / Contact Readiness | NOT_STARTED | M1–M6, safety authorization |
| M8 | Agile Contact + Recovery Integration | NOT_STARTED | M5, separate recovery branch |

## 3. Milestone Details

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

**Status:** Phase 4R.2 restored the disputed reference body hull in an isolated proxy overlay and matched the frozen native labels in both layouts. The existing action map qualified all deterministic development braking starts. The learned candidate stopped at 11/36 development qualifications and trailed Track and constant commands on held-out braking. The unchanged native scorer remains the acceptance reference. [Phase 4 inputs](DESIGN.md#73-phase-4-terminal-inputs)

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

**Status:** `NOT_STARTED`

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

**P0 is independently verified braking improvement on the Phase 3 reset bank.** Actuator characterization remains a dependency of plant freeze.

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
