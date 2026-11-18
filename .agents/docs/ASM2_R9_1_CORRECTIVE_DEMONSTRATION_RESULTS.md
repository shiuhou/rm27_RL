# R9.1 — Corrective-demonstration pilot results

**COMPLETE: corrective-label gate No-Go; supervised A/B NOT_RUN.** The user authorized execution after the [proposal](R9_1_CORRECTIVE_DEMONSTRATION_PILOT.md). Exactly9 declared policy-visited states were optimized and validated, with **0/9 native-qualified corrective trajectories**. The required6/9 plus coverage of all three starts was not met. No correction labels were admitted, no supervised fitting or PPO occurred, and subsequent evaluation/reproduction allocations were not spent. R9 remains Scientific No-Go; Phase4 and Fixed A are preserved.

## What actually ran

The isolated implementation is `.agents/runs/asm2-terminal/r9-1/experiment.py`. It reuses the frozen native plant, original CTBR interpolation/allocator/motor dynamics, terminal-set nearest full-state residual and original scorer. Historical implementations were not edited. [Manifest](../runs/asm2-terminal/r9-1/manifest.json) binds exact9 source states,15 prospective local evaluation states, target support, warm controls, source hashes and model metadata. [Preflight](../runs/asm2-terminal/r9-1/preflight.json) passed without physics steps. Source: final R9 records0604/0605/0606; three named starts but only two independent original families.

The predeclared quarter/half/three-quarter times were used without substitution. All9 initial states had positive finite registered-target clearance (300.7–803.5mm), no initial contact constraints, valid actuator rails and no prior signed contact or initial envelope failure. The preceding held targets match the recorded allocator outputs exactly. Warm controls came from the same seed's first lexicographic qualified terminal-set candidate and were cropped/resampled, without tracking loss.

Every solve used its128-call cap, including numerical-Jacobian probes; all9 selected controls were sealed in [selection.json](../runs/asm2-terminal/r9-1/selection.json) before opening native outcomes. A completed safe prefix outranked unsafe alternatives; cost and call order broke ties. No refinement, state substitution or outcome-driven restart occurred.

Archive limitation: R9 physics records contain complete physical/actual-rotor/held-target state but not the full MuJoCo integration vector. These contact-free states have no active constraints; the frozen plant, fixed target, allocator and Actor are autonomous/stateless. Initial native restore regenerates derived quantities with a local zero clock, preserves original elapsed time as metadata and enforces the original remaining2.3s horizon. It does not claim bitwise recovery of an unavailable historical solver vector. Subsequent optimized-prefix→Fixed A transfer uses the same live instance; its complete integration vector is bitwise unchanged. No extra episode time is granted.

## Native outcomes

All nine first-touch manifolds were body-only and rejected as `FORBIDDEN_AIRFRAME_FIRST_TOUCH`. None had a qualified patch contact. A signed first-touch rejection is not a statement about loaded contact, post-contact mechanics or official force-area-time scoring.

| Seed | Sample time s | Safe prefix | Fixed A suffix | First touch, original elapsed s | Qualification |
|---|---:|---|---|---:|---|
|stationary_0-slope_minus|.285|No|NOT_RUN|1.1030|No|
|stationary_0-slope_minus|.570|Yes|Forbidden body|1.1667|No|
|stationary_0-slope_minus|.855|Yes|Forbidden body|1.2633|No|
|stationary_0-slope_plus|.285|No|NOT_RUN|1.0715|No|
|stationary_0-slope_plus|.570|No|NOT_RUN|1.1058|No|
|stationary_0-slope_plus|.855|Yes|Forbidden body|1.2480|No|
|stationary_2-across_plus|.310|No|NOT_RUN|1.1275|No|
|stationary_2-across_plus|.620|No|NOT_RUN|1.0986|No|
|stationary_2-across_plus|.930|No|NOT_RUN|1.1657|No|

Six candidates touch forbidden body before the scheduled transfer; **these are prefix failures, not Fixed A failures**. Three candidates complete safe contact-free prefixes but Fixed A then touches forbidden body. They have no prefix or suffix allocation saturation and no recorded envelope failure. Native safety checks run every0.1ms; finite-body distance is additionally queried every10ms and at transfer. This is discrete native validation, not continuous-time certification.

Safe transfer states remain materially different from their nearest empirically successful support sample:

| State | Position error m | Velocity error m/s | Attitude error deg | Body-rate error rad/s | Max actual rotor error N |
|---|---:|---:|---:|---:|---:|
|stationary_0-minus at.570|.054|.522|9.5|1.597|.028|
|stationary_0-minus at.855|.078|.257|9.8|.499|.016|
|stationary_0-plus at.855|.080|.266|11.6|.106|.019|

Thus “safe prefix” did not mean entry into the observed Fixed A success support. No exact-state match was demanded for a positive label; actual native qualification was the deciding gate. All families have0 accepted trajectories. The [case table](../runs/asm2-terminal/r9-1/outcomes.csv) and raw contacts retain geometry names, velocities and times.

![Bounded search histories](../runs/asm2-terminal/r9-1/correction-search.png)

The residuals improve but remain8.49–14.99 at the selected endpoints. All solves stop at their call cap, not solver convergence. With32 knot parameters and numerical differences,128 integrations permit only a few nonlinear update steps; the plateaus in this plot include Jacobian probes. Consequently the experiment establishes **failure of this bounded correction generator**, not an inability to recover these states and not failure of supervised learning, which was never attempted.

![Optimized prefix and autonomous suffix trajectories](../runs/asm2-terminal/r9-1/correction-trajectories.png)

## Budget and verification

| Work | Actual | Approved cap |
|---|---:|---:|
|Optimizer native forward integrations|1152|1152|
|Final correction native validations|9|9|
|A/B supervised optimizer updates|0|576 conditional|
|A/B policy evaluations|0|162 conditional|
|Unmodified R9 perturbation evaluations|0|12 conditional|
|Physical replay reproductions|0|6 conditional|
|All native validation/evaluation/replay episodes|9|189|
|PPO transitions|0|0|

Optimizer integrations are separately disclosed real physics work:5,911,941 steps (591.1941 simulator-seconds). Final validation used50,701 steps (5.0701 simulator-seconds). Combined simulator reservations1161/1341. The remaining conditional allowance is closed, not permission for replacements or another method. Nine replay files are views of already-recorded validation histories, not extra physical reproductions.

[Saved-data verification](../runs/asm2-terminal/r9-1/verification.json) PASS: all9 restored physical/actuator states exact; selected prefix control-boundary states and final actuators bitwise equal optimizer records; all50 autonomous Fixed A suffix commands exact; allocator and filter evolution checked (maximum filter residual1.11e−16N); transfer integration state unchanged; offline replay of the unchanged scorer reproduces every contact verdict. This verification adds zero simulator steps. The preflight also checks remaining horizon, cropped control duration, bounded interpolation, actual/held rotor sensitivity and quaternion double-cover invariance. Full state/action/actuator arrays and failed search histories are preserved.

Representative saved replays: [early prefix forbidden body](../runs/asm2-terminal/r9-1/cases/stationary_0-slope_minus-02850/replay.npz), [safe transfer then Fixed A rejection](../runs/asm2-terminal/r9-1/cases/stationary_0-slope_minus-05700/replay.npz), [other source prefix failure](../runs/asm2-terminal/r9-1/cases/stationary_2-across_plus-03100/replay.npz). Each directory retains `physics.npz`, `contacts.json`, start reservation and outcome. No historical artifact, geometry, scorer, reward, actor interface or checkpoint was changed; no commit/push or hardware work.

## Decision and one next research action

**No-Go to corrective-data refitting in this pilot.** The concrete blocker is zero validated corrective trajectories. Continuing with failed controls would violate the supervision definition. There is no new evidence about the potential benefit of the proposed A/B learning mechanism, because its prerequisite failed.

**One recommendation, NOT_LAUNCHED:** a separately preregistered, bounded optimizer-convergence test on these same nine states, changing only the forward-call allowance while preserving states,8-knot parameterization, horizons, support, plant and native acceptance. The current finite-difference solves exhaust their budgets while still reducing residuals; testing search adequacy is more justified than spending another learning budget or attributing the failure to unreachable states. Declare its call/native budgets before execution; do not reuse this pilot's unused allocation. No guarantee that additional search will succeed, and no automatic extension is authorized.
