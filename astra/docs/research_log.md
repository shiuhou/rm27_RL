# Research journal

## 2026-09-05 — Asset audit and reproduction

**Question:** Which existing evidence can support Astra?

**Method:** Inspect local outer/nested repositories; clone the corrected upstream
MuJoCo project at b9e61ba; run its complete tests and M3 benchmark.

**Result:** 134 tests passed (89.73 s); M3 reports overall pass across 11 cases.
Local PX4/Gazebo reports use provisional 170 g/3:1 dynamics, simulated GNSS,
and a camera visual mesh without a camera sensor. Their referenced temporary
logs are absent. Propulsion configs contain zero motors and zero thrust curves.

**Interpretation:** Reuse the upstream estimator/controller tests as regression
assets, but neither repository establishes visual contact or real propulsion.

**Decision:** Keep Astra separate. Do not copy the milestone order or claim
hardware credibility from nominal simulator numbers.

## 2026-09-05 — Development, review and numerical sensitivity

**Question:** Can simple pursuit work with a fixed camera, and does prediction help?

**Hypothesis:** Visibility and delayed observations may dominate nominal thrust.

**Method:** Build a vectorized thrust-vector surrogate and geometric camera;
run 200-case development sweeps. Independent review inspected truth routing,
geometry and comparison fairness.

**Result:** Review caught off-axis clipped spheres being accepted and a comparison
that mixed latency compensation with interception prediction. Both were fixed;
compensated pursuit was added. Mass/supply variation now use independent draws.
An early convergence check changed guidance frequency with integration dt;
guidance was fixed at 100 Hz. At dt 5→2.5 ms with noise-free 25 Hz sensing,
outcome changes were 0/200 pursuit, 1/200 compensated pursuit, 0/200 lead;
p95 per-episode miss differences were about 1.44 cm. A further halving reduced
p95 differences to about 0.72 cm.

**Interpretation:** Small method differences require caution; large visibility
effects deserve the first held-out evaluation. The original two-method development
results are superseded, not confirmatory evidence.

**Decision:** Freeze three baselines and protocol; retain negative results and
run evaluation without trajectory-specific tuning.
