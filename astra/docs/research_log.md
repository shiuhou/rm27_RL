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

## 2026-09-05 — Held-out terminal campaign

**Question:** What dominates nominal terminal contact and does lead materially help?

**Method:** Frozen revision 99f6b44, seed 27991, 1,000 trajectories, 52 conditions,
three methods; 156,000 trials with paired bootstrap comparisons.

**Result:** Nominal pursuit/compensated pursuit/lead: 81.3/92.1/96.2%. Lead versus
compensated: +4.1 points (95% paired interval +2.5 to +5.8). Narrowing FOV 90→60°
reduces lead to 81.5%; 800 ms occlusion reduces it to 61.6%. Raising T/W 2→3
does not help in this 45°-tilt envelope. Raw data and source snapshot retained.

**Interpretation:** A small classical predictor earns a place in the prototype.
Visibility/reacquisition is more urgent than additional optimization complexity.

**Decision:** Keep gains frozen; challenge acquisition and the camera abstraction.

## 2026-09-05 — RGB perception and rendered closed-loop cross-check

**Question:** Does the geometric-camera assumption survive actual image processing?

**Method:** MuJoCo sphere rendering → color segmentation → silhouette-cone fit;
720 calibration evaluations. Then 432 paired closed-loop trials on new seed
28111 at 90/120° FOV, with geometric, unblurred-pixel and blurred-pixel sensing.

**Result:** At 120°/11 px blur, median range bias −19.1%, p95 absolute error
30.8% among detections, 68/80 detected. At 120°, lead contact drops 24→21/24
under blur. Other methods also lose 3/24. No gains changed.

**Interpretation:** Random angular noise alone omits systematic blur bias and
abstention. It would be wrong to turn the geometric success rate into a claim
of visual reliability. The small rendered sample is a mechanism check.

**Decision:** Measure actual camera exposure, blur and size/bearing errors before
selecting FOV or claiming transfer. Do not treat desktop detector time as MaixCAM
latency. The earlier progress comment about larger errors was too broad: the
important new finding is structured bias/abstention, not simply bigger noise.

## 2026-09-05 — Initial acquisition confounds latency sensitivity

**Question:** Why did corrected guidance improve with delay at narrow FOV?

**Method:** New seed 28201, 1,000 paired targets × hold 0/500 ms × delay 0/150 ms
× two methods = 8,000 trials. All gains and target distributions fixed.

**Result:** Without common hold, lead contact rises 80.4→89.1% with delay. With
500 ms common hold, it falls 95.4→92.7%; paired interval −4.0 to −1.5 points.

**Interpretation:** Initial target observation and maneuver timing affect the
latency curve. The data do not support intentionally adding pipeline delay.

**Decision:** Include acquisition/readiness as an explicit mission behavior in
the next full-mission study. Preserve both the original and controlled results.

## 2026-09-05 — Evidence preservation

**Verification:** Astra's 20 tests pass, including numerical convergence,
runtime counterfactual truth isolation, pixel-detector behavior and source
snapshot integrity. A package wheel builds. Replaying the nominal 1,000-case
lead condition with the current default geometric backend reproduces every
saved episode record exactly. All 164,432 retained campaign trials have unique
condition/method/seed/episode keys; compressed records and source hashes verify.

Initial independent code review identified and helped close the visibility and
comparison issues. The subsequent rendered/perception extension was checked
locally; a second independent review could not run because the reviewer service
reported a usage limit. This does not represent completed external validation.

## 2026-09-05 — Assembled prototype changes the next experiment

**Question:** What is the smallest real campaign that can discriminate the current
architecture choices, now that a 1103 9600KV / MicoAir743v2-AIO-35A / MTF-02P
prototype and 1.8-inch/2.0-inch propellers are available?

**Hypothesis:** Field-dependent sensing and powered sensor coupling can invalidate
the simulation ranking before extra guidance complexity becomes useful. A larger
propeller may improve steady power while worsening vibration or image quality.

**Method:** Specify four gated experiments: surveyed props-off flow/ToF over field
materials; real camera acquisition/blur/timing on field fixtures; paired guarded
thrust/power steps; six paired and three field-route low-energy flights with an
independent pose reference. Freeze settings, preserve aborts, fit repetitions 1–2
and withhold repetition 3. Publish exact signals, counts and metadata template.

**Result:** Protocol and logging contract prepared; **no real measurements collected**.
The 9600KV prototype supersedes the earlier 9500KV assumption. Actual battery,
mass, firmware, proven modes and camera remain unknown. Local ArduPilot source
and official logging/sensor documentation were checked; local source is not the
installed firmware. Published FC specifications reveal an unconfirmed yaw-source
dependency. FC logs alone cannot establish camera latency, absolute drift or
fast propulsion response.

**Interpretation:** A generic indoor hover cannot answer competition-field sensing
questions. Independent reference, real lighting/materials and image timestamps
provide greater discrimination than more unreferenced flight time. The proposed
1.4× thrust screen must not be mislabeled validation of the 2× simulation model.

**Decision:** Execute [the real campaign](real_experiments.md) with a human operator
and measured powered-test limits. Keep visual guidance in shadow mode. Recalibrate
in a new evidence version and challenge the frozen guidance comparisons without
changing their thresholds. Defer high-speed/contact tests and high-fidelity
dynamics expansion until measured residuals identify the need.
