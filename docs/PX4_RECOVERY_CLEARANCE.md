# Guard contact and recovery clearance

This is independent simulation geometry scoring. It does not create physical
collision for visual field meshes, certify closed CAD solids, or change the
mission's recovery thresholds.

The current guard contact point is body FLU `[-0.06, 0.06, -0.018]` m.
Only the configured 90×55 mm target patch and 10 mm backing allowance admit
guard intersections. The checker clips that patch from mesh triangles and
tests every remaining fragment. Each continuous translation/rotation cell
uses conservative movement bounds. Unknown cells or any forbidden overlap
fail; capsule subdivision preserves the exact capsule union.

## Completed development evidence

`px4-continuous-recovery-corridor-development-03` passes all six piecewise
regions after 49,970 adaptive cells. The contact region permits tip normal
distance −2 to20 mm and tangent displacement ±15 mm. Every region keeps body
roll/pitch ±7° and yaw42–48°; combined tilt is at most9.89°. The retreat corridor
expands to normal separation0.6 m and tangent displacement±0.12 m. This is
an explicitly bounded geometric family, not an unrestricted recovery claim.

The second startup-observer development flight is outside the original70 mm
tangent boundary at0.1–0.2 m separation for126 recorded samples, reaching
70.65 mm. That failed membership is retained. V4 expands this separated
region to±80 mm and independently certifies it; the contact region and
angular bounds stay unchanged. V4 reuses prior certified subsets only after
checking scene/assets, geometric configuration, saved dependency hashes and
exact geometry function bytes. The changed surface-recording gzip reader
does not alter those geometry functions.

Both startup-observer development flights pass membership in V4: each has
3,100 recorded consecutive1 ms recovery samples spanning3.099 s, with no
sample outside. Their separate every-recorded-pose local surface audits also
pass. The source inputs and failed prior attempts remain in their respective
artifact directories. Reserved acceptance flights must independently satisfy
membership; successful nominal flight alone cannot establish S1 completion.

The reserved nominal V3 set now passes20/20 native-step membership in
`px4-image-nominal-v3-native-membership-01`. Each run contains3100 consecutive
1 ms samples spanning3.099 s; no sample lies outside V4. The same20 runs
pass their independent recorded local surface and physical recovery gates.
This establishes the bounded front-contact/recovery family for those nominal
tasks; arbitrary grazing and disturbed trajectories remain separate.

The same tagged scene passes current-aircraft front/back/grazing/non-target
motor-off probes in `px4-guard-tip-contact-classification-01`. Its finest
0.125 ms classifications and momentum checks pass; the coarse1 ms grazing
momentum check fails. The actual developed front impact separately converges
at the native1 ms step against0.5 ms: impulse2.118%, peak1.097%
(`px4-guard-tip-convergence-development-01`). This does not qualify arbitrary
grazing flight or aggressive contact strategies.

## Commands

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.recovery_corridor \
  artifacts/px4-dock-tags-05/scene.xml \
  --config configs/px4_recovery_corridor_dev_v4.json \
  --output artifacts/px4-clearance-new

/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.recovery_membership \
  artifacts/px4-image-mission-completed \
  --config configs/px4_recovery_corridor_dev_v4.json \
  --output artifacts/px4-membership-new
```

Fresh output directories are required. Read the actual summaries. The
certificate depends on the recorded local source/assets; a moved checkout
must regenerate references or rerun the independent clearance computation.
