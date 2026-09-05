# Frozen first-campaign protocol

Approved first campaign: compare timing, visibility and vehicle authority before
adding simulator sophistication. All target contact is with benign simulated
competition fixtures. Current numerical values are research assumptions, not
confirmed RM27 rules.

Development seed 27001: 200 trajectories. Evaluation seed 27991: 1,000 held-out
trajectories, 250 each static/crossing/receding/maneuvering. Identical trajectories
and random sensor draws are paired across algorithms and conditions. They are
not 156,000 independent trajectories when reused across 156 experiment cells.

Full factorial: five latencies (0/20/50/80/150 ms), three horizontal FOVs
(60/90/120 deg), three nominal thrust/weight values (1.5/2/3). Seven additional
stress conditions test occlusion, ego drift, size-prior errors, mounting angle
and attitude response. Mass and thrust variation are independent uniform ±10%
draws. The nominal target's constant velocity magnitude is 0–5 m/s; maneuvering
sinusoids add bounded velocity/acceleration, so instantaneous speed may exceed
5 m/s. Target initial range is approximately 2–8 m.

Report all contacts, misses, censoring at 12 seconds, flight-envelope failures,
visibility loss, staleness, tracking error and command effort. Success-rate
intervals are Wilson 95%; paired differences use trajectory bootstrap. These
are descriptive per-condition intervals, without multiple-comparison claims.
Keep primary comparisons explicit; do not pick a winner from a pooled grid.

Gates before evaluation: core math/causality/routing tests pass; off-axis
visibility uses exact frustum-plane checks; compensated pursuit included;
halving dt changes <=1% of outcomes and p95 per-episode miss by <3 cm in the
noise-free 200-case convergence sample with fixed 100 Hz guidance.

No minimum contact success is prescribed: poor performance is a result, not a
reason to weaken cases. Changes after held-out evaluation require a new protocol
and new evaluation seed. Do not infer real contact reliability from this model.
