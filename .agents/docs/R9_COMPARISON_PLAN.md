# R9 comparison and diagnostic plan

Status: preregistered, **no new native evaluation authorized or consumed**. [Experiment specification](R9_EXPERIMENT_SPEC.md) governs intervention, selection and gates.

## Matched historical control

| Additional transitions | Archived R8 B earlier /36 | Robust /12 | Braking /36 | Terminal /30 |
|---:|---:|---:|---:|---:|
|0 (BC)|0|0|33|30|
|98304|0|0|29|30|
|196608|0|0|19|30|
|294912|3|1|23|30|
|393216|0|0|23|30|

All three294912 contacts belong to `dev_lateral_positive:730`. This is one anchor, not three independently acquired skills. BC demonstrated starts0/3, all forbidden. R9 shares that zero-dose baseline; do not rerun it. Retain R8 raw outcomes as the primary control, using existing native scorer and suite identities without moving thresholds. A common seed/historical control does not remove all runtime confounds or replace multi-seed evidence.

## Future native budget, maximum831 including replay reproductions

- Four checkpoints × (138 existing development: earlier36, rotation36, braking36, terminal30 + three demonstrated starts) =564.
- Selected and final fresh suites, at most94 each =188: earlier36, rotation18, braking18, terminal18, reached handovers4. If hashes coincide, reuse one evaluation; do not spend savings on new cases.
- Up to79 representative replay reproductions: up to14 per checkpoint (three demonstrations, six earlier, five braking) =56; at most11 selected-fresh and12 final-fresh =23. If cohorts duplicate, reuse evidence. Unused slots expire.

Replay ordering is deterministic: first newly qualified earlier anchor representatives (at most one case per anchor before filling perturbations), then unrepresented native rejection reasons, then lexicographic remaining IDs. Demonstrated-start replays use all three fixed IDs. Braking chooses qualification/regression representatives by identity, not visually favorable trajectories. Every replay consumes the shared ledger and must reproduce its saved native verdict; discrepancies block promotion. They are repeats, not added independent successes. No substitutes, expanded evaluation sources or new static/contact-mechanics campaigns.

Use the frozen R8 development and heldout manifests; never heldout for λ, stopping decisions or checkpoint selection. Seal selected/final hashes first. These are existing historically evaluated fresh cases, not newly unseen globally blind data. Keep fresh offsets within development families separate from independent-source rotation/braking/terminal/handover cohorts. R8 selected B equals final B: final-to-final fresh comparisons are dose matched; any earlier selected R9 versus final R8 B fresh comparison must be labeled dose mismatched, secondary. Do not launch extra R8 evaluations to fill this gap.

## Four separate evidence layers

1. **Action imitation:** weighted raw Gaussian NLL, deterministic raw and tanh action error, physical collective/rate errors, per-action log_std; category/source/trajectory breakdown at every saved checkpoint. Never report density as success probability. Record coefficient and mean pre-clipping PPO/auxiliary gradient norms each update through the isolated adapter; retain original PPO/critic/return metrics. No adaptive coefficient response.
2. **Behavioral retention:** for replayed demonstrated starts, plot autonomous versus archived trajectories at common elapsed times until either terminates. Report target-frame position/velocity, attitude geodesic, body rates, actual/held rotor thrust, and CTBR deviation in their own units. No nearest-path distance score alone and no demonstration controller in rollout. For earlier starts lacking a matching demonstration, compare R9/R8 recentering, standoff, velocity, braking-to-contact chronology; do not force a fabricated trajectory match. Archive complete states/actuator records and inference commands.
3. **Native contact:** unchanged first-contact identity, time, position/target coordinates, signed distance, designated/forbidden and simultaneous pairs, alignment, contact-point normal/tangential speeds, scorer rejection. Report per-reason counts: each forbidden code/geometry combination, NO_SIGNED_TOUCH, FLIGHT_VOLUME_EXIT, SPEED_LIMIT and contact-point-speed codes separately. Preserve raw native enum plus secondary flags; never merge overlapping rejection conditions into an invented mutually exclusive success denominator. Geometry intersection and loaded contact remain separate; no official impact-score claim or new G3.1.2 tails.
4. **Generalization and retention:** source-family and exact source/frame anchor tables with nominal/minus/plus outcomes. Robust anchor=3/3 variants. Demonstrations:3 starts/2 families separately. Report braking/terminal and their envelope failures, selected versus final persistence, fresh cohorts independently. Plot whether better likelihood/coherent recentering merely increases repeated forbidden first touch.

## Falsifiable outcomes

If NLL improves but qualified anchors do not, imitation is not skill bridging. If touch-reaching improves but forbidden-first contacts persist, regularization is preserving insufficient contact precision. If development gains disappear at final or retention fails, reject adoption. If only demonstrated sources benefit, report restricted support, not broad generalization. Source coverage, variance contraction, PPO/regularizer gradient conflict and distribution shift remain hypotheses to interpret using these saved records, not permission for new sweeps.
