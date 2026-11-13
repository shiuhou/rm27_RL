# R5.1 — cross-track sensitivity validation

Completed 2026-11-13 Asia/Shanghai. **Exactly ten new native rollouts: nine qualified, one rejected. Five nominal R5 controls were reused without rerunning them. Zero PPO transitions.**

**Conclusion: mixed sensitivity.** Both cross-track signs preserve qualification at four of five anchors. Together with R5, three anchors have successful samples along both target-tangent perturbation directions at the **same complete physical start**. `dev_fast:816` is direction-sensitive: its negative across variant has forbidden body contact in the first native manifold, while nominal, positive across and both prior slope variants qualify. `stationary_0:330` passes the across tests but lacks slope tests at that same frame.

These are useful multidirectional autonomous transition samples, not a certified reachable region or a demonstrated continuous tolerance interval. They do not establish early-rotation skill bridging, and no reward or potential was constructed.

## Frozen matched experiment

The five user-specified nominal states are `dev_center:725`, `dev_fast:816`, `dev_near_slow:702`, `stationary_0:330`, and `stationary_2:346`. Their complete initial arrays were read from the [R5 manifest](../runs/asm2-terminal/r5/manifest.json), checked against the original source traces, and matched to the successful archived R5 outcomes.

The [R5.1 manifest](../runs/asm2-terminal/r5-1/manifest.json), SHA-256 `425f764ffef8145d2dcf9cfdb3f68d8922181b81b010db05ab8e552a633c64a9`, was frozen before execution. It pins all R5 completion artifacts, relevant historical sources, the new adapter/tests, the nominal controls and every new state. There were no adaptive choices, nominal reruns, replacements or extensions.

The registered rotation maps target coordinates into world coordinates; its second column is the target-across tangent, **`[0, 1, 0]` in world coordinates**. For each sign, only these components change:

`p_variant = p_nominal ± 0.0005 * target_across`

`v_variant = v_nominal ± 0.005 * target_across`

Before freezing, back-projection into target coordinates verified exactly the intended `[0, ±0.0005, 0] m` and `[0, ±0.005, 0] m/s` offsets within 10⁻¹⁵ numerical tolerance. Quaternion, body rates, actual rotor thrust and held motor targets are bit-for-bit identical to each nominal state. Tests also exercise an oblique frame, so a world-Y implementation error would not pass merely because this target's across axis happens to align with world Y.

Every variant passed exact restoration, motor-rail and flight-envelope checks, no signed or loaded initial field contact, and finite detailed-geometry separation. The minimum initial aircraft-to-registered-target distance is **20.762375 mm**. Per-case minimum distances, closest collider pairs, full arrays and transform checks are preserved in the manifest. No plane approximation was substituted for finite-target clearance.

Fixed A remains checkpoint SHA-256 `eba759e7eeea93cd9fe06e0f8653d8f12bafc6cfd05a64bcbc11a7a78086fa3f`, with the same portable actor, 45 actor inputs and Direct-CTBR mapping. The frozen MuJoCo 3.12.0 detailed plant runs at 0.1 ms physics and 5 ms control with the unchanged native scorer and 2.3 s maximum horizon. Held source motor targets are restored, then replaced by the autonomous actor's allocation at its first control boundary. No witness action, source phase, clock or trajectory reference enters the actor.

The [adapter](../../src/rm27_asm2/cross_track_validation.py) reuses R5's exact rollout and recorder with a separate output directory; R5 implementation and evidence were not edited. The rejected R4 potential is not used. Only its already-tested finite-geometry distance utility is reused through the existing R5 implementation.

## Matched outcomes

Times below are the native **first signed-contact interval start**, in milliseconds after reset. All decisions finish one 0.1 ms interval later. “Front” and “rear” denote designated contact groups; “body + front” is an unchanged native simultaneous-contact rejection.

| Anchor | R5 nominal, reused | Negative across | Positive across |
| --- | --- | --- | --- |
| dev_center:725 | Qualified; rear; 99.8 ms | Qualified; rear; 99.9 ms | Qualified; rear; 99.8 ms |
| dev_fast:816 | Qualified; front; 435.5 ms | **FORBIDDEN_AIRFRAME_FIRST_TOUCH**; body + front; 435.9 ms | Qualified; front; 435.1 ms |
| dev_near_slow:702 | Qualified; rear; 216.7 ms | Qualified; rear; 217.0 ms | Qualified; rear; 216.5 ms |
| stationary_0:330 | Qualified; front; 436.2 ms | Qualified; front; 436.8 ms | Qualified; front; 435.7 ms |
| stationary_2:346 | Qualified; rear; 315.7 ms | Qualified; rear; 316.0 ms | Qualified; rear; 315.4 ms |

New-trial denominator: **9/10**. Reused nominal controls: **5/5**, reported separately. Both new signs qualify at **4/5 anchors**. There are zero timeout, flight-envelope or allocator-saturation failures. Checking every saved 0.1 ms state also finds no between-control-boundary envelope violation.

![Cross-track outcomes and first-contact locations](../runs/asm2-terminal/r5-1/cross-track-outcomes.png)

The plot's left panel uses **decision times**, 0.1 ms later than the table's first signed-contact times. The right panel displays bounds of native manifold locations, not a force centroid or continuous physical contact area. `stationary_2` positive has rear-patch points on both sides of the target, so its large bounding span does not imply uniformly loaded area. Exact collider pairs and target-frame coordinate bounds for all 15 matched entries are in [analysis.json](../runs/asm2-terminal/r5-1/analysis.json); original per-point positions, normals, velocities, forces and impulses remain in each `contacts.json` and scorer result.

## The one failure and its limits

At `dev_fast:816` negative across, the first signed manifold contains forbidden **`collision_0_550`** and designated **`contact_front_006`**. The body point has signed distance **−0.420 µm**, at target coordinates approximately **[+33.433, +40.894, +0.02089] mm**. Its reported normal force in that interval is **zero**, while several designated front points carry positive normal force.

Thus this is a **native geometric co-contact rejection**, not evidence that the forbidden body is mechanically loaded in the captured interval. It does not establish continuous first-contact order, post-contact safety, or official impact qualification. No tail continuation or geometry-order audit was run, and the scorer remains unchanged.

The failed variant's across positions are about +40.13 to +40.96 mm; the passing nominal is +42.76 to +43.57 mm and passing positive variant +45.39 to +46.18 mm. The failure is not a simple “farther across the target misses the footprint” effect. Full future contact geometry depends on the autonomous trajectory. The experiment isolates the **coupled initial position/velocity offset** from attitude, rates and actuator initial state, but does not separately identify position sensitivity versus velocity sensitivity or a unique downstream mechanism.

![Matched autonomous transverse trajectories](../runs/asm2-terminal/r5-1/cross-track-trajectories.png)

## Does the support have transverse width?

| Anchor | Both across signs pass | Both R5 slope signs pass at same frame | Interpretation |
| --- | --- | --- | --- |
| dev_center:725 | Yes | Yes | Successful samples in two tangent directions |
| dev_fast:816 | No; negative fails | Yes | Direction-dependent qualification at tested offsets |
| dev_near_slow:702 | Yes | Yes | Successful samples in two tangent directions |
| stationary_0:330 | Yes | **Not tested** | Transverse tolerance sampled; two-direction claim unresolved |
| stationary_2:346 | Yes | Yes | Successful samples in two tangent directions |

At four distinct anchors, successful nominal and both transverse endpoint samples span **1 mm initial across position and 10 mm/s across velocity**, with the two quantities varied together. This is positive evidence that success is not confined to the slope-only samples at those anchors. It is **not** proof that every intervening state succeeds, that the endpoints mark the tolerance limits, or that larger offsets are safe.

At three anchors, R5 and R5.1 jointly provide successful samples along two linearly independent perturbation directions in position–velocity space. This supports *locally two-directional sampled support*; it does not certify a two-dimensional open region, diagonal combinations, independent position/velocity variation, attitude or actuator uncertainty, or full-state reachability. At `stationary_0`, R5's failed slope triplet was at frame 327, not the successful frame 330 used here, and cannot be transferred between frames.

The appropriate overall classification is therefore **mixed sensitivity**, with stronger multidirectional support at three anchors and a specific negative-cross sensitivity at `dev_fast`. The complete autonomous records are useful for future transition-data research with per-state qualification labels. Pooling them into a uniformly robust region or reviving the rejected similarity reward would exceed the evidence. All starts remain already-recentering states; early outward rotation and autonomous reversal remain unproved.

## Records, verification and preservation

Every new case under [cases](../runs/asm2-terminal/r5-1/cases/) contains the complete initial state and then every 0.1 ms physical state, actual rotor thrust and held motor targets in `physics.npz`; 5 ms CTBR commands, control timestamps and allocations; all nonempty contact records; the full native outcome; and an exact viewer-compatible `replay.npz`. **30,091 physics intervals** were captured. All five nominal records are referenced from their existing R5 directories, not duplicated as new trials.

Representative dev_fast replays: [negative, rejected](../runs/asm2-terminal/r5-1/cases/dev_fast-816-across_minus/replay.npz), [positive, qualified](../runs/asm2-terminal/r5-1/cases/dev_fast-816-across_plus/replay.npz), and [R5 nominal, reused](../runs/asm2-terminal/r5/cases/dev_fast-816-nominal/replay.npz). The existing viewer can play them; no new Isaac rendering is claimed.

**Verification PASS:** every initial perturbation and unchanged component is rechecked; all actor commands reproduce exactly from saved observations; allocations and held targets agree; the maximum analytic motor-filter residual is **5.55×10⁻¹⁷ N**. Saved contacts replay through the frozen scorer to reproduce every full result exactly. All replay states match their physics samples and all ten archives pass the existing viewer loader. Two focused coordinate/sign tests pass. All **148 frozen pins** remain intact, including every pinned R5 completion artifact and protected checkpoints. Verification uses saved data and performs zero additional native rollouts.

[Manifest](../runs/asm2-terminal/r5-1/manifest.json) · [Native results](../runs/asm2-terminal/r5-1/results.json) · [Matched analysis](../runs/asm2-terminal/r5-1/analysis.json) · [Verification](../runs/asm2-terminal/r5-1/verification.json) · [Analysis implementation](../../src/rm27_asm2/cross_track_report.py).

```bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src:.
# Saved-data verification only: no new native rollout.
/tmp/rm27-g311-env/bin/python -m rm27_asm2.cross_track_report analyze
/tmp/rm27-g311-env/bin/python -m unittest discover -s tests -p test_asm2_cross_track.py
```

The ten-rollout authorization is exhausted. No held-out data, PPO, reward, potential, controller or reachability model was used or developed. Phase 4, Fixed A, physics, geometry, actuator model, scorer and all historical evidence remain unchanged. No commits or pushes occurred.

## Exactly one next experiment — not launched

**One three-case negative cross-track amplitude-bracketing experiment at `dev_fast:816`.** Freeze coupled offsets at 25%, 50% and 75% of the failed negative perturbation: positions −0.125, −0.250 and −0.375 mm with velocities −1.25, −2.50 and −3.75 mm/s. Preserve all other initial components, reuse the existing nominal and full-offset outcomes, and run frozen Fixed A with the same scorer. No adaptive extension or repeated nominal is needed.

This would determine whether qualification survives smaller negative offsets or shows a nonmonotonic gap before treating this anchor as part of a useful transverse dataset. It targets the one observed failure with three new rollouts and the existing recorder, rather than asserting width from endpoint success or changing the task. It would still provide sampled evidence, not a certified boundary. **Recommendation only; not launched.**
