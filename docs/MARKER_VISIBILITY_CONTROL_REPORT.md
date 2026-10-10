# RM27 marker-aware flight-control study

Date: 2026-09-30

This study addresses the control-side problem visible in the FPV replay: the
vehicle should keep the provisional green guidance lamp in a usable part of
the camera view while approaching the armor. It does not change SLAM, optical
flow, marker detection, or estimator code. The lamp is represented only by a
geometry proxy using the configured CAD/display camera mount.

## Implementation

The controller now has an optional `look_at_marker` heading reference. The
trajectory acceleration still determines the thrust direction; the heading
reference only selects yaw around that thrust axis. The lamp position is read
from `configs/base_target.json`.

The marker offset fix in this iteration is important. The marker offset is
carried from the base asset using the shortest rotation between the nominal and
perturbed armor normals. It is not rotated by
`target.in_plane_rotation_deg`, which describes the rectangular contact patch,
not the lamp mounting pose. Regression tests cover nominal position, patch
heading changes, and normal perturbation.

The visibility proxy requires the lamp centre and eight perimeter samples to
be inside the assumed rectangular frustum. It reports centre/footprint
visibility, image-plane drift, and a conservative angular margin. It models no
occlusion, detector threshold, lens distortion, exposure, motion blur, or
visual estimator timing.

## Paired study

Command:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.smooth_flight \
  --output artifacts/marker-smooth-flight-study-20260930-final \
  --study configs/contact_smooth_flight_study.json --workers 4
```

The design and validation split was frozen before the 32 paired holdout cases
at each uncertainty scale. All three methods passed all 16/16 cases at both
scale 1 and scale 2 in this campaign. The metrics below are medians over the
16 holdout cases.

| method | scale | contact time (s) | mission time (s) | max pre-contact speed (m/s) | marker footprint outside, far | marker vertical RMS (deg) | contact margin (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|
| baseline | 1 | 4.48 | 20.10 | 6.13 | 36.9% | 26.48 | 23.99 |
| heading-only | 1 | 4.48 | 23.53 | 6.13 | 36.9% | 26.48 | 23.99 |
| marker-aware smooth | 1 | 6.80 | 25.81 | 4.36 | 12.3% | 19.72 | 21.49 |
| baseline | 2 | 4.48 | 20.00 | 6.13 | 36.7% | 26.47 | 23.08 |
| heading-only | 2 | 4.48 | 23.43 | 6.13 | 36.7% | 26.47 | 23.08 |
| marker-aware smooth | 2 | 6.80 | 25.73 | 4.36 | 12.0% | 19.67 | 20.38 |

The smooth candidate also reduced median pre-contact jerk RMS from 9.26 to
5.49 m/s^3 at scale 1 and from 9.39 to 5.59 m/s^3 at scale 2. Its maximum
pre-contact tilt fell from about 32.6 to 28.1 degrees. The cost was roughly
2.32 s longer to contact, 1.77 m/s lower peak approach speed, and a small
reduction in contact margin. It introduced about 0.31 s of low-speed interior
flight, so it is not a speed improvement.

## Interpretation

The controller-side yaw reference alone does not improve the marker proxy:
heading-only changes the return orientation but leaves the outbound trajectory
and image-plane statistics unchanged. The improvement comes from selecting a
slower, lower, smoother whole-path trajectory whose thrust-derived attitude
keeps the marker footprint nearer the frustum.

This is evidence for a speed/visibility/contact-margin trade-off, not evidence
of successful green-light tracking. The camera mount and marker pose remain
provisional assumptions, and all trials are MuJoCo truth-assisted closed-loop
direct-motor simulations. The FPV rendering is a recorded-state visualization
only.

## Reproduction and artifacts

The full paired results, per-trial traces, failure records, frozen selection,
and `summary.json` are in
`artifacts/marker-smooth-flight-study-20260930-final/`. The recorded FPV video is:

`artifacts/marker-smooth-fpv-20260930-final/first_person.mp4`

The viewer can replay the selected smooth flight:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view \
  --input artifacts/marker-smooth-flight-study-20260930-final/demo/smooth/seed-None-scale-1.0 \
  --camera fpv --speed 1 --loop
```

The next control-side experiment should use timestamped noisy target/ego
estimates and an explicit speed constraint, then determine whether the same
visibility benefit survives realistic observation timing. A policy or RL stage
is not justified by this proxy result alone.
