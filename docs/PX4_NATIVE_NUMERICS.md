# Native VIO numerical failures and termination development

Updated 2026-10-11. This is host software development; active nominal and
condition batches keep their frozen native installation.

## Preserved failures

`px4-initial-vio-admission-development-01` and `-02` fail during takeoff when
pinned OpenVINS produces negative covariance diagonals and exits. The initial
registration guard does not repair that native numerical failure.

An isolated copied installation supports the exact original covariance form
for diagnosis and a checked innovation solve with the full Joseph update.
Sources, patches, libraries and binary hashes are separately recorded in
`openvins-native-covariance-development-build-01` and `-02`. No eigenvalues
are clipped. The original installation and its libraries remain untouched.

Saved noisy inputs from the second failed flight are replayed with unchanged
acquisition times, receipt ordering, image bytes, IMU and estimator settings.
The original form fails again at native time12.452 s. Its innovation condition
number is approximately2.88e10; the prior minimum eigenvalue is−3.37e−12
at a maximum eigenvalue29680, while the subtractive posterior has minimum
eigenvalue−0.0861 and diagonal−0.00972. The raw matrices remain in
`px4-native-noise-original-replay-01`; the read-only diagnostic is
`px4-native-covariance-diagnostic-01`.

The first Joseph variant incorrectly rejects the nearly singular prior using
LDLT pivots (`px4-native-noise-joseph-replay-01`, failed). The second checks
self-adjoint eigenvalues against an explicit dimension/scale roundoff bound.
It completes that saved segment:251 images,3131 IMU samples,126 native
publications and native exit0 (`px4-native-noise-joseph-replay-02`). This is
recorded-input execution evidence only.

A fresh stochastic composite flight still fails
(`px4-covariance-noise-development-01`). At12.652 s its prior minimum
eigenvalue is−2.598e−9 against the declared3.311e−12 roundoff bound.
`px4-native-covariance-diagnostic-02` preserves that result. The guard rejects
it; the variant is not noise-qualified or a completed numerical repair.

## Continued physical termination

`tools.px4.run_native_variant --terminate-native-failure` adds a development
policy for VIO transport failure in the known initial dock footprint. Admission
requires TAKEOFF/HOVER, fresh valid PX4 navigation, attitude and range,
bounded position/velocity and an armed Offboard vehicle. It consumes no
physical truth and does not admit route failure or stale navigation.

The first LAND reaches automatic disarm and stopped motors but fails its
physical gate:26.189 mm resting offset exceeds25 mm and an unintended contact
occurs (`px4-native-failure-landing-development-01`). It remains a failed
termination. Revised development selects the previously tested fixed-dock
range reference in the same isolated SITL, reads back every changed parameter,
then continues normal LAND. That fallback is unsuitable over varying terrain
and is never written to hardware. All contact/rest/stopped-motor scores remain
independent; the revised run requires its own actual result.

Revised `px4-native-failure-landing-development-02` passes its physical
termination gate:6.452 mm resting XY,22.556 s continuation, no unintended
contact, actual landed/automatic disarm and stopped motors. Its original
mission remains failed. This is one development attempt with the same noisy
input seed, not a reserved fault batch or a VIO repair qualification.

## Reproduce diagnosis

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
/home/shiuhou/venvs/mujoco/bin/python -m tools.openvins.build_covariance_variant \
  --output artifacts/openvins-numerical-build-new
/home/shiuhou/venvs/mujoco/bin/python -m tools.openvins.replay_native_inputs \
  artifacts/px4-initial-vio-admission-development-02/vio \
  --installation artifacts/openvins-numerical-build-new/installation.json \
  --form original --rate 0.5 --output artifacts/native-original-replay-new
```

Use `--form joseph` and another fresh output for comparison. These inputs are
generated simulation records; hardware qualification remains separate.
