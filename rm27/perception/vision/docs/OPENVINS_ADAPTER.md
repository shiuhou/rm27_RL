# OpenVINS → RM27 runtime adapter — 2026-10-07

## Current acceptance result

**VIO-P remains PARTIAL.** Runtime integration, input coverage, same-run raw to
adapter conversion and metric equality passed in `adapter-runtime03`, but the
predeclared historical native02 ATE-consistency check failed. Do not replace
this with PASS because the new ATE happens to be lower.

| Quantity | Native02 baseline | Adapter03 |
|---|---:|---:|
| Visual poses / GT-overlap poses | 2800 / 2780 | 2800 / 2780 |
| ATE RMSE | 0.0701139912 m | 0.0573847744 m |
| 1-second translation RPE | 0.0452377799 m | 0.0440047720 m |
| 1-second rotation RPE | 0.477985477 deg | 0.476994424 deg |
| Path length bias | -1.9414246% | -1.5039354% |
| Applied scale | 1 | 1 |

ATE difference **0.0127292168 m** exceeds the unchanged **0.01 m** repeat-run
limit. All other scalar consistency checks and full coverage checks passed.
Live input received exactly 2912 images/29120 IMU samples, with every image
byte and raw IMU value/time checked against ASL. Independent MCAP equality:
2800 visual poses, 27980 propagated states (including local velocities).
Native/player/recorder/observer all exited 0. Visual velocity speed ranged
0.0004667–1.0283858 m/s (median 0.3955787), not a flight suitability claim.
Postfailure independent integrity verification passed all 17 source/config/
binary/library/input hash checks (`final-integrity.json`). Native resource
report: user 37.03 s, system 2.43 s, wall 300.40 s, peak RSS 146132 KiB;
this excludes observer/recorder and is not a real-time performance benchmark.

Evidence: `adapter-runtime03/run.json` retains FAILED/NATIVE_BASELINE_DRIFT;
`adapter-parity-diagnostic.json` preserves recomputed adapter metrics and both
same-run equality and rejected historical-baseline comparison. No trajectory
was replaced, truncated, rescaled or selected to make the gate pass.

`run-variation.json` shows identical first state and first upstream-state
divergence at row 188 (zero-based), timestamp 1403715288.26227 seconds, before
RM27 conversion. Propagated-emission timestamp sets also differ. This does not
prove estimator IMU loss: its callback ingestion is not instrumented. The
specific cause of run-to-run estimator differences remains unconfirmed.

An additional **native-only** control (`native-ros2-control03`, original native
wrapper, no observer, same binary/config/input/rate/resource limits) completed
with native/player/recorder exit0 and 2800 poses. Its ATE is **0.0624031978 m**,
1s RPE **0.0449588172 m /0.477944980 deg**. Evidence:
`native-control03-evaluation/metrics.json`, `native-control03-console.log`,
`run_native_control.sh`. Thus native execution itself varies without the
adapter. This single control does not establish the full variability envelope
or exclude additional observer timing effects. It does not replace the frozen
native02 reference or waive the rejected historical ATE gate.

Remaining work is a focused native repeatability/ingestion investigation or an
explicitly reviewed acceptance-policy decision. No threshold changes, new
estimator instrumentation, algorithm edits, or downstream hardware work were
silently introduced to turn PARTIAL into PASS.

## Scope

Public EuRoC V1_01_easy mono cam0 + raw IMU baseline only. The estimator remains
**pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**, revision
`69488123ed9362dd44b6f28e7f4680abbff1442b`. No estimator math, input dataset,
calibration, native QoS, or legacy monocular optional-IMU rule was changed.
No M3C, physical capture, autopilot, alternate estimator, commit or push.

The existing experiment runner now dispatches `backend=OpenVINS` to the isolated
VIO path in `experiments/openvins_runtime.py`. Its EuRoC loader requires the
qualified ASL archive and exact verified bag checksums. Unsupported sequences
are refused rather than silently inheriting V1_01_easy frame/calibration rules.

## Runtime and output boundaries

- A read-only ROS2 observer starts before playback and subscribes to images,
  raw IMU, visual-state poses and propagated odometry. Subscription readiness
  is checked for both output streams. It converts pose/quaternion and local
  velocity fields in callbacks, and persists raw observations plus projections.
- The native binary, source/config, input bag and repository are read-only
  container mounts. Only the new run directory is writable. The ASL archive
  and ground truth are not mounted in the estimator container.
- Same half-rate playback, single camera, original public calibration, image
  `sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14`,
  4 CPU/12 GiB limits as native run02. Binary SHA256 remains
  `f99c936597079c6957fb3ac5d0c5ee1e642ff173754d60e8f121d0a5b5979b23`.
- After clean shutdown, original online state text supplies camera/IMU offset
  and world-frame visual velocity. Association is integer-ns, unique and ordered,
  within 6 microseconds of text quantization. Final normalized files are written
  after the run but contain **online states**, never optimized-final poses.
- `raw/pose.jsonl`, `raw/odom.jsonl`, native text, MCAP and logs are separate from
  `online/trajectory.json` (visual states) and `online/propagated.json` (IMU-rate
  propagation). `final_optimized_trajectory` is absent/null.
- Existing output directories are refused without changing their `run.json`.
  A timeout stops only the uniquely named research container. Failed runs remain.

## LocalizationEstimate v2 semantics

The adapter uses the existing v2 field meanings and OnlinePoseRecord envelope;
it does not change the schema to make unavailable evidence look complete.

| Item | Verified interpretation / treatment |
|---|---|
| Pose | `T_backend_world_imu`, maps IMU coordinates to world |
| Quaternion | Native JPL G→I xyzw equals Hamilton I→G xyzw; reorder to wxyz, no conjugation |
| Translation | meters; metric scale designation is not a zero-bias guarantee |
| Visual velocity | Online text v_IinG, world-frame m/s, 6 decimal place export precision |
| Propagated velocity | `/odomimu` local IMU-frame m/s, rotated by Hamilton I→G into world |
| Image time | Original ASL integer nanoseconds; no exposure semantic invented |
| State time | ROS header integer ns, ultimately derived from upstream double seconds |
| Receipt time | Actual observer monotonic clock, separate from sensor clock |
| Publish/completion time | Unknown; receipt is not substituted for either |
| Raw IMU coverage | Verified at adapter subscriber; not proof of estimator callback ingestion |
| Initialization | Publication gate is source-supported as initialized; not a TRACKING event |
| Tracking/validity | UNKNOWN state, false operational validity; does not assert numerical pose is bad or tracking is LOST |
| Epoch | Adapter run owns epoch 0; run ID separates sessions, not an upstream reset count |
| Reset/map/quality | Unknown, not synthesized; covariance retained raw, not reinterpreted |

Pinned `ROS2Visualizer.cpp` documents the quaternion interpretation and local
odometry velocity. `Propagator.cpp:251` explicitly rotates global velocity into
local coordinates before publishing. Visual saved velocity comes directly from
the IMU state alongside its pose. Tests cover known 90-degree rotations and
reject nonunit/nonfinite inputs; conversions do not implicitly normalize them.

All output contracts remain **incomplete v2 projections** because upstream does
not expose RM27 tracking state or true publication time. No fabricated full
`LocalizationEstimate` is emitted. This is the approved missing-evidence policy,
not a flight-ready localization stream. Numerical SE3 evaluation of observed
poses is separate from operational validity/availability assessment; legacy
validity-gated evaluators were not weakened.

## Acceptance gates

1. Native, player, recorder and observer must exit 0.
2. Adapter input timestamps/order, all image bytes and every raw IMU SI value
   must equal the ASL sequence: 2912 images and 29120 IMU samples.
3. Both live pose and propagated streams must be nonempty and exactly match
   independent MCAP records in count, timestamps, positions, quaternions, and
   local propagated velocities. Text visual state associations are one-to-one.
4. Evaluate adapter visual poses using the same integer time association,
   reference normalization, GT-overlap interpolation, fixed-scale SE3, ATE and
   adjacent/1-second RPE as the native evaluator. Same-run metrics must be exact.
5. Compare to native run02: raw/evaluated counts and camera endpoints must match;
   GT hash and fixed-SE3 policy must match; duration differs at most 5 ms.
   Predeclared repeated-run allowances: ATE ≤1 cm difference, 1s translation RPE
   ≤1 cm, rotational RPE ≤0.1 degree, path ratio ≤0.01. These allow scheduling
   variation, not scale correction. Same-run conversion allows no metric drift.
6. Regression and separate sourced ROS2 tests pass. Original GT orientation
   accuracy caveat remains; no claim of flight accuracy or real-time M3C rate.

## Preserved failed attempts and review

External evidence root: `/home/shiuhou/Projects/rm27-vio-20261007`.

- `adapter-runtime01`: observer startup failed because the initial wrapper
  overwrote ROS PYTHONPATH. Fixed by prepending the repository while preserving
  sourced ROS paths; regression test added. No estimator algorithm change.
- `adapter-runtime02`: all processes exited 0 and 2800 poses were saved, but
  observer received only 28681/29120 IMU samples. Missing timestamps occurred
  in bursts throughout the sequence, with no duplicates. Acceptance failed.
  Observer-only best-effort queue depth increased from 5 to 10000; estimator
  subscriptions and QoS were left unchanged. No interpolation/replacement samples.
- Independent read-only code review found insufficient coverage gating and
  missing independent propagated-output comparison. Both are now enforced and
  tested. Re-review found no remaining significant correctness findings; runtime
  acceptance is still based on actual execution, not the review's conclusion.

## Reproduction

Qualified config and immutable installation manifest live outside the repo as
`adapter-config03.json` and `adapter-install03.json`. For another run, copy the
config with a **new** run ID/output path; preserve existing evidence. From the
research worktree in a sourced Jazzy system-Python shell:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m rm27.perception.vision.localization.experiments.runner \
  --config /home/shiuhou/Projects/rm27-vio-20261007/adapter-config03.json
```

Do not rerun that literal config once its output exists. Tests use separate
environments (sourced ROS paths should not leak into the isolated regression venv):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
# separate shell:
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -p test_asl_transport.py
```

Fresh results: 226 passed, 1 ROS-only skip; separate ROS2 7/7 passed, including
the skipped transport case. 18 new runtime tests cover mapping, clock precision,
stream order, velocity, refusal paths, fixed-scale parity and coverage. Test-first
failure logs and final outputs are under the external evidence root. Native
`/usr/bin/time` evidence measures estimator process only, not total adapter cost.
