# Native SE(3) evaluation — 2026-10-07

## Scope and result

Evaluated the existing clean `native-ros2-run02-lifecycle` online `/poseimu`
stream. No estimator rerun, data download, hardware access, algorithm change,
time-offset fit, or Sim(3) scale correction. Designation remains:
**pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**.

Native numerical evaluation: **PASS (evaluation integrity, not flight accuracy)**.
Broader VIO-P adapter qualification: **PARTIAL**. An initial output projection
was implemented only after numerical evaluation and independent cross-check.
Runtime dispatch, EuRoC framework loading and new adapter/native replay parity
are not implemented/qualified yet. Do not confuse saved-output import equality
with an independent adapter execution.

## Results

| Metric | Observed value |
|---|---:|
| Raw visual-state poses | 2800 |
| Evaluated poses in GT overlap | 2780 |
| Excluded after GT ends | 20 |
| Evaluated duration | 138.950884580 s |
| ATE translation RMSE | 0.0701139912 m |
| ATE median / P95 / max | 0.058690731 / 0.114816554 / 0.132865730 m |
| Adjacent RPE translation RMSE | 0.0025019156 m |
| Adjacent RPE rotation RMSE | 0.026057953 degrees |
| 1-second RPE translation RMSE | 0.0452377799 m |
| 1-second RPE rotation RMSE | 0.477985477 degrees |
| Estimated/reference path length | 57.314988367 / 58.449746118 m |
| Path length ratio / bias | 0.980585754 / -1.9414246% |
| Centered trajectory extent ratio | 0.982897437 |
| Applied alignment scale | **1.0 (fixed)** |

Scale diagnostics are estimated/reference path length and centered Frobenius
extent ratios over exactly the matched samples. They are sensitive to trajectory
shape, noise and drift; they are not pure calibration-scale error estimates.
Neither ratio is applied to coordinates. No Sim(3) transform is fitted.

## Association and coordinate contract

- Use MCAP `/poseimu`, not rounded text poses or mixed propagated `/odomimu`.
- Original CSV timestamps and ROS sec/nanosec pairs become Python integer ns;
  interpolation subtracts integer epochs before floating-point division.
- Text state time minus saved online camera/IMU offset identifies original
  camera indices 112–2911 uniquely. Maximum residual 5124 ns under a 6000 ns
  tolerance justified by upstream text time precision (10 microsecond quantum).
- Raw/text state timestamps match one-to-one within 4999 ns, and all position
  and quaternion components match within text rounding (0.501 micro-units).
  The estimator's ROS time still originates from upstream double seconds: no
  claim of original sensor-nanosecond fidelity is made.
- Compare at raw **IMU state time**, not the camera time. Ground truth is
  linearly interpolated in translation, shortest-arc SLERP in orientation.
  Only overlap is used; no extrapolation, maximum allowed bracket 10 ms.
- Upstream `ROS2Visualizer.cpp` publishes JPL G-to-I xyzw; its documented
  equivalence is Hamilton I-to-G xyzw. Reorder to wxyz, **do not conjugate**.
  Both estimate and GT are `T_world_imu`; GT `sensor.yaml` T_BS is identity.
- A single rigid rotation/translation is fitted to all matched positions.
  There is no segmented alignment, reset repair or truth-aided initialization.
  Runtime reset/map events remain unobserved, not invented as known absent.
- RPE is `inv(inv(Tg_i) Tg_j) (inv(Te_i) Te_j)`. Adjacent: 2779 pairs,
  actual dt 0.049747228–0.050511122 s. One-second: 2760 pairs, nearest later
  sample within 25 ms of one second; actual dt 0.999701738–1.001696348 s.
  Every pair and reference bracket is saved, not just aggregate statistics.

## Reference limitation and failed first attempt

The original V1_01_easy orientation reference has an upstream-documented
accuracy caveat. It was not replaced with corrected ground truth; rotation
metrics must retain this limitation.

The first real evaluation correctly stopped on a 2 ppm unit-quaternion check.
Inspection found 6934/28712 GT quaternions outside 2 ppm, with maximum norm
deviation 20.52610684 ppm; all were finite. This exceeds decimal rounding alone.
Reference-only unit normalization is now explicit, bounded at 0.1%, recorded
in metrics, and tested (large deviations still fail). It projects quaternion
direction onto SO(3), does not correct truth orientation or positional scale,
and leaves the archive unchanged. Estimated pose normalization policy was not
loosened. The initial assumption/failure is preserved here rather than hidden.

## Evidence and reproduction

All external paths below are relative to
`/home/shiuhou/Projects/rm27-vio-20261007/`:

- `native-se3-evaluation01/metrics.json`: metrics, alignment, RPE pairs,
  input/archive/script SHA256 values and normalization policy.
- `native-se3-evaluation01/associations.json`: raw pose, camera and GT brackets.
  SHA256 `21246e6b042f83f5e4fa05342f2aae03d5bd073caca53d57964d2d74a9b03b56`.
- `native-se3-evaluation01/scipy-crosscheck.json`: PASS using independent
  SciPy 1.11.4 rotation alignment/SLERP/RPE calculation on original inputs.
  ATE 0.07011399119656574 m, 1s RPE 0.0452377799213236 m / 0.4779854772212334 deg.
- `crosscheck_accuracy.py`, `inspect_truth.py`: independent check and diagnostic.
- Raw MCAP SHA256 `1fb16ae6c577562c4b54f4eb18cb034aada3a3d88e4ecec7af1f6cb591e8c2b0`.
- GT CSV SHA256 `701200e6924ba845ccdeea884bd717c82d1ed91f480913ea6e2e6ccb9c0fbf6d`.

Run from research worktree, using a **fresh** output directory:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/openvins/native_accuracy.py \
  --run /home/shiuhou/Projects/rm27-vio-20261007/native-ros2-run02-lifecycle \
  --archive '/home/shiuhou/Projects/rm27-vio-20261007/vicon_room1(1)/vicon_room1/V1_01_easy/V1_01_easy.zip' \
  --out /home/shiuhou/Projects/rm27-vio-20261007/native-se3-evaluation-repeat
python3 /home/shiuhou/Projects/rm27-vio-20261007/crosscheck_accuracy.py
```

## Adapter start and validation

`experiments/openvins.py` introduces the initial `/poseimu` projection using
existing `OnlinePoseRecord` and partial **LocalizationEstimate v2** fields.
`online.py` adds an empty OpenVINS state map: publication alone cannot imply
TRACKING. Image time, IMU-state time and recorder receipt time are separate;
publication/completion times, raw IMU coverage, velocity, reset/map events and
quality stay unavailable. The epoch is explicitly adapter-owned. Metric units
do not imply perfect scale or flight suitability. No optimized-final export.

`adapter-projection01/summary.json` verifies all 2800 saved messages preserve
position, reordered quaternion and state timestamp exactly. `online.json`
contains 2800 deliberately incomplete records. This is not runtime parity.

Tests (fresh shells; do not mix the isolated regression venv with sourced ROS):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
# 208 passed, 1 skipped (ROS-specific case)
```

In a separate shell:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -p test_asl_transport.py
# 7 passed, including the separately skipped case
```

11 evaluator synthetic cases and 7 projection cases were red before their
implementations. Synthetic tests verify integer precision, overlap/gaps,
quaternion direction/sign, rigid transform, rejection paths, and retention of
2x scale error after SE3. These tests are not real-estimator accuracy evidence.
An accidental combined ROS-environment/venv test attempt failed on ROS imports;
separating the established environments passed without changing tests/packages.

Next software step: implement the EuRoC-specific framework loader and runtime
dispatch, then run native/adapter parity with identical pinned inputs/config.
No physical action requested; no hardware was connected. No commit or push.
