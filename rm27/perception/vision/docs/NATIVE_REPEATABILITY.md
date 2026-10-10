# Native repeatability diagnosis — 2026-10-08

## Status: experiment complete; root cause unresolved; VIO-P PARTIAL

Final evidence: `analysis-13.json` under the evidence root below. Ten baseline
and three separate configuration-control runs completed. All 39 recorded
native/player/recorder exit codes are zero. Frozen artifact verification passed
again after all runs. No estimator, adapter, dataset, evaluator or gate edits.
The earlier interim notes below are retained as experiment history.

## Final results

All runs have 2800 raw poses, 2780 evaluated poses, camera indices 112 through
2911. The first saved state occurs at dataset elapsed 5.6 s; its first 18 text
fields are equal across each group. This is not exact internal initialization
timing, nor proof that every hidden initializer state was identical.

| Run | ATE cm | 1s RPE cm | 1s RPE deg | Path bias % |
|---|---:|---:|---:|---:|
| baseline-01 | 6.3969 | 4.5827 | 0.47818 | -2.27738 |
| baseline-02 | 6.7431 | 4.4401 | 0.48024 | -1.48886 |
| baseline-03 | 5.7280 | 4.4410 | 0.47860 | -1.57169 |
| baseline-04 | 6.4806 | 4.4828 | 0.47865 | -2.02261 |
| baseline-05 | 6.6009 | 4.5259 | 0.47233 | -1.74512 |
| baseline-06 | 5.4320 | 4.5240 | 0.48078 | -1.60453 |
| baseline-07 | 7.6526 | 4.5048 | 0.47917 | -1.61708 |
| baseline-08 | 6.4710 | 4.4951 | 0.47578 | -2.02246 |
| baseline-09 | 7.2613 | 4.4731 | 0.47998 | -2.32749 |
| baseline-10 | 7.3171 | 4.4644 | 0.47813 | -1.59022 |
| cvserial-01 | 5.7951 | 4.5170 | 0.47620 | -2.07304 |
| cvserial-02 | 6.6041 | 4.5286 | 0.48432 | -1.96052 |
| cvserial-03 | 5.5510 | 4.4680 | 0.48198 | -1.40665 |

Baseline ATE mean 6.60835 cm, median 6.54075 cm, sample standard deviation
0.68928 cm, min/max 5.43201/7.65260 cm, empirical p05/p95 5.56523/7.50162 cm.
RPE translation mean 4.49339 cm (range 4.44010--4.58273); rotation mean
0.47818 deg (range 0.47233--0.48078). Path bias range -2.32749 to -1.48886%.
These are observed samples under this frozen environment, not a certified
population range, normal distribution, or flight accuracy bound.

All 45 baseline pairs diverge under the predefined policy below. First differing
text states occur at source-camera indices 113--218; first sustained meaningful
divergence occurs at dataset elapsed 5.650000128--10.950000128 s (0.05--5.35 s
after first saved state). Source-frame association is held fixed; maximum
pairwise state-time differences range 0.170470--1.821518 ms. State timestamp
variation includes estimated camera/IMU time-offset variation; it is not a
measurement of wall-clock callback latency. The full 45 pair records, initial
state fields and trajectory hashes are in `analysis-13.json`.

The cvserial controls were actually run with OpenCV threads=1 and publisher
threading disabled; native logs confirm both ROS overrides. ATE range
5.55095--6.60413 cm, mean 5.98340 cm, sample standard deviation 0.55126 cm.
One of three pairs exceeds 1 cm. All three diverge at 5.65--5.85 s. Thus these
existing controls do NOT make the ROS2 path deterministic. Three samples do not
establish improved variability; both knobs changed together, so their individual
effects are not isolated. Async subscription workers remain enabled.

## Interpretation and unchanged acceptance gate

Native run-to-run variation is reproduced independently of the RM27 adapter.
The seed is already explicitly fixed. Equal initial saved states and later
divergence shift attention to post-initialization execution, without excluding
unlogged initializer internals. Async callback/window timing and the source-level
reference-capture lifetime hazard below remain credible hypotheses, not proven
dynamic causes. No sanitizer, callback-order trace or controlled lifecycle fix
was executed in this diagnosis. RANSAC, thread-local RNG and feature histories
have not been isolated. Do not attribute the numerical spread to a specific bug.

Fifteen of 45 baseline pair ATE differences exceed 1 cm (33.3% of these correlated
pairs, not an independent failure-probability estimate). The current 1 cm gate
is not empirically justified as a guaranteed native repeatability bound or an
adapter-correctness discriminator for this asynchronous setup. It can remain a
desired engineering requirement that the present implementation fails. There is
no evidence here authorizing a replacement threshold. Keep VIO-P PARTIAL and
retain the independent same-run adapter/native exactness result.

Next software-only diagnostic: capture worker lifetime and camera/IMU dispatch
order around the first divergent frames in an isolated diagnostic build, before
proposing any fix. No code fix is claimed or applied here.

## Validation and handoff

Commands completed: `repeat_native.py --freeze --count 10`, then
`repeat_native.py --variant cvserial --start 1 --count 3`, and (with ROS2 sourced)
`python3 repeatability-20261008/analyze_repeats.py`. Final read-only check:
`python3 -c 'import repeat_native; repeat_native.verify()'` from the external
root returned zero. Each run retains command.json, execution.json, summary.json,
raw trajectory/MCAP/logs and evaluation/metrics.json. Regression results from the
previous implementation remain historical (226 pass/1 skip and separate ROS2
7/7); they were not rerun for this experiment-only/documentation task.
Research branch `research/vio-openvins-baseline`, HEAD
`13c7a77bbe0fd256e6765efd4459fbca3426e0b7`, with pre-existing uncommitted work
preserved. No commit, push, hardware, threshold change or Vault write.
Rollback for documentation is the retained pre-final snapshot; immutable run
evidence must not be removed. This completes the planned cohort/control study,
not root-cause closure.

Authorized task: freeze the current public baseline, target ten sequential native
runs, compare trajectories and test existing determinism controls separately.
Do not change adapter, estimator math, data, or the 1 cm acceptance threshold.
VIO-P remains PARTIAL. No hardware, commit, push or Vault operation.

Evidence root:
`/home/shiuhou/Projects/rm27-vio-20261007/repeatability-20261008`.
Controller: parent directory `repeat_native.py`. Console: parent directory
`repeatability-console.log`. The already-started batch is `--freeze --count 10`;
do not launch another copy. App heartbeat `rm27-vio` checks progress and resumes
analysis/control runs; it must be stopped when the task completes.

## Freeze and experimental design

`frozen-manifest.json` hashes the entire source tree, installed build artifacts,
verified input bag, ASL archive, original evaluator/launcher, their frozen copies,
and adapter runtime file (to detect unintended changes). Each run verifies these
hashes before and after execution. Each saves the exact command, launch hash,
manifest hash, host load, raw files, process status, complete evaluation and summary.

Pinned source `69488123ed9362dd44b6f28e7f4680abbff1442b`, designation
**pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**. Original
monocular public config/overrides and native binary retained. Container image
`sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14`,
network none, 4 CPU/12 GiB, half-rate playback. Runs are sequential, not concurrent.
Only output path/container name vary. Host evaluator Python3.12.3, NumPy1.26.4,
SciPy1.11.4; the SE3 evaluator is byte-frozen and no scale is fitted/corrected.

The first two new runs completed successfully with 2800 poses each:

| Run | ATE m | 1s RPE m / deg | Path bias % |
|---|---:|---:|---:|
| baseline-01 | 0.0639688450 | 0.0458272814 / 0.4781830392 | -2.277380 |
| baseline-02 | 0.0674305921 | 0.0444009872 / 0.4802375550 | -1.488862 |

These are interim samples, not the requested final range. Third run was active
at this checkpoint. No previous runs are silently included in the new cohort.

## Pairwise analysis policy fixed before final results

`analyze_repeats.py` reads raw MCAP and text association evidence. Poses are
matched by original camera frame; state-time differences remain separately
reported. A first-pose rigid registration removes arbitrary initial world gauge,
without trajectory-wide fitting or scale adjustment. Meaningful divergence:
translation >1 mm OR orientation >0.01 degree for three consecutive common
camera frames. The first differing text state is also reported separately.

Each run includes first saved initialized-state proxy, camera index/elapsed time,
and initial state; exact internal initialization-completion time remains unknown.
Report sample min/max, mean, median, standard deviation, empirical percentiles,
all pair differences and number exceeding 1 cm ATE. Ten samples do not establish
a population-normal envelope or a flight-accuracy bound.

## Source audit so far: facts vs hypotheses

- `ov_msckf/src/core/VioManager.cpp:66-67`: explicit OpenCV thread count and
  `cv::setRNGSeed(0)`. Lack of a fixed seed is not an established explanation.
- `ov_core/src/track/TrackKLT.cpp:873`: RANSAC fundamental-matrix rejection is
  active, but no RNG sequence/candidate trace has established it as the cause.
- `Grider_FAST.h:104` uses parallel detection but merges per-cell collections
  in fixed index order. Do not assume thread completion order defines output.
- Feature databases use unordered maps; that alone does not prove random
  per-process order. Their contribution is unisolated.
- `run_subscribe_msckf.cpp:83` forces `use_multi_threading_subs=true` after
  config loading; a false configuration value is ineffective for that entry.
- ROS2 uses a MultiThreadedExecutor plus detached camera-update workers.
  Callback/image/IMU ordering and state-window timing remain hypotheses.
- `ROS2Visualizer.cpp` callback_inertial creates local `message`, then captures
  it by reference through `[&]` in a detached worker that reads its timestamp.
  This is a source-level lifetime risk. Dynamic occurrence and contribution to
  observed ATE variation have not been proved; no causal claim or fix yet.
- `thread_update_running` is `std::atomic<bool>` (header line182), not an
  ordinary bool. Do not misreport a flag race as proven.

## Historical control plan — now completed above

After baseline batch finishes, three sequential `cvserial` runs use existing
`frozen/run_cvserial_ros2.sh`: only add `num_opencv_threads:=1` and
`multi_threading_pubs:=false`. Source, binary and stored config stay unchanged;
the override command/hash are recorded as a distinct group. The upstream serial
runner comments recommend similar controls for repeatability, but the ROS2
entry still forces asynchronous subscriptions. This is not a fully serial mode.

No acceptance-policy change is implied by this diagnostic. Final results must
replace this RUNNING status, update baseline/handoff/validation documents, and
explicitly distinguish an identified cause from remaining hypotheses.
