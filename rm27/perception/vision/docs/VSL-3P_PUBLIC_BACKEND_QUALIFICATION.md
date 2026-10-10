# VSL-3P public dataset backend qualification

Follow-up: [VSL-3P.1 online state instrumentation](VSL-3P1_ONLINE_STATE_REPORT.md)
adds runtime observations in new runs. The results and unavailable online fields
below describe the original VSL-3P runs and remain unchanged.

2026-09-23. **VSL-3P: PARTIAL.** RM27 drove unmodified pinned Stella end-to-end
on calibrated public data. ORB-SLAM3 also completed native execution, RM27
execution, normalization and evaluation **with a recorded upstream shutdown
patch**. Its unmodified native example crashed. This qualifies specific software
paths, not real-camera SLAM accuracy or an algorithm selection.

Worktree: `/home/shiuhou/Projects/rm27_drones_slam_integration`, branch
`research/slam-integration`. Initially this branch contained only `da396be`.
The authorized integration repair brought in hardening `eeb356b` as `7085f23`,
future infrastructure `50c07a5` as `5320ec5`, and the subsequent documentation
follow-up `08b29ac` as `307d64d`. The combined baseline passed 175 root tests
before installation. Actual 0921 canonical input refused with
`CALIBRATION_REQUIRED`, without starting a backend. It refused again after
public support and installations were present. No 0921 image was sent to SLAM.

Evidence **A**: [artifacts/vsl-3p-public-20260923-01](../../../../artifacts/vsl-3p-public-20260923-01/).
External source/build/data prefix **B**:
`/home/shiuhou/slam-public-qualification-20260923`.
All executions were offline host processing. Stella was headless; ORB's native
viewer ran in Xvfb display `:93`. No physical capture, flight, controller,
ExternalNav, IMU, VIO or M3C deployment was performed.

## Backend result

| Check | stella_vslam | ORB-SLAM3 |
|---|---|---|
| BUILT | PASS | PASS |
| NATIVE_EXECUTED | PASS, unmodified | PARTIAL: stock FAIL (SIGSEGV); lifecycle-patched PASS |
| RM27_ADAPTER_EXECUTED | PASS | PASS with lifecycle patch |
| OUTPUT_NORMALIZED | PASS, final frame poses | PASS with patch, final keyframes only |
| EVALUATED | PASS, one global Sim(3) | PASS with patch, one global Sim(3) |

Evidence verification `A/summary.json` has `passed: true`, scoped to the checks
above. This does not turn VSL-3P's PARTIAL into an unmodified-ORB qualification.
Unknown online states remain unavailable even when an execution check passes.

## Inputs and calibration scope

One sequence: **TUM RGB-D Freiburg1 xyz**, used as monocular RGB only. Original
archive: <https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz>.
The retrieved archive SHA-256, measured before extraction/analysis, is
`a0236d97b8c30cd93b653656d2b6c293ff7c982a4130ef2a1a8beecdb124ef98`.
This is a local download identity, not a publisher signature. The importer
checks all selected RGB files, `rgb.txt` and `groundtruth.txt` against archive
members. There are 798 original 640×480 PNGs, nominal 30 Hz, with an original
RGB timestamp span of 26.572059 seconds. No resize, decimation, re-encoding,
depth use or stress transformation was applied.

The [provider format/calibration page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)
is retained and hashed. Its Freiburg1 RGB pinhole/radtan5 parameters are:
`fx=517.3, fy=516.5, cx=318.6, cy=255.3`, distortion
`[0.2624, -0.9531, -0.0054, 0.0026, 1.1633]` in
`[k1,k2,p1,p2,k3]` order. Both native and adapter configs use these published
rounded values and BGR input. ORB's native config derives from pinned
`Examples/Monocular/TUM1.yaml`; only published calibration values and the BGR
flag were substituted. Feature/optimizer settings were not tuned for results.

`public_tum.py` introduces a separate, narrowly pinned public-data import and
`PUBLIC_PROVIDER_CALIBRATION` gate. Its artifact status is `PUBLISHED`, not a
claim of independently fitted/validated RM27 intrinsics. Physical lens and focus
remain UNKNOWN. This gate requires the public identity, archive pin, original
geometry and retained provider evidence. It does not satisfy the local hardware
calibration gate. `CALIBRATION_REQUIRED` is checked before public dispatch.

The canonical reader now supports `DATASET_TIMESTAMP` explicitly for this public
kind and a source archive. The existing `encoded_pts_s` CSV column and
`encoded_pts` record keys are retained for compatibility; their semantic is
**dataset time**, not encoded video PTS or verified exposure time. Original
decimal timestamp strings are preserved in both backend input lists.

Stella's TUM loader associates depth even for monocular mode and averages RGB
and depth times. A separate native input view and the adapter both provide
identical RGB timestamp/path aliases in `depth.txt`, explicitly marked as an
**unused association shim**. Source inspection confirms the monocular branch
reads only RGB. The original public `depth.txt` and depth images are untouched.
Input verification checks all 798 timestamp strings and image hashes across
both adapter runs.

Reference is the original provider `groundtruth.txt` (motion-capture camera
poses, metres, T_wc, xyzw). Association uses globally nearest one-to-one matches
within 20 ms, zero offset and no interpolation. This supplies reference poses
for 796 RGB frames; unmatched samples remain unmatched. Provider quaternions
are rounded text, so reference-only unit normalization is explicit, with each
original norm retained. Backend quaternions are checked, never silently fixed.

## Builds and native failures

| Component | Repository | Exact revision |
|---|---|---|
| Stella | `https://github.com/stella-cv/stella_vslam.git` | `e445b5452535e781fdf6777ec33a06f5f8f5e416` |
| Stella examples | `https://github.com/stella-cv/stella_vslam_examples.git` | `defc69eecc36e51cdda22885bb86954f08ad6887` |
| ORB-SLAM3 | `https://github.com/UZ-SLAMLab/ORB_SLAM3.git` | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` |
| Stella g2o | `https://github.com/RainerKuemmerle/g2o.git` | `e8df2004e07ea8f5b8e6a8b9f2dc067b45b45036` |
| Pangolin v0.6 | `https://github.com/stevenlovegrove/Pangolin.git` | `dd801d244db3a8e27b7fe8020cd751404aa818fd` |
| FBoW vocabulary | `https://github.com/stella-cv/FBoW_orb_vocab.git` | `708407ae4cd59219b996eda7ed1f4562c7ae106a` |

Ubuntu x86_64; GCC 13.3.0, CMake 3.28.3, Ninja, system C++ OpenCV 4.6.0,
Eigen 3.4.0, yaml-cpp 0.8.0, spdlog 1.12.0, SQLite 3.45.1 and Boost 1.83.
Python checks use `/home/shiuhou/venvs/mujoco/bin/python` (Python 3.12.3,
OpenCV 5.0.0, NumPy 2.5.1). Exact environment, packages, dependency submodule
pins, CMake caches, Ninja recipes, source diffs and logs are retained in A.
Builds are Release, parallelism 2–3. Stella uses FBoW, CPU, no ArUco, no
march-native flag or viewer. ORB retains upstream march-native settings and
Pangolin. These are different runtime/build conditions, not a throughput contest.

Missing SuiteSparse development files and Xvfb were obtained as Ubuntu packages
and extracted into B/sysroot, without sudo/system installation. The initial g2o
configuration lacked CSparse and was superseded by the recorded CSparse build.
Pangolin v0.6 needed compiler option `-include cstdint` with GCC 13. No Pangolin
source edit was required. `A/build-recipe.sh`, caches and logs record the build
recipe; extracted package filenames and hashes pin the actual packages used.

Stock ORB native execution processed all 798 inputs and wrote keyframes but
exited with signal 11. A separate gdb reproduction showed `Viewer::Run` still
running during shared-library teardown; LocalMapping was also still active.
Pinned `System::Shutdown()` has its worker-wait code commented out. The retained
`A/build-logs/orb-shutdown.patch` requests viewer finish, waits for mapping,
loop closure/global BA and viewer completion, then joins their worker threads
before trajectory export/exit. No estimator algorithm or feature setting was
changed. Patched native execution exited 0. Original failures, debugger output
and original binaries/libraries (`B/install/orb-stock`) are preserved.

This is explicitly **pin plus patch**, not a clean-upstream success claim.
Installation provenance hashes the patch and runtime libraries as well as the
binary and vocabulary; the adapter checks supplied runtime-file hashes.

## Adapter findings and transform verification

Stella's first RM27 execution exited 0 but normalization refused its first
exported timestamp: pinned `trajectory_io.cc` uses 15 significant digits, making
`1305031102.175304` become `1305031102.1753`. The parser now derives its matching
bound from that documented formatting quantum plus one double ULP, requires a
unique match, and retains the residual and original source timestamp. It still
rejects ambiguous associations. Raw output was not rewritten.

ORB's first patched adapter run refused config `Camera.fps: 30.0`: pinned
`Settings.cc` requires an integer YAML node. The adapter now emits `30` for
integral nominal rates and rejects nonintegral rates rather than rounding them.
The next run succeeded. Both findings were invisible to the earlier stub suite
and now have focused regression tests.

Stella `io/trajectory_io.cc` composes `T_cw`, explicitly calls `inverse_pose`,
then writes translation and quaternion from `T_wc`. ORB
`System::SaveKeyFrameTrajectoryTUM` calls `GetPoseInverse()` and writes its
translation and xyzw quaternion. Camera projection code establishes optical
axes x right, y down, z forward; each backend world is arbitrary. RM27 therefore
retains T_wc, reorders xyzw→wxyz, and labels parent `backend_world`, child
`camera`, with ARBITRARY scale and arbitrary translation units in the v2
LocalizationEstimate field projection. No full estimate is fabricated: publish
time, tracking state, epoch and map/reset state remain absent.

Known nonidentity-transform tests exercise both adapters' exported conventions;
the existing T_cw inversion test verifies rotation of translation, not merely
negation. Source files and hashes are in `A/source-evidence/`.

ORB export scope was checked more deeply: at this pin,
`Atlas::GetAllKeyFrames()` returns **the current map's** keyframes under a lock,
not every map in the atlas. Stella exports from one locked map database.
Each evaluated run has one observed initialization/map creation and no reported
reset. The final-coordinate review binds source, logs, normalized output,
reference and run config hashes. It applies to exported final poses only and
does not certify continuous online tracking.

## Observed results

| Measurement | Stella adapter (`adapter-stella-02`) | Patched ORB adapter (`adapter-orb-patched-02`) |
|---|---:|---:|
| Input RGB frames | 798 | 798 |
| Final poses exported | 791 frame poses (99.12%) | 49 keyframes (6.14%) |
| Reference matches | 789 | 49 |
| Unmatched exported poses / reference samples | 2 / 7 | 0 / 747 |
| Process wall time | 11.035 s | 34.307 s |
| Peak sampled process RSS | 133,218,304 bytes | 830,287,872 bytes |
| Native reported tracking mean / median | 8.119 / 8.049 ms | 12.235 / 11.672 ms |
| Per-call timing p50 / p95 / p99 | 8.048 / 10.064 / 14.108 ms (798 calls) | UNAVAILABLE; only aggregate native statistics |
| ATE translation RMSE | 0.021427 m | 0.009658 m |
| RPE translation RMSE | 0.012073 m | 0.011974 m |
| RPE rotation RMSE | 0.011733 rad | 0.012811 rad |
| Single global Sim(3) scale | 1.348148453 | 1.274921704 |

Each run uses **one global Sim(3)** across its matched final trajectory. Rotation,
translation, scale, singular values and all adjacent matched-frame RPE pairs are
retained. RPE intervals differ because ORB exports sparse keyframes. No
per-segment fitting or metric-scale accuracy claim is made. These numbers do
not establish a winner.

Tracking coverage, lost intervals and relocalization are **UNAVAILABLE** in both
stock export contracts; emitted-pose fractions above are not those metrics.
Stella's successful adapter log identifies initialization using frames 0→8;
ORB logs its first keyframes/map but supplies no per-source-frame online state
trace. Initialization evidence is retained, without inventing a complete state
stream. Stella missing final poses are not automatically labeled LOST. Both
supply final optimized output only; online trajectory remains UNAVAILABLE.

ORB sleeps according to input timestamps and renders in Xvfb; Stella runs
`--no-sleep --viewer none`. Wall time includes startup, vocabulary loading and
shutdown. RSS samples cover only the launched process, exclude Xvfb, and can
miss peaks. Native and adapter trajectories differ because these multithreaded
backends were not configured as deterministic; no byte-identical replay claim
is made. Both final variants and all raw outputs are retained separately.

## Evidence and reproduction

| A path | Content |
|---|---|
| `environment.json`, `backend-sources.json`, `build-caches/`, `build-logs/`, `build-recipe.sh` | Environment, exact pins/options/dependencies, initial failures and patch |
| `public-dataset-02/` | Canonical dependent view, published calibration and associated reference |
| `native-input/`, `native-*.yaml` | Native RGB-only association view and configs |
| `native-stella/`, `native-orb/`, `native-orb-gdb/`, `native-orb-patched/` | Exact native commands, raw logs/trajectories and results |
| `adapter-stella-01/`, `adapter-orb-patched-01/` | Preserved failed adapter attempts |
| `adapter-stella-02/`, `adapter-orb-patched-02/` | Successful run, config, raw/normalized output, benchmark and reference evaluation |
| `*-installation.json`, `*-run-config*.json` | Hash-bound installation and run inputs |
| `0921-refusal/`, `0921-final-refusal/`, `absent-*/` | Real-data calibration refusal and public-data BACKEND_NOT_INSTALLED results |
| `baseline-tests.log`, `final-tests.log`, `verify_evidence.py`, `summary.json` | 175 baseline / 182 final tests, explicit evidence checks |
| `implementation-sha256.json`, `checksums.json` | Producer files and final evidence checksums |

Use a **new output directory** on repetition. From the integration worktree:

```bash
export LD_LIBRARY_PATH=/home/shiuhou/slam-public-qualification-20260923/install/lib:/home/shiuhou/slam-public-qualification-20260923/sysroot/usr/lib/x86_64-linux-gnu
export DISPLAY=:93  # start the retained user-local Xvfb for ORB first
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.runner --config /absolute/new-run-config.json
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.evaluation --run /absolute/new-run/run.json --reference /absolute/evidence/public-dataset-02/reference.json --continuity /absolute/new-run/continuity-review.json --variant final_optimized --alignment Sim3 --out /absolute/new-evaluation
```

The retained native `command.json` files give the exact invocations of
`run_tum_rgbd_slam` and `mono_tum`. Public import is available through
`python -m rm27.perception.vision.localization.experiments.public_tum --help`.
A new run needs its own reviewed, hash-bound continuity evidence; copying the
old continuity status is insufficient. Data/build paths are local dependencies,
not a self-contained portable archive. No upstream source/data/binaries were
added to Git. Backend installation remains optional: both public absent-install
probes return `BACKEND_NOT_INSTALLED` without execution.

## Final stage status and decisions

VSL-1B **NOT_STARTED**; VSL-2 **BLOCKED_ON_CALIBRATION_CAPTURE**;
VSL-3 **NOT_STARTED**; VSL-4 **NOT_STARTED**; VSL-6 **NOT_STARTED**.
Only VSL-3P gained public backend execution evidence. Historical preparation
reports retain their original scope; this report supersedes their install-absent
claims for these particular external installations.

**Is at least one real monocular backend proven end-to-end? Yes.** The unmodified
pinned Stella path passed native execution, RM27 execution, normalization and
public-reference evaluation. ORB additionally passed with the explicit lifecycle
patch; unmodified ORB is not qualified.

**Ready to receive real RM27 data once VSL-2 passes? Yes, conditionally as an
offline research harness.** It requires a hash-bound validated calibration,
matching geometry/provenance and supported pinhole/radtan5 model, plus a valid
canonical image view. Another model requires explicit adapter support; no
intrinsics are guessed. Real-camera performance, tracking-state instrumentation
and acceptance remain untested. The next physical gate is still one measured-board
M3C + OS04A10 calibration capture with matched lens/focus/mode and held-out
validation. No physical experiment was started here.
