# Future localization infrastructure implementation report

Completed 2026-09-23 in `/home/shiuhou/Projects/rm27_drones_slam_future`, branch
`research/slam-future-infrastructure`, base
`da396bef63ce51476a71af4b9b70abdda03eed0b`. The dedicated linked worktree already
existed and was clean at the start. Changes remain uncommitted for review.
No reset, clean, history rewrite, submodule update or primary-workspace edit was
performed. The prior calibration-preparation worktree remains independent.

## Delivered implementation and limits

The canonical reader verifies dependencies, preserves original PTS, selects
actual diagnostic subsets and rejects rate selections that require unavailable
frames. The fail-closed calibration gate requires a matching validated artifact
and hashed prior validation report. The runner records identities/configuration,
versions, environment, selection, commands, lifecycle and output hashes.

Two pinned-source external adapters generate exact supported pinhole/radtan5
configs, input sequence links and commands, collect outputs and report missing
installations. Stella uses its TUM monocular path to preserve timestamps rather
than its uniform-time image runner. ORB's native output is final keyframes only.
Both exporters' T_wc/xyzw conventions were verified against upstream source.
Normalized rows reuse existing LocalizationEstimate field names as explicitly
incomplete offline projections; no unknown state, epoch, publish time,
confidence or covariance is fabricated to construct a complete producer record.

Evaluation implements observable behavior metrics, explicit missing evidence,
one global Sim(3) or SE(3) alignment, ATE/RPE, continuity/reference checks and
online-versus-final separation. Comparison requires matching experimental inputs
and produces deterministic JSON/Markdown with no automatic ranking. Derived
image transforms record their parameters, calibration coordinate mapping and
hashes while preserving the canonical inputs. Native masks unsupported by the
pinned CLIs are rejected; an explicit validated mask can be an image-domain
stress ablation, never a guessed final mask.

Host external-process benchmarking records outcomes, timing and sampled resource
use; raw IMU record validation and UNKNOWN camera–IMU references prepare future
VIO evidence without integrating it.

**Only documented/planned:** third-party installation/build steps, real camera
model validation, fair equivalent-pacing backend benchmarks, device deployment,
board temperature/throttle/queue/map telemetry, raw-IMU acquisition and time
synchronization, spatial/temporal camera–IMU calibration, VIO integration and
future ExternalNav qualification. No control wire adapter exists here.

## Actual validation

| Suite/check | Result | Evidence scope |
|---|---|---|
| New infrastructure tests | **55 passed**, 0 failed/skipped, 1.30 s | Reader/gate, adapters/configs, transforms, conventions/parsing, evaluation, comparison, host measurements and VIO sample contract |
| Complete root pytest suite | **105 passed**, 0 failed/skipped, 2.84 s | Original 50 plus 55 new tests; not 105 localization tests |
| Existing localization + visual guidance | **10 passed**, 0 failed/skipped, 0.05 s | Six original localization and four TargetEstimate/guidance regressions |
| Device host CTest | **3 executables passed**, 0 failed/skipped | green_detector_tests, async_frame_tests, async_log_tests; stub-based host build |
| Optional VIN SDK ABI mocks | Not configured/run | Matched SDK include tree unavailable; not counted as passing/skipped |
| Labeled adapter demonstration | Both stub processes completed; comparison generated | TEST_FIXTURE_ONLY, predetermined poses, not installed upstream SLAM |
| Existing 0921 canonical view inspection | 2295 images validated; actual four subsets match | Read-only source/image verification; no SLAM invocation |
| Real-data calibration gate | Refused with CALIBRATION_REQUIRED | Correct fail-closed result, not a failed SLAM experiment |
| 180 Hz request on current sampled view | Refused with RATE_UNAVAILABLE | No invented intermediate images or timestamps |
| Host benchmark CLI | Process completed; result serialized | Explicit TEST_FIXTURE_ONLY Python sleep/print command, not M3C measurement |
| Diff/content and preservation checks | Passed | Original contracts/guidance/stage documents unchanged; imported audit byte-identical |

Focused tests are included in the 105 root tests. This branch intentionally does
not incorporate the earlier worktree's additional hardening tests; its 105 count
must not be confused with that separate branch's 120 count.

All executed validation is **headless host software testing**, read-only
canonical-image inspection or mock/local process measurement. No physical
capture, calibration fit, real estimator run, live-view/closed-loop SITL flight,
recorded-motor replay, hardware actuation or flight testing occurred.

The first standalone evidence-script invocation failed on a Python import path
before creating a dataset or starting a backend. Its local script was corrected
to include the repository root and tests directory; the rerun completed. Product
test suites passed. No gate was relaxed to obtain a passing result.

## Evidence and exact commands

Evidence is in `artifacts/slam-future-prep-20260923-01/` inside this worktree.
`summary.json` is a **software-preparation verification summary**, not a stage
PASS. `canonical-inspection.json` records source/metadata/image identity and
correct refusal outcomes. The source MP4 retains SHA-256
`437d72cb0030cdb33d1ea8377b2d61e234280813dbb83c85ab3c8e3e44fe50bb`.
`fixture-demo/` and `fixture-comparison.*` are explicitly synthetic.
Upstream reviewed source bytes and hashes are in `upstream/`, ignored by Git.
No full external repository, binary, vocabulary or camera dataset was installed.

Initial isolation inspection:

```bash
git status --short --branch
git branch --show-current
git rev-parse HEAD
git worktree list
```

Review input copied unchanged:

```bash
cp /home/shiuhou/Projects/rm27_drones/rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md
```

Upstream reference discovery (read-only; exact returned commits are pinned in
`experiments/upstream_contracts.json`):

```bash
git ls-remote https://github.com/stella-cv/stella_vslam_examples.git HEAD
git ls-remote https://github.com/stella-cv/stella_vslam.git HEAD
git ls-remote https://github.com/UZ-SLAMLab/ORB_SLAM3.git HEAD
```

Source-review downloads used Python urllib against the exact commit-specific
raw URLs retained in `upstream_contracts.json`; no checkout/build was executed.
Neither `command -v run_tum_rgbd_slam` nor `command -v mono_tum` found an executable.
This establishes absence from PATH; it is not an exhaustive search of every disk.

Final validation commands, working directory this worktree:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_slam_future_dataset.py tests/test_slam_future_backends.py tests/test_slam_future_evaluation.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_localization_contract.py tests/test_visual_guidance.py -q
cmake -S /home/shiuhou/Projects/rm27_drones/rm27/perception/vision/maixcam2_dart_vision/tests -B artifacts/slam-future-prep-20260923-01/device-host-build
cmake --build artifacts/slam-future-prep-20260923-01/device-host-build -j 2
ctest --test-dir artifacts/slam-future-prep-20260923-01/device-host-build --output-on-failure
/home/shiuhou/venvs/mujoco/bin/python artifacts/slam-future-prep-20260923-01/verify_infrastructure.py
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.benchmark --out artifacts/slam-future-prep-20260923-01/host-process-benchmark --timeout-s 2 -- /home/shiuhou/venvs/mujoco/bin/python -c 'import time; print("TEST_FIXTURE_ONLY"); time.sleep(0.2)'
git diff --check
```

Use new output directories when repeating artifact-producing commands. The
standalone verification script contains its exact labeled fixture configuration;
runner `run.json` files retain full command arrays, module hashes and environment.
All shell suite outputs are saved in named logs in the evidence directory.

Final primary-workspace checks were read-only:

```bash
git status --short --branch
git -C rm27/perception/vision/maixcam2_dart_vision status --short
git -C rm27/perception/vision/maixcam2_dart_vision rev-parse HEAD
```

The device checkout used for host tests is clean at
`c7d47b20a5a948db1e55f99fb128ee7616d02e1c`; build files are isolated in this
worktree's artifacts. The primary root is **not clean**: its nested/untracked
state and concurrent unrelated contact-research additions remain untouched.
Other nested repositories were not modified and are not claimed clean.

## Added/changed files

All paths are relative to this worktree. The experiment code is under
`rm27/perception/vision/localization/experiments/`.

| File | Purpose |
|---|---|
| `experiments/__init__.py` | Offline-only package boundary |
| `experiments/common.py` | Strict JSON, errors, hashes, environment, private labeled-fixture gate |
| `experiments/dataset.py` | Canonical reader, dependency checks, subsets and original-ID rate selection |
| `experiments/calibration.py` | Fail-closed matched calibration/validation-report gate |
| `experiments/backends.py` | Pinned external CLI preparation, exact config generation and output collection |
| `experiments/transforms.py` | Derived grayscale/resize/blur/brightness/mask experiments and provenance |
| `experiments/trajectory.py` | Explicit pose convention, TUM association and existing-contract projection |
| `experiments/evaluation.py` | Conditional behavior, global alignment/ATE/RPE, evidence-bound reference CLI |
| `experiments/benchmark.py` | Host process replay measurement, timeout/resources/quantiles/clock-aware age |
| `experiments/runner.py` | Versioned ExperimentRun lifecycle and normalized artifacts |
| `experiments/compare.py` | Deterministic matched-input comparison JSON/Markdown |
| `experiments/vio.py` | Future raw-IMU sample contract validation, no estimator |
| `experiments/run.template.json` | Non-runnable placeholder for a future real experiment configuration |
| `experiments/vio_calibration.template.json` | UNKNOWN extrinsic and clock relation placeholders |
| `experiments/upstream_contracts.json` | Reviewed source commits/URLs/hashes and actual CLI/output limits |
| `tests/slam_future_fixtures.py` | Explicitly synthetic images, calibration/report and executable stubs |
| `tests/test_slam_future_dataset.py` | Dependency, selection/rate, provenance and calibration refusal cases |
| `tests/test_slam_future_backends.py` | Missing backends/configs, command execution, normalization and comparison |
| `tests/test_slam_future_evaluation.py` | Inversion, malformed outputs, alignment, events, transforms, benchmark and IMU tests |
| `rm27/perception/vision/README.md` | Navigation to future infrastructure documents only |
| `docs/SLAM_RECOVERY_AND_AUDIT.md` | Byte-identical import of the untracked review input |
| `docs/VSL-3_EXPERIMENT_HARNESS.md` | Architecture, exact future commands, conventions, gates and limitations |
| `docs/VSL-4_M3C_BENCHMARK_PLAN.md` | Implemented host format versus planned hardware measurement |
| `docs/VSL-6_VIO_DATA_REQUIREMENTS.md` | Optional raw IMU, calibration evidence and future ExternalNav gates |
| `docs/FUTURE_INFRASTRUCTURE_STATUS.json` | Preparation status separately from unexecuted stages |
| `docs/FUTURE_INFRASTRUCTURE_REPORT.md` | This completion, verification and handoff report |

In the table, `experiments/` and `docs/` abbreviate paths inside the vision unit;
`tests/` is root-level. The machine-readable full paths and final hashes are in
`artifacts/slam-future-prep-20260923-01/changed-file-sha256.json`.

## Remaining blockers and next action

- **Without calibration:** real RM27 geometric SLAM, tracking robustness,
  supported physical lens-model conversion and accuracy cannot be tested.
  Pinhole-only converters explicitly block other models.
- **Without installed upstream backends:** real CLI/build/library compatibility,
  TUM association behavior and export completeness remain unexecuted. Stub
  tests verify orchestration, not upstream algorithms. Vocabulary files and
  dependency/build manifests are external inputs, not supplied here.
- **Without M3C hardware:** sustained processing rate, memory/thermal behavior,
  throttling, queue latency, sensor timing, device stability and resource growth
  cannot be qualified. No acceleration benefit is assumed.
- **Without raw IMU data:** sample quality, bias/noise, saturation, camera–IMU
  synchronization/extrinsics, metric VIO and fusion cannot be tested.
- **Without reference and continuity evidence:** no positional accuracy, metric
  scale accuracy or fair ATE/RPE claim. Final optimized/keyframe trajectories
  do not establish online behavior or map continuity.

Preparation status: **VSL-3_INFRASTRUCTURE = IMPLEMENTED;
VSL-4_BENCHMARK_HARNESS = IMPLEMENTED; VSL-6_DATA_CONTRACT = IMPLEMENTED**, each
with host tests. No upstream backend is BUILT/EXECUTED here. No hardware or
flight test was performed. Preserve **VSL-1B NOT_STARTED; VSL-2
BLOCKED_ON_CALIBRATION_CAPTURE; VSL-3/4/6 NOT_STARTED**. No historical report or
VSL-2_GATE_STATUS.json was rewritten.

**Exact next physical action:** return to the separately prepared
`/home/shiuhou/Projects/rm27_drones_slam` capture workflow. Generate/print the
9×6-inner-corner (10×7-square) chessboard at actual size, mount flat, measure both
printed axes and enter actual metre dimensions and physical-board ID. Confirm
the matched official device build/SDK/SSH helpers, record actual lens/focus/crop
provenance, then bench-record the fixed M3C + OS04A10 **full180 1344×760** camera
with 30–50 sharp views spanning center/edges/corners, distance, tilt and roll.
Keep lens/focus unchanged and retain raw recordings/logs/hashes and >=20%
deterministic holdout. The exact capture/extraction commands are in that
worktree's `rm27/perception/vision/docs/VSL-2_CALIBRATION_PROCEDURE.md`.

**STOP after real capture and provenance/image review.** Do not start fitting,
SLAM, VIO, ExternalNav, M3C deployment or flight automatically.
