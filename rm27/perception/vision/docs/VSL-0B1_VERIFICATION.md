# VSL-0B.1 / VSL-2-PREP verification

Base `da396bef63ce51476a71af4b9b70abdda03eed0b`; worktree
`/home/shiuhou/Projects/rm27_drones_slam`; branch
`research/slam-vsl0b1-calibration-prep`. Changes remain uncommitted for review.
This file accompanies [semantics/status](VSL-0B1_CALIBRATION_PREP.md) and the
[physical capture handoff](VSL-2_CALIBRATION_PROCEDURE.md).

## Actual results

| Suite | Result | Scope |
|---|---|---|
| Root safe host pytest | 120 passed, 0 failed/skipped | Complete root suite, 2.93 s |
| Localization + manifest | 44 passed, 0 failed/skipped | Existing six updated for explicit v2 semantics plus 38 focused cases |
| Video analyzer + canonical view | 7 passed, 0 failed/skipped | Decode acceptance, provenance, image-write failures, timing span, dependency checks |
| Calibration capture | 25 passed, 0 failed/skipped | Diversity >24, duplicate rejection, rounded holdout, dimensions, targets, IDs, provenance, integrated write failures |
| TargetEstimate / visual guidance | 4 passed, 0 failed/skipped | Existing target schema, guidance, synthetic/truth and command boundary tests |
| Device host CTest | 3 executables passed, 0 failed/skipped | green_detector_tests (includes target/motion), async_frame_tests, async_log_tests |
| Optional VIN ABI mocks | Not configured/run | Matched SDK include tree unavailable; not counted as passing/skipped |
| 0921 offline reproduction | PASS, source hash unchanged | 13,766 declared/decoded; hardened timing/integrity gates true; no write failures |
| Historical diagnostic comparison | Exact match | timing/quality CSV, interval JSON/YAML, all 2,295 JPEGs byte-identical |
| Canonical view comparison | Exact sampled data match | frames.csv, intervals.json, subsets.json; new metadata and image hashes added |
| Printable board generation | PASS, SVG dimensions/35 black squares checked | Nominal print geometry only, not physical measurements |
| Starter bundle SHA256SUMS | All three entries OK | Updated manifest checksum after UNKNOWN timestamp-unit correction |
| Content/diff review | git diff --check clean | Historical audit copied byte-identically; no runtime target/guidance edits |

Focused Python suites are subsets of the 120 root tests, not extra tests added
to that total. All runs were headless host testing or offline encoded-video
analysis. No real calibration data, intrinsics fitting, SLAM, VIO, ExternalNav,
hardware capture/actuation or flight was run.

During development, the first capture/analyzer run had four failures: one
synthetic board was below the retained minimum-area threshold; the three
analyzer fixtures exposed a Git-status lookup that failed for sources outside
the repo. The fixture was enlarged without weakening the threshold and the
external-source provenance path was fixed. Affected suites and full root suite
were rerun; final results above supersede those intermediate failures.

Existing tests were adapted only to deliberate contract changes: unsupported
version is now 999 because v2 exists; RESET is no longer relocalizing; velocity
explicitly carries its unit; calibration references now need an ID; unverified
exposure claims are retained under the shared policy. Rejection gates were not
removed to mask failures.

## Exact commands

Initial state inspection, from the isolated worktree:

```bash
pwd
git status --short --branch
git branch --show-current
git rev-parse HEAD
git worktree list
```

The worktree was already present; no second worktree creation, reset, clean,
history rewrite or submodule update was performed. The review input was copied
unchanged from the primary workspace:

```bash
cp /home/shiuhou/Projects/rm27_drones/rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md
```

Final verification commands below ran from the isolated worktree. stdout/stderr
for named suites and analyzer commands are retained under
`artifacts/vsl0b1-calibration-prep-20260922-01/`. The source argument outside this
worktree is read-only. Device sources were read from the existing clean device
checkout; all CMake build output goes to the isolated artifact directory.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_localization_contract.py tests/test_localization_hardening.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_video_analyzer.py tests/test_canonical_dataset.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_calibration_capture.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_visual_guidance.py -q
cmake -S /home/shiuhou/Projects/rm27_drones/rm27/perception/vision/maixcam2_dart_vision/tests -B artifacts/vsl0b1-calibration-prep-20260922-01/host-build
cmake --build artifacts/vsl0b1-calibration-prep-20260922-01/host-build -j 2
ctest --test-dir artifacts/vsl0b1-calibration-prep-20260922-01/host-build --output-on-failure
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.analyze_video /home/shiuhou/Projects/rm27_drones/rm27/perception/vision/video/0921.mp4 --out artifacts/vsl0b1-calibration-prep-20260922-01/vsl1a --sample-step 6 --large-gap-factor 1.5
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.prepare_vsl2_dataset --vsl1 artifacts/vsl0b1-calibration-prep-20260922-01/vsl1a --output artifacts/vsl0b1-calibration-prep-20260922-01/canonical
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.generate_calibration_target --out artifacts/vsl0b1-calibration-prep-20260922-01/print-test/chessboard.svg --corners-x 9 --corners-y 6 --square-mm 25
/home/shiuhou/venvs/mujoco/bin/python artifacts/vsl0b1-calibration-prep-20260922-01/compare_evidence.py
git diff --check
```

Repeat artifact-producing commands with **new output directories**. Analyzer
provenance includes absolute command, pre/post source hashes, script hash and
versions. Raw 0921 SHA-256 remains
`437d72cb0030cdb33d1ea8377b2d61e234280813dbb83c85ab3c8e3e44fe50bb`.

Additional development test invocations (same interpreter/environment):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_calibration_capture.py tests/test_video_analyzer.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_canonical_dataset.py tests/test_video_analyzer.py tests/test_localization_contract.py tests/test_localization_hardening.py -q
```

From the starter package directory
`rm27/perception/vision/M3C_OS04A10_SLAM_实验起步包_v1/M3C_SLAM_experiment_plan_v1/`:

```bash
sha256sum -c SHA256SUMS.txt
```

Primary workspace final read-only checks:

```bash
git status --short --branch
git -C rm27/perception/vision/maixcam2_dart_vision status --short
git -C rm27/perception/vision/maixcam2_dart_vision rev-parse HEAD
```

The device checkout remains clean at
`c7d47b20a5a948db1e55f99fb128ee7616d02e1c`. The primary root still lists its
pre-existing nested/untracked state; it is not claimed clean. Other nested
repositories were not changed or used for implementation. In the isolated
worktree, submodule checkouts were not initialized or changed.

## Changed files and reasons

Paths below are relative to the isolated worktree. Generated evidence is ignored
and is not added to Git. This complete list includes the unchanged audit import.

| File | Reason |
|---|---|
| `README.md` | Remove stale current test count. |
| `docs/MIGRATION_PROVENANCE.md` | Label migration-era test counts historical rather than current. |
| `rm27/perception/camera/README.md` | Correct live-runtime and estimator overclaims. |
| `rm27/perception/guidance/FRAMES.md` | Document existing target bearing signs and separate localization frames. |
| `rm27/perception/guidance/README.md` | Clarify that guidance consumes TargetEstimate, not localization. |
| `rm27/perception/guidance/schema.py` | Fix only the stale frame-document reference; no semantic or behavioral change. |
| `rm27/perception/vision/M3C_OS04A10_SLAM_实验起步包_v1/M3C_SLAM_experiment_plan_v1/SHA256SUMS.txt` | Refresh only the changed starter manifest checksum. |
| `rm27/perception/vision/M3C_OS04A10_SLAM_实验起步包_v1/M3C_SLAM_experiment_plan_v1/experiment_manifest.template.yaml` | Replace unsupported ns timestamp placeholder with UNKNOWN. |
| `rm27/perception/vision/README.md` | Add surviving localization/docs navigation. |
| `rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md` | Import the supplied untracked review input unchanged. |
| `rm27/perception/vision/docs/VSL-0B1_CALIBRATION_PREP.md` | Record new semantics, limits, acceptance policy and current VSL status. |
| `rm27/perception/vision/docs/VSL-0B1_VERIFICATION.md` | Record file rationale, exact verification commands, failures/fixes and suite evidence. |
| `rm27/perception/vision/docs/VSL-0B_CONTRACT.md` | Append v2 supersession link while retaining historical v1 description. |
| `rm27/perception/vision/docs/VSL-2_CALIBRATION_PROCEDURE.md` | Replace unsupported target wording and give measured-board physical capture/STOP handoff. |
| `rm27/perception/vision/docs/VSL-2_GATE_STATUS.json` | Update preparation status without claiming calibration or SLAM. |
| `rm27/perception/vision/localization/__init__.py` | Expose scale and consumer stream-validation types. |
| `rm27/perception/vision/localization/analyze_video.py` | Derived acceptance, pre-analysis provenance, checked image export, explicit duration metrics. |
| `rm27/perception/vision/localization/calibration_session.template.json` | Provide fillable physical camera/target/build/timing provenance fields. |
| `rm27/perception/vision/localization/capture_calibration.py` | Replace 24-cell selection and holdout; measured target/session validation, IDs and checked writes. |
| `rm27/perception/vision/localization/generate_calibration_target.py` | Generate a deterministic printable chessboard with explicit nominal size. |
| `rm27/perception/vision/localization/manifest.py` | Separate metadata/frame/reference checks; format-aware stride and evidence-aware clocks. |
| `rm27/perception/vision/localization/prepare_vsl2_dataset.py` | Verify source/dependencies and record hashes without copying the image archive. |
| `rm27/perception/vision/localization/schema.py` | Versioned units/scale, stable unit rotation, types/strict JSON, state and epoch semantics. |
| `tests/test_calibration_capture.py` | Exercise >24 diverse views, duplicate rejection, deterministic holdout, measured targets, real detectors and extraction/write-failure integration. |
| `tests/test_canonical_dataset.py` | Exercise source hash mismatch, missing images and dependent-view identity. |
| `tests/test_localization_contract.py` | Adapt existing assertions to explicit version/state/timestamp/reference policies. |
| `tests/test_localization_hardening.py` | Add numerical, units, JSON, state/epoch, clock/stride and missing-calibration regression cases. |
| `tests/test_video_analyzer.py` | Exercise premature decode, write failures, provenance and nonzero-first-PTS duration. |

## Remaining issues and handoff

No current physical camera dataset or calibration exists. The user must measure
a supported board and record actual module/lens/focus/crop/build provenance.
The capture host must have the matched official driver build, SDK/media files
and configured SSH helpers; those deployment inputs are absent here. The host
selection tool's known protocol defects are resolved, but synthetic/host tests
do not establish device timing or real-image quality. Geometric fitting and
numerical residual acceptance are a later authorized pass after real data.

VSL-0A is PASS for repository audit only; VSL-0B PASS_WITH_LIMITS;
VSL-1A PASS/REPRODUCED for encoded scope; VSL-1B NOT_STARTED;
VSL-2 BLOCKED_ON_CALIBRATION_CAPTURE; VSL-3 NOT_STARTED.

The next physical task, once recorder prerequisites exist, is the measured
9×6-inner-corner chessboard session documented in the capture procedure.
Stop after capture, extraction and provenance/image review. No calibration fit
or VSL-3 follows automatically.
