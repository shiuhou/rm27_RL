# OpenVINS public baseline report — 2026-10-07

## 2026-10-08 repeatability study — completed; VIO-P PARTIAL

`NATIVE_REPEATABILITY.md` records 10 frozen native runs and 3 independent
configuration controls, all clean exits. Baseline ATE mean 6.608 cm, sample SD
0.689 cm, range 5.432--7.653 cm; 15/45 pair differences exceed 1 cm.
RPE remains 4.440--4.583 cm / 0.47233--0.48078 deg. All runs save 2800 poses.
The existing fixed seed plus OpenCV single-thread/publisher-off controls do not
remove variation. Root cause remains unproven; early post-initialization
divergence and async lifetime/timing evidence guide further diagnosis.
The gate lacks support as a guaranteed native repeatability bound or an adapter
correctness discriminator, but is NOT changed. Same-run conversion equality
remains distinct from cross-run variation. No estimator/adapter/data edits.

## Historical 2026-10-08 in-progress checkpoint — superseded above

See `NATIVE_REPEATABILITY.md`. Frozen sequential native cohort targets ten
runs, followed by a separately labeled existing-configuration control. Initial
two results ATE6.3969/6.7431 cm, 2800 poses each; not a final distribution.
No adapter/math/data/threshold change. Prior PARTIAL gate remains in force.

## Latest runtime integration — VIO-P PARTIAL

`OPENVINS_ADAPTER.md` documents full live adapter run03, input/output exactness,
frame/time/velocity/validity semantics, failures and tests. Runtime integration
completed; four processes exit 0; 2912 images/29120 IMU inputs exact; 2800 poses
and 27980 propagated states agree with independently recorded raw messages.
Same-run native and adapter SE3 metrics match exactly. ATE 5.7385 cm, 1s RPE
4.4005 cm /0.476994 deg, path bias -1.50394%. No scale correction.

Historical native02 ATE consistency fails the predeclared gate: 1.2729 cm
difference exceeds 1 cm. Therefore no overall VIO-P PASS is claimed.
Native-only control03 also varies (ATE6.2403 cm) with clean exit and 2800 poses;
it does not replace the frozen baseline or waive the gate. Prior
native/lifecycle PASS remains scoped to that earlier work. Tests: 226 pass/1
skip, independent ROS2 7/7. Unknown state/time fields remain incomplete v2
projections; no operational or flight qualification. No hardware or push.

## Latest continuation — native numerical evaluation

See `NATIVE_SE3_EVALUATION.md`: timestamp association and independent SciPy
cross-check passed on clean run02. 2780 poses in GT overlap; ATE RMSE
0.0701139912 m; 1s RPE 0.0452377799 m / 0.477985477 deg. SE3 scale fixed 1.
Path-length ratio 0.980585754 reported separately, never applied. Original
orientation-truth caveat and explicit reference-only normalization retained.
Native evaluation integrity PASS; broader VIO-P PARTIAL pending adapter runtime
and replay parity. Initial adapter projection preserves all 2800 raw message
poses/times but deliberately cannot claim complete LocalizationEstimate v2.
208 regression tests pass/1 ROS skip; separate ROS2 suite 7/7 pass.
No hardware, Sim3, estimator changes, commit, push or Vault updates. The
lifecycle-only scope/pending-evaluation statements below describe prior work.

## Current result — supersedes historical blockers below

See `ROS2_LIFECYCLE_FIX.md` for gdb root cause, patch, tests and fresh full run.
**pinned OpenVINS + explicit ROS2 lifecycle compatibility patch** now passes
the current request's native execution/lifecycle gate. Gdb observed that global
visualizer destruction occurred after Fast DDS factory destruction. Moving only
the ROS2 entry-point owners into main fixes that order; three algorithm-library
hashes remain unchanged. Ten no-data shutdowns pass; the full sequence again
saves 2,800 online poses, with native/player/recorder all returning 0.
`ASL_DATASET_INTAKE.md` and `ROS2_NATIVE_PROGRESS.md` preserve input validation,
the pristine Jazzy header failure and the previous header-only shutdown failure.
This is not unmodified-upstream success. Adapter and SE3 evaluation are pending
and were explicitly excluded from this lifecycle task.
Regression: 190 passed / 1 ROS2-specific skip; all 7 transport tests passed in
the separate sourced Jazzy run. No host package/config, main workspace or Vault changes.

## Subsequent ROS2 review

The original ROS1 transport choice is superseded by the user's ROS2-native
preference, not by an RM27 architecture migration. See `ROS2_DATASET_REVIEW.md`:
pinned native ROS2 source exists; its build/runtime are unverified. Official
Drive page and download endpoint returned HTTP 404 through the explicit existing
proxy. No bag content, calibration association or reference data was validated.
No conversion or VIO algorithm change. Older logs/paths below describe the first
attempt; evidence now resides at `/home/shiuhou/Projects/rm27-vio-20261007`.

## Status

**VIO-P: PASS — scoped to the current native public-sequence/lifecycle request.**
The original broader adapter/parity/SE3 plan remains PARTIAL; this PASS does
not cover those unperformed tasks, metric accuracy, M3C performance or flight.
**VIO-S: PARTIAL — source audit/preparation implemented; no verified raw IMU stream.**
VIO-R, VIO-B, VIO-I and VIO-M: **NOT_STARTED**.

This is a partial delivery, not fulfillment of the entire approved plan. Native
qualification is an explicit prerequisite for writing the runtime adapter.
Network failures were not replaced by synthetic execution or a different dataset.

The ROS2 review and first-attempt sections below are historical. Their missing
dataset/build/connectivity statements are superseded by the current result
and the handoff's later read-only board inventory note.

## Workspace and baseline

Host `shiuhou@10.4.135.84`, Ubuntu 24.04 x86_64, 16 logical CPUs, 31 GiB RAM.
Research worktree `/home/shiuhou/Projects/rm27_slam_vio_openvins`, branch
`research/vio-openvins-baseline`.

Merged host base `acc70e34ed08f5bbc315391f5c5f2d08d7628a48` with local Windows
`b8e7299cf090f5150f9f262ebd3b232a824cc526` via Git bundle. Merge commit
`13c7a77bbe0fd256e6765efd4459fbca3426e0b7`. The only conflict was the fake backend
fixture launcher; both platforms' support was retained. Subsequent changes are
uncommitted working-tree state, not a published release.

Baseline command:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q`
from the research worktree: **163 passed**. With new IMU tests: **184 passed**.
No old stage gates were weakened or rewritten. Both primary checkouts were
preserved, no mainline merge/push, no Vault changes.

## External provenance

Evidence root: `/home/shiuhou/rm27-vio-20261007/`.

| Input | Frozen evidence |
|---|---|
| OpenVINS | https://github.com/rpng/open_vins at `69488123ed9362dd44b6f28e7f4680abbff1442b` |
| Source retrieval | official GitHub commit archive, not a completed Git checkout |
| Archive SHA256 | `283dde4096d9380956f6a9c8ed7e54e2d1303ee64af002d9a97f9b2196e677f4` |
| Source patches | none |
| Docker base | ROS Noetic/Focal amd64, digest `sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d` |
| Qualified build image | unavailable: dependency build canceled after repeated download errors |
| Compiler/dependency/binary manifest | unavailable: no completed build; do not substitute host package versions |
| EuRoC V1_01_easy bag/hash | unavailable: download did not succeed |

Selected local input snapshots are in `inputs/`, with `SHA256SUMS.txt`. They are
not automatically committed and no full recordings were transferred:

- `new_plan.md`: `dda97ad4afc2918975d19d6c06c150db3ad9ca565297b0ff950457001a9676e4`.
- User attachment: `d14a2479e374c9df53b2652f66f7392badccddc731e56369c8e0a7af3adc385a`.
- OS04A10 calibration: `1ef473ffb4dbf024ff682b41907e6f499397199168fa49dc82fd9890365191f8`.

The calibration snapshot is historical camera geometry evidence, not EuRoC
calibration or camera–IMU calibration. Its geometric-check status is preserved.

## Failed attempts and stop boundary

- Initial Docker pull: anonymous-token TLS handshake timeout (`docker-pull.log`).
  Retry succeeded (`docker-pull-second.log`); the pinned base image remains.
- Full/shallow Git clones: unexpected EOF/index-pack failure (`clone.log`,
  `clone-retry.log`). Official commit archive retrieval succeeded instead.
- Official EuRoC HTTPS and HTTP URLs, including bounded direct IPv4 attempts:
  SSL timeout/connection close/empty reply. Windows HTTPS also timed out.
  Logs: `download-bounded.log`, `download-http.log`, `download-direct.log`,
  `download-http-direct.log`. Scoped host search found config/truth files, not
  an available V1_01 image+IMU bag; these were not treated as a full dataset.
- Container dependency build encountered repeated package connection failures,
  including libbluray2, libtesseract4, libopencv-contrib4.2, objdetect and photo
  development packages. After these failures and the independent dataset block,
  the task's buildx process was deliberately interrupted; `docker-build.log`
  ends in CANCELED/context canceled. This was **not a compiler failure** and
  **not a completed image**. No task container remains running.
- M3C SSH timed out; no sensor inventory result inferred from that absence.

No proxy/system/network configuration was changed to work around these failures.
The native command and pinned recipe are retained under `tools/openvins/`.

## Results that are and are not available

Implemented: strict raw-IMU sequence checks, static descriptive statistics and
overlapping Allan deviation, metadata template and evidence-qualified audit.
The 21 new tests are explicitly synthetic. An additional receipt-clock regression
failed before the acquisition-time prerequisite was added.

Native runtime result, adapter parity, ATE/RPE/scale, initialization time, pose
coverage, visual/propagation/publication rates, CPU and peak RSS: **UNAVAILABLE**.
The prior M3C optical-flow benchmark is not reused as VIO evidence.

Pinned source findings include optional truth initialization, separate state and
publication timestamp semantics, JPL quaternion convention and the documented
V1_01_easy original-orientation-truth limitation. These are recorded in
`RM27_VIO_DATA_CONTRACT.md`, not claimed as tested adapter semantics.

## Resume

Obtain an accessible copy of the exact V1_01_easy bag with provenance and restore
container package access. Finish the pinned image/native build, record package
and binary manifests, and run native mono+IMU with no estimator `path_gt`.
Only then implement the RM27 adapter/EuRoC loader and parity/evaluation tests.
Do not change historical PASS criteria or start M3C deployment to bypass this gate.

Exactly one next physical action: restore M3C power/network reachability at
`10.18.198.1` for read-only inventory, as detailed in `IMU_SOURCE_AUDIT.md`.
