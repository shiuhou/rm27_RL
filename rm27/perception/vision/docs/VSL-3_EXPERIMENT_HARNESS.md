# Future VSL-3 offline experiment harness

Current integration note (2026-09-23): this historical preparation report used
LocalizationEstimate v1. The unified branch now includes v2 hardening and
[VSL-3P public backend qualification](VSL-3P_PUBLIC_BACKEND_QUALIFICATION.md),
including timestamp-export precision and integer ORB FPS fixes. Its explicit
public-provider calibration scope does not satisfy RM27 hardware calibration.

Implementation: **IMPLEMENTED**, Python utilities and two external-process CLI
adapters. Validation: host software tests and explicitly labeled stub executions;
upstream source contracts reviewed at pinned commits. Neither upstream backend
was installed, built or executed. **VSL-3 remains NOT_STARTED.**
Physical blocker: no validated calibration matched to camera/lens/focus/mode.
Next gate: real VSL-2 measured-board capture, fitting and held-out validation,
then a reviewed calibration/dataset association. Nothing runs on a camera or
flight controller through this package.

## Boundary and compatibility

Code lives in `rm27/perception/vision/localization/experiments/`. The flow is:
canonical dependent view → calibration gate → deterministic selection/derived
inputs → external backend → retained raw output → normalized offline output →
conditional evaluation → comparison. There is no plugin registry, active
runtime estimator, IMU integration or MAVLink sender.

This worktree starts at `da396bef63ce51476a71af4b9b70abdda03eed0b`, independently
of the uncommitted VSL-0B.1/calibration-preparation changes in `rm27_drones_slam`.
It does not import or overwrite that work. Existing LocalizationEstimate v1,
TargetEstimate, guidance, audit and stage-status files retain their semantics.
The historical audit is imported unchanged. VSL-0B stays PARTIAL in this branch.
A later integration must reconcile the separately prepared v2 contract; the
existing v1 tests still pass here.

## Canonical reader and selections

`CanonicalDataset(dataset_manifest.json)` reads the current schema-1 canonical
view (`frames.csv`, `intervals.json`, `subsets.json`). Image paths resolve
relative to `source.artifact_directory`, not to the canonical view directory.
Relative artifact/source paths resolve against the manifest directory. The
reader verifies the source video, supplied hashes, every image's presence,
actual decoded dimensions, frame order, one encoded timeline, interval bounds,
and subset membership. It records current hashes even when an old artifact has
no image-hash inventory. A newly computed hash establishes current identity,
not independently proven historical acquisition provenance. Images are not copied.

Actual 0921 subset names and counts:

| Subset | Frames |
|---|---:|
| `all_decoded_sampled` | 2295 |
| `high_motion_proxy` | 221 |
| `severe_blur` | 224 |
| `low_texture` | 30 |

There are no current `baseline`, `rotation-heavy`, `target approach` or
`exposure transition` subset entries to invent. Selecting a missing name fails.
Diagnostic subsets may contain disjoint intervals: the backend receives their
original PTS gaps. A future experiment may intentionally test gap sensitivity;
these selections are not asserted to form continuous physical motion windows.

Rate selection uses **original source frame IDs**, phase anchored at frame 0:
for nominal 180 Hz, steps 1/2/3/4/6 select 180/90/60/45/30 Hz respectively.
Each selected frame retains its original decimal PTS text. No new uniform
clock is generated. The reader rejects a requested source-frame grid if those
images are unavailable. **The existing 2295-image view is already sampled every
sixth source frame; it cannot supply 180/90/60/45 Hz.** Later extract a new
dense view from the preserved source using `analyze_video --sample-step 1`
and the canonical-view utility; this pass does not duplicate that archive.
These are offline input-rate ablations, not real-time throughput claims.

## Calibration gate and future artifact fields

Normal CLI runs always require all of:

- Dataset status other than `CALIBRATION_REQUIRED`; its calibration reference
  has `status: VALIDATED`, `calibration_id`, `file`, and exact `sha256`.
- Referenced canonical calibration schema 1 has the same ID and VALIDATED
  status; every frame references that ID. File paths are relative to the
  containing manifest. Missing files and hashes fail closed.
- Image width/height match; `camera_module`, `lens_id`, `focus_setting`,
  `camera_mode_id` and `pixel_geometry` match the dataset's `observed_geometry`.
  The matching identity/crop/resize information cannot be UNKNOWN.
- `fx`, `fy` are positive finite, `cx`, `cy` finite, and distortion coefficients
  are finite ordered values. No provisional detector values are inserted.
- A `validation` object with `status: PASSED`, `criteria_id`, `report_file`,
  `report_sha256`. The retained report must say `passed: true` and identify
  the same calibration and criteria. This records a prior VSL-2 validation;
  it does not perform one or independently authenticate an operator's claim.

These are additive fields for the existing canonical calibration template,
not a fitted artifact supplied by this pass. Example **structure only**:

```json
{"validation":{"status":"NOT_RUN","criteria_id":null,"report_file":null,"report_sha256":null}}
```

The current templates with UNKNOWN calibration still fail the gate. There is
no `--ignore-calibration` or public fixture bypass. Tests use a private identity
token and label configs, datasets, calibrations, validation reports and
installation manifests `TEST_FIXTURE_ONLY`; all resulting runs retain that
label. Normal execution rejects such labeled inputs. This is accidental-use
prevention, not protection against someone intentionally falsifying metadata.

Config conversion currently supports **pinhole + `opencv_radtan5`** with exactly
`[k1,k2,p1,p2,k3]`. Width, height, intrinsics and distortion are preserved.
Other models return `BACKEND_CALIBRATION_UNSUPPORTED`, including fisheye;
no approximation or silent rectification is attempted. A real fisheye result
would require a separately reviewed exact conversion before that experiment.
Generated YAML records source calibration ID/hash. FPS is an explicitly named
nominal backend setting, not a replacement for input timestamps.

## Pinned external adapters

Source URLs, SHA-256 values and commits are recorded in
`localization/experiments/upstream_contracts.json`. Source files fetched for
review live only in ignored artifacts; no third-party implementation is vendored.

| Backend | Reviewed source commit | Executable |
|---|---|---|
| stella_vslam | `e445b5452535e781fdf6777ec33a06f5f8f5e416` | external `run_tum_rgbd_slam` |
| stella examples | `defc69eecc36e51cdda22885bb86954f08ad6887` | same executable's example source |
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | external `Examples/Monocular/mono_tum` |

The small `BackendAdapter` provides `describe_version`, `prepare` and
`collect_outputs`; shared `execute_process` runs argument arrays without a
shell, captures stdout/stderr, enforces timeout and measures host resources.
Absent installation manifest/executable gives **BACKEND_NOT_INSTALLED**.
Unsupported source revision gives BACKEND_VERSION_UNSUPPORTED.

An operator-supplied external installation manifest contains `backend`, absolute
`binary`, `binary_sha256`, absolute `vocabulary`, `vocabulary_sha256`, `commit`
and, for stella, `examples_commit`. A hash binds the executable bytes to the
manifest; it does not prove the binary was built from the claimed source.
Record build/dependency flags as additional installation metadata. The adapters
do not assume a nonexistent universal `--version` switch.

Optional future checkout, **not executed here**, outside the RM27 root:

```bash
git clone https://github.com/stella-cv/stella_vslam.git /absolute/external/stella_vslam
git -C /absolute/external/stella_vslam checkout --detach e445b5452535e781fdf6777ec33a06f5f8f5e416
git clone https://github.com/stella-cv/stella_vslam_examples.git /absolute/external/stella_vslam_examples
git -C /absolute/external/stella_vslam_examples checkout --detach defc69eecc36e51cdda22885bb86954f08ad6887
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /absolute/external/ORB_SLAM3
git -C /absolute/external/ORB_SLAM3 checkout --detach 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
```

Follow each pinned upstream build/dependency instructions on the intended host;
no CUDA/NPU or installed package is assumed. Verify licensing and record the
vocabulary source/hash. This is an external build task, not a hidden prerequisite
for the host fixture tests.

**Stella:** `run_image_slam` increments timestamps by `1/fps`, so it is not used.
The pinned TUM program has an explicit monocular branch that reads RGB pixels
and their timestamps. Its shared input parser still requires `depth.txt`.
The adapter writes identical RGB timestamp/path aliases there, with a retained
note that they are an unused association shim, **not depth measurements**.
Identical associated times preserve PTS. `Camera.setup=monocular`, `--viewer
none`, `--frame-skip 1`, `--no-sleep`, `--auto-term` and `--eval-log-dir` are
explicit. The actual backend has not been built/tested here; review this shim
at the first installation smoke test before any RM27 result is accepted.

**ORB-SLAM3:** native `mono_tum vocabulary settings sequence` reads `rgb.txt`,
uses MONOCULAR, and saves `KeyFrameTrajectory.txt` after shutdown. Its stock
example enables the viewer and sleeps between original timestamps. It therefore
needs suitable viewer/display support; its wall-clock time must not be compared
to stella's unpaced runtime as equivalent throughput. The adapter deliberately
does not rewrite upstream algorithms or silently claim all-frame output.

## Output convention and evidence

Raw backend files are retained under `raw/`, apart from normalized JSON under
`normalized/`. Both pinned exporters emit **T_wc** (camera coordinates to world)
and TUM quaternion order **xyzw**. Verified source evidence:
stella `trajectory_io.cc` explicitly inverts `cam_pose_cw` before exporting;
ORB `SaveKeyFrameTrajectoryTUM` calls `GetPoseInverse()`.
The normalizer reorders quaternion components to **wxyz** and uses the existing
RM27 parent/child convention. A separate tested inversion helper handles
explicit T_cw input; no direction is guessed. Non-unit/nonfinite quaternion
input is rejected, and each output timestamp must match exactly one selected
frame within 1 microsecond (ORB's six-decimal timestamp serialization).

Each normalized row has source ID/original PTS and evidence, raw line reference,
variant/scope and a **partial projection of LocalizationEstimate fields**:
`parent_frame`, `child_frame`, `translation`, `orientation_wxyz`, `valid`.
It uses no competing PoseEstimate class. `contract_complete: false` prevents
mistaking it for a complete producer message. `materialize_estimate` invokes the
existing LocalizationEstimate only when a caller supplies actually observed
missing state, publish/source timestamps and epoch. Default TUM outputs cannot
do that. Backend state/map/reset/processing-time fields remain null if absent;
no covariance/confidence is invented. Monocular scale is explicitly ARBITRARY
in the offline envelope, translation unit `arbitrary`.

`valid` here establishes a finite emitted pose only. It does not establish
online tracking, frame continuity, freshness, metric accuracy or flight safety.
Stella final frame trajectories omit lost frames. ORB emits final **keyframes
only**, potentially across maps without identifying them. Neither stock output
establishes online state or reset continuity. Whole-stream tracking coverage,
initialization, lost intervals and relocalization counts remain unavailable
unless an explicit complete state observation stream is supplied to the
behavior evaluator. Partial state logs are labeled PARTIAL.

`online_trajectory` and `final_optimized_trajectory` are separate run fields.
Current adapters provide only the latter. The normalizer/evaluator supports
both variants but never substitutes final loop-corrected poses for online data.

## Evaluation and comparison

Behavior evaluation records input/observed processed counts, emitted-pose
fraction (explicitly not tracking coverage), available state events, per-frame
processing statistics, wall time, failure and resource measurements. Missing
metrics are null. Stella's unmodified `track_times.txt` permits processing-call
timing; ORB's stdout summary is retained raw, not promoted to per-frame data.
There is no fabricated positional accuracy: default is **NO_GROUND_TRUTH**.

Reference evaluation supports one **global Sim(3)** Umeyama alignment for
monocular shape, or **SE(3)** with scale fixed to 1 for future metric VIO. It
computes translation ATE RMSE, adjacent matched-pose translation/rotation RPE,
duration, aligned distance, unmatched counts, singular values and alignment
parameters. It refuses static/collinear alignment and mixed variants/epochs.
RPE pair IDs expose gaps; segments are never independently rescaled.

The reference uses the same `localization_fields` projection and source IDs/PTS,
with `reference_status: VERIFIED`, `timestamp_association: VERIFIED`, dataset
ID/manifest SHA-256 and `translation_unit: m`. A separate continuity report
must bind `run_config_sha256`, `normalized_sha256`, `reference_sha256` and
`status: VERIFIED_SINGLE_COORDINATE_FRAME`. The stock exports do not supply this
proof themselves; unknown multi-map continuity must block ATE rather than be
hidden by alignment. Reference evaluation is a separate artifact:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.evaluation --run /absolute/run/run.json --reference /absolute/reference.json --continuity /absolute/continuity.json --variant final_optimized --alignment Sim3 --out artifacts/new-reference-evaluation
```

The deterministic comparator requires matching dataset manifest, selected frame
IDs and image hashes, calibration, timestamps and transform configuration. It
includes installation versions, failures, timing/resources and available
reference metrics, without automatic ranking. Identical selected images do
not make the two programs' pacing/timing instruments equivalent.

## Future commands and derived experiments

From `/home/shiuhou/Projects/rm27_drones_slam_future`, once physical gates pass,
fill `localization/experiments/run.template.json` with absolute existing paths
and a new output directory. Run-config paths resolve from the working directory;
calibration references resolve from their manifest directory. Then:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.runner --config /absolute/run-config.json
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.compare --runs /absolute/stella/run.json /absolute/orb/run.json --out artifacts/new-comparison
```

Optionally add `--reference-evaluations /absolute/reference_evaluation.json ...`
to comparison. Reports retain online/final variant labels and reference metrics
separately. A run reaching process completion is **EXECUTED**, not automatically
PASSED or VSL-3 PASS. Refusals/failures are saved in `run.json`; no overwrite of
an existing output directory is allowed.

`transforms` accepts `grayscale`, `scale` (0 < value <= 1), `blur_sigma` (0..20
pixels), `brightness_gain`, `brightness_offset`, and optional `mask` reference.
Order: grayscale → OpenCV linear resize → Gaussian blur → affine brightness
and clipping → binary mask. Downscale updates intrinsics using pixel-center
mapping `(coordinate+0.5)*scale-0.5`; actual rounded dimensions and independent
x/y scales are recorded. Distortion stays unchanged. These are derived research
stressors, not validated exposure, rolling-shutter or sensor-physics models.
No transforms uses symlinked input images; transforms write new PNGs, hashes
and `derived_inputs.json`, leaving canonical sources untouched.

`native_mask` defaults to null. The pinned TUM CLI interfaces do not expose
native masks and refuse non-null native_mask with BACKEND_MASK_UNSUPPORTED.
An explicitly reviewed mask may instead be used as the **image-domain masking
ablation**, with `status: VALIDATED`, absolute file, SHA-256, source geometry and
binary 0/255 values. Blacking pixels is not guaranteed feature exclusion; mask
boundaries can create features. **No final prop/body/wiring mask was guessed.**

Exact next physical action: use the separately hardened capture worktree's
measured **9×6-inner-corner chessboard** procedure, fixed M3C + OS04A10 full180
1344×760 lens/focus, 30–50 diverse views and >=20% holdout. Verify the matched
device build/SDK first. Stop after capture and provenance/image review. This
worktree's original capture tool is not silently upgraded by this preparation
pass. Calibration fitting and first backend experiment need subsequent work.
