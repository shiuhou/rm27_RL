# VSL-3P.1 online localization state instrumentation

2026-09-23. **Online instrumentation: PASS on the existing public sequence.**
Both adapters now retain runtime pose returns and tracking-state observations,
separately from final optimized exports. **Overall VSL-3P remains PARTIAL**:
stock ORB's lifecycle failure and unavailable map/reset/relocalization channels
remain. No accuracy improvement or flight readiness is claimed.

Worktree `/home/shiuhou/Projects/rm27_drones_slam_integration`, branch
`research/slam-integration`; starting commit `8d7e0d9`. The existing baseline
passed 182 tests. New evidence **A** is
[artifacts/vsl-3p1-online-20260923-01](../../../../artifacts/vsl-3p1-online-20260923-01/).
The original VSL-3P evidence directory was preserved: all 6,550 entries in its
checksum index match before and after this task. Its original stock failure,
patched run, final outputs and reference evaluations remain intact.

## Observation points and unchanged estimator code

The exact previously qualified pins remain in use:

| Backend | Revision | Observation |
|---|---|---|
| Stella | `e445b5452535e781fdf6777ec33a06f5f8f5e416` | Return value of synchronous `system::feed_monocular_frame`, followed by public `frame_publisher::get_tracking_state()` |
| Stella examples | `defc69eecc36e51cdda22885bb86954f08ad6887` | Copied TUM monocular example loop, CSV write before next input |
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` plus the existing lifecycle patch | Return value of `System::TrackMonocular`, followed by public `System::GetTrackingState()` |

New example drivers were built externally in
`/home/shiuhou/slam-online-instrumentation-20260923`. The builder copies the
pinned examples and adds only return-value capture, state reads and logging.
It rejects edited example inputs. It links the already qualified libraries;
**no Stella or ORB estimator library was rebuilt or modified**. Original
binaries, libraries and upstream worktree diffs match their pre-task hashes.
The ORB shutdown patch is still a separate dependency, retained in the prior
installation manifest. It is not part of either observation patch.

Stella `tracking_module::feed_frame` constructs a separate `Mat44_t` containing
`curr_frm_.get_pose_wc()` when a pose is available. `system::feed_frame` updates
the frame publisher synchronously before returning that snapshot. The public
state getter locks the publisher. The observer uses this returned snapshot,
not the map viewer's possibly retained pose. It is **T_wc**.

ORB `TrackMonocular` returns the `Sophus::SE3f Tcw` from
`GrabImageMonocular`, after copying the current tracking state under
`mMutexState`. `GetTrackingState()` reads that copy under the same mutex. The
observer copies the returned matrix by value. Its online return is **T_cw**,
although ORB's final TUM export is **T_wc**. Normalization explicitly inverts
online T_cw and checks the resulting LocalizationEstimate pose projection.
Neither backend's later optimization can rewrite these logged snapshots.

The new drivers preserve original feature settings, optimizer settings, frame
order, timestamps, Stella's headless/no-sleep mode, and ORB's paced viewer mode.
Observation and CSV flushing add overhead and can affect asynchronous scheduling;
this is not deterministic replay or an instrumentation-overhead benchmark.

## Record and normalized semantics

`experiments/online.py::OnlinePoseRecord` is a version-1 **experiment observation
envelope**, not a replacement for LocalizationEstimate. A supplied pose uses
its v2 fields in `localization_fields`: `T_parent_child`, parent
`backend_world`, child `camera`, wxyz orientation, ARBITRARY scale and arbitrary
translation units. `contract_complete` is false. It is not a controller message.

The envelope records the canonical source frame ID and original timestamp
metadata, backend revision/binary hash, raw state, normalized state,
initialization state, pose convention, validity/reason, optional pose, backend
input timestamp, observed processing completion, observation completion,
processing duration and raw CSV row reference. Optional publication time, map
ID, epoch, reset event and relocalization event remain null. Covariance and
confidence are absent; no values or quality scores are synthesized.

| Public backend state | Normalized state | Initialization observation |
|---|---|---|
| Stella `Initializing` | INITIALIZING | INITIALIZING |
| Stella `Tracking` | TRACKING | INITIALIZED |
| Stella `Lost` | LOST | UNKNOWN |
| ORB `SYSTEM_NOT_READY=-1`, `NO_IMAGES_YET=0` | UNINITIALIZED | UNINITIALIZED |
| ORB `NOT_INITIALIZED=1` | INITIALIZING | INITIALIZING |
| ORB `OK=2` | TRACKING | INITIALIZED |
| ORB `RECENTLY_LOST=3`, `LOST=4` | LOST | UNKNOWN |
| Any other value, including unused ORB `OK_KLT=5`, empty Stella state | UNKNOWN | UNKNOWN |

RECENTLY_LOST retains its distinct raw value; its recovery window does not
qualify a current tracking measurement. The pinned monocular path does not
exercise OK_KLT, so it is conservatively unmapped. Neither public getter exposes
a RELOCALIZING state. LOST→TRACKING is reported as an observed state transition,
**not** a relocalization event. Missing poses never create a LOST state.
No state transition manufactures an epoch increment, reset or map ID.

Validity requires both an observed TRACKING state and a finite rigid pose
return. A numeric ORB return during initialization can be an identity placeholder;
it is preserved with `valid: false`, and is counted separately from valid online
poses. Matrix rigidity and quaternion checks reject malformed output without
silently repairing it. LOST poses, if returned, similarly remain invalid current
tracking measurements. These semantics do not establish control eligibility.

## Timing and source association

The same original 798 RGB PNGs from TUM Freiburg1 xyz were used, with the
previously pinned public calibration and archive. No RM27 camera data or 0921
run was used. The existing calibration gates were not edited.

Raw `input_index` is the selected input ordinal, not an assumed internal backend
frame ID. The normalizer maps it to the canonical source ID and preserves the
original decimal timestamp and its `DATASET_TIMESTAMP` evidence. The CSV also
stores the double timestamp supplied to the backend at 17 significant digits.
Association requires the matching input ordinal and a timestamp within binary64
rounding tolerance; it never replaces the original source time. Stella retains
the existing, explicitly unused RGB/depth timestamp association shim.

Each CSV row contains steady-clock nanoseconds immediately before and after the
synchronous backend call, and after the state read/before serialization. These
are host processing/observation timestamps in a run-specific process clock.
Call duration excludes image loading, the state read and CSV writing. It is not
source-to-output latency. Publication time, exposure time and dataset-to-host
clock mapping are unavailable. No subtraction across those clocks is performed.
The CSV is flushed before the next frame; normalized JSON is generated afterward
from those runtime observations. No live controller transport was added.

## Artifact separation and evaluation

For each new run:

```text
raw/online.csv                 runtime backend API returns, states and clocks
raw/*trajectory*.txt           native final exports, unchanged format
online/trajectory.json         normalized runtime observations, including invalid/unknown records
online/evaluation.json         sampled online behavior only
normalized/final_optimized.json final pose normalization, existing convention
evaluation.json                existing final-output diagnostics
run.json                       distinct online_trajectory / final_optimized_trajectory paths
```

Online recording is requested with `"online": true`. An uninstrumented
installation refuses it with `ONLINE_OBSERVATION_UNAVAILABLE`; default runs
remain compatible and backend installation remains optional. Installation
manifests hash the observation sources, binary and original runtime libraries.
A successful instrumented run requires one observation per selected input.
Partial traces are retained after backend failure, with incomplete coverage;
no missing observation is inferred from a final trajectory.

Online metrics include observation coverage, numeric pose-return count, valid
pose count, observed tracking coverage, first valid/first tracking sample,
state transitions, sampled LOST intervals and per-call timing percentiles.
Coverage and initialization claims require a complete known-state trace. A
separate first-observed-valid field describes partial traces. LOST duration
uses sample-and-hold intervals on original dataset time and remains null when
samples/states are missing or a terminal loss is right-censored. It is not an
exact internal event duration. Unknown event channels remain null, including
when no such event appears in the logs.

Reference evaluation still requires an explicit `--variant`, matching artifact
header and per-record variants, plus separately reviewed continuity evidence.
For online input it uses only valid online poses and reports the excluded count.
It never falls back to final output. Online coordinate continuity remains
UNVERIFIED because these hooks do not provide map/reset epochs; no real online
ATE/RPE evaluation was claimed or run in this instrumentation task.

## Actual public-data validation

Both new adapters exited 0. Stella ran headlessly; patched ORB ran its native
viewer in Xvfb `:94`. These were offline host public-image executions.

| Measurement | Stella | Lifecycle-patched ORB-SLAM3 |
|---|---:|---:|
| Runtime observations / input frames | 798 / 798 | 798 / 798 |
| INITIALIZING observations | 7 | 3 |
| TRACKING observations | 791 | 795 |
| Valid online poses | 791 | 795 |
| Numeric pose returns including invalid placeholders | 791 | 798 |
| Sampled tracking coverage | 99.1228% | 99.6241% |
| First valid source frame (zero-based) | 7 | 3 |
| Dataset time elapsed to first valid sample | 0.235954 s | 0.100022 s |
| Sampled LOST intervals | none | none |
| Observed call p50 / p95 / p99 | 8.003 / 9.726 / 10.317 ms | 11.843 / 14.326 / 30.832 ms |
| Separately exported final poses | 792 frame poses | 50 keyframes |
| Process wall time | 10.932 s | 35.122 s |

These values describe different backend observation/export contracts, not an
accuracy or performance ranking. In particular, final Stella output includes
a retrospective initial-frame pose that was not available as a valid online
pose for that initial input. Final ORB keyframe counts cannot replace its online
frame-state coverage. Source matrices, normalized records and both final exports
remain separate and hash-addressed.

The sequence exercised initialization and tracking in both backends. It did
**not** exercise real loss/recovery. LOST, RECENTLY_LOST, unknown states, partial
traces and recovery transitions are covered by synthetic tests, explicitly not
public-run observations. Explicit relocalization remains unavailable even though
an observed LOST→TRACKING transition could be counted in a future trace.

## Changed files and checks

| File under `localization/experiments/` unless noted | Change |
|---|---|
| `observers/build.py`, `observers/online_csv.hpp` | External copied-example build and synchronous CSV observation writer |
| `online.py` | Strict observation serialization, state mapping, source association, online pose conversion and behavior metrics |
| `backends.py` | Validate declared observer source hashes and format |
| `runner.py`, `run.template.json` | Opt-in online artifacts, missing-hook refusal and partial-trace retention |
| `evaluation.py` | Reject mixed variants and explicitly filter invalid online poses for reference evaluation |
| `tests/test_slam_online.py` | 24 focused tests, including parameterized mappings |
| VSL-3P report/status, vision README, this report | Historical follow-up link and current instrumentation scope |

**206 root tests passed**, including the 24 focused tests; baseline was 182.
Tests cover serialization and absent optional fields, UNKNOWN states, mappings,
no fabricated covariance/confidence, retained timestamps, T_cw inversion,
invalid matrices, incomplete traces, variant separation, uninstrumented refusal,
failed-process trace retention and explicit online reference selection. Existing
BACKEND_NOT_INSTALLED tests continue to pass. No tests use real RM27 imagery.

The evidence verifier checks all prior artifact checksums, unchanged upstream
library/binary hashes and source diffs, exact source-image/time association,
full observation counts, successful exits and unknown fields. It re-parses each
raw CSV and obtains exactly the retained normalized records. Added explanatory
metric labels were verified afterward in `A/verification/`; the originally
emitted online metrics and all final evaluations were preserved.

## Reproduction and retained evidence

Exact runner commands executed from the integration worktree:

```bash
export LD_LIBRARY_PATH=/home/shiuhou/slam-public-qualification-20260923/install/lib:/home/shiuhou/slam-public-qualification-20260923/sysroot/usr/lib/x86_64-linux-gnu
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.observers.build --upstream /home/shiuhou/slam-public-qualification-20260923 --output /home/shiuhou/slam-online-instrumentation-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.runner --config artifacts/vsl-3p1-online-20260923-01/stella_vslam-config.json
/home/shiuhou/slam-public-qualification-20260923/sysroot/usr/bin/Xvfb :94 -screen 0 1024x768x24 -nolisten tcp
# With that virtual display running, in a separate process:
DISPLAY=:94 /home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.experiments.runner --config artifacts/vsl-3p1-online-20260923-01/ORB-SLAM3-config.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_slam_online.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
PYTHONPATH=. /home/shiuhou/venvs/mujoco/bin/python artifacts/vsl-3p1-online-20260923-01/verify_online.py
```

Repeat with new build/run output directories and updated installation hashes;
the tools reject existing output directories. Xvfb is stopped after validation.
`A/environment.json` and each run's `command` retain the exact native invocation.
`A/*-installation.json` binds the new binaries and copied-driver source hashes
while retaining the original library and ORB lifecycle-patch hashes.

A contains build logs and CMake recipe, `driver-sources/`, the two distinct
example-observation patches, upstream observation-point source copies,
`preservation-before.json`, `preservation-after.json`, raw/normalized/final run
outputs, test logs, `verification/`, `implementation-sha256.json`, and
`summary.json` with `passed: true` for this instrumentation scope. Large source,
dataset and binary dependencies remain outside Git. The original qualification
and this instrumentation evidence are separate directories.

## Status and remaining limits

Only VSL-3P status metadata is updated: overall **PARTIAL**, online
instrumentation **PASS for these public runs**. VSL-2 remains
BLOCKED_ON_CALIBRATION_CAPTURE; VSL-3, VSL-4 and VSL-6 remain NOT_STARTED.
The observation layer exposes what these public APIs returned at runtime; it
does not add a complete LocalizationEstimate stream, confidence/covariance,
map/epoch continuity, relocalization events, calibrated timing or control use.
Stock ORB's previous failure is not reclassified as success.

Real RM27 camera localization still requires:

- validated M3C + OS04A10 calibration;
- matching geometry;
- controlled dataset;
- later real-camera experiment.
