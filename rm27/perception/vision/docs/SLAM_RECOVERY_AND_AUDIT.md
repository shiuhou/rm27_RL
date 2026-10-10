# RM27 SLAM recovery and independent audit

Audit date: 2026-09-22. Root commit: `da396bef63ce51476a71af4b9b70abdda03eed0b`.
Device component: `c7d47b20a5a948db1e55f99fb128ee7616d02e1c`.

The localization foundation survived. It is useful for continued offline research,
but its validators and calibration preparation are not yet reliable acceptance
gates. The encoded-video results reproduce exactly within the original OpenCV
measurement scope. No matched camera calibration, localization trajectory,
metric accuracy result, or ExternalNav implementation was found.

This report is the only new product documentation. No implementation, target
contract, guidance behavior, hardware configuration, or historical artifact was
changed. Proposed fixes below are **not implemented**. All execution was
headless host testing or offline encoded-video analysis; no live-view flight,
closed-loop SITL flight, recorded-motor replay, or hardware capture was run.

## Audit boundary and evidence index

Initial commands were `pwd`, `git status --short --branch`,
`git branch --show-current`, `git rev-parse HEAD`, and `git worktree list`.
The working directory was `/home/shiuhou/Projects/rm27_drones`, branch
`astra/research`, tracking `origin/main`; the only worktree was this directory.
There was no local `main` branch or separate recovery/review worktree. This is
the primary workspace, but not a checked-out `main` branch. Investigation was
read-only except for this requested report and new ignored evidence. Use a
separate linked worktree before implementing the recommendations.

Pre-existing root status included dirty baseline/ArduPilot/legacy submodules,
an untracked field `.venv`, `pelican_bicycle.html`, `references/BEE_NAV/`,
`references/realcontroller/`, and `rm27/perception/vision/video/`. The device
vision submodule was separately checked and clean. Other nested repositories
were **not** clean; their exact states are retained in the audit evidence.
No reset, clean, checkout, rebase, history rewrite, deletion, or garbage
collection was performed.

New evidence directory, abbreviated **A** below:
[`artifacts/slam-recovery-audit-20260922/`](../../../../artifacts/slam-recovery-audit-20260922/).
Paths in tables are repository-relative unless explicitly marked device-relative.
The document is inside the existing VSL documentation directory because this is
camera-localization evidence, not a new root runtime layer.

| Evidence | Contents and limits |
|---|---|
| `A/environment.json` | Python 3.12.3, OpenCV 5.0.0, NumPy 2.5.1, Linux x86_64, interpreter and root commit |
| `A/file-provenance.csv` | Every localization module, VSL document/template, starter file and contract test; first/last reachable commit and current SHA-256 |
| `A/all-history.txt`, `path-history.txt`, `content-history.txt`, `reflog.txt` | Reachable root history, path additions, keyword history, reflog |
| `A/nested-status.json` | Actual HEAD and status of each configured nested component |
| `A/unreachable*.json`, `unreachable.txt` | Read-only object archaeology and classification; no unreachable commits found |
| `A/deleted-device-artifacts.json` | All 52 deleted device artifact paths and per-path commit history |
| `A/recovered-device-artifacts/` | Separate exports of deleted JSON/Markdown/text evidence, from `b70894e^`; original tree not restored |
| `A/source.sha256`, `vsl1a/`, `canonical/` | Source hash taken before analysis, full fresh analysis, fresh canonical dataset view |
| `A/reproduction-comparison.json` | Structured comparison against the historical `20260921b` artifacts |
| `A/contract-probes.json` | Counterexamples accepted by current validators; these are audit findings, not test-suite failures |
| `A/pytest-root.log`, `pytest-focused.log`, `ctest.log` | Current safe test results; commands below |
| `A/summary.json` | Narrow audit-evidence verification, not flight/calibration/SLAM acceptance |

## A–B. Current file map and Git provenance

### Chronology reconstructed from Git

Root history and nested device history are independent. Root `git log --all`
alone cannot recover the nested source changes. Searches covered branches,
reachable commits, tags, stashes, reflogs, name-status/rename/deletion history,
untracked/local artifacts and documentation references. Keywords included VSL,
SLAM/slam, localization, LocalizationEstimate, PoseEstimate, localization_epoch,
calibration, 0921, analyze_video, manifest, FrameMetadata, LatestFrameSlot,
visual_motion and ExternalNav. Root and device tags/stashes were empty.

| Commit / date | Verified event | Interpretation |
|---|---|---|
| Device `0f3018e`, 2026-09-02 | `main/src/visual_motion.cpp` introduced; still its last modifying commit | Image-motion primitive predates the localization contract |
| Device `9eae1c8`, 2026-09-08 | OS04A10 capture tools and validation reports | Earlier camera evidence, not a VSL localization pass |
| Device `3467d1a`, 2026-09-08 | `main/include/dart/async_frame.hpp` added; still its last modifying commit | Latest-slot/metadata/scheduler history preserved |
| Device `4ed8af2`, `3408a47`, 2026-09-08 | NV21 pipeline, then VIN lease/business harness | Reusable capture boundary |
| Device `5840ee7`, `0dbb97c`, `395c0e7`, `0033c38`, `0a6ee3c`, 2026-09-08 | Motion cadence, clock-order correction, asynchronous logs, fusion preservation, stage cadence | Current pipeline includes these fixes; `0a6ee3c` is last change to `nv21_pipeline.cpp` |
| Root `8e9dfd9`, 2026-09-12 | Old `rm_uav_lab_m600/maixcam2_dart` gitlink pins `f4c153f`; visual guidance pins `994727e` | Recorded pre-migration component boundary |
| Device `b70894e`, 2026-09-18 | Recorder changes also delete 52 tracked files under device `artifacts/` | Separate component evidence deletion, recoverable from its parent; not caused by root namespace migration |
| Device `c7d47b2`, 2026-09-20 | Recorder durable checkpoint changes | Current root pin is newer than old `f4c153f` |
| Root `da396be`, 2026-09-21 | Adds current perception tree, localization code, all VSL reports/tests and starter package; removes old gitlinks | First **and last** reachable root commit for localization files |

Historical VSL-1 artifacts record producer HEAD `8e9dfd9`, but the localization
scripts are absent from that committed tree and first appear in `da396be`.
Therefore the earlier analysis used work not represented by its recorded HEAD.
The precise pre-commit edit chronology is **UNCOMMITTED_HISTORY_UNKNOWN**;
file modification times cannot repair that provenance. Current rerun hashes and
results provide a reproducible anchor without inventing historical commits.
`git log --follow` confirmed the addition boundaries, rather than a hidden
root rename chain for these files.

`git fsck --no-reflogs --unreachable` found 155 trees and 132 blobs, no commits.
Keyword hits in unreachable blobs resolved to CAD/rulebook assets, not lost
localization implementations. Relevant tree paths included simulation dynamics
calibration and Astra manifests. The separate forensic exports under
`A/recovered-objects/` are keyword false positives and are not SLAM evidence.
No destructive recovery or old-directory restoration was needed.

### Artifact map and disposition

For grouped current root files, first=last=`da396be`, no tracked predecessor or
rename/deletion exists, and no newer replacement was found. Exact filenames and
hashes are in `file-provenance.csv`; nested deleted-file first/last commits are
in `deleted-device-artifacts.json`.

| Original / historical item | Current location | Classification | Tests, artifacts, documentation and evidence quality |
|---|---|---|---|
| LocalizationEstimate v1 | `rm27/perception/vision/localization/schema.py`, `__init__.py` | CURRENT | Six tests in `tests/test_localization_contract.py`; `VSL-0B_CONTRACT.md`; code and current probes |
| Manifest validator | `rm27/perception/vision/localization/manifest.py` | CURRENT | Same six-test file; starter manifest; validation limits below |
| Encoded video analyzer | `rm27/perception/vision/localization/analyze_video.py` | CURRENT | `VSL-1A_0921.md`; two old artifact sets plus current complete reproduction |
| Canonical dataset utility | `rm27/perception/vision/localization/prepare_vsl2_dataset.py` | CURRENT | `VSL-1_GATE_REVIEW_0921.md`; old/new canonical manifests, CSVs and subsets |
| Calibration observation extractor and target template | `rm27/perception/vision/localization/{capture_calibration.py,target_definition.template.json}` | CURRENT | Code review only; no qualifying capture or automated tests found |
| VSL-0A/0B/1A/gate review, VSL-2 protocol/procedure/status, calibration template, command log | `rm27/perception/vision/docs/` | CURRENT | Report claims reviewed against code, tests and actual outputs; status documents are not independent measurements |
| M3C starter route, environment collector, manifest, SHA256SUMS | `rm27/perception/vision/M3C_OS04A10_SLAM_实验起步包_v1/M3C_SLAM_experiment_plan_v1/` | CURRENT | All three listed SHA-256 checks pass; protocol only; earlier `research/vision` placement is documented, not proven by a root rename commit |
| Old device checkout `rm_uav_lab_m600/maixcam2_dart` | `rm27/perception/vision/maixcam2_dart_vision/` | MOVED; old checkout SUPERSEDED | Root deletion/addition `da396be`, old/new pins `f4c153f` → `c7d47b2`; nested history preserved, not byte-identical snapshots |
| Camera producer boundary | `rm27/perception/camera/{README.md,__init__.py}` | CURRENT | Documentation/package marker only; no Python acquisition runtime |
| Old `visual_guidance/{uav_guidance,v1_sitl,v2_gazebo}` | `rm27/perception/{guidance,sitl,gazebo}/` | MOVED / ADAPT_TO_CURRENT_ARCHITECTURE | Source `994727e`, import adaptation in `da396be`; four current guidance tests; old root gitlink recoverable |
| Reportedly `rm27/perception/vision/0921.mp4` | `rm27/perception/vision/video/0921.mp4` | UNCOMMITTED_HISTORY_UNKNOWN | Source exists untracked, hash matches; no Git proof of original path or rename |
| First VSL-1A / canonical outputs | `artifacts/vsl-1a-0921-20260921/`, `artifacts/vsl-2-0921-canonical-20260921/` | ARTIFACT_ONLY; earlier duplicate derivation | Same source hash/count/sample policy; preserve as historical outputs, prefer documented `b` set |
| Documented VSL-1A / canonical outputs | `artifacts/vsl-1a-0921-20260921b/`, `artifacts/vsl-2-0921-canonical-20260921b/` | ARTIFACT_ONLY | Complete recorded evidence; `b` set reproduced below |
| Device business/rate/official/retest summaries, plots, images | Device `artifacts/{business_full180_20260908,business_full180_rates_20260908,os04a10_official_20260908,os04a10_retest_20260908}/` | DELETED_BUT_RECOVERABLE | 52 paths deleted in `b70894e`; JSON/text recovered separately into A; per-path history retained |
| Older raw device run CSVs, recordings, verification ZIP, matched SDK | Referenced device `.maixpy/runs/`, `.maixpy/official-build/`, artifact video paths | NOT_FOUND in inspected checkout | Hashes and summary references survive, but are not the underlying samples; do not equate a recoverable summary with a recovered raw run |
| Separate PoseEstimate implementation | None | NOT_FOUND | Name appears as an alternative in docs; actual class is LocalizationEstimate |
| Matching camera calibration / SLAM trajectories / ExternalNav adapter | None in this research line | NOT_FOUND | No qualifying provenance or execution evidence |

There is no demonstrated loss of unique **localization source**. Some earlier
camera evidence has disappeared from the current nested checkout, but tracked
summaries are recoverable. Referenced raw bundles were not found; their
completeness cannot be certified. This is a qualified recovery result, not a
claim that every historical byte survives somewhere.

## C. Current implementation and reuse matrix

Normal device runtime in `main/src/main.cpp` constructs
`maix::camera::Camera(...FMT_RGB888...)`, rejects configured rates above 60,
reads `camera.read(true, -1)` into `unique_ptr<Image>`, samples `ticks_us()`
after return, optionally runs `VisualMotionEstimator::update`, then calls
`GreenLightDetector::process` and the target JSON serializer. The owned image
stays alive through synchronous processing; the motion estimator copies a
small grayscale representation for its previous-frame state.

High-FPS runtime is the separate `tools/os04a10_highfps/official_capture.cpp`
business entry: `AX_VIN_GetYuvFrame` → `VinNv21Frame::adopt` →
`HighFpsPipeline::submit` → latest slot → mapped NV21 → source-coordinate
RGB ROI / Y thumbnail → detector and asynchronous image-motion worker →
TargetEstimate snapshots → output predictor / JSON. Full180 explicitly
invalidates calibrated angles/pose and `safe_for_control`.

The Python `guidance/schema.py` is a v2 **subset adapter**, not a live camera
transport implementation. `UAVGuidance` consumes target estimates;
`GuidanceCommand` is the only accepted MAVLink adapter input. SITL supplies
synthetic targets; Gazebo supplies truth-derived targets and a separate truth
stop gate. Passing those paths does not establish live camera integration.
The root CLI does not launch a localization estimator.

The localization branch currently ends at a data type and offline experiment
utilities: camera data → **future estimator, absent** → LocalizationEstimate
v1 → **future consumer/ExternalNav adapter, absent**. No consumer interpretation
of `T_parent_child` can be tested end-to-end yet. The contracts are separate;
TargetEstimate was not changed by this audit.

| Component | Localization relevance | Classification / decision |
|---|---|---|
| Normal RGB888 capture | Host-owned synchronous images; lacks source sequence/exposure evidence | Reuse producer boundary, document timing limits |
| VIN/NV21 lease | Preserves source metadata and stride without compulsory full-frame RGB copy | Reuse single-reader ownership model; qualify any additional consumer |
| FrameMetadata / SequenceStats | Sequence, raw PTS, receipt and anomaly counters | KEEP; no timestamp reinterpretation |
| LatestFrameSlot | Safe pending replacement under documented ownership; not an ordered stream | **REUSABLE_WITH_ADAPTER**, not direct SLAM/VIO handoff |
| TimestampSchedule | Absolute receipt-time stage cadence, skips late work | Target scheduling utility, not sensor synchronization |
| VisualMotionEstimator | Sparse features / two-dimensional image registration | **B: reusable visual frontend / diagnostic primitive** |
| MotionPrior / target tracker | Image-space compensation and optional orientation/rate fields | Preserve target use; not LocalizationEstimate or real IMU evidence |
| TargetEstimate, guidance, synthetic/truth adapters | Separate target-relative product | KEEP unchanged; useful for boundary regression only |
| LocalizationEstimate / manifest | Small offline foundation in correct current namespace | FIX narrowly; retain architecture |
| Video analyzer / canonical view | Reproducible uncalibrated encoded-image evidence | KEEP data, FIX acceptance/provenance details |
| Calibration collector | Preliminary target-image extractor | FIX before using its selection as qualification |

Documentation discrepancies: `vision/README.md` omits the surviving
`localization/` and VSL `docs/`; `guidance/README.md` suggests sibling SLAM
publishes the estimate consumed by guidance, which must be clarified to mean
target recognition only. `camera/README.md` overstates the present live runtime
connection; code is currently an adapter boundary. `guidance/schema.py` points
to `docs/frames.md`, absent from this migrated package. Root README's “31
passing” and migration table's “35” are stale counts, not current results.
The starter manifest's `timestamp_unit: ns` with unknown source is a template
default, not established device timestamp evidence.

## D. Timestamp evidence table

“Host” below means the application host on the camera board, not necessarily
the analysis PC. No cross-device synchronization is established.

| Source / field | Raw value available | Unit | Clock domain | Semantic and evidence |
|---|---|---|---|---|
| Normal `main.cpp` timestamp | Runtime `maix::time::ticks_us()`; no current board sample | API expresses microseconds | Maix runtime clock; implementation/epoch not verified here | Sampled after `camera.read`; application receive, not exposure |
| VIN `FrameMetadata.sequence` | `stVFrame.u64SeqNum` | Sequence count, not time | SDK sequence space; reset/wrap unspecified | Copied in `VinNv21Frame::adopt`; skips detectable, sensor provenance not proved |
| VIN `pts_raw` | Exact `stVFrame.u64PTS`; current localization sample absent | **UNKNOWN** | **UNKNOWN** | **UNKNOWN** event; never exposure by inference. Older summary observes ~1e6 ticks/s, insufficient to establish semantics |
| VIN `received_us` / `frames.csv.monotonic_us` | `stamp=now_us()` after successful GetYuvFrame | Microseconds | C++ `steady_clock` on board | Software receipt; `official_capture.cpp::now_us`, acquisition loop |
| `vision.csv.started_us/finished_us`, `motion.csv` processing times | Runtime `monotonic_us()` | Microseconds | C++ `steady_clock` on board | Software processing boundaries; not capture or transport latency |
| MotionPrior `timestamp_us` | Input image receipt time propagated unchanged | Microseconds in current callers | Caller receipt clock | Endpoint label for previous→current image displacement; no stored start time or dt |
| Full180 TargetEstimate `timestamp_us` | `now` sampled after snapshot acquisition | Microseconds | C++ `steady_clock` on board | Output generation; separate `source_received_us` and raw source PTS retained |
| Full180 `measurement_age_us` | Computed tracker age | Microseconds | Same software clock assumed by caller | Elapsed software measurement age; exposure age remains unknown; target contract only |
| MP4 `CAP_PROP_POS_MSEC` | First 0; last 76472.22222222223 ms | API milliseconds, saved as seconds | Encoded timeline; hardware relation UNKNOWN | OpenCV/FFmpeg presentation-time observation; `timing.csv` has every value |
| Canonical `encoded_pts_s` | Same sampled PTS | Seconds | Encoded timeline | Not VIN, receive, or exposure time |
| Localization `source_time`, `publish_time` | No estimator records; only test values 10/20 etc. | Explicit evidence field; can be UNKNOWN | Explicit evidence field; can be UNKNOWN | Correct separation in schema; a caller's assertion is not independent verification |
| File mtime, analysis creation time | Filesystem / UTC metadata | Seconds / ISO date | Filesystem / host wall clock | File/session provenance only, not frame timing or Git chronology |

A structurally valid localization record can become stale later. There is no
serialized permanent measurement age, which is correct. Source-to-publish and
current age cannot be computed across UNKNOWN or unrelated clocks without a
verified mapping. A software receive age is only a lower bound on exposure age.
**SENSOR_CAPTURE_DROP_RATE = UNKNOWN for 0921.mp4.**

## E. Buffer ownership and LatestFrameSlot review

`VinNv21Frame` owns the obligation to release one borrowed SDK image after
`adopt`; it does not own the DMA allocation. `map()` checks NV21 VU format,
even dimensions, bounded Y/UV strides, and frame size; maps cached Y/VU planes
and invalidates cache before CPU reading. Its destructor unmaps and calls
`AX_VIN_ReleaseYuvFrame`; counters track map/release errors separately. The
`released` counter counts release attempts, so balance alone is insufficient:
`official_capture.cpp` also checks error counters and joins readers before VI
teardown.

`Nv21View` is a borrowed pointer view. Retaining the `shared_ptr<Nv21Frame>`
through every pixel access is mandatory. Current vision-loop locals satisfy
that lifetime. ROI RGB images and motion thumbnails own copied pixels;
`motion_loop` never reads the DMA view after the lease is released.

`LatestFrameSlot` contains one pending `shared_ptr`, protected by a mutex.
Publishing replaces the pending frame and destroys the old reference outside
the mutex. A taken frame belongs to the consumer and is unaffected by later
replacement. Counts distinguish published, taken, replaced, shutdown-discarded
and rejected frames. Source metadata travels inside the frame object, so it
stays associated with its pixels. Shutdown drops pending work and wakes waiters;
its owner must join them before destroying the slot.

Skipped frames are not wholly “silent”: consumer sequence values expose gaps,
producer `SequenceStats` records upstream gaps, and slot replacement / scheduled
skip counts record application losses. However there is no ordered gap event,
contiguous-stream guarantee, or per-consumer drop ledger. Multiple `take()`
consumers compete for frames; this is not broadcast. Holding many outstanding
leases can starve a finite VIN pool even while lifetime remains valid.
`VinNv21Frame::map()` itself is not synchronized, so parallel lazy mapping on the
same object is not safe. After a partial mapping/cache failure it must be
abandoned; the existing path throws and stops, rather than retrying it.

**Decision: REUSABLE_WITH_ADAPTER.** Keep the target implementation. A future
localization consumer needs an explicit sampling/gap/reset policy and bounded
lease ownership; if ordered input is required, supply an appropriate separate
handoff. Do not reuse the detector's motion-thumbnail queue as a localization
sequence: it retains only a receipt timestamp and drops source sequence/raw PTS.
No adapter was implemented here.

Current host tests exercise replacement, retained slow-consumer ownership,
close/reject behavior, concurrent replacement, sequence counters and schedule
regressions. These prove host mechanics, not DMA/SDK behavior. Optional VIN ABI
mock tests were not configured because a matched SDK include tree was not
available in this checkout.

## F. Independent visual_motion review

`main/src/visual_motion.cpp` down-samples RGB888 using weighted grayscale.
Features use a 3×3 gradient structure tensor/minimum eigenvalue, threshold 600,
a two-pixel search grid, minimum separation four and cap 80. Tracking uses a
bounded integer patch search over 5×5 patches, photometric/ambiguity rejection,
then up to four subpixel Lucas–Kanade refinement iterations. This is a custom
small frontend, not ORB tracking or a general multi-scale SLAM frontend.

The robust fit enumerates feature-pair hypotheses for a 2-D similarity
`[a -b; b a] p + [tx ty]`, gates residuals at 1.5 downsampled pixels, requires
at least four inliers and refits on the best set. Despite the RANSAC function
name, hypothesis enumeration is deterministic rather than random sampling.
The output is center displacement scaled to input-image pixels, image-plane
rotation in radians, scale and a heuristic confidence. It is neither camera
translation in metres nor a 3-D orientation estimate.

The timestamp is only copied to MotionPrior; the estimator stores no previous
timestamp, checks no regression, and divides by no dt. High-FPS `motion_loop`
resets after a receipt gap >100 ms and accepts recent results (≤50 ms) in the
vision loop, but a recent delta still lacks its start-frame association.
Asynchronous target compensation therefore must not be promoted to a pose
increment without adaptation. `TemporalTracker::apply_motion_prior` transforms
the target prediction about its configured principal point; no ego-pose is
integrated. The image-center/principal-point and thumbnail rounding conventions
also need review if extracted for another purpose.

MotionPrior's quaternion/angular-velocity fields and interpolation helper are
future-prior plumbing. The visual estimator leaves those fields at defaults;
there is no IMU acquisition, synchronization or calibrated fusion here. The
host test recovers an artificial (8,4)-pixel shift and checks interpolation;
it does not validate flight trajectory, drift or metric scale.

Classification **B**, reusable diagnostic/frontend primitive, with temporal and
coordinate adapters before broader reuse. Its bounded search and 2-D model are
actual model limitations; no evidence here demonstrates a failed SLAM approach.

## G. Independent VSL-0B contract review

Keep the dataclass-scale design. Parent/child direction is explicit:
`T_parent_child` maps child coordinates into parent coordinates. Translation,
`wxyz` quaternion, independent timestamp evidence, state, epoch and optional
velocity/map/reset/quality are sensible foundations. Missing covariance is not
fabricated: there is no dedicated covariance field; arbitrary `quality` can
carry metadata. Docs should not imply a validated covariance interface exists.

The record does **not** specify translation/velocity units, metric versus
arbitrary monocular scale, frame axes/handedness, or a unit-quaternion policy.
Those are small but necessary contract decisions before estimator integration.
No consumer exists to supply missing semantics implicitly.

Reproduced validator gaps (`A/contract-probes.json`):

- `(2,0,0,0)` quaternion accepted; only nonzero norm is enforced. A huge finite
  quaternion can also overflow the norm computation. Specify and validate
  rotation semantics rather than silently assuming normalized inputs.
- Boolean schema version `true` equals integer 1; boolean translation becomes
  1.0. Tighten numeric types and finite JSON serialization.
- `quality={"covariance": NaN}` serializes as nonstandard JSON `NaN`;
  quality is only checked as Mapping and is mutable despite a frozen dataclass.
  `map_id` and `reset_reason` also lack type validation. Preserve absent quality,
  require finite serializable supplied values, and document estimator provenance.
- TimestampEvidence accepts `EXPOSURE` with `UNVERIFIED`, whereas the manifest
  validator rejects that combination. Keep raw evidence but apply a consistent
  claim-validation policy; do not confuse free-form strings with verification.
- `LOST` with `valid=True` is accepted. This can describe an existing last pose,
  but state/valid/initialized/relocalizing relationships are undefined. Clarify
  whether this is retained pose or valid current measurement; consumers must
  gate tracking/freshness independently. Do not silently infer flight eligibility.
- Epoch is a non-negative integer, but continuity changes are only documented.
  Tests construct a RESET record; none enforce increments across a stream or
  prevent a new process from restarting epoch 0. Define stream/session ownership
  and reset handling before a consumer can mistake a jump for motion.
- Missing calibration file passes validation when no hash is supplied. Hash
  checking runs only when file, hash **and** base directory are present. A bare
  `{"schema_version":1}` passes. This is partial metadata validation, not a
  calibration/dataset qualification gate.
- Stride is checked only against width, not bytes-per-pixel/plane layout;
  width-byte stride for RGB888 passes. Monotonic timestamp raw values are compared
  without requiring consistent unit/clock; equal times pass. Unknown timing
  should remain allowed, but comparisons must respect known clock semantics.

The canonical dataset and calibration capture manifests use different structures
from `manifest.py` (`frames.csv` / `records` rather than embedded `frames`).
They can pass that permissive validator without their actual samples being
validated. Name/document its scope or add narrowly scoped validators later;
do not build an unnecessary universal schema framework.

The six existing localization tests pass and are useful. They do not cover
these cases, real reset continuity, frame composition, staleness or calibration
qualification. This is a **PARTIAL** VSL-0B foundation, not an inherited PASS.

## H. VSL-1A reproduction and dataset integrity

Source, hashed before processing: `rm27/perception/vision/video/0921.mp4`,
230,317,363 bytes, SHA-256
`437d72cb0030cdb33d1ea8377b2d61e234280813dbb83c85ab3c8e3e44fe50bb`.
M3C + OS04A10 flight origin is user-provided provenance; the MP4 alone cannot
identify the physical module, lens, focus or capture firmware.

| Historical claim | New result | Classification |
|---|---|---|
| Source SHA-256, size | Exact match | REPRODUCED |
| H.264, 1344×760, nominal ~180 FPS | OpenCV FourCC `h264`, 1344×760, 180.0 | REPRODUCED within decoder scope |
| 13,766 / 13,766 frames | Declared=decoded=13,766 | REPRODUCED |
| ~76.47 seconds | First PTS 0; last PTS 76.47222222222223 s | REPRODUCED as timeline span, not independently probed container duration |
| Average encoded cadence | (13766−1)/(last−first)=180 FPS | REPRODUCED; MP4 metadata FPS is not sensor FPS |
| Monotonic PTS, no encoded timing gaps | 13,765 intervals; mean 5.555556 ms; zero nonpositive and >1.5× nominal gaps | REPRODUCED |
| No exact duplicates/frozen runs | Zero adjacent decoded-frame hash repeats, zero runs with ≥2 repeated transitions | REPRODUCED under stated diagnostic |
| No near duplicates | Zero sampled comparisons with mean grayscale difference <1 | REPRODUCED for every-sixth-frame sample, not all frame pairs |
| No decode failures | Full declared count decoded, premature-read-failure counter zero | REPRODUCED; does not exclude codec error concealment |
| Blur, luminance, spatial features, flow proxies | Entire `quality/frame_quality.csv` byte-identical | REPRODUCED |
| Diagnostic intervals | JSON and YAML byte-identical | REPRODUCED |
| 2,295 JPEG diagnostics | All 2,295 images byte-identical, JPEG quality 95 | REPRODUCED |
| Five representative example images automatically reproduced | Old directory has 5; current analyzer generates none | NOT_REPRODUCED by checked-in analyzer; historical examples preserved |
| Canonical frames/intervals/subsets | CSV, intervals and subsets byte-identical; manifest intentionally has new artifact path/producer commit | REPRODUCED_WITH_MINOR_DIFFERENCE |
| Sensor capture drop rate | UNKNOWN | No hardware sequence evidence in MP4; not measurable here |

No ffprobe/ffmpeg executable, PyAV or imageio_ffmpeg was available. This rerun
uses the same OpenCV version; it independently executes the committed script,
but is not an independent decoder implementation or packet-level timing audit.
Container stream count, time base, DTS, codec profile and native pixel format
remain unverified. Duration is currently calculated as last PTS, which only
matches elapsed span because this file starts at zero; even here it omits final
frame presentation duration. The bitrate is a file-size/span approximation.

Analyzer review found `analysis_complete.json.passed` is unconditionally true
once execution reaches the end, even for premature decode or timing problems.
It must not be used as an acceptance gate by itself. This audit explicitly read
counts/timing/integrity and compared outputs. The analyzer also hardcodes the
backend label, records the source hash only at the end, records an incomplete
command and no script hash, ignores `imwrite` return values, and does not create
the representative examples described by the report. Fix these details later;
the actual current 0921 reproduction remains valid.

`prepare_vsl2_dataset.py` creates a **view**, not a self-contained frame archive:
`frames.csv.image` resolves relative to `source.artifact_directory`, not the
canonical directory. It trusts upstream metadata and does not rehash the source
or images. Retain that design if documented, but verify the dependency and
content hashes before moving or sharing the dataset. No need to rerun all video
analysis merely because the root namespace changed; it works at its current path.

## I. Calibration blocker and missing experiments

No calibration satisfying all of M3C + OS04A10 + actual physical lens + current
focus + current crop/resize/mode + 1344×760 geometry was found. Evidence:
`configs/real_hardware.json` has `camera_intrinsics: null`; the calibration
artifact template is UNKNOWN; canonical manifest is `CALIBRATION_REQUIRED`;
`green_detector_full180.conf` explicitly calls its fx/fy=1344, cx=672, cy=380
and zero distortion provisional pixel normalization, not measured intrinsics.
The device's `tools/dataset/select_calibration_images.py` concerns INT8 network
quantization, not geometric camera calibration. Root simulation calibration
and archived autopilot sensor-calibration material are also unrelated.

Searches covered current vision/config/reference/artifact paths, root reachable
and unreachable objects, nested source/history and the external legacy archive
filename inventory. This cannot prove no calibration exists on another machine;
none is available with qualifying provenance in the inspected workspace.
The September 8 device summaries report a full180 mode and sensor registers,
but do not identify the lens/focus/mode of the later 0921 recording. Matching
resolution and sensor name cannot transfer that provenance.

The capture utility is preliminary, not “ready-to-use” qualification:

- A unique 4×6 **centroid** cell accepts only one view; therefore at most 24
  views can be accepted. It cannot meet the documented 30–50-view protocol and
  discards useful tilt/distance variation at the same centroid.
- Every fifth accepted frame is holdout: at the cap, 4/24 = 16.7%, below the
  documented at-least-20%. There is no independent capture validation session.
- Target validation rejects placeholders but does not require finite positive
  dimensions or minimum detection coverage; partial ChArUco detections can pass.
  ChArUco corner IDs are discarded from outputs (images permit later redetection).
- Procedure mentions AprilGrid; implementation accepts chessboard or ChArUco.
  These are not interchangeable target types. Choose a supported measured target.
- Lens/focus/mode provenance remains hardcoded UNKNOWN; no calibration fitting,
  artifact validation, minimum-view gate or independent residual test is present.

**VSL-2 remains BLOCKED_ON_CALIBRATION_CAPTURE**, with partial encoded dataset
preparation. This is missing evidence plus preparation-tool defects, not a
migration failure or an algorithm performance result.

There is useful earlier direct-capture evidence: recovered device
`measurements.json` reports the 324,244-frame full180 run, and business summary
`205938-5a29e4.json` retains receive/sequence and lease statistics. They support
that a prior target-camera test campaign occurred. They are summaries of an
older campaign, not independently reprocessed raw logs, nor current lens/mode
qualification. Thus “no direct camera tests ever ran” would be false, while
**current localization VSL-1B has not been executed/qualified**. Preserve this
distinction when reading the old NOT_STARTED label.

## J. Migration damage versus technical defects

| Issue | Root cause class | Evidence / consequence |
|---|---|---|
| Old device/visual-guidance root paths absent | REPOSITORY_MIGRATION | Deliberate ownership change in `da396be`; current imports and tests work; do not restore old layout |
| Device artifact links point to deleted files | REPOSITORY_MIGRATION (component evidence cleanup) | `b70894e` deletes 52 paths before root migration; recoverable summaries, broken report links |
| Unrecorded pre-`da396be` localization edit history; missing raw device bundles | MISSING_EVIDENCE | HEAD-only historical provenance cannot recreate uncommitted producer source; raw hashes do not substitute for files |
| Stale frame-doc link, omitted localization README entries, 31/35 test counts | REPOSITORY_MIGRATION / STALE_DESIGN documentation | Runtime architecture outpaced its directory summaries |
| Non-unit rotation, permissive JSON/types/calibration-file checks | HISTORICAL_IMPLEMENTATION_BUG | Current executable probes; not an import/move problem |
| Undefined units/scale, flag relationships, epoch session owner | STALE_DESIGN | Foundation needs small explicit contract decisions before estimator integration |
| Analyzer unconditional pass, incomplete example generation/provenance | HISTORICAL_IMPLEMENTATION_BUG | Code and reproduction; current valid data does not excuse weak gating |
| Calibration selection cap/holdout/target mismatch | HISTORICAL_IMPLEMENTATION_BUG | 24-cell bound conflicts with 30–50 views and ≥20% holdout |
| Missing lens/focus/mode/calibration and exposure semantics | MISSING_EVIDENCE | Never established by the available video/tests |
| 2-D similarity / bounded image tracking lacks metric depth and trajectory | ACTUAL_ALGORITHM_LIMITATION | Visible model limitation of visual_motion; no SLAM experiment has failed here |
| Claimed 50 localization tests | MISSING_EVIDENCE / scope misstatement | Current root suite is 50 total, only six localization tests |
| Current test failures caused by migration | No BROKEN_TEST identified | All executed suites pass; missing coverage is not a broken test |

## K. Keep / fix / redo decisions

| Component | Historical purpose | Current location / evidence | Current correctness | Action | Reason |
|---|---|---|---|---|---|
| Source MP4 | Initial real encoded sequence | `vision/video/0921.mp4`, matching hash | Intact | KEEP | Raw evidence; no re-encoding or replacement |
| Historical VSL-1A frames/tables | Qualification and diagnostic subsets | Two ignored artifact sets, `b` reproduced | Valid within encoded scope | KEEP | No redo needed for migration |
| Starter protocol/collector | Platform/environment planning | Starter directory, 3/3 hashes | Useful protocol, not executed qualification | KEEP_WITH_DOC_FIX | Clarify ns placeholder and actual scope |
| FrameMetadata / sequence counters | Capture lineage | Device `async_frame.hpp`, host tests | Useful with unknown PTS semantics | KEEP | Do not relabel timestamps |
| VIN lease and target latest-slot code | Low-latency target processing | Device VIN header / async header | Host lifetime sound in existing single-reader use | KEEP | Preserve target path; hardware qualification remains separate |
| Localization handoff using latest slot | Candidate reuse | Same slot | No ordered/gap-aware estimator contract | ADAPT_TO_CURRENT_ARCHITECTURE | Separate bounded localization consumer, only when authorized |
| visual_motion | Image-motion compensation | Device `visual_motion.cpp`, synthetic shift test | Correct scope is image diagnostic | KEEP_WITH_DOC_FIX | No VO/SLAM/metric claim; adapt temporal interface only if later needed |
| TargetEstimate / guidance / simulator adapters | Target-relative control boundary | `guidance/`, `sitl/`, `gazebo/`; 4 focused tests | Separate product preserved | KEEP | No localization fields or behavioral changes |
| Localization schema | Ego-pose record | `localization/schema.py`, six tests + probes | Partial | FIX | Units/scale/rotation, JSON/type checks, state/epoch semantics |
| Manifest validation | Metadata guard | `localization/manifest.py` | Partial, not qualification | FIX | Conditional file check, clock/format-aware validation and truthful scope |
| Encoded analyzer | Deterministic qualification | `analyze_video.py`, exact reproduction | Metrics useful; acceptance unsafe | FIX | Explicit gates, complete provenance, duration/example handling |
| Canonical dataset view | Stable sampled input | `prepare_vsl2_dataset.py`, matching CSV/subsets | Useful dependent view | KEEP_WITH_DOC_FIX | State image path base; validate/hash dependencies before portability claims |
| Calibration extractor | Prepare measured board observations | `capture_calibration.py` | Cannot meet its stated capture protocol | REIMPLEMENT_SMALL_PART | Replace centroid-only selection/holdout logic; validate target/provenance |
| Earlier device reports/evidence | Board acquisition baseline | Current tools docs + deleted paths in nested Git | Summary evidence survives, raw reproduction incomplete | ARCHIVE | Preserve source commit and recovery index; no wholesale restoration |
| Camera qualification for current localization configuration | VSL-1B / timing evidence | No matched raw run | Incomplete | REDO_EXPERIMENT | Older campaign cannot qualify present lens/mode or current stream |
| Geometric calibration | VSL-2 | No valid artifact | Blocked | REDO_EXPERIMENT | A real capture is required; do not guess intrinsics; this may be first qualified capture |
| README/frame/provenance references | Navigation and claims | Current docs | Several stale/overbroad statements | KEEP_WITH_DOC_FIX | Preserve current architecture and narrow claims |

No component warrants blanket removal or a rewrite of the localization research
line. Proposed redo items are requirements for later work, not permission to
execute multiple experiments now.

## L. Independently assigned current VSL status

| Stage | Status | Evidence / commit / checks | Remaining uncertainty |
|---|---|---|---|
| VSL-0A | PASS for repository/code audit only | This inventory; root `da396be`, device `c7d47b2`, nested history and ownership inspection | Actual M3C board, toolchain/resources and lens not independently qualified; original platform gate is not wholly passed |
| VSL-0B | PARTIAL | Surviving schema/manifest at `da396be`; six contract tests pass; accepted malformed/underspecified cases recorded | Units/scale/rotation, epoch streams, finite quality and validation scope require fixes |
| VSL-1A | PASS for encoded qualification; REPRODUCED | Same SHA; 13,766 decoded; exact timing/quality/intervals and 2,295 JPEGs; A reproduction at `da396be` | Same decoder; no packet audit, exposure timing or sensor-drop proof; representative export not reproduced |
| VSL-1B | NOT_STARTED for current localization qualification | Current protocol only; older device summaries recoverable from `b70894e^` | Not claiming direct camera work never happened; matched raw capture and overhead/sequence evidence absent |
| VSL-2 | BLOCKED_ON_CALIBRATION_CAPTURE; dataset preparation PARTIAL | Canonical dataset reproduced; null/UNKNOWN calibration; collector defects at `da396be` | Actual lens/focus/mode, measured target, capture and validation residuals missing |
| VSL-3 | NOT_STARTED | No estimator implementation, trajectories or baseline-run artifacts found in this line | Installation/tracking/drift/scale/runtime remain untested |

## Verification commands and actual results

Working directory for commands: `/home/shiuhou/Projects/rm27_drones`.
Interpreter: `/home/shiuhou/venvs/mujoco/bin/python`. Root commit `da396be`,
device commit `c7d47b2`; full versions are recorded above. Fresh outputs go to A.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests/test_localization_contract.py tests/test_visual_guidance.py -q
cmake -S rm27/perception/vision/maixcam2_dart_vision/tests -B artifacts/slam-recovery-audit-20260922/host-build
cmake --build artifacts/slam-recovery-audit-20260922/host-build -j 2
ctest --test-dir artifacts/slam-recovery-audit-20260922/host-build --output-on-failure
sha256sum rm27/perception/vision/video/0921.mp4
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.analyze_video rm27/perception/vision/video/0921.mp4 --out artifacts/slam-recovery-audit-20260922/vsl1a
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.prepare_vsl2_dataset --vsl1 artifacts/slam-recovery-audit-20260922/vsl1a --output artifacts/slam-recovery-audit-20260922/canonical
```

Commands above were executed with stdout/stderr retained in A. Default analyzer
sample step was 6, gap factor 1.5. To repeat, use a new output directory.
Starter checksum command: `sha256sum -c SHA256SUMS.txt` from the starter package
directory. All three checks passed.

| Suite | Current result | Scope |
|---|---|---|
| Root `pytest tests -q` | 50 passed, 0 failed, 0 skipped, 2.03 s | Entire current root suite; not “50 localization tests” |
| Focused two files | 10 passed, 0 failed, 0 skipped, 0.02 s | Six localization + four visual-guidance tests; subset of the 50, not additional coverage |
| CMake/CTest device host build | 3 test executables passed, 0 failed, 0 skipped | `green_detector_tests`, `async_frame_tests`, `async_log_tests`; stubs, no board libraries |
| Optional VIN ABI mock suite | Not configured/run | Missing matched SDK; not counted as a passing or skipped CTest |
| Offline video / canonical reproduction | Exit 0; counters and byte comparisons verified | No localization accuracy or real-time performance measured |

Historical “approximately 50 tests passed” cannot be independently tied to the
old uncommitted implementation using its HEAD alone. Current results above are
fresh and separately scoped. The anomaly probes are direct construction and
serialization checks against current code; no repository tests were added or
altered to make the audit pass.

## Single next experiment — proposed, not executed

**One bench geometric-calibration capture of the actual fixed M3C + OS04A10
camera/lens/focus at verified 1344×760 localization geometry.**

Before that capture, the separately authorized preparation increment must
resolve the collector's view/holdout defect or use a reviewed manual selection,
freeze a supported physically measured checkerboard or ChArUco definition,
record board/build/mode/crop/lens/focus/exposure provenance, and predeclare
numerical validation criteria. Preserve the raw recording and hashes; collect
30–50 varied sharp views across center/edges/corners, distance and tilt; reserve
at least 20% before fitting. Keep held-out observations identifiable. Current
protocol is a starting point, not evidence that those prerequisites are met.

If the current mode/lens/focus cannot be matched to 0921, label the calibration
as a new configuration and do not apply it retroactively to 0921. Success means
a traceable calibration dataset and, after fitting/validation, a matching
calibration artifact; failure means an explicit remaining blocker. No flight,
SLAM, VIO, ExternalNav or parallel experimental campaign is part of this step.

## Final decisions

**1. Is the previous localization foundation trustworthy enough to continue?**

Yes for continued offline research, after the narrow validation and calibration
preparation fixes above. It is not an accepted control-facing localization stack.
VSL-0B is PARTIAL; exact VSL-1A reproduction is a strong surviving evidence base.

**2. Which previous components should be kept unchanged?**

The raw MP4, historical frames/tables, target contract and guidance behavior,
FrameMetadata/raw PTS representation, existing target latest-slot/lease usage,
and visual_motion's bounded diagnostic role. Preserve the starter package and
historical evidence; clarify documentation without rewriting their history.

**3. Which components require small fixes?**

Localization type/rotation/unit/state/epoch semantics, manifest validation scope,
analyzer acceptance/provenance/example handling, calibration selection/holdout
and target checks, and stale documentation. Any localization queue adapter is
future work, not a reason to modify the working target path now.

**4. Which experiments must actually be repeated?**

Current-configuration direct capture qualification and geometric calibration
must be performed with matched provenance; old camera summaries cannot replace
them. No migration-driven repeat of the 0921 encoded analysis is necessary—it
was reproduced here. No previous SLAM run was found to repeat.

**5. Did repository reorganization lose any unique information?**

No unique localization implementation was found lost. Root migration preserved
current code; nested cleanup removed 52 evidence files that remain Git-recoverable.
Some referenced raw device bundles are unavailable, and pre-commit localization
edit chronology is unrecorded. Therefore complete preservation of all historical
experimental information cannot be certified.

**6. Is VSL-2 still correctly blocked on calibration?**

Yes. There is no validated artifact tied to the actual module, lens, focus, mode
and 1344×760 geometry. Provisional full180 values and a reproduced video do not
satisfy that requirement.

**7. What is the single next experiment?**

The one matched bench calibration capture specified above, after separately
authorized preparation fixes. Stop at this audit; none of those fixes or
experiments has been started.
