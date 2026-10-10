# VSL-0B.1 and VSL-2-PREP implementation update

Work performed 2026-09-22–23 in linked worktree
`/home/shiuhou/Projects/rm27_drones_slam`, branch
`research/slam-vsl0b1-calibration-prep`, base commit
`da396bef63ce51476a71af4b9b70abdda03eed0b`. The branch/worktree already existed
when this implementation resumed and was clean. The primary workspace and its
unrelated nested changes were not edited. The untracked
[recovery audit](SLAM_RECOVERY_AND_AUDIT.md) was copied unchanged as review input;
it remains a historical audit, not the current implementation report.

## New contract semantics

`LocalizationEstimate` is now **schema version 2**. Version 1 is rejected rather
than silently upgrading underspecified units/state. Python construction defaults
to unknown scale/units; serialized v2 records must explicitly contain both fields.
No estimator, flight eligibility policy or covariance model is introduced.

| Scale state | Translation unit | Velocity unit, only if present |
|---|---|---|
| `METRIC` | `m` | `m/s` |
| `ARBITRARY` | `arbitrary` | `arbitrary/s` |
| `UNKNOWN` | `UNKNOWN` | `UNKNOWN` |

Velocity names its expression frame. Scale and units must agree; monocular
localization is never assumed metric. Absent velocity/quality stays absent.
Supplied quality must serialize with `allow_nan=False`; serialization rechecks
it, including if an external caller mutates a referenced nested mapping later.
Quality metadata is not validated covariance. Booleans and numeric strings are
not accepted as numbers. Integer fields require actual integers.

`T_parent_child` maps **coordinates expressed in child into parent**:
`p_parent = R(q_wxyz) p_child + translation`. Rotation uses a right-handed
orthonormal basis and the Hamilton unit-quaternion convention, stored scalar
first. Producers must document physical axis directions, origin, and frame
identity in their experiment metadata; the schema cannot infer them from names.
There is **no chosen final world frame**, gravity alignment or ENU/NED/optical
axis assumption. No per-record CW/WC direction switch exists. A frame-convention
change breaks continuity and requires a new epoch.

Quaternion components must be four finite real int/float values, excluding
bool. `math.hypot` computes a stable norm; absolute `abs(norm-1) <= 1e-6` is
required, without normalization. Scaled, zero, huge/nonfinite values are rejected.

| State | initialized | valid | relocalizing |
|---|---|---|---|
| UNINITIALIZED / INITIALIZING / RESET | false | false | false |
| TRACKING | true | true, current producer pose | false |
| LOST | true | true means retained last pose; false means unavailable | false |
| RELOCALIZING | true | true means retained last pose; false means unavailable | true |

Pose fields in invalid records are finite placeholders with **no measurement
meaning**. A LOST/RELOCALIZING retained pose keeps its original measurement
source time; publish time is the new publication. TRACKING does not establish
freshness, accuracy or flight safety. Consumers must check those separately.

The producer owns `localization_epoch`. Increment it on RESET, new origin,
map replacement, discontinuous scale correction, or any loss of coordinate
continuity. Each RESET record represents one event, not a repeatedly published
state. On estimator restart, either persist/increment the epoch in the same
session or establish an explicit new session ID outside the record. Never
concatenate sessions under the same ID with epoch zero.
`LocalizationStreamValidator` validates one externally identified session:
it rejects decreasing epochs, RESET without increment and visible frame/map/
scale changes within an epoch. Its optional caller-selected translation-step
limit detects some unannounced jumps; it cannot distinguish all jumps from
real motion or detect every changed origin/rotation. It is not a kinematic or
flight gate. A new origin under the same frame name remains producer-owned.

## Timestamp and manifest policy

Both TimestampEvidence and frame manifests retain `EXPOSURE + UNVERIFIED` as
an **unverified claim**. The raw value and claim are not discarded or promoted
to facts. Only independently established VERIFIED interpretation may support
clock/age reasoning. The schema does not itself verify the producer's evidence.
Raw device PTS continues to allow UNKNOWN unit, domain and semantic.

The experiment manifest retains schema 1 and has three narrow entry points:

- `validate_experiment_metadata`: basic metadata, strict JSON and camera geometry.
- `validate_frame_records`: embedded frame metadata/sequence/stride/timestamp checks.
- `validate_calibration_reference`: claimed ID/file/SHA-256 consistency and bytes.

`validate_manifest` composes them for its embedded-frames format; **it does not
validate canonical `frames.csv`, capture `records`, fit residuals, dataset
completeness or camera qualification**. Their producer utilities do their own
narrow checks. An empty schema-1 experiment remains representable as metadata.

Claiming any known calibration ID/file/hash requires the complete bundle and
an existing matching file. Relative files require an explicit manifest base
directory, even without a previous hash-only trigger. UNKNOWN/null calibration
with `CALIBRATION_REQUIRED` is representable. A matching hash establishes file
identity, not a valid camera model or matching physical configuration.

RGB888 stride is at least 3 × width bytes; GRAY8 at least width. The supported
NV21 metadata describes Y followed by VU, even geometry/strides, equal plane
row pitches, and sufficient combined bytes when supplied. It does not certify
SDK buffer lifetime or support arbitrary vendor layouts. Unknown formats with
a supplied stride are rejected rather than guessed.

Timestamp ordering is checked only for explicit `monotonic=true`, VERIFIED
interpretation, known compatible ns/us/ms/s units, identical domain and event
semantic. Values are compared with integer unit conversion, nondecreasing
order (equal timestamps may reflect clock quantization). Separate clocks,
unknown timing or unverified claims are not falsely compared. Sequence lists
are explicitly one increasing sequence space; split lists on sequence reset.

## Encoded video and canonical view

Analyzer v2 hashes source bytes **before decoding** and again afterward; saves
analyzer file SHA-256/version, Git base, interpreter/OpenCV/NumPy versions,
actual backend if available, working directory, full command and parameters.
Because this work is not committed, file hashes identify the executed edits;
Git HEAD alone is not their identity.

`analysis_complete.json` separates execution from acceptance. Aggregate PASS
requires positive declared count equal to decoded count, unchanged source,
no image-write failures, >=2 finite strictly increasing PTS values, known
nominal cadence, and no intervals above the configured gap factor. Unknown
frame count or cadence cannot establish PASS. Premature decoding is explicit.
Duplicate/blur/feature/flow diagnostics keep their prior scientific meaning;
they are observations, not automatic localization-success criteria. OpenCV
cannot rule out decoder concealment or certify sensor drops/exposure timing.

Duration now states first PTS, last PTS and their span. The compatibility field
`duration_from_pts_s` means span, excluding final presentation duration.
Count/fps estimates are separately named; container duration remains unavailable.
**Representative example images are not a current analyzer product.** The five
historical examples remain historical; select images via `frame_quality.csv`.
Historical VSL-1 reports are not rewritten to imply automated reproduction.

Canonical output remains a **dependent view**: `frames.csv.image` resolves
relative to `source.artifact_directory`. The utility rehashes the source MP4,
requires every referenced image, records image hashes and upstream metadata
hashes/identity, and writes no copied frame archive. These are current byte
identities, not proof of the original acquisition lineage. For relocation,
retain the upstream artifact dependency, update its path explicitly, and verify
the saved source/image/metadata hashes before asserting integrity or portability.

## Capture preparation and limits

[The capture procedure](VSL-2_CALIBRATION_PROCEDURE.md) contains target printing,
physical measurements, provenance entry, exact existing-recorder flags, host
extraction command, feedback and STOP instructions. The host selector now
supports more than 24 observations using position, area, projective shape,
roll and image-edge coverage. Identical views are rejected; default cap 50.
Deterministic post-selection farthest-point holdout reserves ceil(20%) across
geometry. Dimensions must be positive finite measured metres. ChArUco IDs are
retained; AprilGrid is rejected. Operator lens/focus fields are preserved.

Selection is a heuristic and does not certify conditioning, lens model,
flatness, physical dimensions or residuals. Synthetic tests exercise selection
and actual OpenCV target detection, never fit camera intrinsics. No 0921 images
are used as calibration-target observations. No calibration artifact is claimed.

**Host tooling is ready for a real measured-board capture workflow.** Physical
capture still requires the user's existing matched device build/SDK and
configured SSH helper package; these were absent in the inspected checkout.
No turnkey board deployment is claimed. If unavailable on the user's capture
host, identifying those inputs is a remaining operational prerequisite. The
known 24-view and undersized-holdout protocol defects are fixed.

## Status

| Stage | Current status | Limits |
|---|---|---|
| VSL-0A | PASS, repository/code audit only | No new board/resource qualification |
| VSL-0B | PASS_WITH_LIMITS | v2 host contracts; no estimator or qualified consumer |
| VSL-1A | PASS / REPRODUCED, encoded scope | Same OpenCV decoder, no hardware timing proof |
| VSL-1B | NOT_STARTED, current localization configuration | Older device evidence remains historical |
| VSL-2-PREP | HOST_TOOLING_READY | Matched deployment inputs remain user-supplied |
| VSL-2 | BLOCKED_ON_CALIBRATION_CAPTURE | Waiting for actual measured-board capture, then later fitting/validation |
| VSL-3 | NOT_STARTED | No SLAM backend started |

All execution in this pass is headless host tests or offline encoded-video
analysis. No live-view/closed-loop SITL/recorded-motor replay, hardware capture,
SLAM/VIO, ExternalNav or flight-controller changes occurred.

## Verification and changed files

See [exact commands, suite results and file list](VSL-0B1_VERIFICATION.md).
Evidence directory: `artifacts/vsl0b1-calibration-prep-20260922-01/` in the
isolated worktree. Historical and primary-workspace evidence remains unchanged.
