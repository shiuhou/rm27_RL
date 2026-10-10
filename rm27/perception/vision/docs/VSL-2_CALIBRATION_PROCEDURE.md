# VSL-2 geometric-calibration capture preparation

Updated 2026-09-22 for VSL-2-PREP. Host selection tooling is prepared; no physical
capture or calibration fitting was performed. VSL-2 remains
`BLOCKED_ON_CALIBRATION_CAPTURE`. See [implementation evidence](VSL-0B1_CALIBRATION_PREP.md).

## Target and physical measurements

For the photographed symmetric circular grid with 7 columns, 7 rows and
30 mm center spacing, use target type `circles_grid`, `pattern_cols: 7`,
`pattern_rows: 7`, and `center_distance_m: 0.03`. The capture tool uses
OpenCV's symmetric circles-grid detector and assigns row-major point IDs.
Measure the spacing on the physical print; a nominal value alone is not a
measurement. Its target JSON also requires a unique `physical_target_id`.

Use the generated **chessboard: 9 × 6 inner corners (10 × 7 squares)** for the
first session. The generator produces a 300 × 225 mm SVG including a white
margin, with nominal 25 mm squares. Print on A3 at **100% / actual size**, no
fit-to-page. Mount it flat on a rigid board; do not bend or laminate with glare.
Nominal printed dimensions are not measurements.

From `/home/shiuhou/Projects/rm27_drones_slam` on the host:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.generate_calibration_target --out artifacts/calibration-board-01/chessboard.svg --corners-x 9 --corners-y 6 --square-mm 25
```

Use a ruler or caliper to measure the full 10-square horizontal span and
7-square vertical span, checking several rows/columns and target flatness.
Divide by the respective square counts; both estimates must agree. Enter actual
metres, not millimetres, in `square_size_m`. The tool rejects axis measurements
that differ by more than 1% from this value (a preparation sanity check, not a
metrology accuracy certification). Reprint if print scaling is anisotropic.
Give the physical board a unique ID and retain its SVG, measurements and photo.

Create `artifacts/calibration-board-01/target.json` with these keys, replacing
`MEASURED_METRES` and `YOUR_BOARD_ID` with the actual number and string:

```json
{
  "type": "chessboard",
  "corners_x": 9,
  "corners_y": 6,
  "square_size_m": "MEASURED_METRES",
  "physical_target_id": "YOUR_BOARD_ID"
}
```

The quoted measurement is intentionally invalid until replaced with a measured
JSON number. `charuco` is also implemented, with positive measured
`square_length_m > marker_length_m`, square counts and an OpenCV dictionary;
IDs are preserved. ChArUco requires at least 70% of inner corners spanning 70%
of each board axis. **AprilGrid is not implemented.** Do not substitute it.

## Camera and provenance

Use the existing OS04A10 **`full180` mode, 1344 × 760, requested 180 fps,
full field of view**. Use its direct-VENC recording path. This is the named
configuration for a new calibration session; matching pixel dimensions does
not prove the lens/focus/mode matches 0921. No scaling of images or provisional
full180 fx/fy values is permitted as calibration.

Copy `localization/calibration_session.template.json` to
`artifacts/calibration-board-01/session.json`. Fill actual lens identity (or
`UNKNOWN` if unidentifiable), fixed focus note, observed crop/resize state,
device build commit, board revision, executed recording command and measured
spans/counts. Record any unknown sensor-internal crop honestly; no downstream
resize/crop is allowed. Keep the physical lens, mount, and **focus unchanged
through every take**. Preserve actual exposure/gain if available; leave null
otherwise. Raw VIN PTS remains UNKNOWN unit/domain/semantic unless independently
verified. Encoded PTS never becomes exposure time by inference.

## Existing recorder prerequisite and exact capture command

The existing device recorder source was inspected at device commit
`c7d47b20a5a948db1e55f99fb128ee7616d02e1c`. Its **matched official driver build,
MaixCDK/media dependencies and configured SSH helper package are not present in
the inspected workspace**. Their existence on the user's device host must be
confirmed before running the command below. This pass does not build, deploy,
flash, or connect to a device. If those inputs are unavailable, stop and supply
them; do not substitute a guessed SDK or another camera mode.

On the configured device host, from that existing device component checkout,
set these three paths to the actual installed directories (no default path is
asserted to exist), then run one bounded 30-second take:

```bash
export VSL_DRIVER_BUILD=/absolute/path/to/matched-official-build
export VSL_MAIXCDK=/absolute/path/to/matched/MaixCDK
export VSL_SSH_HELPERS=/absolute/path/to/configured/maixpy
/home/shiuhou/venvs/mujoco/bin/python tools/os04a10_highfps/run_device.py --driver-build "$VSL_DRIVER_BUILD" --sdk "$VSL_MAIXCDK" --skill "$VSL_SSH_HELPERS" --official-mode full180 --seconds 30 --direct-venc --record --system-media-lib --remote-root /root/os04a10-tests
```

Stop other camera apps before starting; do not run two camera readers. The
recorder creates a unique `.maixpy/runs/os04a10-<timestamp>-<suffix>/` on its host
and `/root/os04a10-tests/os04a10-<timestamp>-<suffix>/` on the device. Retain
**the whole run**, especially `record.h264`, `run.json`, `capture.json`,
`process_exit_code`, `frames.csv`, `encoded_frames.csv`, sensor/VIN/MIPI and VENC
readbacks. Read their actual results; a directory name does not prove success.
Hash the raw recording before transfer/analysis and verify after transfer.

Repeat 2–4 short takes with the same fixed configuration if needed, holding each
pose still for about a second after movement settles. Aim for **30–50 diverse,
sharp, unclipped views**, not rapid motion or repeated copies of one view:

- Cover center, intermediate positions, all four borders and all four corners.
  Keep the entire board visible; bring detected corners near each image edge.
- Vary distance so the board occupies small, medium and large image areas.
- At each distance include frontal and tilted views about both board axes,
  typically 15–45 degrees as an operator guideline, plus positive/negative roll.
- Avoid blur, glare, shadows hiding corners, focus changes and target bending.

The recorder does not show acceptance feedback. **Selection feedback is offline**
after transfer, not live on the M3C. Preserve raw takes unmodified in a new
host artifact directory, for example `artifacts/m3c-calibration-01/raw/take01/`.

## Host extraction and holdout

From `/home/shiuhou/Projects/rm27_drones_slam`, after the real files exist:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.vision.localization.capture_calibration --source artifacts/m3c-calibration-01/raw/take01/record.h264 --source artifacts/m3c-calibration-01/raw/take02/record.h264 --target artifacts/calibration-board-01/target.json --session artifacts/calibration-board-01/session.json --out artifacts/m3c-calibration-01/selected-01 --step 30 --max-views 50 --min-sharpness 50
```

Supply one `--source` for each actual take, in recorded order. Each inspected
frame prints JSON with accepted/rejected and a reason (`target_not_detected`,
`similar_geometry`, `sharpness_below_threshold`, clipped/small board, or cap).
Accepted images are PNGs under `selected-01/images/`; all decisions, corner
coordinates/IDs, hashes, source indices and provenance are in
`capture_manifest.json`. Failed writes are failures, never accepted images.
Rejected frames remain available in the original recording rather than copied.

Selection compares image centroid, log board area, projective shape/tilt
proxies, roll modulo 180 degrees and distances to image edges. It is a heuristic
for view diversity, not a calibrated board-pose estimate. The cap defaults to
50 and is configurable; repeated identical observations cannot fill it. The
sharpness threshold of 50 is provisional and must not be lowered just to fill
slots. Review the selected images for erroneous detections and coverage.

After all takes, deterministic farthest-point selection reserves
`ceil(0.20 × accepted)` validation observations across the geometry descriptors;
ties use source/acceptance order. Remaining observations are marked `fit`.
Keep this split frozen before any fitting. It is a **same-session holdout**,
not independent capture. Inspect both subsets for edge/distance/tilt coverage;
if insufficient, collect more varied views into a new selection directory.

`selection_checks_passed` requires at least 30 accepted, at least 20% holdout,
and no known source mutation, image-write or decode-count failure. Bare H264
may provide no declared frame count: completeness then remains explicitly
UNKNOWN and must be checked against the retained recorder logs. EOF alone
cannot prove raw-stream completeness. Tool success never marks VSL-2 PASS.

**STOP after capture, extraction and image/provenance review.** Preserve and
provide the real dataset and recorder logs. Do not fit intrinsics, choose a lens
model by sensor name, start SLAM/VIO/VSL-3, or connect to ExternalNav. A later
fitting pass must predeclare numerical residual gates, choose a justified model,
validate the held-out data and produce a provenance-matched calibration artifact.
