# Generated PX4 recording storage

Completed generated `frames.jsonl` files can be stored losslessly as
`frames.jsonl.gz`. The archive first hashes the original bytes, compresses to
a temporary file, verifies decompression and checks that the source has not
changed. It creates a byte/hash manifest before removing the uncompressed copy.
An existing archive cannot be overwritten. Hardware logs and user attachments
are outside the supported command.

Replay, contact/recovery scoring, surface audits and perception export accept
the logical `frames.jsonl` path through `telemetry.recording.open_recording`.
Their recording hashes refer to the original decompressed bytes. Scripts that
read files directly can use Python `gzip.open(..., 'rt')` or `gzip -dc`.
The frozen image batch can enable `compress_recordings`; compression occurs
after independent scoring and retains unsuccessful attempts as well.

Verified storage runs `artifacts/px4-lossless-frames-storage-01` and `-02`
archive 40 completed S2 recordings, saving 5,180,678,508 bytes. A replay from
the first compressed flight passes with zero error over 38,796 input frames
in `artifacts/px4-archived-recording-replay-01`. This is recorded-input physics
replay, not a new autonomous flight or a flight determinism claim.

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
/home/shiuhou/venvs/mujoco/bin/python -m rm27.telemetry.recording \
  artifacts/px4-image-nominal-completed \
  --output artifacts/px4-storage-new
```

Use this command only after all workers have stopped. It requires each run's
PX4 metadata and final summary. It does not infer mission success from archive
integrity or alter existing acceptance summaries.

`px4-recording-archive-10` subsequently verifies116 additional closed generated
recordings, saving8,005,049,140 bytes. The three active nominal/condition/return
fault batches are excluded from that sweep. Failed attempts remain present,
with exactly the same decompressed frame bytes and logical hashes.

## Auxiliary logs and camera images

`tools.px4.archive_closed_auxiliary` verifies exact gzip roundtrips of closed
generated native JSONL and ULog before removing uncompressed copies. The
completed `px4-auxiliary-storage-sweep-02` saves12,406,988,419 bytes across256
closed runs. Original summaries and logical hashes remain. Restore auxiliary
files before tools which require raw JSONL or `.ulg` paths.

`tools.px4.camera_storage` stores generated gray/RGB frames in lossless video
with a pinned local ffmpeg7.0.2 from imageio-ffmpeg0.6.0. Every decoded frame
must reencode to exactly the original PNG byte SHA before any source PNG is
removed. A larger video leaves originals intact. Timestamp and
sequence-timestamp filenames retain acquisition order. This is storage,
not a video approximation or flight qualification. A full3332-frame flight
passes every original byte roundtrip:117,000,260 original bytes to52,459,272
video bytes (`px4-lossless-camera-full-run-development-01`). Restore cameras
before offline tools that expect individual PNG paths. Batch storage happens
after flight scoring and preserves failures as well as successes.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m tools.px4.archive_closed_auxiliary \
  artifacts/COMPLETED_RUN --restore --output artifacts/NEW_AUX_RESTORATION
/home/shiuhou/venvs/mujoco/bin/python -m tools.px4.camera_storage \
  artifacts/COMPLETED_RUN/return_camera --restore --output artifacts/NEW_CAMERA_RESTORATION
```

Restore verifies source hashes and refuses to overwrite different existing
files. Original hardware evidence and user attachments are outside these
commands. Keep every output directory unique and run storage only after all
flight/scoring workers for those inputs have stopped.

The encoder provisioner is reproducible in an isolated new directory:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m tools.px4.setup_video_storage \
  --runtime artifacts/deps/NEW_VIDEO_RUNTIME \
  --output artifacts/NEW_VIDEO_PROVISION
```

It installs `imageio-ffmpeg==0.6.0` without dependencies and verifies the
Linux x86_64 ffmpeg binary version and SHA-256. The isolated development
installation in `px4-lossless-video-provision-development-02` passes and
matches the original validated runtime. Installing another copy does not
change a frozen batch's runtime or evidence.

At the user's pause, `px4-camera-storage-sweep-02` is suspended with its
current encoder transaction preserved. Its rolling `passed` field describes
completed camera archives only; the sweep lacks `complete: true` and remains
unfinished. Process identities and completed-camera counts are recorded in
`artifacts/px4-user-pause-checkpoint-01`. Resume this transaction before
starting another archive for its camera directory.
