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
