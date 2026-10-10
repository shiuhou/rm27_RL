# Published RM27 bench/offline research

Only an explicit curated subset of this normally ignored directory is versioned.
Paths are retained so the already tested imports keep working; no research module
has been silently promoted into the production estimator or flight stack.

Start with `../HANDOFF.md` and
`../rm27/perception/vision/docs/PX4_M3C_CONNECTION.md`.
Newest constraints supersede old next-action paragraphs: no spare UART, Stage A/B
retained, Stage C closed. All dated validation reports remain historical snapshots.

## Included

- Stage A/B report, heartbeat and synthetic ODOMETRY helpers + unit tests.
- Passive IMU decoder/receiver, timestamp boundary and rate experiment reports.
- Integral diagnostics, fixed PX4 message definitions, observation-model tests.
- Strict finite ULog-over-MAVLink reassembly, synthetic tests, saved replay script,
  validation report and not-yet-executed trial proposal.
- Selected offline VIO timestamp/FIFO source, OpenVINS diagnostic patches/reports.
- Windows handoff/proposed-Vault-update archives. No Vault write is claimed.

## Offline tests (no hardware)

Existing research environment used pymavlink 2.4.49, pyulog 1.2.4 and NumPy 2.5.3.
These are research requirements, not changes to the root production requirements.
Use an already compatible Python environment; don't install onto a device by
running this guide. `python` below denotes that environment.

```sh
python -m unittest discover -s .maixpy/m3c-ping-20261009 -p test_heartbeat.py -v
python -m unittest discover -s .maixpy/mavlink2-20261010 -p test_odometry.py -v
python -m unittest discover -s .maixpy/imu-input-20261010 -p test_imu_input.py -v
python -m unittest discover -s .maixpy/imu-design-20261010 -p test_offline.py -v
python -m unittest discover -s .maixpy/imu-design-20261010/followup -p test_model_review.py -v
python -m unittest discover -s .maixpy/imu-design-20261010/single-uart/reassembly -p test_ulog_mavlink.py -v
```

The `.msg` files are small definitions from the audited PX4 revision, required by
the layout tests. Synthetic BMI088 IDs are fixture labels, not a real capture.
Tests do not start the hardware scripts, and passing them does not qualify VIO.

The finite reassembler returns ACK *intents*: no serial IO. Live controller remains
planned. Hardware helpers open/configure UARTs only when explicitly launched;
their presence is not permission to execute them or rerun completed Stage A/B.

## Not included / local-only evidence

Raw ULogs, UART captures, full console/JUnit logs, public camera/IMU datasets,
trajectories, PDFs, credentials, virtualenvs, wheels, vendor trees and builds stay
outside Git. Reports identify them and their checksums where available.
`replay_saved.py` requires the exact two local ULogs at its documented paths;
it cannot reproduce that saved-file result from a fresh clone alone.

Older reports link to such local-only artifacts. Their manifests describe the
original experiment directories, **not this published subset**; no claim that a
clone contains every manifest entry is made. Use the connection/resume documents
as the portable current summary, rather than running old capture/restore scripts.
