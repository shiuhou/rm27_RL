# PX4 / M3C software and read-only log tools

This document describes host simulation and offline tooling. Hardware UART
capture, parameters, firmware and motors are not operated by these commands.
Connection evidence and raw-input references are in
[PX4_M3C_CONNECTION.md](../rm27/perception/vision/docs/PX4_M3C_CONNECTION.md).

## Interfaces

The historical validated connection is M3C UART2 to flight-controller UART3,
115200 baud, 8N1, no flow control. MTF optical-flow/range remains a separate
link. Stage A/B establishes MAVLink2 heartbeat and synthetic ODOMETRY reception;
it does not establish actual EKF2 external-vision fusion or real VIO flight.
The observed HIGHRES_IMU stream near 50 Hz is telemetry, not independent raw
high-frequency IMU evidence.

`rm27.telemetry.logging_session.LoggingSession` accepts an explicitly injected
transport with `write(bytes)`. It opens no UART. The caller supplies monotonic
receipt timestamps and delivers raw MAVLink2 wire bytes. Its finite lifecycle
is IDLE -> STARTING -> STREAMING -> STOPPING -> DRAINING -> COMPLETE. Command
rejection, sequence discontinuity, CRC failure, reversed receipt time and
start/stream/stop timeouts terminate the session. Reliable logging retries
produce ACKs without duplicating the bytes. A STOP ACK and completed drain are
required before returning a validated log. No partial log is reported complete.
Even a framed complete finite session has UNKNOWN upstream source completeness;
it does not automatically qualify IMU input for OpenVINS.

The reassembler is the imported
[finite research implementation](../.maixpy/imu-design-20261010/single-uart/reassembly/ulog_mavlink.py).
It supports the documented ULog v1 subset, verifies format/binding order and
checks the result with pyulog. Appended sections and unsupported flags are
rejected. Existing saved-data replay still requires its original local inputs.

## Baud and processing budgets

115200 / 8N1 has a physical ceiling of 11,520 bytes/s. The existing conservative
software budget is 5,760 bytes/s. The proposed complete 200 Hz IMU-only ULog
stream is about 13,940 bytes/s before startup, metadata and competing telemetry:
121% of the physical limit and 242% of the software budget. That profile cannot
be qualified on this link by a framing test. A faster baud rate and measured
loss/CPU/clock performance remain later hardware experiments.

Host native OpenVINS camera processing is reported separately from end-to-end
latency and separately from M3C performance. The pinned recorded development
run `artifacts/px4-current-flight-openvins-06` has p95 21.09 ms, maximum 39.66 ms
native camera processing. It runs in a host Docker environment, uses synthetic
calibration, and does not establish M3C real-time eligibility.

## Read-only ULog analysis

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.telemetry.ulog \
  PATH_TO_ORIGINAL.ulg --output artifacts/ulog-analysis-new

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python \
  -m pytest tests/test_px4_logging_session.py -q
```

The analyzer hashes the input before decoding and reports initial parameters,
mode/arming state, topic rates, gaps, dropouts and actual estimator fusion flags.
Its `passed` means the file decoded without corruption. Fusion, control or
mission qualification needs the appropriate topic evidence and independent
mission scorer; file decoding alone is insufficient. Existing `.BIN` historical
analysis remains available under the ArduPilot research entries.

The six root session tests use fake wire bytes only. The saved-input end-to-end
tool `rm27.telemetry.logging_replay` additionally wraps the complete 52,640,764
byte development V4 SITL ULog into MAVLink2, starts the finite session, retries
the reliable header, checks ACKs, stops and drains it. The accepted
`px4-ulog-saved-session-01` result reassembles all 211,409 packets byte-identically
with SHA256 `65c69ac1752f4bc865aa8aa6c941fc853c02e6edd4fc2dded8e41c1fc4a8dc6f`.
Dropping packet100 in `px4-ulog-saved-session-loss-01` correctly rejects the
sequence gap and writes no partial ULog. Source bytes remain unchanged.
This is saved-input replay over a synthetic wire, without device capture or
UART timing; upstream source completeness and real VIO eligibility stay unknown.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.telemetry.logging_replay \
  PATH_TO_SAVED.ulg --output artifacts/ulog-wire-replay-new
# Expected terminal loss rejection, not a successful logging session:
/home/shiuhou/venvs/mujoco/bin/python -m rm27.telemetry.logging_replay \
  PATH_TO_SAVED.ulg --drop-packet 100 --output artifacts/ulog-wire-loss-new
```

Research test commands,
read-only fixture reports and local evidence limitations are listed in
[.maixpy/README.md](../.maixpy/README.md). No device deployment or live capture
qualification is implied by these software tests.
