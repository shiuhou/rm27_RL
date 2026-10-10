# Camera–IMU timestamp boundary — preparation only

## Integral / DDS follow-up — 2026-10-10

Exact-source audit in `../imu-transport-20261010/VALIDATION.md` changes no admission
rule below. A future integral record must retain delta_angle/delta_velocity,
separate dt, source and publication timestamps, both IDs, clip/calibration counters,
coning/calibration semantics, boot/source epochs and original bytes. One common
gyro anchor does not establish a separate accel acquisition endpoint. Conversion to
OpenVINS point samples is NOT implemented/qualified by dividing by dt alone.
DDS serializer shifts timestamp and timestamp_sample by session time_offset;
retain source/reversible mapping provenance and uncertainty rather than labeling
agent-domain values PX4_HRT. Stock DDS sensor_combined10ms latest-copy loses
intermediate~200Hz integral updates; a packet rate test cannot waive this gate.

Reuse, do not replace:
`../offline-vio-20261009/rm27/perception/vision/localization/experiments/vio_dataset.py`
and its existing test_vio_dataset.py, RM27_VIO_DATA_CONTRACT.md and
RM27_VIO_CAPTURE_PROCEDURE.md. No production schema, adapter or evaluator changed.

`imu_input.imu_record` exports diagnostic telemetry records only. Current fields:

| Field | Meaning / rule |
|---|---|
| source_timestamp,source_unit | Integer FC time, us for HIGHRES_IMU; never receipt time |
| source_domain,source_epoch | PX4_HRT plus capture segment label; segment is not a proven cross-capture boot identity |
| receipt_mono_ns | M3C monotonic time after the read completing this frame; chunk-level software arrival only |
| timestamp_semantic | GYRO_INTEGRATION_END for this HIGHRES sender; accel endpoint not separately provided |
| integration_dt_us | null: MAVLink omits both gyro and accel integration durations |
| measurement_semantic | Calibrated integral/dt minus matching estimated bias; not an acquisition sample |
| hardware_device_id | null on wire; separate pre/post FC selection evidence binds likely source |
| mapped_ns,clock_map | null: no fit or synchronization performed |
| synchronization_status,vio_admissible | UNKNOWN,false |

Keep gyro/accel units rad/s and m/s^2, body FRD; do not negate axes or remove
gravity without a separately specified interface. Full raw bytes and fields_updated
are preserved. id=0 is not a BMI270/BMI088 device identity. Treat source changes,
clock reversal/reboot and gaps as segment boundaries, never interpolate silently.

Future camera record must retain image/frame ID, actual mode/lens/crop, original
VIN timestamp with unit/domain/epoch, event semantics (exposure vs readout vs
receipt), and separate M3C receipt_monotonic_ns. Do not invent MID_EXPOSURE from
video PTS. Camera-to-FC map remains absent until its evidence exists.

Future qualified maps use existing `map_timestamp(raw,mapping,source_epoch=...,
source_domain=...,source_unit=...)`: explicit integer anchors, decimal scale,
direction, valid interval, bounded uncertainty and verified artifact reference.
That function checks declared structure, not truth of evidence. Bracketing uses
existing `bracket_mapped_streams` with separate gyro and accel streams and verified
ACQUISITION/MID_EXPOSURE semantics. Today's processed telemetry MUST fail that
admission; do not rename its semantic to make it pass. An integral-aware estimator
input interface would be a separate reviewed decision, not this task's conversion.

Tests: existing29 tests pass plus new rejection of UNKNOWN map and processed
telemetry admission. No calibration, clock fitting, camera capture or VIO executed.
Full Stage C remains the later REAL VIO -> PX4 qualification gate; synthetic
ODOMETRY and this passive diagnostic are not substitutes.
