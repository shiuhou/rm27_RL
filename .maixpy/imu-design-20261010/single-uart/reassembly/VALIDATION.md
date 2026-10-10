# Offline ULog/MAVLink reassembly validation — 2026-10-11

Work began2026-10-10 and completed after midnight Asia/Shanghai on2026-10-11.

## Result

**VERIFIED:** finite byte reconstruction and synthetic negative-path tests.
**PARTIAL:** high-rate PX4→M3C IMU qualification. Live ULog transport, selected
BMI088 capture, latency, source generation coverage and camera clock mapping remain
UNKNOWN. OpenVINS admission remains false. Stage C is not started.

No serial/SSH, hardware capture, firmware/parameter/baud/wiring changes, dependencies,
commit/push or Vault access. EV_CTRL0/Disarmed are last observed states, not fresh
device observations. Existing Stage A/B report SHA unchanged:
`d9c79727ad749e8ec6ba970a2e41c0ef4aa5ccc3ac6e6e7bc53c0f1bbb7ffd0f`.

## Implementation and reuse

`ulog_mavlink.py` imports the existing imu_input.Decoder (pymavlink2.4.49 common
MAVLink2 CRC/parser), uses pyulog1.2.4 for ULog binary decoding and calls the existing
offline.audit for vehicle_imu diagnostics. No new general protocol/type framework.

- `Reassembler.feed_wire(wire, receipt_ns)` accepts bytes and yields logging ACK
  sequence **intents**. It never sends an ACK or opens a device. The receipt token
  passes through the reused decoder; this tool does not retain a timing index or
  infer latency/rate from it.
- Validates source1/1, target1/191, logging lengths/offsets and16-bit logging sequence.
  Other decoded telemetry may interleave. Logging sequence is separate from the
  outer MAVLink8-bit sequence. Expected initial sequence0; wrap65535→0 accepted.
- Only an identical retransmission of the last ACKed logging packet is idempotent.
  Conflicting, unACKed or older duplicates, gaps/reordering, route/source changes,
  bad CRC/data, truncation and resource limits terminally fail the session.
- `finish()` checks all finite ULog record boundaries against first_message_offset,
  topic format/binding order and pyulog corruption. It returns bytes only on success.
  No lost-packet resynchronization, synthesized messages or repaired output file.
- Version1 finite logs without appended sections are supported. Limits apply to
  raw reconstructed bytes64MiB and300000 accepted packets; these are not a claim
  that total Python/pyulog working memory is64MiB. This is an offline finite-session
  tool, not a bounded-latency live decoder.
- `inspect_imu(...)` requires the audited vehicle_imu schema, explicit instance,
  gyro/accel hardware IDs and evidence epoch. Preserves two dt and delta vectors,
  clips/calibration counts; detects intervals/order/gaps/calibration transitions.
  Means remain diagnostics; unknown accel endpoint and camera clock are retained.

## Fresh checks

Commands run with existing `.maixpy/imu-offline-20261008/venv/Scripts/python.exe`:

```powershell
python -m unittest discover -s .maixpy/imu-design-20261010/single-uart/reassembly -p test_ulog_mavlink.py -v
python .maixpy/imu-design-20261010/single-uart/reassembly/replay_saved.py
python -m unittest discover -s .maixpy/imu-design-20261010 -p test_offline.py -v
python -m unittest discover -s .maixpy/imu-input-20261010 -p test_imu_input.py -v
```

Final results: **28 new tests +9 existing diagnostic +10 existing decoder =47
tests passed**, all command exits0. Logs: tests-final.txt, regression-diagnostic.txt,
regression-decoder.txt. Two saved-log replays exit0 in replay-01.txt.
Synthetic two-instance vehicle_imu fixture tests rad/rad-per-second and m/s/m/s²,
dt/ordering, finite values, clipping/calibration/identity and zero-duration dropouts.
Fixtures bearing BMI088 IDs are synthetic labels, not new BMI088 measurements.

| Saved real bytes, wrapped in synthetic MAVLink | Bytes | Packets | SHA unchanged | Logger dropout events |
|---|---:|---:|---|---:|
| Historical micoair-v1.15.2 BMI270 |6896444|27697|d94a142c…|391|
| Saved PX4 d6f12ad1 A2 BMI270 instance1 |5597928|22482|3815e9e8…|306|

Both reconstructions are byte-identical to the saved original, not merely equivalent
parsed data. Historical FIFO counts remain12648gyro/11933accel and current-firmware
saved A2 counts10251gyro/9626accel. Dropout sums9804/8325ms unchanged; corruptionfalse.
A2 BMI270 is not the currently selected BMI088. Neither file is a live MAVLink ULog
capture; framing, routing and receipt tokens were generated offline.
Missing-packet/reorder/truncated-wire injections on both files were rejected, and
finish() remained rejected afterward. The replay never writes a new ULog.

## Failures preserved

tests-red.txt:24 tests fail because implementation absent. tests-green-01.txt:
23 pass,1 test errors because its first-header offset was rejected earlier than the
intended final boundary assertion. Moved the injected error to a later packet,
retaining the rejection requirement;24 pass in tests-green-02.txt.
tests-red-02.txt: new source-completeness summary test errors because summary absent;
implemented explicit UNKNOWN status, then28 pass. No acceptance threshold relaxed.
The sequence-wrap fixture metadata was corrected to a valid keyed ULog I message
before implementation; no failure was hidden or counted as passing.

## Important observable limits

Clean reconstructed framing does not prove source completeness. If a stream ends
at a valid record boundary and no later sequence arrives, omitted terminal packets
can be undetectable without independent source/close counts. The tested complete
prefix case therefore reports upstream_source_complete UNKNOWN and inadmissible.
Likewise continuous logging packet sequences do not reveal integrals overwritten
in vehicle_imu queue1 or before logging. Saved O messages remain loss evidence,
including duration0 events; there is no loss-free or sample-count PASS.

The actual writer can leave pending partial chunks on stop. A live trial must record
startup, header/data transition, stop handshake and tail coverage; a truncated tail
cannot be fixed by padding. The ACK intents need a separately reviewed single-owner
live session controller before real streaming. Existing passive receiver cannot
complete the start/ACK handshake by itself. No firmware change has been selected.

Next trial proposal is `PROPOSED_TRIAL.md`; it requires communication/logger approval.
The current implementation is sufficient to review and test reassembly offline.
The next software task is the live session controller and its offline timeout/
restoration tests, before attempting this proposal on devices.
