# Windows proposed Vault update archive — not ingested

Preserved before fast-forwarding main to the remote OpenVINS checkpoint.
This is a repository proposal, not evidence of any Vault write.

## Offline ULog reassembly evidence — 2026-10-11

`.maixpy/imu-design-20261010/single-uart/reassembly/VALIDATION.md` records47 passing
tests, two SHA-identical saved-log replays with synthetic transport, source/dropout
preservation and terminal fault rejection. Complete-prefix test demonstrates why
packet/framing continuity cannot prove source completeness. ACK intents are offline;
live controller/restore remains planned. No selected BMI088 capture or OpenVINS PASS.
Repository proposal only; Vault not accessed or ingested.

## Latest single-UART constraint and transport alternative — 2026-10-10

User reports no spare UART: supersede the prior spare-pin verification action.
See `.maixpy/imu-design-20261010/single-uart/REVIEW.md`. Exact PX4 source supports
MAVLink ULog carrying full vehicle_imu records without inventing a new IMU dialect;
this is a bench acquisition candidate, not qualified real-time OpenVINS input.
Unsigned IMU-only200Hz wire model13.94kB/s exceeds115200 capacity. Logger polling,
queue16 transport, queue1 source, ACK blocking and batching require validation.
Six offline framing/arithmetic check groups pass; no hardware/settings changes.
Only repository proposal updated, not the Vault. VIO-S0 PARTIAL, Stage C closed.

## Latest offline scheduling/model/budget follow-up — 2026-10-10

`.maixpy/imu-design-20261010/followup/VAULT_UPDATE.md` proposes the exact-source
sync-wait risk, metadata-inclusive budget and synthetic observation-model findings.
92 related tests pass, no hardware/firmware/parameter changes, VIO-S0 PARTIAL.
No Vault access/ingestion; source/assumption/hardware boundaries in REPORT.md.

## Latest interface feasibility evidence — 2026-10-10

`.maixpy/interface-inventory-20261010/VAULT_UPDATE.md` records the approved
read-only inventory: USB is current SSH gadget and host ACM is not compiled;
spare UART still needs carrier-pad evidence. No settings changed, no capture or
Vault access. Prior design remains conditional, not a deployable/qualified input.

## Latest offline input architecture — 2026-10-10

`.maixpy/imu-design-20261010/VAULT_UPDATE.md` proposes pinned DDS/measurement
semantics and wire-budget findings, conditional architecture/fallback and remaining
hardware gates.100 related offline tests pass,2 skipped; VIO-S0 remains PARTIAL.
All previous hardware evidence preserved, no new capture or device/configuration
change. No Vault access/ingestion. See its HANDOFF.md for exact commands and limits.

## Latest IMU scheduler and integral-source evidence — 2026-10-10

`.maixpy/imu-transport-20261010/VAULT_UPDATE.md` proposes the controlled50/100Hz
results and exact-source vehicle_imu/DDS/OpenVINS comparison.50.001Hz measured;
100Hz capped62.112Hz at unchanged115200/5760B/s. Restoration verified. Startup
invalid bytes and unqualified input/clock semantics explicitly retained. No Vault
access/write; no Stage C or settings escalation.

## Latest passive IMU evidence — 2026-10-10

`.maixpy/imu-input-20261010/VAULT_UPDATE.md` proposes the new exact-firmware,
passive receive evidence and source semantics.10.11Hz HIGHRES_IMU verified, not
raw or VIO-qualified; current budget blocks original200–400+Hz target. Stage A/B
preserved,Stage C not started,no PX4 settings changes. No Vault access/ingestion.

## Latest MAVLink2 transport evidence — 2026-10-10

See `.maixpy/mavlink2-20261010/VALIDATION.md` and its VAULT_UPDATE.md:
real HEARTBEAT+ODOMETRY parsing and visual-odometry uORB publication verified,
with stop/restart attribution. Synthetic bench data only; no VIO/EKF/time-sync
acceptance or PX4 parameter changes. No Vault access or ingestion performed.

## Latest replacement-card retry — 2026-10-09

SD listing succeeds after restart; parameter import failed because SD params were
missing at boot. Accelerometer calibration IDs changed to0. Same-condition capture
paused for recovery authority; no writes/capture performed. See replacement-card
report latest section. No Vault access or ingestion.

## Replacement-card preflight — 2026-10-09

`.maixpy/px4-sd-control-20261009/VAULT_UPDATE.md` proposes a bounded follow-up:
SD stat/readdir times out after user-reported card swap, while old runtime/mount
state persists. No new capture or writes; RAM advertised parameters preserved.
Clean power-cycle/config audit required before comparing cards. Do not label the
card defective or filesystem empty. No Vault modification or ingestion occurred.

## PX4-only buffer comparison — 2026-10-09

The offline handoff's proposed hardware comparison is now executed. New durable
findings and evidence references: `.maixpy/px4-buffer-ab-20261009/VAULT_UPDATE.md`,
`HANDOFF.md` and `PX4_BUFFER_COMPARISON.md`. Actual64/128/64KiB all lossy;
128KiB is not an accepted fix. Preserve distinct writer/subscriber/sensor evidence,
the initial explicitly excluded tool abort, raw hashes and verified restoration.
VIO-S0/P remain PARTIAL; SD-media-only cause and synchronization remain UNKNOWN.
No Vault read/write or ingestion performed. Historical offline findings unchanged.

## Offline VIO continuation — 2026-10-09

Proposed durable findings are in `.maixpy/offline-vio-20261009/VAULT_UPDATE.md`
and its evidence-backed HANDOFF/OFFLINE_VALIDATION reports: distinct PX4 writer
and subscriber loss boundaries; no-overwrite raw export and explicit clock/epoch
contracts; camera model compatibility; dynamically evidenced native lifetime
defect and isolated input-delivery controls. Keep candidate/test/synchronized
hardware qualification separate. Original thresholds and failure evidence remain.
Both physical devices were disconnected throughout; no Vault read/write occurred.
The research repository is authoritative for VIO source and software experiments;
this is only a proposal and Windows pointer, not completed Vault ingestion.

Record that M3C OS04A10 full180 camera capture recovered after operator-reported hardware repair, and a 2026-10-07 fixed-focus calibration experiment passed its predeclared same-session geometric checks.

Authoritative details and evidence paths are in HANDOFF.md and `artifacts/m3c-calibration-20261007/calibration_result.json`. Preserve distinction between geometric calibration checks and overall VSL-2/SLAM acceptance. Lens identity, physical metrology details and timing semantics remain unresolved.

Reusable workflow: retain complete recorder runs; measure six intervals across a 7x7 circles target on both axes; freeze fit/holdout split before fitting; fit intrinsics on fit observations only and estimate held-out board pose with fixed intrinsics; retain protocol, residuals, input hashes and model coefficients.

Vault was not read or modified for this task. Import only on explicit request.
