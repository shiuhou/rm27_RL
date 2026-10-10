# Engineering Handoff

## CURRENT — entirely offline VIO continuation, 2026-10-09

### Repository state

Research repo `/home/shiuhou/Projects/rm27_slam_vio_openvins`, branch
research/vio-openvins-baseline, HEAD4637a5f589801f54ecab146a389db0c81ca1d13f plus
scoped uncommitted additions. Prior dirty HANDOFF/VAULT_UPDATE/IMU diagnostic docs
preserved in offline-20261009/repo-before. Windows main @b8e7299 remains the camera
workspace; `.maixpy/offline-vio-20261009` is a local editing/evidence mirror, not a
new estimator framework. No commit/push, dependencies, Vault or physical access.

### Task objective

Exhaust useful saved-data/software work across S0 logging, native reproducibility
and real-pipeline preparation while M3C and PX4 are disconnected. Preserve original
criteria and failure evidence; no fabricated synchronized inputs or simulator PASS.

### Actual changes

- Added/tested experiments.px4_ulog for identity-bound separate FIFO expansion,
  exact association diagnostics, transport budget and no-overwrite raw ULog export.
  Original12648/11933 samples,391 dropouts and11930 matched timestamps reproduced;
  source hash unchanged. This is not a loss-free or synchronized dataset.
- Added/tested experiments.vio_dataset for explicit affine clock maps/epochs,
  bounded validity/uncertainty, separate bracketing, calibration/noise/model checks.
  Existing admission/schema/adapter/evaluator and acceptance thresholds unchanged.
- Camera model diagnostic uses fixed27-fit/7-validation records; explicit4-model
  refit RMS0.171/0.202px passes original geometric gates. Candidate only: lens/mode
  provenance, timing/extrinsics/noise are still unqualified. Original5-model retained.
- Isolated ASan replay dynamically proves OpenVINS callback stack-use-after-return.
  Minimal timestamp-value-capture patch and separate ASan/Release builds retained.
  Fixed ASan full replay:2800 poses/all3 exits0/no finding in instrumented TU.
  Three Release controls and existing-adapter saved-output parity completed;
  VIO-P still fails unchanged historical1cm criterion in one new run.
- Native trace2 proves differing IMU feed omissions137/133; joined-worker controls
  worsen coverage and are rejected. Reliable-input trace2 receives all29120/2912
  source timestamps. Uninstrumented2 and full existing-adapter1 also complete;
  all FIVE candidate state text hashes identical, ATE0.07011501076521641m.
  Full adapter reports ADAPTER_RUNTIME_AND_METRIC_PASS against ORIGINAL native02,
  ATEdelta0.0000010195686508396307m. No reference/threshold/math/adapter changes.
  Candidate patches and launcher/QoS recipe retained separately, not default/live
  promotion. Failed original/lifetime-only/serial cohorts are not reclassified.
- Prepared PX4_NEXT_CAPTURE_PLAN.md and RM27_VIO_CAPTURE_PROCEDURE.md, including
  actual-buffer64/128/64KiB matrix, independent loss boundaries, transport budgets,
  strict clock/frame semantics and staged calibration procedure. Not executed.

### Verification commands and observed results

`py -3 -m pytest -q --junitxml=validation/windows-final-v3-tests.xml` in local mirror:
282passed/1skipped. Linux `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
/home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q` likewise282passed/1skip.
ROS2-sourced test_asl_transport.py:7passed. Linux system full:281passed/1skip/1fail;
failure is the pre-existing missing ChArUcoDetector API, not hidden or waived.
External/local OFFLINE_VALIDATION.md records commands/versions/inputs/results.
verify_saved_ulog.py, camera_model_audit.py, run_fixed_cohort.py --count3,
analyze_cohort.py fixed and verify_saved_adapter.py all exit0 for their scoped
checks. Lifetime-only run03 historical gate remains FAIL. Later reliable candidate
saved parity and full runtime pass the same original gate. final_offline_audit.py
exit0 verifies original and candidate frozen manifests, five identical state files,
all16 candidate process exits0 and all six diagnostic trace re-parses.

### Failed attempts

Initial partial-ASan Eigen allocator mismatch preserved, diagnostic macro corrected;
subsequent ASan reproduced the independent stack lifetime fault. Linux system
ChArUco API failure persists as baseline environment evidence. Expected TDD red
tests retained in execution record. Remote shell quoting failure was bounded and
retried safely; no data/hardware state inferred from failed command output.
OpenCV5 synthetic repeat found ~1.37e-10 K differences even with identical solver
inputs. A cross-call bitwise equality test was invalid for holdout isolation.
Replaced that assertion with exact captured solver-input equality and exact
same-call output retention; kept geometric limits/fit data/code unchanged.
cv5-refit-diagnostic.json and failed/passing test XML retained. This is no claim
of fixing OpenCV numerical repeatability. Patch check without context initially
failed; corrected context before application, original launcher not modified.

### Decisions and rationale

Reuse existing framework, immutable data, source and evaluator. Fix only the
dynamically evidenced lifetime defect; keep callback/QoS controls separate from
estimator math and frozen acceptance. Independent sensor streams and explicit
maps beat inventing correspondence, offsets or calibration metadata. Original
criteria remain in force even if a new run is numerically more accurate.
See `.codex/tasks/2026-10-09-vio-offline/decision-record.md`.

### Verified facts

Saved ULog recording loss is real; card-only cause is not isolated. Lifetime fix
is exercised on saved real input. Reliable-input public replay candidate is
VERIFIED for this finite cohort, including unchanged historical metric gate and
full adapter runtime. VIO-P rollup stays PARTIAL: failed old/default paths and
isolated/not-promoted candidate must remain distinct. Input-delivery controls
combine reliability/depth/discovery; individual effects and every historical
failure are not uniquely decomposed. No universal determinism/performance claim.
No real synchronized RM27 camera/IMU record or qualified transform/clock map exists.

### Assumptions and open questions

Firmware source hashes are prior readbacks, not current disconnected-device state
or reproduced binaries. Timestamp helpers trust evidence claims structurally;
existing artifact/identity gates must verify actual evidence. M3C VGTR/FC TX3/RX3
labels do not qualify electrical/OS port mapping or timing. No deployment choice
is implied by a diagnostic control or camera-model candidate.

### Risks

Old lossy data can test software but cannot qualify VIO, noise or hardware clocks.
No-dropout is not enough without subscriber/timestamp/identity checks. Replacing
the old historical baseline with a more convenient run would invalidate its gate.
Diagnostic instrumentation affects timing; no M3C performance inference is valid.

### Next actions

Independent useful offline work on available evidence is complete. Remaining
BOUNDARIES: new-firmware loss-free capture, sensor/clock/exposure evidence,
rigid extrinsics/noise and real synchronized RM27 data need physical evidence.
Transport and four-model calibration adoption, plus promoting the isolated
reliable-only replay profile, are explicit interface/acceptance decisions, not
automatic defaults. Existing evidence cannot reconstruct omitted physical samples.

Exactly ONE next physical action: connect ONLY PX4 via USB for the prepared
new-firmware A1/B/A2 actual64/128/64KiB logging experiment (fresh props-off/disarmed,
identity/config backup and restoration checks). It verifies buffer sensitivity
of writer dropouts separately from logger subscription gaps. No M3C required.
Experiment remains NOT EXECUTED; no device reconnection occurred in this task.

### Rollback point

Restore only this task's additive docs from offline-20261009/repo-before if needed;
new helper/tests/patches are separate scoped additions. Preserve original/failed
raw data, old13-run cohort, original source/build/config and all new evidence.
Do not use old FC configuration backups as restore targets for new firmware.
VAULT_UPDATE is a proposal only; no Vault read/write or ingestion occurred.

## CURRENT — exact-hash VehicleIMU/logger comparison, 2026-10-09

Read-only source task at research HEAD4637a5f with all prior dirty docs retained.
Compared old4817c061 to newd6f12ad1; added `PX4_V117_SOURCE_AUDIT.md`.
VehicleIMU gap increments once per successful sensor_accel/gyro read with
nonconsecutive uORB generation, not per missing raw sample or timestamp gap.
Both consumed SI queues are8; FIFO queues1/4 belong to different subscribers.
Startup generation initialization can contribute historical counts. Recent
idle-logger increments still lack event-correlated scheduling/lifecycle evidence.

Run/UpdateAccel/UpdateGyro/UpdateIntegratorConfiguration functions identical;
BMI270 driver and four sensor messages byte-identical. Logger adds memory-aware
buffer capping: -b64 is requested, actual allocation unverified while idle.
Backend parameter/startup/status/default-topic/watchdog/metadata changes recorded;
core ring rejection/write/fsync/reclaim and dropout/high-water behavior unchanged.
Do not infer an upgrade fix, physical sensor loss or exact missing-sample count.

Verification: offline verify_source.py reports23 paired files/13 identical,
16 source assertions and7 abstract queue cases PASS; not a compiled PX4 test.
External evidence px4-v117-source-audit-20261009 retains sources/hash URLs/diffs.
Initial404 paths resolved using upstream tree; no hardware or source changes.
Audit/contract/baseline notices and this handoff updated, no schema changes.
Next: read-only current IMU_INTEG_RATE/work-queue/quiet-interval counter check,
not executed here. No new capture, parameter writes, M3C, Vault, commit or push.
VIO-P/S0 PARTIAL. Rollback only these dated docs using repo-before; retain evidence.

## CURRENT — v1.17.0 runtime baseline verified, 2026-10-09

User confirmed COM19/QGC availability; existing read-only helpers verified board
MICOAIR_H743_V2, Release1.17.0, hash d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
Selected IMU is now BMI088 instance0 (gyro6684690/accel6946834), not BMI270;
BMI270 remains instance1/device3604506. Logger idle before/after, process
`logger start -b 64 -t -m all`,183 subscriptions, SDLOG_BACKEND3/profile1/mode0.
Commander Disarmed. No logger start/stop, parameter writes, reboot or new capture.

Evidence/commands/limits: `rm27/perception/vision/docs/PX4_V117_BASELINE.md`;
external Windows/host `px4-v117-baseline-20261009` contains four readback logs.
All helper sessions exit0, absent-custom-path command failures retained. One-second
uORB rates are publications only: BMI088 gyro666Hz/accel802Hz; BMI2701580Hz each.
Drivers had nonzero historical counts unchanged on recheck; VehicleIMU1 gap
counts increased accel7->8/gyro4->5 despite idle logger. Cause not yet isolated;
do not attribute this to old SD loss or claim firmware upgrade solved it.

Updated IMU audit/contract runtime notices, added baseline report, preserved all
previous dated work at research HEAD4637a5f plus prior dirty docs. No production
code/schema/tests changed; verification is live readback, not synthetic tests.
Next: read-only exact-new-hash VehicleIMU/BMI270/logger comparison before any
controlled capture. SD root param_import_fail.txt exists but content/cause/date
uninspected. No parameter repair, IMU switch, M3C or UART qualification implied.
VIO-P/S0 PARTIAL unchanged. Rollback: this pass changed documentation only,
repo-before snapshots retained; no device restoration needed. No commit/push
or Vault action. Earlier USER-REPORTED-only version notice is superseded here.

## CURRENT — historical logger loss diagnosis / firmware boundary, 2026-10-09

### Repository state and task objective
Research branch `research/vio-openvins-baseline`, baseline HEAD
`4637a5f589801f54ecab146a389db0c81ca1d13f`, clean before this documentation
update. Objective: read-only investigation of existing failed BMI270 ULog.
Windows baseline `b8e7299` with prior dirty files preserved. No Vault access.

### Actual changes and verified facts
Added `rm27/perception/vision/docs/PX4_LOGGER_LOSS_DIAGNOSIS.md` and an additive
notice in the historical capture report. Host-only frame/load/gap analysis and
embedded perf extraction saved externally under `imu-logger-diagnosis-20261009`.
Old ULog raw SHA256 unchanged. Embedded postflight `logger_sd_write`: 248 calls,
71.36544 ms mean, 132.993 ms max; fsync 30 calls, 12.076 ms max. Old pinned
source confirms write/fsync-before-buffer-reclaim and overflow returns. Full-rate
four-topic model is 868,870 B/s; 64 KiB holds only 75.43 ms with zero drain.
The misleading high_water=0 is reset on new dropout, not absence of overflow.
All 164 gyro FIFO gaps >10 ms overlap gaps in the other three topics. This
localizes a recording/storage-path bottleneck, not proven card-only failure.

### Verification commands and observed results
Windows `.maixpy/imu-offline-20261008/venv/Scripts/python.exe -m unittest discover
-s .maixpy/imu-logger-diagnosis-20261009 -p test_loss.py -v`: 3 passed.
`analyze_loss.py .maixpy/imu-offline-20261008/bmi270-disarmed-01.ulg`: independent
ULog frame counts agree with pyulog; final output `loss-analysis-v2.json` retained.
Source/hash references, exact commands and limits are in the diagnosis report.
No new device capture or production regression result is claimed.

### Failed attempts / assumptions and open questions
Existing PX4 reference checkout dd0ad74 does not contain the reported old
4817c061 object; read exact-hash upstream sources instead. GitNexus unavailable,
direct source inspection used. Old log has no logger_status topic and no exact
per-call write/queue timeline. Media vs driver/filesystem vs scheduling and
additional accel FIFO losses remain unisolated.

### Decisions, risks and next actions
User reports flashing PX4 v1.17.0 during this pass. Treat as USER-REPORTED, not
read-back verified. Old diagnosis remains historical v1.15.2 evidence. Before
any experiment read back new full hash/board, selected device IDs/instances,
logger process, SDLOG parameters and custom topic file. Do not assume instance1
or restore old backups to new firmware. A buffer-only contrast across firmware
versions is not a controlled comparison. New capture needs new-baseline backup,
disarmed/props-off verification and bounded logging-only approval. No hardware
assembly, camera calibration, M3C integration or live stream is authorized here.
VIO-P and VIO-S0 remain PARTIAL; no threshold change or automatic PASS.

### Rollback point
Only additive research documentation changes; pre-edit files preserved in
external `imu-logger-diagnosis-20261009/repo-before`. Preserve all raw evidence.
No FC/M3C connection or settings change to restore this pass; no commit/push.

## CURRENT — authorized offline BMI270 capture, 2026-10-08

### Repository state / objective
Research worktree remains research/vio-openvins-baseline at13c7a77 with existing
uncommitted VIO work preserved. Objective: short props-off/disarmed FIFO ULog
capture, verify offline data chain, restore original logger settings.

### Actual changes and verified results
One ULog captured through temporary9-topic list, BMI270 instance1/device3604506;
logger reader2500Hz, original64KiB buffer, logger on/off only. User explicitly
confirmed prop removal. No parameter changes. FC original retained at
/fs/microsd/log/sess100/log100.ulg. Two downloads match6,896,444 bytes and SHA256
d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d.
pyulog1.2.4 parses without file corruption. Actual18.122s file/17.9s IMU span.
Gyro/accel FIFO12648/11933 retained samples, median633us cadence but effective
706.681/666.823Hz due to gaps.391 ULog dropouts, max sample gap111.409ms. Sample
times ordered/no duplicates; no NaN/clipping; SI/FIFO scaling association agrees.
No loss-free-data PASS. Audit, contract, status, this handoff, Vault proposal and
validation record updated; new `PX4_IMU_OFFLINE_VALIDATION.md` is detailed report.
Evidence root /home/shiuhou/Projects/rm27-vio-20261007/imu-offline-20261008;
Windows source .maixpy/imu-offline-20261008. Helpers/tests are research-only.

### Restoration, safety and failed attempts
Fresh ps confirms original `logger start -b 64 -t`; idle mode all; seven SDLOG
values unchanged and custom topic file/created dirs absent again. Temporary
file181bytes preserved locally before deletion; no existing logs removed.
Runtime subscription count142->147 is recorded, not hidden; this may reflect
optional-topic availability after restart; exact difference not isolated. ULog records and
observed heartbeats disarmed. Source explains logger-start perf counter reset;
do not mistake lower counters for a reboot or compare them cumulatively.
Remote CRC request failed generically; second independent download/hash used,
not a fabricated remote checksum. Windows helper source-field and Unix-temp-path
issues corrected before capture; failed evidence retained.5 guard +3 synthetic
analysis tests pass, no production regression rerun/claim for doc/helper work.

### Decisions, risks, next action and rollback
VIO-S0 PARTIAL: physical raw export verified, completeness failed for tested
logger config; precise acquisition/exposure timing and synchronization unqualified.
VIO-P unchanged PARTIAL. Do not conflate sensor~1580Hz with logged~667--707Hz.
Next: isolate logger buffer/SD/scheduling bottleneck before any accepted dataset;
no second acquisition or logger tuning performed. No physical action needed now.
Hardware logger config already restored. Revert only this dated documentation
change if needed; docs-before backup is in evidence root, preserve raw ULog and
all older research. No camera calibration, M3C, live stream, firmware, flight
parameters, commit, push or Vault write.

## CURRENT — raw export/timing/transport audit, 2026-10-08

### Repository state and objective
Research worktree /home/shiuhou/Projects/rm27_slam_vio_openvins, branch
research/vio-openvins-baseline, prior recorded HEAD13c7a77; existing dirty VIO
implementation/reports retained. Objective: confirm available PX4 raw export
and clock/transport semantics without collecting data or changing hardware.

### Actual changes and verification
Updated IMU_SOURCE_AUDIT.md, RM27_VIO_DATA_CONTRACT.md, this handoff,
VAULT_UPDATE.md proposal and existing validation record. Windows-local diagnostic
helper gained only logger/SD read-only command allowlist entries. No schema,
adapter, estimator, PX4 config or firmware changes. Exact command/results and
pinned source paths are in the audit's newest section. COM19 logger/parameter/ls
queries exited0: logger idle, profile1/mode0/boot_bat0, SD readable. No tests of
runtime algorithms are claimed for this documentation/diagnostic-only work.

### Findings, rationale and remaining risks
Default sensor_gyro/accel ULog intervals1000ms cannot qualify VIO. FIFO profile
bits target instance0, not selected BMI270 instance1. Proposed first raw export:
explicit instance/device-bound SD ULog then host download, with capture/dropout
qualification still required. HIGHRES/SCALED_IMU are interval-derived; neither
is a raw substitute. Source supports ULog MAVLink streaming for later USB-link
tests; throughput, M3C USB-host/power setup, time mapping and camera exposure
semantics remain unverified. FC timestamps use HRT boot microseconds, driver
DRDY/read-related rather than proven ADC latch. FIFO last-anchor reconstruction
is source-supported, not capture-validated. VIO-P/S0 remain PARTIAL unchanged.

### Failed attempts and verified facts
Custom logger directory/file inspection returned Not a directory; subsequent
ls /fs/microsd succeeded. Do not infer missing SD. Shell sysinit warning persists;
status commands work. Supported source behavior does not prove vendor binary
reproducibility. No sustained export test or new sensor dataset exists.

### Next actions and rollback
Next software action requires approval for reversible logger configuration and
bounded disarmed capture; do not execute from this handoff. User confirms camera
and FC are separate on the desk, superseding the prior photograph request below.
No physical action now. Single later physical step: rigidly mount camera and FC
together when synchronized capture is authorized; do not wire/assemble yet.
Rollback only this dated doc addendum and new local helper allowlist entries;
preserve all previous VIO work. No commit/push/Vault write. Report edits are
working-tree evidence, not a new commit or qualification PASS.

## CURRENT — live COM19 read-only qualification, 2026-10-08

Read official MicoAir manual first; target micoair_h743-v2, BMI088+BMI270,
SPI2/SPI3 corroborated by official v1.16.0 source. Windows COM19 open/read succeeded
at57600; actual ver all says MICOAIR_H743_V2 / micoair-v1.15.2 / hash
4817c0618a1286846116e90c6eb8919efaa013cf, build Dec21 2024. This supersedes
the prior unavailable-FC state and local dd0ad74 source assumption. Device IDs
identify BMI088 instance0, BMI270 instance1 selected. All raw/FIFO/vehicle_imu
topics have two instances. One-second uorb top shows accel/gyro802/666 Hz and
1580/1580 Hz; vehicle_imu199/196 Hz. Internal topic rates only, not link performance.
No raw dataset acquired. Some existing gap counters nonzero; no no-loss claim.

M3C and FC NOT connected (user-confirmed). PX4 USB CDC is current Windows
inspection link; configured57600 is not proof of future UART throughput. DDS
client not running. HIGHRES_IMU remains interval-derived and potentially
estimator-bias corrected; preferred future path preserves one identified
sensor gyro/accel or qualified FIFO pair with sample timestamps and clock mapping.
Raw export, host/FC clock mapping and camera exposure timing are not qualified.
Keep VIO-P and VIO-S0 PARTIAL. Read-only parameter/status checks only; no firmware,
stream settings, parameters, wiring, pinmux, logging, calibration or M3C integration
changed. Diagnostic clients close COM19/release shell ownership. Commands and
observations are detailed in IMU_SOURCE_AUDIT.md; no user console commands needed
to repeat these successful queries. No regression run for documentation-only work.

Single next physical verification: photograph current camera/FC placement with
the FC direction arrow visible, to check relative orientation and rigid mounting;
do not alter wiring or assemble a new setup. Official manual says45deg mounting,
current SENS_BOARD_ROT=0: do not infer a correction without actual mounting evidence.

Changes: audit, contract, status, handoff and Vault proposal; read-only helper
scripts remain Windows-local under .maixpy/s0-audit-20261008. No push/Vault write.

## Historical PX4 correction before COM19 inspection

User confirms PX4, not ArduPilot, on the actual FC. All earlier ArduPilot/local
ICM pairing recommendations below are superseded for this VIO line. Prioritize
OS04A10 + actual PX4 gyro/accel; M3C onboard ICM is NOT assumed. VIO-P PARTIAL
and its reproducibility gate remain unchanged; S0 PARTIAL for missing deployed
identity, link and timing/rate evidence.

Read-only source audit: clean legacy PX4 checkout dd0ad74fdadc68a62b479129ca3f382c006426f4
provides sensor_gyro/accel and FIFO definitions, candidate MicoAir target drivers,
vehicle_imu integration, HIGHRES_IMU, DDS topic list and TIMESYNC implementation.
It is not the installed firmware. sensor_* carries SI rates/accel and driver
sample times; FIFO can be averaged. HIGHRES_IMU uses sample time but derives
interval averages and may remove estimator bias; do not call it raw acquisition.
Default inspected DDS list does not export raw sensor topics. All actual rates
and synchronization quality remain unmeasured. Camera VIN PTS remains opaque.

M3C reachable; no identified FC USB or bridge endpoint found. Windows serial
inventory shows Bluetooth ports only; host no ttyACM/ttyUSB. UART wiring absence
is NOT proved. No serial reads, sample collection, parameters, firmware, pinmux,
calibration, EKF2 integration or M3C port performed. Updated audit/contract/status
and Vault proposal only; no push. Source hashes are in IMU_SOURCE_AUDIT.md.

One next physical verification step: connect PX4 FC to Windows by USB data cable
without the propulsion battery, to permit read-only board/firmware/IMU identity
inspection. No speculative UART wiring. Earlier completed USB action referred
to M3C and does not establish FC connectivity.

## Subsequent connectivity correction — 2026-10-08

User reconnected M3C; helper and SSH now succeed at 10.18.198.1. Read-only sysfs
shows generic spidev1.0/2.1/2.3 and platform ADC, not an identified IMU. Bundled
imu_ahrs app is code presence only. No acquisition/register probe/configuration
performed. Earlier unreachable status and requested USB action below are
superseded; S0 remains PARTIAL for unverified sensor identity/timing, not network.

## 2026-10-08 VIO-S0 continuation — PARTIAL

VIO-P study closed with the actual repeatability result below; no forced PASS,
threshold change or algorithm fix. S0 now identifies CODE-SUPPORTED local
ICM-42688-P SPI/FIFO tooling in camera source snapshot c7d47b2, previously omitted
from the audit. Existing OS04A10 + local ICM is the preferred conditional pairing,
not a confirmed available device. Separate camera-board IMU remains UNKNOWN;
MicoAir743v2 BMI088/BMI270 and filtered/send-time HIGHRES_IMU are fallback source
evidence, not a verified direct M3C acquisition path. Live M3C check timed out from
Windows; Linux host returned No route to host. No new raw data was collected.

Updated `IMU_SOURCE_AUDIT.md`, `RM27_VIO_DATA_CONTRACT.md`, `VIO_STATUS.json` and
Vault proposal. Corrected obsolete adapter-not-implemented text; schema/code
unchanged. Source review distinguishes sensor PLL time, opaque VIN PTS, software
receipt after FIFO/file write, and FC send-time clock. All mappings/offsets/drift,
mounting transform and real sensor noise remain unqualified. Fresh evidence:
helper connectivity result, source/hardware records and SHA256 values in audit.
Documentation-only validation; no new regression or runtime PASS claimed.
External research unnecessary: this is availability/source audit, not datasheet
qualification. Main workspaces, physical settings and Vault unchanged.

Exactly one next physical action: reconnect M3C's existing USB data link to the
Windows computer to restore `10.18.198.1` for read-only inventory. No SPI rewiring,
FC connection, capture or calibration authorized by this handoff. Audit remains
PARTIAL until current device identity/access evidence can be obtained.

## Native repeatability study — completed 2026-10-08, cause unresolved

Ten frozen native runs plus three separate configuration controls completed;
all 39 native/player/recorder exits are zero, final frozen hash check passed.
See `rm27/perception/vision/docs/NATIVE_REPEATABILITY.md` and external
`repeatability-20261008/analysis-13.json` for per-run and all-pair evidence.
Baseline ATE 5.432--7.653 cm, mean 6.608 cm, sample SD 0.689 cm; 15/45 pairs
exceed 1 cm. First saved states agree; sustained divergence starts at 5.65--10.95 s
of dataset time. OpenCV single-thread/publisher-thread-off controls still vary
(ATE 5.551--6.604 cm). No unique cause proved; async dispatch and a reference
capture lifetime risk need dynamic tracing. The 1 cm gate is not demonstrated
as a native repeatability guarantee, but remains unchanged. VIO-P PARTIAL.
Only experiment evidence and documentation changed. No new regression-suite
claim: prior 226 pass/1 skip plus ROS2 7/7 are historical. Next action is isolated
worker-lifetime/callback-order diagnosis, not hardware. No push or Vault write.

## Historical active-diagnosis checkpoint — superseded above

Native repeatability cohort is RUNNING; do not mark completed. See
`rm27/perception/vision/docs/NATIVE_REPEATABILITY.md` for frozen artifacts,
controller, source audit, interim results and planned separate configuration
control. First two of target ten new native runs: ATE6.3969/6.7431 cm, each
2800 poses. Existing adapter/math/data/1cm gate unchanged; VIO-P stays PARTIAL.
App heartbeat `rm27-vio` will resume checking and final analysis; never start a
duplicate batch. No final population range or causal root-cause claim yet.

## Latest continuation — runtime adapter implemented, acceptance PARTIAL

See `rm27/perception/vision/docs/OPENVINS_ADAPTER.md`. The existing experiment
runner dispatches OpenVINS to a frozen EuRoC VIO loader, isolated container run,
live input/output observer, exact output reconciliation, normalization and SE3
evaluation. Legacy monocular handling and LocalizationEstimate v2 are unchanged.
Unknown tracking/publish evidence still produces incomplete v2 projections.

`adapter-runtime03`: all four processes exit 0; all 2912 image bytes and 29120
raw IMU samples/time/order match ASL. Live 2800 visual +27980 propagated states
match independent MCAP exactly. Adapter and same-run native metrics are exact.
ATE 5.7385 cm; 1s RPE 4.4005 cm /0.476994 deg; path bias -1.50394%, no rescaling.
All 17 final immutable-artifact checks pass; binary/config/data remain unchanged.

**Overall VIO-P remains PARTIAL**: ATE differs from historical native02 by
1.2729 cm, above the predeclared 1 cm limit. Other metric/coverage checks pass.
This is not waived because the new error is smaller. Raw upstream states first
diverge at row188, before adapter conversion. Cause is not yet established.
Failed runs01/02 and run03's rejected gate are preserved; no cherry-picking.
Native-only control03 also exits0 with 2800 poses: ATE 6.2403 cm, 1s RPE
4.4959 cm /0.477945 deg. Native variability exists independently of adapter,
but one control does not establish its full cause or bound. Historical baseline
and acceptance thresholds remain unchanged; no PASS granted from this control.

Fresh tests: 226 passed /1 ROS-only skip; separate sourced ROS2 suite 7/7 pass.
Independent read-only review findings on coverage and propagated-stream checks
were fixed and re-reviewed. Changes remain uncommitted on HEAD `13c7a77`.
No hardware, main-checkout modification, Vault write, commit or push.

Next action is software-only: resolve historical native-run reproducibility /
acceptance discrepancy before granting VIO-P PASS. Do not weaken the fixed
comparison thresholds or start downstream hardware work to bypass this gate.

## Latest continuation — native SE3 evaluated, adapter started

See `rm27/perception/vision/docs/NATIVE_SE3_EVALUATION.md` for exact commands,
policies, hashes and limitations. Existing clean run02: 2780/2800 poses within
reference overlap, SE3 scale fixed 1, ATE RMSE 0.0701139912 m, 1s RPE RMSE
0.0452377799 m / 0.477985477 deg. Path-length ratio 0.980585754 is a diagnostic
only, never a correction. Independent SciPy cross-check PASS. Original GT
orientation caveat retained; bounded reference quaternion normalization explicit.

Native accuracy evaluation integrity is PASS, not flight qualification. Broader
VIO-P remains PARTIAL: adapter projection implemented and exact against 2800
saved raw poses, but no new runtime adapter replay/parity. Unknown state and
publish/completion times stay missing; output is not a complete v2 estimate.
Fresh regression: 208 passed / 1 ROS skip; separate ROS2 tests 7/7 passed.
The earlier lifecycle-only sections below are historical; latest authorization
now permits adapter work after evaluation. No hardware/Sim3/push/Vault changes.

Next software action: EuRoC framework loading and runtime dispatch, then full
native/adapter replay parity. The historical physical-action suggestion below
is **not** a requested action for this no-hardware continuation.

## Latest verified result — lifecycle task

**VIO-P PASS for the current request's native execution/lifecycle gate.**
Designation: **pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**.
The original broader adapter/parity/SE3 roadmap is still incomplete, and those
items were explicitly not started in this task. This is not flight or M3C
qualification. See `rm27/perception/vision/docs/ROS2_LIFECYCLE_FIX.md`.

Gdb proved DDS factory static destruction preceded the global visualizer's
publisher destruction after main returned. Only ROS2 `sys`/`viz` ownership
was moved into main; ROS1 behavior and all three algorithm-library binaries
are unchanged. The original four header substitutions are retained separately.

- No-data regression: before 2/2 abort; after 10/10 clean exit 0, publisher
  thread on/off. Gdb after-fix confirms visualizer destruction precedes factory.
- Full data: `native-ros2-run02-lifecycle`, 2,912 camera-update logs, exactly
  2,800 online states and 2,800 `/poseimu` messages, native/player/recorder exit 0.
- Existing suite: 190 passed, 1 ROS-only skip; ROS2-specific suite: all 7 pass,
  including that skipped case. Evidence is under `lifecycle-debug/`.
- Original failed run01, no-data crash logs, old source/build and gdb traces
  remain intact. No data download/regeneration, main-workspace or Vault change.

Source `upstream/openvins-jazzy-lifecycle`, build `native-ros2-lifecycle`, under
`/home/shiuhou/Projects/rm27-vio-20261007`. Patch is
`tools/openvins/ros2-lifecycle.patch`; smoke test and gdb commands are alongside.
Research changes remain uncommitted; no push/mainline merge performed.

## Latest transport decision — 2026-10-07

The user superseded the ROS1-first baseline choice: prefer official ROS2 data and
the pinned native ROS2 path; host stays ROS2-oriented, future M3C ROS-free where
practical. See `rm27/perception/vision/docs/ROS2_DATASET_REVIEW.md` for source
checks and the observed official Drive HTTP 404. That review predates successful
ASL intake and ROS2 packaging above. No algorithm modification performed. The earlier ROS1 instructions
below describe historical attempts, not a requirement to continue with ROS1.
External evidence has moved to `/home/shiuhou/Projects/rm27-vio-20261007`.
The earlier M3C SSH timeout is historical: a subsequent read-only check connected
and saw an ADC plus generic spidev devices, not a verified IMU stream.

## Repository state

- Project: RM27 OpenVINS public qualification and IMU preparation.
- Repository: `/home/shiuhou/Projects/rm27_slam_vio_openvins` on `10.4.135.84`.
- Branch: `research/vio-openvins-baseline`.
- Commit: `13c7a77bbe0fd256e6765efd4459fbca3426e0b7` (authorized integration merge).
- Working tree: IMU implementation/tests, container recipe, VIO reports and this
  handoff are uncommitted task changes. No push or mainline merge performed.
- Linux primary checkout remains `acc70e3`; Windows primary remains `b8e7299`
  with its original unrelated modified/untracked files preserved.

## Task objective

Current task: fix only the evidenced ROS2 shutdown lifetime defect and qualify
no-data/full-data clean exits on the existing pinned public baseline. Completed
for this scoped gate. Original roadmap's adapter, parity and SE3 evaluation are
still pending; no authorization to start them in this task. VIO-S remains
PARTIAL; VIO-R/B/I/M remain NOT_STARTED.

## Actual changes

- Added `ros2-lifecycle.patch`: one upstream ROS2 entry file, local instead of
  global ownership. Algorithm shared-library hashes unchanged.
- Added real-runtime `lifecycle_smoke.py`, debugger image recipe and gdb command
  files; documented root-cause order and clean full rerun in ROS2_LIFECYCLE_FIX.

- Created sibling worktree; imported Windows commits via bundle and merged with
  host baseline. Resolved only the fixture launcher conflict by retaining both
  platform behaviors. Baseline suite: 163 passed.
- Extended existing `experiments/vio.py` without changing the sample schema or
  LocalizationEstimate: sequence/time/device validation, gap and saturation
  observations, optional static statistics/overlapping Allan deviation and CLI.
- Added 21 synthetic tests and a nullable capture metadata template.
- Added `IMU_SOURCE_AUDIT.md`, `RM27_VIO_DATA_CONTRACT.md`,
  `VIO_BASELINE_REPORT.md`, and `VIO_STATUS.json` under vision docs.
- Added pinned `tools/openvins/Dockerfile` and native-resume instructions.
- Retrieved pinned Docker base and official source archive outside the repo;
  retained selected input snapshots/hashes and failure logs.
- Created `.codex/tasks/2026-10-07-vio-openvins/` lean evidence packet and
  `VAULT_UPDATE.md` proposal. No Vault content was read or modified.

## Verification commands and observed results

From the research worktree:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
git diff --check
```

Current lifecycle validation: **190 passed / 1 skip**, plus **7/7 ROS2-specific**
tests and **10/10** real no-data lifecycle cases. Full playback exits 0 with
2,800 online poses; see latest report for exact commands and hashes.
Historical initial IMU implementation: **184 passed**, diff check clean. Evidence logs
under `/home/shiuhou/rm27-vio-20261007/`. `imu-tests-red.log` records 18 expected
missing-feature failures; `imu-tests-red2.log` records the receipt-clock regression
before its correction. Tests are not physical sensor or real backend evidence.

## Failed attempts

- Preserved header-only full run: shutdown SIGSEGV/139; no-data SIGABRT. Gdb
  isolated global/static teardown ordering and the minimal patch fixes it.
- The following download/connectivity attempts are historical, not blockers:

- Full and shallow Git clones failed (EOF/index-pack); official commit archive
  succeeded. Source remains an extracted archive, not a Git checkout.
- Initial Docker base pull timed out, retry succeeded. Dependency image build
  subsequently had repeated package download failures and was deliberately
  canceled. No native compilation or execution took place.
- Official EuRoC bag download failed through HTTP/HTTPS and direct attempts;
  local Windows download also timed out. No substitute dataset used.
- M3C SSH at `10.18.198.1` timed out; board-side inventory unavailable.

## Decisions and rationale

- Preserve the user's native-first gate: do not implement/claim a runtime wrapper
  before the native program is qualified. Adapter and evaluation remain pending.
- Selected ROS2 Jazzy container isolates research dependencies; Noetic was a
  historical attempt, not an RM27 architecture choice. No host apt/proxy changes.
- Hardware source code is CODE-SUPPORTED, not proof of the installed board.
- Unknown IMU timing/units/extrinsics/noise remain unknown; no flight integration.

## Verified facts

- Upstream revision: `69488123ed9362dd44b6f28e7f4680abbff1442b`; archive and base
  image hashes appear in `VIO_BASELINE_REPORT.md`.
- Pinned native code has a truth-initialization option; do not set estimator
  `path_gt`. It also distinguishes camera/state/IMU propagation times.
- Candidate MicoAir743v2 hwdef supports BMI088 via SPI2 and BMI270 via SPI3.
- Inspected HIGHRES_IMU sender uses `AP_HAL::micros64()` at send time, not a
  verified acquisition timestamp. This does not prove live telemetry availability.

## Assumptions and open questions

Actual IMU identity, firmware, output configuration, wiring/transport, samples,
acquisition clock and synchronization remain unverified. Public data is verified
and native runtime metrics exist; RM27 adapter/parity and SE3 accuracy metrics
do not. The propagated-odometry publication count is not an IMU receipt count.

## Risks

An online saved trajectory must not be labeled optimized. Original V1_01_easy
orientation truth has a documented limitation. No host result may be presented
as M3C VIO throughput. The old camera geometric result does not qualify camera–IMU
timing/extrinsics. Avoid copying simulation parameters to real hardware.

## Next actions

Stop at this task's verified native/lifecycle gate. Adapter, M3C, autopilot,
alternative estimators and unrelated refactors were not started. Any later
adapter/parity/SE3 work must be a separate authorized continuation. Follow
`tools/openvins/README.md`; preserve failed and clean runs.

Exactly one next physical action: photograph the intended IMU/flight-controller
board so its model and revision markings are legible. Generic SPI devices and
historical firmware definitions do not identify the actual raw-IMU source.

## Rollback point

Pre-feature baseline is merge `13c7a77`; both parent commits and main checkouts
remain intact. Review/revert individual task diffs if desired; do not reset or
clean the user's repositories or delete external evidence directories.
