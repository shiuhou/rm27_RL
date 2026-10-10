# Engineering Handoff

## CURRENT — local integration from published SLAM/VIO repository, 2026-10-11

- Source: `https://github.com/shiuhou/rm27_slam.git`, `main`, full commit
  `3ea1e16086965176db68b88e1c42990f92c5913e`. The source was extracted from
  baseline `96b72c28036aa9ea395f1b45cb7a58e450995abf` and was checked out
  detached in `/tmp/rm27_slam_source_3ea1e16` before review.
- Integration branch: `codex/integrate-slam-3ea1e16`, based on target
  `da396bef63ce51476a71af4b9b70abdda03eed0b`. This is a local review branch;
  it was not committed, pushed, or merged.
- Integrated scope: current localization schema/manifest/analyzer/calibration
  utilities; `localization/experiments` public-backend, online-observation,
  trajectory, evaluation, and OpenVINS/VIO helpers; `tools/` OpenVINS host
  scripts and patches; corresponding focused tests and VSL/VIO reports/templates.
  Added `requirements-slam.txt` for NumPy/OpenCV/pytest.
- Preserved: RM27 flight/control/simulation modules, existing device vision
  submodule, root working-tree changes, raw datasets, credentials, virtual
  environments, generated artifacts, and backend binaries. The source root
  README was not used to replace the RM27 project README.
- Verification: source SHA and detached state verified; focused SLAM/VIO tests
  are the required next check in this worktree. No hardware, calibration,
  controller, flight, Vault write, commit, or push was performed.
- Limits: imported PASS/PARTIAL reports retain their original scope. No real
  M3C/OS04A10 calibration, live VIO stream, PX4 fusion, or flight acceptance is
  implied by this local code integration.

## CURRENT — SLAM → VINS/VIO roadmap index, 2026-10-11

- Baseline: Windows main `fba7457bf3656e4c4a5833611190579821f45fb0`, matching
  remote main before this change; untracked `tests/camera_capture.py` / `tmp/` preserved.
- User objective: make the original SLAM plan and later VINS/VIO plan accessible
  to other Codex instances alongside the handoff, then publish to GitHub.
- Verified source state: original 2026-09-20 VSL-0–8 roadmap already tracked since
  `bd25acf`; the 2026-10-07 v1.0 VIO `new_plan.md` was published in `4dba7fd`.
  Neither full original is replaced or duplicated by this documentation task.
- Actual changes: `rm27/perception/vision/docs/PLAN_HISTORY.md`, README and resume
  links; proposal note in VAULT_UPDATE. Explicit historical→current relationship,
  candidate roles, and planned-gate vs measured-result distinctions.
- Decision/risks: reuse existing plans/evidence and unchanged acceptance criteria.
  VINS-Fusion/sqrtVINS remain candidates, not implied completed implementations;
  Stage A/B transport PASS does not imply G4 fusion or real Stage C qualification.
  Some older plan assumptions and next actions are superseded by current reports.
- Verification: 27 published local plan/entry links checked, both full source plans
  confirmed unchanged, and `git diff --cached --check` exited 0. Normal push is
  checked against remote main after completion; no runtime/physical tests or new PASS.
  No failed experiment, remote device access, deployment or Vault write in this task.
- Next: follow CODEX_RESUME's current software task and safety boundary, not every
  old future-stage operation. Rollback only this doc supplement relative to fba7457;
  do not erase source plans, raw evidence, or unrelated work.

## CURRENT — SSH workspace location supplement, 2026-10-11

### Repository state / objective

Windows main at `4dba7fd79cb2ecad739c1abf22c8a0fc577dc07c` before this supplement;
remote main matched. Unrelated untracked `tests/camera_capture.py` / `tmp/` preserved.
User explicitly requested publishing the SSH-host/M3C/local location inventory.

### Actual changes and verified facts

- Added `rm27/perception/vision/docs/WORKSPACE_LOCATIONS.md` and linked it from the
  resume guide. Source locations were checked against saved handoffs, validation
  reports and experiment scripts, not by new SSH/device access.
- Linux research worktree: `/home/shiuhou/Projects/rm27_slam_vio_openvins/` on the
  historically recorded `shiuhou@10.4.135.84`; separate evidence/build/data root
  `/home/shiuhou/Projects/rm27-vio-20261007/`.
- M3C: historical `root@10.18.198.1`, deployments
  `/root/rm27-mavlink-test-20261010/` and `/root/slam-bench-20261007/`.
- Local Windows `.maixpy` holds editing mirrors and later offline IMU/reassembly
  work. Selected publication is not a complete backup or automatic SSH deployment.

### Verification / decisions / risks

Documentation-only: reviewed changed diff; `git diff --cached --check` exited 0,
and the local-link check verified 16 versioned targets with exit 0. Normal push
is verified against remote main after completion, not pre-claimed. No software/hardware tests
rerun and no new runtime PASS claimed. Preserve both historical and current notes.
No failed experiment in this supplement; remote current state remains UNKNOWN.
Addresses/paths can change, old Linux worktree may still be dirty, and deployed
board decoder can predate current local analysis. No credentials included.

### Next actions / rollback

Other Codex instances start with CODEX_RESUME and WORKSPACE_LOCATIONS; inspect
remote identity and dirty Git state before integrating, never blindly overwrite.
VIO software next task and Stage C boundaries below are unchanged. No Vault access.
Rollback only this documentation supplement relative to `4dba7fd`; do not reset
the repository, remote worktree, saved experiments or device deployments.

## CURRENT — portable PX4 / M3C handoff publication, 2026-10-11

This section supersedes older CURRENT/next-action paragraphs below. User explicitly
requested commit/push and connection documentation for other Codex instances.
No hardware or Vault access is part of this publication task.

- Start: `rm27/perception/vision/docs/CODEX_RESUME.md`, then
  `PX4_M3C_CONNECTION.md`, root `new_plan.md` and `.maixpy/README.md`.
- Git baseline: local main b8e7299 fast-forwarded without conflict to existing
  remote main 7a91a02e575794ba6756012d31eeb66d73182f35 (merged OpenVINS checkpoint).
  Windows history preserved in `.maixpy/HANDOFF_WINDOWS_20261011.md`; remote
  history below retained. No reset, force push, or replacement of upstream work.
- Actual changes: portable wiring/port/source/clock/status summary, curated existing
  research scripts/tests/reports at their original paths, publication index and
  additive README/plan/Vault-proposal pointers. Large raw evidence remains local.
  `.gitignore` keeps generated `.maixpy` content excluded; only selected files
  are explicitly versioned. Unrelated `tests/camera_capture.py` / `tmp` remain out.
- Physical link: M3C UART2 `/dev/ttyS2` ↔ FC UART3/PX4 `/dev/ttyS2`, crossed TX/RX,
  common ground, tested 115200/8N1/no flow. COM19 is independent USB inspection.
  Current audited PX4 v1.17.0 hash d6f12ad1c4f70ad3230afd7d86e971421e02fef4.
- VERIFIED historical Stage A/B: MAVLink2 HEARTBEAT and synthetic ODOMETRY→uORB,
  stop/restart attribution. Preserve, don't rerun. EKF2_EV_CTRL=0 / Disarmed are
  last observed states, not fresh observations from this task.
- PARTIAL VIO-S0: HIGHRES 50Hz transport measured; requested100 capped62.112Hz.
  Integrated measurement semantics, selected BMI088 source completeness, timing
  and camera clock mapping remain unqualified. VIO-P finite reliable-input cohort
  is promising, overall PARTIAL. Stage C remains NOT_STARTED.
- Superseding constraint: user reports no spare UART; no more spare-pad requests.
  USB-host fallback is driver/role/management blocked. Single-UART stock MAVLink
  ULog is a candidate, not a hardware transport or OpenVINS PASS. 921600 is planned,
  NOT applied/approved by this publication.
- Next software work: single-owner live start/ACK/stop/timeout/tail/restore controller
  with offline tests, reusing the finite reassembler and existing decoder. No live
  controller deployment, fusion, calibration, flight or new capture in this task.
- Evidence availability: versioned reports summarize earlier results; original raw
  byte/log/trajectory artifacts and full vendor source builds are not shipped.
  See `.maixpy/README.md` for reproducible fixture commands and local-only limits.

Publication validation is recorded in
`rm27/perception/vision/docs/PUBLICATION_VALIDATION.md`; the resulting commit and
push are identified by Git history/remote state, not a pre-recorded success claim.

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
