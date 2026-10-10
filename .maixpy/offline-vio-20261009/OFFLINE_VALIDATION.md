# RM27 offline validation record — 2026-10-09

Scope: existing saved data and host software only. User disconnected M3C/PX4.
Research repository research/vio-openvins-baseline @4637a5f589801f54ecab146a389db0c81ca1d13f,
Windows main @b8e7299cf090f5150f9f262ebd3b232a824cc526. Existing dirty work preserved.
Local files here are the editing/evidence mirror; actual VIO code lives in
/home/shiuhou/Projects/rm27_slam_vio_openvins on the existing research computer.
No Vault access, commit, push, dependency install, devices, firmware or new capture.

## Final boundary summary

VERIFIED: offline helpers/tests, real saved ULog export, native lifetime defect
and bounded correction, complete instrumented reliable-input replays and full
candidate adapter runtime with ORIGINAL historical metric/coverage gates.
PARTIAL: VIO-S0 (old lossy capture), VIO-P overall (old/default failures retained;
reliable replay candidate isolated, not default/live promotion).
BLOCKED on physical evidence: new-firmware capture completeness and a real
synchronized RM27 Camera+IMU dataset. UNKNOWN: exposure/clock map, rigid extrinsics,
noise, transport/electrical mapping, card-versus-scheduling event-level attribution.
Choice boundaries: production/live transport, adopting camera4-model calibration
and promoting the tested offline delivery profile. No gate is silently replaced.

## Verified offline results

- Existing saved BMI270 ULog, size6896444, SHA256
  d94a142c5a474e7a755bffd534a41cf3255081d87d4868dabf63493a5146f46d unchanged.
  `../imu-offline-20261008/venv/Scripts/python.exe verify_saved_ulog.py` exit0:
  12648 gyro/11933 accel,391 dropouts, original single-sample timestamps retained,
  wrong-device and no-overwrite paths rejected. Not synchronized VIO-eligible.
  Export report SHA25657bb7e21aad1383a8c498f2d45efc370f0a61e3715b9d0ef3ee2ce065f32516a.
- Saved exact PX4 source/23-file comparison and old loss-analysis-v2 reused,
  not redownloaded. New source still rejects full writer buffers and reclaims
  after write/fsync; subscriber/VehicleIMU/writer losses remain distinct.
- `py -3 tools/openvins/camera_model_audit.py --root ../../artifacts/m3c-calibration-20261007
  --output camera-model-audit.json` exit0. Original input hashes retained, no
  changed split/outlier removal;27fit/7validation. Four-model candidate RMS
  0.17095151/0.20187855px, all9 original geometric checks true. Direct k3 deletion
  on saved board rays yields model displacement RMS2.30268/max20.41825px.
  This is model compatibility, not new blind validation or VIO admission.
  Report SHA256a629f16cd23ba92bbb994599b156e7a512be3a8ff0e61a2b7c6fb2868862dab6.
- ASan bad/fixed replay uses saved full EuRoC input and immutable image
  sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14.
  Callback-TU instrumented failure: stack-use-after-return at source474, local
  message line441. Capture-scalar-by-value correction replay completes2800 poses,
  all3 exits0, no ASan finding. Partial instrumentation is not whole-program proof.
  Failing log SHA2564f4274828624ec7051a2dca90b0b3dc7464c61f691111ec89590e685ce0127f0;
  fixed log SHA2562c0c3a1795dc463e689025483034ed1174ac48bff8f2e2650823cf9f77e8ae35.
- `run_fixed_cohort.py --count 3` in sourced ROS2 environment exit0, same frozen
  launcher/evaluator, sequential native controls. ATE0.06787816/0.06228441/0.05982140m.
  All2800/2780 poses; first trajectory differences source frames190/219.
  `analyze_cohort.py fixed` exit0; first meaningful differences9.5/11.0s.
- `verify_saved_adapter.py` exit0: reused existing wire_pose, metric_consistency,
  coverage_consistency and frozen evaluator on all8400 saved raw poses. Every
  same-run projected metric equals native. Historical gate fails run03 at
  ATEdelta0.010292590676210092m >0.01m. Threshold/reference unchanged, VIO-P PARTIAL.

## Tests and environment

Test-first additions cover FIFO last-anchor/fractional dt/order/device/shape,
pairing, bandwidth, affine map/large integer/epoch/validity/uncertainty, IMU
brackets/event semantics, proper extrinsics/noise units/model order and no
silent k3 deletion; synthetic calibration proves holdout does not alter K/D.
Callback-trace parser tests reject ordering/nonfinite/unmatched completions.
All fixture-derived results are TESTS, not physical qualification.

Windows Python3.12.4, NumPy2.5.3, OpenCV4.13.0, pytest9.1.1, existing environments.
Initial complete suite:274 passed/1 skipped, exit0, windows-final-tests.xml.
After strict trace-parser negative tests:282 passed/1 skipped, exit0 in15.75s,
`py -3 -m pytest -q --junitxml=validation/windows-final-v2-tests.xml`.
Parser red/green evidence retained for rejecting malformed/truncated/empty logs
and accepting explicit control metadata. No-overwrite behavior is regression-tested.
`source /opt/ros/jazzy/setup.bash; python3 -m pytest tests/test_asl_transport.py -q`
on research host:7 passed, including synthetic real-ROS2 bag serialization and
byte/time readback. ros2-tests.xml retained; no physical ROS source/device.

Linux SYSTEM suite after additions:273 passed,1 skipped,1 failed; JUnit
linux-system-tests.xml. The same ChArUco API failure existed before changes
(baseline225pass/1skip/1fail): system cv2 lacks aruco.CharucoDetector. No test
weakened and no dependency installed. Windows exercises that test successfully.
Relevant Linux tests94/94, exit0, targeted-tests.xml (earlier helper revision).
Final same seven suites:102/102, exit0, targeted-final-tests.xml.
Final system-v2 suite:281passed/1skip/1fail, same pre-existing ChArUco API gap.
Existing /home/shiuhou/venvs/mujoco (OpenCV5.0.0, no installation) full suite:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
--junitxml=<evidence>/linux-existing-venv-final-tests.xml`:282passed/1skip, exit0.
Final Windows windows-final-v3-tests.xml:282passed/1skip, exit0 in15.23s.
Final ROS2-sourced test_asl_transport.py:7pass/exit0, ros2-final-tests.xml.

Initial OpenCV5 venv run retained281pass/1skip/1fail: new test incorrectly required
cross-call bitwise calibration equality. cv5-refit-diagnostic.json records FOUR
identical-input synthetic fits, K maxdelta1.3653789210366085e-10; all solver input
arrays/parameters identical, including a fifth altered-holdout case. Numerical
implementation cause is not uniquely isolated. Test now checks exact solver
input equality and same-call K/D retention, not an unsupported bitwise solver
guarantee. No implementation/data/fit criterion/geometric threshold change; the
original real camera-model report was not refit or overwritten to pass a test.

## Failed attempts retained

- First mixed Eigen/ASan build bad-free before replay; corrected diagnostic TU
  allocator macro to match uninstrumented Eigen baseline. Not original-app proof.
- Expected red tests before implementation (missing functions/rejection paths).
- Linux system ChArUco environment mismatch, unchanged baseline failure above.
- A remote grep alternation lost PowerShell quoting; command interrupted and
  rerun with separate -e arguments. No evidence inferred from failed grep.
- Source paths inferred for an evaluator were corrected by read-only file listing
  before its execution; actual native02 reference is native-se3-evaluation01.

## Remaining evidence boundary

Old ULog has no per-event SD/scheduler trace or logger subscription counters, so
saved data cannot uniquely identify card versus scheduling/storage-stack cause.
No new-firmware loss-free capture, M3C transport, exposure-clock mapping, rigid
camera/IMU extrinsics, noise calibration or real synchronized RM27 dataset.
These remain PARTIAL/UNKNOWN, not zero-valued metadata or simulated PASS.

## Additional native source-input diagnosis

Two full async trace runs:28983/28987 successful IMU-feed timestamps of29120;
137/133 missing source indices, including105/101 internal omissions and32 initial
omissions each. Camera dispatch2908/2912, identical sequences missing first4 only.
Trace markers parse completely; timestamp matching residual<=160ns (1000ns bound).
Source and installed QoS contract verify best-effort KEEP_LAST5 for IMU; old
readiness lacked a publisher-start barrier.126 camera trigger timestamps differ.
ATE6.24108/5.34366cm, first meaningful trajectory divergence source frame126/6.3s.
No claim that every omission in these instrumented runs also occurred historically.

Two joined-worker controls: all6 exits0, ATE19.87286/8.63275cm, IMU feeds28777/28756
(missing343/364), camera dispatch2912/2908. This is NOT a fix; with the same shallow
best-effort subscription, joining subscription/initializer workers does not ensure
complete delivery. All derived summaries and full logs retained separately.

Reliable-input trace control completed with all6 process exits0. Both runs feed
ALL29120 IMU source timestamps and dispatch ALL2912 camera source timestamps.
No duplicates or reorder; no missing initial/internal records. ATE identical
0.07011501076521641m and raw state SHA256 identical
564d71e12912aab55c2d4a045394347c157bcf2f14c973fcbd38b79349de541e.
69 differing trigger timestamps did not produce trajectory divergence here.
analyze_cohort.py reliable, verify_trace_coverage.py reliable and callback_trace.py
all exit0; detailed metrics/coverage/trace summaries retained locally and externally.
This control combines reliable depth2000, explicit player QoS and paused publisher
discovery, not an isolated test of their individual effects or a universal bound.
Uninstrumented delivery2 controls also completed all6 exits0 with identical
ATE/state hashes to both trace controls. analyze_cohort.py delivery and
verify_saved_adapter.py --variant delivery --count2 exit0; all5600 saved raw pose
projections give exact same-run metrics, both ORIGINAL historical metric/coverage
gates pass (ATEdelta0.0000010195686508396307m). Native ingestion count remains
UNKNOWN for uninstrumented runs. Full unchanged adapter runtime is being checked
separately; do not call these saved projections a live-observer runtime PASS.

## Final full adapter candidate and integrity

From isolated delivery-adapter-repo with sourced ROS2, existing runner invoked
--config ../delivery-adapter-config01.json: exit0, delivery-adapter01/run.json
EXECUTION_COMPLETED / ADAPTER_RUNTIME_AND_METRIC_PASS. Native/player/recorder/
observer exits0.2912images/29120IMU byte/value/time/order equal ASL;2800 visual
poses and27998 propagated states equal independent MCAP. Same-run metrics exact;
original native02 historical metric and coverage gates pass. Original evaluator,
adapter/observer, ground truth and1cm criterion unchanged. No operational tracking,
latency or native ingestion counter is fabricated in this uninstrumented run.

final_offline_audit.py exit0: original baseline + all candidate manifests intact,
all16 candidate process exits0, all FIVE state text SHA256 identical
564d71e12912aab55c2d4a045394347c157bcf2f14c973fcbd38b79349de541e.
Scope is this finite saved-bag cohort. Artifact identity is installation/command
manifest; frozen evaluator's generic old variant label is not the candidate ID.
Original failed baseline/adapter03/lifetime-only/serial experiments remain intact.

No further evidence-backed independent implementation is required before the
remaining physical/interface decisions. Not a claim that arbitrary optimization
or larger repeat cohorts are impossible. Exactly ONE next physical action:
connect ONLY PX4 via USB for PX4_NEXT_CAPTURE_PLAN.md's freshly backed-up,
disarmed/props-off A1/B/A2 actual64/128/64KiB logging comparison. It tests writer
buffer sensitivity and independent subscription gaps; no M3C needed. NOT EXECUTED.

## Final repository / transfer review

Both repositories retain their original HEADs/branches above. Windows and research
`git diff --check` exit0; existing unrelated dirty work remains.29 scoped source,
test, report, handoff and task-packet files are byte-identical between editing
mirror and actual research worktree: validation/source-transfer.json. Original
ULog d94a142c... and camera calibration1ef473ff... freshly hash-checked unchanged.
No source commit/push or Vault update. Root Windows HANDOFF/VAULT_UPDATE and old
active IMU report now point here without replacing historical evidence.

Final transfer-check first attempt used unsupported PowerShell string-pipeline
binding for Get-FileHash; it failed without mutation. Corrected explicit
-LiteralPath array produced the checked29-file result. No success inferred from
the failed invocation. Relevant test XML, positive/negative diagnostics and all
native failed/successful cohorts retained; no material files deleted.
