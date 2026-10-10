# OpenVINS ROS2 lifecycle compatibility — 2026-10-07

## Result and scope

**PASS for the current request's VIO-P native execution/lifecycle gate.**
Required designation: **pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**.
Not unmodified upstream. The prior four Jazzy header substitutions still apply.

This acceptance covers the public sequence, repeated no-data shutdown, clean
full-data shutdown and existing regression/ROS2 tests. The original broader
plan's RM27 adapter, parity and SE3 accuracy evaluation remain unimplemented;
they were explicitly excluded from this task. No flight, metric-accuracy,
M3C-throughput or physical-IMU qualification is implied.

## Confirmed root cause

The pinned ROS2 subscriber entry point owns `sys` and `viz` as process-global
shared pointers. ROS middleware is initialized later, inside `main()`.
At process exit in this Jazzy/Fast DDS environment, the middleware's static
factory is destroyed before the global visualizer. The visualizer then tries
to destroy ROS publishers/subscribers backed by already-torn-down DDS resources.

Gdb directly observed both destructor breakpoints in this order:

```text
Before: main returns -> DDS DomainParticipantFactory destructor
                    -> global ROS2Visualizer destructor -> failure
After:  main-scope ROS2Visualizer destructor -> main returns
                    -> DDS DomainParticipantFactory destructor -> normal exit
```

Evidence root: `/home/shiuhou/Projects/rm27-vio-20261007/lifecycle-debug`.

- `gdb-before.log`: crashing **thread 1**, SIGABRT following
  `std::system_error: Invalid argument`. Frames 14/20/25/27 include
  `rclcpp::PublisherBase::~PublisherBase`, `ROS2Visualizer::~ROS2Visualizer`,
  global `shared_ptr` destruction and libc `exit`. All-thread full backtraces
  are retained. The earlier full-data attempt exited via SIGSEGV/139 instead;
  these are different manifestations of the same invalid teardown ordering.
- `gdb-order-before.log`, lines 38–55: DDS factory destructor **before**
  visualizer destructor; both reached during libc exit processing.
- `gdb-order-after.log`, lines 16–32: visualizer destruction is reached from
  `main`, **before** DDS factory destruction. The inferior exits normally.
- `gdb-after.log`: destructor stack plus all-thread backtraces after the change.
- `before/summary.json`: real no-data tests fail with SIGABRT (-6), both with
  the background image publisher enabled and disabled. No sensor sample sent.
- `after/summary.json`: same test passes ten times (five per publisher mode),
  all exit 0, with no timeout, class-loader severe warning or destruction error.

The crash snapshots do not show active estimator/update/image-publisher worker
threads. Turning off the optional image-publisher thread did not fix the old
binary. The failing mechanism is confirmed static-resource destruction order,
not an assumed worker join bug, dataset error or estimator-math problem.
No assertion of a proven double-free or a specific heap allocation UAF is made.
The upstream no-data final-visualization duration has an uninitialized sentinel
value; it is excluded from metrics and left unchanged as outside this fix.
ASan/UBSan were not needed: the two destructor-order traces and same-environment
before/after runtime tests directly establish and validate this bounded fix.

## Attribution

| Candidate | Evidence-based conclusion |
|---|---|
| Pinned upstream OpenVINS | ROS2 entry point's global ownership is present in the pinned source. This is the origin of the lifecycle defect. |
| ROS2 integration | Defect manifests at ROS2/Fast DDS static-resource teardown. Scope is the tested Jazzy runtime, not every ROS distribution. |
| Prior header compatibility patch | Only include filenames changed; the ownership defect predates it. The same patched algorithm libraries run cleanly after the entry-point-only fix. |
| RM27 instrumentation/wrapper | Not required to reproduce: gdb launches the native executable with no data, recorder, playback or RM27 adapter. |
| Runtime/build environment | Exposes the invalid order but is unchanged between ordinary before/after tests and full replay. Debug image adds gdb packages without upgrading runtime packages. |

## Exact patch and immutability checks

`tools/openvins/ros2-lifecycle.patch` changes only
`ov_msckf/src/run_subscribe_msckf.cpp`: ROS2 `sys` and `viz` are moved from
process-global storage to local shared pointers in `main()`, declared after
the node/parser/options and before their existing construction statements.
The executor is destroyed before those owners; node/parser outlive them.
ROS1 globals and control flow are retained. No tracking, propagation, update,
initialization math, dataset, timestamp, config or LocalizationEstimate edit.

Patch SHA256:
`e79953f1a116b26d87e8977e9d584ac3832261dd673b0672a1f981915015c21a`.
`git apply --check` against the previous header-compatible source passed.

Original and header-only source/build directories were preserved. New copies:

- Source: `upstream/openvins-jazzy-lifecycle`.
- Build: `native-ros2-lifecycle` (incremental fork of the header-compatible build).
- Clean full replay: `native-ros2-run02-lifecycle`.

All three algorithm libraries are byte-identical before and after:

| Artifact | SHA256 |
|---|---|
| `libov_core_lib.so` | `529d44cb72ebc955fdcc679fe246af743ac7a2a0ca58922e2f4964837e3749e6` |
| `libov_init_lib.so` | `f805b8b3e58f097dc92329405d0b9ccbd6e7d14912ddf7ee96eb5fe218bab896` |
| `libov_msckf_lib.so` | `fbfef81dbbc7a96843d24eadd0ff3503dd8cba42c158d55c3849b75bab91b4f2` |

Entry executable changed from
`34fe1a7b7cdaae406ef1a95e6890571c098380833b4d421bbd724757d412864f`
to `f99c936597079c6957fb3ac5d0c5ee1e642ff173754d60e8f121d0a5b5979b23`.

## Validation

| Check | Before | After |
|---|---|---|
| No-data native shutdown, publisher on/off | 2/2 failed, SIGABRT | 10/10 passed, exit 0 |
| Gdb native shutdown | Main thread abort in late global visualizer destruction | Correct destructor ordering, inferior exits normally |
| Full sequence camera update log count | 2,912 | 2,912 |
| Saved online state / recorded `/poseimu` count | 2,800 / 2,800 | 2,800 / 2,800 |
| Native / player / recorder exit | 139 / 0 / 0 | 0 / 0 / 0 |
| Existing repository suite | 190 pass, 1 ROS-specific skip | 190 pass, same 1 skip |
| ROS2-specific transport suite | 7 pass | 7 pass, including separately running the skipped integration case |

After-run `summary.json` verifies finite states, strictly increasing state time,
unique image associations spanning source indices 112–2911, max association
residual 5,124 ns (within the documented 6,000 ns rounded-text tolerance), and
absence of shutdown-error messages. First saved pose remains at source +5.6 s.

Propagated `/odomimu` output counts differ: 27,995 before versus 27,987 after.
This is a separate asynchronous publication stream, not the visual update count
or a verified IMU ingestion counter. No claim of byte-identical trajectories or
complete IMU callback receipt is inferred from the unchanged pose count.

Post-fix full run retained original configuration and half-rate replay, original
research image, 4 CPU limit and 12 GiB memory limit. Native process resource
scope (not Docker client): user 36.65 s, system 1.78 s, wall 300.37 s, peak RSS
137,816 KiB. Internal timing median/P95 7.24/10.731 ms is not end-to-end latency
or proof of real-time capacity. Outputs remain online, not final-optimized.

## Reproduction and evidence

Ordinary runtime image:
`sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14`.
Debugger image:
`sha256:d1ef27b6df5db0f9e11d3e9963403a9622ea38d198cc460dc65e0a3a2e1bf019`.
`Dockerfile.gdb` and `lifecycle-debug/debug-env-diff.txt` describe additions.

Within an isolated container with read-only `/upstream` source and `/ws` build
and writable evidence mount, source `/ws/install/setup.bash`, then:

```bash
python3 /smoke.py --count 5 --out /evidence/NEW_SMOKE_DIRECTORY
gdb --batch -x /evidence/shutdown-order.gdb --args \
  /ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf --ros-args \
  -p config_path:=/upstream/config/euroc_mav/estimator_config.yaml \
  -p use_stereo:=false -p max_cameras:=1 -p verbosity:=INFO
```

`/smoke.py` is repository `tools/openvins/lifecycle_smoke.py`; gdb command
files are retained in `tools/openvins/debug/`. Run before/after with the
corresponding immutable source/build directory mounts; no dataset is needed.
Full playback uses existing `run_native_ros2.sh`, read-only `/data` bound to
`euroc-v101-ros2-verified`, and a fresh writable `/out` directory. Inputs were
not redownloaded or regenerated. Source/build pins and resource records remain
under their existing external evidence directories.

Repository regression:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q`.
ROS2 test after `source /opt/ros/jazzy/setup.bash`:
`python3 -m unittest discover -s tests -p test_asl_transport.py`.
Logs: `lifecycle-debug/regression.log` and `ros2-tests.log`.

No commit, push, main-workspace mutation or Vault ingestion. No adapter, M3C,
autopilot, alternate VIO engine or unrelated refactor was started. The original
crash logs and failed full run remain intact for comparison.
