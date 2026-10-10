# Pinned native OpenVINS preparation

## Offline lifetime correction and diagnostics (2026-10-09)

`ros2-imu-lifetime.patch` is a separately validated minimal correction after the
Jazzy/lifecycle patches: a detached camera worker captures the IMU trigger scalar
by value, not the callback's expired local `message`. Real saved-bag ASan replay
reproduced the fault and the fixed instrumented replay completed cleanly. Apply
only to an isolated pinned-source copy and build a fresh manifest; the historical
source/build/config and acceptance reference remain frozen. The patch does not
by itself fix run-to-run ATE variation. See NATIVE_REPEATABILITY.md for controls,
exact hashes/evidence and retained VIO-P PARTIAL status.

`camera_model_audit.py --root <saved-calibration-directory> --output <new.json>`
performs a fixed-k3=0 diagnostic refit using the original fit/validation split.
It never overwrites original intrinsics or admits calibration to the real pipeline.
`callback_trace.py --logs <diagnostic-native.log> ... --output <new.json>` parses
isolated instrumentation; log-order/trigger differences are not internal IMU
integration bounds or performance measurements.

### Offline reliable-input candidate (not the default live subscription)

Apply `ros2-offline-reliable-imu.patch` only AFTER the lifetime correction to a
separate pinned-source copy. It changes the IMU subscription from best-effort
depth5 to reliable depth2000. A reliable-only reader will NOT match a best-effort
live writer; this is a saved-bag replay candidate, not a hardware transport fix.
The player also needs explicit reliable QoS and a PAUSED publisher discovery
barrier. The external `offline-20261009/run_reliable_ros2.sh` and YAML retain the
native-only experiment; `ros2-offline-reliable-adapter.patch` applies the same
startup control to an isolated copy of the existing adapter launcher, expecting
the retained `offline_replay_qos.yaml` under tools/openvins in that copy.

Original run_native_ros2.sh, run_adapter_ros2.sh, observer, runtime adapter and
native_accuracy.py remain unchanged, including frozen historical checksums.
Use new source/build/run/install manifests. Never apply these patches in place
to historical evidence. See NATIVE_REPEATABILITY.md for observed input coverage,
repeat hashes, ablation limits and whether the uninstrumented/runtime controls
actually completed. Queue capacity is not a sustained-load or real-time guarantee.

Offline raw export (requires only the existing research pyulog environment):

```bash
python -m rm27.perception.vision.localization.experiments.px4_ulog \
  --ulog <original.ulg> --output <new-directory> \
  --instance 1 --gyro-device 3604506 --accel-device 3604506
```

These IDs describe the saved old BMI270 file, not a hardcoded future capture.
The exporter preserves independent streams and does not mark them VIO-eligible.
Timestamp/IMU-window/calibration helpers in experiments.vio_dataset are pure
offline checks, not substitutes for existing admission gates or actual evidence.

## Runtime adapter continuation

The experiment runner now accepts the separate OpenVINS VIO config through
`python3 -m rm27.perception.vision.localization.experiments.runner --config ...`.
See `../../rm27/perception/vision/docs/OPENVINS_ADAPTER.md` for reproduction,
frozen installation/input manifests, online partial-v2 semantics and evidence.
Full live runtime and same-run conversion/SE3 equality are verified, but VIO-P
remains PARTIAL due to historical-run ATE consistency. Do not infer a PASS from
successful process exit alone. Existing output directories are refused.

## Current ROS2 path

See `../../rm27/perception/vision/docs/ROS2_LIFECYCLE_FIX.md` for current
lifecycle root cause, explicit patch, tests and clean full-sequence result.
`ROS2_NATIVE_PROGRESS.md` preserves earlier data/build/shutdown failure evidence.
The ASL input is now available; the historical download blocker below is resolved.

`asl_transport.py INPUT.zip NEW_OUTPUT_DIRECTORY` runs in a sourced Jazzy
environment. It writes only cam0 + imu0, preserving integer header/storage
timestamps and all decoded pixels / IMU values, then reads the entire bag back
against the source. Existing output directories are rejected; failed output
must not be reused or treated as verified without `source_verification.json`.
Synthetic tests are in `tests/test_asl_transport.py`; real-data counts and hashes
are in the progress report. Ground truth is never included in estimator input.

`Dockerfile.ros2` uses the immutable imported official Jazzy parent image.
`build_native_ros2.sh` expects read-only `/upstream` plus writable `/ws`.
`run_native_ros2.sh` expects read-only `/upstream`, `/ws`, `/data`, fresh writable
`/out`, and isolated container networking. It records stock native node output
at half-rate playback, checks subscriber readiness and signals clean shutdown.
It is a feasibility runner, not a proof of complete IMU callback ingestion.

Pristine pinned source failed under Jazzy due to renamed ROS headers.
`jazzy-headers.patch` records the separate include-only compatibility copy;
never call a compatibility-build run pristine-upstream success. No VIO
algorithm edits are permitted merely to change ROS transport.

The header-only native process also had a confirmed global/static teardown
ordering defect. Apply `ros2-lifecycle.patch` to that source variant; it changes
only ROS2 entry-point ownership from global to main-scoped. Preserve the old
source/build/run. The tested separate copy is `upstream/openvins-jazzy-lifecycle`
with `native-ros2-lifecycle` build, under the existing external evidence root.
Use the designation **pinned OpenVINS + explicit ROS2 lifecycle compatibility patch**.
Do not call it unmodified upstream or imply adapter/accuracy qualification.

`lifecycle_smoke.py` sends zero sensor samples, checks subscription readiness,
then tests clean SIGINT shutdown with publisher thread on and off. Run it in
the ordinary isolated research image after sourcing `/ws/install/setup.bash`,
with `--count 5 --out /evidence/NEW_DIRECTORY`. It refuses existing outputs.
`Dockerfile.gdb` adds only diagnostic packages. `debug/*.gdb` capture the
original crashing thread/all-thread stacks and before/after destructor order.

## Historical ROS1 preparation (not the current path)

**Current preference is native ROS2, not ROS1.** See
`../../rm27/perception/vision/docs/ROS2_DATASET_REVIEW.md`. The ROS1 recipe and
commands below are historical fallback documentation, not the selected next
execution path. Do not convert data just to use them. External evidence moved
to `/home/shiuhou/Projects/rm27-vio-20261007`; old absolute paths below are historical.

Research-only container recipe; runtime is unqualified until the native gate runs.

## Frozen inputs

- Upstream: https://github.com/rpng/open_vins
- Commit: `69488123ed9362dd44b6f28e7f4680abbff1442b`
- Official archive API:
  `https://api.github.com/repos/rpng/open_vins/tarball/69488123ed9362dd44b6f28e7f4680abbff1442b`
- Retrieved archive SHA256:
  `283dde4096d9380956f6a9c8ed7e54e2d1303ee64af002d9a97f9b2196e677f4`
- Base: `ros:noetic-ros-base-focal` digest
  `sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d`
- Data: EuRoC `V1_01_easy.bag`, official URL:
  `http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag`

Git clone failed twice; source is an extracted official pinned archive, not a
successful Git checkout. No source patches applied. The archive digest records
retrieved bytes, not a promise that future tarball compression stays identical.

## Resume commands (not evidence of successful build/run)

Run on the Linux host. External evidence directory:
`/home/shiuhou/rm27-vio-20261007`. Use new output directories for retries.
Never mount the main project or flight-controller devices into the container.

```bash
docker build -t rm27-openvins:noetic-20261007 -f tools/openvins/Dockerfile tools/openvins
docker image inspect rm27-openvins:noetic-20261007
```

After successful image build, record its immutable image ID and dependency list
(`dpkg-query -W`). Use that image ID rather than a mutable tag for comparisons.
Create `catkin-native` as the host user before running the following command.
Mount upstream read-only and link it into a new external catkin workspace:

```bash
docker run --rm --network none --user "$(id -u):$(id -g)" \
  -v /home/shiuhou/rm27-vio-20261007/upstream/rpng-open_vins-6948812:/upstream:ro \
  -v /home/shiuhou/rm27-vio-20261007/catkin-native:/ws \
  rm27-openvins:noetic-20261007 bash -c \
  'mkdir -p /ws/src && ln -s /upstream /ws/src/open_vins && cd /ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON -DDISABLE_MATPLOTLIB=ON -j4'
```

Record compiler/OpenCV/Eigen/Ceres versions, CMake cache, build log and hashes of
the native executable/libraries. Build exit code alone is insufficient.

For the native run use an isolated container/ROS master, built workspace,
read-only bag/config/upstream mounts and a new writable output directory. Source
`/ws/devel/setup.bash` inside the container and invoke the stock launch file:

```bash
roslaunch ov_msckf serial.launch config:=euroc_mav \
  max_cameras:=1 use_stereo:=false bag_start:=0 \
  bag:=/data/V1_01_easy.bag dolivetraj:=false \
  dosave:=true path_est:=/out/native_pose.txt \
  dotime:=true path_time:=/out/native_timing.txt
```

Do not set the estimator node's `path_gt`: native code can initialize from it.
Hash the bag and included calibration files; inspect topic counts, header and
bag timestamps, units and usable IMU coverage. Preserve raw outputs and exit /
timeout evidence even on failure. Capture `/usr/bin/time -v` resource output
and name its process-tree scope; Docker client RSS is not estimator RSS.

Native pose export is a saved online stream, not an optimized final trajectory.
Only after native success should the RM27 adapter be implemented and compared
against these exact inputs and configuration.

## Evaluation caveat

Pinned upstream docs warn that original V1_01_easy orientation truth is
inaccurate. Choose and hash the reference explicitly and report that limitation
alongside rotational RPE. Do not silently substitute corrected truth or feed
either truth into estimation.
