# Native ROS2 baseline preparation — 2026-10-07

> Historical preparation/failed-run record. The later lifecycle task is
> resolved in `ROS2_LIFECYCLE_FIX.md`: current native/lifecycle gate PASS,
> ten clean no-data exits and a clean full sequence with 2,800 online poses.
> Original failure evidence below remains unchanged. Adapter/SE3 work is pending.

This addendum supersedes the old dataset-download blocker, not the native
qualification gate. VIO-P remains PARTIAL until native execution, adapter parity,
SE3 evaluation and resource reporting all pass. No RM27 or M3C runtime claim.

## Inputs and transport verified

External evidence root: `/home/shiuhou/Projects/rm27-vio-20261007`.
The selected ASL ZIP SHA256 is
`a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa`.
See `ASL_DATASET_INTAKE.md` for source attribution and its limitations.

`asl-intake-20261007/calibration-comparison.json` records 18 successful
checks against the unchanged pinned upstream configuration: camera model,
intrinsics, dimensions, distortion coefficients, camera-to-IMU extrinsics,
IMU noise terms, IMU rate and identity body transform. ASL
`radial-tangential` corresponds to upstream `radtan`. Comparison does not
establish physical camera/IMU synchronization or validate a real RM27 sensor.

`tools/openvins/asl_transport.py` packages cam0 + imu0 into ROS2 SQLite/CDR.
The full dataset has been written and independently read back, comparing
every message with the original ZIP:

- 2,912 camera images: unchanged decoded mono8 pixels and dimensions.
- 29,120 IMU samples: unchanged six floating-point SI values.
- Header and storage timestamps: original integer nanoseconds, no rounding
  or reconstruction of nominal cadence.
- Global chronological order, deterministic IMU-first ties, unchanged
  per-sensor ordering. No cam1 or ground truth in the bag.

Output: `euroc-v101-ros2-verified/`. Verification manifest:
`euroc-v101-ros2-verified/source_verification.json`.
SQLite SHA256:
`3549e000b9f64c9a669c6398d55b599c617be6207d1017aa2af4235f32fbc4a9`.
Metadata SHA256:
`f2118cf6f09da7229acb93ccb05e40d1e87d5ddc6e56cfa0ac3c964ff66435da`.
This is ASL-to-ROS2 transport, not ROS2-to-ROS1 conversion. It proves stored
data equivalence, not subscription delivery or estimator ingestion counts.
Native upstream converts stamps to double seconds internally; this is not
modified or misrepresented as integer-nanosecond internal precision.

## Environment recovery

The host Jazzy Python API imports successfully after sourcing its setup file.
No host packages or global proxy/Docker settings were modified.
Docker's direct registry request failed with `connection reset by peer`.
An explicit request through the existing loopback proxy reached the registry
and returned the expected unauthenticated HTTP 401 challenge.

Ubuntu skopeo was downloaded/extracted under `registry-tool/`, not installed.
The official `docker.io/library/ros` image was copied through that proxy with
TLS/content-digest verification. No image-signature verification is claimed;
the scoped policy accepts unsigned images only from that image repository.

- ROS Jazzy base index digest:
  `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca`.
- Imported local Docker image ID (immutable Dockerfile parent):
  `sha256:e331f1ff8bee6dc648d253015aba97ab7f112a5575b1b4a57fcd705cc8ccde48`.
- `registry-tool/ros-jazzy-inspect.json`, `copy.log`, `policy.json` and
  `ros2-docker-pull.log` retain diagnostics and import provenance.
- `tools/openvins/Dockerfile.ros2` installs research dependencies only inside
  the container; `ros2-docker-build.log` captures the build.
- `tools/openvins/build_native_ros2.sh` mounts stock upstream read-only,
  records source/dependency/compiler/binary hashes and build-resource scope.

## Tests

Seven transport tests first failed for the missing implementation
(`asl-transport-red.log`). All seven passed in the sourced Jazzy environment
(`asl-transport-green.log`), including a real ROS2 bag round-trip of explicitly
synthetic image/IMU input and refusal to overwrite existing output.

Full repository regression command:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q`
gave **190 passed, 1 skipped** (`tests-after-transport.log`). The skipped test
is the ROS2 integration test already passed separately; that test is not an
estimator run. Earlier baseline was 184 passing tests.

## Remaining gate

Build and run pinned native OpenVINS first, establish subscription readiness,
input delivery/processing evidence, drain and clean shutdown. Only then add
the RM27 adapter, confirm transforms and compare identical processing ranges.
Report SE3-only ATE/RPE and scale bias, with original V1_01 orientation-truth
limitations and reference overlap explicitly retained. The native/adapter
runtime, accuracy and resource qualification are not yet established here.

## Native build attempts

ROS2 dependency image successfully built, immutable ID
`sha256:643499e1381c9d799ccfdbf78d57d735be6309733b453b1cfa27ff116df3af14`.
The stock pinned-source build completed `ov_core` and `ov_init`, but failed
`ov_msckf` on the legacy `image_transport/image_transport.h` include under
Jazzy. `native-ros2-build/build.exit` is 2; its logs/source hash manifest and
resource output are retained unchanged.

A separate `upstream/openvins-jazzy-headers` copy has four include substitutions
across two headers: image_transport, cv_bridge and tf2_geometry_msgs `.h` to
`.hpp`. `tools/openvins/jazzy-headers.patch` records the exact difference.
This is a compatibility patch, not an estimator algorithm change, and must
never be labeled pristine upstream success. The initial two-include attempt
exposed the additional tf2 header rename; `build-attempt1.log` preserves it.
The compatibility build workspace is `native-ros2-jazzy-headers/`, forked from
the failed build to reuse unchanged object files. Its `/upstream` mount is
the compatibility copy, read-only; original sources remain unchanged.

The four-include compatibility build **succeeded**, all three packages built;
`native-ros2-jazzy-headers/build.exit` is 0. Executable SHA256:
`34fe1a7b7cdaae406ef1a95e6890571c098380833b4d421bbd724757d412864f`.
Compiler, complete dependency list, caches and library hashes are in that
workspace. This does not supersede the pristine-source build failure.

## Full playback attempt — FAILED shutdown, not qualified

`native-ros2-run01/` contains the first complete public-data playback attempt.
Container networking was disabled, input/source/build mounts read-only,
CPU limit 4 and memory limit 12 GiB. Stock node was launched with cam0 only,
stereo false, state/timing outputs enabled, and no truth parameter or truth
input topic. Upstream online calibration defaults were retained; this did not
redo RM27's physical camera calibration. Input replay rate was 0.5.

Observed evidence (`summary.json`, `native.log`, exit files, raw MCAP):

- Player exited 0 after the full sequence. 2,912 camera-update log entries.
- 2,800 saved online states and 2,800 recorded `/poseimu` messages;
  27,995 recorded propagated `/odomimu` messages. These streams are distinct.
- First saved pose associates with source image 112, about 5.600 s after the
  first image; final saved pose associates with image 2911. Associations are
  unique after subtracting saved camera/IMU offset, maximum residual 5,124 ns,
  within the stated 6,000 ns tolerance for the upstream rounded text output.
- States are finite; state timestamps strictly increase. Pose-emission
  fraction is 96.15%, **not certified online tracking coverage**.
- Upstream internal timing over 2,800 samples: median 7.33 ms, P95 10.75 ms.
  This excludes parts of transport/visualization and is not end-to-end latency.
- GNU time measured the native executable (not Docker client): user 35.40 s,
  system 2.81 s, elapsed 300.38 s, peak RSS 137,160 KiB. Half-rate replay,
  startup and drain are included; no M3C or real-time capacity inference.

After final visualization, publisher/subscriber destruction errors occurred
and the native executable terminated by signal 11; recorded native exit is
**139**. GNU time also prints `Exit status: 0` despite its explicit signal-11
line: that line must not be used to claim success. The recorder initially
ignored SIGINT in its non-interactive background invocation. A targeted
SIGTERM flushed the cache, saved MCAP metadata and returned 0. Its exact
as-executed script is `runner-as-executed.sh`; the repo runner now uses the
observed working SIGTERM recorder shutdown. Full updated runner not rerun.

No-input startup/shutdown also reproduced a `std::system_error: Invalid
argument` / core-dump report (`native-shutdown-noinput.log`). Disabling only
the upstream optional multi-threaded image publisher did not eliminate it
(`native-shutdown-noinput-singlepub.log`). These no-input runs do not qualify
estimation; they demonstrate failure independent of dataset contents. Their
uninitialized huge printed duration is invalid and excluded from measurements.
The precise destruction stack/root cause is still unresolved; no speculative
lifecycle/algorithm patch was applied.

Next software work: obtain a destruction-time backtrace and fix/qualify the
ROS2 compatibility layer or choose a compatible isolated ROS2 environment.
Do not start the RM27 runtime adapter before the native gate is satisfied.
IMU callback count remains unknown, and SE3 accuracy/evaluation, input-delivery
qualification and native/adapter parity remain pending. VIO-P is PARTIAL.
