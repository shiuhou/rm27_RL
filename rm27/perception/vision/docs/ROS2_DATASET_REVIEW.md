# Native ROS2 dataset/path review — 2026-10-07

## Current decision (supersedes the ROS1-first transport choice)

RM27 host development stays ROS2-oriented; practical future M3C deployment stays
ROS-free. ROS1 was only an isolated baseline candidate, never a project architecture
decision. Prefer provenance-verifiable official data and the pinned native ROS2
entry point. Do not convert just to preserve the old plan; do not modify the VIO
algorithm for dataset transport. No ROS1 migration or data conversion performed.

## Source verified, execution not verified

Pinned source: `69488123ed9362dd44b6f28e7f4680abbff1442b`, now at
`/home/shiuhou/Projects/rm27-vio-20261007/upstream/rpng-open_vins-6948812`.

- `ov_msckf/cmake/ROS2.cmake` builds and installs `run_subscribe_msckf` using
  ament/rclcpp and `ROS2Visualizer.cpp`.
- `ov_msckf/launch/subscribe.launch.py` exposes `config=euroc_mav`,
  `max_cameras`, `use_stereo`, RViz and state-saving options. Select mono with
  `max_cameras=1`, `use_stereo=false` once a validated dataset is available.
- `run_subscribe_msckf.cpp` uses the existing VioManager and a ROS2
  MultiThreadedExecutor. This is a native upstream estimator path, not a new
  algorithm. Build/runtime compatibility with host Jazzy remains untested.
- `ROS2Visualizer.cpp:173` uses SensorDataQoS for IMU; its mono image subscriber
  at lines 213–214 uses a depth-10 default QoS. Match playback offered QoS and
  observe delivered counts/order before claiming no input loss. An existing
  executable/launch file alone does not prove safe consumption of a bag.

Host has Jazzy rosbag2 player/recorder and installed cv_bridge, image_transport,
tf2_geometry_msgs. `dpkg-query` did not report libceres-dev installed. No host
dependencies installed or system proxy settings changed during this review.

## Official provenance and observed failure

Pinned `docs/gs-datasets.dox:33` identifies Vicon Room 1 01 and links ROS2 data to:
https://drive.google.com/file/d/1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh/view

The current upstream GitHub document was checked via `gh api` and still contains
the same ID. This verifies the **upstream recommendation**, not the bytes of the
file or who currently hosts it.

Observed from the Linux host:

- Default network path: Drive page and download endpoint timed out.
- Explicit per-command proxy at the existing loopback port 7897: CONNECT
  succeeded; file page returned **HTTP/2 404**, title `网页未找到`.
- `drive.usercontent.google.com/download?id=1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh&export=download`
  through the same proxy also returned **404**.
- Windows command-line page check timed out.

No authenticated access was attempted; 404 does not establish whether the file
was removed, its sharing changed, or another access issue exists. It is not a
valid dataset download. Error HTML was not renamed as a bag/archive.

Raw response evidence is retained under the external experiment directory's
`ros2-link-review-20261007/`. Raw headers are local diagnostics only, not intended
for Git/Vault ingestion. No trusted dataset bytes were obtained.

## Validation state and next gate

Sequence identity, image contents/message types, IMU values, actual topic names,
header vs storage timestamps and precision, chronological ordering, calibration
matching and reference trajectory: **UNVERIFIED / NO DATA AVAILABLE**. Nominal
topic names from source are not observed bag topics. Original V1_01 orientation
truth has an upstream-documented limitation; identify any corrected reference.

VIO-P remains PARTIAL. Native runtime and adapter remain unqualified. Stop at the
user's explicit exception: the official ROS2 copy cannot currently be retrieved
for validation. Resume with a repaired public upstream link or user-provided copy
traceable to it. First hash/inspect the data; then build/run native ROS2 in an
isolated research environment. No unofficial ROS1 mirror substitution.

If conversion is later demonstrated necessary, keep it separate, record tool
versions and input/output hashes, and require equality of sample counts, image
payloads, IMU values, integer timestamps and ordering. No conversion now.
