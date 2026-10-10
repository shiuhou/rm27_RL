# ASL dataset intake — 2026-10-07

Supersedes the previous NO DATA AVAILABLE blocker. This is dataset intake only,
not a native estimator run or a VIO-P PASS. No format conversion was performed.

User supplied an ETH Vicon Room 1 collection and extracted it under
`/home/shiuhou/Projects/rm27-vio-20261007/vicon_room1(1)/vicon_room1`.
Original collection: `/home/shiuhou/下載/vicon_room1(1).zip`, 6042263426 bytes,
SHA256 `fe73c27be6dc8ac00493b78b750d36b144daf49eea7fdf3163e934527c1b5297`.
Source per user: ETH item `bcaf173e-5dac-484b-bc37-faf97a594f1f`, bitstream
`02ecda9a-298f-498b-970c-b7c44334d880`. We could not retrieve the server checksum;
this attribution is user-reported, not independent server authentication.

Selected sequence: `V1_01_easy/V1_01_easy.zip`, 1149702102 bytes,
SHA256 `a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa`.
This matches the previously observed Hugging Face LFS SHA256, but this experiment
uses the user's collection, not the mirror. The collection also contains the
original `V1_01_easy.bag`; its messages have NOT yet been inspected or compared.

## Fresh read-only verification

- `unzip -tq` passed for the outer collection and selected inner ZIP.
- Both camera CSVs have 2912 rows; all 5824 listed PNGs decode as 752x480 uint8.
  Filenames match integer CSV timestamps; timestamp order is strictly increasing.
- IMU CSV has 29120 rows and six finite values per sample, explicit rad/s and
  m/s^2 headers. Integer timestamps strictly increase; cadence is approximately
  200 Hz. Camera cadence is approximately 20 Hz.
- Reference CSV has 28712 rows, all values finite and timestamps strictly
  increasing. It does not cover the full image interval; use matched overlap
  for evaluation. The original orientation-reference caveat remains applicable.
- cam0/cam1/imu0/reference sensor.yaml files exist and hashes are recorded.
  cam0 intrinsics are 458.654,457.296,367.215,248.375 with radtan distortion;
  IMU config identifies ADIS16448. These are EuRoC parameters, not RM27 parameters.

Detailed timestamps, CSV/calibration hashes and ordered image-payload digests:
external `asl-intake-20261007/intake.json`; diagnostic source: `inspect_asl.py`.

## Remaining work

Compare published calibration conventions to the pinned OpenVINS configuration;
choose a source-preserving ASL replay path into native ROS2, verifying delivered
image/IMU values, counts, integer timestamps and ordering. Do not blindly force
CSV timing to exact 20/200 Hz or use float seconds as canonical timestamps.
No claim yet about actual ROS topics, replay loss, native compilation, estimator
performance or reference-trajectory accuracy. The discovered original ROS1 bag
does not change the user's ROS2-oriented architecture or authorize conversion.
