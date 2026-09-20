# RM27 M3C + OS04A10 command log

Run commands from `/home/shiuhou/Projects/rm27_drones`. Replace angle-bracket
values only with measured or recorded values. Do not overwrite source videos or
existing artifact directories.

## Environment and tests

```bash
cd /home/shiuhou/Projects/rm27_drones
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

## Verify the 0921 source

```bash
sha256sum rm27/perception/vision/video/0921.mp4
git rev-parse HEAD
git status --short -- rm27/perception/vision/video/0921.mp4
```

## Re-run VSL-1A encoded-video analysis

```bash
/home/shiuhou/venvs/mujoco/bin/python -m \
  rm27.perception.vision.localization.analyze_video \
  rm27/perception/vision/video/0921.mp4 \
  --out artifacts/vsl-1a-0921-<utc-id> --sample-step 6
```

## Freeze a VSL-2 dataset view from VSL-1 output

```bash
/home/shiuhou/venvs/mujoco/bin/python -m \
  rm27.perception.vision.localization.prepare_vsl2_dataset \
  --vsl1 artifacts/vsl-1a-0921-<utc-id> \
  --output artifacts/vsl-2-0921-canonical-<utc-id>
```

## Record direct M3C/NV21 metadata (VSL-1B)

This reuses the existing VIN path and records sequence, raw PTS, host receive
time, stride, queue, lease, and processing diagnostics. Keep `pts_raw`
unchanged; it is not an exposure timestamp.

```bash
python3 -B rm27/perception/vision/maixcam2_dart_vision/tools/os04a10_highfps/run_device.py \
  --driver-build <verified-build-directory> \
  --official-mode full180 --nv21 --itp-depth 4 --queue-depth 4 \
  --system-media-lib --remote-root /root/os04a10-tests \
  --seconds 60 \
  --business-config rm27/perception/vision/maixcam2_dart_vision/config/green_detector_full180.conf
```

## Prepare a Charuco target definition

The installed OpenCV supports ChArUco. You must measure and fill in board
geometry before capture: square count X/Y, square length, marker length,
dictionary, and a physical board ID. Do not guess these values.

```bash
cp rm27/perception/vision/localization/target_definition.template.json \
  artifacts/calibration-target-<id>.json
${EDITOR:-vi} artifacts/calibration-target-<id>.json
```

## Capture calibration observations from a recorded M3C stream

Keep the current lens and focus mechanically fixed. Use a new session directory.
The tool preserves encoded PTS as encoded-stream timing and writes lossless PNG
observations plus rejected frames and a capture manifest.

```bash
/home/shiuhou/venvs/mujoco/bin/python -m \
  rm27.perception.vision.localization.capture_calibration \
  --source <recorded-1344x760-video> \
  --target artifacts/calibration-target-<id>.json \
  --out artifacts/calibration/<session-id> \
  --step 30 --min-sharpness 50
```

This is the current ready-to-use setup. VSL-2 remains blocked until the target
dimensions, lens/focus notes, and a valid calibration image session exist.
The capture tool refuses placeholder target dimensions and refuses to overwrite
an existing session.

## Required calibration capture coverage

Capture about 30–50 genuinely different board views, covering center, four
corners, all four edges, multiple distances, pitch/yaw/roll, and oblique views.
Reject blur, clipping, glare, partial boards, and weak detections. Preserve the
camera mode at 1344×760 and record crop/resize/stabilization state.

No SLAM backend, VIO, or ExternalNav command belongs in this file yet.
