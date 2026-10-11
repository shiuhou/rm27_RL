# Vision-to-SITL adapters

These adapters provide synthetic target estimates and translate a
`GuidanceCommand` into ArduPilot body-frame velocity/yaw-rate MAVLink messages.
The adapter boundary never accepts a `TargetEstimate` directly.

`MujocoCamera` is the PX4 visual SITL input. It renders an RGB frame from a
vehicle-following MuJoCo camera and attaches the simulator timestamp. The
frame is passed to `ImageTargetDetector`, which segments the provisional blue
armor and green marker colors and emits the existing `TargetEstimate` schema
v2. The detector has no pose, contact, or target-identity input. Camera size,
field of view, colors, and latency are explicit simulation settings; they do
not qualify OS04A10 calibration or a real VIO stream.

Example headless smoke test:

```bash
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m pytest \
  tests/test_px4_visual_sitl.py -q
```

Current PX4 development uses native OpenVINS on simulated camera and body-FLU
IMU input. Recorded evaluation and live control have separate evidence. The
native publisher reports initialization, visual updates, covariance, source
time, monotonic publication time and epoch; uninstrumented ROS poses cannot
qualify the localization contract. The original OpenVINS installation remains
read-only. Instrumentation builds a separate copy with hashes and a saved patch.

Run from the migration worktree with fresh output directories:

```bash
# Export images and IMU from an existing recorded PX4 flight.
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.perception.sitl.dataset artifacts/px4-s2-1ms-dev-01 \
  --duration 110 --output artifacts/px4-images-new

# Build the separate pinned native evidence publisher (requires local Docker image).
/home/shiuhou/venvs/mujoco/bin/python \
  -m rm27.perception.sitl.instrument_openvins \
  --output artifacts/openvins-native-new

# Run the native estimator, mounting sensor data only; no truth input.
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.sitl.openvins \
  artifacts/px4-images-new --stationary-init \
  --installation artifacts/openvins-native-new/installation.json \
  --settings configs/px4_sim_vio_dev.json --output artifacts/px4-vio-new

# Separate scorer can access recorded physics. One initial rigid registration,
# fixed metric scale; this is recorded-input development evidence.
/home/shiuhou/venvs/mujoco/bin/python -m rm27.perception.sitl.evaluate_vio \
  artifacts/px4-vio-new --dataset artifacts/px4-images-new \
  --output artifacts/px4-vio-score-new

# Current-aircraft contact step convergence on recorded motor inputs.
/home/shiuhou/venvs/mujoco/bin/python -m rm27.autopilot.px4.contact_convergence \
  artifacts/px4-s2-1ms-dev-01 --output artifacts/px4-contact-convergence-new

# Airborne response development probe; repeat for each layer and axis.
/home/shiuhou/venvs/mujoco/bin/python -m rm27.autopilot.px4.response \
  --layer rate --axis yaw --profile response --instance 25 \
  --output artifacts/px4-response-new
```

The VIO development settings use conservative filter noise margins; they do not
add measured sensor noise or establish OS04A10 calibration. Native tracking may
be LOST while stationary or when visual updates are insufficient. An execution
pass and a trajectory score do not grant flight qualification. Live development
admission is restricted explicitly to the MuJoCo source clock; real sensor
clocks still require independent qualification.
