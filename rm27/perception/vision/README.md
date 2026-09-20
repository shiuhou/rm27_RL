# Vision perception unit

This directory owns the camera-based part of RM27 perception. It is the home
for both localization and recognition; `vision` is not an independent layer
beside `perception`.

- `M3C_OS04A10_SLAM_实验起步包_v1/` defines the M3C + OS04A10 monocular
  camera calibration, localization and SLAM experiment route.
- `maixcam2_dart_vision/` contains the device-side MaixCDK target/green-light
  and red/blue armor recognition application, replay tools, configuration and
  reports. Its output is the versioned `TargetEstimate` consumed by the rest of
  `rm27/perception/`.

The two tracks answer different perception questions and must keep their
evidence separate: camera recognition answers *what target is visible and where
is it in the image*, while SLAM/localization answers *how the camera is moving
and where it is in the scene*. Neither result is automatically metric or
control-safe. Both must pass calibration, timing, frame-convention and
failure-state checks before feeding ExternalNav or flight control.
