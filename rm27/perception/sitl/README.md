# Vision-to-SITL adapters

These adapters provide synthetic target estimates and translate a
`GuidanceCommand` into ArduPilot body-frame velocity/yaw-rate MAVLink messages.
The adapter boundary never accepts a `TargetEstimate` directly.
