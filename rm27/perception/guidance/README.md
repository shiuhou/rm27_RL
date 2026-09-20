# Visual guidance interface

This package is the migrated RM27 visual-guidance boundary. Its control path is

```text
TargetEstimate → UAVGuidance → GuidanceCommand → MAVLink adapter
```

`TargetEstimate` follows the MaixCAM2 JSON schema v2 subset. Guidance accepts
only fresh, valid and safe estimates; rejected or missing estimates produce an
explicit zero-motion command. The guidance layer does not read Gazebo truth,
camera frames or motor outputs.

The implementation originated in the M600 `visual_guidance` component. That
checkout is recorded for provenance only; root code imports this package
directly. Camera recognition and SLAM live in the sibling `../vision/` unit,
which publishes the estimate consumed here.
