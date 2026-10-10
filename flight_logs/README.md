# Real-flight log intake

Place original real-aircraft evidence under `flight_logs/data/`. That directory
is ignored by Git so large logs and location data are not committed by accident.
Keep this README tracked; generated analysis should go under `artifacts/`.

Historical M600 SITL/Gazebo logs are grouped under
`flight_logs/data/m600_sitl/`. They are simulator evidence, not real-flight
logs, and must not be used as hardware measurements.

Recommended layout for one flight:

```text
flight_logs/data/YYYY-MM-DD_short-description/
├── original.ulg
├── parameters.param
├── notes.md
└── video.mp4                 # optional
```

Do not convert, rename internally or edit the original PX4 `.ulg`. A useful
`notes.md` records aircraft mass, battery, propeller, exact MTF variant,
PX4 version/build hash and board target, flight modes, indoor/outdoor conditions, floor and lighting,
maneuver order, abnormalities and video synchronization cue.

Historical ArduPilot `.BIN` files remain read-only evidence. The existing
DataFlash inventory workflow does not parse PX4 ULog; ULog analysis support
requires separate implementation.

Before analysis, record SHA-256:

```bash
cd /home/shiuhou/Projects/rm27_drones
sha256sum flight_logs/data/YYYY-MM-DD_short-description/original.ulg
```

The first analysis is read-only. It may propose simulation calibration values;
it must not write parameters to the real flight controller. See
[`docs/REAL_HARDWARE.md`](../docs/REAL_HARDWARE.md) for the expected hardware
and fields still awaiting evidence.

The read-only inventory command and simulation profile workflow are documented
in [CALIBRATION_WORKFLOW.md](../docs/CALIBRATION_WORKFLOW.md).
