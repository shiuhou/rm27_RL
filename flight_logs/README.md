# Real-flight log intake

Place original real-aircraft evidence under `flight_logs/data/`. That directory
is ignored by Git so large logs and location data are not committed by accident.
Keep this README tracked; generated analysis should go under `artifacts/`.

Recommended layout for one flight:

```text
flight_logs/data/YYYY-MM-DD_short-description/
├── original.BIN
├── parameters.param
├── notes.md
└── video.mp4                 # optional
```

Do not convert, rename internally or edit the original `.BIN`. A useful
`notes.md` records aircraft mass, battery, propeller, exact MTF variant,
ArduPilot version, flight modes, indoor/outdoor conditions, floor and lighting,
maneuver order, abnormalities and video synchronization cue.

Before analysis, record SHA-256:

```bash
cd /home/shiuhou/Projects/rm27_drones
sha256sum flight_logs/data/YYYY-MM-DD_short-description/original.BIN
```

The first analysis is read-only. It may propose simulation calibration values;
it must not write parameters to the real flight controller. See
[`docs/REAL_HARDWARE.md`](../docs/REAL_HARDWARE.md) for the expected hardware
and fields still awaiting evidence.
