# Unified RM27 migration plan

`rm27/` is the only root runtime package. The new subpackages define the final
product boundaries:

```text
rm27/
├── integration/  scene, dock, base and mission composition
├── simulation/   MuJoCo plant, rendering and replay
├── autopilot/    ArduPilot/SITL adapters and launch contracts
│                 vendor checkout and frozen baselines stay below this boundary
├── perception/   camera, calibration, SLAM, recognition and ExternalNav adapters
└── experiments/  repeatable route, calibration, contact and log tools
```

The first migrated modules are now called through their final paths. No legacy
compatibility modules are retained after a migration is verified.

## Completed first slices

Route analysis and scene/asset integration now live under `experiments/` and
`integration/`. Tests and the root CLI use those final paths directly.

## Migration order

1. Move one root module at a time into its final namespace and update all
   imports, commands and tests before running the root test suite.
2. Migrate only code from the former M600 lab that is used by the current
   scene, SITL bridge or camera contract; record its source commit and verify
   the final `rm27/` path.
3. Review the former PX4/RM27 lab against current frames, vehicle models and
   controller assumptions. Migrate reusable code selectively; keep obsolete
   experiments in the external `../rm27_drones_legacy/` archive.
4. Keep raw CAD under `references/cad_raw/` and logs under `flight_logs/data/`;
   the old lab workspace names must not return as runtime paths.

No G1 result, PX4 result, camera result or old scene revision becomes evidence
for the current vehicle merely because its code was moved.

Source commits and the rule for importing independent history are recorded in
[`MIGRATION_PROVENANCE.md`](MIGRATION_PROVENANCE.md).
