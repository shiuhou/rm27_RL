# Autopilot layer

This namespace is the root project's boundary to ArduPilot and SITL. It
contains adapters and launch contracts, while the ArduPilot source checkout
under `vendor/` and the verified G1 bridge under `baselines/` retain their
independent Git history.

The selected real-aircraft firmware is PX4 v1.17.0. ArduPilot remains the
explicit default for historical reproduction, while `--autopilot px4` runs the
PX4 v1.17.0 MuJoCo HIL backend. The implementation and evidence status are in
the [PX4 migration status](../../docs/PX4_MIGRATION_STATUS.md); the research
note retains protocol and hardware limitations.
