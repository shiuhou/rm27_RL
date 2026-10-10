# Autopilot layer

This namespace is the root project's boundary to ArduPilot and SITL. It
contains adapters and launch contracts, while the ArduPilot source checkout
under `vendor/` and the verified G1 bridge under `baselines/` retain their
independent Git history.

The selected real-aircraft firmware is PX4 v1.17.0. The current runners still
use ArduPilot; the [PX4 migration study](../../docs/PX4_MIGRATION_RESEARCH.md)
records the verified board target, protocol differences and staged migration
plan. No PX4 closed-loop result is established by that study.
