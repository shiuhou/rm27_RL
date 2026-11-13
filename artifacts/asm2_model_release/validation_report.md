# ASM2 v1.0.0 validation report

**Release validation date** November 10, 2026 (America/New_York)  
**Scope** Model transfer and deterministic simulation regression  
**Host** Linux x86-64, CPython 3.12.3, RTX 4090, NVIDIA driver 580.178.04

## Fresh validation

| Check | Status | Result |
|---|---|---|
| CAD and compiled rigid body | CONFIRMED | Reconstructed 0.142 kg, zero body-frame COM and full inertia from the source parts and point mass. Tensor reconstruction max error 2.71e-20 kg·m². Actuated MJCF compiled tensor max error 3.67e-14 kg·m². |
| Detailed MuJoCo assets | CONFIRMED | Byte-preserved original MJCF loaded with 14 visuals and 11,874 colliders. The detailed actuator loader compiled with the same geometry and four filtered site actuators. |
| Rotor geometry and dynamics | CONFIRMED | Four CAD centers in FLU, +Z thrust, signed yaw reactions and lever-arm torques reproduced. Rotated-body force and torque probes, hover trim, exact one-step lag and pre-update force application passed. |
| Patch identity and proxy | CONFIRMED | Original mesh bindings, source-part IDs and mesh bytes for `contact_front_000` through `contact_front_013` and `contact_rear_000`. The live corrected USD had 3,522 colliders per aircraft, including the restored body hull `collision_0_570`. |
| Conventional MuJoCo control | CONFIRMED | Hover 19/19, Shift 4/4, moving arcs 2/2. Worst final Hover and Shift errors were 0.050 mm and 0.333 mm. Zero saturated controls. |
| Detailed native contact controls | CONFIRMED | 10/10 exact labels and the archived first-signed sample times. Seven loaded designated contacts, two forbidden airframe first touches, and one contact-free receding near miss. |
| Phase 4R.2 Isaac gate | CONFIRMED | 64/64 exact labels across two 16-world layouts and two assignments per layout. 48 loaded designated, 12 forbidden, four near-miss placements. Every sampled label and signed tick matched the corresponding archived Isaac row. Max native-to-Isaac signed sample time delta 0.5 ms, within the frozen 2 ms criterion. |
| Transfer integrity | CONFIRMED | Archive extracted in an independent directory and every payload SHA-256 verified. Native smoke suite reran using the pinned dependencies in a fresh Python environment. |
| Destination RTX 5060 Ti Isaac runtime | NOT_RUN | Both fresh GPU layouts ran on RTX 4090. The exact Isaac image is an external dependency and its 5060 Ti runtime compatibility remains unverified. |

[Free-flight regression](evidence/release/flight.json) · [Detailed native gate](evidence/release/native-gate.json) · [Isaac layout 0](evidence/release/isaac-layout0.json) · [Isaac layout 1](evidence/release/isaac-layout1.json)

The native regression uses MuJoCo 3.12.0 and NumPy 2.5.1. Before export the selected 1,822 Field shapes, 25 registered shapes, 1,680 unique mesh hulls and native contact materials were checked against the pinned historical MJB. The release carries the verified Field subset and original registration. [Field source and hashes](evidence/field-materials.json)

The source STEP, MJCF and the layered USD entry points retain their source bytes. The exporter selected the referenced OBJ meshes and metadata, produced the free-flight MJCF from the existing builder, and materialized a verified Field snapshot and ten diagnostic resets with their command tapes. The portable runner uses the existing gate, plant and scorer code, with a snapshot loader in place of historical run discovery. [Source inventory and upstream revisions](evidence/source_provenance.json)

## Validated PhysX configuration

- Image `xtrainer-astra:isaac-2.1.0`, ID `sha256:6deeed4a37c67b8a737c8de9e0ecc6c049dbf60ed93b83c24e9dbc2c83a1ebad`. Isaac Sim 4.5.0-rc.36 (Kit build `b735e627`), Isaac Lab runtime package 0.36.21, Python 3.10, NumPy 1.26.4, PyTorch 2.5.1+cu118. The readback JSON contains the resolved scene and body settings.
- GPU broadphase and dynamics, PGS, stabilization enabled, 16 position and one velocity iteration on the aircraft, with scene CCD, body CCD and speculative CCD disabled. Gyroscopic forces enabled, zero damping and zero sleep threshold. Scene gravity is zero, with aircraft gravity applied in the wrench. Physics ticks are 0.1 ms and CTBR holds are 5 ms.
- The composed scene used 0.1 µm contact offset and zero rest offset on every aircraft and Field shape. Detailed contact processing and readback are enabled, with zero aircraft contact report threshold. Aircraft contact material uses 0.6 static and dynamic friction and zero restitution. Normalized aircraft collider points use a 1000× cooking scale with a 0.001 local scale, 255 convex hull vertices maximum and zero minimum thickness.
- The overlay keeps the original `collision_0_570` identity and classification. In the fresh gate it restored the concurrent body and front-patch event for the disputed forbidden control. The matched classification is a signed first-event result. Actual reported pairs can vary by layout.
- The historical near-miss control starts at a 153.44 µm patch-to-registered-surface gap and remained contact-free in the reruns. The largest fresh reset state deviation was 1.97e-7 in state units. The largest first-target or lag readback error was 1.19e-7 N.

## Limits

- The original mass is user-reported measured. COM, inertia and rotor placement are CAD estimates. The rotor cap, 40 ms motor time constant, yaw reaction and controller parameters are inherited provisional settings. The source CAD electronics body uses an approximate surface calculation, and the battery balance translation is already applied in simulation geometry. No bench actuator identification, hardware contact calibration or recovery performance claim is included.
- The Phase 4R.2 gate samples ten controls in one static Field crop. A tiny offset and 0.1 ms physics were material to the result. The restored hull overlaps adjacent reduced hulls. Historical matched measurements reported up to 1.92% first-interval net impulse change on other signed controls. The evidence establishes classification on these samples. Full manifold, force, rebound, friction, high-speed collision, guard clearance and post-contact conformance remain unverified. The known reference guard opening false positive remains.
- PhysX reported CPU collision-detection fallback for some Field hulls during cold cooking. The classification gate still passed. This evidence covers the rigid-body setup only.
- The upstream source inventory includes hashes for unshipped alternate mesh formats and build intermediates. The release manifest is the complete inventory for transfer.

[Archived Phase 4R.2 backend receipt](evidence/phase4r2/backend-receipt.json) · [Original geometry diagnosis](evidence/phase4r2/geometry-distance.json) · [Native reference gate](evidence/phase4r2/native-gate-braking.json)
