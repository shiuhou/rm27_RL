# RM27 integrated development contract

## Accepted objective (2026-09-12)

Develop inside this root project with a unified entry point, while retaining
independently versioned simulator and third-party repositories. First establish
one vehicle's repeatable complete mission under the gates in
[MILESTONES.md](MILESTONES.md), then expand quantity. Existing four parked
proxies remain available; multi-controller scheduling is deferred.
The selected vehicle takes off, contacts the
intended blue armor below the green dart-guidance lamp, stabilizes after the
impact, returns to its assigned slot, lands and disarms before the next vehicle
is released. A failed sortie stops the sequence. The green round light is a
localization marker, not the contact target. Moving-dart interception is a later
research objective, not this gate.

User-provided references: `references/images/base.jpg`, `simtohit.mp4` (29 s), the September 9
RM2027 preliminary rules PDF, `references/cad_raw/base2.STEP` and
`references/cad_raw/dock.STEP`. No inference of successful physical interception is made
from the reference video. Its frames have been sampled for context only.

## Component ownership

Root `rm27/` owns composition, asset audit and the future mission coordinator.
`configs/` owns selected assets and placements. `artifacts/` contains generated,
ignored run outputs. Source CAD remains external to generated models.

`rm27/autopilot/baselines/mujoco_uav_ardupilot/` supplies the verified G1 bridge
and regression baseline. `rm27/simulation/field/rmuc2026_battlefield_mjcf/`
supplies upstream field geometry. `research/astra/` remains
an independent observation-only research package. The current camera, SLAM,
target-recognition and guidance code is under `rm27/perception/`; the old
`maixcam2_dart/` and `visual_guidance/` checkouts are migration sources and are
not runtime entry points. ArduPilot remains an external dependency.

This organization was selected over merging all repositories (unnecessary
history and ownership changes) or continuing separate ad-hoc launch scripts
(no shared scene, asset or mission contract).

## Coordinates and replaceable assets

World retains native field coordinates in metres with +Z up. The existing
bridge interprets world axes as NWU. Dock CAD uses mm and +Y up; map CAD
`(X,Y,Z)` to local dock `(X,-Z,Y)/1000`. Dock origin is bottom center.
Placement is computed from the named platform collision top surface and its
orientation, not from a repeated hard-coded world coordinate. Default red H
platform is an integration choice; it is configurable, not a rules assertion.

Slot centers are at local `(±0.07375, ±0.07375, 0.16)` m. Numbering is
lexicographic X then Y, independent of any team-specific meaning of "front".
Each cavity is 142.5 × 142.5 × 40 mm. Yaw and team platform are configurable.

Every future base adapter must expose `base_frame`, `marker_center`,
`armor_center`, `armor_normal`, and an actual armor collision region. Photograph
colors identify user intent but cannot supply metric coordinates. Target
geometry and contact surface must be audited from STEP before flight.

## Vehicle provenance

User reports 1103 9600KV motors, 2-inch propellers, approximately 180 g mass;
120 × 120 mm temporary outer footprint is authorized. The integration proxy
uses four 50.8 mm propeller disks, centers `(±30, ±30)` mm and a 120 mm guard
rail envelope. It is a replaceable geometric/mass model, not calibrated hardware.
The rail cage does not establish the rulebook's full-enclosure requirement.

Inertia is estimated from a 40 × 30 × 20 mm central box plus four 8 g motor
point masses. Central mass includes all other vehicle mass as an approximation.
Thrust/weight 2.5, 40 ms motor lag and yaw torque coefficient 0.018 are explicit
engineering assumptions. Voltage, battery, actual thrust, inertia and controller
gains have not been measured. No G1 flight success transfers to this new plant.

G1's 1 kg vehicle has ~436 mm propeller width. Existing M4-A propellers span
~146.9 mm, despite its 120 mm *diagonal motor spacing*. Neither fits a 142.5 mm
slot as-is. New vehicle static clearance is 11.25 mm per side; this does not
establish return-to-slot accuracy or propeller-wall aerodynamic behavior.

The current real stack is MicoAir743v2-AIO-35A with user-selected PX4 v1.17.0
(updated 2026-10-10), an MTF-02P optical-flow/range sensor, and an M10G-5883
used only for its compass. The installed PX4 build, board target, ports and
orientations await the real `.ulg` and full parameter export. Existing ArduPilot
SITL evidence does not establish PX4 integration. GNSS is not an intended navigation input. See
[the real-hardware baseline](REAL_HARDWARE.md). These
components are not yet represented by calibrated sensor models in MuJoCo.

The RM2027 preliminary mass-size relation gives a maximum dimension of about
116.3 mm at 180 g. The current 120 mm proxy is therefore a useful simulation
envelope but is not provisionally rule-compliant at that mass. At 120 mm, the
same preliminary formula gives 175.5 g. Actual ready-to-fly mass and maximum
dimension must be measured, and the formal rules supersede this preview.

## Mission interface and acceptance stages

One shared MuJoCo model must ultimately contain all four physical vehicles.
Each ArduPilot instance needs unique ports, state, actuator mapping and origin.
Only the coordinator advances physics once all required motor inputs are
available. Four independent world simulations do not constitute this mission.

Per-sortie states: PARKED → TAKEOFF → HOLD → APPROACH → IMPACT_RUN →
VALID_IMPACT → RECOVER → RETREAT → RETURN → LAND → DISARM → COMPLETE. Any
timeout, wrong surface, unsafe attitude, unrecovered post-impact motion or
communication fault transitions to FAILED and blocks the next launch. Success
requires an actual designated UAV/armor impact, controlled post-impact flight,
then a stable landing in the assigned slot and disarmed telemetry. Proximity,
a geometric touch and teleporting the vehicle back do not count.

The working high-speed envelope is deliberately separate from the current
low-speed integration gate. RMUC 2026 armor documentation gives greater than
10 m/s as the 42 mm projectile detection threshold, while the RM2027 preview
says aerial-robot impact causes projectile damage but does not define its speed
or classification. Until the formal RM2027 rules resolve that mapping, 10 m/s
is a research hypothesis rather than a compliance threshold. Impact-speed
development must progress in measured steps and record the pre-impact velocity
vector and armor-normal component, contact surface, impulse/force, peak
acceleration, attitude excursion, actuator saturation, time to controlled
flight, structural state and return outcome. The 2026 detector threshold uses
the velocity component normal to the struck armor surface.

Acceptance stages and batch criteria are maintained in
[MILESTONES.md](MILESTONES.md): S0 repeatability, S1 contact measurement,
S2 recovery, S3 sensor-only navigation, S4 visual return, S5 complete single
mission reliability, then S6 quantity expansion. S2 may use truth to isolate
physics; S5 must join recovery and sensor/vision guidance in the same system.
Separate demonstrations cannot be combined into a full-mission pass.

## Baselines and status

Frozen G1 commit: `2369d7d207818160e2d140b16118a53f99491b66`, field `dcf1202`.
Current development field: `67c13270e213cf3f69da2619e349eed273432330`.
These are different environments. Restore/pin the old field in a separate
checkout for old-baseline reproduction; do not label a new-field test as the
original G1 run. New geometry invalidates previous corridor assumptions.

Implemented: root scene config, dock CAD conversion and geometric checks,
static field placement, four physical vehicle proxies, scene viewer command,
and a separate visual-only base STEP preview (see [base audit](BASE_AUDIT.md)).
The [single-vehicle flight gate](MICRO_FLIGHT.md) passed twice with real
ArduPilot SITL: 0.6 m takeoff, ten-second hover, 0.12 m offset, return to slot 1,
physical landing and automatic disarm. All four bodies share one world; the
other three retain zero motor input. Precision return uses simulation truth
and an external position loop. This validates the bounded dock maneuver only.

The separate [staging-route experiment](CORRIDOR_REPORT.md) has now completed
two truth-assisted SITL transits to a point 0.6 m outside the provisional armor
and back to the assigned slot. The final contact segment and recovery are not
part of those runs. Static surface clearance does not certify real geometry.

Base collision/semantic integration, a base-contact sortie, four-controller
coordination, camera perception, interception and charging mechanisms remain
unimplemented. The `build-dock` output describes its static scene; flight
evidence is generated separately by `python -m rm27 micro-flight`.

## Vision hardware update

The provisional user-selected onboard vision stack is Sipeed M3C 自研視覺核心板
with OS04A10. The evolving camera/recognition implementation is maintained in
`rm27/perception/vision/maixcam2_dart_vision/`; the old `maixcam2_dart`
checkout is source history only.
See [REAL_HARDWARE.md](REAL_HARDWARE.md#provisional-onboard-vision) for the
OS04D10 reference-baseline distinction and calibration/transport unknowns.
This selection does not change the current simulation-truth flight gate.
