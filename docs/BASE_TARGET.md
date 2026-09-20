# Provisional base target integration

The existing field base is retained. `rm27.integration.base_target` adds a green circular
visual marker, a blue 90 × 55 mm test patch with 6 mm collision thickness,
and named marker/armor sites on the positive-X base. This is a static integration
gate, not a successful flight-contact mission or a physical-base calibration.

The inward/upward patch normal was derived from triangles of
`field__visual__37`: approximately (-0.4617, 0, 0.887). The patch occupies a
bounded portion of the candidate upper plate; its dimensions are not the full
armor dimensions. The marker diameter is 55 mm, following the RMUC 2026 V2.2.0
dart guidance lamp specification. Its 520 nm wavelength and approximate 10 cd
point-source intensity are recorded but not simulated. Marker world position is
still a configurable integration estimate based on `references/images/base.jpg` and the field
model. Rendering exposed an initially occluded marker; the final position makes
its face visible. No claim of exact pose correspondence to the photograph is made.

Configuration: `configs/base_target.json`. World units are metres. Site
`rm27_base_armor_center` is the outward surface center and its local +Z is the
outward normal. `rm27_base_marker_center` is the marker center. The static
`rm27_base_frame` body currently uses world coordinates at identity.
Only `rm27_base_armor` is collision-enabled; the marker has masks 0/0.
Neither surrounding tower collision nor the aerial approach corridor is certified.
The green disk is colored geometry, not calibrated lamp emission or perception.
The RMUC 2026 rules also made base-lamp availability depend on match state, so
guidance must not assume it is continuously illuminated. RM2027 behavior awaits
the formal rules.

The builder checks the field XML hash and compares input field geom names,
world poses and sizes with that source before composing. It preserves vehicle
DOF and actuator counts. Original scene/CAD/upstream files are not rewritten.
Output records the source scene hash and assumptions in `target.json`.

From the repo root:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.integration.base_target artifacts/dock-final/scene.xml --output artifacts/my-base-target
/home/shiuhou/venvs/mujoco/bin/python -m rm27 view artifacts/my-base-target/scene.xml
```

Choose a new output directory for each build. Current inspected scene:
`artifacts/base-target-v4/scene.xml`; close-up: `artifacts/base-target-v4/target.png`.
The viewer is a static scene preview and does not fly the aircraft.

Validation: root tests include a 180 g sphere approaching the patch along its
normal at 0.08 m/s, under zero gravity, dt=0.002 and RK4. It makes named armor
contact and remains on the outward side with finite state. This isolated probe
checks the contact primitive, not UAV contact dynamics, a landing or the whole
field. Root suite: 15 passed. The generated full-field scene also compiles and
was rendered for visual inspection.

## Work possible before real flight logs

The authoritative order and acceptance criteria are in [MILESTONES.md](MILESTONES.md).
Single-vehicle reliability, sensor and visual return gates precede any quantity
expansion; the list below describes available work, not permission to bypass them.

1. Validate surrounding base collision/keep-out geometry and an approach corridor.
2. Implement single-vehicle SITL approach, low-speed contact detection, retreat,
   return and disarm, using explicitly labeled simulation truth initially.
3. Expand impact speed in measured stages, then validate attitude/velocity
   recovery and return after impact. Greater than 10 m/s is currently a research
   hypothesis based on the RMUC 2026 42 mm detection threshold, not a confirmed
   RM2027 aerial-robot requirement.
4. Sequence four independent vehicle/controller states, releasing the next only
   after valid impact, post-impact stabilization, return, landing and disarm;
   abort on any failed sortie.
5. Exercise interrupted communication, missed impact, failed recovery, return
   error and model-parameter variations; add camera/marker localization as a
   separate gate.
6. Use real BIN/parameters and independent flights to calibrate and validate
   dynamics, estimator behavior and sensor models when supplied.

Items 1–5 can advance with provisional parameters. They do not establish real
flight readiness. None of the future contact/sequence gates is marked passing
by this static target implementation.
