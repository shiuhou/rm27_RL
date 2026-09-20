# Field assets

`rmuc2026_battlefield_mjcf/` is the pinned upstream battlefield model used by
the composed root scene. RM27 objects are placed by `rm27/integration/`; do not
edit the upstream model to add docks, drones or target semantics.

The selected revision is recorded in `configs/scene.json` and in the
integration contract. Visual presence in this field model does not establish
collision or target semantics for the current mission.
