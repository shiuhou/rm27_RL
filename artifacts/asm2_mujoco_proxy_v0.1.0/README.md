# ASM2 MuJoCo collision proxy v0.1.0

`models/asm2_isaac_proxy.xml` is an actuated aircraft with the Phase 4R.2 Isaac collision geometry. It contains 3,507 body colliders, 14 front patches, one rear patch, and 14 visual meshes. The 3,507 body colliders include the restored `collision_0_570`. The collision count is 70.3% lower than the 11,874-collider MuJoCo reference.

## Load the model

Use MuJoCo 3.12.0. Keep `models/` and `assets/` together to load this directory independently. For an existing extracted ASM2 Model Release v1.0.0, copy only `asm2_isaac_proxy.xml` into its `models/` directory. The model resolves the proxy OBJ, original restored hull and visual meshes already present in that release. Use a working copy of the extracted release if retaining its original manifest verification.

```bash
python -c 'import mujoco; m = mujoco.MjModel.from_xml_path("models/asm2_isaac_proxy.xml"); print(m.ngeom, m.nu)'
```

The expected output is `3536 4`, including 14 noncolliding visuals. The aircraft has no Field or ground fixture. The body frame and 142 g mass, CAD-derived COM and inertia, four rotor positions and provisional actuator dynamics come from the actuated release model. For hover, initialize `data.act` and `data.ctrl` to `hover_motor_thrust_N` from the v1.0.0 `models/plant.json`. This MJCF uses 0.1 ms physics with MuJoCo Newton and implicitfast, and the existing MuJoCo contact settings. The contact regression updates CTBR every 5 ms.

## Verification

- **CONFIRMED** with OpenUSD 22.11. Compared all 3,522 composed Phase 4R.2 collider names, groups, face indices and body-frame vertices. The maximum coordinate difference was 7.26 nm after applying the USD mesh transforms. [USD geometry audit](usd_geometry.json)
- **CONFIRMED** with MuJoCo 3.12.0. Standalone compile and v1.0.0 drop-in compile passed. The largest compiled mesh vertex deviation was 4.13 nm. Mass, COM, full inertia, rotor positions and one-second hover trim passed.
- **CONFIRMED** on November 11, 2026. Replayed the ten Phase 4R.2 starts in both MuJoCo models with the same static Field, reset state, commands and scorer. All labels matched, including seven qualified, two forbidden and one contact-free near miss. Eight of nine contacts occurred in the same 0.1 ms interval. The inverted body case occurred 0.5 ms earlier in the proxy and remained forbidden. The restored-hull case retained both `collision_0_570` and `contact_front_007` at 126.8 ms. [Native comparison](validation.json)

The evidence covers sampled first-contact classification. Contact force, rebound, recovery dynamics, and runtime speedup remain unmeasured. Keep the 11,874-collider model as the detailed reference for qualification.

**XML SHA-256** `2356bc3f8372fe14002521a82d4a9533867c688f8ed3d13cf172c74e17905ae4`

To reproduce from the source workspace, choose a fresh output directory.

```bash
export PYTHONPATH=src:. PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
py=.agents/runs/rm27-stage1/native-env/bin/python
"$py" -m rm27_asm2.proxy_mjcf export /tmp/asm2-proxy-rebuild
"$py" -m rm27_asm2.proxy_mjcf validate /tmp/asm2-proxy-rebuild
```
