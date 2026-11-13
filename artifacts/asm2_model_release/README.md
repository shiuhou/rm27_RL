# ASM2 Aircraft Model Release v1.0.0

A simulation package for the 142 g RM27 ASM2. Use the detailed MuJoCo plant for contact and recovery studies, and the isolated free-flight plant for control and trajectory optimization. The Isaac proxy has its own sampled first-contact validation.

## Install and verify on the RTX 5060 Ti workstation

These commands target Linux x86-64 with Python 3.12, `uv`, and standard tar and SHA-256 tools. The headless MuJoCo checks run on the CPU. Copy the archive and its `.sha256` sidecar into the same directory and run:

```bash
sha256sum -c asm2-model-v1.0.0.tar.gz.sha256
tar -xzf asm2-model-v1.0.0.tar.gz
cd asm2-model-v1.0.0
python3.12 verify_manifest.py

run_dir=$(mktemp -d ../asm2-smoke.XXXXXXXX)
uv venv --python python3.12 "$run_dir/venv"
uv pip install --python "$run_dir/venv/bin/python" -r requirements.txt
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src:$PWD" OPENBLAS_NUM_THREADS=1
"$run_dir/venv/bin/python" validate_release.py
"$run_dir/venv/bin/python" -m rm27_asm2.flight_native --output "$run_dir/flight"
"$run_dir/venv/bin/python" -m rm27_asm2.release_gate --backend native --output "$run_dir/native-gate.json"
```

Expected results are 15 verified patches, 19/19 Hover, 4/4 Shift, 2/2 moving arcs, and ten matched native contact labels. Test outputs and Python packages live beside the extracted release. The manifest covers every payload file. Its `payload_sha256` is SHA-256 of the canonical sorted JSON file-to-hash map. The detached SHA-256 covers the complete archive including the manifest.

## Model entry points

| Model | Entry point |
|---|---|
| Original detailed MuJoCo reference | `assets/asm2/source/mujoco/model.xml` via `mujoco.MjModel.from_xml_path`. Contains 14 visuals and 11,874 collision geoms. |
| Actuated detailed MuJoCo airframe | `from load_aircraft import load_detailed` then `model = load_detailed()`. Add an environment for custom contacts. For the verified static Field fixture use `rm27_asm2.contact_native.NativeContact(load_profile())` with `load_profile` from `rm27_asm2.flight`. |
| Isolated actuated MuJoCo plant | `models/free_flight.xml`. Conventional position PD and CTBR allocation are in `rm27_asm2.flight_suite` and `rm27_asm2.flight`. |
| Original and detailed Isaac assets | `assets/asm2/source/usd/drone.usdc` and `assets/asm2/candidate/drone.usda`, with default prim `/Drone`. |
| Optimized Isaac proxy | `.agents/runs/asm2-contact/proxy-247f076685d8.usda` (3,521 colliders). Phase 4R.2 entry point is `.agents/runs/asm2-terminal/phase4r2/proxy-with-reference-body.usda` (3,522 colliders). Keep relative paths intact. |

For direct rotor control, write thrust targets in N to `data.ctrl[0:4]` and call `mujoco.mj_step`. `data.act` is filtered thrust in N. Set both vectors to `hover_motor_thrust_N` from `models/plant.json` before stepping from hover. The exact first-order filter uses a provisional 40 ms time constant and applies current thrust before advancing the filter. CTBR is `[T_N, p, q, r]` with body rates in rad/s, held for 5 ms. State layout is world position, body-to-world quaternion WXYZ, world linear velocity, then body angular velocity. The free-flight plant integrates at 1 ms and the actuated contact plant at 0.1 ms.

`models/plant.json` records the full inertia tensor, rotor order and axes, signed yaw coefficient, mixer, thrust limits, COM, CAD transform, and 15 original patch IDs. Body axes are inferred FLU, with +X camera-forward, +Y left and +Z up. Use SI throughout. With adjusted CAD points in mm, `p_body_m = 0.001 * cad_to_body @ (p_cad_mm - cad_com_mm)`. The model meshes already include the battery adjustment. The source STEP predates that adjustment. The Field regression translates the source surface center to the origin without rotating the world axes.

The 0.142 kg total is user-reported measured mass. COM, inertia and rotor centers are CAD estimates. Thrust direction assumes upward thrust, and motor lag, thrust caps, yaw reaction and rate gains are inherited provisional values. The package has no experimentally calibrated motor or contact model. See the source mass metadata for the electronics surface approximation and battery balance assumptions. The source inventory also records checksums for upstream files outside the release selection.

## Optional Isaac regression

The release gate was run with the local `xtrainer-astra:isaac-2.1.0` image on RTX 4090. For the exact runtime, install Docker with NVIDIA Container Toolkit and make that image available locally. The image is an external dependency. The following check confirms the image identity before running both layouts:

```bash
test "$(docker image inspect --format '{{.Id}}' xtrainer-astra:isaac-2.1.0)" = \
  sha256:6deeed4a37c67b8a737c8de9e0ecc6c049dbf60ed93b83c24e9dbc2c83a1ebad &&
bash run_isaac_gate.sh 0 "$run_dir/isaac" &&
bash run_isaac_gate.sh 1 "$run_dir/isaac"
```

Each layout must exit successfully and report `passed: true` across 32 placements. The 5060 Ti Isaac run is **NOT_RUN**. The frozen image uses a CUDA 11.8 PyTorch build. A compatible runtime on the destination GPU is required, and results from a different image need their own gate run. The CPU workflow above is independent of this image.

The PhysX gate applies PGS, 0.1 ms physics, 5 ms control, 0.1 µm contact offset and zero rest offset on both the aircraft and Field. The gate measures sampled first-contact labels on the specified fixture. The proxy overlaps some hulls and can differ from detailed MuJoCo in geometry, contact manifolds, and post-impact response. See [validation report](validation_report.md) for configuration, provenance and limits.
