# RM27 interactive replay and monitoring

From the main project root on the 4090 workstation:

```bash
./run_visual_replay.sh --detach --paused
```

The launcher prints the private workstation IPv4 address (currently `10.4.137.50`). Open NVIDIA **Isaac Sim WebRTC Streaming Client 1.0.6** on the Mac, enter that address in **Server**, and click **Connect**. The Mac must reach the workstation over its private LAN or VPN. This is a desktop streaming client, not a browser URL. It needs TCP 49100 and UDP 47998; SSH TCP forwarding alone does not carry the media stream. Only one client can connect at once.

Official client downloads: [Apple silicon](https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.0.6-macos-arm64.dmg), [Intel Mac](https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.0.6-macos-x64.dmg). Version 1.0.6 is used for the installed Isaac Sim 4.5 backend.

The launcher publishes only the selected private address, with a fixed media port. It refuses public/non-RFC1918 addresses and never changes the host firewall. No router forwarding/public exposure is needed. Use `--bind PRIVATE_IPV4` if the automatically selected LAN/VPN interface is unsuitable. Optional Kit UI extensions and render caches stay under `.agents/runs/rm27-viewer/cache/`. New launches use the [repaired Isaac image](DESIGN.md#34-isaac-runtime-on-the-rtx-4090); existing running viewers keep their original image.

Decoded video, mouse camera control, demonstration/mode selection and playback controls were verified with NVIDIA's Linux 1.0.6 client. The Mac client and your Mac-to-workstation route have not been directly tested. The installed streaming backend crashed during a disconnect/reconnect test; session reuse is not validated. Restart the viewer process before each new client session using the stop and launch commands below. Pause, camera movement and replay within one connected session work without a restart.

## Selecting and controlling a demonstration

The **RM27 | Replay** panel contains three choices:

- **A PPO terminal contact**: selected update 244 checkpoint, a MuJoCo-qualified 41.9 cm moving frontier snapshot. This is terminal-braking capability; the initial inward velocity came from a verified witness prefix.
- **B deterministic witness**: full stationary 1.25 m feasibility maneuver, prominently labeled **NOT PPO**.
- **C PPO stationary failure**: original 1 m nominal stationary-hover attempt by the same selected actor, rejected for flight-volume exit.

**Recorded MuJoCo states** displays the original authoritative 1 ms history without integration. The rendered actor is positioned and rotated from that history; playback does not run a policy. **New Isaac simulation** independently resets the original state and motor activation. For A and C it reloads and runs the actual selected PPO actor through the existing Direct-CTBR decoder, rate controller, motor dynamics and GPU PhysX scene. B replays deterministic CTBR commands in Isaac and remains labeled **NOT PPO**. The panel separately shows the archived MuJoCo reference outcome and the newly observed Isaac outcome. PhysX qualification is a proxy and never overwrites the native result.

Use **Play/Pause**, **Restart** and **0.25x / 0.5x / 1x**. Episodes stop at their original physical task termination; Restart resets the selected demonstration. Camera choices are **Free orbit**, **Follow drone**, **Fixed target** and **Bumper detail**. In free mode, Alt+left drag orbits, middle drag pans and the wheel zooms. Overlays independently toggle body axes, target normal, velocity and contact points. The amber sphere is the exact underside fixture; green faces are the original 25 registered contact colliders.

Eight original aircraft visual meshes and 35 original field visual meshes are exported from the hash-verified frozen MJB into a visual-only USD. The collision scene and source mesh transforms remain unchanged. A target-center translation is applied equally to visual field, aircraft and overlays for display. Source records and camera movement never change physics.

Live Isaac uses the validated local training collision scene (2,069 shapes, including all 25 registered contact colliders), with GPU PGS, 1 µm shape contact offsets and zero rest offset. The distant full-field visual meshes do not expand that local collision scene. Recorded native trajectories remain from the original full-field MuJoCo model; fresh PhysX runs are separately labeled and do not replace its scorer or establish force-level equivalence.

Stop from the workstation:

```bash
./run_visual_replay.sh --stop
```

Then use `./run_visual_replay.sh --detach --paused` for a fresh connection. If a previous streaming crash already removed the container, launch directly. Inspect startup with `docker logs --tail 40 sf-rm27-visual-replay`; wait for `RM27_VIEWER_READY` before connecting. The stop command bounds shutdown at 12 seconds because the installed Kit runtime can stall during shutdown.

For foreground use, omit `--detach`; Ctrl+C also stops the viewer. `--paused` starts without moving. `--demo witness` or `--demo failure` selects the opening demonstration. `./run_visual_replay.sh --self-test` runs bounded rendering/inference verification and exits; it does not train PPO or modify archived results.

## Optional W&B logging

The intended project is entity `uhshi371-hkustgz`, project `rm27-agile-flight-rl`, with separate new group `sf-rtx4090`. Online project access is confirmed through `https://api.wandb.ai`. Credentials remain in the workstation user’s private `~/.netrc`, mounted read-only into future launcher containers. The completed Pilot is published as run `213r6akj`; its stationary-failure replay is run `pluewm3p`, both in group `sf-rtx4090` and labeled no-new-learning.

```bash
python3 -m simpleflight_lab.monitoring probe
```

Logging is disabled by default. For a future separately authorized training/evaluation command, prefix the existing launcher with `SF_WANDB=1` to enable online logging. Set `SF_WANDB_MODE=offline` explicitly when offline capture is wanted. This enables a bounded background process using the installed SDK. The local simulation does not wait for service initialization or log transmission; a wedged worker is terminated after a bounded final flush. No training authorization is implied by enabling monitoring.

Namespaces separate `isaac_proxy`, `mujoco_independent`, PPO, Track/constant controls and deterministic witnesses. Metrics include available actor/critic losses, raw Gaussian entropy, transitions, throughput, proxy training counts, native physical-contact qualifications, normal speed, alignment, rejection reasons, safety exits, mixer clipping and actual-actor Hover/Shift results. Historic CSV fields absent from the record are not fabricated. Videos are explicitly selected, limited to three per run and 50 MB each. Scalar logging is integrated into future train/eval commands. Video publication uses the saved-evidence command below at sparse evaluation milestones; automatic periodic rendering/scheduling is not implemented.

To log saved evidence without simulation or training:

```bash
SF_WANDB_MODE=offline ./run_lab.sh -m simpleflight_lab.monitoring report \
  --output .agents/runs/rm27-viewer/my-new-evidence-run \
  --name rtx4090-saved-evidence \
  --training-csv .agents/runs/rm27-stage1/pgs-braking-pilot/candidate/training.csv \
  --evaluation-json .agents/runs/rm27-stage1/pgs-braking-pilot/frontier-selected.json \
  --video .agents/runs/rm27-stage1/pgs-braking-pilot/frontier-success.mp4
```

Choose a fresh output path. Saved-evidence imports are labeled `no_new_learning`; they do not represent new training. Log deterministic witness records in a separate run with `--controller deterministic_witness`; use matching `--source` for simulator identity.

The configured online service base is `https://api.wandb.ai`; rerun the read-only probe to verify credentials/project access. Explicit saved-evidence publication allows up to 120 seconds for upload; training/evaluation hooks retain their five-second final-flush bound. Do not place a key in a script or this document. `run_lab.sh` forwards explicitly set auth variables by name. Only switch `SF_WANDB_MODE=online` after the probe confirms project access. No historical online run is resumed or overwritten.
