# RM27 Agile Flight RL — DESIGN

**Version:** 1.0  
**Date:** 2026-11-10  
**Status:** Provisional ASM2 flight and native handover verified. Isaac terminal contact conformance remains open.

---

## 1. Design Overview

本文件描述 RM27 Agile Flight RL 的目標系統架構與主要技術決策。它不代表所有模組已實作。

核心架構：

```text
Perception / VIO / Target Tracking
              |
              v
     Conventional Navigation
              |
              v
    Handover State Validation
              |
              v
       Agile Policy (RL)
              |
              v
     CTBR / Low-level Control
              |
              v
       Motor / Rigid Body
              |
              v
     Full-Underside Contact
              |
              v
     Independent Contact Scorer
              |
              v
       Separate Recovery
```

任務層分工：Navigation 提供可行接管狀態；Agile Policy 負責接管後的自主敏捷機動；Recovery 在接觸事件後獨立處理。

## 2. Main Components and Responsibilities

| Component | Responsibility | Key output |
|---|---|---|
| Target Perception | 綠燈/裝甲目標檢測 | Target observation |
| VIO / Localization | 自身 pose、velocity、uncertainty | Vehicle state estimate |
| Target Tracking | 目標相對狀態與時間更新 | Target-relative estimate |
| Navigation | 接近目標並產生 handover candidate | Candidate handover state |
| Handover Validator | 檢查狀態是否在已驗證集合及安全範圍 | Takeover permission / reason |
| Agile Actor | Closed-loop agile control | CTBR action |
| Low-level Rate Controller | 將 body-rate/thrust command 轉為 actuator commands | Motor allocation |
| Actuator Model | Motor lag、thrust、limits | Rotor forces/torques |
| Physics Model | Rigid-body and contact dynamics | Physical state |
| Contact Scorer | 獨立判定接觸是否合格 | Qualified / rejection reason |
| Recovery Controller/Policy | 接觸後穩定、估計器恢復 | Recovery state |

模組應透過明確資料契約連接，避免 Actor 直接依賴 simulator-specific internals。

## 3. Airframe and Simulation Asset Design

### 3.1 Geometry layers

新機型至少維持三種幾何表示：

1. **Visual Mesh**：Viewer / rendering。
2. **Physics Collision Proxy**：訓練時可計算的碰撞表示。
3. **High-fidelity Evaluation Geometry**：獨立接觸驗證。

Collision Proxy 可由合理的 primitives 或 convex decomposition 構成，但必須對照高精度幾何測試 first-contact event、contact location、normal、impulse 及 multiple-contact pattern。

不得在 CAD 尚未完成前凍結底殼 proxy。

### 3.2 Frozen Plant

Frozen Plant v0 將機體與控制鏈分開描述：

- Rigid body: mass, COM, inertia tensor。
- Rotor layout: position, axis, thrust/torque model。
- Actuator: motor/ESC lag, saturation, voltage dependence（若量測可得）。
- Control: CTBR decoder, rate controller, mixer/allocation。
- Physics: gravity, integration step, collision and contact parameters。

每個參數應帶來源標記：`MEASURED`、`CAD_ESTIMATE`、`MANUFACTURER`、`IDENTIFIED`、`PROVISIONAL`。

### 3.3 ASM2 provisional free-flight baseline

`configs/rm27_asm2_flight.json` pins source parameters: user-reported 0.142 kg, COM at the origin, full CAD inertia tensor, and four CAD motor centers. The historical Ver2 rate loop and motor assumptions remain provisional. Physical CTBR is `[T_N,p,q,r]` in FLU. The allocator uses the full inertia tensor and gyroscopic term, plus each rotor's force lever arm and alternating yaw reaction. It holds targets for 5 ms, applies current thrust at each 1 ms tick, then advances the exact 40 ms thrust filter. No ground effect, drag, battery dynamics or calibrated thrust curve is modeled.

`IsaacFlight` advances batched, collider-free GPU rigid bodies. `NativeFlight` independently integrates site actuators and `filterexact` in MuJoCo. See [Isaac Sim local-frame wrench conventions](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.core.prims/docs/index.html) and [MuJoCo actuator integration](https://mujoco.readthedocs.io/en/latest/computation.html#actuation-model). The default controller uses position/velocity feedback, acceleration feedforward and attitude error to produce CTBR. Frozen Ver2 Track inference is an optional transfer comparison with the original 45 inputs and hover-centered collective scaling for ASM2. Neither path writes checkpoints.

The benchmark reuses the 19 historical Hover starts and four 0.20 m Shift cases with their 4 s horizon and settled criteria sampled every 5 ms. It adds two analytic moving arcs with declared error thresholds. This is isolated flight evidence; the detailed contact candidate and [its limitations](ASM2_COLLISION_INTEGRATION.md) remain separate.

Run from the project root with unused output prefixes:

```bash
./run_asm2_flight.sh --output .agents/runs/asm2-flight/my-isaac
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 .agents/runs/rm27-stage1/native-env/bin/python \
  -m rm27_asm2.flight_native --output .agents/runs/asm2-flight/my-native
./run_asm2_flight_viewer.sh .agents/runs/asm2-flight/isaac-pd.npz \
  --episode moving_left_arc --self-test
```

For interactive private-LAN playback, replace `--self-test` with `--detach --paused` and use the Isaac Sim WebRTC client. If the legacy viewer is using TCP 49100 / UDP 47998 on that address, stop it with `./run_visual_replay.sh --stop` before switching. `./run_asm2_flight_viewer.sh --stop` stops only ASM2 playback. The viewer uses actual CAD visuals and recorded Isaac poses, without running a policy or physics during playback.

### 3.4 Isaac runtime on the RTX 4090

The root launchers share `docker/isaac-runtime/launch.sh` and use the
`isaac_image` field in `configs/rm27_asm2_collision.json`. The current image is
`rm27-sim:isaac2.1-fixed`, derived from the unchanged, digest-pinned
`xtrainer-astra:isaac-2.1.0` image. Build it from the project root:

```bash
sudo -n docker build --network none -t rm27-sim:isaac2.1-fixed docker/isaac-runtime
./run_lab.sh -u /sf/tests/isaac_runtime_check.py \
  --output /sf/.agents/runs/isaac-runtime-check/result.json
```

Choose an unused output file. The build downloads checksum-pinned official
packages, upgrades both Kit Warp extensions to 1.8.1 and replaces Kit's malformed
torchvision wheel with the official `0.20.1+cu118` wheel. Kit continues to select
PyTorch `2.5.1+cu118`; the separate cu124 pip installation is not substituted.
Warp's [official 1.8.1 release](https://github.com/NVIDIA/warp/releases/tag/v1.8.1)
includes the `cuDeviceGetUuid` driver-entry-point fix. The image's original
Isaac Lab/Sim code, PhysX settings, aircraft assets and scorer are retained.

Launchers use `sudo -n docker` for the default local socket when the calling
account cannot write it, without changing socket permissions. Containers retain
the caller's UID/GID and receive a writable temporary home with existing cache
mounts. Existing containers continue using their original images.

**CONFIRMED, 2026-11-12, RTX 4090 / driver 580.178.04:** after actual Kit startup,
the Isaac Lab Warp CUDA raycast kernel passed hit/miss and Torch-buffer checks;
PNG read/write, CUDA JPEG decode and CUDA NMS passed. The targeted Warp UUID and
torchvision ELF startup errors were absent. ASM2 deterministic flight passed
19/19 Hover, 4/4 Shift and 2/2 moving cases. The unchanged v1.0.0 release
first-contact gate passed 32/32 placements in each of two layouts. These remain
sampled simulation checks, and do not authorize new learning or establish
post-impact conformance. The sealed release and Goal Hybrid copy retain their
original runtime references.

## 4. Control Interface

### 4.1 Initial candidate: Direct-CTBR

第一個新機型候選保留：

\[
a=[T_c,\ p,\ q,\ r]
\]

其中 \(T_c\) 為 collective thrust command，\(p,q,r\) 為 body-rate commands。具體 normalization、limits、units 和 mapping 必須依新機體與 PX4 rate-loop 實測決定。

優點：
- 延續 SimpleFlight / RM27 Direct-CTBR 經驗。
- 比 direct motor RPM 更容易隔離低階控制與 policy learning。
- 與既有 rate-controller deployment path 較容易對接。

限制：
- Policy 的行為依賴 rate controller、mixer 和 motor dynamics。
- 大角度機動時仍需驗證 rate limits、thrust authority 和 saturation。
- 新機型不能直接沿用舊 150 g 模型的參數。

### 4.2 Alternatives

保留以下候選作有問題導向的比較，不在第一階段同時實作所有接口：

- Attitude + collective thrust。
- CTBT (collective thrust + body torque)。
- Direct motor RPM / rotor thrust。
- Residual action over a conventional controller。

只有當 CTBR 出現可證明的限制，才啟動相應 action-space ablation。

## 5. Policy Observation Design

初步設計以 target-relative state 為核心：

- Relative position in a documented frame。
- Relative linear velocity。
- Target surface frame / normal。
- Aircraft orientation representation。
- Body angular rate。
- Previous action。
- Short state/action history where needed for partial observability and actuator lag。
- Validity, confidence and observation age when using estimated/vision-based inputs。

新底殼接觸任務可能需要：
- Contact-patch relative pose。
- Projected contact-region offset。
- Local surface normal information。

這些 feature 要在 CAD 幾何可用後再確定，避免用未定義的 point-contact proxy。

Actor 不得取得未來完整 reference trajectory 或 scorer outcome。Critic 可評估 privileged training-only state（例如真實 dynamics parameters），但必須明確記錄並確保不洩漏到部署 Actor。

Temporal actor / TCN / recurrent policy 是候選設計，不是已凍結要求。先用最小充分 observation baseline，再由可觀測性或延遲測試決定是否增加 history。

## 6. Handover Architecture

### 6.1 Handover set

定義：

\[
\mathcal H =
\{x:\ p_{rel}\in P_H,\ v_{rel}\in V_H,\ R\in\mathcal R_H,\
\omega\in\Omega_H,\ x\in\mathcal S_{safe}\}
\]

具體範圍不得預先猜測。

建立方式：
1. 由新機型 feasibility analysis 找到可行機動狀態。
2. 量測/模擬 Conventional Navigation 能提供的狀態分布。
3. 將定位誤差、延遲、速度和姿態變化納入。
4. 交叉檢查 handover states 是否在可行集合內。
5. 凍結 training / evaluation distributions，避免為提高成功率事後縮小範圍。

### 6.2 Runtime handover

Handover Validator 至少檢查：
- State estimate validity / age。
- Relative pose and velocity validity。
- Safety-volume membership。
- Handover-set membership。
- Actuator/controller health。
- Target estimate validity。

失敗時保持原導航/安全控制，不應把無效狀態交給 Agile Actor。

### 6.3 ASM2 Phase 3 feasibility

The Phase 1B position controller flies frozen navigation references on the detailed native plant. At the fixed 2.4 s gate the planner reads live pose, velocity, body rates, filtered rotor thrust and held targets. The approach continues on the same plant. A polynomial matches the live position, velocity, acceleration and jerk, then targets the Phase 2 patch plane with a low normal speed and aligned thrust. The planner ranks a bounded grid of durations and depths using contact-free previews. The detailed scene and Phase 2 scorer determine qualification. The position-derivative mapping follows the quadrotor flatness formulation. [Trajectory generation and control reference](https://faculty.washington.edu/minster/bio_inspired_robotics_2020/papers/mellinger_michael_kumar_Trajectory_Generation_Control_aggressive_quadrotors_Jroboticsresearch2012.pdf)

The sampled set measures navigation using ground-truth simulator state. The feasible observations describe a finite bank of actual states and cover the tested static crop. Navigation estimation and latency remain open. The gate retains rejected navigation attempts and body-first outcomes in their original split denominators.

Use a fresh run directory to reproduce the suite and reset corpus.

```bash
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1
.agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.handover \
  --output .agents/runs/asm2-handover/repeat
.agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.handover_curriculum \
  --run-dir .agents/runs/asm2-handover/repeat --verify-native
./run_asm2_contact_isaac.sh --handover \
  .agents/runs/asm2-handover/repeat/dev_center.npz \
  .agents/runs/asm2-handover/repeat/held_diagonal.npz \
  --output .agents/runs/asm2-handover/repeat/isaac-matched.json
./run_asm2_contact_isaac.sh --handover \
  .agents/runs/asm2-handover/repeat/dev_center.npz \
  .agents/runs/asm2-handover/repeat/held_diagonal.npz \
  --terminal-lookback-ms 50 --output .agents/runs/asm2-handover/repeat/isaac-terminal-reset.json
./run_asm2_flight_viewer.sh .agents/runs/asm2-handover/repeat/dev_center.npz \
  --self-test-output .agents/runs/asm2-handover/repeat/viewer/contact.png
```

For interactive recorded-state playback, launch the viewer with `--paused` in place of the self-test arguments and connect to its printed private IP from the Isaac Sim WebRTC client. Choose an address with the viewer's ports free if the legacy stream is still active.

## 7. Witness-Based Curriculum

### 7.1 Role of deterministic witness

Deterministic planner / witness 的用途：
- Feasibility witness。
- Benchmark against RL。
- Generate reset-state candidates。
- Estimate feasible state tube / terminal set。

主 policy 不使用 witness action labels，不做必要的 behavior cloning，也不追蹤預先指定的完整 trajectory。

### 7.2 Curriculum concept

暫定順序：

1. Terminal contact neighbourhood。
2. Terminal braking region。
3. Rotation / translation states。
4. Expanded witness-derived state tube。
5. Validated navigation handover distribution。
6. Broader handover boundary / uncertainty。
7. Stationary-hover extension and OOD stress tests。

每個階段要有重疊的 reset distribution 和 held-out evaluation。只有在 backward reachability 被實際計算、驗證後，才使用該術語；否則稱為 witness-based reverse curriculum。

### 7.3 Phase 4 terminal inputs

Use the `reset-dataset/manifest.json` from the Phase 3 run to verify sources, array hashes and split membership. Begin with the `terminal` rows in `development.npz`. Restore the 13 state values, four filtered motor thrusts and four held motor targets together. The state order is COM xyz, body-to-world wxyz, world linear velocity and FLU body rates. Positions use world axes translated to the target center. Store control actions separately from the reset corpus.

For evaluation, `heldout.npz` holds qualified phase snapshots and `heldout-handover.npz` holds the full four-state reached handover benchmark, including the body-first case. The source run retains all failed navigation cases and their recipes. The witness outcome flag belongs to evaluation metadata.

The runtime inputs are `configs/rm27_asm2_flight.json`, `assets/asm2/proxy/manifest.json`, `configs/rm27_asm2_contact.json`, and the frozen 2027 Field registration pinned by the manifest. CTBR has a 5 ms update interval. Native qualification uses 0.1 ms physics and the Isaac comparison uses 1 ms physics. The existing normalized Ver2 output is `[a_p, a_q, a_r, a_T]`, with rates `2*[a_p, a_q, a_r]` rad/s and collective `(1 + 0.5*a_T)*m*g`. Its range contains every measured witness command from the sampled terminal regions. This is an action-range check for the terminal subset.

Phase 4 needs an Isaac reset and contact-event regression before a learner starts. Both 50 ms terminal replays missed within the recorded windows at 1 ms in Isaac and MuJoCo, while the source trajectories qualified at 0.1 ms. Use a fixed set of positive and forbidden-contact controls, repeat the reset and signed-event checks across world layouts, and include a constant-command comparison. Keep native MuJoCo as the independent qualification reference. The Phase 3 manifests record deterministic evidence and carry `training_allowed=false`. [Terminal replay evidence](../plans/plan_new_airframe_phase0_01.md#5-phase-3-deterministic-feasibility-and-handover)

### 7.4 Resumable fixed and adaptive comparison

The comparison continues PPO from the verified Phase 4 checkpoint `checkpoint-186368.pt`. Both arms use the same seed, learner settings, observations, actions, reward and episode rules. Isaac uses the verified Phase 4R.2 overlay with 16 worlds, 0.1 ms physics and a 5 ms control interval. The fixed sampler preserves the existing stage mixtures and promotion gates. [Comparison configuration](../../configs/rm27_asm2_resumable.json)

The adaptive sampler estimates each reset's success fraction `p` from three complete autonomous rollouts in native MuJoCo. It tests the nominal state and paired shifts of ±0.5 mm position and ±0.005 m/s velocity along the first target tangent. At initialization and every 49,152 transitions it refreshes the probabilities using the latest saved actor. It assigns 55% of resets in proportion to `0.05 + 4*p*(1-p)`, 10% uniformly across the bank, 10% uniformly to terminal states and 25% uniformly to braking states. Earlier states, including the five development handovers, can receive more weight when their outcomes place them near the success boundary. PPO samples the original bank rows with their filtered rotor thrust and held motor targets. The perturbations are for estimating reset difficulty.

The launcher runs A then B. Both arms have a one-update restart check charged to the campaign. Each arm has at most 300,000 transitions and 4,500 learning seconds. The shared limits are 600,000 transitions and 9,000 learning seconds. The ledger reserves a complete update before collection and conservatively charges an interrupted reservation. A checkpoint at a complete update stores the learner, both optimizers, normalization, schedule, RNG and sampler state, configuration and evaluation history. The simulator snapshot is diagnostic because its PhysX solver and contact caches cannot be restored exactly. On restart, the learner discards active episodes and collects a new PPO batch from valid resets with the restored actor.

Run these commands on the host using the project's native MuJoCo environment. After interruption, repeat the `run` command with the same configuration and output directory once the previous launcher has exited. It reloads the arm's `resume.pt` and continues against the existing ledger. When the host Python has the W&B SDK and configured project access, set `SF_WANDB=1` before `run` to enable the existing monitor. Local JSONL records are always available.

```bash
cd /home/dsa/project/ceo_pj/RM27_Agile_Flight_RL_Project
export PYTHONPATH=src:. OPENBLAS_NUM_THREADS=1
rm27_python=.agents/runs/rm27-stage1/native-env/bin/python
rm27_config=configs/rm27_asm2_resumable.json
rm27_run=.agents/runs/asm2-terminal/resumable-comparison

"$rm27_python" -m rm27_asm2.terminal_workflow --workflow-config "$rm27_config" freeze
"$rm27_python" -m rm27_asm2.terminal_workflow --workflow-config "$rm27_config" run
```

The host launcher services each development request automatically. For a saved request, run the same evaluator directly after the launcher has exited.

```bash
"$rm27_python" -m rm27_asm2.terminal_workflow --workflow-config "$rm27_config" evaluate \
  --request "$rm27_run/A/evaluation-request.json"
```

Use only development results for sampling and checkpoint selection. The native first-touch evaluator compares the selected checkpoints on the frozen final perturbation suite and the historical regression suites of 105 phase starts, four reached handovers and 18 braking starts. The final suite contains 18 rotation starts, 18 braking starts and four perturbed navigation-reached handovers. It uses new perturbations of the historically examined `held_center_mid`, `held_diagonal`, `held_fast_offset` and `held_outer_slow` source trajectories. Treat the four perturbed handovers separately from the original four reached states. Simulator outcomes in the training log describe Isaac feedback. Native evaluation reports contain the complete first-touch outcomes.

After both arms complete, run the final evaluator.

```bash
"$rm27_python" -m rm27_asm2.terminal_workflow --workflow-config "$rm27_config" final
```

The final evaluator writes `native-comparison.json`, per-suite episode results and representative native trajectories in `final-replays/`. To play an available rotation outcome from arm A, use the existing viewer.

```bash
rm27_replay=$(rg --files "$rm27_run/final-replays" -g 'A-rotation-*.npz' | sort | head -n 1)
./run_asm2_flight_viewer.sh "$rm27_replay" --paused
```

### 7.5 Instrumented earlier-reset comparison

The Phase R2 continuation uses the frozen Fixed A checkpoint and the existing PPO learner. A preserves 10% terminal, 45% braking and 45% late-rotation reset probabilities. B transfers 25 percentage points of late rotation to capped-feasible earlier resets. The bank and reserved development variants are frozen before either arm. The experiment driver verifies the archived source contracts and adds scoped observation of the learner and native evaluation. [Configuration](../../configs/rm27_asm2_rotation_bridge_instrumented.json)

Each arm completes 98,304 transitions with learning time measured. The ledger retains complete-update reservations and elapsed time, with transition count as the pilot limit. Resume restores the actor, critic, optimizer, normalization, RNG and sampler, then starts fresh valid simulator episodes. The Docker launcher recognizes this experiment's transition-only authorization.

Regional telemetry captures the band active before each step, the original reward and critic tensors, raw GAE from the learner's existing computation, normalized GAE, sampled actions and exploration statistics. It publishes a compressed archive only after the matching completed-update log row. CPU parity verifies that observation does not change the learner's tensors or RNG. The native evaluator annotates its existing autonomous traces with entry into the frozen transition/braking descriptor before contact or a safety exit.

The readout reports the equal-transition endpoint, development-selected checkpoints, scientific improvement and strict promotion. Episode-start probabilities and transition shares are both measured because longer earlier-reset episodes can dominate PPO batches. [Protocol, outcomes and reproduction](../plans/plan_new_airframe_phase0_01.md#20-instrumented-phase-r2-earlier-reset-pilot)

## 8. Reward and Termination

Reward 分為：

- **Progress shaping**：相對幾何、制動準備、姿態對準、安全距離、控制平滑度等。
- **Qualification event**：只由獨立 contact scorer 決定。

避免：
- 單調鼓勵距離變小而懲罰必要的繞行/旋轉。
- timeout 時仍可取得成功 bonus。
- 提前失敗比繼續控制更有利。
- proximity-only success。
- reward improvement 被誤報為 task success。

Reward 具體形式須在新 contact contract 和 observation 定義後設計，並以固定 evaluator 驗證。

## 9. Contact Geometry and Scoring Architecture

### 9.1 Intended geometry

正式目標為整個機身底部外殼的允許接觸區域。舊 9 mm 球形 fixture 僅保留為 legacy task。

ASM2 integration candidate 使用收到的單剛體 USD `/Drone`，保留 `/Drone/Collision/*` 的 `contact_group` 與 source-part 身分；body frame 為來源推定 FLU、COM origin、metres / Z-up。新的 `src/rm27_asm2/` 與不可覆寫的 `assets/asm2/source/` 分別負責 integration 及來源。可在同一 candidate hierarchy 參照既有 `/Stage1Scene/Field`，但不得繼承舊 Drone 或 sphere。候選 contact events 須保留每個 collider、relative point velocity、normal、force / impulse、signed separation 與 step ID；重複報告不能重複積分。接入 scorer-compatible event record 不代表新 qualification scorer 已定義或凍結。

可獨立引用的 `assets/asm2/candidate/drone.usda` 保留原 USDC，僅以 mesh-local points ×1000 / local scale ×0.001 避免 PhysX small-mesh cooking 失真；不改世界幾何、mass frame 或 collider 語意。候選設定由 `configs/rm27_asm2_collision.json` 固定，與舊 production scene 分離。逐 collider callback 的 force 是 report impulse / dt 的診斷量；在 impulse–momentum consistency 未解決前不得用作已校準物理門檻。驗證結果與限制見 [ASM2_COLLISION_INTEGRATION.md](ASM2_COLLISION_INTEGRATION.md)。

### 9.2 Scorer responsibilities

獨立 scorer 應判定：
- Actual physical contact event。
- First-contact component / region。
- Allowed underside patch membership。
- Local surface normal relationship。
- Projected overlap / contact centroid。
- Relative normal and tangential velocity。
- Angular rate。
- Impulse / penetration / rebound（按可取得的可靠物理量）。
- Forbidden contacts and safety termination。

對非平面底殼，使用局部幾何表示；不得用單一法線掩蓋明顯曲面差異。

### 9.3 Simulator conformance

MuJoCo 是獨立任務評估參考候選；Isaac Lab 是目前高吞吐訓練候選。新底殼必須建立新的 matched-state contact suite。

舊 PGS + 1 µm fixture validation 不自動適用新幾何。

### 9.4 ASM2 Phase 2 contact contract

The Phase 2 evaluator uses the detailed 142 g ASM2 with 14 front patches, one rear patch and the remaining body colliders. The target-centered scene includes 1,822 collision shapes from the frozen 2027 MJB, including all 25 registered target shapes. The native builder verifies the geometry against the archived model and restores per-geom contact parameters including priority and solver mixing. MuJoCo integrates at 0.1 ms for qualification. CTBR commands remain at 5 ms. The matched 1 ms test measures sampling and backend effects.

The scorer records the first manifold with nonpositive signed distance. It rejects initial overlap, body contact or unregistered target contact at first touch or before qualification. The first manifold must satisfy the frozen registered footprint, contact-point velocity and alignment gates. A matching first-touch pair must accumulate the configured compressive load in a physics interval within 1 ms of first touch. Positive-separation reports remain in diagnostic evidence. First qualification terminates the trial. The native contact solver and Isaac contact report interfaces are documented in the [MuJoCo computation reference](https://mujoco.readthedocs.io/en/latest/computation/) and [NVIDIA contact report API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html).

| Provisional gate | Value |
|---|---:|
| Point approach speed along normal | 0.02–0.35 m/s |
| Point tangential speed | ≤0.25 m/s |
| Body, manifold and target normal alignment errors | ≤8° |
| Body rate magnitude | ≤1.5 rad/s |
| Penetration | ≤0.25 mm |
| Registered plane deviation | ≤0.6 mm |
| Compressive load on first-touch pairs | ≥1e-7 N·s within 1 ms |

The configuration records the gentle simulation envelope and source pins. These thresholds carry provisional engineering status. Official competition qualification and hardware limits require separate evidence. Isaac uses per-collider signed events for comparison and reports its impulse data alongside the aircraft momentum change. Native MuJoCo supplies the independent qualification result.

From the project root, choose unused output paths.

```bash
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1
.agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.contact_native \
  --output .agents/runs/asm2-contact/my-native.json
.agents/runs/rm27-stage1/native-env/bin/python -m rm27_asm2.contact_witness \
  --output .agents/runs/asm2-contact/my-witness
./run_asm2_flight_viewer.sh .agents/runs/asm2-contact/my-witness.npz \
  --self-test-output .agents/runs/asm2-contact/viewer/my-contact.png
```

The witness starts level and stationary, holds hover for 0.2 s, and follows a smooth gravity-consistent terminal reference using the provisional CTBR plant. The replay includes recorded poses and the registered triangles from the evaluated model. The witness measures one continuous near-field approach.

### 9.5 ASM2 collision proxy

`assets/asm2/proxy/manifest.json` pins a 3,521-hull candidate generated on the RTX 4090 workstation from CAD-derived meshes. CoACD generates the reduced body parts on the host CPU. The build retains exact parts and outer support regions where the coarse decomposition missed geometry. The 15 contact meshes retain their source bytes and identities. Isaac cooks the proxy and detailed model with the same normalized coordinate convention, PGS solver, 1 µm contact offset and zero rest offset. [CoACD source](https://github.com/SarahWeiii/CoACD)

Selection uses sampled directional support, near-patch support, silhouette distance, guard ray classifications, finite-width guard corridors and PhysX contact regressions. The selected proxy retains the 50 µm carrier cut and the four lateral support extrema. The reference guard false positive at ray 198 remains. The proxy's contact envelope is a sampled approximation.

The manifest records the generator hash, source hashes and dependencies. The existing task environment contains CoACD 1.0.14, NumPy 2.5.4, SciPy 1.18.1, trimesh 5.2.0, embreex 4.4.0.post1 and rtree 1.4.1. Run these commands from the project root with fresh output paths.

```bash
OMP_NUM_THREADS=3 OPENBLAS_NUM_THREADS=1 PYTHONPATH=src \
  .agents/runs/asm2-contact/proxy-env/bin/python -m rm27_asm2.proxy_build \
  --output .agents/runs/asm2-contact/proxy-rebuild
OPENBLAS_NUM_THREADS=2 PYTHONPATH=src \
  .agents/runs/asm2-contact/proxy-env/bin/python -m rm27_asm2.proxy_geometry \
  --output .agents/runs/asm2-contact/geometry-recheck.json
./run_asm2_contact_isaac.sh --geometry proxy --suite fixtures --worlds 8 \
  --output .agents/runs/asm2-contact/my-proxy-fixtures.json
./run_asm2_contact_isaac.sh --geometry proxy --suite field --worlds 8 --layout 1 \
  --output .agents/runs/asm2-contact/my-proxy-field.json
./run_asm2_contact_isaac.sh --geometry proxy --suite bench --worlds 32 \
  --output .agents/runs/asm2-contact/my-proxy-bench.json
```

Use `--geometry detailed` for the matched immutable reference. Benchmarks use a plane fixture, 50 warmup steps and 200 measured physics steps per reporting mode. The Field suite separately checks the registered target at matched starts. Both modes use the same provisional plant.

## 10. Learning Algorithm Strategy

- PPO：第一個 baseline。
- TD3：主要 off-policy challenger。
- SAC：必要時作次要比較。
- Planner/MPC：保留為 feasibility and control baseline。
- Residual RL：當近場擾動/模型殘差被證明重要時再引入。
- Differentiable learning：獨立研究，不阻塞首個可工作的 policy。

演算法比較必須固定其他條件；先驗證同一 task formulation，再比較 algorithm。

## 11. Recovery Separation

Contact policy 在 qualified contact 或明確失敗事件終止。Recovery branch 從事件後真實狀態開始，獨立研究：
- Impact detection。
- Attitude/rate stabilization。
- Position/velocity recovery。
- Estimator/VIO reinitialization。
- PX4 control handover。

整合時必須保留 contact 前後的 state continuity，包括 pose、velocity、attitude、body rate、actuator state、controller state 及 estimator confidence。

## 12. Simulator and Evaluation Flow

```text
CAD / Physical Characterization
             |
             v
      Frozen Plant v0
             |
       +-----+------+
       |            |
       v            v
   Isaac Lab     MuJoCo
   Training      Independent Evaluation
       |            ^
       v            |
   Checkpoint ------+
       |
       v
  Replays / W&B / Evidence
```

所有 evaluation 必須載入明確 checkpoint，記錄 hash、模型版本和完整 suite denominator。

## 13. Key Architecture Decisions

| ID | Decision | Status | Revisit when |
|---|---|---|---|
| ADR-001 | Navigation → Handover → Agile Policy → Recovery | PROPOSED | Navigation cannot reliably provide handover |
| ADR-002 | New task uses CAD underside regions, not legacy sphere fixture | ASM2 15-PATCH CANDIDATE / CONTRACT-PENDING | CAD intent or official rules clarified |
| ADR-003 | Direct-CTBR is first action candidate | PROVISIONAL | New plant/control response shows limitation |
| ADR-004 | PPO baseline, TD3 challenger | PROPOSED | Controlled comparison evidence |
| ADR-005 | Witness states allowed; teacher actions not required | PROPOSED | Research objective changes |
| ADR-006 | Isaac training candidate, MuJoCo independent evaluation | PROVISIONAL | New conformance / throughput evidence |
| ADR-007 | Recovery is separate branch | PROPOSED | Integrated mission evidence justifies coupling |

## 14. Open Design Questions

- Exact CAD-defined underside contact patch.
- Whether the red/yellow underside structure is itself the full contact shell or a subassembly.
- CAD mass / COM / inertia supplied; physical calibration and thrust / motor response unknown.
- PX4 rate-loop bandwidth and CTBR mapping.
- Target armor geometry and official contact judgement details.
- Navigation-achievable handover distribution.
- Required temporal observation architecture.
- Whether one policy or skill hierarchy is best inside the agile phase.
- Contact proxy fidelity and PhysX–MuJoCo conformance.

### Regional update balancing experiment

The isolated `regional_update_experiment` continues both arms from Fixed A with the same earlier-reset bank and mixture. Only B's actor/critic scalar loss reductions receive mean-one regional weights; global advantage normalization, ValueNorm, optimizer and minibatch RNG remain unchanged. Missing regions are reported and their target mass normalized over present regions. All-one weighting reproduces the original PPO update exactly. The pinned learner and historical workflow sources remain unmodified. [Protocol and completed evidence](../plans/plan_new_airframe_phase0_01.md#21-regional-ppo-update-balancing)

The completed weighted candidate raised braking loss mass from 20.3% to 46.5%, but produced only two qualifications on one previously successful reserved anchor, with 33/36 earlier envelope exits and no multi-anchor gain. This remains an isolated experimental adapter, not the default learner or a promoted policy. The frozen baseline remains unchanged. A concurrent Docker image tag rename was verified to resolve to the identical immutable image and handled by `regional_update_runtime` without rewriting the frozen experiment or changing training semantics.
