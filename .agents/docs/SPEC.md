# RM27 Agile Flight RL — SPEC

**Version:** 1.0  
**Date:** 2026-11-10  
**Status:** Provisional ASM2 flight, native contact and sampled simulated handovers verified.  
**Research direction:** Contact-Constrained Agile Surface Interception

---

## 1. Purpose

本文件定義 RM27 Agile Flight RL 的研究目標、任務範圍、核心需求、硬性約束及驗收原則。它是本專案的需求基準；具體架構見 `DESIGN.md`，階段與依賴見 `ROADMAP.md`。

本專案不是單純追求較高 PPO reward，而是研究如何讓微型四旋翼在受限室內環境中，透過 closed-loop learning 完成物理可行、可驗證的敏捷接觸機動。

## 2. Research Problem

### 2.1 Working problem statement

**Can a resource-constrained quadrotor learn a closed-loop agile manoeuvre that coordinates translation, large attitude rotation, terminal braking and controlled physical contact with a designated surface, from a validated navigation handover state, under realistic actuator and contact dynamics?**

中文暫定名稱：**接觸約束敏捷表面攔截**（Contact-Constrained Agile Surface Interception）。

「Perching」是重要先行研究類別，但 RM27 目前的主要 terminal event 是合格的瞬時物理接觸，不要求在表面持續附著或停留。因此不應把本任務直接等同傳統 perching。

### 2.2 Mission context

整體任務預期由以下模組協作：

1. 從原生機庫起飛。
2. Green-light detection / target tracking。
3. VIO / localization 提供自身狀態。
4. Conventional navigation 將飛機送入可驗證的 handover region。
5. Agile policy 自主完成 translation、attitude rotation、braking 和 underside contact。
6. Contact event 後由獨立 recovery branch 處理脫離、姿態穩定與估計器恢復。
7. 後續重新搜尋或繼續任務。

本 SPEC 聚焦第 4–6 項中的 Agile Flight RL 與其接口；不代表上述整個 mission loop 已實現。

## 3. Research Scope

### 3.1 In scope

Phase 1B 已完成隔離自由飛行基準。Phase 2 以 ASM2 的 15 個接觸 patch 與 frozen 2027 target 建立獨立接觸判定、比較 simulator contact dynamics、驗證簡化 GPU proxy，並重現 deterministic contact witness。詳細碰撞資產為驗證參考。此階段授權限 simulation engineering。

Phase 3 驗證既有位置控制器實際到達的模擬 handover states，並以連續原生物理 rollout 判定接觸。接管狀態與 terminal reset samples 分別保存。Held-out navigation recipes 保持完整分母，包含未通過的接觸嘗試。

- 新機型的飛行動力學建模與驗證。
- Direct-CTBR 等候選 action interface 的評估。
- State-based agile contact policy。
- Navigation-to-policy handover contract。
- Witness-based reset curriculum。
- 完整底部外殼接觸幾何與獨立接觸評分。
- Isaac / MuJoCo 的接觸事件一致性驗證。
- PPO baseline 與公平的 TD3 challenger comparison。
- 後續 vision-based policy adaptation 的接口設計。
- 與 PX4 rate-control / actuator chain 的部署兼容性研究。
- 獨立 collision recovery branch 的接口與後續整合規劃。

### 3.2 Out of scope for the initial contact-learning phase

- 直接以 RL 取代全部 navigation / target detection。
- 將 post-impact recovery 強行併入第一版 contact policy。
- 未經驗證的 raw-RGB end-to-end policy。
- 真機接觸或真機碰撞測試。
- 為提高成功率而修改正式比賽規則或獨立 scorer。
- 把舊球形 fixture 的成功率當成新底殼任務的成功率。

## 4. Target System Architecture

預期高層流程：

`Perception / VIO / Tracking → Conventional Navigation → Validated Handover Set → Agile Policy → Qualified Contact → Separate Recovery`

Navigation 負責提供可行的接管狀態；Agile Policy 必須在接管後自主決定如何完成敏捷機動。Handover Set 不可只為了讓 RL 容易成功而任意縮小，必須由實際導航能力、定位誤差、延遲及安全空間共同決定。

## 5. Contact Objective

### 5.1 Intended physical contact

新機型預期使用**原有機身整個底部外殼**作為接觸區域，不再使用舊任務的人工 9 mm 球形 fixture 作正式接觸定義。

使用者已提供 142 g ASM2 STEP / USD / MJCF collision package。本次 integration 必須保留其 14 個 `contact_front`、1 個 `contact_rear` 及其餘 `body` colliders 的語意與個別身分；這是 candidate geometry 分類，不自動凍結新任務的物理 qualification。原有 full-underside 研究意圖、實際允許 shell 範圍與正式數值門檻仍須設計確認。

### 5.2 Geometric intent

目標是底部外殼與裝甲表面形成有效接觸，且接觸區域盡可能對齊及重合。

若兩個面均使用各自物體的 outward normal，理想面對面關係為：

\[
\mathbf n_d \cdot \mathbf n_t \rightarrow -1
\]

此式只表達理想方向關係，不代表已設定數值容差。若底殼為曲面或多平面結構，應使用 local surface normal / contact patch，而不是強行以單一全域平面代表。

### 5.3 Contact admissible set

新任務的有效接觸集合至少需要考慮：

- 合法底部區域是否發生真實物理接觸。
- 首次接觸部件是否符合允許條件。
- 接觸位置及投影重合程度。
- 接觸面局部法線與目標表面法線的關係。
- Normal / tangential relative velocity。
- Contact impulse、penetration 及 rebound。
- Body angular rate。
- 是否有 forbidden contact。
- 是否違反安全空間或機體限制。

`configs/rm27_asm2_contact.json` 凍結 Phase 2 的 provisional engineering thresholds。正式比賽判定及硬體驗收門檻仍為 TBD，依規則、機械結構及實測另行確認。

### 5.4 Independent qualification

Proximity、低 reward、接近目標或 deterministic witness 成功均不能代替正式接觸評分。

RL 成功必須由獨立 event-based scorer 判定，並報告完整分母及拒絕原因。

## 6. Learning Requirements

### 6.1 Policy autonomy

主研究 policy 應在執行期間自主選擇 closed-loop actions。

主方案不得依賴：
- Teacher-action replay。
- Behavior cloning 作為必要前提。
- 完整預設 flight trajectory tracking。
- Runtime deterministic planner 直接接管 policy 的全部動作。

Deterministic planner / witness 可以用於 feasibility analysis、baseline 及 reset-state curriculum。這不等於提供 teacher actions。

### 6.2 Handover and curriculum

訓練應由可驗證的 feasible terminal / approach states 逐步擴展到實際導航可提供的 handover distribution。

建議使用 witness-based reverse curriculum，但只有在真正計算並驗證 reachability set 後，才可稱為正式 backward reachability analysis。

Stationary-hover starts 仍是重要的完整任務能力測試與後期擴展目標，不應被永久排除。

### 6.3 Algorithm

- PPO：保留為第一個主要 baseline，因已有 RM27 basic-flight 和 terminal-braking 資產。
- TD3：作為主要 off-policy challenger。
- SAC：可在有明確研究問題時加入次要比較。
- Model-based planner / MPC：保留作 feasibility、benchmark 和對照。
- Residual RL / differentiable learning：依據後續證據再決定，不預先納入第一輪必要實作。

不得在任務、Observation、Action、Reward 和 reset distribution 不一致時宣稱演算法比較公平。

## 7. Physical Model Requirements

新機型正式訓練前，應建立 Frozen Plant v0，至少包含：

- Total mass。
- Center of mass。
- Full inertia tensor。
- Rotor positions / axes。
- Thrust and torque characteristics。
- Motor / ESC response and lag。
- Mixer / allocation and saturation。
- Low-level rate-controller response。
- Control and physics update rates。
- Contact-relevant geometry and material assumptions。

CAD estimates、manufacturer specifications、bench measurements 和 inferred parameters 必須分開標記。

先進行 system identification，再按量測不確定性決定 selective domain randomization。

## 8. Simulation Requirements

目前主要候選環境：

- Training candidate: Isaac Lab 2.1.0 / Isaac Sim 4.5。
- Independent physics/evaluation reference: MuJoCo。

舊模型的 Isaac PGS + 1 µm contact offset fixture validation 只適用於已測試的舊場景，不代表新底殼模型已通過驗證。

新模型必須建立自己的 contact conformance suite，至少比較：
- First-contact time / component / location。
- Surface normal。
- Contact impulse。
- Penetration。
- Multiple-contact pattern。
- Rebound / friction response。

## 9. Evaluation and Evidence

分開建立以下 benchmark：

1. Basic Flight Benchmark。
2. Agile Manoeuvre Benchmark。
3. Physical Contact Benchmark。
4. Handover Robustness Benchmark。
5. Vision Robustness Benchmark。
6. Collision Recovery Benchmark。

每個評估必須記錄：
- Evaluation distribution、seed、分母。
- Qualified success count / rate。
- Contact kinematics and geometry。
- First contact component。
- Forbidden contacts。
- Safety exits / timeout。
- Mixer saturation / action limits。
- Manoeuvre time。
- Checkpoint hash、model version、simulator version。
- Replay、CSV/JSON、W&B run（若有）。

必要時報告 binomial confidence interval。單次 20/20 不得直接宣稱普遍可靠或真機泛化。

## 10. Sim-to-Real Requirements

進入真機前至少完成：
- 新機型 mass / inertia / thrust / motor-response characterization。
- CTBR-to-PX4 rate-loop interface validation。
- Free-flight sim-to-sim checks。
- Observation timing / latency characterization。
- Policy inference runtime profiling。
- Safety supervisor / failsafe design。
- 由低風險自由飛行逐步進入近距離非接觸測試。

真機接觸需另行制定安全測試方案及授權，不包含在目前研究授權內。

## 11. Historical Evidence and Reuse

歷史結果只可在原任務定義下引用：

- SimpleFlight Direct-CTBR basic flight：保留作 control baseline。
- RM27 old Ver2 Hover / Track：保留作舊模型 flight baseline。
- Old sphere-contact PPO terminal-braking 20/20 moving frontier：保留作 terminal-skill evidence。
- Old sphere-contact stationary-hover contact 0/19、handover 0/57：保留作失敗證據。
- Deterministic stationary-hover witnesses：保留作舊模型 feasibility evidence。
- Old sphere scorer / contact assets：保留為 legacy，不作新底殼任務正式 scorer。

所有歷史 checkpoint、scene、scorer、W&B run、replay 及失敗結果不得覆蓋。

## 12. Non-functional Constraints

- 可重現、可追溯。
- 不因新模型開發而破壞舊研究資產。
- 訓練與獨立評估分離。
- 不把未驗證的 simulator result 標記為物理真實。
- 先做最小必要驗證，避免冗餘診斷。
- 不自動追加訓練預算。
- 不 commit / push，除非另行授權。

## 13. Current Status

| Item | Status |
|---|---|
| Research direction | Proposed baseline |
| Complete new-airframe CAD | ASM2 STEP RECEIVED; focused collision/mass review completed; full design freeze pending |
| New-airframe mass / inertia | User-reported measured 142 g; full inertia and rotor centers CAD_ESTIMATE; actuator calibration pending |
| New full-underside geometry | ASM2 15-patch candidate supplied; NOT_FROZEN |
| New simulation model | Provisional isolated ASM2 flight baseline verified in Isaac and native MuJoCo; detailed collision candidate preserved |
| New contact scorer | ASM2 provisional first-contact contract implemented for native MuJoCo |
| New handover set | Finite navigation-reached sample set verified in native simulation |
| New contact RL training | NOT_AUTHORIZED |
| Legacy evidence preservation | REQUIRED |

## 14. Change Control

下列變更需要更新本 SPEC：
- 任務目標或 RL 接管範圍改變。
- 正式接觸定義或判定規則改變。
- 真機部署邊界改變。
- 核心安全約束改變。
- 新研究方向取代目前主目標。

單純實作步驟或實驗進度變化只更新 PLAN / ROADMAP。
