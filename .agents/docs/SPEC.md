# RM27 Agile Flight RL — SPEC

**Version:** 1.1

**Date:** 2026-11-18

**Status:** N1/R9 alignment complete; published G0 contracts pinned locally at 9ea80518259173c15882ae8a3ab6059eb2643336. R9-A offline consumer PASS (55/55 CPU tests); overall R9-A, formal RL evaluation and R9-B remain BLOCKED by the published dataset and compatibility gaps. Historical R8 and demonstration-regularized R9 remain Scientific No-Go; no new training/native budget.
**Research direction:** Contact-Constrained Agile Surface Interception

---

## 1. Purpose

本文件定義 RM27 Agile Flight RL 的研究目標、任務範圍、核心需求、硬性約束及驗收原則。它是本專案的需求基準；具體架構見 `DESIGN.md`，階段與依賴見 `ROADMAP.md`。

本專案不是單純追求較高 PPO reward，而是研究如何讓微型四旋翼在受限室內環境中，透過 closed-loop learning 完成物理可行、可驗證的敏捷接觸機動。

## 2. Research Problem

### 2.1 Working problem statement

**Can a resource-constrained quadrotor learn a closed-loop agile manoeuvre that coordinates translation, large attitude rotation, terminal braking and controlled physical contact with a designated surface, from a validated navigation handover state, under realistic actuator and contact dynamics?**

2026-11-18 對齊的完整研究問題：在導航實際可達、且具有物理可行性證據的一組帶速進場狀態中，RL 能覆蓋多大的狀態範圍，完成 **qualified first contact → safe separation → stable recovery**？與 Non-RL 使用相同 scenario／模型身份及驗收語義比較。物理可行性與控制器能力均以有限實證表達，不宣稱已認證 reachable set。既有 first-contact 結果維持原任務語義。

中文暫定名稱：**接觸約束敏捷表面攔截**（Contact-Constrained Agile Surface Interception）。

「Perching」是重要先行研究類別。既有 RL benchmark 的 terminal event 是合格首次物理接觸；完整研究任務還要求分離與穩定恢復，不要求在表面持續附著或停留。

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

### 4.1 視覺導航與 RL 分工確認（2026-11-15）

來源：使用者本輪目標對齊。`CONFIRMED` 指需求已確認，不代表能力已驗證。

- 使用者側實作視覺導航，以綠燈為識別物，控制飛機朝目標方向及指定高度持續接近，可以減速，不在固定距離懸停等接管。
- 本 RL 專案負責給出有驗證依據的進場要求：飛多近、飛多高、飛到什麼相對位置，以及速度、姿態、角速度和執行器狀態的允許範圍。導航主動配合這個範圍；本專案不替使用者開發導航或 MPC。
- 最終目標是 RL 在接近途中根據即時狀態選擇接管時機。N1/R9-A/B 先使用事前凍結的 fixed-handover slices 評估控制能力，暫不新增 discrete handover policy。若一直未接管，導航在安全退出餘量耗盡前退出，重新進場；不得強制在最後一刻交給 RL。
- 完整任務目標為自主接管、居中對齊、合法底面觸板，以及觸板後脫離並恢復穩定可控飛行。量化 gate、恢復實作及與本地 evaluator 的相容性依 §4.2 辦理，不由附件敘述直接更改本地 scorer。
- 既有首次接觸 scorer 與歷史任務結果保持原義；首次接觸合格不代表恢復成功，也不代表官方撞擊計分合格。初期 contact-only 研究與完整任務目標分開驗收。
- 0.65 m 靜止懸停、固定進場路徑、先向外繞均不是本次任務要求。接管範圍不得由單一孤立成功狀態或幾何相似性直接宣稱有效。

2026-11-15 對齊當時僅授權文件整理；2026-11-18 後續 R9-A 的 CPU consumer 授權依 §4.2 辦理，仍無新訓練、native rollout、硬體操作或導航實作授權。接口與候選算法見 [DESIGN §6.4](DESIGN.md#64-視覺導航至-rl-接管接口2026-11-15)。

### 4.2 N1/R9 共用研究邊界（2026-11-18）

- **共用的權威輸入**為 5060 正式發布的 G0 Target/State、Navigation-to-Handover、Physics/Evaluation 契約及 Scenario Set 的格式與版本身份。4090 僅做 consumer 與本地適配，不另外定義同名契約、手填 schema 或重建 scenario generator。本次取得發布 commit `9ea80518259173c15882ae8a3ab6059eb2643336` 的 80 個檔案，保留於 `upstream/rm27_g0/9ea8051/`；契約為 `0.1.0-draft / PROPOSED`、snapshot 為 `rm27.handover/1`、scenario manifest 為 version `1`。來源、逐檔 SHA-256 與 producer receipts 見 `.agents/runs/n1-r9-a/acquisition.json`；接收方產生的 inventory 不冒稱 producer 正式 release checksum manifest。
- 此發布版本只有 3 筆 legacy navigation diagnostic records、1 個 `dev` source family、**0 個 admitted recorded scenarios**，不是可直接評估 RL 的完整 dataset。另取得的 producer closure receipt 描述 3 筆 `RECORDED_SIM_ESTIMATED`、單一 admitted dev family，但明示 `published=false`；該未發布 fixture 與 remote dirty code 不匯入、不混入已釘選資料。producer 未提供 committed standalone checksum/test receipt 時，保留原 receipt 與其版本差異，不聲稱交付完整。
- 共同比較需要核對 frame、unit、quaternion convention、綠燈到板面標定、機體／碰撞／執行器／scorer hashes、控制與物理時序、scenario/source/split 身份。5060 路徑與 Git revision 是發布方 provenance，必須與 4090 本地路徑及實際 hashes 分開。
- **維持獨立**：RL observation encoder、action decoder、PPO objective、Actor/Critic、normalizer、optimizer、reset sampler 及 training pipeline。共用 snapshot 不等於共用 Actor 輸入，更不授權移植 Non-RL planner 或 teacher actions。
- runtime observation 只可使用發布契約中可取得的估測／歷史欄位；simulator GT 僅供恢復物理初態、驗證及診斷。既有 45D Actor 使用模擬真值的歷史評估不重新命名為 estimated-runtime conformance。缺少可用欄位時記錄 blocker，不以 GT 補齊。
- 已發布 protocol 的接管輸出是 `REFERENCE_TRAJECTORY`；其 Snapshot、ACK、lease 與 authority conformance 不代表 Direct-CTBR runtime authority。資料重放僅為離線逐筆重放，不代表 plant state restore 或新 native rollout。G0 D1 的 3522 與本地 RL 3521 proxy 等模型身份須另行比對，不能以同名 ASM2 推導相容。
- split 按獨立 navigation source trajectory／family 凍結，相關 frames 不跨 train/dev/heldout；既有 R8/R9 fresh suite 已被使用，不能重新當成未見盲測集。Scenario 需含完整物理／執行器狀態與控制歷史，runtime view 和 diagnostic GT 分離。
- Fixed-handover capability 與自主選擇接管的 E2E 評估分開。前者所有預定合法 slices 都入分母，不能由 selector 挑易例；後者所有合法 navigation attempts 入分母，另報 acceptance、abort、conditional success。
- 完整任務獨立記錄 first-contact、forbidden contacts、separation、stable recovery、飛行包線和 timing。附件引用的 8 cm 分離、1 s hold、0.95 m/s speed、5 mm clearance 為來源報告 gate；其量測對象、事件、窗口和版本須由正式 G0 release 核對，**不自動成為本機新門檻**。沒有 full-task evaluator 時，full-task 欄位為未評估，不能由 contact-only PASS 推導。
- 同軸、居中、相向面接觸為 nominal 設計目標，並非經證明的全域最優策略；接觸點速度、衝量作用線與 COM 必須使用一致座標。官方撞擊計分與研究工程資格持續分開。

當前授權 R9-A 的最小 consumer 實作、離線 loader/schema/provenance/observation-isolation/replay CPU tests 與 fixed-handover protocol 準備；不啟動 plant step、native contact campaign、PPO 或硬體，也不啟用歷史剩餘預算。最終驗證與缺項以 `.agents/runs/n1-r9-a/` 原始輸出為準。若任務內容指向其他 host 或 Non-RL component，應先停止並提醒使用者，不在 5060 執行該工作。R9-A/B 的階段與 gates 見 [ROADMAP §2.1](ROADMAP.md#21-n1r9-研究路線2026-11-18)。

## 5. Contact Objective

### 5.1 Intended physical contact

物理目標是原有機身底部朝向裝甲板的接觸，不再使用舊任務的人工 9 mm 球形 fixture。當前 ASM2 工程合法部位維持指定 15 個 front/rear patches，其餘 body 維持 forbidden；full-underside 幾何研究意圖不代表整個底殼均已獲准接觸。

使用者已提供 142 g ASM2 STEP / USD / MJCF collision package。保留其 14 個 `contact_front`、1 個 `contact_rear` 及其餘 `body` colliders 的語意與個別身分。現有 provisional first-contact 工程契約已實作；任何共用 G0 模型／判定相容性差異均需明確處理，不在研究計劃對齊中修改物理、幾何、接觸偏移或 scorer。

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

N1/R9 的當前基線仍為既有 PPO + Direct-CTBR。R9-A 只實作離線資料消費、CPU conformance 與 protocol 準備，R9-B 是需獨立批准的單一可反證 learning-mechanism experiment；機制與預算尚未選定。歷史 R9 demonstration regularization 已完成 No-Go，不能照附件較早的提案重跑。以下其他算法僅為候選清單，不構成 sweep 或實作承諾。

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
| Research direction | N1/R9 shared fixed-handover capability first; autonomous handover and full recovery are future gates |
| Complete new-airframe CAD | ASM2 STEP RECEIVED; focused collision/mass review completed; full design freeze pending |
| New-airframe mass / inertia | User-reported measured 142 g; full inertia and rotor centers CAD_ESTIMATE; actuator calibration pending |
| New full-underside geometry | Existing ASM2 legal 15-patch engineering contract retained; no geometry changes authorized |
| New simulation model | Provisional isolated ASM2 flight baseline verified in Isaac and native MuJoCo; detailed collision candidate preserved |
| New contact scorer | ASM2 provisional first-contact contract implemented for native MuJoCo |
| New handover set | Finite navigation-reached sample set verified in native simulation |
| Learned baseline | Phase 4 checkpoint-186368.pt retained; Fixed A is continuation reference |
| Historical R8 / R9 / R9.1 | Completed No-Go under their original definitions; evidence preserved |
| Shared G0 local identity | 9ea80518259173c15882ae8a3ab6059eb2643336; 0.1.0-draft / PROPOSED; 80 published files pinned locally; acquisition provenance retained |
| Shared G0 dataset / formal RL evaluation | BLOCKED: published 3 diagnostic records, 1 dev family, 0 admitted recorded scenarios; unpublished closure is separate evidence |
| R9-A consumer | Offline PASS, 33 official + 22 local CPU tests; overall R9-A BLOCKED; receipt .agents/runs/n1-r9-a/results.json; no Actor or plant changes |
| New contact RL training | NOT_AUTHORIZED; historical training budgets closed |
| Legacy evidence preservation | REQUIRED |

## 14. Change Control

下列變更需要更新本 SPEC：
- 任務目標或 RL 接管範圍改變。
- 正式接觸定義或判定規則改變。
- 真機部署邊界改變。
- 核心安全約束改變。
- 新研究方向取代目前主目標。

單純實作步驟或實驗進度變化只更新 PLAN / ROADMAP。

### R8 bounded demonstration-assisted authorization — historical, closed

R8 explicitly permits supervised action learning from the seven saved, native-qualified optimized-prefix/Fixed A connections, followed by ordinary PPO. This is demonstration-assisted learning, not a claim of pure model-free learning. Source/phase, future actions and full actuator state remain provenance/verification metadata, never Actor inputs. Independent evaluation must generate every action through the learned45-D Direct-CTBR Actor, with no planned prefix, handover or teacher switching. Frozen first-contact qualification and protected baselines are unchanged.
