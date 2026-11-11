# AGENTS.md — RM27 Agile Flight RL

## 專案定位

本專案負責 RM27 新一代四旋翼的 **Contact-Constrained Agile Surface Interception** 研究與工程實作。目標不是單純提高 PPO reward，而是建立一套能在實際新機型上驗證、可追溯、可逐步部署的敏捷接觸控制系統。

本文件是 Agent 的工作規範。需求以 `.agents/docs/SPEC.md` 為準，架構以 `.agents/docs/DESIGN.md` 為準，階段狀態以 `.agents/docs/ROADMAP.md` 為準，當前工作以 `.agents/plans/` 中明確標記為 Active 的 plan 為準。

同時遵循根目錄 [engineering.md](engineering.md) 的工程規則。既有 `simpleflight_lab/`、`configs/`、`upstream/`、`.agents/runs/` 及舊測試保留原位；`src/` 為未來新機型實作入口，不表示既有程式已遷移或新模型已完成。

## 文件權責與更新規則

| 文件 | 權責 | 何時更新 |
|---|---|---|
| `SPEC.md` | 研究目標、需求、約束、驗收原則 | 目標、範圍、硬性約束或驗收定義改變 |
| `DESIGN.md` | 系統架構、模組責任、資料流、技術決策 | 架構、接口、演算法方案或模組分工改變 |
| `ROADMAP.md` | Milestone、依賴、優先級、階段狀態 | 階段狀態、里程碑、依賴或優先級改變 |
| `plan_*.md` | 當前具體工作、步驟、結果、阻塞 | 任務進度或實驗結果改變 |
| `AGENTS.md` | Agent 長期工作規則 | 工作規則改變 |

不要把短期任務細節塞進 SPEC，也不要只改 PLAN 而遺漏已改變的正式需求或架構。

## 工作原則

1. **先確認現況，再修改。** 開始工作時檢查相關 repository、branch、working tree、submodule pin、現有程式與文件；不得假設路徑、檔案或實作存在。
2. **只檢查與本任務相關的內容。** 已有有效證據且未受本次變更影響的測試，不要無理由重跑。
3. **Working implementation first。** 在明確範圍內自主完成實作、測試與驗證，不要每完成一個小檢查就停下來等批准。
4. **遇到真正的決策點才詢問。** 若選擇會改變研究目標、正式判定、機體幾何、物理假設、安全邊界或不可逆資產，先提出簡短選項與證據；普通工程細節可自行決定並記錄。
5. **不虛構完成狀態。** 區分程式存在、測試通過、模擬器驗證、獨立評估、真機驗證。
6. **不以 reward 代替任務成功。** 所有 RL 成功率以獨立 evaluator 的正式判定為準。
7. **不混合不同任務的分母。** Basic flight、agile manoeuvre、contact、vision robustness、recovery 必須分開報告。
8. **保留失敗證據。** 不刪除或覆蓋歷史 checkpoint、log、replay、scorer、舊模型及失敗實驗。
9. **不把 deterministic witness 算作 RL 成功。** Witness 可用於 feasibility、baseline、reset-state curriculum；除非另有明確研究比較，不向主 policy 提供 teacher actions，也不把完整 witness trajectory 當作 tracking reference。
10. **不任意縮小 Handover Set。** 其範圍必須與實際導航/Tracking 可提供的狀態相符，並以獨立測試證明可達。
11. **不預先杜撰門檻。** 接觸角度、速度、重合率、衝量、成功率等數值，必須由正式規則、CAD 幾何、物理測試或預先註冊的實驗依據決定。
12. **新舊模型分開管理。** 舊 Ver2 + 球形 fixture 的結果只能標記為歷史任務結果，不得重新命名成新機型全底殼接觸成果。
13. **先量測再 randomize。** Domain Randomization 的範圍要有物理依據；不得用任意大範圍 randomization 掩蓋未知模型。
14. **保留獨立驗證。** Isaac training result 不自動等於 MuJoCo qualification；新接觸模型必須重新建立 simulator conformance evidence。
15. **安全預設。** 不操作真機、不解鎖、不發送 motor/flight-control commands、不更改 PX4 firmware，除非使用者另行明確授權並定義安全程序。

## Bounded Autonomous Research Engineering

Operate autonomously within the approved objective, scope and resource budget.

When encountering a blocker:

1. Stop the invalid experiment, not necessarily the engineering work.
2. Form a testable hypothesis and attempt the smallest justified correction.
3. Validate against frozen baselines, acceptance criteria and negative controls.
4. Continue only when evidence supports the change. Otherwise revert and investigate a distinct hypothesis.
5. Bound exploration by the task's diagnostic budget. Avoid repeating unsuccessful approaches or expanding scope without justification.

**Escalate when:**

- Multiple justified attempts fail to establish progress.
- The evidence contradicts the current research assumptions.
- The solution requires changing objectives, acceptance criteria, protected assets or substantially increasing resource budgets.

Never weaken evaluation criteria, conceal failures, or substitute apparent progress for independently verified results.

**Principle: Autonomy in execution, discipline in experimentation, human oversight for strategic decisions.**

## 證據標記

在文件、報告、表格和實驗記錄中使用以下標記：

- `CONFIRMED`：由目前可用的直接證據確認。
- `REPORTED`：歷史報告有記錄，但本次未重新獨立重現。
- `INFERENCE`：根據已知證據作出的推論。
- `HYPOTHESIS`：待實驗驗證的假設。
- `UNKNOWN`：目前沒有足夠資料。
- `NOT_RUN`：尚未執行。
- `BLOCKED`：因明確依賴項未完成而不能繼續。

同時標明證據來源、日期、模型版本、commit 或 artifact identifier（若有）。

## 實驗規範

每個正式實驗至少記錄：

- Experiment ID、日期、目的與假設。
- Airframe/model version。
- Simulator、physics configuration、control rate。
- Policy architecture、observation、action、reward。
- Reset distribution / seed。
- Training transitions、wall-clock、硬體資源。
- Checkpoint hash。
- 獨立 evaluation suite、分母、成功數及失敗原因。
- Replay、CSV/JSON、W&B run（如適用）。
- 與 baseline 的 matched comparison。
- 結論、限制及下一步。

一次實驗應回答一個主要問題。若同時改變多個因素，必須明確標記為 architecture candidate，不得聲稱因果已隔離。

## 執行節奏

- 優先完成當前 Active PLAN，不要擅自擴大範圍。
- 每次更新以「做了什麼、結果如何、下一步」簡短回報。
- 只有遇到實質阻塞、風險或需要使用者決策時才中斷。
- 不自動追加訓練預算。
- 不 commit、push、改動其他 repository 或 submodule，除非使用者明確要求。
- 完成階段後更新 PLAN、ROADMAP；若正式需求或架構改變，再同步更新 SPEC / DESIGN。

## 當前專案狀態（2026-11-10）

- 新無人機完整 CAD：`ASM2_PACKAGE_RECEIVED / PARTIALLY_REVIEWED`。
- 新機型物理參數：使用者提供的 142 g 實測值；inertia/rotor centers 為 CAD estimate，actuator/control 為 provisional。
- 新機型模擬模型 `PROVISIONAL_FREE_FLIGHT_PASS`。Phase 2 接觸判定與 proxy 驗證狀態以 [ROADMAP](.agents/docs/ROADMAP.md) 為準。
- 新機型正式接觸訓練：`NOT_AUTHORIZED`。
- 舊 Ver2 + 9 mm sphere 的歷史實驗：保留為 `LEGACY_EVIDENCE`。
- 下一個 Active PLAN：`plan_new_airframe_phase0_01.md`。

目前不得開始新一輪 PPO/TD3 contact training。
