# RM27 快速啟動

目前主線：先讓一架飛機穩定完成起飛、飛向基地、返回機庫及降落，再擴展至多機。基地高速接觸、碰撞後恢復與視覺導航尚未完成。

## 每次開機

```bash
cd /home/shiuhou/Projects/rm27_drones
source /home/shiuhou/venvs/mujoco/bin/activate
python -m rm27 --help
```

所有指令都在 repo 根目錄執行。不需要啟動 IsaacLab，也不需要設定 `PYTHONPATH`。

## PX4 v1.17.0 迁移工作区

迁移代码位于 `/home/shiuhou/Projects/rm27_drones_px4`，分支为
`codex/px4-v1.17-migration`。[验收状态和限制](docs/PX4_MIGRATION_STATUS.md)。
以下均为本机 SITL，150 g 参数中的惯量、推力和电机滞后仍是工程估计。

四個停機位綠色區域已在獨立 PX4 仿真場景中改為 80 mm AprilTag，
機腹中心增加專用下視相機。建立場景和運行當前視覺回庫開發鏈路的命令見
[視覺 SITL 操作說明](rm27/perception/sitl/README.md)。下視相機實物估重
2-3 g；仿真質量尚未加入。短程閉環已完成指定標記引導、正常 LAND
和自動停槳 20/20，最大物理落位誤差 3.74 mm；完整目標接近／接觸任務
及故障矩陣仍需驗收。[PX4 Teacher／Student 接觸研究入口與失敗證據](docs/PX4_CONTACT_RESEARCH.md)。

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
# 由 PX4 EKF 遥测控制短程往返；数值光流下限 8 cm，无 GPS/ODOMETRY。
/home/shiuhou/venvs/mujoco/bin/python -m rm27 micro-flight \
  artifacts/px4-dock-150g-02/scene.xml --autopilot px4 --profile mtf \
  --dynamics configs/dynamics_150g.json --instance 26 \
  --output artifacts/px4-mtf-new

# 低速接触/恢复/回库：诊断真值辅助，不是视觉验收。
/home/shiuhou/venvs/mujoco/bin/python -m rm27 route-flight \
  artifacts/px4-base-target-150g-02/scene.xml --autopilot px4 \
  --audit artifacts/px4-corridor-audit-02/summary.json \
  --mission configs/px4_corridor_mission_s2.json \
  --dynamics configs/dynamics_150g.json --instance 24 \
  --output artifacts/px4-contact-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.autopilot.px4.recovery \
  artifacts/px4-contact-new --output artifacts/px4-contact-score-new
```

加 `--view --speedup 1` 可观看当前闭环飞行；无桌面时省略 `--view`。
每次使用新的 output。读 `summary.json`，恢复门槛还需查看独立 scorer。
`--profile ekf` 使用合成外部定位；`flow` 使用 1 cm 数值光流下限；
`combined` 使用合成外部定位加数值光流。均不代表实机视觉已合格。

```bash
# 精确重放已录制的电机输入，验证物理状态。
/home/shiuhou/venvs/mujoco/bin/python -m rm27.autopilot.px4.replay \
  artifacts/px4-mtf-new --output artifacts/px4-mtf-replay-new
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

响应探针和图像/OpenVINS 输入命令见
[仿真感知说明](rm27/perception/sitl/README.md)。

## 最新研究：Phase 5 無 GPS EKF 接觸／恢復

[完整報告與精度要求](docs/ESTIMATOR_AWARE_HYBRID_MANEUVER_REPORT.md)。真實 ArduPilot SITL 已使用合成外部定位測量、關閉 GPS；actor 讀取 EKF 遙測。留出測試 9/12 通過，尚未接入真實相機／VIO，也不是完整機庫往返。

直接觀看已記錄的通過樣本（Space 暫停，V 切換全場／跟隨）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase5-sitl-externalnav-01/evaluation/flight --speed 1 --loop
```

這是與原飛行狀態核對過的馬達輸入回放，不啟動新的飛行。[完整動作 GIF](artifacts/phase5-render-01/maneuver.gif)。

重新跑實際固件閉環（僅本機 SITL；每次使用新的 output 目錄）：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation telemetry --action-layer attitude --parameters configs/contact_phase5/externalnav.parm --study configs/contact_phase5/externalnav.json --instance 47 --duration 9 --output artifacts/phase5-externalnav-new
```

這些參數只用於模擬，沒有真機標定。失敗 trial 保留且退出碼非零；主動中止也不算接觸任務成功。

精度／不確定性 batch：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sensitivity --config configs/contact_phase5/guard_holdout.json --output artifacts/phase5-guard-new
```

batch 的 `summary.json: passed` 表示執行完整；每個 trial 的 `passed` 和 `safe_abort` 分別表示接觸成功與安全中止，不可混算。

## 查看場景

新 STEP 飛機：單台放在機庫一號位（靜態外觀）：

```bash
python -m rm27 view artifacts/cad-drone-dock-20260916-v3/scene.xml
```

結構判讀、尺寸與重建指令：[CAD_DRONE.md](docs/CAD_DRONE.md)。

獨立測試場地（只有機庫、支柱、現用藍方基地與地面）：

```bash
python -m rm27 view artifacts/isolated-dock-base-20260916-v6/scene.xml
```

重建：`python -m rm27 build-isolated --output artifacts/isolated-new`
此場景沒有飛機；基地塔身仍是視覺模型，碰撞範圍見 [獨立場景說明](docs/ISOLATED_SCENE.md)。

新增場地：沿用上面的 v6，只移除紅方 H 機庫支柱、頂板及柱腳，加入紅方飛鏢發射區；獨立停機庫與藍方基地保留原位。

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27 view artifacts/isolated-blue-base-red-dart-20260927-v1/scene.xml
```

重建：`python -m rm27 build-isolated-dart --output artifacts/isolated-red-dart-new`
停機庫仍固定在原高度；此場景沒有飛機或飛鏢發射動畫。來源、碰撞範圍與驗證見 [獨立飛鏢場景](docs/ISOLATED_RED_DART.md)。

無人機先短距離起飛、飛鏢發射後主動攔截的獨立 MuJoCo 仿真。在專案根目錄執行互動回放：

```bash
cd /home/shiuhou/Projects/rm27_drones
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.dart_interception view artifacts/dart-interception-aligned-20261009-v1 --speed 0.5 --camera free
```

預設自由視角，可用滑鼠旋轉、平移與縮放。按 `1` 看全場，`2` 跟隨無人機，`3` 跟隨飛鏢，`4` 自動切換，`5` 回到自由視角。飛機從機庫 `slot_1` 垂直起飛並等待發射；飛鏢碰撞前的位置與姿態沿用 `flight-04-apex-5m`，首次碰撞後切換為動態剛體。撞前朝向屬於指定規則，非空氣動力模型。此回放的通過條件包含攔截結果，未包含撞後穩定恢復。

SSH 無桌面時，請在有桌面的機器用影片播放器開啟 `artifacts/dart-interception-aligned-render-20261009-v1/flight.mp4`。重跑、無攔截對照與限制見 [飛鏢攔截實驗](docs/DART_INTERCEPTION.md)。

完整場地、機庫與四架靜態飛機：

```bash
python -m rm27 view artifacts/dock-final/scene.xml
```

加入基地綠燈與測試接觸片的場景：

```bash
python -m rm27 view artifacts/base-target-v4/scene.xml
```

Viewer 只用來查看場景，不會自動開始飛行。

## 單機短程起降

```bash
RM27_RUN="artifacts/micro-flight-$(date +%Y%m%d-%H%M%S)"
python -m rm27 micro-flight artifacts/dock-final/scene.xml \
  --output "$RM27_RUN" --instance 31 --view
python -m json.tool "$RM27_RUN/summary.json"
```

流程：起飛、懸停、短距離平移、返回、降落及停槳。省略 `--view` 可在終端快速執行。這個任務使用 MuJoCo 真值輔助精準返回。

## 飛往基地前等待點並返回

```bash
RM27_ROUTE="artifacts/route-$(date +%Y%m%d-%H%M%S)"
python -m rm27 route-flight artifacts/base-target-v4/scene.xml \
  --audit artifacts/corridor-reviewed/summary.json \
  --output "$RM27_ROUTE" --instance 43 --view
python -m json.tool "$RM27_ROUTE/summary.json"
```

這是低速、真值輔助的完整往返，約 594 秒物理時間；目前不會撞擊裝甲板。省略 `--view` 可無視窗執行。

分析路線或驗證馬達輸入重播：

```bash
python -m rm27.experiments.analyze_route "$RM27_ROUTE" --output "${RM27_ROUTE}-analysis"
python -m rm27.simulation.replay_micro_flight "$RM27_ROUTE"
```

## 重建機庫場景

只有 CAD、場地版本或配置改變時才需要重建：

```bash
RM27_SCENE="artifacts/dock-$(date +%Y%m%d-%H%M%S)"
python -m rm27 build-dock --output "$RM27_SCENE"
python -m rm27.integration.check_scene "$RM27_SCENE/scene.xml"
python -m rm27 view "$RM27_SCENE/scene.xml"
```

重新生成基地目標場景：

```bash
RM27_BASE="artifacts/base-target-$(date +%Y%m%d-%H%M%S)"
python -m rm27.integration.base_target artifacts/dock-final/scene.xml --output "$RM27_BASE"
python -m rm27 view "$RM27_BASE/scene.xml"
```

## 測試

整段 maneuver（機庫上方 → 接觸 → 恢復確認），全局軌跡直接準備接觸姿態：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver --parameters configs/contact_maneuver_early_pose.json --view --output "artifacts/maneuver-view-$(date +%Y%m%d-%H%M%S)"
```

已跑通 nominal 約 7.10 秒；更快版本使用 `configs/contact_maneuver_fast.json`，約 5.67 秒，
但 stress 留出集有一次禁碰。比較與限制見 [整段 maneuver 報告](docs/WHOLE_MANEUVER_REPORT.md)。
計時從空中初態到恢復保持完成，不含起飛／返航／降落。以下 `arena` 入口仍可觀看原往返任務。

獨立 aggressive contact / recovery 研究（MuJoCo 真值、直接馬達控制，非 SITL 任務）：

```bash
python -m rm27.experiments.aggressive_contact episode --output artifacts/contact-my-demo
python -m json.tool artifacts/contact-my-demo/summary.json
```

包含連續進場、底部接觸件與傾斜板的物理接觸、脫離及恢復；從空中初始狀態開始。

完整現有 RMUC 場地的閉環任務（機庫上方 → 藍方基地接觸 → 恢復 → 返回機庫上方），
在有桌面的終端貼上下面**一整行**即可觀看：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena --view --output "artifacts/contact-arena-$(date +%Y%m%d-%H%M%S)"
```

Space 暫停，V 切換全場／跟隨視角；`--speed 0.5` 慢速，去掉 `--view` 為 headless。
需要本機已有的 `artifacts/dock-final/scene.xml` 及其場地／機庫資產；不會重新生成假場地。
這是空中起始、return-near-dock，沒有起飛／精準降落／停槳，也不是 SITL 或真機驗收。
重看已驗證的 v2 記錄（播放已存狀態，不是新一次閉環飛行）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/contact-arena-selected-v2-20260923 --speed 0.5 --loop
```

每次新實驗使用新目錄。更多批次比較、參數選擇、恢復區域與回放命令見
[研究模組說明](rm27/experiments/aggressive_contact/README.md)及
[v2 場地／魯棒性報告](docs/AGGRESSIVE_CONTACT_V2_REPORT.md)。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

只在首次配置或依賴改變時安裝：

```bash
python -m pip install -r requirements-integration.txt
```

## 查看已有結果

```bash
python -m json.tool artifacts/s0-batch-01/summary.json
python -m json.tool artifacts/route-flight-reviewed/summary.json
xdg-open artifacts/route-analysis-03-fixed/route.png
```

不要按資料夾名稱判斷成功；以 `summary.json` 中的 `passed: true` 為準。

## 常見錯誤

| 問題 | 處理方式 |
|---|---|
| `No module named rm27` | 回到 repo 根目錄，重新執行「每次開機」三行指令。 |
| 輸出目錄已存在 | 使用新的時間戳目錄，保留舊實驗證據。 |
| `Address already in use` | 確認沒有同一測試仍在執行，或改用另一個 `--instance`。 |
| Viewer 無法開啟 | 在本機桌面終端執行；SSH 環境只跑無視窗測試。 |

## 詳細文件

| 內容 | 文件 |
|---|---|
| Milestone 與驗收條件 | [docs/MILESTONES.md](docs/MILESTONES.md) |
| 專案架構與責任邊界 | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| 單機短程飛行結果 | [docs/MICRO_FLIGHT.md](docs/MICRO_FLIGHT.md) |
| 基地路線與安全走廊 | [docs/CORRIDOR_REPORT.md](docs/CORRIDOR_REPORT.md) |
| 真機硬件與待確認資料 | [docs/REAL_HARDWARE.md](docs/REAL_HARDWARE.md) |
| `.BIN`、動力與接觸校準 | [docs/CALIBRATION_WORKFLOW.md](docs/CALIBRATION_WORKFLOW.md) |

`docs/archive/chatgptre.md` 只作歷史參考；當前規格以最新指示、原始 CAD／日誌、官方資料及可重現測試為準。

## Whole-Maneuver Teacher：完整场地进场、接触与返航

最新 [Teacher 报告](docs/WHOLE_MANEUVER_TEACHER_REPORT.md) 包含三档速度／余量比较、
rate/thrust 代理、dataset 与证据。balanced 从机库附近空中初态到保持恢复约 6.1 秒；
当前依然是 MuJoCo truth + motor controller，未验证 ArduPilot SITL 或真机。

先生成一次完整往返（新 output 目录），再在本机桌面观看录制状态：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-return --parameters configs/contact_teacher_balanced.json --output artifacts/teacher-my-demo
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/teacher-my-demo --speed 1 --loop
```

`Space` 暂停，`V` 切换全场／跟随；`1` 跟随、`2` 全场、`6` 镜头第一视角。每条命令整行粘贴，不要在 `-m` 后断行。
这包含 departure → physical bottom contact → separation → held recovery → return-near-dock；
起点已经在空中，尚无起飞／精准落库。已有录制可用 `artifacts/teacher-return-01/balanced`。

直接从镜头位置观看这段已有录制：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/teacher-return-01/balanced --camera fpv --speed 1 --loop
```

[第一视角 MP4](artifacts/teacher-return-fpv-20260927-v2/first_person.mp4) 已离屏生成。
相机依据 CAD 前端小圆柱端面与向下 20° 轴向；CAD 到 Teacher 简化机体的对齐仍是展示假设，
不是实机外参标定。位置、朝向与假设记录在 `configs/contact_fpv_camera.json`；
`--fovy 60` 调整垂直视野，`--camera-config PATH` 指定另一份安装配置。
完整说明见 [Teacher 第一视角](docs/WHOLE_MANEUVER_TEACHER_REPORT.md#镜头第一视角回放2026-09-27)。

重新导出第一视角视频（新目录）：

```bash
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render-maneuver --input artifacts/teacher-return-01/balanced --output artifacts/teacher-fpv-new --camera fpv --speed 1
```

## Whole-Maneuver Distillation：观看实际 student

[Distillation 报告](docs/WHOLE_MANEUVER_DISTILLATION_REPORT.md) 记录了新数据、BC/history/DAgger 的负结果，
以及有效的 **learned reference + model feedback** 混合 student。它不是已成功的纯神经控制器。
本轮 hybrid 理想观测 72/72、synthetic noisy 70/72；额外 10 ms ego delay 下明显退化。

直接观看已跑通的 noisy 完整往返回放：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/distill-mission-03 --speed 1 --loop
```

重新运行一条闭环任务（使用新 output 目录）：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation mission --policy artifacts/distill-spline-01/policy.npz --profile noise_filtered --output artifacts/distill-my-demo
```

`Space` 暂停，`V` 切换视角。任务为空中出发→接触→恢复→model-based return-near-dock；
没有精确降落、SITL、真实视觉或硬件验证。student actor 不读 MuJoCo contact truth。

## Hybrid Student Phase 3：延遲觀測、完整場地接觸與返航

本輪已完成的完整任務：50 Hz synthetic ego estimates、10 ms delay、10% dropout，
機庫上方出發 → 底部接觸 → 恢復 → 返回機庫附近，共20.014秒。
詳見 [Phase 3 report](docs/HYBRID_STUDENT_DEPLOYMENT_REPORT.md)。沒有SITL或真機驗證。

直接在MuJoCo viewer看這次實際飛行（整行貼上；這是記錄狀態播放）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase3-mission-01 --speed 1 --loop
```

重新計算閉環並保留新結果（output目錄必須尚不存在）：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3 mission --config configs/contact_phase3/deployment_selected.json --output artifacts/phase3-mission-new
```

完成後把viewer的`--input`改為新目錄即可。
無桌面環境可看 [完整動作GIF](artifacts/phase3-render-01/maneuver.gif)；
[閉環比較圖](artifacts/phase3-results-01/paired_trajectory.png)顯示舊hybrid在20 ms延遲下的恢复振盪。

## Phase 4：真实 ArduPilot 固件控制接触实验

[结果、接口语义及限制](docs/HYBRID_POLICY_SITL_REPORT.md)。已完成一个真值辅助的 nominal 接触／脱离／恢复；EKF 遥测版本仍先碰边框。不是完整机库往返，也不是视觉或真机验证。

观看已记录的通过样本（完整 RM 场地；按 V 切换总览，Space 暂停）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase4-unshaped-truth-01/evaluation/flight --speed 1 --loop
```

观看失败的 EKF 样本：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase4-unshaped-telemetry-01/evaluation/flight --speed 1 --loop
```

以上是与原飞行状态完全匹配的已记录马达输入回放，不启动新的 SITL。
重新运行实际固件闭环（只使用本机 SITL；输出目录必须尚不存在）：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.flight_control.sitl --mode maneuver --observation truth --action-layer attitude --parameters configs/contact_phase4/attitude_unshaped.parm --instance 45 --duration 9 --output artifacts/phase4-truth-new
```

把 observation 改成 telemetry 可复现当前 EKF 接触误差；必须另选输出目录。参数文件仅用于这个仿真植物，不能直接用于真实飞控。当前研究采用空中初始化 fixture 和固定目标地图；尚未接入真实 VIO／光流或视觉目标。
