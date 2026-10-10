# RM27 轉用 PX4 v1.17.0：現有介面與遷移研究

研究日期：2026-10-10。範圍是讀取當前程式、既有報告、PX4 v1.17.0
原始碼及廠商文件。尚未編譯、運行 PX4 閉環、刷機或修改真機參數。

## 建議與結論

可保留現有 MuJoCo 場地、機庫、180 g 暫定物理模型、任務評分器及研究策略，
在根層新增 PX4 後端。先讓 PX4 在同一物理模型上完成單機起飛、懸停、短程
往返、降落停槳，再接無 GPS 估測和撞擊恢復。不要把更換韌體視為解決定位
偏差的證據，也不要將 ArduPilot 的歷史通過數轉記成 PX4 成績。

真機仍採 MicoAir743v2-AIO-35A、MTF-02P、僅取羅盤的 M10G-5883、
M3C + OS04A10。廠商明確指定這塊 v2 AIO 的 PX4 目標為
**`micoair_h743-v2_default`**，不是看到 AIO 名稱便選 `micoair_h743-aio`。
v1.17.0 上游有對應板級程式；實際板修訂、bootloader 及安裝 build 待確認。

## 已讀取的 ArduPilot 路徑

| 現有內容 | 實際責任 | PX4 遷移處理 |
|---|---|---|
| `rm27/autopilot/micro_flight.py` | 啟動 arducopter、參數讀回、GUIDED、起降與入位狀態機 | 新增獨立 PX4 launcher/runner；逐項移植物理門檻 |
| `rm27/autopilot/bridge/{protocol,bridge}.py` | SIM_JSON 二進位 PWM 輸出 → 一次物理步進 → JSON 狀態回覆 | 新增 PX4 MAVLink 仿真橋接，不能直接連用 |
| `rm27/autopilot/bridge/frames.py` | NWU→NED、FLU→FRD、四元數與比力 | 數學可重用；名稱及介面整理，增加 PX4 對照驗證 |
| `rm27/simulation/micro_plant.py` | 同一世界中選中一架；馬達映射、物理狀態及碰撞 | 保留物理模型；將飛控輸出解碼與物理步進分開 |
| `configs/micro_hover.parm`、`bridge/hover.parm` | AP 馬達、EKF3、PID、Guided 及降落設定 | 新建 PX4 仿真配置，不按名稱替換或直接複製數值 |
| `aggressive_contact/flight_control/sitl.py` | 真實 AP EKF/PID/mixer 與接觸恢復研究 | 新建 PX4 實驗入口，保留共同 plant/evaluator |
| `flight_control/estimator.py` | 位置/姿態遙測時間對齊、原點註冊、資料年齡 | 重用合約；重新驗證 PX4 時鐘、重置及有效性資訊 |
| `flight_control/diagnostics.py` | 合成外部導航、特權真值診斷 | 改寫 PX4 量測封包；維持 actor 與真值隔離 |
| `distillation/deployment.py`、`teacher_interface.py` | timestamped estimate → 策略 → 角速度與物理推力 | 策略合約可保留；新增 PX4Adapter，重新辨識推力與動態 |
| `rm27/experiments/log_intake.py` | `.BIN` DataFlash 唯讀清單 | 新增 `.ulg` ULog 路徑，現有命令不支援 ULog |

也讀取 MICRO_FLIGHT、MILESTONES、Phase 4/5 報告及其配置。
AP 原始碼 checkout 為 `96f98857f7fc4bd564fbbdc43c5f26114246369f`；
G1 submodule HEAD 為 `2369d7d207818160e2d140b16118a53f99491b66`。
兩者工作樹均已有修改，因此 hash 不是整個工作樹未修改的證明。
本次未改動這些第三方樹。

倉庫中的舊 PX4 工作區位於
`rm27_drones_legacy/rm_uav_lab/PX4-Autopilot`，HEAD 為
`dd0ad74fdadc68a62b479129ca3f382c006426f4`，提交描述是舊 C1/Gazebo
airframe 4022。其讀取時狀態輸出為空，但不能把它視為 v1.17.0 或當前
MuJoCo 模型的已驗收後端。舊 Gazebo C1 報告僅供參考。

## PX4 v1.17.0 核對結果

官方 tag 解析到 commit **`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`**。
以下程式判讀針對該 tag；官方文件和廠商頁面仍可能更新。

### 硬體與 MTF-02P

- [廠商 v2 AIO 手冊](https://micoair.cn/zh/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)
  指定 PX4 `micoair_h743-v2`；編譯名稱為 `micoair_h743-v2_default`。
- [v1.17.0 板級感測器啟動](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/boards/micoair/h743-v2/init/rc.board_sensors)
  包含 BMI088、BMI270、SPL06、QMC5883L，與目前硬體記錄相符。
  `firmware.prototype` 記錄 board ID 1179；這是上游目標資料，非本機 USB 實測。
- 不可套用另一個 `h743-aio` 的啟動設定：本次對照可見它使用 DPS310，
  BMI270 bus 也不同。板名相似並不代表可互刷。
- v2 預設包含 `BAT1_N_CELLS=4`、電壓/電流比例、rate gains、
  `CBRK_SUPPLY_CHK` 等設定。它們是板級預設，不能成為本機電池、校準或
  控制增益的實測值；目前電池節數仍未知。
- [MTF-02/02P 手冊](https://micoair.cn/zh/docs/sensors/sensors/mtf-02-02p-sensors)
  說明 Mavlink_px4 與 AUTO 模式。優先使用已有 MAVLink 路徑，接收
  `OPTICAL_FLOW_RAD` 和 `DISTANCE_SENSOR`，不必先寫自訂 Micolink 驅動。
  UART、供電、115200 baud、方向、品質及原始更新週期仍需驗證。
- 廠商提供的 1.17.x 說明另列 `MAV_PROTO_VER=1`。v1.17.0 參數定義中
  1 表示先使用 v1、偵測 v2 後自動升級；不是永久鎖定 v1。需要分別
  驗證 MTF 連線與 M3C 的 MAVLink 2 `ODOMETRY` 連線。
  QGC Inspector 有訊息不等於 EKF2 已接受並融合，地面站轉發頻率也不是
  感測器採樣頻率。仿真應驗證 uORB 資料及融合狀態；真機確認另需授權。
- 廠商建議 Bluejay 使用 DShot300，並說明不支援 PWM 訊號。
  現有 AP 仿真的整數 PWM 是軟體協議，不能當作真機 ESC 連線設定。

### MuJoCo ↔ PX4：需要新的仿真橋接

現在 AP 使用 UDP SIM_JSON：40/72-byte servo packet、frame_count、
1000–2000 PWM；每個新 frame 推進 MuJoCo 0.002 s，再回 JSON 狀態。
PX4 v1.17.0 的 `simulator_mavlink` 是另一套介面：

- 向 PX4 發送 `HIL_SENSOR` 等感測量測，接收 `HIL_ACTUATOR_CONTROLS`。
  HIL_* 是此 SITL 介面的訊息名稱，不代表本次已有硬體在迴路。
- `px4-rc.mavlinksim` 預設建立 TCP `4560 + instance` 的仿真連線；
  任務/GCS MAVLink 要另設端點，不能混同 AP 的 9002/5760 端口公式。
- PX4 仿真時鐘由感測資料推進，程式有 lockstep 排程。啟動時先送初始
  sensor frame，收到輸出後按明確時間協議推進；不能照搬 AP 的
  frame_count 去重或假設輸出與物理步長天然一一對應。
- `HIL_ACTUATOR_CONTROLS.controls` 來自 actuator_outputs；應查明所選
  仿真輸出驅動的範圍、通道功能、disarmed 值及推力曲線。不能套用
  AP 的 `[2,0,3,1]` 馬達映射或把數值直接當牛頓。
- 初期維持現有 500 Hz 物理步長。先驗證可用感測/控制更新率及同步，
  PX4 腳本的 IMU integration 預設為 250 Hz，不表示所有介面都必須 500 Hz。
- 先發現有感測模型能正確生成的資料；補足磁場、氣壓、溫度等訊息語義。
  無 GPS 階段停用 GPS 量測與融合；真值只做感測來源與獨立評分。

建議新增 `rm27/autopilot/px4/`，包含 launcher、simulator_bridge、
offboard、telemetry、配置核對。第三方 PX4 放在獨立且固定版本的依賴
checkout。這是建議佈局，本次尚未建立或安裝後端。

### GUIDED → Offboard：控制語義不只改模式名稱

[PX4 Offboard 文件](https://docs.px4.io/v1.17/en/flight_modes/offboard.html)
要求預先連續發送 setpoint 超過一秒，持續速率 >2 Hz；一般 GCS heartbeat
不能替代。需要驗證正常進入 Offboard、解鎖、命令 ACK、模式確認、停止
setpoint 後的 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` 行為。

低速起降優先以 PX4 自身的位置/速度控制完成。現有 AP `GUID_OPTIONS=16`
外層真值補償並非可直接搬移的部署導航。入位精度不達門檻時記錄失敗，
不修改機庫或放寬門檻來宣稱遷移通過。

高動態策略目前輸出 FLU rad/s 和總推力 N。PX4 也接受
`SET_ATTITUDE_TARGET`，但要特別處理：

- 角速度換成 FRD：`[p,-q,-r]`；姿態仍是 NED←FRD 的 WXYZ 四元數。
  根層世界是 NWU，不要套一般 ROS ENU 的轉換公式。
- scalar `thrust` 為歸一化值，四旋翼接收端轉成 `thrust_body.z=-thrust`。
  物理 N→此值需獨立校準。現有 `SpinMapping` 是 AP spin range 的名義
  仿真反函數，不能作為 PX4 推力校準。
- **姿態+角速度前饋有差異**：v1.17.0 `mavlink_receiver.cpp` 姿態分支
  只將 yaw rate 設為 `yaw_sp_move_rate`；roll/pitch rate 並沒有作為該
  姿態分支的同等前饋。因此不能假設 Phase 4/5 的姿態+三軸 rate 封包
  在 PX4 的行為相同。
- 初期比較「純姿態+推力」與「角速度+推力（ignore attitude）」兩條
  介面。若研究確實需要姿態加完整三軸前饋，再評估 uORB/ROS 2 或飛控內
  專用介面；先不為此移植整套 ROS 2。
- roll/pitch/yaw 的階躍測試、飽和、推力及懸停測試先通過，再評估撞擊。
  `ATC_*`、`GUID_OPTIONS`、`MOT_*` 與 PX4 `MC_*`/控制分配器的數值
  沒有直接可複製的對應關係。

### EKF3 → EKF2：目前速度量測封包不能沿用

Phase 5 的 AP 合成外部導航每秒 50 次發送 `VISION_POSITION_ESTIMATE`
及 `VISION_SPEED_ESTIMATE`。官方 PX4 文件明確列出不支援後者，
v1.17.0 MAVLink receiver 亦沒有對應接收分支。

建議新後端用 **MAVLink 2 `ODOMETRY`** 送 pose、velocity、時間戳、
協方差、quality 及 reset counter。選定並固定 frame_id/child_frame_id；
LOCAL_FRD 是固定的局部參考系，BODY_FRD 是旋轉機體系，兩者不能混用。
確保姿態、位置、速度、相機外參與目標註冊指向同一套座標語義。

`EKF2_EV_CTRL`、`EKF2_EV_DELAY`、`EKF2_EV_POS_*`、`EKF2_HGT_REF`
需按量測來源另設；GPS 融合需按任務關閉。先測合成外部導航與 PX4 EKF2，
再加入 MTF 光流/測距模型，最後用 M3C 真實或錄製量測。目標燈/裝甲
辨識結果不是自動可用的完整 ego odometry。

保留 source/receipt 時鐘、TIME_SYNC、重置與掉幀資訊，避免將 held sample
重標成新量測。官方建議外部視覺約 30–50 Hz；本項目可先以 50 Hz
驗證。碰撞前後的資料年齡、速度和姿態持續偏差需單獨評分。

Phase 5 報告的 9/12 是 AP 閉環、合成外部導航下的接觸恢復結果，並非
相機/VIO 或返庫成功。轉 PX4 後必須重新評分，不能繼承該比例。

### M3C 與日誌

M3C 初期走 MAVLink，PX4 保留內部 EKF2、姿態/rate PID 與馬達控制分配。
M3C 負責視覺、任務與外部引導。既有策略的桌面執行時間不代表 M3C 效能；
需實測相機採樣→辨識/定位→傳輸→飛控融合/輸出的延遲、抖動與掉包。
能否運行目前 Python/SciPy 策略亦未驗證。

新增 ULog 唯讀分析：保存原 `.ulg` SHA-256、全部參數、PX4 build、
板級資訊、IMU/flow/range、估測 innovation、setpoint、actuator outputs、
飽和、模式及 failsafe。原 DataFlash 分析仍供 AP 歷史資料使用。

## 建議執行順序與交付物

1. **固定依賴**：獨立 PX4 v1.17.0 checkout，記錄 tag/commit/差異；
   建立可重複 SITL build 和獨立參數/日誌目錄。真機目標以 v2 手冊為準。
2. **橋接先驗收**：同一 MuJoCo plant 的感測單位、重力符號、姿態、
   馬達位置/轉向/力矩、輸出範圍、disarm zero force、lockstep 與缺包處理。
3. **PX4 S0**：headless 閉環完成起飛、10 s 懸停、0.12 m 往返、
   物理降落與自動停槳；用 MICRO_FLIGHT 原有門檻重跑 10 次獨立啟動。
   保存全程 output/sensor/physics 和 `summary.json`；必須 `passed:true`。
   錄製輸入回放與獨立閉環成功分開報告。
4. **無 GPS 估測**：合成 ODOMETRY→EKF2→Offboard；再測 MTF 數值模型，
   包含 delay、bias、品質下降、reset 與斷流。入位真值研究和 sensor-only
   任務分別記錄，後者不得隱含真值補償。
5. **高動態介面**：以相同參考與 plant 比較姿態/角速度兩路，完成階躍、
   飽和與中止測試，再重跑接觸恢復。新的 PID/推力配置要有自己的證據。
6. **真實視覺與完整往返**：同步 ego/target 觀測、回庫視野與失效策略，
   再依 MILESTONES S2–S5 完整驗收。先單機，多機仍延後。

第一個具體開發交付物應是 **PX4 v1.17.0 控制當前 MuJoCo 飛機完成 S0**。
硬體支援核對可與仿真並行；未知電池、安裝方向和 bootloader 不妨礙
先做軟體後端，但會阻止將仿真配置稱為可直接上機的配置。

## 證據與尚未完成的驗證

本次抓取來源和 SHA-256 清單在
`artifacts/px4-migration-research-20261010-01/sources.json`，本地程式/配置
hash 及 root/nested Git 狀態在同目錄 `local_inputs.json`。
來源快照是本地生成資料，不納入 Git。初次不存在的 `rc.simulator` URL
已由實際 `px4-rc.simulator`/`px4-rc.mavlinksim` 來源補足。

參考：[PX4 v1.17.0 原始碼](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0)、
[MAVLink receiver](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/mavlink_receiver.cpp)、
[simulation bridge](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)、
[外部導航文件](https://docs.px4.io/v1.17/en/ros/external_position_estimation.html)。

本次驗證類型：**靜態程式/文件研究**。沒有新 headless 飛行、live-view、
PX4 closed-loop SITL、硬體試飛或 recorded-input replay 結果。
待確認：實機板修訂、installed build/bootloader、電池、ESC、端口與安裝
方向、MTF 模式、M3C 傳輸及實際性能。這些未知量不改記為量測。
