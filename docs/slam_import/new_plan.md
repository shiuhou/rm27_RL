# RM27 轻量级 VIO 与 GPS-Denied 定位研究方向重构报告

> 2026-10-11 Git 交接入口：`rm27/perception/vision/docs/CODEX_RESUME.md` 与
> `PX4_M3C_CONNECTION.md`。已发布的研究子集见 `.maixpy/README.md`；原始日志/
> 数据集仍在原实验主机。本计划门槛不变；下列早期“备用焊盘”下一步已被用户
> 确认无第二可用 UART 的约束取代。当前先完成 single-owner 会话控制的离线测试。

> 2026-10-11 離線重組完成：見
> `.maixpy/imu-design-20261010/single-uart/reassembly/VALIDATION.md`。
> 28項新測試及19項既有回歸通過；兩份保存ULog經合成MAVLink封裝重播後
> 逐位元組／SHA一致，391與306次dropout保留。封包完整不代表上游樣本完整，
> 積分資料仍未通過OpenVINS模型／時鐘門檻。下一軟體工作為限時會話控制與
> 還原測試；試驗提案已寫明，未改baud或開始實機串流，Stage C仍關閉。

> 2026-10-10 單UART約束更新：用戶確認沒有第二組可用UART，不再要求載板焊盤核對。
> 見`.maixpy/imu-design-20261010/single-uart/REVIEW.md`。先評估現成MAVLink ULog
> 完整記錄串流作採集原型，再決定是否需要自訂韌體；不是直接餵OpenVINS。
> 200Hz IMU-only線速模型13.94kB/s，115200不足；六組離線封包／計算檢查通過。
> 未改baud、參數、韌體或接線，未實機採集。原門檻、Stage A/B與Stage C邊界不變。

> 2026-10-10 盤點後離線補充：見`.maixpy/imu-design-20261010/followup/REPORT.md`。
> DDS時間同步含同步等待，queue1完整性不能只靠提高匯出頻率；尚無DDS丟包實測。
> 200Hz加身份／時鐘狀態預算最低16.78kB/s；400Hz保守轉義上限64.74kB/s。
> 92項相關離線測試通過，積分平均值仍不能直接等同瞬時輸入。USB備選存在
> 已確認的host驅動／管理連線限制；下一步僅提請批准斷電載板接口核對。
> 原始門檻、Stage A/B及EV_CTRL0約束保留；未接觸硬件或啟動Stage C。

> 2026-10-10 接口只讀盤點：飛控d6f12ad1／Disarmed／EV_CTRL0已重查。
> M3C USB正提供SSH管理網路，內核CONFIG_USB_ACM未啟用，不能視為即插即用
> CDC-host備選。ttyS4被launcher使用；獨立UART仍須設計者確認載板焊盤。
> 見`.maixpy/interface-inventory-20261010/VALIDATION.md`。未改任何設定／接線、
> 未採集、未啟動Stage C；VIO-S0仍PARTIAL。

> 2026-10-10 純離線輸入設計：詳見
> `.maixpy/imu-design-20261010/VIO_S0_INPUT_DESIGN.md`。vehicle_imu200Hz連同
> XRCE串口封裝最低15.8kB/s，115200不足；DDS預設10ms且未匯出vehicle_imu。
> 優先評估獨立高速UART/DDS保留完整積分與身份，但積分平均值仍不能直接
> 當OpenVINS瞬時樣本；模型不成立則評估全速SI雙流／USB與明確前處理。
> 100項相關離線測試通過、2項Linux測試跳過；沒有硬件存取或設定變更。
> VIO-S0仍PARTIAL，原門檻不變，Stage A/B保留、Stage C未開始。

> 2026-10-10 Track A/B 補充：已定位10Hz為MAVLink總發送預算縮頻。
> 同115200／5760B/s，暫減其他遙測後，50Hz請求實收50.001Hz；100Hz請求
> 實收62.112Hz，並非100Hz PASS。設定已還原。啟動無效位元組保留，不宣稱
> 零錯誤。vehicle_imu約198Hz但屬積分／圓錐補償／校正資料；預設DDS
> sensor_combined為10ms訂閱且缺硬件ID。下一方案須批准通訊／匯出變更，
> 原200–400+Hz、資料語義及同步門檻不放寬。詳見
> `.maixpy/imu-transport-20261010/VALIDATION.md`；Stage C仍未開始。

> 2026-10-10 進度補充（原目標／門檻不變）：Stage A/B MAVLink2 + 合成
> ODOMETRY -> PX4 uORB 已驗證；真實 PX4 -> M3C 被動測試收到303筆
> HIGHRES_IMU，約10.11Hz。這是積分／校正／偏置處理後遙測，未達第21節
> 200–400+Hz目標，不是原始或同步VIO資料。見
> `.maixpy/imu-input-20261010/IMU_INPUT_VALIDATION.md`。Stage C 保留為後續
> 真實VIO -> PX4驗證，尚未執行；EKF2_EV_CTRL維持0。下一步須決定輸入
> 語義與批准通訊方案，不能因鏈路通了而跳過感測器／時間／標定門檻。

**版本：v1.0**  
**日期：2026-10-07**  
**适用项目：RM27 Micro UAV / M3C + OS04A10 / GPS-Denied Autonomous Flight**

---

# 摘要

RM27 当前的视觉定位研究已经完成了相机数据链检查、相机几何标定、Stella VSLAM 与 ORB-SLAM3 后端复现、online pose/tracking-state instrumentation，以及 M3C 板端稀疏视觉前端初步性能测试。

现阶段最重要的新事实是：M3C 在当前 CPU 实现下，即使仅运行简化的稀疏 KLT 视觉前端，在 `336×190 / 180 features / 1 thread` 条件下也仅达到约 **29 FPS**；`672×380 / 400 features` 约为 **10 FPS**。这些测试尚未加入完整 SLAM 所需的 pose estimation、keyframe、mapping、optimization 与 loop closure。

因此，继续以“将完整 monocular SLAM 部署到 M3C”为下一阶段主目标，很可能投入大量工程资源，却解决了当前 RM27 并不一定需要的问题。

本报告将研究方向重构为：

> **优先建立 Camera + IMU → Lightweight VIO → LocalizationEstimate → PX4/ArduPilot → GPS-denied flight 的最小可用闭环，再根据真实飞行中的 drift、tracking loss、relocalization 和算力问题决定是否增加完整 SLAM。**

新的核心研究问题不再是：

> “M3C 能否运行完整 SLAM？”

而是：

> **在 RM27 的重量、功耗、算力和已知室内赛场约束下，最少需要多少视觉惯性定位能力，才能支持可靠的自主飞行？**

该方向延续项目此前的 **Minimum Sufficient State Estimation** 思想：优先满足起飞、悬停、短距离自主导航、目标接近、返航与终端降落，而不是预设必须实现任意未知环境中的完整 3D SLAM。

---

# 1. 研究背景

RM27 空中机器人面临四项同时存在的限制：

1. **GPS-denied 室内环境**
2. **150–249 g 级微型飞行平台**
3. **严格的 Size / Weight / Power / Compute 限制**
4. **需要自主导航、目标定位和返回机库**

传统完整 SLAM 系统往往同时解决：

- visual odometry；
- mapping；
- loop closure；
- relocalization；
- map maintenance；
- long-term global consistency。

但 RM27 的实际比赛环境与未知森林、地下空间等典型 SLAM 场景存在本质差别：

> **赛场的绝大部分静态几何在比赛前可以获得。**

因此，项目过去已经提出利用 Prior Arena Map，把 online sensing 集中在机器人自身位姿、动态目标和临时障碍上，而不是在线重新重建整个静态赛场。

这意味着：

\[
\text{Full Online SLAM}
\]

并不是 RM27 自动成立的需求。

真正不可缺少的是：

\[
\text{Reliable Online State Estimation}
\]

即持续得到足够可靠的：

\[
p=(x,y,z)
\]

\[
v=(v_x,v_y,v_z)
\]

\[
R\ /\ q
\]

以及相应的：

- timestamp；
- tracking state；
- validity；
- covariance / uncertainty；
- reset/epoch semantics。

---

# 2. 当前项目基线

截至本报告重构时，项目已有以下基础。

| 项目 | 状态 |
|---|---|
| 旧 SLAM 工作整理 | ✅ |
| `LocalizationEstimate` 契约 | ✅ |
| M3C + OS04A10 真实视频数据 | ✅ |
| Camera geometric calibration | ✅ |
| Stella VSLAM PC 复现 | ✅ |
| ORB-SLAM3 PC 复现 | ✅，需保留 shutdown patch |
| Online pose / tracking-state instrumentation | ✅ |
| Online pose 与 final optimized trajectory 分离 | ✅ |
| M3C sparse frontend benchmark | ✅ 初步完成 |
| M3C 完整 SLAM | 未完成 |
| M3C 长时间 thermal / memory / latency | 未完成 |
| VIO | 未开始正式闭环 |
| PX4 / ArduPilot ExternalNav | 未开始正式闭环 |

当前 M3C 已知性能点：

| 输入 | Features | 当前性能 |
|---|---:|---:|
| 336×190 | 180 | ≈29 FPS |
| 672×380 | 400 | ≈10 FPS |

其中 29 FPS 仍然只是：

```text
grayscale image
↓
sparse visual frontend
↓
feature tracking
```

尚未包含完整状态估计后端。

因此：

> **“180 FPS Camera”与“180 FPS Localization”必须彻底解耦。**

Camera 可以高频采集，但视觉状态更新完全没有必要跟随 180 Hz。

---

# 3. 研究方向重构

## 3.1 旧路线

旧方向可以概括为：

```text
Camera qualification
↓
Camera calibration
↓
Stella reproduction
↓
ORB-SLAM3 reproduction
↓
online instrumentation
↓
RM dataset benchmark
↓
M3C full SLAM
↓
embedded optimization
↓
VIO
↓
ExternalNav
```

该路线的优势是证据严格。

但问题是：

> **Qualification infrastructure 被放在了最终 capability 之前。**

我们可能在还没有证明飞机真正需要完整 SLAM 的情况下，先花大量时间优化完整 SLAM。

---

# 4. 新路线

新的研究主线改为：

```text
Camera geometry
✅

↓


Camera + IMU
↓
Visual-Inertial Odometry
↓
LocalizationEstimate
↓
Autopilot adapter
↓
PX4 / ArduPilot estimator
↓
GPS-denied flight
↓
真实 mission benchmark
↓
根据 failure 再增加能力
```

核心原则：

> **Capability first, characterization second.**

即：

> 先建立一个能工作的 baseline，让真实系统暴露问题，再针对问题做实验。

而不是在系统飞起来之前预先把所有可能的 SLAM 能力做完。

---

# 5. 新研究问题

本项目正式研究问题定义为以下四个层次。

## RQ1 — Minimal Capability

> **RM27 的短距离 GPS-denied autonomous flight 是否仅依靠 lightweight VIO 就已经足够？**

---

## RQ2 — Embedded Feasibility

> **在 M3C 双核 A53 级算力下，可以实现什么视觉频率、状态更新率和定位精度？**

---

## RQ3 — Capability per Compute

> **不同 VIO architecture 在 accuracy / robustness / latency / memory / power 之间的 Pareto frontier 是什么？**

---

## RQ4 — When is SLAM Actually Necessary?

> **什么真实 failure 会使 RM27 必须从 VIO 升级到 loop closure / relocalization / full SLAM？**

---

# 6. 核心研究假设

## H1

对于约 5–15 m 级短时自主任务：

> Lightweight VIO + terminal absolute reference 已经足够完成主要 RM27 任务。

例如：

```text
VIO
+
AprilTag Dock
```

可能已经足够。

---

## H2

M3C 不需要处理 OS04A10 的全部 180 FPS。

更加合理的结构是：

```text
Camera
20–30 Hz visual update
        +
IMU
200–400+ Hz
        ↓
VIO
        ↓
continuous state estimate
```

---

## H3

完整 online SLAM 只有当：

- drift 无法接受；
- tracking loss 无法恢复；
- mission duration 增长；
- 必须 reuse prior map；
- 多 sortie 要共享一致 world frame；

时才值得增加。

---

## H4

最终方案评价指标不应该是：

\[
\min ATE
\]

而应该是：

\[
\max
\frac{\text{Useful Autonomy Capability}}
{\text{Weight + Power + Compute + Latency}}
\]

即 **capability per SWaP-C**。

---

# 7. 新系统架构

目标 architecture：

```text
                 OS04A10 Camera
                       │
                20–30 Hz images
                       │
                       ↓
                 VIO Frontend
                       ↑
                       │
                200–400 Hz IMU
                       │
                       ↓
                VIO Estimator
                       │
        ┌──────────────┼──────────────┐
        │              │              │
      pose          velocity       health
        │              │              │
        └──────────────┼──────────────┘
                       ↓
              LocalizationEstimate
                       │
               Autopilot Adapter
                       │
            ┌──────────┴──────────┐
            ↓                     ↓
           PX4                ArduPilot
          EKF2                  EKF3
            │                     │
            └──────────┬──────────┘
                       ↓
                  Flight Control
                       ↓
             GPS-denied autonomy
```

在定位失效或初始化失败时：

```text
Flow + ToF
```

仍应作为低空和安全 fallback。

项目过去已经明确把 Flow + ToF 保留为 VIO fallback，而不是在进入高级定位后删除。

---

# 8. 第一轮算法候选冻结

本阶段不继续无止境搜索算法。

第一轮只保留三个正式候选。

## 8.1 VINS-Fusion — Reference Baseline

角色：

> **成熟 optimization-based VIO reference。**

VINS-Fusion 官方将其定义为 optimization-based multi-sensor state estimator，支持 monocular camera + IMU、stereo + IMU，并提供 spatial calibration、temporal calibration 和 visual loop closure。

研究用途：

```text
VINS-Fusion
↓
回答：
“一个成熟 optimization VIO
在我们的 sensor 上应该达到什么水平？”
```

它是：

> **Reference**

而不默认是：

> **Final embedded implementation**

---

# 9. OpenVINS — Embedded Baseline

OpenVINS 使用：

```text
IMU
+
Sparse Visual Features
↓
EKF / MSCKF
```

其核心 estimator 是 Extended Kalman Filter，并通过 MSCKF sliding-window formulation 利用稀疏视觉 feature tracks 更新状态，而不需要把所有视觉 landmark 都永久加入滤波状态。

因此它非常适合作为：

> **M3C Embedded Baseline**

角色：

```text
成熟
+
结构清晰
+
Filter-based
+
已有 evaluation tools
```

---

# 10. sqrtVINS — Embedded Challenger

截至 2026 年目前最值得加入旧计划的新候选之一是：

**sqrtVINS**。

它建立在 OpenVINS 基础之上，但采用 square-root filter，并针对：

- numerical stability；
- memory efficiency；
- embedded platform；
- limited precision；

进行优化。

官方实现特别强调其可使用 32-bit single precision，并展示了在 Jetson Nano 5 W、单线程、低于 1 GHz CPU 环境下运行的案例。

需要注意：

> **Jetson Nano 的表现不能直接等价于 M3C。**

因此这只是很强的研究候选，而不是已经证明能在 M3C 达标。

角色定义：

> **Embedded Challenger**

---

# 11. ORB-SLAM3 新角色

ORB-SLAM3 不删除。

但从：

> 主线 blocker

降级为：

> **Full-SLAM Reference**

ORB-SLAM3 同时支持：

- Visual SLAM；
- Visual-Inertial SLAM；
- Multi-Map SLAM；
- monocular；
- stereo；
- RGB-D。

这意味着它解决的问题明显大于目前 RM27 的最小需求。

只有以下问题出现时，ORB-SLAM3 才重新升级优先级：

```text
严重长期 drift
tracking lost 后无法恢复
需要 prior map relocalization
需要 loop closure
需要跨 sortie map consistency
```

---

# 12. 候选体系最终冻结

第一轮：

| Track | 系统 | 角色 |
|---|---|---|
| B0 | Flow + ToF | Safety baseline |
| B1 | VINS-Fusion | Optimization reference |
| B2 | OpenVINS | Embedded baseline |
| B3 | sqrtVINS | Embedded challenger |
| B4 | ORB-SLAM3 | Full-SLAM reference |

现阶段不再新增主要候选。

SVO、Basalt、OKVIS 等可保留到第二轮，但不得阻塞 vertical slice。

---

# 13. Sensor Pipeline 重构

当前离线测试：

```text
MP4
↓
FFmpeg decode
↓
resize
↓
color conversion
↓
frontend
```

只适合 reproduction / benchmark。

真实系统应尽可能改成：

```text
OS04A10
↓
VIN / ISP
↓
Y / grayscale plane
↓
preallocated image buffer
↓
frame selector
↓
VIO frontend
```

目标：

减少：

```text
H.264 encode/decode
RGB conversion
malloc/free
memory copy
IPC pipe
```

这必须和 estimator algorithm 分开 benchmark。

否则：

> decode bottleneck 会被误认为 VIO bottleneck。

---

# 14. Visual Frequency 研究

原 Camera：

\[
f_c \approx180Hz
\]

不意味着：

\[
f_{VIO}=180Hz
\]

新的核心实验使用原 180 FPS 数据进行真实时间抽帧：

```text
180 Hz:
0,1,2,3,...

60 Hz:
0,3,6,9,...

30 Hz:
0,6,12,18,...

20 Hz:
0,9,18,27,...

15 Hz:
0,12,24,36,...
```

分别测量：

- track survival；
- average feature lifetime；
- geometric inliers；
- initialization success；
- tracking loss；
- pose smoothness；
- CPU；
- latency。

最终寻找：

\[
f_{\text{vision}}^*
\]

满足：

> 最低计算量下仍然保持足够可靠的视觉约束。

---

# 15. Resolution × Feature × FPS Pareto Search

不能只比较：

```text
336×190
vs
672×380
```

应该建立：

\[
Resolution
\times
FeatureCount
\times
VisualRate
\]

参数空间。

例如：

| Resolution | Features | Vision Rate |
|---|---:|---:|
| 336×190 | 100 | 30 Hz |
| 336×190 | 150 | 20 Hz |
| 480×270 | 120 | 20 Hz |
| 480×270 | 180 | 15 Hz |
| 672×380 | 100 | 15 Hz |

评价：

\[
Accuracy
\]

\[
Robustness
\]

\[
Latency
\]

\[
CPU
\]

最后得到：

> **M3C VIO Pareto frontier**

而不是单纯：

> 最高 FPS。

---

# 16. VIO 必要的新标定

Camera geometric calibration 已完成：

\[
K,D
\]

因此不再重复该工作。

但 VIO 新增：

## Camera–IMU Spatial Calibration

\[
T_{CI}
\]

必须确定：

- translation；
- rotation。

---

## Camera–IMU Temporal Calibration

需要估计：

\[
\Delta t_{CI}
\]

因为 VIO 中：

\[
\text{10–30 ms timing error}
\]

在快速运动条件下就可能变成显著状态误差。

VINS-Fusion 官方本身也提供 online spatial 与 temporal calibration，并明确指出 VIO 对硬件质量和同步非常敏感。

---

## IMU Noise Characterization

至少获得：

- gyro noise density；
- gyro random walk；
- accelerometer noise density；
- accelerometer random walk。

第一版可以使用静态数据估计，再逐步完善。

---

# 17. 新 Dataset

建立正式：

# `RM27_VIO_DATASET_v0`

每条数据必须包含：

```text
camera image
camera timestamp

gyro xyz
gyro timestamp

accelerometer xyz
accelerometer timestamp

camera intrinsics
camera distortion

T_camera_imu

sensor configuration
software revision
```

动作覆盖：

```text
static

slow translation
X / Y / Z

roll
pitch
yaw

combined 6DoF

fast yaw

fast translation

loop trajectory

low texture

motion blur
```

以后如果条件允许增加：

```text
Ground Truth
```

来源可包括：

- external tracking；
- Teacher LiDAR/LIO；
- accurately measured trajectory；
- other high-confidence reference。

---

# 18. Stage 0 — Public Dataset Reproduction

目标：

> 不修改算法，证明 software pipeline 正常。

首先：

```text
EuRoC
↓
VINS-Fusion
```

然后：

```text
EuRoC
↓
OpenVINS
```

再：

```text
EuRoC
↓
sqrtVINS
```

验收只需要：

```text
build PASS
initialize PASS
trajectory PASS
state output PASS
reproducible PASS
```

不要在这个阶段开始算法优化。

---

# 19. Stage 1 — RM27 VIO Replay

把自己的 Camera + IMU 数据输入：

```text
VINS-Fusion
OpenVINS
sqrtVINS
```

第一轮只看：

### Initialization

- success；
- time-to-initialize。

### Tracking

- tracking coverage；
- lost count；
- recovery。

### State

- trajectory sanity；
- velocity sanity；
- attitude sanity。

### Compute

- CPU；
- RAM；
- processing latency。

目标不是立刻选 winner。

而是：

> **筛掉明显不适合的方案。**

---

# 20. Stage 2 — M3C Embedded Benchmark

优先移植：

```text
OpenVINS
vs
sqrtVINS
```

VINS-Fusion主要保留 PC reference。

测试至少持续：

```text
10 min continuous operation
```

记录：

### Timing

\[
T_{frontend}
\]

\[
T_{propagation}
\]

\[
T_{update}
\]

\[
T_{total}
\]

并报告：

- mean；
- p50；
- p95；
- p99；
- maximum。

---

### Compute

记录：

```text
CPU %
RAM MB
thread utilization
```

---

### Runtime

记录：

```text
visual processed Hz
IMU input Hz
state output Hz
dropped frame
queue depth
```

---

### Thermal

记录：

```text
temperature
frequency throttling
```

---

### Stability

记录：

```text
crash
OOM
NaN
tracking loss
state reset
```

---

# 21. M3C 第一阶段目标

不要要求：

```text
180 Hz vision
```

初始目标建议：

```text
Vision:
20–30 Hz

IMU:
200–400+ Hz

Estimator:
continuous propagation

LocalizationEstimate:
≥30 Hz when practical
```

真正需要的视觉 FPS 要由实验决定，而不是提前固定。

---

# 22. PX4 作为第一 ExternalNav 集成路径

本报告建议：

> **PX4 作为第一条 integration reference，同时保持 ArduPilot adapter。**

理由不是 PX4 必然成为最终比赛飞控，而是 PX4 当前 External Vision pipeline 定义清晰。

PX4 支持通过 MAVLink `ODOMETRY` 将 external estimator 数据送入：

```text
vehicle_visual_odometry
↓
EKF2
```

并且 `ODOMETRY` 可以携带 linear velocity。

PX4 当前官方文档建议 external vision 消息保持在约 **30–50 Hz**；频率过低可能导致 EKF2 不进行融合。

因此 estimator architecture 可以是：

```text
Camera
20–30 Hz
     ↓
Visual update

IMU
200–400 Hz
     ↓
Propagation
     ↓
VIO state
30–50 Hz publish
     ↓
MAVLink ODOMETRY
     ↓
PX4 EKF2
```

这再次说明：

> **Camera FPS 和 external odometry publish rate 不必相同。**

---

# 23. PX4 第一阶段 Fusion 策略

第一阶段不建议一次把所有 VIO state 都交给 PX4。

先：

```text
XY position
XY velocity
```

高度仍使用：

```text
ToF / barometer
```

姿态仍由：

```text
PX4 onboard IMU
```

主要负责。

验证稳定以后，再逐步测试：

```text
VIO Z
VIO velocity Z
VIO yaw
```

PX4 的 `EKF2_EV_CTRL` 可以分别控制 external-vision position、velocity 和 yaw fusion；`EKF2_EV_DELAY` 用于处理 external estimator 相对 IMU 的时延。

---

# 24. Coordinate Frame Contract

这一项必须在进入 PX4 前冻结。

至少定义：

```text
camera frame
imu frame
VIO body frame
VIO world frame
airframe body frame
PX4 FRD
PX4 NED/local FRD
```

PX4 与 ROS/VIO 常用 frame convention 并不相同，官方文档明确要求对外部 pose 进行必要 frame transformation。

必须建立 unit test：

```text
forward movement
→ +X

right movement
→ +Y

up movement
→ expected sign

yaw clockwise / counterclockwise
→ expected quaternion
```

ExternalNav 上飞机之前必须 PASS。

---

# 25. Stage 3 — PX4 Replay Integration

先不要直接自由飞。

使用 recorded VIO：

```text
Recorded LocalizationEstimate
↓
adapter
↓
MAVLink ODOMETRY
↓
PX4 SITL
↓
EKF2
```

验证：

- timestamp；
- coordinate；
- orientation；
- velocity；
- stale detection；
- reset；
- tracking loss；
- estimator switch。

这一步可以把大部分 integration bug 在没有真实飞行风险的情况下消掉。

---

# 26. Stage 4 — Hardware-in-the-loop State Validation

下一阶段：

```text
Live Camera + IMU
↓
M3C VIO
↓
MAVLink
↓
真实飞控
```

飞机暂时不自主飞行。

比较：

```text
VIO pose
vs
PX4 EKF2 state
```

重点检查：

```text
innovation
timestamp
position direction
yaw
latency
reset
```

---

# 27. Stage 5 — Flight Validation

第一次 VIO 飞行不要直接：

```text
15m target mission
```

采用渐进式测试。

### F0

```text
Arm
Takeoff
Hover 10 s
Land
```

### F1

```text
Takeoff
Forward 1 m
Backward 1 m
Land
```

### F2

```text
2 m square
```

### F3

```text
5 m out-and-back
```

### F4

```text
10–15 m A → B
```

### F5

```text
A
↓
target initially invisible
↓
navigate into visual range
↓
visual terminal guidance
```

到 F5，RM27 的主要空间自主链已经形成。

---

# 28. Flight-Level 指标

算法指标不能替代飞行指标。

最终核心指标应该包括：

### Hover

\[
RMSE_{position}
\]

---

### Endpoint Error

\[
e=
\|p_{target}-p_{actual}\|
\]

---

### Return Error

\[
e_{return}=
\|p_{start}-p_{return}\|
\]

---

### Tracking

```text
tracking coverage %
lost count
longest lost duration
reset count
```

---

### Estimator Health

```text
innovation
pose jumps
stale events
EKF reject events
```

---

### Mission

```text
mission success %
```

这比单纯 ATE 更接近 RM27 的真实价值。

---

# 29. 算法评价指标

完整 benchmark 分五类。

## Accuracy

```text
ATE
RPE
drift / distance
```

---

## Robustness

```text
initialization success
tracking coverage
lost rate
recovery time
```

---

## Real-Time

```text
visual Hz
output Hz
mean latency
p95 latency
p99 latency
queue growth
```

---

## Compute

```text
CPU
RAM
power
temperature
```

---

## Mission Utility

```text
hover success
waypoint error
return error
mission success
```

最终不能只根据：

```text
ATE lowest
```

决定方案。

---

# 30. Failure Taxonomy

建立统一 failure 编码：

```text
F1 Initialization Failure

F2 Low-Texture Failure

F3 Motion-Blur Failure

F4 Fast-Yaw Failure

F5 Exposure Failure

F6 Dynamic-Object Corruption

F7 Long-Term Drift

F8 Relocalization Failure

F9 CPU Overload

F10 Queue / Latency Growth

F11 Memory Growth

F12 Thermal Throttling

F13 Timestamp Failure

F14 Coordinate-Frame Failure

F15 ExternalNav Fusion Failure
```

以后每个实验直接输出：

```text
OpenVINS:
F1 PASS
F3 WARN
F9 PASS
...

sqrtVINS:
...
```

这比一个综合分数更有工程意义。

---

# 31. 什么时候停止继续优化 VIO

这是新路线很重要的 Stop Rule。

如果：

```text
10–15 m mission
+
return
+
target approach
```

已经能够达到：

- 稳定 tracking；
- 可接受返航误差；
- 实时运行；
- 无 queue accumulation；
- M3C thermal 稳定；
- mission success 达标；

那么：

> **停止增加 localization complexity。**

即使 ORB-SLAM3 能得到更漂亮的地图，也不继续投入。

---

# 32. 什么时候升级到 Absolute Correction

如果发现：

```text
短期 VIO 好
长距离 drift 不够好
```

优先增加：

```text
AprilTag
```

或：

```text
UWB
```

而不是直接升级 full SLAM。

结构：

```text
VIO
↓
continuous relative motion
        ↑
        │
AprilTag / UWB
↓
occasional absolute correction
```

这是比完整 SLAM 更符合 RM27 已知赛场条件的路线。

---

# 33. 什么时候升级到 Full SLAM

只有出现以下需求之一：

1. tracking loss 后必须在未知位置恢复；
2. 必须长期保持 map consistency；
3. prior map relocalization 成为核心能力；
4. mission 长度导致 drift 无法通过 sparse anchor 解决；
5. 多次 sortie 需要共享 persistent map；
6. dynamic mission 真正要求 online map update。

此时才重新启用：

```text
ORB-SLAM3
```

等 full SLAM architecture。

---

# 34. Teacher Platform 与 Micro Platform 分离

最终仍保留两套角色。

## Teacher Platform

允许使用：

```text
MID360
strong computer
camera
IMU
```

运行：

```text
FAST-LIO
LIO/VIO
ORB-SLAM3
high-quality reference localization
```

目的：

> 为 micro VIO 提供 ground/reference。

---

## Micro Platform

最终目标：

```text
Camera
+
IMU
+
Flow
+
ToF
+
lightweight VIO
```

原则仍然是：

> **重型系统负责告诉我们“正确答案是什么”，轻量系统负责真正上比赛机。**

---

# 35. 当前不研究的内容

为避免再次发散，以下项目不进入本阶段 P0：

```text
Dense semantic SLAM

full 3D point-cloud mapping

NeRF / Gaussian-Splatting localization

end-to-end learned odometry

NPU-based learned VIO

multi-UAV distributed SLAM

full map sharing

event-camera VIO

advanced loop-closure research
```

不是这些方向没有价值。

而是：

> **当前没有证据证明它们是 RM27 的 blocker。**

---

# 36. 关于 NPU

M3C 即使具有 AI accelerator，也不应直接假设传统 VIO 可以靠 NPU解决。

当前候选的主要计算包括：

```text
KLT / feature tracking
geometry
triangulation
matrix operations
EKF / MSCKF
nonlinear optimization
```

这些并不是典型 CNN workload。

因此第一阶段优化重点应是：

```text
NEON
buffer reuse
zero-copy
memory layout
feature count
image resolution
pyramid levels
compiler optimization
threading
```

只有未来切换 learned feature / learned VIO 时，再单独评估 NPU。

---

# 37. 研究成果形态

这条方向最终应该产出四种成果。

## A. Engineering

```text
M3C real-time lightweight VIO
```

---

## B. Dataset

```text
RM27-VIO-Dataset
```

---

## C. Benchmark

```text
VINS-Fusion
vs
OpenVINS
vs
sqrtVINS
```

在 micro-compute 条件下的：

```text
accuracy
robustness
latency
compute
mission utility
```

---

## D. Flight System

```text
Camera + IMU
↓
M3C VIO
↓
PX4 / ArduPilot
↓
GPS-denied autonomous flight
```

---

# 38. 潜在科研问题

如果系统复现完成以后需要进一步发展为论文级课题，我认为最值得发展的不是：

> “我们又实现了一个 VIO。”

而是：

## Direction A — Minimum Sufficient Localization

> **微型无人机完成实际空间自主任务到底需要多少状态估计能力？**

比较：

```text
Flow+ToF
↓
VIO
↓
VIO+Anchor
↓
VIO+Relocalization
↓
Full SLAM
```

寻找 mission success 的能力饱和点。

---

## Direction B — Compute-Adaptive VIO

让系统根据：

```text
motion
texture
CPU
tracking confidence
```

动态改变：

```text
visual FPS
resolution
feature count
```

例如：

```text
Hover
→ 15 Hz

normal flight
→ 20 Hz

fast yaw
→ 30 Hz
```

这比始终固定 30 Hz 更符合 micro UAV。

---

## Direction C — Failure-Aware Estimation

根据 tracking health：

```text
NORMAL
DEGRADED
LOST
```

动态切换：

```text
VIO
Flow
ToF
AprilTag
```

并向 autopilot 输出明确的 uncertainty。

这对于真实无人机比单纯追求更低 ATE 更有价值。

---

# 39. 研究主假设可正式写成

> **H-RM27-VIO：在已知室内竞赛环境中，基于低频视觉更新、高频惯性传播、轻量级视觉惯性状态估计以及稀疏绝对校正的定位架构，可以在显著低于完整 online SLAM 的计算与功耗成本下，为微型无人机提供足以支持自主导航、返航和目标接近的空间状态。**

它可以进一步被实验拆解验证。

---

# 40. 新开发 Gate

最终把整条路线压成六个 Gate。

## G0 — Sensor Correctness

```text
Camera intrinsics ✅
Camera–IMU extrinsics
Time synchronization
IMU characterization
```

---

## G1 — VIO Reproduction

```text
EuRoC
↓
VINS-Fusion
OpenVINS
sqrtVINS
```

PASS。

---

## G2 — RM27 Replay

```text
RM27 Camera + IMU
↓
3 VIO backends
```

得到稳定 localization。

---

## G3 — M3C Real-Time

```text
OpenVINS / sqrtVINS
↓
M3C
```

满足：

```text
no queue growth
acceptable latency
stable memory
stable thermal
```

---

## G4 — Autopilot Integration

```text
LocalizationEstimate
↓
ODOMETRY
↓
PX4 EKF2
```

PASS。

---

## G5 — Flight

完成：

```text
hover
↓
1 m
↓
square
↓
5 m out-and-back
↓
10–15 m navigation
```

---

## G6 — Necessity Test

问：

> **现在到底还缺什么？**

如果答案不是：

```text
loop closure
relocalization
mapping
```

就：

> **不进入 full SLAM。**

---

# 41. 推荐软件工程结构

建议保留现有 RM27 repo，但把定位研究重新组织：

```text
localization/
│
├── contracts/
│   └── localization_estimate
│
├── datasets/
│
├── calibration/
│
├── backends/
│   ├── vins_fusion/
│   ├── openvins/
│   ├── sqrtvins/
│   └── orb_slam3_reference/
│
├── adapters/
│   ├── px4/
│   └── ardupilot/
│
├── benchmark/
│   ├── accuracy/
│   ├── runtime/
│   └── flight/
│
├── m3c/
│
└── docs/
```

重要原则：

> 上游 estimator 与下游 autopilot adapter 分离。

这样最终从：

```text
PX4
```

换回：

```text
ArduPilot
```

不会要求重新修改 VIO core。

---

# 42. Codex 开发方式

本阶段不要一开始就 Ultra + 多 Agent。

建议：

```text
Codex
Plan Mode
High
Single main agent
```

先完成：

```text
G1 → G2
```

尤其是：

```text
dataset contract
calibration contract
backend wrapper
common evaluator
```

当 OpenVINS 与 sqrtVINS 进入真正并行 M3C port 时，再：

```text
Main Agent
│
├── Worktree A → OpenVINS
└── Worktree B → sqrtVINS
```

两个 Agent 不修改共同 evaluator。

共同接口先由 main/base branch 冻结。

最后由 Main Agent：

```text
merge
benchmark
integration
validation
```

---

# 43. 近期具体执行顺序

新的实际执行顺序冻结为：

```text
Camera geometric calibration
✅

↓

确认 IMU 数据源
↓

Camera–IMU timestamp contract
↓

Camera–IMU extrinsic calibration
↓

EuRoC → VINS-Fusion
↓

EuRoC → OpenVINS
↓

EuRoC → sqrtVINS
↓

录 RM27-VIO dataset
↓

3 backend replay
↓

筛选 embedded candidate
↓

M3C port
↓

resolution × FPS × feature benchmark
↓

LocalizationEstimate
↓

PX4 ODOMETRY
↓

EKF2
↓

Hover
↓

1 m
↓

Square
↓

5 m out-and-back
↓

10–15 m mission
↓

判断是否真的需要 SLAM
```

---

# 44. 最终判断

RM27 当前已经没有必要继续把：

> **“完整 SLAM 是否能塞进 M3C”**

作为最主要问题。

新的核心问题应该是：

> **“M3C 能否以足够低的算力和延迟产生可供飞控真正使用的 VIO state？”**

如果答案是 Yes：

```text
继续优化 lightweight VIO
↓
飞起来
```

如果答案是：

```text
VIO本身够
但长期 drift 不够
```

加入：

```text
AprilTag / UWB / prior-map anchor
```

如果答案是：

```text
tracking loss 后无法恢复
```

才增加：

```text
relocalization
```

如果答案最终是：

```text
任务真的需要 persistent mapping
+
loop closure
+
map reuse
```

才重新把：

```text
ORB-SLAM3 / full VI-SLAM
```

升级为主线。

---

# 45. 一句话定义新的研究方向

> **RM27 不再以“复现完整 SLAM”为目标，而是研究如何在极低 SWaP-C 的微型无人机平台上，通过低频视觉、高频惯性、轻量 VIO 和必要时的稀疏绝对校正，实现足以支持真实 GPS-denied 自主飞行的最小定位能力。**

这个方向比“把 ORB-SLAM3 塞进 M3C”更贴近 RM27，也更容易形成一个真正可验证、可优化、最终甚至可以发展成科研问题的技术主线。
