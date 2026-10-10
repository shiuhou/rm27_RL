# RM27：从 SLAM 到 VINS / VIO 的计划索引

整理日期：2026-10-11。两版完整计划已保留在 Git 中；本页建立版本关系和接续
入口，不复制第二份计划，不改原门槛，也不把路线图中的目标写成完成结果。

## 1. 早期 SLAM / 单目定位路线（历史原文）

完整原文：[M3C＋OS04A10 单目定位实验路线 v1.0](../M3C_OS04A10_SLAM_实验起步包_v1/M3C_SLAM_experiment_plan_v1/M3C_OS04A10_单目定位实验路线_v1.md)。
原文日期 **2026-09-20**，随仓库提取提交 `bd25acf` 已纳入版本管理。

主要路线是目标相机成像 → 标定/固定数据集 → PC 算法基线 → 同算法板端回放
→ 板端实时处理 → 米制定位 → 飞控接口 → 低风险飞行，按 **VSL-0～VSL-8** 分阶段。
stella_vslam 是第一条纯单目几何基线，ORB-SLAM3 作几何/完整 SLAM 对照。
旧计划已经列 OpenVINS 单目＋IMU 为 VIO 候选，并要求可靠 IMU、时间与外参；
转向并不是从“完全没有 IMU 的计划”另起一个工程。

这是拟执行协议。产品规格假设、旧飞控集成选择、开发性能目标和阶段列表不等于
当前实机事实或通过记录。原文和配套文件保留，不用新的 PX4 状态改写旧实验历史。
相关早期协议/结果可从 [SLAM_RECOVERY_AND_AUDIT](SLAM_RECOVERY_AND_AUDIT.md)、
[VSL-3 实验框架](VSL-3_EXPERIMENT_HARNESS.md)、
[VSL-4 板端测试计划](VSL-4_M3C_BENCHMARK_PLAN.md) 接续。

## 2. 后来转向的轻量 VIO 新计划（当前研究路线）

完整原文：[new_plan.md — RM27 轻量级 VIO 与 GPS-Denied 定位研究方向重构报告](../../../../new_plan.md)。
原文 v1.0 日期 **2026-10-07**，顶部保留后续进度通知；于提交 `4dba7fd` 发布到 GitHub。

新计划依据保存的板端稀疏前端测试和实际任务需求，优先解决有界状态估计，
不把完整建图、优化、回环全都作为最小机载链路的前置要求。
这些前端测试不是完整 SLAM/VIO 在 M3C 上的性能资格证明。

目标链路：Camera＋IMU → lightweight VIO → 既有 LocalizationEstimate adapter
→ PX4；后面的外部视觉融合和 GPS-denied 飞行仍是未执行的未来 gate。
原文也保留 ArduPilot adapter 方向，不表示目前更换飞控或运行两套融合。

| 新计划中的候选 | 计划角色，不是全部已实现/验证 |
|---|---|
| Flow＋ToF | Safety baseline |
| VINS-Fusion | PC optimization/reference baseline |
| OpenVINS | Embedded baseline；当前已开展的主要 VIO 软件研究 |
| sqrtVINS | Embedded challenger；仍是研究候选，不宣称 M3C 已达标 |
| ORB-SLAM3 | Full-SLAM reference；有证据需要回环/重定位/地图能力时再提升优先级 |

完整采集、时钟/空间标定、噪声、公共数据复现、RM27 replay、M3C 实时、autopilot
和飞行要求均以新计划原文为准。第 21 节的 vision 20–30 Hz、IMU 200–400+ Hz
是目标，不是当前测量值。第 40 节 G0～G6 中的 PASS 字样描述应达到的 gate，
不能据此宣布全已完成；也不要把路线阶段与后来链路实验的 Stage A/B/C 混号。

## 3. 当前执行位置与证据优先级

先读 [CODEX_RESUME](CODEX_RESUME.md) 和根 [HANDOFF](../../../../HANDOFF.md) 的最新段落，
再看 [接线/接口](PX4_M3C_CONNECTION.md) 和 [主机/部署位置](WORKSPACE_LOCATIONS.md)。
这些是当前约束及可接续工作入口，不改变原计划的验收标准。

- **Stage A/B VERIFIED**：当前 d6f12ad1 的 MAVLink2 HEARTBEAT、合成 ODOMETRY
  → PX4 uORB，含停/启来源归因。仅 transport/parser/uORB，不是真实 VIO 或融合。
- **VIO-P PARTIAL**：保存数据的可靠输入 candidate 与既有 adapter 有有限成功证据，
  历史失败保留，不声称普遍确定性、M3C 实时或全部候选复现通过。
- **VIO-S0 PARTIAL**：高频完整选中 BMI088 输入、观测语义和相机时钟仍未资格化。
  已有有限离线重组器，live single-owner start/ACK/stop/timeout/restore controller 尚待实现。
- **Stage C / 真 RM27 VIO / 空间时间标定 / EKF 融合 / 飞行未开始**。
  保持 `EKF2_EV_CTRL=0`，不 ARM；本次整理计划没有批准新设备操作。

没有第二组可用 UART；早期备用焊盘/独立 UART 建议已被用户约束取代。
不要按历史计划重新找接口或无授权改 baud、固件、参数、接线。
新计划中的远期任务仍是计划，不自动成为当前任务授权。

新旧原文均未因本页被改写；本次没有新运行/硬件测试或 Vault 写入。
