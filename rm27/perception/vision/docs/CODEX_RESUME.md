# RM27 VIO 接续入口 — 2026-10-11

本文件与根 [HANDOFF.md](../../../../HANDOFF.md) 的最新段落优先于历史记录中的
“CURRENT / Next”。用户要求发布交接资料；没有授权本次改变任何硬件配置。

## 阅读顺序

1. [PX4 ↔ M3C 接线、端口、命令与边界](PX4_M3C_CONNECTION.md)。
   同时读 [SSH 主机、板端部署与本地资料位置](WORKSPACE_LOCATIONS.md)，不要混淆
   Linux 工作树、外部实验目录、M3C 部署与 GitHub 发布子集。
2. 根 [new_plan.md](../../../../new_plan.md)：原 VIO 目标和验收门槛不放宽。
   [计划历史索引](PLAN_HISTORY.md) 同时链接早期 SLAM 原文，并说明 VINS-Fusion /
   OpenVINS / sqrtVINS 的计划角色，不表示这些候选全已验证。
3. [Stage A/B](../../../../.maixpy/mavlink2-20261010/VALIDATION.md) 和
   [IMU 50/100 Hz 测量](../../../../.maixpy/imu-transport-20261010/VALIDATION.md)。
4. [最新单 UART 设计](../../../../.maixpy/imu-design-20261010/LATEST.md) →
   REVIEW → reassembly/VALIDATION → PROPOSED_TRIAL（提案，不是已执行实验）。
5. [离线 VIO / OpenVINS 进展](../../../../.maixpy/offline-vio-20261009/OFFLINE_VALIDATION.md)。
6. [已发布文件与离线测试入口](../../../../.maixpy/README.md)。

## 状态与下一工作

| Gate | 状态 | 意义 |
|---|---|---|
| Stage A/B | VERIFIED / PASS | 当前 d6f12ad1 上 MAVLink2、HB、合成 ODOMETRY、uORB 和停/启归因；不重做 |
| VIO-S0 | PARTIAL | 50 Hz transport 测得；完整高频选中 BMI088 输入、时钟、观测模型尚未合格 |
| VIO-P | PARTIAL | 保存数据 finite 五次 candidate 同轨迹，完整 adapter 原 gate 通过；不是普遍确定性保证 |
| ULog 离线重组 | VERIFIED | fixture / 保存日志合成封装逐字节一致；没有 live UART streaming PASS |
| Stage C / real RM27 VIO | NOT_STARTED | 禁止拿合成 ODOMETRY 或诊断均值绕过真实输入、标定与同步门槛 |

目前可继续的软件任务：基于已存在的 receiver/decoder/reassembler，实现并离线
测试 single-owner、fresh-disarmed-heartbeat、start/ACK/stop、startup/tail 完整性、
限时退出及精确还原。不要另造通用框架，不要把缺失记录插值修好后报 PASS。

实机前必须有该 controller 和还原测试，再按 PROPOSED_TRIAL 的具体范围取得
通信/logger 变更授权。用户已否定第二个可用 UART；不要重新要求找备用焊盘。
USB host/CDC 路径仍受内核、角色、SSH 管理条件限制。

## 证据与源码

PX4 当前已核实 hash：`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
后续源码必须匹配此 hash，不能用标签或历史 micoair-v1.15.2 偷换。
[精确版本源码入口](https://github.com/PX4/PX4-Autopilot/tree/d6f12ad1c4f70ad3230afd7d86e971421e02fef4)。
本仓库保留小型 `.msg` 定义和研究脚本依赖，未发布整个第三方源码 / 构建树。

这些报告是保存观察的总结。本次推送不重新观测 Disarmed、EV_CTRL 或设备连接。
实际硬件任务开始前要重新核对固件、Disarmed、EV_CTRL=0、所有权与原始配置。
不自动恢复旧 SD 卡 / 旧固件参数，不动用户其他飞控参数。

完整 raw 证据未上传，fixture tests 可在 clone 中运行；完整保存日志重播要取得
既有本地文件并先比 SHA，不重新下载已有公共数据集。绝对路径和日志文件缺失
应报告为当前主机不可用，不得伪造验证成功。
