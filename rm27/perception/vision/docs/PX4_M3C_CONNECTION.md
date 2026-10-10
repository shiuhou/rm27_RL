# PX4 ↔ M3C 接线与验证状态

交接整理：2026-10-11。下面的实机结论来自 2026-10-09/10 保存的实验记录，
不是本次推送时的在线状态；本次没有访问设备。先读 [当前接续入口](CODEX_RESUME.md)。

## 已使用的硬件和接口

| 项目 | 已记录配置 / 证据边界 |
|---|---|
| 飞控 | 用户确认 MicoAir743V2-AIO-35A；PX4 报告 MICOAIR_H743_V2 |
| 当前已验证固件 | PX4 v1.17.0，完整 hash `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| M3C | 用户所称 Sipeed M3C Linux 板；镜像 DT 报 MaixCAM2，Linux 4.19.125 / aarch64 |
| M3C 数据口 | 硬件设计者指定 UART2，Linux `/dev/ttyS2` |
| 飞控数据口 | 用户标记 UART3，PX4 `/dev/ttyS2`，MAVLink instance #1 |
| 已验证串口条件 | **115200 baud，8N1，无硬件/软件流控**；Linux 测试期间 raw、关闭 echo |
| MAVLink 身份 | PX4 `sysid=1 / compid=1`；M3C `sysid=1 / compid=191` |
| MAVLink 版本 | 此链路 MAVLink2 已通过 Stage A/B，不需要从 v1 重做 |

实际信号方向是交叉连接：

```text
M3C UART2 TX  ───→  飞控 UART3 RX
M3C UART2 RX  ←───  飞控 UART3 TX
M3C GND      ─────  飞控 GND
```

两端都叫 `/dev/ttyS2`，不代表它们的板上接口编号相同。不要改成 TX 对 TX，
也不要据 Linux 设备名重新猜接线。用户用示波器确认过 M3C TX 有波形；
随后 PX4 解析到 HEARTBEAT 才构成 M3C→飞控方向的协议证据。
模块原理图的 J1 UART2 引脚记录在接口报告中，但不是已验证的载板焊盘图；
不得把模块引脚号当作重新接线指令。

## 管理连接不是数据 UART

- M3C 经 USB gadget 网络管理，已使用 `ssh root@10.18.198.1`。
  认证使用操作者自己的配置；本仓库不保存密码、私钥或认证 token。
- 飞控经 Windows **COM19 / USB VID:PID 1B8C:0036** 检查控制台和文件。
  COM19 的配置值 57600 是 USB CDC 设置，**不是**上述 UART 的物理 baud。
- PX4 USB 曾报告 `/dev/ttyACM0` / instance #3；其显示的虚拟 baud
  同样不能拿来当 UART 带宽。端口号是保存的快照，接入时要重新确认身份。
- 已观察到 M3C USB 处于 device/gadget 角色并承载 SSH；内核
  `CONFIG_USB_ACM` 未启用。`CONFIG_USB_CONFIGFS_ACM=y` 是 gadget 支持，
  不等于 host CDC 驱动。直接切 USB host 可能断开管理连接，不应自动操作。

用户已明确报告**没有第二组可用 UART**。旧报告中的“核对备用焊盘 / 独立 UART
DDS”是历史条件方案，不再是当前下一步。

MTF-02P 光流 / ToF 已在另一条飞控链路工作：保存状态为 instance #2、
`/dev/ttyS3 @115200`、compid 88、msgid 106/132 约 48 Hz。不要停用、
重配或占用它。其他枚举出的 tty 也不等于空闲可用口。

## 已完成的 Stage A/B（保持，不重跑）

完整报告：[MAVLink2 VALIDATION](../../../../.maixpy/mavlink2-20261010/VALIDATION.md)。
该报告 SHA256：`d9c79727ad749e8ec6ba970a2e41c0ef4aa5ccc3ac6e6e7bc53c0f1bbb7ffd0f`。

- Stage A：M3C HEARTBEAT 1 Hz，PX4 instance #1 RX 21.0 B/s，
  sysid 1 / compid 191 / msgid 0，版本 2。180 秒是上限，实际在 gate 通过后
  于 108 次发送处停止；不能称“完整 180 秒耐久测试”。
- Stage B：HEARTBEAT 1 Hz + **合成** ODOMETRY 10 Hz，msgid 331；
  LOCAL_NED / BODY_FRD，pose `(1, 2, -0.5)` m，identity quaternion。
  PX4 `vehicle_visual_odometry` 更新；停止后 timestamp 不再更新，重启恢复。
  保存 RX 约 2470 B/s。此为 transport/parser/uORB PASS，不是真实 VIO。
- 最后观察 commander Disarmed、`EKF2_EV_CTRL=0`；未开启外部视觉融合、
  未 ARM、未飞行。Linux termios 精确还原，测试程序已退出。

已部署测试目录：`/root/rm27-mavlink-test-20261010/`。
其 venv：Python 3.13.2 / pymavlink 2.4.49。
以下只是已完成实验的复现命令，**不是接续时自动执行的任务**：

```sh
/root/rm27-mavlink-test-20261010/venv/bin/python /root/rm27-mavlink-test-20261010/heartbeat_only.py --mavlink2 --seconds 180
/root/rm27-mavlink-test-20261010/venv/bin/python /root/rm27-mavlink-test-20261010/mavlink_odometry_test.py --seconds 45 --bench-ev-disabled-confirmed
```

脚本已选择性纳入 `.maixpy`。heartbeat 默认仍为 MAVLink1，复现 Stage A 必须带
`--mavlink2`。ODOMETRY 的确认 flag 是操作者声明，不是读取 PX4 参数的替代。
不得同时打开多个 UART owner。测试结束还原的 idle termios 曾是
9600/canonical/echo，所以单看 idle `stty` 不能否定测试窗口内的 115200。

## PX4 → M3C IMU：仍为 PARTIAL

保存的 HIGHRES_IMU 接收结果：请求 50 Hz 在原遥测负载下实际 **10.11255 Hz**；
临时减少竞争遥测、仍保持 115200 / 5760 B/s 软件预算后，50 Hz 请求实收
**50.00105 Hz**；100 Hz 请求只有 **62.11152 Hz**，对应缩频系数 0.621。
因此已验证主要是 MAVLink 软件总发送预算缩频，不是“传感器只有 10 Hz”。
启动无效字节 / CRC 错误保留，未宣称完全无损。临时 stream 和 termios 已还原。
见 [速率实验](../../../../.maixpy/imu-transport-20261010/VALIDATION.md)。

- 当前选中 BMI088：gyro ID **6684690** / accel ID **6946834**，instance 0。
  历史 BMI270 device 3604506 / instance 1 不能替代当前选中传感器证据。
- `vehicle_imu` 是校正、coning 积分后的 delta_angle / delta_velocity，
  含两个 dt、IDs、clip/calibration 计数。保存发布约 198 Hz，不是 >=200 Hz PASS。
- `HIGHRES_IMU` 使用积分/dt 后再减匹配 EKF bias 的值；缺 dt 和完整硬件身份。
  `sensor_gyro`/`sensor_accel` 也是驱动处理的 SI/batch 均值，不是 untouched ADC。
- 分别保存 PX4 HRT 样本/发布时间和 M3C monotonic 接收时间；接收时间不是采样时间。
  相机曝光时间、时钟映射、刚体外参、噪声和 OpenVINS 输入模型均未资格化。

当前候选：在现有单 UART 上评估 stock MAVLink ULog 完整记录采集，先于自定义
固件；**不是直接喂 OpenVINS**。200 Hz IMU-only 线速模型约 13.94 kB/s，
已超过 115200 / 8N1 的 11.52 kB/s 物理上限，还未计 startup/metadata。
921600 和软件预算 46080 B/s 只是待审实验提案；当前未改 baud、参数、固件或接线。
详见 [最新设计入口](../../../../.maixpy/imu-design-20261010/LATEST.md)。

## 接续边界

先做 live session controller 的离线 start/ACK/stop/timeout/restore 测试。
已有重组器只返回 ACK intents，不发送，不是可直接实机运行的 controller。
保留 Stage A/B 和原验收门槛，Stage C / 空间时间标定 / EKF 融合保持未开始。
本次“push”只授权仓库发布，不授权新的设备实验或通信设置改变。

原始 ULog、UART raw bytes、完整控制台日志、数据集、第三方构建仍在原实验主机，
没有随 Git 发布；报告内旧绝对路径是来源记录，不保证其他主机存在。
因此另一个 Codex 可继续代码与离线 fixture 工作，不能把报告摘要当作新实机验证。
