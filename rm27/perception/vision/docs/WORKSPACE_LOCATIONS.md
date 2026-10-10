# SSH 主机、M3C 与本地资料位置 — 2026-10-11

本清单回答“哪些东西通过 SSH 写入/运行，在哪里”。它依据保存的交接、验证与
脚本路径，不是 2026-10-11 的远端盘点；本次没有登录研究主机或 M3C。
地址、文件存在性、branch/HEAD 和进程现状可能已变化，接续时须只读核查。
部分源文件先在 Windows 编辑，再传到 Linux 验证，并非全部在远端直接编写。

## 1. Linux 研究主机：OpenVINS / VIO 软件与实验

当时 SSH 目标：`shiuhou@10.4.135.84`。这是研究电脑，不是 M3C。

### 代码工作树

```text
/home/shiuhou/Projects/rm27_slam_vio_openvins/
```

保存状态：branch `research/vio-openvins-baseline`，
HEAD `4637a5f589801f54ecab146a389db0c81ca1d13f` 加 scoped 未提交修改。
这是历史状态，不能据此声称它现在已同步 GitHub main。

| 此工作树内的位置 | 做过的工作 |
|---|---|
| `rm27/perception/vision/localization/experiments/` | 既有 OpenVINS→LocalizationEstimate adapter/runtime；新增 `px4_ulog.py` FIFO 导出和 `vio_dataset.py` 时间/标定接口 |
| `tools/openvins/` | 原生 ROS2 启动、adapter 启动、评估、callback trace、camera model audit、lifecycle/lifetime/reliable-input 补丁和 QoS recipe |
| `tests/` | 对应 adapter、FIFO、时间接口、trace/model 等离线回归 |
| `rm27/perception/vision/docs/`、`HANDOFF.md`、`VAULT_UPDATE.md` | 验证报告、采集/标定提案、交接；Vault proposal 不代表写过 Vault |

Windows 镜像的保存 transfer 记录曾比对 29 个 scoped 文件逐字节一致。
这只证明当次传输，不代表当前两份工作树完全相同；也不代表所有实验资料进了 Git。

### 仓库外的源码、构建、数据和实验结果

总目录：`/home/shiuhou/Projects/rm27-vio-20261007/`。下面均相对此目录：

| 子路径 | 内容 / 身份 |
|---|---|
| `upstream/openvins-jazzy-lifecycle/` | 使用上游源码的独立 lifecycle 修补副本，不是全新自研估计器 |
| `native-ros2-lifecycle/` | 对应 ROS2 构建 / 安装目录 |
| `repeatability-20261008/` | 原始原生重复运行、冻结配置/manifest、轨迹与评估结果 |
| `offline-20261009/` | 后续 lifetime、trace、serial、reliable、delivery 对照及诊断脚本、日志、manifest |
| `offline-20261009/{variant}-source/`、`{variant}-build/`、`{variant}-repeatability/` | 按 `fixed/trace/serial/reliable/delivery` 分开的实验族；这是命名模式，不是单个目录的字面名字 |
| `offline-20261009/delivery-adapter-repo/` | 隔离的完整 adapter candidate 副本；不是替换原工作树的默认配置 |
| `euroc-v101-ros2-verified/` | 已有、已验证 ROS2 输入包；数据不是本次实验生成的“真实 RM27”数据 |
| `imu-offline-20261008/` | 历史 BMI270 ULog 证据副本，不是当前选中 BMI088 的数据 |

这些目录包含第三方源码/构建和已有数据，也包含本任务生成的实验产物；不要把
所有文件都称为“自己写的代码”。不得为了整理或重新构建而删除旧失败/成功运行。
旧报告的 `/home/shiuhou/rm27-vio-20261007/` 是迁移前路径，当前记录使用
`/home/shiuhou/Projects/rm27-vio-20261007/`；不要误建第二套重复实验资料。

## 2. M3C：板端部署与 UART 验证

保存的 SSH 目标：`root@10.18.198.1`，经 USB gadget 网络管理。
接线/物理串口与 SSH 管理接口区别见 [PX4_M3C_CONNECTION.md](PX4_M3C_CONNECTION.md)。

| 板端目录 | 已部署/运行的东西 |
|---|---|
| `/root/rm27-mavlink-test-20261010/` | `heartbeat_only.py`、`mavlink_odometry_test.py`、`probe.py`，后续 `receive_imu.py` / IMU decoder 及对应测试/日志；现有隔离 `venv/` 为 Python 3.13.2 / pymavlink 2.4.49 |
| `/root/slam-bench-20261007/` | 早期 Python/C++ 稀疏视觉前端性能测试、可执行文件、灰度帧和标定资料；不是完整 SLAM/VIO 实机系统 |

Stage A/B 和 passive IMU/rate 测试结束时记录程序已退出、termios 已还原。
这不是现在没有进程的实时保证。部署的 capture decoder 和后来 Windows 的诊断
修订可能不同；不能把 GitHub 最新脚本当作已经部署的板端版本。

## 3. Windows：编辑镜像与近期离线 IMU 工作

```text
C:\Users\USER\OneDrive\RM27\rm27_slam
```

- `.maixpy/offline-vio-20261009/`：Linux VIO 工作的编辑/证据镜像。
- `.maixpy/mavlink2-20261010/`、`imu-input-20261010/`、`imu-transport-20261010/`：
  板端脚本本地副本、解码分析、实机验证报告和保存证据。
- `.maixpy/imu-design-20261010/`，尤其 `single-uart/reassembly/`：近期离线
  高率输入研究和有限 ULog/MAVLink 重组器；不是已经部署到 M3C 的 live controller。
- `rm27/perception/vision/docs/`：GitHub 可直接阅读的接线、接续和本清单。

飞控未通过 SSH 写 Linux 程序；相关 PX4 控制台/下载使用 Windows COM19 USB，
不能与 M3C 的物理 UART 或 Linux 研究主机混为一谈。

## 4. GitHub 发布与接续规则

2026-10-11 的 `4dba7fd` 从 Windows 仓库发布了选择性的代码/报告/补丁。
完整 raw ULog、UART bytes、数据集、轨迹、第三方源码树、构建、虚拟环境和认证
资料未上传。最新接线文档不是主机磁盘备份；详见 [.maixpy 发布范围](../../../../.maixpy/README.md)。

**Push 不会自动更新 SSH 主机或 M3C 部署。** 另一个 Codex 在研究主机开始工作时，
先确认 `hostname`、实际目录、`git status --short --branch` 和 `git log -1`，
比较未提交修改；不要直接覆盖、reset/clean 或盲目 pull 到旧 dirty 工作树。
不重下载已有数据，不重复已完成 Stage A/B，不自行启动采集、改串口或 EKF 融合。

## 保存证据入口

- [Linux 离线 handoff](../../../../.maixpy/offline-vio-20261009/HANDOFF.md)
  与 [验证报告](../../../../.maixpy/offline-vio-20261009/OFFLINE_VALIDATION.md)。
- [Stage A/B 部署与运行报告](../../../../.maixpy/mavlink2-20261010/VALIDATION.md)
  与 [IMU 输入报告](../../../../.maixpy/imu-input-20261010/IMU_INPUT_VALIDATION.md)。
- [Windows 历史 handoff](../../../../.maixpy/HANDOFF_WINDOWS_20261011.md)：
  末尾记录 `/root/slam-bench-20261007` 的板端 staging。

文件位置是保存记录支持的事实；当前可达性和当前磁盘/进程状态仍是 UNKNOWN。
