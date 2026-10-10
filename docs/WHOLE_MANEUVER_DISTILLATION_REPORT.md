# RM27 Whole-Maneuver Policy Distillation 实验报告

2026-09-23；`astra/research`，HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。

**本轮有效的是“学习连续参考 + 保留模型反馈”的混合 student。纯动作 MLP、短历史 MLP 和两种纠偏数据训练均未达到任务成功。**
混合 student 在新的完整场地留出中，理想 ego estimate 为 **72/72**，synthetic noisy estimate 为 **70/72**；
teacher 为 **72/72**。加入 10 ms ego-observation delay 后降到 **59/72**。
额外要求 margin≥8 mm 时，noisy 为 **69/72**，noise+delay 为 **57/72**。
这支持继续发展结构化 imitation，不支持宣布纯 neural policy 已解决任务，也不支持立即把剩余问题归给 RL。

当前可运行 demo 是 learned hybrid student 从空中机库附近进场、指定接触、脱离恢复，再交给模型控制器返航。
它使用 causal estimated contact，actor 不读 simulator contact truth；ego 仍来自 truth/synthetic estimator。
没有视觉、ArduPilot firmware、SITL 或真机验证。

## 先审查实际基础

读过现有 `WHOLE_MANEUVER_TEACHER_REPORT.md`、`WHOLE_MANEUVER_REPORT.md`、aggressive-contact v1/v2 报告，
以及 `teacher_data.py`、`teacher_interface.py`、`control.py`、`maneuver.py`、`experiment.py`、`model.py`、`recording.py` 和现有 dataset schema/index/benchmark。
继续使用 root README、START_HERE、INTEGRATION、REAL_HARDWARE 的组件与坐标契约；检查了当前分支、dirty status、worktrees。
没有清理旧 artifacts、其他 worktree 或嵌套仓库。

实际 teacher 已经是参数化全局轨迹 + geometric feedback，在线并不是大网络，也不是每个控制周期重新做昂贵搜索。
不能把本轮说成“把一个原本跑不实时的 teacher 变实时”。昂贵部分主要是离线场景搜索与验证。
已有 action projection 是 **FLU body rates + physical collective N**，控制 200 Hz、MuJoCo 1 kHz；
rate proxy 与 ArduPilot 内环不相同。仿真 proxy 使用当前真实 body rate 作为理想化内环 gyro；
noise/delay profile 主要作用于外环 ego observation，没有验证真实 gyro／飞控内环误差。

旧 dataset v1.1 有 320 episodes，但其中 240 是旧 holdout，design/validation 的独立物理 seeds 也很少。
**本轮未把旧 holdout 加入训练**。重新生成了 66 条完整场地 teacher demonstrations：
48 train（16 个完整物理 draws × 3 modes），18 validation（6 draws × 3）。全部成功只是本次生成结果，原有失败数据仍保留。
随后另外生成 24 条 autonomous student + privileged oracle labels（全部任务失败）、24 条带 70% teacher 干预的训练轨迹（全部通过）。
这 48 条纠偏轨迹都保留；有干预的成功不算 autonomous student 成功。

场景不变：原 RMUC field、dock、micro-UAV、物理 armor patch/bumper/rim。整座 base 的视觉 mesh 仍不代表完整实体碰撞。
约 180 g、inertia、motor tau/thrust/yaw 系数和 contact properties 仍为 engineering assumptions。
本轮未手调 teacher 的 trajectory、contact/recovery 参数或稳定 baseline。

## 数据、观测与动作

配置：`configs/contact_distillation.json`；新数据：`artifacts/distill-data-01/`。
train seeds **810000–810015**，validation **820000–820005**；偶数 1×、奇数 2× 原有物理扰动。
最终 untouched test：small **830000–830011**、stress **840000–840011**。
同一 seed 在不同 operating point／方法里属于同一 split；不随机拆邻帧。训练 shuffle 只在 train split 内进行。
额外 DAgger 数据只准使用 `split=train` 的 oracle run。数据审计和训练入口都检查分组泄漏。

原始 teacher NPZ/JSON 保存 privilege 标记、task/config、hidden draw、reference、motor states、events、metrics 和 source/model hashes。
student-specific 契约在 `artifacts/distill-student-bundle-02/schema.json`，本轮约定如下：

| 可供 actor 的信息 | 来源与限制 |
|---|---|
| elapsed time / planned duration / mode | 已知任务时钟与 aggression command，不是未来接触时间 |
| ego position−committed target、world velocity、rotation 前两列、body rate | 当前 truth 或 synthetic estimator；不是已验证机载估计 |
| motor-force observer / effective thrust ratio | 自己过去控制和 ego 增量推算，非真实 `data.act` |
| previous proposed rate/thrust | 本策略上一条 proposal；有 queue 时不等于已作用的 motor force |
| estimated contact latch / elapsed since latch | 仅由过去速度创新、估计位置和 nominal target 得到 |
| ego observation age | acquisition timestamp 与 control clock 的差 |

基础向量 **32 维**；历史版串联 `[0,2,6]` 个控制步，即 0/10/30 ms，共 96 维。
单帧 MLP 是没有额外 stacked history 的网络；整个系统仍有 actuator observer、previous action 和 latch，
所以不能把它叫作完全无记忆的飞行系统。

禁止 actor 输入：hidden mass/thrust/tau draw、true motors、contact truth、teacher phase、future state、teacher reference。
reference 和 projected action 只作为训练监督；混合 student 运行时查询的是自己的拟合函数，不是原 teacher reference。
MLP 未显式输入 target normal，当前固定场景中的 nominal normal 用于接触检测；混合控制器也用 committed normal。
改变场地／目标面大范围方向不在本次泛化声明内。

输出统一为 **body rates (FLU, rad/s) + collective force (N)**，通过已有 rate-P proxy 到真实 MuJoCo motors。
最终对比中 teacher/student 都用 200 Hz、rate ±20 rad/s、同一 motor force bounds。
早期开发验证使用过 student ±15 rad/s；原结果保留。最终 `distill-confirmatory-01` 已统一为 ±20，
noisy hybrid 和补偿 ablation 也按此重新核对。
原 teacher dataset 的监督保留原始 motor→rate 投影，少数瞬时标签可超过 20（最高约 25.14 rad/s）；
线上统一限幅。因此原始监督误差不能当成完全相同 actuator action 的闭环证明。

`StudentAPI.infer(VehicleEstimate, now=control_clock) -> RateThrust` 不需要 MjData、ManeuverSkill 或 teacher 查询。
`NominalContext` 只含名义校准和任务常数；`actuator_estimate()` 可取内部估计。
支持非零任务起始时钟以及采集时间／控制时间分离；专门测试了 1000 秒时钟平移不改变动作。
`StudentAPI.from_bundle(...)` 在 1802 个相同 recorded estimates 上与仿真 actor 的最大 action 差为 **7.26e-14**，
见 `distill-standalone-01`；这个路径没有构造 teacher 或 optimizer。
研究 runner 仍创建 teacher 对象用于统一评价 reference、或显式 DAgger label；普通 student action 不调用其 command。

这个接口不是已完成的飞控移植。ArduPilot Guided `SET_ATTITUDE_TARGET` 的 mask、FRD 符号和 thrust 语义沿用前轮审查：
默认 thrust 是爬升率，切换选项后仍有 angle boost、滤波、thrust linearization。
**N 不能直接当 normalized throttle**；本轮没有 sender、没有写参数。
共享 Python 模块仍依赖当前 NumPy/SciPy/RM27 环境；小 checkpoint 不等于整个机载软件已经移植完成。

## 实际训练了什么

没有安装 Torch 等新框架；实现 NumPy tanh MLP + Adam、train-only normalization、固定 seed 92317、batch 512、lr .001。
BC/history 各 160 epochs；DAgger/corrective 240 epochs。按整条 validation episode 的 action loss 选择 checkpoint，
**最终选择用闭环任务结果，不用 MSE 代替飞行结果**。

| model | hidden layers | learned parameters | checkpoint bytes | validation action RMSE (roll,pitch,yaw rad/s; N) |
|---|---|---:|---:|---|
| bc | [64, 64] | 6532 | 27027 | 0.0367, 0.2090, 0.0129, 0.0266 |
| history | [64, 64] | 10628 | 43183 | 0.0348, 0.1812, 0.0116, 0.0264 |
| dagger | [64, 64] | 10628 | 43310 | 0.1179, 0.3354, 0.0559, 0.1070 |
| corrective | [128, 128] | 29444 | 113370 | 0.0249, 0.1708, 0.0104, 0.0254 |
| noisy-bc | [64, 64] | 10628 | 43259 | 0.0338, 0.2430, 0.0123, 0.0310 |

- BC：32→64→64→4。
- History：96→64→64→4；离线 pitch-rate RMSE 从约 .209 降到 .181 rad/s，闭环仍失败。
- DAgger：history student 在训练 draws 上独立飞，teacher 从这些 state 查询 action labels；重新拟合原 demos + 24 条失败轨迹。
- Corrective：70% teacher + 30% history proposal 混合执行收集 24 条近参考轨迹，再训练 128×128 网络。改变了数据与网络容量，不能隔离两者贡献。
- Noisy BC：把同样的 timestamped ego noise 注入训练 observation，重新推进 causal observer/latch，label 仍由 privileged teacher 提供；在独立 noisy validation 上评价。

这些直接动作网络在相应 18 条 validation 闭环中都为 **0/18**；noisy training 也没有解决根本漂移。
未把这些负结果删掉，也没有据此声称所有 MLP/DAgger 都不适合 RM27。
本轮只有一个固定训练 seed，未完成神经网络 training-seed robustness 研究。
不同 DAgger 数据的 label normalization scale 不同，不能拿它们的 normalized MSE 直接横比。

### 有效的替代：reference distillation + feedback

用 train episodes 的 pre-contact reference p/v/a 监督拟合 **degree-5 B-spline**，每档 69 个三维控制系数，
三档共 **621 learned coefficients**，checkpoint **7824 bytes**。
均匀时间 knots，不复制原 minimum-snap polynomial coefficients；用位置、速度、加速度导数一致的最小二乘和微小 jerk 正则。
拟合三档 nominal reference 的整体位置 RMSE 约 13/14/27 mm，包含 teacher 随初态变化的轨迹差异，不能当作接触误差。

运行时：fitted reference → geometric feedback / actuator compensation → rate/N action；
估计到碰撞后，用观察到的当前位置建立恢复锚点，并沿用明确标注的 model-based recovery。
当前状态不断修正控制，真实接触时机由观测决定，所以不是纯开环回放；
但**反馈律、observer 和 recovery 逻辑没有被神经网络蒸馏掉**，这是一种明确的 hybrid student。

这种结构避免网络在狭窄 expert 状态流形上同时学习名义机动和高增益稳定反馈。
它保留已知物理结构，学习成本低，适合本轮固定场景；尚未证明能泛化到新场地／新目标布局。
原 teacher 本身也是紧凑 polynomial + feedback；621 个拟合系数不代表比原 teacher 的参数更少。
因此本轮没有证明这个 hybrid 比直接缓存并执行原 teacher 更划算，不能将其包装成整个控制器的成功压缩。
它证明了结构化 imitation 可以保持性能，也暴露了直接 action distillation 的失败；工程上仍应保留原 model-based teacher。

## Compounding error 与最难的 phase

在 teacher-state validation 上，最大动作误差确实出现在接触后 100 ms：
BC pitch-rate RMSE **1.126 rad/s**、force **.157 N**；approach 是 **.154 rad/s / .016 N**。
history 将接触 pitch-rate RMSE 降到 **1.030 rad/s**，仍远大于普通 recovery 的 .046 rad/s。
teacher 使用 contact truth 的即时切换；student 只能等观测出现冲击证据，部分切换误差不能凭单帧完美复现。

但闭环失败不只在接触：固定 validation balanced seed 820000，BC **.535 s** 就偏离 teacher 超过 10 cm，
history 更早到 **.460 s**，远早于约 4.5 秒的接触。
同一 draw 下 hybrid noisy 的全程位置差最大 **2.71 cm**（含 recovery，不是终端误差）。
这支持 distribution drift / 未学到足够稳定反馈的解释；不是严格隔离了唯一原因的因果证明。

首轮 DAgger 收入大量已经远离可恢复飞行范围的状态，没有改善成功率；teacher 在这些状态也不是全局最优救机 oracle。
带干预的近参考数据确实更接近任务分布，但撤掉 teacher 后较大网络仍 0/18。
后续若坚持纯网络，需要更好的闭环训练／稳定性结构，而不能只继续降低 expert-state MSE。

证据：`distill-phase-audit-01/`、`distill-dagger-data-val-01/`、`distill-interventions-01/`、`distill-corrective-val-01/`。

## Observation/action 敏感性与改进

本轮 synthetic noise 假设：position σ=2 mm、velocity σ=.02 m/s、attitude σ=.2°、body-rate σ=.02 rad/s。
它们不是 M3C 或飞控实测噪声。已有物理 target pose perturbation 继续存在；未接入相机 FoV 或在线 target stream。

最初单步 contact innovation threshold 会被 velocity noise 提前触发，导致离板 recovery 在真正触板前开始：
noisy validation 只有 **11/18**。换成 20 ms signed normal velocity-innovation 窗口，
要求 residual impulse>.12 m/s 且在 nominal target 附近，提升到 **18/18**。
这些是同一组物理 draw、同一 noise seed、相同开发 action bound 的配对实验；最终 ±20 bound 也得到 18/18。
该时间窗不是 learned confidence model，也不等于真实 tactile sensor。

| validation 条件 | 联合成功 | 说明 |
|---|---:|---|
| hybrid、ideal ego estimate | 18/18 | retained feedback + estimated event |
| noisy、单步检测 | 11/18 | 提前误触发、多为 TARGET_MISS |
| noisy、20 ms 窗口 | 18/18 | 有效的 temporal information 用途 |
| noisy + 10 ms ego delay、窗口 | 15/18 | latency 与 noise 组合仍退化 |
| 20% ego packet dropout、窗口 | 18/18 | 仅这组配置，无宽泛 dropout 保证 |
| 200 Hz action、额外 20 ms action delay | 18/18 | 不同于前轮 50 Hz+20 ms，不能混为一谈 |
| 关闭 actuator compensation + effective-thrust adaptation | 12/18 | 两者一起关闭，不能单独归因给某一项 |

最后一项有统一 ±20 bound、相同 perfect_filtered profile 的 on/off 复核。
Causal motor model 仍参与接触创新，因此它也不是“删除所有 actuator-related state”的纯 ablation。
直接 MLP 是否能单独受益于 motor observation，本轮没有正面闭环证据。

视觉／SITL 资产核查：`REAL_HARDWARE.md` 的 M3C+OS04A10、TargetEstimate JSON v2、
`SLAM_RECOVERY_AND_AUDIT.md` 的 source/output timestamp 差异和实际镜头 calibration blocker 都仍相关。
当前 micro-flight telemetry/bridge 是 500 Hz plant + 不同 guidance path；不能把 MP4 PTS 或旧 truth-aided telemetry 当本 maneuver 的已验证 ego state。
本轮因此使用明确的 SyntheticEstimator boundary，未将未标定录像伪装成 metric sensor training data。

## 冻结后独立测试

`distill-confirmatory-01/frozen_selection.json` 在 test rollout 前保存了方法、checkpoint SHA256、seeds、profile、共同 action limit。
最终完整 benchmark **384 episodes**：三档 teacher/BC/hybrid-state/hybrid-noisy/hybrid-noisy-delay 各 72，
另 adaptive_nominal 24。不根据最终结果修改 policy 或 detector。
后续 standalone clock API 修补不改变零起始仿真，已用完整 4019 样本轨迹逐元素相等验证。
额外 noisy-BC 研究只在 train/validation 上做，未加入这个冻结 test cohort。

| operating point | Teacher small / stress | Hybrid state small / stress | Hybrid noisy small / stress | Hybrid noisy+10 ms small / stress | BC small / stress |
|---|---|---|---|---|---|
| aggressive | 12/12 / 12/12 | 12/12 / 12/12 | 12/12 / 12/12 | 11/12 / 8/12 | 0/12 / 0/12 |
| balanced | 12/12 / 12/12 | 12/12 / 12/12 | 12/12 / 12/12 | 12/12 / 9/12 | 0/12 / 0/12 |
| conservative | 12/12 / 12/12 | 12/12 / 12/12 | 12/12 / 10/12 | 12/12 / 7/12 | 0/12 / 0/12 |

同一 physical seed 被三个 modes 复用；72 episodes 不是 72 个独立物理 draws。
每 mode/split 只有 12 次，12/12 的 Wilson 95% 下界约 75.8%。这是有限条件的仿真证据。

额外 8 mm margin qualification：state teacher/student 都 72/72；noisy 69/72，noise+delay 57/72。
尤其 noisy balanced stress 原 PASS 是 12/12，但只有 **11/12** 保持 8 mm margin；不隐去这一退化。

从空中出发到 held recovery 的中位数（有恢复确认的 trials，失败保留）：

| mode | Teacher small / stress s | Hybrid state small / stress s | Hybrid noisy small / stress s |
|---|---:|---:|---:|
| aggressive | 5.471 / 5.647 | 5.471 / 5.653 | 5.571 / 5.613 |
| balanced | 6.052 / 6.233 | 6.052 / 6.238 | 6.054 / 6.202 |
| conservative | 7.659 / 7.724 | 7.659 / 7.724 | 7.728 / 7.972 |

adaptive_nominal small/stress 均 12/12，时间中位数 **15.104 / 15.336 s**，未弱化其 adaptive controller。
state hybrid 几乎保留 teacher 时间；noisy balanced stress **6.202 s** 对 teacher **6.233 s**，
这个细小差异来自接触瞬态和离散事件，不是 student 找到了更优 maneuver。

balanced stress 接触位置误差中位数：teacher **7.49 mm**，state hybrid **7.46 mm**，noisy hybrid **8.65 mm**；
region margin 中位数 **24.42 / 24.44 / 23.46 mm**；post-contact max body rate 的中位数 **4.12 / 4.09 / 4.64 rad/s**。
noisy + 10 ms ego delay 时，balanced recovery-time 中位数从 .75 s 上升到 **1.013 s**，联合成功只有 9/12。
估计接触的典型延迟从约 3–4 ms 增加到约 13 ms。

失败迁移明确：纯动作网络多数在 approach 失控／miss；noisy conservative 的两个 stress 失败是 edge/wrong-first/undesigned contact；
noise+delay 则增加 recovery timeout，并保留几何失败。
失败不是都能靠追加一个 recovery residual 消除。

纯动作 NN 的 trace reference 栏是当前状态诊断值，不是它规划的参考；轨迹偏差比较使用配对 teacher 轨迹，
公共 evaluator 的 tracking error 也仍对其独立持有的 teacher trajectory 计算，均不送入 actor。

机器可读完整指标（接触速度／姿态／margin、body rate、separation/secondary、高度损失、saturation、失败分类）
在原 episode `summary.json` 和 `distill-analysis-final-01/benchmark.json`。
[留出比较图](../artifacts/distill-analysis-final-01/holdout.png) · [3D 轨迹](../artifacts/distill-analysis-final-01/trajectory3d.png) · [闭环状态叠图](../artifacts/distill-plots-final-01/closed_loop.png)。

## 模型大小、runtime 和完整 demo

host：Intel Core **i5-14400**，Linux x86_64，`OPENBLAS_NUM_THREADS=1`。

| 部分 | 参数 | checkpoint | median / p99 host inference |
|---|---:|---:|---:|
| BC MLP | 6532 | 27027 B | 9.53 / 16.46 μs |
| History MLP | 10628 | 43183 B | 10.74 / 12.24 μs |
| Hybrid fitted reference | 621 | 7824 B | 11.23 / 13.17 μs |
| 完整 hybrid 路径 | 加上现有反馈／observer／proxy | 不是只有 spline | 372 / 622 μs |

完整路径 benchmark 包含 synthetic estimator、observer、event detector、参考、geometric feedback、rate proxy 和 log append，
输入为 recorded states，不计 MuJoCo physics step。200 Hz 的周期是 5000 μs；host 算力有余量。
Python 测试进程 max RSS 约 **121 MiB**，主要包括解释器、NumPy/SciPy/MuJoCo 依赖，不能称为 7.8 KB 的完整系统。
没有 M3C benchmark、ONNX/int8、硬实时或 onboard vision-cost 结论。
在线 teacher 原本也不昂贵，所以没有宣称混合 student 相比 teacher 获得 CPU 加速。

当前 `distill-mission-03`：noisy balanced learned hybrid 完成 contact/recovery 后，同一 live plant 交给 model-based return。
完整往返 **20.086 s**；held recovery 后约 14 秒是原返航路线。只验证了这个 nominal 往返，
不能把上面的 72 条 maneuver 测试说成 72 条 return 成功。
起点在 dock 上方空中；无停桨起飞、无精确降落。返航 handoff 目前仍用仿真 evaluator 的 held-recovery gate，未验证 onboard mission gate。

`distill-mission-02` 与 clock/API 修补后的 `-03` 的 4019×43 trace **逐元素相等**。
录制 motor replay 的 qpos/qvel/motor 最大误差均 **0**，不是异步闭环确定性保证。
EGL 完成了完整运动渲染，未运行桌面 GUI。
[完整运动 GIF](../artifacts/distill-render-02/maneuver.gif) · [接触 GIF](../artifacts/distill-render-02/contact.gif)。

## 文件与复现

新增 `rm27/experiments/aggressive_contact/distillation/`：
`observation` / `network` / `dataset` / `policy` / `spline` / `evaluation` / `campaign` / `mission` / `analysis` / `audit` / CLI。
新增配置、`tests/test_contact_distillation.py` 和本报告。
对既有模块只增加 `maneuver.episode(skill_factory=...)` 注入点，以及 recursive evidence snapshot；不改旧 teacher/gates。
`StudentAPI` 是独立推理入口；仿真-only wrapper 才接 evaluator 与 MuJoCo。

训练输出包括 seed、全部 epoch metrics、normalization、checkpoint、train-source hashes、纠偏数据 provenance。
`distill-student-bundle-02` 包含 checkpoint、schema、nominal vehicle constants、task modes、split 声明与 source hashes；
nominal vehicle 文件明确不是实测校准。
所有 run 使用新目录；失败的 raw trace、physical draw、configuration 和 summary 保留。

以下核心命令实际执行过（artifact 后缀可换成新的，不能覆盖原目录）：

```bash
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation generate --output artifacts/distill-data-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation train --input artifacts/distill-data-01 --output artifacts/distill-bc-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation train --input artifacts/distill-data-01 --history --output artifacts/distill-history-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation spline --input artifacts/distill-data-01 --output artifacts/distill-spline-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation evaluate --policy artifacts/distill-spline-01/policy.npz --hybrid --profile noise_filtered --output artifacts/distill-eval-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation evaluate --policy artifacts/distill-history-01/policy.npz --split train --oracle --mix 0.7 --output artifacts/distill-corrective-data-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation train --input artifacts/distill-data-01 --history --extra artifacts/distill-interventions-01 --width 128 --epochs 240 --output artifacts/distill-corrective-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation confirm --input artifacts/distill-bc-01/policy.npz --policy artifacts/distill-spline-01/policy.npz --output artifacts/distill-confirmatory-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation mission --policy artifacts/distill-spline-01/policy.npz --profile noise_filtered --output artifacts/distill-mission-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/distill-mission-03 --speed 1 --loop
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

`confirm --input` 是 BC checkpoint，`--policy` 是 hybrid checkpoint。若用于新的模型选择研究，
应更换配置中的最终 holdout seeds，不能反复用本轮 test 选模型。
`benchmark`、`plot`、API `phase_audit/summarize/audit.run` 提供性能、图和 schema/hash 复核。

最终根 tests：**82 passed**（新增 8 个）。检查了 history causality、whole-scenario split、contact truth 独立性、
训练/序列化、噪声 contact 判定、standalone API、时钟平移；既有 74 项继续通过。
另有完整场地 demo、录制输入 replay、dataset hash/split audit 和上述闭环 benchmarks。
没有将 campaign 的 `passed:true` 当成所有 flight 成功；它表示 runner 完整执行。

## 回答研究问题与下一阶段

1. **保留多少性能？** Hybrid state 在本次 72/72 几乎保持 teacher 时间和 precision；noisy 70/72、8 mm qualification 69/72。
2. **最难 imitation 的 phase？** expert-state action error 最大在接触后 100 ms；但纯网络实际闭环从 approach 就漂移。
3. **memory 是否必要？** 10/30 ms stacked history 降了离线误差但未带来任务成功；20 ms event window 明确减少噪声误触发。尚无 GRU 必要性的证据。
4. **actuator estimate 是否有用？** hybrid 的补偿/有效推重比 adaptation 联合 ablation 有明显下降；不能分离两者或替神经网络输入消融下结论。
5. **noisy/deployable gap？** 单独小噪声损失有限；叠加 10 ms ego delay 降到 59/72。deployable 仅指输入语义，不是已验证 onboard pipeline。
6. **是否 compounding error？** 早期 trajectory drift 与低 expert-state MSE 并存；这是强证据，但未排除表示／优化不足。
7. **DAgger 是否有效？** 本轮 autonomous aggregation 与 teacher-intervened corrective 数据均未提高纯动作网络的成功率。
8. **需要多大 policy？** 有效 learned reference 621 个系数；6.5k/10.6k/29.4k MLP 均未成功，不应把小网络体积当任务解决。
9. **机载实时是否现实？** host 完整控制路径 p99≈.62 ms，计算规模可行；M3C、实际飞控、估计延迟、接触传感和 throttle mapping 都尚未验证。
10. **现在进入 Residual RL 吗？** **不建议马上进入。** 已知主要 gap 是可观测事件、ego timing 和稳定反馈结构；
先以当前 hybrid 为基线完成真实飞控 rate/thrust 内环与 timestamped estimator 的闭环验证。
若之后仍有可重复、可测量的接触／恢复残差，再让 residual learning/RL 解决具体残差，不能让 RL 掩盖接口误差。

本轮 status：IMPLEMENTED、RUN、TESTED；在列出的分布上 SIM-VALIDATED hybrid；HARDWARE-VALIDATED **无**。
