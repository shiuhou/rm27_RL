# RM27 aggressive contact v2：真实场地任务与鲁棒性瓶颈

2026-09-23，当前 `astra/research`，HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。
本报告对应工作区中的实现及保存的源代码快照，不表示这些修改已经提交。

**已在现有完整 RMUC 场地跑通：机库上方空中出发 → 连续飞向蓝方基地 → 底部 bumper 接触 → 脱离恢复 → 返回机库上方。**
单次任务 32 s，没有飞行中重置/传送状态。默认改进控制器的 6 次场地任务全部通过；
场景优化后的 v2 参数也完成了同一全场 nominal 任务。提供新闭环运行和记录状态观看两种 MuJoCo viewer 入口。

局部研究完成 **768 次训练、1,280 次新留出评价、192 次配对因果干预及 18 次步长检查**。
与真实基地一致的测试片方向下，2× stress 的联合成功数从 baseline **26/64**、v1 **21/64**，
提高到简单 actuator/thrust adaptive controller **64/64**。更复杂的 scenario-selected v2
也是 **64/64**：它改善余量和恢复瞬态，但没有证明成功率高于这个更简单的改进。

所有飞行结果都是 **headless、MuJoCo truth-assisted、direct motor control**。
没有验证 SITL、视觉、真机或裁判判伤；没有起飞、精确入库或停桨。
GUI 的独立状态接口经过测试，实际 EGL 图像/动画已渲染核查；本轮没有启动桌面 GUI 会话。

## 1. 直接运行和观看

在本机仓库根目录，将每条命令作为一整行粘贴。不要在 `-m` 后直接按 Enter。
桌面终端的新闭环任务，默认是 nominal 轨迹加马达补偿与推力自适应：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena --view --output "artifacts/contact-arena-$(date +%Y%m%d-%H%M%S)"
```

Space 暂停，V 全场／跟随；`--speed 0.5` 慢速。去掉 `--view` 为 headless。
重看已经实际跑通的 selected v2 完整任务：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/contact-arena-selected-v2-20260923 --speed 0.5 --loop
```

后者是**记录状态播放**，不重新计算反馈或物理。`replay` 则是记录马达输入重新推进物理。
新实验使用全新 output；退出 viewer 会保存失败/中止状态，不伪报任务完成。
若要在新任务中使用优化后的参数，加上：
`--parameters artifacts/contact-v2-confirmatory-20260924/selected_parameters.json`。

![Existing RMUC field](../artifacts/contact-arena-selected-render-20260923/arena.png)

[底部接触动画](../artifacts/contact-arena-selected-render-20260923/contact.gif) ·
[接触瞬间](../artifacts/contact-arena-selected-render-20260923/contact.png) ·
[完整任务轨迹／姿态／控制图](../artifacts/contact-arena-selected-plots-20260923/trajectory.png)

## 2. 场地资产核对与真实关系

实际复查 `README.md`、`START_HERE.md`、`AGENTS.md`、`docs/INTEGRATION.md`、
`ISOLATED_SCENE.md`、`CORRIDOR_REPORT.md`、`BASE_TARGET.md`、第一轮接触报告，
`configs/scene.json`、`isolated_scene.json`、`base_target.json`、`drone.json`；
读取 `integration/isolated_scene.py`、`base_target.py`、`drone.py`，
`experiments/route_experiment.py`、`corridor_audit.py`，及 aggressive-contact 全部实现。
本轮没有新增文献实验主张；路线选择依据代码核对、配对干预和真实运行结果。

| 资产 | 当前实际角色与处理 |
|---|---|
| `rm27/simulation/field/rmuc2026_battlefield_mjcf/` | 当前场地组件，pin `67c13270e213cf3f69da2619e349eed273432330`，未编辑上游几何 |
| `artifacts/dock-final/scene.xml` | 完整场地 + 当前机库 + 4 个静态任务预览用动态机体；作为全场组合源 |
| `artifacts/base-target-v4/scene.xml` | 存在，但仍是旧正 X 基地目标配置；没有复用其目标位置 |
| `artifacts/isolated-dock-base-20260916-v6/scene.xml` | 已有机库／负 X 蓝方基地提取场景；可通过 `--scene isolated` 使用 |
| `configs/base_target.json` + `integration/base_target.py` | 当前蓝方 armor patch／绿灯标记语义；绿灯是引导标记，接触对象为 armor |
| `integration/drone.py` + `configs/drone.json` | 复用现有 180 g 微型机及马达模型；新增底部 bumper |
| 新 `arena.py` + `configs/contact_arena.json` | 根层组合、路径迁移验证、单架任务及研究接触 proxy；不改 frozen G1 |

原完整场景 XML SHA256：
`768545c221a1b9337ba1dbc5ed49dc5fa56dec15eed875c3d73812efbbf646cb`。
isolated 场景 SHA256：
`70ef7bf0462cac5a0a5380cf3874f914df8d9886b8f767f31a8660fe433f36aa`。
完整源 XML 中 **617 条旧绝对资产路径**仍指向 `rm_uav_lab_m600/...`；
对应文件实际在当前 `rm27/simulation/field/...` 下。实现只在输出副本迁移这个明确前缀，
逐个 hash 原始资产，加载并对比 **801 个保留 geom 的位置、朝向、尺寸与 collision masks**。
移除 4 架预览机体后只加入一架活动研究无人机；没有重新造假场地，也没有搬动原始资产。

当前 slot_1 为 `(0.651751442, 1.082427939, 1.74)` m；目标为
`(-11.468, 0.02, 1.0758)` m，法向约 `(0.4617, 0, 0.887)`。
起点高于 slot 0.23 m，巡航高度 2.6 m，返回点高于 slot 0.6 m。
往返路径复用已有走廊方向，使用 p/v/a/j 连续的七次分段曲线，中间节点速度非零。
terminal 入口保持 `vx=-0.3 m/s`，在**同一个 plant 和连续状态**上接管；马达估计和控制器历史也转移。

**几何纠正：**第一轮局部 patch 的 90 mm 长轴沿斜面；当前 `base_target.add()` 的
90 mm 长轴沿 world Y，斜面方向只有 55 mm。新增 `target.in_plane_rotation_deg`，
默认 0 保留旧结果，arena 为 90°。矩形的轴向与无人机的目标推力方向分开处理。
下面两组几何内各自公平比较，不能直接拿旧几何的成功率当完整基地结果。

基地塔身的原始 visual-only mesh 仍不参与物理碰撞。可接触板来自当前 root proxy；
外围半透明 forbidden rim 是显式研究假设。不能据此宣称 `base2.STEP` 已有认证碰撞语义。
完整场景依赖本机已有的 ignored assets；仅 clone Git 不保证包含它们。
缺失／hash 不符会中止并留 traceback，不自动用简化模型替代。

## 3. 为什么 v1 在 stress 下退化：配对干预

在第一轮的历史 stress 种子 `202000..202031` 上冻结 v1，保持同一随机抽样，
每次只恢复指定物理量为 nominal。这些种子现在是机制研究数据，**不是新的留出证据**。
每个条件 32 次，共 192 次，完整结果在
[causal summary](../artifacts/contact-v2-causal-20260924/summary.json)。

| 干预 | joint PASS | valid first contact | RECOVERY_TIMEOUT |
|---|---:|---:|---:|
| 不干预 | 12/32 | 25/32 | 16 |
| 仅 nominal motor lag | 22/32 | 27/32 | 3 |
| 仅 nominal thrust + mass | 16/32 | 32/32 | 16 |
| 仅 nominal target pose | 12/32 | 25/32 | 16 |
| 仅 nominal bumper offset | 15/32 | 见原始 summary | 见原始 summary |
| nominal lag + thrust + mass | 32/32 | 32/32 | 0 |

这支持两个可区分的瓶颈：lag 主要影响撞后按时恢复，推力／质量失配主要影响接触轨迹漂移。
在本样本、规定的小 target-noise 范围内，目标误差不是主导项。
这不等价于真实相机误差可以忽略，也不是 motor lag 已被真机测量。

## 4. 实现的改进与公平比较

保留第一轮配置／门槛和默认控制行为。新增三个可解释方法：

| 方法 | 与 baseline 的差异 |
|---|---|
| `nominal` | 原固定连续轨迹 + 原 feedback |
| `v1` | 冻结第一轮 recoverability-aware 参数 + 原 feedback |
| `actuator_only` | nominal 轨迹 + command-history motor observer／补偿 |
| `adaptive_nominal` | 上项 + bounded effective thrust/mass adaptation |
| `robust_v2` | 上项控制器 + margin-aware scenario 参数选择 |

马达预测：`fhat_next = exp(-dt/tau)*fhat + (1-exp(-dt/tau))*u_prev`，
`tau=40 ms` 为假设；命令 `u = clip(f_des + 0.5*(f_des-fhat))`。
不读取真值马达状态或本次随机 lag。有效推力／质量比由 ego velocity 差分投影至机身推力轴估计，
相对预测 collective thrust 低通，时间常数 0.12 s，范围 `[0.7,1.3]`。
首次 contact 后冻结，避免把冲量误认为推力变化。它不分别辨识推力和质量。
actor 不读取随机物理参数；测试验证相同观测／历史在不同 hidden plant 下输出一致。
**当前 ego velocity 仍是真值，其差分对真实 estimator 噪声／延迟的适应性尚未验证。**

v2 搜索 48 个候选，每个用 `301..308` × small/stress 两级，共 768 次。
目标包括联合失败、无效接触、12 mm signed region margin 缺口、位置误差、恢复时间和 post-rate；
风险聚合为 mean loss + 0.5 × worst-quartile mean。
优化 duration、端点速度／推力、penetration、斜面 aim offset、retreat/lift。
训练几何为 arena patch，选中候选 **2**，参数先冻结再评价。
这不是全局优化、learned viability function，也没有证明某个单独 cost 项的因果收益。

新留出 small 种子 `432000..432063`，stress `442000..442063`；
五方法 × 两几何 × 两分布 × 64 = 1,280 次，相同 draw、相同门槛、失败不删。
两几何复用种子，不能视为独立样本后混合扩大统计显著性。
设置：[aggressive_contact_v2.json](../configs/aggressive_contact_v2.json)；
原 v1 baseline 与 selected nominal 的最终代码逐帧回归误差均为 **0**。

## 5. 新留出结果

| 方法 | 旧 patch small | 旧 patch stress | arena patch small | arena patch stress |
|---|---:|---:|---:|---:|
| nominal | 56/64 | 32/64 | 55/64 | 26/64 |
| frozen v1 | 61/64 | 30/64 | 45/64 | 21/64 |
| actuator_only | 64/64 | 62/64 | 62/64 | 50/64 |
| adaptive_nominal | 64/64 | 64/64 | 64/64 | 64/64 |
| robust_v2 | 64/64 | 64/64 | 64/64 | 64/64 |

![Paired held-out results](../artifacts/contact-v2-plots-20260923/comparison.png)

与第一轮 43/64、49/64 等数值不同，是因为这里使用**新的评价种子**；没有替换旧报告。
arena stress 的 v2 相对 nominal 为 +59.375 pp，paired bootstrap 95% `[46.875,71.875]` pp；
相对 v1 为 +67.1875 pp，区间 `[54.6875,78.125]` pp。
相对 adaptive_nominal，样本成功差为 0；不能据 bootstrap `[0,0]` 宣称总体方法等价。
单项 64/64 的 Wilson 95% 下界约 **94.34%**，不是“保证不会失败”。
分布由工程假设定义，不代表真实世界成功率。

arena stress 中位数／边缘尾部：

| 指标 | nominal | v1 | actuator_only | adaptive_nominal | robust_v2 |
|---|---:|---:|---:|---:|---:|
| 接触 region margin P05 (mm) | -6.01 | -20.35 | -6.01 | 8.74 | 16.27 |
| 接触位置误差中位数 (mm) | 14.55 | 19.32 | 15.92 | 11.58 | 7.64 |
| 实际法向速度中位数 (m/s) | 0.748 | 1.106 | 0.729 | 0.720 | 0.651 |
| 恢复进入时间中位数 (s) | 1.564 | 1.417 | 0.810 | 0.797 | 0.600 |
| post-contact 峰值角速度的中位数 (rad/s) | 7.060 | 5.266 | 6.842 | 5.957 | 4.688 |

恢复时间统计只包括实际进入恢复集合的 trial，nominal/v1 有缺失和超时，不能单凭中位数排名。
adaptive/v2 该 split 均全通过，v2 恢复时间中位数约低 25%、post-rate 约低 21%。
其实际撞击速度也较低，是动作选择带来的折中；没有证明在等撞击能量下仍有同样收益。
没有已知裁判伤害阈值，不能把较慢接触解释为同等判伤能力。

**负结果与被修正的假设：**

- 只优化撞后代价不足。v1 换到斜面方向更窄的真实 patch 后，small 也退化到 45/64。
- 只补偿 lag 不足。arena stress 虽然恢复明显改善，但仍有 14 次 `UNDESIGNATED_CONTACT`；有效 first contact 只有 52/64。
- 正确 first contact 不保证整段接触安全。开发 seed302 的 adaptive nominal 先有效接触，余量仅 4.12 mm，随后发生 forbidden contact；三个步长下均失败。v2 在这个开发情景通过。
- 场景搜索并非成功率提升的必要条件：simple adaptive_nominal 已在当前新留出集达到同样计数。
- 矩形内的点余量不是完整 bumper swept-volume 证书。12 mm 是搜索目标，并非严格约束；v2 stress 最小实际余量为 11.85 mm。

可观看失败：`evaluation/arena_patch/stress/v1/seed-442002` 首先碰 `rim_y_-1`，
region margin 为 -22.02 mm。所有失败的 state/control/model/event 均保留在 campaign 下。

## 6. 完整场地任务及验证

| 运行 | 结果 |
|---|---|
| `contact-arena-batch-20260923` | adaptive nominal：无扰动 + seeds501–503 small + seeds504–505 stress，**6/6 PASS**；仅小规模开发任务集 |
| `contact-arena-selected-v2-20260923` | 优化参数完整任务 **PASS**，32 s，接触速度 0.662 m/s，margin 25.33 mm，0.570 s 进入恢复集合并保持 1 s |
| `contact-arena-selected-replay-20260923` | 6,400 条记录马达输入回放，position/velocity/motor-state 最大误差 **0** |
| `contact-arena-selected-audit-20260923` | 686 个 retained mesh，往返 499 个保守扫掠球采样，无 triangle surface 命中 |
| `contact-arena-selected-render-20260923` | EGL 生成全场、接触、恢复、返航图及 GIF，已实际检查 |
| `contact-v2-numerical-20260923` | 两新方法 × 三条件 × 1/.5/.25 ms；control 保持 5 ms；18 次，全部相邻步长检查通过 |
| `contact-v2-legacy-regression-20260923` | 原 nominal/v1 selected trace 逐帧完全相等 |
| `contact-v2-final-verification` | 根测试日志／最终源代码与状态记录 |

selected mission 往返最大 reference tracking error 3.66 mm，return 位置误差约 0.05 mm，
满足 **0.1 m** 返回门槛。这些是理想 MuJoCo truth 反馈数值，不是视觉定位或真实飞行精度。
没有用 return 误差作为毫米级导航能力的证据。
route audit 的机体包络半径 85.98 mm，再加约 57.13 mm 采样间隔膨胀；
只检查记录的 OUTBOUND／RETURN，**排除 intended-contact terminal**，不证明 mesh 内部实体或真实场地净空。

步长检查同一成功／失败分类，相邻步长最大 impulse 相对差约 1.28%，峰值力差约 3.64%，
在预设 5%／10% 数值检查门槛内。这里不把未知接触材料变成“测得真实值”。
root tests **64 passed**，新增 6 项覆盖真实板轴向、actor hidden-state 隔离、曲线连续性、
非零时间接管、接触时冻结估计、viewer 私有状态和完整场地保留（部分合并在同一测试）。

## 7. 修改、运行和证据位置

新文件：`arena.py`（场地组合／任务），`viewing.py`（live／记录播放／EGL），
`arena_audit.py`（表面净空审计），`robust.py`（干预／scenario search／数值检查），
`configs/contact_arena.json`、`configs/aggressive_contact_v2.json`、`tests/test_contact_arena.py`。
扩展 `control.py`、`model.py`、`experiment.py` 的时间／参考／板方向／observer／日志接口；
扩展 CLI、plot、evidence snapshot，更新根和模块入口文档。没有新增依赖。
参数来源仍为 NOMINAL／ASSUMPTION，没有写任何硬件参数。

实际执行的主要命令（输出目录已存在；复现要更名）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena --output artifacts/contact-arena-full-v2-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena --parameters artifacts/contact-v2-confirmatory-20260924/selected_parameters.json --output artifacts/contact-arena-selected-v2-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact validate-v2 --input artifacts/contact-v2-confirmatory-20260924 --output artifacts/contact-v2-numerical-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact replay --input artifacts/contact-arena-selected-v2-20260923 --output artifacts/contact-arena-selected-replay-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact arena-audit --input artifacts/contact-arena-selected-v2-20260923 --output artifacts/contact-arena-selected-audit-20260923
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render --input artifacts/contact-arena-selected-v2-20260923 --output artifacts/contact-arena-selected-render-20260923
MPLCONFIGDIR=/tmp/rm27-mpl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact plot --input artifacts/contact-v2-confirmatory-20260924 --output artifacts/contact-v2-plots-20260923
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

长批次实际通过同一 Python API 执行 `robust_campaign(load_config(), output)` 与
`causal_study(load_config(), output)`；等价可复制 CLI：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact robust-campaign --candidates 48 --trials 64 --output artifacts/contact-v2-new-campaign
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact causal-study --trials 32 --output artifacts/contact-v2-new-causal
```

随机场地例：`arena --seed 504 --scale 2 --output artifacts/contact-new-stress-mission`。
局部 v2 复现要同时使用 campaign 的 `selected_config.json` 和 `selected_parameters.json`，
因为参数文件不包含 controller／rectangle orientation。
campaign `passed: true` 表示 runner 无异常且 selected nominal demo 通过，不表示所有方法／情景通过。
目录中 `20260924` 只是本轮 artifact ID，不从目录名推断日期或成功。

主统计及配对区间：[campaign summary](../artifacts/contact-v2-confirmatory-20260924/summary.json)。
每次 trial 的真实编译模型、physical draw、参考／状态／控制、events、失败原因都保留；
训练选择在留出评价前冻结。final-verification 保存最终文件 hash、运行日志和嵌套仓库状态。
根 branch／HEAD 不变；其他 SLAM worktree 和 nested dirty changes 未重置／清理。

## 8. 证据边界与下一条研究线

IMPLEMENTED／RUN／TESTED：场地组合、闭环任务、两种 viewer 接口、observer/adaptation、
scenario search、配对干预、评价／绘图／回放。SIM-VALIDATED：上述明确任务和分布／门槛。
桌面 GUI 只完成接口测试及 EGL 视觉核查，未声明桌面实机观看验证。
HARDWARE-VALIDATED：**无**。ArduPilot SITL：本轮没有运行。视觉：没有接入。

剩余最关键的不确定性是：**这个依赖 ego velocity 差分的有效推力估计，在有延迟、带噪、
碰撞时可能失真的机载估计下，是否还能保留当前收益？**
下一步优先在保持同一 evaluator 的情况下加入 timestamped ego/target estimator replay，
比较 observer/adaptation 与固定控制器对 delay/noise/dropout 的敏感性，再做 IMU specific-force
版本和 ArduPilot rate/thrust 内环匹配。当前结果不足以支持先扩大神经网络或直接真机高速碰撞。
保持简单 adaptive controller 作为新参考；scenario optimization 的价值目前主要是提高余量和恢复品质。
