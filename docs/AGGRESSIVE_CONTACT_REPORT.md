# RM27 aggressive contact / recovery 实现与实验报告

> 本文保留第一轮的局部实验与结果。第二轮的真实场地组合、测试片轴向核对、
> v1 stress 因果干预和 actuator-aware 改进见
> [AGGRESSIVE_CONTACT_V2_REPORT.md](AGGRESSIVE_CONTACT_V2_REPORT.md)。

日期：2026-09-23。已完成一个可运行的六自由度 MuJoCo 局部闭环：
**空中初态 → 连续进场 → 底部接触件首先接触倾斜板 → 脱离 → 恢复**。
实现了 baseline、恢复感知参数优化、独立恢复区域实验、自动评价、轨迹与马达回放。
不是只生成参考曲线；没有在运行中改写位置、速度或施加外部“救机”力。

主要研究结果：恢复感知优化在小扰动留出集把联合成功率从 **43/64 提高到 49/64**；
在两倍扰动下只有 **26/64**，没有胜过 baseline。它改善了特定分布中的恢复，
但牺牲了接触位置余量，不能称为已经解决鲁棒接触，更不能迁移为真机结论。

## 1. 当前仓库核对与插入位置

工作目录 `/home/shiuhou/Projects/rm27_drones`；branch `astra/research`；
HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。已有另外两个 worktree：
`../rm27_drones_slam` 和 `../rm27_drones_slam_future`。未切换分支、创建提交、
清理或覆盖其他研究工作；在现有 research branch 中使用全新模块隔离。

实际检查的当前资料/入口：

- 根 `AGENTS.md`、`README.md`、`START_HERE.md`，`docs/INTEGRATION.md`、
  `docs/MILESTONES.md`、`docs/BASE_TARGET.md`、`docs/REAL_HARDWARE.md`。
- `configs/drone.json`、`configs/base_target.json`，
  `rm27/integration/drone.py`、`rm27/simulation/micro_plant.py`、
  `rm27/autopilot/micro_flight.py`，重点检查状态、motor、坐标和 guidance 路径。
- `rm27/experiments/contact_probe.py`、`contact_metrics.py` 和现有接触测试。
- `research/astra/README.md`，旧 M4-A benchmark 的模型/控制入口，
  `rm27/perception/vision/docs/SLAM_RECOVERY_AND_AUDIT.md`。
- Git HEAD、status、worktrees、submodule pins，以及各嵌套仓库的独立 status。

开始前根测试 **50 passed**。已有 baseline、ArduPilot、legacy 子仓库有本地改动，
field 有未跟踪环境；这些状态被保留。旧 Astra 是推力向量近似和 observation-only
guidance；旧接触 probe 是独立球体测量。它们都不是现成的底部接触恢复能力。
SLAM 标定仍未完成，本实验没有依赖它。

`docs/BASE_TARGET.md` 的部分文字仍描述旧正 X 目标；本轮采用更新的
`configs/base_target.json` 蓝方目标法向 `(+0.4617, 0, +0.887)`，只平移到局部坐标。
没有重写历史文档或场地。brief 提及的几份状态汇总、Perching/BeeNAV/MINCO PDF
在本轮当前仓库文件检索中未找到；本报告不把 brief 中的论文数字当作亲自复现的证据。

## 2. 实际交付文件

| 文件 | 作用 |
|---|---|
| `rm27/experiments/aggressive_contact/model.py` | 复用当前微型机生成器；补充接触几何、局部板、估计接口与显式随机化 |
| `control.py` | p/v/a/j 约束七次轨迹、SO(3) 反馈、马达分配、接触事件状态机 |
| `experiment.py` | 独立接触/恢复评价、失败分类、逐步状态/控制记录 |
| `campaign.py` | 同候选集的两种目标函数、独立训练/评价种子、配对统计 |
| `recovery.py` | 可批量初始化姿态、速度、角速度、高度和马达状态的恢复地图 |
| `studies.py`、`ablation.py` | 步长收敛、单因素挑战、接触后 rate feedforward 消融 |
| `visualize.py`、`evidence.py`、`__main__.py` | 科学图、马达输入回放、版本/源代码快照、失败 traceback 和 CLI |
| `configs/aggressive_contact.json` | 全部实验门槛、动力/接触假设、随机化范围与优化配置 |
| `tests/test_aggressive_contact.py` | 8 项新增测试；含真实物理接触、恢复、回放和失败保留 |
| `requirements-contact-research.txt` | 现有依赖加 SciPy，无 RL/GPU 框架 |
| 模块 `README.md`、根 `README.md`、`START_HERE.md` | 运行入口、边界与复现说明 |

没有修改现有 vehicle generator、接触测量函数、飞控桥接、视觉代码或子仓库。
新增模块使用它们已有的模型/测量职责，没有复制出一个脱离 RM27 的 toy plant。

## 3. 技术选择与明确假设

选择 **末端状态约束轨迹 + 几何反馈 + empirical recoverability-aware scenario search**。
理由是先把“接触准”和“撞后恢复好”分离成可检验的目标函数差异，同时保留便宜、
可解释的闭环。没有为使用 AI 而加入神经网络，也没有把本方法称为全局最优。

固定初始/末端 p/v/a/j 和时间后，用唯一七次多项式生成连续参考；其加速度决定
推力轴，末端 `a = alpha*n - g*ez`，COM 终点由底部接触件偏置反推。
实际 first contact 是物理事件，通常早于参考终点，因此**实测接触速度不等于
配置的末端参考速度**。没有独立叠加姿态动画。

保留根模型的 180 g、惯量估计、四马达位置、2.5 倍推重比、40 ms lag、护栏碰撞。
新球形 bumper 半径 9 mm，中心在机体下方 42 mm；总质量预算仍为 180 g，
其结构质量分布、柔顺结构、损伤及桨气动没有建模。板为 90 × 55 mm 的物理测试片，
法向倾角约 27.5°；外围 forbidden rim 是研究假设。MuJoCo 软接触参数不是实测材料参数。

使用 1 ms physics / 5 ms control。actor 接收 `VehicleEstimate`、`TargetState`
和 latched contact bit，输出四路牛顿单位 rotor force，经过原有马达滤波进入 MuJoCo。
本轮 ego estimate 和接触位是真值；目标为启动时取得并 commit 的静态带误差估计。
只有启动时 stale 检查，没有持续视觉更新、FoV 或 estimator reset。

当前 root SITL 入口固化为 500 Hz、四实体场景和已有 position/velocity guidance。
本轮复用其 vehicle generator，另建研究控制循环，以免为末端姿态/接触研究重写桥接。
这不是判定 ArduPilot 不稳定；**本轮没有运行 SITL 飞行，也没有实现可直接发送到
ArduPilot 的 attitude/rate adapter**。未来需重做坐标、时序、推力语义和内环回归。

## 4. 运行与证据

所有主要实验均为 **headless MuJoCo、闭环直接马达控制、truth teacher**。
主要 artifact 目录均有实际读取的 `summary.json`，未按目录名称判断成功。

| 目录（相对于 `artifacts/`） | 实际内容 |
|---|---|
| `contact-dev-01` | 最初过高姿态增益的失败轨迹，保留了持续振荡/恢复超时 |
| `contact-development-02` | 24 候选、4 训练种子、每条件 12 次的开发实验，不作为最终留出证据 |
| `contact-confirmatory-20260923` | 48 候选 × 8 训练情景 = 384；3 方法 × 2 分布 × 64 = 384 留出实验；另有 nominal selected demo |
| `contact-nominal-demo-20260923` | 默认 baseline 的完整成功轨迹 |
| `contact-recovery-map-20260923` | 75 网格初态 + 24 多轴随机初态 |
| `contact-validation-20260923` | 27 次步长实验 + 66 次单因素挑战 |
| `contact-rate-ablation-20260923` | 4 种 rate feedforward 设置 × 8 开发种子 |
| `contact-nominal-replay-20260923`、`contact-selected-replay-20260923` | 记录马达输入重新推进物理，1200 个采样点对比 |
| `contact-verification-20260923` | 最终测试日志、最终源代码快照、嵌套仓库状态、当前默认行为与冻结运行逐帧对比 |

训练种子 `101..108`；最终 small evaluation `192000..192063`；stress
`202000..202063`。开发实验的 `92000..`/`102000..` 没有混入最终留出集。
方法选择文件先于留出实验写入；本轮没有根据最终留出结果重选参数或放宽门槛。
所有失败 trial 留在分母和磁盘中。物理模型二进制保留运行时 mass/gear/lag 改动；
随机场景不能仅用 nominal XML 重建。源代码内容/hash、配置、seed、版本均在父目录。

实际运行的主要命令（下列输出目录已经存在，复现时换成新名称）：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact campaign --output artifacts/contact-confirmatory-20260923 --candidates 48 --trials 64
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact recovery-map --output artifacts/contact-recovery-map-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact validate --input artifacts/contact-confirmatory-20260923 --output artifacts/contact-validation-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact rate-ablation --output artifacts/contact-rate-ablation-20260923
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact replay --input artifacts/contact-confirmatory-20260923/demo --output artifacts/contact-selected-replay-20260923
MPLCONFIGDIR=/tmp/rm27-mpl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact plot --input artifacts/contact-confirmatory-20260923 --output artifacts/contact-comparison-plots-20260923
```

最小 working demo，一条命令即可运行，无需 CAD、GPU、SITL 或相机：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact episode --output artifacts/contact-your-new-demo
```

高级方法使用 `--parameters artifacts/contact-confirmatory-20260923/selected_parameters.json`。
可通过 `--config` 使用父 campaign 的冻结 `config.json`。逐项复现说明见
[模块 README](../rm27/experiments/aggressive_contact/README.md)。

## 5. 定量结果

联合 PASS 要求正确部位/正面/有效区域 first contact、法向速度在研究区间、
姿态/切向速度满足限制、离板至少 8 cm、无禁碰/二次碰撞，并在接触后 2 s 内
进入 tilt ≤10°、rate ≤30°/s、speed ≤0.3 m/s，保持 1 s；高度损失 ≤0.3 m。
还检查局部空间、电机连续饱和和过大角速度。这不是裁判判伤定义。

| 方法 | 小扰动 joint PASS | 95% Wilson 区间 | 两倍扰动 joint PASS | 95% Wilson 区间 |
|---|---:|---:|---:|---:|
| 名义轨迹 + feedback | 43/64 = 67.2% | 55.0–77.4% | 28/64 = 43.8% | 32.3–55.9% |
| 仅接触目标优化 | 44/64 = 68.8% | 56.6–78.8% | 29/64 = 45.3% | 33.7–57.4% |
| 恢复感知优化 | **49/64 = 76.6%** | 64.9–85.3% | **26/64 = 40.6%** | 29.5–52.9% |

小扰动：位置初态 σ=10 mm，目标误差 σ=3 mm / 1.5°，推力 ±6%，质量 ±3%，
motor lag ±15%，另有摩擦/接触 solver/接触偏置变化。stress 把这些变化相对于
nominal 放大 2 倍；范围全是工程假设，不能解释为真实失效率。

恢复感知相对名义方法提高 9.375 个百分点，配对 bootstrap 95% 区间
`[3.125, 17.1875]`；相对仅接触方法提高 7.8125 个百分点，区间
`[1.5625, 15.625]`。后者只有 5 对改善、0 对退化，样本仍小。
stress 相对仅接触方法为 -4.6875 个百分点，区间 `[-12.5, 3.125]`；没有泛化优势证据。

小扰动三个方法的 first-contact 有效次数都是 64/64，但联合任务只通过上表次数。
这直接说明“碰到正确位置”不能替代“能够按时恢复”。接触位置误差中位数为
10.29 / 8.55 / 15.12 mm；post-contact 峰值角速度的跨 trial 中位数为
6.26 / 6.00 / 4.59 rad/s。恢复感知方法用部分位置精度换取更好的恢复瞬态，
stress 时有效 first contact 降到 54/64，板边缘余量不足成为新的失败来源。

无扰动 demo 的逐次实测：

| 指标 | baseline | 恢复感知 |
|---|---:|---:|
| 实际接触法向速度 | 0.745 m/s | 1.102 m/s |
| 接触姿态误差 | 1.814° | 0.647° |
| 接触位置误差 | 9.10 mm | 12.76 mm |
| 进入恢复集合时间 | 1.635 s | 1.195 s |
| post-contact 最大角速度 | 6.45 rad/s | 4.38 rad/s |
| 接触后高度损失 | 2.26 mm | 2.75 mm |

两者都保持恢复状态至少 1 s、离板且没有二次接触。高级 demo 峰值飞行速度
2.064 m/s、最大倾角 38.16°、电机饱和比例 0；属于有限速度包络，不能称为
10 m/s 级高速碰撞。其改进并不是简单降低实测撞击速度，也没有证明唯一因果机制。

![Paired comparison](../artifacts/contact-comparison-plots-20260923/comparison.png)

轨迹/姿态/马达图：[selected trajectory](../artifacts/contact-comparison-plots-20260923/trajectory.png)。

## 6. 恢复区域、数值检查与负结果

恢复 runner 的 99 个初态有 **89/99** 达到其恢复判据。网格覆盖高度
0.6/1.2/1.8 m、pitch ±60°、pitch rate ±12 rad/s；随机组额外覆盖 roll、
三轴速度/角速度和初始马达推力。10 个高度损失超限、9 个恢复超时、2 个触地
（类别可重叠）。这是指定初态网格上的经验结果，不是随机接触后的 89.9% 成功率，
也不是完整六维 viability kernel。它与接触任务不同：没有板，恢复到初始位置附近，
不要求离板或 first contact，也不宣称飞行走廊已审查。

![Recovery map](../artifacts/contact-recovery-map-plots-20260923/recovery-map.png)

1 / 0.5 / 0.25 ms、控制周期固定 5 ms 的检查覆盖三方法 × 三初态。
最后两级最大法向冲量相对差 **0.343%**、峰值力相对差 **1.441%**，分类一致，
9/9 收敛检查通过。该证据限于所选案例，不是全部随机接触的 solver 认证。

单因素挑战发现：motor lag 从 40 ms 增至 52/64 ms，三方法都出现恢复超时；
推力变成 85% 会导致错误接触，115% 也可能因恢复瞬态而超时。60° 目标面出现
速度/切向滑移超限。提高推力并不能保证更好恢复。

额外测试了“接触后不要对跳变的期望姿态求导”的简单修正：

| 开发种子上的 rate feedforward 设置 | 联合成功 | post-contact 峰值角速度中位数 |
|---|---:|---:|
| 原方法 | 8/8 | 6.82 rad/s |
| blank 50 ms | 7/8 | 7.17 rad/s |
| blank 200 ms | 6/8 | 5.31 rad/s |
| 整个 recovery 关闭 | 0/8 | 4.27 rad/s |

关闭 feedforward 虽降低峰值角速度，却把恢复时间中位数变成 2.736 s，超过原门槛。
这项负结果被保留为可执行消融，未启用到主方法，也没有拿它重新选择最终留出结果。
这些可选实验开关加入后，最终代码的三个默认策略与冻结运行重新独立闭环执行，
逐帧 trace 差为零；证据见 verification summary。

## 7. 失败案例与测试状态

失败 trial 未删除，也没有通过放宽 2 s 门槛把失败改成成功：

- baseline 小扰动 `seed-192001` 正确碰到 bumper/armor，但恢复进入时间 2.040 s，
  判为 `RECOVERY_TIMEOUT`。
- 高级方法 stress `seed-202012` 同时碰到 armor 与 rim，first contact
  被判 `WRONG_FIRST_CONTACT`，还记录 `UNDESIGNATED_CONTACT` 和恢复超时。
  不能因为其中包含 armor 接触就算命中。
- 初始过高姿态增益试验持续振荡，最终降低到与 40 ms nominal motor lag
  更相容的带宽后才得到 baseline。失败原始记录保留在 `contact-dev-01`。

最终根测试 **58 passed**。新增 8 项检查覆盖末端导数与刚体接触点几何、
随机化与 controller 隐藏参数边界、物理 first contact + recovery、马达回放、
错过目标、stale estimate、短 bumper 导致护栏/机身先碰、联合目标函数和错误配置日志。
名义和高级马达输入回放的 qpos/qvel/motor-state 最大差均为 **0**。
这是 recorded-input replay，只支持给定输入的确定性，不替代独立闭环成功率。

完整日志与最终检查：[verification summary](../artifacts/contact-verification-20260923/summary.json)。
正式留出结果：[campaign summary](../artifacts/contact-confirmatory-20260923/summary.json)。
这里的 campaign `passed: true` 表示无 runner error 且 selected nominal demo 通过；
不表示全部随机飞行通过。recovery-map / validation 的 passed 范围也写在各自 summary。

## 8. 证据支持到哪里

**IMPLEMENTED / RUN / TESTED**：真实可执行代码、配置、批次、失败日志、绘图、
回放和测试均已交付。**SIM-VALIDATED**：默认和优化后的局部物理接触—脱离—恢复
demo 通过；随机化范围内的性能仅限上表，离高可靠完整任务仍有距离。
**HARDWARE-VALIDATED：没有。** S2–S5 未更改为 passed。

本轮支持：底部指定接触件在当前有限模型中可以先碰、离板、恢复；接触目标和
联合任务目标的选优结果不同；恢复可改善而接触精度变差；motor lag 和板边缘余量
是当前强敏感因素。没有证明底部方案优于所有侧面结构，没有学习到可部署神经策略，
没有证明结构能承受这些载荷。完整机库起降/返航、真实传感器、视觉、ArduPilot
内环、碰撞后的估计失效、正式判伤和机械合规均未验证。

下一步最值得做的一项研究是**带接触区域余量约束和执行器状态预测的鲁棒联合优化**：
把当前暴露的 motor-lag 恢复超时及板边缘侵占同时纳入约束，再使用全新的留出种子。
在这之前，增加网络规模或宣称 sim-to-real 收敛都没有当前实验依据。
