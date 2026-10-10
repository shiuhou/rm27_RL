# 从机库到接触、恢复：整段动态 maneuver

2026-09-23；工作区 `astra/research`，root HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。
本轮在现有完整 RMUC MuJoCo 场地、车辆和接触评价上继续实现，没有替换场地或修改飞控子仓库。

**现在优化的是从机库上方空中初态，到有效接触、脱离并保持恢复状态的总时间。**
不再先用固定 12 秒走到一个慢速接管点，再独立调用接触轨迹。
得到两个已运行的选择：快速版本 nominal **5.672 s**；更早完成姿态准备的版本 **7.097 s**。
重建的原 v2 分段参考约 **14.848 s**。计时都包含同样的 1 秒稳定保持，而非只计到碰板。

任务起点仍为现有 slot_1 上方 0.23 m 的**空中状态**，不是停桨起飞；终点是恢复确认，
本轮计时不含返航／降落。原 `arena` 往返入口保留。所有结果都是 truth-assisted、直接马达控制，
没有 SITL、视觉或真机结论。

## 实现变化

新增 `maneuver.py`：完整进场采用三段七次多项式，在既有走廊的两个内部位置节点上，
**联合求解速度、加速度、jerk**，最小化整条路径积分 squared snap。
节点不是 hover 约束；位置／速度／加速度／jerk 连续，也没有接管点状态重置。
各段时间、节点位置／高度、接触端点速度／推力、接触瞄准偏置、恢复 retreat/lift 一起参与外层搜索。
控制器仍是 v2 actuator observer + thrust adaptation + geometric feedback。

此外加入一个可选的**提前对准区间**，其推力方向由动力学约束产生。令接触端点为
`(p_f,v_f,a_f,j_f=0)`，其中 `a_f=alpha*n-g*ez`；在接触前 `h` 秒开始的参考状态为：

```text
p_align = p_f - v_f*h + 0.5*a_f*h²
v_align = v_f - a_f*h
a_align = a_f
j_align = 0
```

全局曲线直接满足这个区间的入口导数，随后沿恒定加速度弧线保持目标推力方向。
这让姿态准备改变整个进场轨迹，不是最后额外播放旋转动作。
实际接触由 MuJoCo 物理事件触发，通常早于参考末端。

碰撞的速度跳变仍由物理模型产生。接触后沿用事件反馈，恢复偏移参数在**整个任务的成本函数**中优化。
这里没有假装能把冲量前后连成物理上 C3 的轨迹，也没有新建独立的“到达后再救机”优化阶段。
但恢复反馈律本身仍是已有模型控制器；尚未实现自由形式的 post-impact trajectory optimization。

`maneuver` demo 在真实事件满足恢复条件并保持 1 秒后结束。
批次则保留较长观察窗口（计划接触时刻后 4.5 秒）继续检测失败，成本仍使用首次恢复确认时间；
因此不能把批次的固定观察窗口误报为动作时间。旧 episode/arena 默认行为保持不变。

## 公平比较与选择

同一完整场地、物理扰动、控制器、接触门槛及恢复判据：

| 方法 | 优化范围 |
|---|---|
| staged_v2 | 原 12 秒 transit + 独立局部接触参考，重建其状态接管 |
| compressed_staged | 只压缩 transit 时间，仍以 0.3 m/s 进入同一局部动作；六种时间尺度搜索 |
| joint_contact_only | 全局曲线候选，只用接触有效性／接触时间选择 |
| joint_recovery / fast | 同一全局候选，用联合成功、到恢复确认总时间、余量及姿态准备代价选择 |
| early_pose | 从同一训练候选中选择更早建立姿态的时间／准备程度折中 |

所有方法保留原接触／恢复门槛，增加共同的研究飞行包络：速度 ≤9 m/s、倾角 ≤65°、高度 ≤4.5 m。
这些是可配置实验约束，**不是已知比赛限值或真机极限**。短时马达饱和仍按原门槛评价。

训练：36 个全局候选 + 6 个分段时间尺度，各用 `(601,1×),(602,1×),(603,2×),(604,2×)`，
共 **168 次**。成本为 mean + 0.5 worst-case；联合失败惩罚优先于时间。
early_pose 在训练上要求全部成功、接触前 0.5 秒姿态误差均值 ≤10°、每次持续准备时间 ≥0.25 秒，
再选择最短 mean + 0.5 worst-case 总时间。其规则只用训练结果设定，并在该方法评价前冻结。

新留出：small `552000..552023`，stress `562000..562023`。
主比较 4 方法 × 2 分布 × 24 = **192 次**；early_pose 另 **48 次**，合计 **240 次**。
方法间使用相同 draw；失败保留在分母。训练只有四个情景，不能推断广泛鲁棒性。

选择结果：分段时间尺度 **0.5**；joint_contact_only 和 joint_recovery **都选中候选 24**；
early_pose 选中 **16**。因此本轮**没有证据证明加上恢复成本改变了候选选择或提高成功率**。
主要收益来自整段路径／导数／时间分配，提前姿态约束则得到另一个可解释的折中。

## 定量结果

下表为留出集统计，时间是从机库上方初态到恢复确认的中位数。
fast stress 中位数包括那次已恢复但发生禁碰的失败 trial，成功计数单列，不能忽略该失败。

| 方法 | small PASS | small 时间 | stress PASS | stress 时间 | small 姿态准备提前量 |
|---|---:|---:|---:|---:|---:|
| 原 staged_v2 | 24/24 | 14.944 s | 24/24 | 14.855 s | 0.0185 s |
| 仅压缩 transit | 24/24 | 8.867 s | 24/24 | 8.856 s | 0.0140 s |
| 全局 fast | 24/24 | **5.768 s** | **23/24** | **5.679 s** | 0.193 s |
| 全局 early_pose | 24/24 | **7.103 s** | 24/24 | **7.101 s** | **0.405 s** |

joint_contact_only 与 fast 完全相同，不重复列出。24/24 的 Wilson 95% 下界仅约 86.2%，
不能称为可靠性已达 100%。两种全局动作之间也没有足够样本证明总体成功率差异。

姿态准备定义为：到实际接触前一直保持 `body +Z` 与真实板法向误差 ≤10°，且机身角速度 ≤1 rad/s。
它是独立观测指标，不是把 controller phase label 当作完成姿态。
在接触前 0.5 秒，small 集的姿态误差／飞行速度中位数分别为：

| 方法 | 姿态误差 | 速度 |
|---|---:|---:|
| staged_v2 | 45.88° | 0.939 m/s |
| compressed_staged | 46.12° | 0.919 m/s |
| fast | 25.21° | 2.522 m/s |
| early_pose | **7.63°** | **2.692 m/s** |

early_pose 的 stress 准备提前量中位数 0.402 s、最小 0.283 s；接触前 0.5 秒误差最大 14.71°，
说明训练上的均值条件不是每个测试 trial 都满足 10° 的保证。
从起始 0.5 秒后到接触前 0.5 秒，速度低于 0.4 m/s 的时间，staged_v2 中位数为 1.773 s，
compressed_staged 0.880 s，fast **0**。优化确实消除了慢速接管造成的时间损失。

![Whole maneuver comparison](../artifacts/maneuver-comparison-plots-01/whole-maneuver.png)

small 集 early_pose 相对原分段方法，配对平均省时 7.844 s，bootstrap 95% `[7.804,7.883]` s；
相对仅压缩 transit，中位数再降低约 **19.9%**。fast 比原分段方法中位数短约 **61.4%**，
比仅压缩 transit 短约 **35.0%**。统计描述仅适用于本仿真假设和固定任务。
配对时间区间仅使用两方法都 PASS 的 trial；完整失败计数不从成功时间统计中删除。

## 失败、代价和未被证明的主张

- `maneuver-aligned-development-01`：首次尝试 0.45 s 提前对准，姿态已好，但原瞄准偏置导致
  bumper 同时碰 armor 与 rim，失败。提前对准改变整个接近动力学，不能简单沿用局部动作的接触瞄准点。
- 更激进的分段压缩（0.4、0.3、0.25）在训练中均未通过全任务判据。
  0.4 的主要问题是接管附近越过共同的 65° 倾角包络；部分 trial 仍完成接触恢复。
  因而不能宣称分段方案“物理上绝对救不回来”；它违反了本轮声明的额外飞行限制。
- fast stress `seed-562010` 正确首次接触，但余量仅 **4.34 mm**，随后出现 `UNDESIGNATED_CONTACT`。
  不能把其 5 秒级完成时间包装成无代价改进。整段任务仍需考虑有限接触件的扫掠体积。
- 全局最小 snap 不等于全局最短时间最优解。节点位置／时间只进行了有限候选搜索。
  early_pose nominal 峰值高度 3.615 m，fast 3.295 m，比原分段路径更高；提前对准版本还形成明显的下降弯曲。
  未证明这些路径是能量最优、视野最优或最符合规则的路线。
- 接触后的反馈结构仍相同。joint_contact_only 与 recovery-aware 选中同一候选，是明确负结果；
  本轮不把整段提速归因于“恢复目标已被证明更优”。

## 实际运行和观看

推荐观看 early_pose：它在同一整段轨迹中提前建立姿态，仍显著快于原方案。
从仓库根目录，在桌面终端整行粘贴：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver --parameters configs/contact_maneuver_early_pose.json --view --output "artifacts/maneuver-view-$(date +%Y%m%d-%H%M%S)"
```

快速版本替换参数文件为 `configs/contact_maneuver_fast.json`。
Space 暂停、V 切换视角。去掉 `--view` 为 headless。新目录不可覆盖旧实验。
重看已验证动作（记录状态，不是新闭环运行）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/maneuver-early-demo-01/flight --speed 0.5 --loop
```

[完整动作动画](../artifacts/maneuver-early-render-01/maneuver.gif) ·
[接触瞬间](../artifacts/maneuver-early-render-01/contact.png) ·
[主要结果 JSON](../artifacts/maneuver-confirmatory-01/summary.json) ·
[early-pose 结果 JSON](../artifacts/maneuver-early-pose-01/summary.json)

实际执行的主要命令／等价批次入口（以下目录已有结果，复现请更名）：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-campaign --output artifacts/maneuver-new-campaign
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-pose-study --input artifacts/maneuver-new-campaign --output artifacts/maneuver-new-pose
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver --parameters configs/contact_maneuver_fast.json --output artifacts/maneuver-fast-demo-01
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver --parameters configs/contact_maneuver_early_pose.json --output artifacts/maneuver-early-demo-01
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-validate --output artifacts/maneuver-numerical-01
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-audit --input artifacts/maneuver-early-demo-01/flight --output artifacts/maneuver-early-audit-01
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact replay --input artifacts/maneuver-early-demo-01/flight --output artifacts/maneuver-early-replay-01
MPLCONFIGDIR=/tmp/rm27-mpl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact maneuver-plot --input artifacts/maneuver-confirmatory-01 --pose-input artifacts/maneuver-early-pose-01 --output artifacts/maneuver-comparison-plots-01
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render-maneuver --input artifacts/maneuver-early-demo-01/flight --output artifacts/maneuver-early-render-01
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

主 campaign 实际通过同一 Python API 调用 `campaign(load_config(), 'artifacts/maneuver-confirmatory-01')`。
baseline/reference、控制器和已有报告已复查；本轮新增研究模块、记录模型增量、CLI 和图表，
没有清理其他工作区修改或提交 Git。场地 source hash／801 个保留 geometry 验证沿用 v2 compositor。

## 验证与下一步

- 根测试 **69 passed**。新增覆盖全局多项式、内部 p/v/a/j 连续性、提前对准的推力轴动力学、
  model overlay 精确重建、事件终止后的完整恢复保持和马达回放。
- 两种全局动作 × nominal/一个 stress × 1/.5/.25 ms，共 **12 次全场步长试验通过**。
  control 保持 5 ms；相邻步长恢复确认时间差 ≤2 ms，impulse 相对差最高约 4.59%，峰值力差约 3.64%。
- 记录马达输入回放：early demo 的 **1,421** 个状态样本完全相等；compact trial 回放也完全相等。
  大型 270 MB 场地模型只在 campaign 保存一次，每个 trial 保存 hash 引用和完整变动物理数组，
  viewer/replay 自动重建；保留失败可复现性而不重复写数十 GB 相同 mesh。
- 两个 nominal demo 的进场 mesh surface 审计均通过，检查 686 个 mesh。
  **审计排除最后 0.5 秒与恢复段**；保留原碰撞 mask，基地 visual-only tower 没有被伪称为物理碰撞。
  这不是实体内部／真机净空认证，也没有逐个审计所有随机 trial 的 visual-only mesh。
- 实际执行 headless simulation 与 EGL 图像核查；桌面 viewer 没有在本轮启动，SITL／视觉／硬件均未验证。

下一步最有价值的是在这个**整段任务时间—提前准备—接触余量**权衡上加入可见性／估计延迟与
完整 swept-volume 约束，而不是回到固定慢速 staging point。fast 的一次 stress 禁碰表明：
要进一步缩短任务时间，需要约束整个接触期间的空间余量，不能只优化 first-contact 点或某一个瞬间姿态。
