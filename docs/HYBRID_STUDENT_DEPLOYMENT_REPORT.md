# RM27 Phase 3 — Hybrid student analysis and deployment-oriented control

2026-09-23。工作区 `/home/shiuhou/Projects/rm27_drones`，branch `astra/research`，HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。本轮修改尚未提交，证据对应每个 artifact 的源代码快照，不能只用 HEAD 重建。

**已实现、训练并完成 1,698 次研究闭环评价，另有一次完整往返 demo。** 最终冻结留出为 744 次：新方案明显改善延迟和中等噪声，但没有提高无延迟小噪声的成功率。根测试 **91 passed**。所有结果仍是 **headless MuJoCo、synthetic truth-derived estimates、nominal rate-loop proxy**；有 EGL 图像、记录状态 viewer 和 motor replay，**没有 SITL、真实视觉或真机验证**。

## 主要结果

每个 maneuver 方法在完全相同的 24 个物理 draws、三档动作上评价，共 72 episodes；adaptive_nominal 只评一条 staged 路线，共 24。这里的“成功”继续使用原 evaluator，另报告至少 8 mm contact margin 的成功数。

| 方法 / 观测条件 | 原 hybrid | 改进 hybrid | 改进且 margin ≥8 mm |
|---|---:|---:|---:|
| 小噪声、无延迟 | 66/72 | 66/72 | 64/72 |
| 小噪声 +10 ms delay | 60/72 | 65/72 | 64/72 |
| 小噪声 +20 ms delay | 16/72 | 68/72 | 63/72 |
| 中等噪声 +10 ms delay | 0/72 | 62/72 | 58/72 |
| 小噪声、50 Hz estimates、10 ms delay、10% dropout | 未加入最终留出；validation 0/18 | 68/72 | 67/72 |

同一新 cohort：perfect-state/contact teacher **70/72**，margin-qualified **68/72**；adaptive_nominal **23/24**。上一阶段的 72/72 是另一个 cohort，不能当总体可靠率。本轮 teacher 的两个失败也保留了。

20 ms 条件救回 52 条、无回退；10 ms 条件救回 7 条但回退 2 条。以物理 draw 为 block、同时重采样其三档动作，10,000 次 paired bootstrap 的成功率差 95% 区间分别为 **[55.6, 86.1] 百分点**和 **[-4.2, 19.4] 百分点**。10 ms 的改善不能宣称统计上明确。小噪声无延迟失败的是完全相同的六条，但失败类别有所变化。bootstrap 只描述这批有限数据，不是部署可靠率保证；原 aggregate 中逐 episode 的 Wilson 区间没有处理三档共享 draw 的相关性。

证据：[完整 benchmark](../artifacts/phase3-results-01/benchmark.json)、[逐 trial CSV](../artifacts/phase3-results-01/trials.csv)、[配对分析](../artifacts/phase3-results-01/paired.json)、[比较图](../artifacts/phase3-results-01/holdout.png)、[轨迹与恢复瞬态](../artifacts/phase3-results-01/paired_trajectory.png)。

## 实际 architecture：学习部分究竟是什么

当前有效系统**没有**学习一个 `u_model + Δu_network` 电机残差。准确形式是：

\[
(p_r,v_r,a_r,j_r)=s_\theta(t,mode),\quad
u=\operatorname{RateThrustProjection}\{C(\hat x,\hat f,\hat\rho,p_r,v_r,a_r,j_r,\hat c)\}.
\]

- `s_theta`：用训练 demonstrations 的 reference p/v/a 拟合的 degree-5 B-spline。它只依赖时间和 aggression mode，不看即时 ego/target/contact。
- `C`：原 geometric position/attitude feedback、nominal motor-lag observer、effective thrust/mass adaptation。它们没有被学习替代。
- `c_hat`：由观测速度创新估计的 contact latch。事件后使用估计位置建立 recovery anchor，恢复控制仍是 model based。
- `u`：FLU body rates rad/s + collective physical force N。研究 plant 由 nominal rate-P mixer 把这个 action 变成 motor force。

因此 hybrid 的 state/phase/actuator/history 消融必须针对真正使用它们的 feedback / detector，不能把用于日志的 32 维 vector 清零后宣称某输入不重要。B-spline 的时间是 maneuver phase；contact 触发真实混合切换；actuator history 用于动力补偿。参考系数始终冻结，适应的是非学习的有效推重比估计。

原 teacher 本身也是紧凑 trajectory + feedback，在线并不昂贵。保留 teacher 原 trajectory、使用相同估计接触和反馈的 `model_reference` 对照 **12/12**，拟合参考也是 **12/12**。**没有证据证明当前 reference learning 优于直接保存 teacher trajectory，或者压缩了整套控制器。** 它证明了一个可复用的结构化 imitation 表示。

## Pure BC 失败的实验解释

### 离线 action 拟合没有复制反馈响应

原 train bank 48 episodes / validation 18 episodes，均按完整 physical scenario 划分。本轮重新训练 11 个 NumPy MLP：seed **92323**，160 epochs、batch 512、Adam lr .001，train-only normalization，以原 validation action loss 选 checkpoint；闭环评价另用新 860000–860005 scenarios。

| 输入 / 结构 | validation pitch-rate RMSE rad/s | force RMSE N | 新场景闭环 |
|---|---:|---:|---:|
| state only | .281 | .109 | 0/18 |
| state + time/mode phase | .266 | .111 | 0/18 |
| state + actuator observer | .214 | .031 | 0/18 |
| state + contact latch/time | .257 | .108 | 0/18 |
| state history，0/10/30 ms | .264 | .111 | 0/18 |
| all inputs，64×64 | .189 | .026 | 0/18 |
| all，16×16 | .258 | .032 | 0/18 |
| all，128×128 | .177 | .026 | 0/18 |
| all history，64×64 | .195 | .028 | 0/18 |

`state` 是 relative p、world v、R 的前两列和 body rate；phase 为归一化时间、duration 和 mode one-hot；actuator 为四个估计推力和 ratio；contact 为估计 latch/time。all 还含 previous action 和 observation age。每个模型的确切输入 mask 在 `features.json`；未使用的第一层权重确实为零，运行时没有偷偷恢复输入。

**Actuator 信息对 force imitation loss 有明显价值，但没有转化成任务成功。** 更大网络继续降低 expert-state MSE，却没有解决闭环问题。不能从这些配置推导“所有 BC 都不适合”。

在原独立 validation rollouts 上，用 train-only state bank 的标准化近邻距离审计：expert validation 中位距离 **.041**；BC **42.05**、history **19.27**、hybrid noisy **.490**。超过 expert validation p99=1.110 的帧占比为 **96.5%、93.6%、9.0%**。BC/history 持续 50 ms 越界的中位首次时刻均为 **.085 s**，远早于碰撞。此距离不是概率或正式 OOD detector；hybrid 的少量出界也能成功。

还在同一 balanced expert episode 上，对当前帧施加小扰动、固定过去的 controller/history，再比较 action 的局部导数：teacher 对 position-X / velocity-X 的 pitch command 导数中位数约 **-450 / -197**，BC 仅 **-.256 / -.202**；teacher 对当前 pitch-rate 的导数为 **-.5**，BC 为 **+.049**，history **+.182**。这说明低 nominal action error 没有保留相同的局部纠偏响应。

这些很大的 teacher 导数包含其对反馈姿态的有限差分，**不是低频闭环增益或稳定性证明**；它们也暴露出 teacher 原动作表示对瞬时噪声敏感。本轮改进恰好去除了这种噪声差分，尚未重新蒸馏这个新 analytic controller，不能宣称它的纯 BC 已被解决。

证据：`phase3-training-01/`、`phase3-bc-validation-01/`、[近邻分析](../artifacts/phase3-bc-analysis-01/distribution.json)、[局部反馈](../artifacts/phase3-bc-analysis-01/feedback.json)。上一轮 phase audit 仍表明 expert-state 误差在接触后 100 ms 最大，但本轮进一步确认：**实际闭环先在 approach 漂移，不能只怪 contact timing。**

### DAgger / corrective imitation

本轮复用保存完好的 training-only oracle 数据：24 条自主 student 失败轨迹，以及 24 条 70% teacher intervention 轨迹。重新用同样的 **64×64、history、160 epochs、seed 92323** 训练，与新 all-history BC 配对比较，避免上一阶段 corrective 同时扩大网络的混杂。

BC、DAgger、corrective 在新 validation 均 **0/18**。失败数据、oracle labels 和 hashes 都保留。新 phase 没有把旧干预飞行重新计为本轮自主成功，也没有重新收集这 48 条轨迹。这个负结果说明这一次 aggregation 方案不够；不证明 DAgger 在更好的 oracle、稳定性结构或新 analytic teacher 上无效。当前不值得继续盲目增加同类数据。

## Hybrid 控制组件：配对消融

设计 seeds 850004–850007，三档，共 12；相同小噪声、相同 draw/seed，不修改 evaluator：

| 干预 | 成功 |
|---|---:|
| learned reference + 原完整反馈 | 12/12 |
| teacher/model reference + 相同反馈/估计事件 | 12/12 |
| 移除 position/velocity feedback，保留姿态内环 | 0/12 |
| 单独关闭 motor-lag compensation | 6/12 |
| 单独关闭 effective-thrust adaptation | 12/12 |
| 两者一起关闭 | 5/12 |
| 不把估计 contact 交给 recovery switch | 2/12 |
| 用单帧检测代替 20 ms event history | 9/12 |
| direct motor 输出对照 | 12/12 |
| nominal rate/thrust proxy | 12/12 |

有效组件首先是稳定反馈、电机动态补偿和事件处理。这个小样本中推重比 adaptation 的成功率收益不明显；不能继续把以前“两项一起关闭”的结果全部归功于 adaptation。禁用事件仍偶有物理恢复，不代表可以依赖这种偶然行为。

目标估计独立偏差实验也保留：在 nominal target estimate 上加位置 σ=5 mm 为 **10/12**，15 mm 为 **6/12**（margin-qualified 4/12），normal σ=1° 为 **9/12**。这是固定参考的刚体 registration prototype，实际物理 target perturbation 仍在；不是一个已验证的动态视觉 tracker。相机/目标估计精度可能比继续扩大网络更重要。

## 已实施的改进与时间处理

1. **有界动态状态传播。** 用 raw acquisition timestamp、估计 motor force、effective ratio、body rate 将延迟 state 传播到控制时刻；传播 horizon 上限 35 ms。不使用未来状态，也不能预知尚未观测的碰撞。仅恒速/恒角速外推在 20 ms 新 validation 是 **4/18**，加入估计加速度为 **18/18**；原始延迟反馈也是 **4/18**。
2. **可部署的 motor observer。** 旧 wrapper 把使用 plant 即时真值 gyro 算出的 motor command 回填 observer。这是 sim actor 的信息泄漏。新 observer 用 actor 自己的角速度估计和 nominal mixer 推算 motor command；真实 gyro 仅属于 simulated inner plant。专门测试改变隐藏 inner gyro 不改变 actor actions/observer。portable 版本在 0/10/20 ms validation 各 **18/18**。
3. **分开 measurement clock 与 control clock。** `CausalState` 只对新的采集时刻更新速度增量，重复 sample 不更新比值；shared adaptation 把这个 estimate 明确传给 feedback，替代对 stale/repeated samples 使用固定 5 ms 的差分。200 Hz 固定延迟下单独改它没有解决问题，不能夸大这一项的收益。
4. **解析计算期望姿态变化率。** 使用 reference jerk 和 nominal estimated acceleration 计算 force derivative，再求期望推力方向的角速度；不再对含噪反馈姿态做相邻控制 tick 差分。仍保留原 recovery 的 rate-feedforward suppression。50 Hz estimates +10 ms delay +10% dropout：旧方案、仅动态传播均 **0/18**；再加入此项及 shared adaptation，独立 validation **18/18**。
5. **利用估计不确定度设置 contact 阈值。** 中噪声失败常在碰撞前误触发 latch。用 `max(.12, 4√2 σ_v_normal)` m/s 替代统一 .12；σ 来自输入的 velocity covariance。设计 6/12→12/12，新 validation 从 analytic 的13/18→**16/18**，最后留出 **62/72**。四 sigma 只是独立 Gaussian endpoint 的启发式；未知相关噪声、加速度模型误差和冲击下协方差失真都未覆盖，不是 calibrated contact confidence。

小噪声假设：p σ=2 mm、v=.02 m/s、attitude=.2°、body rate=.02 rad/s；中噪声分别为 5 mm、.05 m/s、.5°、.05 rad/s。这些都不是机载测量。另实际测试 5/10/20 ms，10±5 ms variable delay（dynamic validation18/18），dropout 和50 Hz降频；不同干预先分开设计，最终方法/条件在留出前冻结。

## 时间、margin 和失败迁移

balanced、24 个 draws，以下为有 recovery confirmation 的条目中位数；失败保留在分母和 CSV 中，没有把未恢复时间当零：

| 方法 | 到确认 recovery / recovery time (s) | 首次 contact margin 中位数 |
|---|---:|---:|
| teacher | 6.064 / .581 | 22.46 mm |
| 原 hybrid，小噪声 | 6.221 / .761 | 22.81 mm |
| 改进，小噪声 | 6.079 / .577 | 22.39 mm |
| 原 hybrid，20 ms | 7.574 / 2.110（17/24 有恢复确认） | 22.55 mm |
| 改进，20 ms | 6.023 / .530 | 22.21 mm |
| 改进，50 Hz estimates | 6.053 / .561 | 22.59 mm |
| adaptive_nominal staged | 15.124 / .754 | 18.38 mm |

主要改善在 recovery 的振荡和延迟闭环，不是重新优化了 outbound 或让 teacher 跑得更快。20 ms balanced 的 post-contact peak-rate 中位数 **5.16→4.84 rad/s**。更快恢复没有改善几何 margin；20 ms 成功68条中只有63条满足8 mm。10 ms还有两条回退；原方法成功数和 qualified 数不是同一指标。

原20 ms有 **52 次 RECOVERY_TIMEOUT**；改进20 ms没有此类超时，剩余四条为 edge / undesignated / wrong-first-contact。改进 medium 仍有8次undesigned contact、5次edge、6次wrong-first、2次target miss（同一trial可有多个类别）。teacher 的两个 stress 失败同样涉及错误/非指定接触。改进的失败已更多集中于 geometry / contact-region margin，不能把它们全部交给 recovery residual。

原20 ms small 为4/36、stress12/36，并不表示更大的 uncertainty 有益：两组使用独立 seeds，有限样本和分布包含不同 motor draws。不能从该非单调计数推导机制。

## 模型大小与 host runtime

只用相同 train reference 数据重新拟合三个尺寸；在相同新 validation、小噪声+10 ms动态传播+portable observer下：

| spline coefficients（三档合计） | 文件 | validation | 参考查询 median |
|---|---:|---:|---:|
| 117 | 2,871 B | 12/18 | 10.49 μs |
| 261 | 4,345 B | 18/18 | 10.47 μs |
| 原621 | 7,824 B | 18/18 | 10.45 μs |
| 1,197 | 13,281 B | 18/18 | 10.53 μs |

117 的 conservative 六条全部失败，尽管位置拟合 RMSE 看起来接近；导数一致性和末端瞬态不能只用平均位置误差检查。261 是有价值的压缩候选，本轮最终冻结仍使用原621，未用留出挑尺寸。

重新训练 MLP：16×16为868参数、5,978 B、7.00 μs；64×64为6,532、26,982 B、9.35 μs；128×128为21,252、81,863 B、14.36 μs；history64×64为10,628、43,172 B、10.56 μs。四者自主validation都是0/18。

host Intel i5-14400，单线程BLAS。完整 standalone API（包括 covariance validation、observer、传播、参考、模型反馈、nominal proxy 和日志，排除 MuJoCo stepping / packet serialization）：**median596 μs、p99 758 μs、max1181 μs**，200 Hz周期为5 ms。进程峰值RSS约118 MiB。checkpoint很小不代表整个Python/SciPy系统只占几KB；没有M3C、硬实时或视觉算力测量。

证据：`phase3-runtime-01`、`phase3-api-benchmark-02`。较早、其他实验同时运行时的 host benchmark 也保留在 `-01`，未删除较慢结果。

## Deployable API 与 ArduPilot 边界

新入口是 `deployment.DeploymentPolicy.from_bundle(...)` 和：

```python
command = policy.infer(VehicleObservation(vehicle_estimate, velocity_covariance), now)
# command.body_rates_flu_rad_s, command.collective_force_N
actuator = policy.actuator_estimate()
```

`VehicleEstimate` 提供 world NWU p/v、body FLU attitude/rate、采集timestamp；`TargetEstimate` 提供world位置/法向/covariance/timestamp/confidence，在reset时commit，拒绝future/stale/无效target；nominal参考按target estimate作刚体registration。此轮目标固定，**没有在线移动目标跟踪、FoV处理或target dropout闭环**，也没有把已有未标定视觉log伪装成metric观测。

`VehicleObservation` 的velocity covariance用于contact检测。API拒绝大于100 ms、future或非有限ego samples；35 ms propagation cap 与100 ms输入保护不是硬件安全证明。实际demo最大observation age为65 ms。硬件仍需独立验证lost-target/miss/stale/fallback/abort策略；研究runner没有证明这些状态下能安全退出。当前Python模块仍使用项目环境，并非已完成依赖裁剪的机载库。

在 demo 的 **1,203 个完全相同 raw estimated inputs** 上，独立API与sim actor action的最大差 **2.73e-11**。这条API路径不构造ManeuverSkill、不读取qpos/qvel、true contact或true actuator。仿真观测本身仍由truth生成；这不等于已经有onboard estimator。

ArduPilot核查：vendor HEAD `96f98857f7fc4bd564fbbdc43c5f26114246369f`，`ArduCopter/GCS_MAVLink_Copter.cpp` 的 `handle_message_set_attitude_target`，`mode_guided.cpp` 的rate与throttle调用。选择 **body rate + collective force** 作为研究action；adapter转FLU→FRD，`SET_ATTITUDE_TARGET` mask128、三轴rates有效。默认thrust是climb-rate语义；需确认GUID_OPTIONS bit3。即使切换，AP仍有angle boost、filter、thrust linearization、PID/shaping，**N/max_N不是真实AP throttle标定**。

新增 `ThrottleCalibration` 单调lookup与 `ArduPilotAdapter`，必须显式给calibration来源和thrust semantics；只构建packet，没有sender。pymavlink实际encode/decode验证了mask、符号和thrust字段。demo导出的packet使用明确标注的 nominal linear mapping，只用于离线接口检验。

同一MuJoCo plant上旧hybrid direct motor与200Hz rate proxy都是12/12；这两者代数上接近可逆，不能当成AP firmware通过。旧50Hz action+20ms delay为0/12。改进后，保持10ms ego delay：50Hz action无额外action delay为12/12；50Hz+20ms action delay **10/12**；100Hz+10ms为12/12；200Hz但inner gain×.7为11/12。**部署接口仍会损失性能；没有SITL/真实飞控结果。** 注意50Hz estimates和50Hz action是两种不同实验。

## 数据、实验划分和证据

- 训练：原`distill-data-01`的48条train（810000–810015×三档）和18条validation（820000–820005×三档）；48,699 /18,263 supervised frames。整条scenario划分；同一draw三档不跨split。
- 本轮新design：850000–850007；新closed-loop validation：860000–860005。
- 本轮 untouched holdout：870000–870011，1×；880000–880011，2×。未读取旧holdout来选模型；本轮holdout只在选择冻结后执行，结果未用于继续调参。
- 固定seed92323。失败rollout不删除；counterfactual干预、旧较慢host结果和全部负模型均保留。
- [冻结selection](../artifacts/phase3-frozen-01/selection.json)记录checkpoint/hash、完整variants、case list和config hashes，各shard运行前再snapshot。选中checkpoint SHA256：`bc2e0cf18b32708cb94984be762c092359c5fb54d3d8c60905cc3ba8631a6568`。
- [deployment bundle](../artifacts/phase3-student-bundle-02/schema.json)提供v2 schema、nominal config/vehicle/task、deployment options、source hashes、split规则。以前demo `privileged_only`字段继续保留；student records包含raw32维obs、age、action、oracle label（未启用时NaN）和contact innovation。raw truth/actual motor/event在flight日志，不能喂给actor。
- 全部1,698次batch评价runner完整执行；campaign `passed`表示无runner exception，不代表所有飞行通过。另有1次完整往返。root tests从82增到91。
- [最终证据核查](../artifacts/phase3-verification-01/summary.json)包含各campaign实际条数、artifact验收和91项测试状态；同目录保存完整pytest/command logs、独立nested Git状态及最终source snapshot。冻结后的实际actor/control/observation/spline文件hash复核一致。

## 文件、实际检查与复现

实际读了root AGENTS、README、START_HERE、INTEGRATION、REAL_HARDWARE、teacher/distillation/v2报告；`control.py`、`model.py`、`maneuver.py`、`experiment.py`、`teacher.py`、`teacher_interface.py`、原distillation的policy/observation/dataset/network/spline/evaluation/mission/audit/analysis/CLI、相关tests和上述AP源码。检查了root HEAD/worktrees/status及四个nested repo的独立status；原有dirty内容未reset或修改，G1 baseline/field/vendor文件未由本轮编辑。

新增：`distillation/deployment.py`、`deployment_bundle.py`、`deployment_benchmark.py`、`phase3.py`、`phase3_training.py`、`phase3_analysis.py`、`phase3_results.py`；`configs/contact_phase3/*.json`；`tests/test_contact_deployment.py`和本报告。

增量修改：`control.Skill.track`增加可选shared ratio与analytic rate（旧默认保持）；`policy.py`增加实验/部署选项、portable observer；`observation.py`增加timestamp jitter/降频及可配置contact阈值；`spline.fit`允许不同interval数；`mission.py`允许显式观测/部署选项；`evidence.py`纳入新配置/测试；README/START_HERE更新入口。未新增依赖。

以下命令实际执行过对应的`-01`/`-02`输出；复现时使用新的目录名。每条命令应整行粘贴，不能在 `-m` 后换行。

```bash
# 重训11个模型，含同容量DAgger/corrective
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3_training --output artifacts/phase3-training-new
# 任意保存的研究配置；把config换成components_design、improved_validation等即可
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3 run --config configs/contact_phase3/timing_design.json --output artifacts/phase3-timing-new
# 拟合较小reference
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3 fit --input artifacts/distill-data-01 --intervals 24 --output artifacts/phase3-spline-new
# 完整往返：50Hz estimate +10ms delay +10% dropout
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.phase3 mission --config configs/contact_phase3/deployment_selected.json --output artifacts/phase3-mission-new
# 查看本轮实际飞行：记录状态播放，不是新的闭环
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/phase3-mission-01 --speed 1 --loop
# 重新输出部署bundle和独立API验证
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.deployment_bundle --output artifacts/phase3-bundle-new
OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact.distillation.deployment_benchmark --bundle artifacts/phase3-student-bundle-02 --input artifacts/phase3-mission-01 --output artifacts/phase3-api-new
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

本轮还实际运行了五个`holdout_0/1/2/3/adaptive.json` shards、全部设计/validation配置、`phase3_analysis`、`phase3_results`，以及`render-maneuver`和`replay`。精确参数见每个`provenance.json`和`frozen_manifest.json`。`phase3_results`汇总本轮固定目录；新研究应使用新seed和新manifest，而不是反复把本轮holdout当validation。旧train dataset若需重建，用既有distillation `generate`入口；训练/评价配置中的checkpoint路径应指向新输出。

完整demo **20.014 s**，从机库上方空中出发，没有起飞/降落/停桨；回程沿用model-based return，handoff仍使用truth evaluator的held-recovery gate。不能把744条maneuver评价说成744次return通过。motor replay **4,004 samples，qpos/qvel/motor误差均0**；这是记录输入的确定性验证。

[完整运动GIF](../artifacts/phase3-render-01/maneuver.gif) · [接触GIF](../artifacts/phase3-render-01/contact.gif) · [3D轨迹](../artifacts/phase3-results-01/trajectory3d.png)。EGL渲染和图像已检查，未启动桌面GUI。新tests覆盖causal propagation、降频/jitter/dropout时钟、target commit、API covariance/staleness、portable observer不读hidden gyro、analytic hover、calibration约束和真实pymavlink序列化；共**91 passed**。

## 判断与下一步

本轮支持的结论是：**稳定反馈结构、观测时间一致性、噪声下的姿态参考导数和接触事件，比扩大网络或追加同类BC数据更关键。** Motor observer对hybrid闭环重要；history作为event window有明确价值，直接堆叠NN history尚未带来成功。无需为了路线图立即进入Residual RL。

仍未知：真实ego/target误差、时间同步与sensor covariance是否可信；AP实际内环、throttle/angle-boost语义和延迟；实际motor lag/thrust/mass；冲击时estimator跳变；目标normal/位置误差；失联、漏检和安全退出。当前模型反馈胜过所测pure BC，不等于已证明所有pure policies不可行。

下一件最值得做的是：**在隔离的真实ArduPilot SITL链路上，用本轮packet/observation contract替换nominal rate proxy，测量body-rate/collective传递和恢复性能**，先解释接口损失，不让RL补偿未知的接口语义。若之后仍有可重复、可观测的动态残差，再评价residual imitation或RL；几何margin失败则优先处理目标估计/轨迹和接触结构。

状态：**IMPLEMENTED / RUN / TESTED**；只在所列条件下 **SIM-VALIDATED**；**HARDWARE-VALIDATED：无**。
