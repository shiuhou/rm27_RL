# Whole-Maneuver Teacher：时间、接触余量、接口与 demonstrations

2026-09-23；root `astra/research`，HEAD `da396bef63ce51476a71af4b9b70abdda03eed0b`。

本轮实际交付了完整场地的三档 teacher、独立留出比较、恢复预测器、rate/thrust 代理闭环、连续返航、版本化 dataset 和可观看回放。
**建议把 balanced 作为下一阶段研究默认点：保持本次 48/48 联合成功，出发至恢复确认约 6.1 秒，较 staged robust_v2 缩短约 59%。**
这是当前有限分布的仿真证据，不是可靠率保证，也没有证明优于旧 whole-maneuver 在所有场景的表现。

当前 checkout 已经存在 `WHOLE_MANEUVER_REPORT.md` 的全局 minimum-snap maneuver；本轮直接延续，不能把已有算法说成从零新建。
新的工作是时间／姿态 operating-point 选择、teacher 数据和接口、recoverability 筛选实验以及同一植物恢复后直接返航。

## 代码与场景依据

实际核查 root README、START_HERE、INTEGRATION、REAL_HARDWARE、三份 aggressive-contact / whole-maneuver 报告；
`arena.py`、`maneuver.py`、`experiment.py`、`control.py`、`model.py`、`recording.py`、`robust.py`、恢复和评价代码；
`configs/contact_arena.json`、`contact_maneuver*.json`、`aggressive_contact*.json`、场景／无人机配置；
`rm27/autopilot/bridge/frames.py`、`rm27/simulation/micro_plant.py`、`rm27/autopilot/micro_flight.py` 及下面列出的 ArduPilot 源码。

完整场地继续由 root `arena.compose` 组合：

- 既有 `artifacts/dock-final/scene.xml`，SHA256 `768545c221a1b9337ba1dbc5ed49dc5fa56dec15eed875c3d73812efbbf646cb`。
- RMUC 场地子仓库 commit `67c13270e213cf3f69da2619e349eed273432330`；dock 源仍是 `dock.STEP`。
- root micro-UAV generator、真实物理 armor patch、bumper 和 forbidden rim。
- `base2.STEP` 整体视觉模型仍无完整碰撞语义；未改上游场地，未复活已退休 whole-field STEP。

本轮按用户指示在当前 dirty worktree 工作，未清理或改写其他 worktree／子仓库。
G1 baseline 仍为 `2369d7d207818160e2d140b16118a53f99491b66`；其既有结果不转移到本研究。
精确运行代码快照在各 campaign `source/` 与 `provenance.json`，仅 root HEAD 不足以重建未提交研究文件。

新增：`teacher.py`、`teacher_data.py`、`teacher_interface.py`、`recoverability.py`、`teacher_return.py`、
`teacher_tools.py`、`teacher_validation.py`、`tests/test_contact_teacher.py`、`configs/contact_teacher*.json`。
修改：`experiment.py` 增加 prefix stopping、observer/phase 记录和撞前状态；`maneuver.py` 增加代理与同植物返航回调；
`__main__.py` 增加命令；`evidence.py` 扩充快照；README/START_HERE 增加入口。原有控制和判据保持。

## Teacher 实际优化什么

复用全局三段七次多项式，在固定两个走廊位置节点上联合解内部速度、加速度和 jerk，最小化积分 squared snap。
不是在节点停住。终端由 bumper 几何、法向速度和 `a=alpha*n-g*ez` 共同定义；可提前进入固定推力方向的动力学弧段。
位置、速度、加速度、jerk 在飞行分段连续；物理冲量处允许状态跳变，恢复控制由真实接触事件触发。
恢复 retreat/lift 延续前轮 whole-horizon 搜索得到的参数，本轮未重新训练恢复律。

外层是**参数化场景搜索，不是全局最优求解器**：已有 fast/early-pose 两种全局设计各乘六个时间尺度
`[.75,.85,.95,1,1.1,1.25]`，连同 alignment lead 一起缩放；另测试两种 terminal specific thrust（9、14 m/s²），总共 14 候选。
设计分数先惩罚联合失败／margin<8 mm，再比较平均与最坏的出发到 held recovery 时间。
8 mm 是额外研究资格线；原 2 mm edge gate、接触／恢复标准未放宽。
内部轨迹求解和外部 rollout 同时包含终端姿态、真实 motor lag、冲量、separation 和 recovery。
它不是自由形式 hybrid direct collocation，也没有证明超过这些候选的最优性。

设计 seeds 71001–71004（前二 1×，后二 2×）；验证 seeds 72001–72004；每档验证设计排序前二。
按计划进场时长 ≤4.4、≤5.8、其余分 aggressive/balanced/conservative。
最终 candidate **2 / 4 / 10** 在 `selection.json` 冻结后，才运行从未用于选参的新留出：
small 772000–772023，stress 782000–782023。三档和两个 baseline 共 240 次，配对同 seed／物理 draw。
三档标签只是操作点名称，不能理解成跨所有指标单调的安全排序。

物理不确定性沿用已有敏感性配置：初态位置／速度／姿态、目标位置／法向、thrust/mass、motor tau、摩擦、软接触和 bumper offset。
small 如位置 σ=1 cm、速度 σ=.025 m/s、姿态 σ=2°、target σ=3 mm、motor tau scale .85–1.15；
stress 按已有 scenario 规则放大到 2×。它们是工程假设分布，非实测 population。

## 公平留出结果

时间从 slot_1 上方空中初态计到满足恢复并连续保持 1 秒；不是接触时间，也不是固定录制窗口末尾。
所有方法均保留 actuator observer 和 thrust/mass adaptation；adaptive_nominal 使用原 nominal terminal 参数，robust_v2 使用原 v2 参数。
原 PASS 与额外 8 mm qualified PASS 在本次留出的计数相同。

| 方法 | small PASS | small 时间 s | stress PASS | stress 时间 s | stress margin P05 mm |
|---|---:|---:|---:|---:|---:|
| adaptive_nominal | 24/24 | 15.025 | 24/24 | 15.154 | 9.97 |
| robust_v2 | 24/24 | 14.850 | 24/24 | 14.908 | 14.76 |
| conservative | 24/24 | 7.659 | 24/24 | 7.679 | 15.15 |
| balanced | 24/24 | 6.056 | 24/24 | 6.111 | 15.89 |
| aggressive | 24/24 | 5.471 | 23/24 | 5.547 | 8.68 |

每组只有 24 次；24/24 的 Wilson 95% 下界约 86.2%，23/24 的区间约 79.8–99.3%。
没有证据声称新 teacher 成功率超过两个 24/24 baseline；改善是时间，同时 balanced 保持本次接触余量和恢复表现。
前轮还测过 compressed staged 和 contact-only 全局搜索；本轮不重复把它们冒充新 holdout 方法。

[完整结果](../artifacts/whole-teacher-01/summary.json)；[operating-point 图](../artifacts/teacher-analysis-01/operating_points.png)。

## 速度、姿态、推力与失败迁移

下面是 stress 的中位数。姿态准备定义为接触前持续满足姿态误差≤10°且角速率≤1 rad/s；
减速提前量是日志有限差分的 `v·a<0` 最后连续区间，不是声称控制器只有一次制动开关。

| 方法 | 姿态准备提前 s | 最后一段持续减速提前 s | 速度／推力夹角 ° | 撞前总推力 N | 撞后最大角速率中位数 rad/s | effort proxy |
|---|---:|---:|---:|---:|---:|---:|
| adaptive_nominal | 0.041 | 0.270 | 176.145 | 2.051 | 6.033 | 19.101 |
| robust_v2 | 0.018 | 0.248 | 160.986 | 2.152 | 4.747 | 18.707 |
| aggressive | 0.179 | 2.089 | 168.985 | 2.113 | 4.711 | 7.528 |
| balanced | 0.222 | 0.557 | 166.643 | 2.112 | 4.195 | 8.199 |
| conservative | 0.449 | 0.720 | 160.838 | 1.866 | 3.971 | 10.395 |

推力／速度夹角约 160–169°，说明接触前仍朝装甲运动，推力已经朝相反方向。这在原 baseline 也存在；
全局设计的贡献是更早建立这些状态，同时压缩整个进场。balanced 接触前约 .22 秒准备完成，原 v2 只有 .019 秒；
conservative 约 .45 秒，换来了更长总时间。姿态在整段加速／制动时变化，不是从出库就保持接触姿态；
现有动力学和图像也不支持“倾角单调增加”这种过强叙述。

接触前各马达已带推力（约总量 2.1 N；conservative 约 1.87 N），不是碰后才从零启动。
真实 motor states、causal observer 和 command 全部保存。effort 是 `∫sum(act_i^1.5)dt`，积分到 held recovery，
**不是电能／Joule**，也未把隐藏 thrust scale 混进该 actuator-state proxy。
原 campaign 的旧 effort 字段包含额外观察保持窗口；比较应使用 `teacher-analysis-01/holdout_metrics.json` 或 v1.1 dataset 的修正字段。

更快先损失的是几何 margin：aggressive stress seed **782023** 首次 bumper→armor 接触有效，margin 仅 **6.63 mm**，
但同次过程出现后续非指定部位接触，记录 `UNDESIGNATED_CONTACT`。它随后仍能恢复，不能把该失败说成“没碰上”或“救不回来”。
设计候选 .75/.85 fast 时间尺度已出现 2/4、1/4 同类失败，早于严重持续饱和。
这再次说明 recoverability 不能替代接触几何判据。

特意改变 terminal thrust 的两个候选也保留了负结果：9 m/s² 四次全部触发高度上限；
14 m/s² 四次全部出现饱和、环境接触、target miss 和倾角超限。
改变终端加速度会改变整条全局多项式；更强的末端 outward thrust 不会自动带来更好的制动／接触。

存在时间－余量－姿态提前量／撞后角速率的经验折中，**不是三点在任意二维图上都必然 Pareto 最优**。
balanced 在 stress 时间和 margin P05 上都胜过 conservative；后者价值在更长的姿态准备和略低的撞后角速率。
没有证明一个平滑、完整的 Pareto frontier。

[3D 轨迹](../artifacts/teacher-analysis-01/trajectory3d.png) · [速度／加速度／姿态／motor 图](../artifacts/teacher-analysis-01/mechanisms.png) · [阶段时间线](../artifacts/teacher-analysis-01/phase_timeline.png)

## RecoverabilityModel：有预测价值，暂未省下计算

把历史 1318 条可用接触 rollout 在 first-contact+120 ms 的状态作为输入，使用标准化距离加权 kNN（15 邻居），
预测既有 controller 的恢复分数、剩余恢复时间和高度损失。支持域外标记由训练近邻距离定义。
含 actuator 版本有真实 motor state 和 hidden mass/thrust/tau；只供 teacher 查询，非 student actor 输入。
它是 controller-conditioned 经验模型，不是 viability kernel，分数也不是经校准的概率。
旧 99 条 free-air recovery-map 的锚点和 separation 判据不一致，明确排除，未强行混用标签。

历史 whole-maneuver 验证 240 条全部恢复成功：时间 MAE 从 kinematic **.128 s** 降到 actuator **.084 s**，
常数基线 **.326 s**；这些样本不能验证失败识别。
额外使用未用于拟合的历史 v1 evaluation 383 条接触，其中只有 224 条满足 recovery label：
actuator 时间 MAE **.567 s**（kinematic .727、常数 .941）；Brier **.274**（kinematic .340、常数 .385）。
以 .8 分数阈值仍有 **78/159 失败被判可恢复**，并错拒 20/224 成功；360/383 在支持域。
标签只判 recovery，并不要求 first-contact 有效；这正是不能把模型分数当整个任务成功率的原因。

实际优化器先跑 56 个 prefix、冻结 surrogate top6，再完整跑 56 条 design oracle 作核对。
筛选保留了 oracle 最优 candidate 2，但计算收益为负：prefix **51.44 s**；全量 design **81.27 s**；
估计 prefix+top6 full **83.47 s**。虽然可避免 32 次完整未来 rollout，重复前缀抵消了收益。
本次实际仍运行全部 oracle，不能宣称已节省这 32 次。
下一步若继续用此模型，应缓存／恢复 MuJoCo state 和 controller state 从 prefix 接续，且所有最终点仍需完整 rollout 验证。
本轮不把小 kNN 换成网络来掩盖样本标签和支持域问题。

证据：`teacher-recoverability-01/`、`teacher-recoverability-failures-01/`、主 campaign `surrogate_selection.json` 和 `prefix/*/queries.json`。

## 部署动作：实际核查与代理实验

ArduPilot vendor HEAD `96f98857f7fc4bd564fbbdc43c5f26114246369f`。
源码 `ArduCopter/GCS_MAVLink_Copter.cpp:890` 接收 Guided `SET_ATTITUDE_TARGET`；
三轴 rate 要么全部生效、要么全部忽略。mask=128 忽略 attitude，保留 rate+thrust。
`mode_guided.cpp:1025` 将 rates 输入 `input_rate_bf_roll_pitch_yaw_rads`。
**默认 thrust 是爬升率语义；`GUID_OPTIONS` bit 3 才切为 normalized thrust。**
`mode_guided.cpp:1032` 又调用 `set_throttle_out(..., true, ...)`，
`AC_AttitudeControl_Multi.cpp:365` 的 angle boost、滤波和后续 motor thrust linearization 仍存在。
因此 `force_N / max_force_N` 不能自动当成可用于真机的 MAVLink throttle。

新接口保留两层：teacher rotor force → nominal body-rate + physical collective N → 显式标定 callable 才生成 MAVLink 字段。
frame NWU/FLU 到 NED/FRD 均为 diag(1,-1,-1)，角速率用 rad/s，四元数 WXYZ。
已有 pymavlink 真正编码／解析测试验证 mask、符号和 thrust 字段；无 sender、未发飞控参数。

投影采用名义 rate-P 内环逆映射：由 motor wrench 扣除 gyro feedforward 后得到期望 rates；
在独立 MuJoCo proxy 内环重新混控，经过 sample/hold、delay、限幅，闭环运行到恢复。
这不是单纯把日志改名，但 matched 内环的等价是构造出来的，不能当跨控制器迁移成功。

| proxy 条件 | conservative | balanced | aggressive |
|---|---:|---:|---:|
| matched 200 Hz | 4/4 | 4/4 | 4/4 |
| held 50 Hz、零额外延迟 | 4/4 | 4/4 | 4/4 |
| held 50 Hz、20 ms 额外延迟 | 2/4 | 2/4 | 2/4 |

使用冻结的四个 validation draws。20 ms 延迟的两个 stress seeds 均恢复超时，较慢 conservative 也未避免。
这比“motor teacher 已可部署”的说法更有用：实际飞控内环动态／延迟必须进入下一轮 teacher。
最终审计结果在 `teacher-interfaces-02/`，已把 proxy 自身混控饱和计入 evaluator；
主 campaign 内较早版本仅合并原 teacher 饱和，完整保留但不作为最终接口验收依据。
**没有运行 ArduPilot firmware、SITL、视觉或硬件。** 当前 root bridge 是 500 Hz 四机命名场景，
contact plant 是 1 kHz 单机物理＋200 Hz 控制，尚需单机桥接及真实 throttle 语义验证。

## 恢复后直接返航

在第一次 held recovery 事件立刻把同一个 MjData、motor state、observer 接入原有 return route，
未 reset／teleport。返回路线固定 3+7+3 秒并稳定保持 1 秒，所有方法相同。

| 方法 | nominal 恢复确认 s | 完成 return-near-dock s | PASS |
|---|---:|---:|---|
| adaptive_nominal | 15.022 | 29.022 | True |
| robust_v2 | 14.848 | 28.848 | True |
| aggressive | 5.474 | 19.474 | True |
| balanced | 6.066 | 20.066 | True |
| conservative | 7.655 | 21.655 | True |

balanced 完整 nominal 空中往返比 robust_v2 缩短 **30.4%**；没有单独优化返航速度。
只跑了五个 nominal 往返，不能把 outbound 的 48 次扰动结果扩展为 return 48/48。
任务仍从 dock slot 上方 .23 m 空中状态开始，返回 slot 附近；未实现停桨起飞／精确落库。

combined trace 的 4015 个样本经过 recorded-motor replay，qpos/qvel/motor state **最大误差均 0**；
这证明同一输入的数值回放，不证明异步闭环每次完全一致。
三档 nominal approach 的 686 mesh surface audit 通过；balanced return 250 个球包络采样无表面相交。
approach audit 明确排除接触前最后 .5 秒和 recovery；视觉 base 不是已认证实体碰撞模型。

EGL 渲染实际完成，已检查接触和曲线图；本轮未打开桌面 GUI。
[完整 balanced 往返 GIF](../artifacts/teacher-return-render-01/maneuver.gif) · [接触 GIF](../artifacts/teacher-return-render-01/contact.gif)。

## 镜头第一视角回放（2026-09-27）

新增 [第一视角 MP4](../artifacts/teacher-return-fpv-20260927-v2/first_person.mp4)，
沿用 `teacher-return-01/balanced` 的完整 recorded-state trace，未重跑控制或改变物理模型。
输出为 1280×720、30 fps、603 帧、约 20.1 秒，默认垂直视野 60°。
`summary.json` 的 `passed: true` 表示这次离屏渲染完成且原输入雜湊未改变，
不增加原始飞行的成功率／视觉控制／实机证据。

使用者确认真实外形 CAD 前方的小圆柱是镜头。本次先核对原始 STEP 的 SHA-256
`2c7ef76d08ea9b51c4d9f1a429cbffa4d4002d4b5eba3fff224d1b6420480893`，再读取几何：

- `2627drone_ver1.STEP` 第 364 个实体（0 起算）的第 20 个面是直径 15 mm 的圆形镜头端面。
- 面中心 CAD 坐标为 `(0.12962617, 7.68025266, 979.35893312)` mm；朝外轴约为 `(0, 0.34202014, -0.93969262)`。
- 沿既有反桨布局变换后，镜头朝外形模型 +Y，向下 20°。这来自 CAD，不是光学标定。

**两种机体的区别与对齐假设：** 这段 Teacher 飞行用简化的对称机体，没有 CAD 镜头几何。
仅为当前回放，把 CAD 的镜头侧 +Y 映射到 Teacher 机体 -X（初始朝向下的进场侧），
CAD 包络中心映射到 Teacher 机体原点；未确认真实质心、飞控机头轴或相机入口瞳。
得到虚拟相机位置约 `(-53.9003, 0.00588, -4.64678)` mm，光轴为
`(-0.939693, 0, -0.342020)`（Teacher 机体系）。
完整几何依据和假设保存在 `configs/contact_fpv_camera.json`；可用 `--camera-config` 替换。

相机严格随记录的机体姿态旋转，包括滚转；不锁定绿灯、不按速度方向转镜头、也不稳定地平线。
原录制返航没有调转机头，因此返航画面仍朝基地，表现为后退远离目标。
接触附近镜头非常接近绿灯／基地，可能被局部表面填满或裁切；此视点没有独立的碰撞验证。
这仍是理想针孔预览，没有真实畸变、曝光、模糊、滚动快门或噪声。

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/teacher-return-01/balanced --camera fpv --speed 1 --loop
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render-maneuver --input artifacts/teacher-return-01/balanced --output artifacts/teacher-fpv-new --camera fpv --speed 1
```

窗口按键：`1` 跟随、`2` 全场、`6` 第一视角、`V` 全场／跟随、空格暂停。
第一视角复用私有显示模型里的 `overview_top` 相机槽位，原始模型文件与控制植物保持不变；
自由视角的全场／跟随仍可使用。`--fovy` 可修改垂直视野。
MP4 无文字叠图；另存进场、接触、恢复、返航等 PNG，
以及 `camera_poses.npz`（视频时间、实际仿真时间、trace 索引、相机世界位置与姿态）。
视频按输出时刻选择最近的原记录状态；真值时间以 NPZ 为准，不能当作重新运行的相机闭环。
`summary.json` 含输入／源码雜湊、相机内参 K 和约定，`camera_mount.json` 保存本次安装配置。
已完成 EGL 画面检查；桌面 GUI 尚未人工操作验证。

## Dataset v1.1 与使用边界

最终 `artifacts/teacher-dataset-02/`：**320 demonstrations，762836 个时间样本，12 个失败 trial 原样保留**。
分 design 56、validation 24、holdout 240；同 scenario seed 跨方法归于同 split，审计无 split leakage。
**不要用 holdout demonstrations 训练 student 后仍把同一批作为 student test。**
未训练任何 student，也未声称仅 80 条 design/validation 已足以训练泛化策略。

每 trial `demo.npz` + `metadata.json`；根有 `index.json`、`schema.json`、source snapshot。
保存 task／nominal target／hidden physical draw、初态、参考 p/v/a/q/rate/force、motor command/state、
真实 contact／phase、撞前精确状态、pre-impact／120 ms post-impact／held-recovery 的样本索引和时间偏差、
成功失败标签、成本分量、源 config／trajectory／summary／model hash、rollout provenance 与 exporter source hash。
无 recovery event 时明确 `available:false`，不拿最后一帧伪造恢复。
时间为 simulation 单调秒，通常 200 Hz；事件结束样本允许非固定间隔。

actor 只允许：

- `observation_ego`：p/v/quaternion/body-rate，当前用 truth 代替未来估计值。
- `observation_target`：committed nominal target 的 body-relative position/normal、age、confidence；不读取 physical target truth。
- `observation_motor_estimate`：因果 observer 和 effective thrust ratio。

`teacher_motor_N`、`teacher_rate_thrust` 是监督标签；reference、phase、contact truth、true actuator、
hidden draw、event samples 和未来 outcome 均标 `privileged_only`，不能悄悄喂给 actor。
这些只是可部署的观测**类型**；没有已验证的 onboard estimator、confidence calibration 或真实 contact sensor。
student 任务／aggression 指令和 history window 可由数据索引／任务参数构造，尚未给出已训练接口。

## 运行、测试和复现

以下命令在 repo 根运行；每次更换 output 目录。首轮 model fit、main campaign、return campaign 通过 Python API 实际执行，
后面的 interfaces、numerical、dataset export、plot、render、replay 通过同名 CLI 实际执行；CLI 复现入口已补齐。
不需要增加深度学习框架。使用既有 `requirements-contact-research.txt`（含 integration requirements、SciPy），
pymavlink 已在当前 MuJoCo 环境中可用。

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact recoverability-study --output artifacts/teacher-recoverability-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-campaign --output artifacts/whole-teacher-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-return --parameters configs/contact_teacher_balanced.json --output artifacts/teacher-return-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact view --input artifacts/teacher-return-01/balanced --speed 1 --loop
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-export --input artifacts/whole-teacher-01 --output artifacts/teacher-dataset-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-interfaces --input artifacts/whole-teacher-01 --output artifacts/teacher-interfaces-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-validate --input artifacts/whole-teacher-01 --output artifacts/teacher-numerical-new
/home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact teacher-plot --input artifacts/whole-teacher-01 --output artifacts/teacher-analysis-new
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m rm27.experiments.aggressive_contact render-maneuver --input artifacts/teacher-return-01/balanced --output artifacts/teacher-render-new
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q
```

`contact_teacher.json` 明确引用本次冻结 predictor `artifacts/teacher-recoverability-01/actuator_state.npz`。
若重建到 `*-new`，只改该配置的 surrogate_model 路径；不覆盖历史 artifact。
场地 composition 和历史模型拟合都需要既有本地 assets／历史 artifact；未把大 CAD 或 mesh vendor 到 Git。
单条命令应整行粘贴；不能在 `python -m` 后直接换行而不加 shell 续行符。

最终 root tests **74 passed**（含 5 个新增 teacher tests）；原动作／arena／maneuver checks 保持通过。
18 个 timestep checks：三档×nominal/一个 stress validation draw×1/.5/.25 ms 全部通过；
恢复时间跨步长变化≤4 ms、margin 变化<.4 mm。这只覆盖所列场景，不是全 holdout 的数值证明。
运行级 `passed:true` 在 campaign/model study 中表示 pipeline 无 runner error；逐 flight 成败见上述表，不能把二者混淆。

## 本轮结论与下一步

IMPLEMENTED / RUN / TESTED：whole-horizon parameter teacher、proxy、dataset、返航、自动评价与回放。
SIM-VALIDATED：上述固定场景、draw 和门槛下的三档动作；balanced 在这批测试提供最有用的时间／余量折中。
HARDWARE-VALIDATED：无。无真实判伤、结构强度、比赛合规、视觉或 SITL 声明。

被实验否定／限制的假设：更保守必然所有指标更好；更强 terminal thrust 必然更好；
恢复模型能省算力；高 recovery score 足以保证任务安全；rate/thrust 字段能无代价替换 direct motors。

**最值得继续的一件事是把真实飞控内环和链路延迟纳入 teacher**：建立单机 ArduPilot SITL rate/thrust adapter，
明确 angle boost／throttle calibration，复用完全相同 paired evaluator；再决定是否蒸馏为具备 history 的 student。
20 ms proxy delay 已经足以击穿两个 stress draws，单纯继续提高 outbound 速度或扩大网络不会消除这个接口缺口。
