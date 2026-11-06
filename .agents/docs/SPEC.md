# RM27 Agile Flight RL — SPEC

当前主研究路线为 **PerchRL-style Two-Stage PPO**：首先从稳定悬停接管状态自主学会连续机动与
指定底部接触件的受控物理接触，再研究估计观测微调、学习效率比较和接触后恢复整合。
用户确认的任务范围与本轮工程边界见 §10；旧 I4／E0／E1-A 保持各自历史语义。

## PDF season and historical interface batch (2026-10-30)

Current season source and component role are stated in the repository README.
Previous I0–I4 task contracts, checkpoints, scoring and training budgets are historical
experiment boundaries and are preserved. The new season goal is repeated contact
and recovery. The allocation described by this historical interface batch does not
override the current two-stage PPO route in §10. This interface batch itself grants
no training authorization. `interface_worker.py` uses contact_world_v1, the existing
observation formula and NormalizedCTBR with an untrained fixed-seed actor. This is
an inference/transport test, not First Contact implementation or policy success.
Physics stays external and no parent implementation is imported by the worker.

日期：2026-10-13。I0／I1／I2／I3 契约状态：**ADOPTED**，依据用户对各批次的明确实施请求；I3 依据“PLEASE IMPLEMENT THIS PLAN: I3 最终计划”获准开展下述有界 PPO 工程闭环。采用不等于验证通过；运行结果仅记录在现有活动计划。服务器及硬件任务未获授权。

研究名称现为 RM27 Agile Flight RL。自主敏捷飞行及 I4 无接触末端到达方向为 **USER-CONFIRMED**；用户2026-10-13明确批准以新活动计划执行I4，接口、首轮研究配置和有界实验范围为 **ADOPTED**，能力与效果须按实际结果报告。I0—I3、S1/S2 的 ADOPTED 契约和历史证据继续有效；S3 未实施。此前文档合入本身不授予执行权限；本次批准不包含Git发布、服务器或硬件。

## 0. 来源、权威与适用范围

本文约束独立 RM27 Agile Flight RL 及其获准触及的共享 Physics。工程约定来源为 `engineering.md`，已采用规则完整维护于 §2；根 `AGENTS.md` 提供入口，[DESIGN.md](DESIGN.md) 维护设计。保留的输入材料仅承担来源职责，不持续维护第二套正式规范，也不在根目录保留并行规范或设计副本。

总体研究路线依新 PDF：自主敏捷进场、指定部位接触、脱离、恢复、重新获取目标与同架次重复接触；返航不是本季必需交付。I0—I3 的局部接触范围、固定配置和运行预算继续限定对应历史任务，不升级为整个研究线的永久限制；新任务也不得反向改写旧任务契约。

原 RM27 仓库的集成和硬件文档继续承担原职责，不迁入独立 RL 仓库。本文的频率、动作边界、机体估计及局部验收不成为整个 RM27 的要求。

| 来源／状态 | 本研究采用的内容 | 证据边界 |
|---|---|---|
| I0—I3 用户任务目标 | 固定装甲附近连续进场、转姿、指定接触、脱离并恢复；接触后不附着 | 局部研究，不包含机库起降、全场导航、返航或调度 |
| 工程约定 | §2 E1—E8、单一文档职责、根 configs／tests 及按需目录 | 已采用；不能扩大任务授权 |
| 静态源码与既有审计 | 源 PX4 工作区提交 `ec9db0480a86b4262077929ce324fd44e5585256` 的接触、控制和模型资产 | 历史报告不当作本轮 PASS，不假设其他分支成果不存在 |
| I0／I1 已采用设计 | 原生 MuJoCo、150 g＋bumper、物理 CTBR、共享逐步 evaluator、Gymnasium 接口 | 研究配置及工程估计，不是实机能力或标定结果 |
| I2 已采用设计 | 单一 CPU headless 容器、原哈希锁、源码与结果分离、未提交来源记录 | 定义、构建及运行分别报告；不包含 GPU 或训练 |
| I3 已采用设计 | 既有名义任务、CPU PPO、观测归一化、checkpoint 与一次续训验证 | 有界工程闭环；不要求预算内学会任务，不证明稳定学习或泛化 |
| 后续候选 | 课程、历史观测、随机化、GPU、服务器和 PX4 部署对照 | 待对应任务批准与验证 |

## 1. I0—I3 目标及首版边界（保留契约）

首版交付可逐步交互的六自由度接触／恢复环境和现有控制基线接入。指定接触成立不等于任务成功；成功还要求脱离、进入允许状态并连续保持，且无不可接受碰撞或违规。

首版是低维仿真真实状态输入。它不证明视觉、机载闭环、真机安全、机械存活或裁判有效触发。未知执行器状态及 baseline 内部状态也使 actor 输入仍可能部分可观测。

原 direct-motor runner 在原 RM27 分支保留回归职责；本仓库保留特权 CTBR baseline，不复制旧 runner。I1 首版不新增 PPO、历史网络、随机化实验矩阵、MuJoCo Warp、SITL、视觉／VIO 实现或硬件操作。I3 的有限 PPO 授权见 §7，不改变 I1 的环境契约。

## 2. 工程规范：来自 engineering.md


以下八项是本项目采用的工程约定，保持源材料的顺序和要求。

### E1. 先定位原因，再修正负责该行为的实现

修改前检查相关源码、调用链、数据流、状态与资源所有权、已有测试和工作区状态，区分现象、已证实原因和假设，以最小复现或针对性检查检验假设。优先修正错误产生处的计算、契约、初始化、生命周期或状态转换，不在下游不断补偿上游错误。允许开展可撤除的局部诊断，未证实原因不得包装为根因。

### E2. 最小充分修复，复审多轮累计结果

允许在授权范围内重写出错函数、整理状态或合并重复路径，不为缩小 diff 保留错误设计，也不借机开展无关重构。新增标志位、兜底、重试、延时或兼容层须对应真实契约、外部故障或支持义务，明确边界与失败语义，不能只为协调前轮补丁。合理校验与必要容错不因追求简洁而删除。外部缺陷暂不可修时，只在授权范围采用隔离且可撤除的绕行，说明限制与移除条件，不宣称根治。

每轮检查当前源码及本任务起始状态以来的累计改动，区分用户、其他任务和本任务修改，不默认上一轮方案正确。出现关联回归、同一问题连续两次修复经验证仍失败，或新状态仅用于协调旧补丁时，先复审因果链和方案再继续。次数是诊断触发点，不是放弃或少测的理由。最终实现应无需了解修复历史也能解释。

### E3. 改进是保留改动的条件

动手前用已有任务记录或简短说明明确有效基线、预期收益、必须保留的行为与可接受代价；基线至少可定位代码状态和相关运行条件，不另存源码副本，也不以自己上一轮的退化状态抬高收益。收益可为满足真实新需求、修正行为、提高可靠性或性能、降低资源消耗或维护复杂度，不要求所有维度同时提升，但必须满足已确认验收及受保护约束。不得事后放宽标准迁就实现。

可比验证显示无有效收益、收益不能覆盖约定代价或存在不可接受退化时，应调整或重构出错方案；仍不能达到目标则撤除本任务的无效改动及其专属依赖、配置、测试脚手架和文档，不因投入工时而保留代码，不继续叠加补偿。保留仍有独立价值的回归测试。验证受阻只表示收益未证实，不等于已证实无收益；标明 NOT_RUN 或 BLOCKED，不宣称改进完成，也不默认把候选方案长期并入正式实现。撤除不等于恢复已知错误或危险状态，不回退用户修改；无法得到可接受状态时明确任务未解决。

### E4. 少而清晰，接口与命名明确

减少无实际职责的函数、文件和目录，保持高内聚、低耦合以及清晰依赖、所有权和生命周期，不合并独立职责或制造巨型函数来凑数量。优先复用，不为单次转发建立空壳 manager、service、helper，不为假设需求搭框架。接口明确输入、输出、错误与副作用，命名表达语义和必要单位，遵循项目惯例。不得散落路径、密钥、设备身份或无来源阈值；合理常量命名说明，可变项复用配置机制。禁止幽灵函数、未接入功能或占位实现伪报成功；判断未使用前检查回调、注册、动态与外部调用。

### E5. 替代完成后退役旧实现，不永久双轨

新实现通过改进验收、覆盖仍有效需求且相关迁移完成后，默认在本次替代范围内删除无用途的旧实现、重复入口、专属配置、依赖、补偿分支和失效说明，不仅因“也许以后有用”保留开关、回退链或备份实现。既有代码或功能过去存在，不自动构成永久保留理由；已明确废弃的旧能力无需继续支持，但改进实现不授权擅自删除仍有效的用户能力。

退役前核实实际调用、公开接口、回调注册、配置及持久数据兼容和部署依赖。确有真实支持、迁移、分阶段发布或回滚要求时才保留过渡路径，在现有文档记录保留依据和可核验的退出条件；不编造期限，不无限延长已满足退出条件的过渡层。旧实现测试应迁移到当前有效行为，不随实现替换删除必要回归覆盖。不能确认依赖已清除时，仅阻止相应删除，不据此永久保留全部旧路径。

### E6. 验证真实行为，不为补丁堆测试文件

优先扩展相关现有测试，以真实生产路径暴露原始错误并检查相邻行为；不复制生产算法充当验证，不以字符串断言或过宽 mock 代替行为检查。测试按行为或模块组织，不按轮次新建近似脚本。能安全执行时验证修复前失败、修复后通过；缺少修复前运行证据须说明，不破坏工作区补证。不通过删测试、放宽断言、隐藏异常或伪造输出过关。区分 PASS、FAIL、NOT_RUN、BLOCKED，编译通过不等于集成或目标设备通过。性能收益须有可比测量，结构收益须说明减少了哪些重复或耦合并通过相关回归，不能只用代码行数证明。

### E7. 产物按需有界，在收敛节点清理

简单任务不新增计划、报告或状态文件，直接在回复说明结果。复杂任务只更新一个活动计划及简短结果区，不为每轮新建编号、日报或交接稿。除用户另有指定，新增辅助文档、计划、测试分别放 .agents/docs/、.agents/plans/plan_<object>_<number>.md、tests/；优先更新已有文件，不迁移原生入口。确需落盘的诊断材料使用 .agents/runs/<task-id>/，同一任务复用；目录按需创建，不逐轮备份源码、制作 ZIP 或保存重复 diff，不擅自提交或重写历史。

成功命令默认不额外保存完整输出，不持续 verbose/debug 或逐帧、逐样本打印。必要诊断先限定对象、时长或次数、体积与保留范围，保留错误上下文，说明采样或截断。不能为限制输出隐藏失败、改变退出码或提前终止必要验证；优先复用采集能力，不另造日志框架。达到上限时调整采集，不无限落盘或删除唯一证据。

在方案被淘汰或合并、原始问题与相关回归验收闭合、替代与迁移完成、任务交付或交接等节点，清理归属清楚且不再需要的临时诊断、重复输出、失效草稿、孤立引用和备份，不等到任务结束才处理。只在现有计划或回复保留必要原因、最终结果和未决项，不另建清理报告或归档目录。已解决失败的逐轮原始输出，确认不再承担复现、比较或留存责任后可删除；有意义的失败结论和有效回归覆盖保留，不把失败改写为从未发生。

清理限于本任务废弃改动、获准退役的旧实现及授权范围内无用产物。保护用户及其他任务修改、归属不明文件、未决失败的唯一必要证据、正式实验数据和要求留存的审计或安全日志。不按时间戳盲删，不批量清空工作区；工具自身和生产日志遵循既有策略。必要破坏性操作遵守明确授权与环境审批。

### E8. 单一事实来源，按最终状态交付

规范、需求、设计和任务状态各维护一处，其他位置引用；小任务不为模板补无关章节，不把项目要求升级为全局规则。完成条件包括：相对有效基线的收益已验证，约束与相关回归通过，无效改动已撤除或重构，已满足退役条件的旧路径已清理，剩余兼容有真实依据，必要证据保留且无无用新增产物。未满足项如实报告，不以报告数量、代码量或补丁轮数当进度。交付简述收益依据、根因修正、撤除与退役内容、验证和未决风险，不另增文档。

## 3. 文件与版本管理

- 根 `AGENTS.md` 引用本文；工程规则在本文维护一处，DESIGN 不复制规则或逐轮状态。
- `.agents/` 只有 `docs/` 纳入版本管理；其他本地状态忽略，按实际需要创建。不能取消其他任务已有文件的跟踪或删除其资料。
- 每个任务只有一个当前活动计划。I4使用仓库内
  `.agents/plans/plan_agile_terminal_arrival_01.md`，多轮实施、重试和调试
  更新同一文件，不另建02、03版本或并行报告。
- 该计划是被Git忽略的本地文件，不随Git分发，也不是理解正式规范和设计的必需文件。
  正式要求与设计在本文及DESIGN中独立维护。
- 活动计划开头的“§0 进度与交接”区块按共用 `engineering.md` 第 3 条维护（唯一进度／todo／交接位置）；I4 即上述计划的 §0。
- 原RMRL/contact_rl_01计划保留为历史记录，不再承载活动任务；S3仍未实施。
- 可变值与 profile 归根 `configs/`；可复用测试归根 `tests/`，研究包不另建重复测试目录。
- 原始资产、正式实验数据、唯一必要失败证据受保护。虚拟环境、缓存、构建产物与 artifacts 忽略；不预建训练、checkpoint 或其他空目录。
- 不依赖个人绝对路径或逐脚本 `sys.path` 补丁。安装契约为完整 checkout 的 editable 安装，根配置仍为源码资产；I1 不承诺脱离 checkout 的 standalone wheel。

## 4. 首版环境契约（I1 ADOPTED）

### 模型、动作与观测

采用 `configs/dynamics_150g.json` 的完整 profile 加原机体几何、局部 armor／rim 和 bottom bumper。质量 0.15 kg 有用户来源；惯量、推力、滞后、偏航系数与碰撞近似仍为工程估计。保留禁碰几何，不将 guard-tip 混入允许接触集合。不悄悄回退旧 180 g 配置。

动作依次为 `[collective_force_N, roll_rate, pitch_rate, yaw_rate]`。推力沿 body +Z，以 N 为单位；角速度为 FLU body frame rad/s。推力边界来自四倍 profile 单旋翼上限，rate 边界来自研究配置。动作经过校验、边界裁剪、既有 rate proxy、控制分配、电机限幅及 filterexact 执行器动态；不能通过写位姿／速度执行动作。proxy 不等价于 PX4 PID。

观测为 float32 26 维，依次是世界系目标相对位置 3、目标法向 3、自身速度 3、按行展开 body→world 旋转矩阵 9、body 角速度 3、上一 CTBR 4、剩余任务秒数 1。没有真实 contact bit、精确电机状态、evaluator phase 或随机参数。

上一动作指上一条经边界裁剪且实际送入 rate proxy 的 CTBR，不是实现的推力／角速度，也不是电机限幅后的控制效果。诊断量分别命名。初始 `[mg,0,0,0]` 只是悬停初始化约定，不声称有真实控制历史。

### 生命周期、时钟与错误

`reset(seed, options)` 初始化本实例随机源和名义场景，不开启随机化。电机激活和命令均为 `mg/4`；baseline 的电机观察器与此前命令相同，specific thrust ratio 为 1，初始速度／姿态记忆取 reset 状态，时间记忆清空。每个实例独立拥有 MjData、评价器、动作与接触锁存、随机源和 baseline 可变状态；固定只读模型可复用，reset 不改共享模型参数。

物理 1 ms、控制／策略 5 ms、rate ±20 rad/s、任务 6 s 都是配置中的研究设定，未标为实机能力。正常 `step()` 推进五个物理区间；任一子步终止后锁存该时刻结果并停止剩余子步。返回状态、实际时间、reward 和原因一致；不可逆违规不能被后续状态覆盖。

接触事件与区间积分分别取样，每个真实区间只结算一次。`mj_forward()` 不推进时间，额外快照不能增加冲量、接触时长或恢复 hold。唯一顺序和来源见 DESIGN §4。

输入形状错误或非有限值在推进前报错，不更新动作记忆；有限越界按接口裁剪并报告。终止后 `step()` 报错，不自动 reset。默认 headless，导入不初始化 GUI、飞控或串口。`close()` 释放本实例资源。

### 任务评分、reward 与终止

I1原runner与环境曾共用评分实现；分离后本仓库使用Physics中的唯一evaluator，原消费者接入待S3。保持首次同时碰撞分类、短触锁存、脱离、危险二次接触、hold 中断清零及原失败类别。hold 按实际时间戳连续成立；`recovery_time_s` 保留稳定段起点相对首次接触的含义，不改为 hold 完成时间。

保持原任务阈值和 2 s 内进入稳定再连续保持 1 s 的含义。6 s 为有限时域任务期限；成功、不可逆违规和任务超期使用 terminated，首版正常返回 truncated=False。中途快照不做最终 TARGET_MISS 结算。

首版事件奖励只服务最小环境：合法首次接触 +1、首次脱离 +1、成功 +10；失败终止当步为 -10 且优先于正奖励，其余为零。权重归配置。reward 不定义物理成功，不宣称已证明适合 PPO；本阶段不调参。

## 5. 验收与报告边界

| 验收 | 必要证据 |
|---|---|
| A0 工作区与规范 | 固定来源、受保护 dirty 内容、隔离、文档单一权威与 ignore 正确 |
| A1 环境与执行链 | reset／step 空间、输入错误、动态状态隔离、执行器作用及终止时钟一致 |
| A2 评价 | 真实生产路径覆盖同时首触、短触、hold 中断、终止锁存、单区间结算与重置 |
| A3 基线 | 一次固定名义初态、最长 6 s 的 150 g CTBR 回合，真实成功或失败分类及必要证据 |
| A4 安装 | worktree 专属环境可从仓库外导入包并读取根配置，不依赖共享 venv |

I0／I1 针对性检查仅包括已选旧测试节点、新环境／子步测试及一次有界基线；不默认全仓回归、长 benchmark、多 seed、SITL、PPO 或 GPU 仿真。I3 只执行 §7 规定的接入检查和有界训练。具体结果更新原活动计划，不另建报告。

分开报告环境工程检查、150 g 特权 CTBR 基线结果及学习能力。基线失败不必然阻止环境契约通过；保存必要结果，不放宽阈值或搜索初态。若暴露本次环境错误，只局部处理该错误，不扩成控制器优化。学习能力在 I0／I1／I2 阶段为 NOT_RUN；I3 的证据边界见 §7。

结果严格区分 PASS、FAIL、NOT_RUN、BLOCKED；历史通过不占本轮 PASS。候选方案、安装成功、代码存在与行为通过不能互相替代。

## 6. 后续边界与变更

I2 已获批准：复用同一依赖来源，容器环境在 `/opt/venv`，镜像包含所需代码与配置；开发时可只读挂载 `/workspace`，结果单独外置。固定基础镜像 digest，记录 HEAD、dirty、实际代码／配置与锁标识、运行镜像 ID；未提交实现不能仅凭 HEAD 恢复。只运行一次构建与一次合并的导入／配置／reset／step 和既有基线检查；明确打包错误允许局部修正及受影响重查。禁止修改物理、评分、成功阈值或搜索初态以通过容器检查。Docker 不解决宿主 NVIDIA 驱动或 GPU 架构兼容问题。服务器条件、实机推力映射及标定仍未知，不阻塞有边界的仿真环境。

以后批准目标／约束时更新本文对应条目；设计更新 DESIGN，实施／验证更新唯一活动计划。I0／I1 不自动授权提交、推送、合并、其他 worktree repair／prune、Docker、训练、服务器或硬件动作。临时禁令按任务范围解释，不变成永久研究禁令。

## 7. PPO 工程闭环（I3 ADOPTED）

沿用 §4 的名义 ContactRecoveryEnv、完整 150 g＋bumper profile、固定初态、原评分器与最长 6 s 任务期限。事件 reward 保持接触 +1、脱离 +1、成功 +10、失败 −10；不新增悬停课程、视觉、随机化、新动力学或 reward 调参。任务期限仍为 terminated；采样 rollout 的边界不成为环境终止或虚构 truncation。

采用 Stable-Baselines3 PPO 2.9.0、官方 CPU PyTorch 2.14.1+cpu；训练依赖进入唯一根打包声明的 train extra 及同一哈希锁，不改变已有仿真依赖版本。仅使用本 worktree 的隔离环境和单一 headless CPU 镜像，不要求 5060 Ti 的 GPU 功能或七张 4090。冲突、构建与运行结果分别报告，不以解析或安装成功替代运行检查。

策略四维 [-1,1] 动作经原定线性映射到物理 CTBR；不为训练成绩修改映射。根据实际 profile 计算并记录悬停对应值：名义 mg=1.4715 N、上限4.4145 N时悬停归一化动作约为 [-1/3,0,0,0]，零动作总推力2.20725 N。上一动作仍为送入 proxy 的物理 CTBR。26维观测只由 VecNormalize 归一化，关闭奖励归一化；actor 不读取 info 中的接触真值、评分状态或特权基线内部状态。

训练固定 seed=0、4个独立 DummyVecEnv、CPU和一个Torch计算线程、两层各64单元MLP。每环境rollout512步，batch256、5 epochs、学习率3e-4、gamma0.999、GAE lambda0.95；其余采用固定版本默认值并完整导出。单次运行合计不超过65,536 transitions及30分钟学习墙钟，所有学习阶段和保存／加载共用计时器、不重置预算。

必须在同一预算内完成前两个rollout/update（4,096 transitions）、保存并重新加载模型／优化器／归一化状态和训练计数、再完整优化一次（累计6,144）。恢复采用新环境，不恢复进行中的Episode或rollout buffer，不声称轨迹级精确续跑。随后只使用剩余59,392 transitions；预算先到即停止，续训未完成时标明未完成，不超时补跑。初始、续训前、最近有效及最终checkpoint均使用独立运行目录。

独立评价训练前和最终重新加载的策略，各一次原名义回合；使用新环境、确定性动作和冻结归一化统计。模型和统计成套保存恢复。前后使用对应checkpoint统计时，只报告完整训练流程行为变化；不单独归因于Policy参数。参数收益比较须使用同一观测归一化状态，不在本批额外安排此实验。

最少行为检查为动作映射与原CTBR链一致、上一动作语义、向量auto-reset保留终止观测／原评分结果、rollout边界不伪造终止、有限更新与参数变化、保存恢复后的确定性动作与状态一致、一次续训的参数更新／步数延续、评价统计冻结及持久化。前两个更新和续训检查属于同一次试跑，不另增训练预算；不重跑全部14项旧检查或扩成回归项目。

| I3 结果层级 | 证据边界 |
|---|---|
| 环境工程接入 | 接口、执行链、时钟、终止及记录检查 |
| PPO更新与续训 | 有限有效优化、状态恢复、参数变化及步数延续；是必须完成的工程项 |
| 完整训练流程行为变化 | 前后单次名义评价的回报、事件、时长及失败分类，不归因单一因素 |
| 名义接触恢复成功 | 原evaluator确认指定接触、脱离、恢复和全部门槛 |
| 稳定学习、可靠性与泛化 | 本轮未验证；参数变化、reward上升或单次成功各自不能证明这些结论 |

预算耗尽、不收敛或任务失败时保留必要结果并停止，不追加seed、搜索初态、改变成功条件或自动调参。工程接口／数值错误只做本批最小充分修正，并重查受影响行为；出现非有限loss／参数／梯度、来源不一致或不能持久化时停止并记录。I3完成后停止，不自动进入GPU、课程、服务器或后续研究。

## 8. Repository Separation（S1＋S2 ADOPTED；S3 PENDING）

来源 `578b09794407edd74e72327d5a89c5811c40e2a7`；独立 rm27_RL checkout
维护 RL 源码；唯一活动 Physics package 由独立 rm_uav_mujoco 仓库维护，保留祖先历史。移除的全系统目录仍在
旧分支和原工作区保存。工程 E1—E8 及研究接口保持不变。

Physics 发行名 rm27-contact-physics，导入名 rm27_contact_physics，版本0.1.0；
只依赖MuJoCo、NumPy、SciPy。模型、标记几何和profile通过显式配置输入。
从相邻 rm_uav_mujoco checkout 安装 Physics；未来原RM27通过完整Git SHA＋子目录或wheel
SHA-256固定来源，发行版本号不能替代内容固定。第三方只维护原一份哈希锁。
RL依赖Physics；Physics不依赖RL、Gymnasium、PPO、Torch或父工作区路径。

S2验收：实际Physics wheel在无RL/训练栈环境中安装及最小接口；独立checkout
安装、八个环境检查和三个PPO接入检查；已提交来源独立Docker构建；容器一次
原CTBR基线和一次旧I3 final冻结统计评价。六份配置原文不变；状态、事件、终止
及物理区间计数一致；连续数值rtol=1e-9/atol=1e-10、事件时间atol=1e-9 s。
旧checkpoint文件、模型/优化器/统计/计数和确定性探针保持一致，不追加训练。
任务失败可构成兼容检查通过，不改阈值或容差迁就迁移。

正常提交并发布非默认refactor/repository-separation；保留research/contact-rl。
不自动改变默认分支。S3需另行核对原仓库集成目标、消费者及缺失前置成果，
不能将来源研究提交当作原main可直接合入补丁。在S3完成前原系统仍使用旧实现，
不宣称两边已统一消费。历史结果只读，实际迁移结果更新原活动计划。

## 9. I4 无接触敏捷末端到达（I4 ADOPTED；能力按结果判断）

由策略自主决定 CTBR，从已定义初始区域连续进入指定终端状态集合；位置、速度、完整姿态、角速度和安全条件必须在同一物理采样时刻成立。首次合格到达立即终止，不要求非零速度状态在固定位置保持。`ARRIVAL_SUCCESS` 不属于 ContactRecoveryEnv 的接触、脱离或恢复成功。训练不使用 Teacher 动作监督、模仿损失或未来状态输入。唯一例外（用户2026-10-17明确批准）：训练 reset 可从离线动力学可行参考轨迹出发、经真实 `step` 推进得到的状态开始（informed reset／reverse curriculum）；参考只决定起始状态，不提供动作标签、观测或奖励，前缀转移不进入学习；评价始终从原 9 个初态完整起飞，终端集合与安全门槛不变。

用户确认的名义法向到达速度为 **3 m/s**，其来源为 45 g、10 m/s 与 150 g、3 m/s 的等动量参考：均为 0.45 kg·m/s。该参考不定义碰撞力、等效碰撞、裁判触发或真机安全；两者动能分别为 2.25 J 与 0.675 J。原 Contact 任务的 0.2—1.5 m/s 接触门槛不变，后续接触任务另行确定相容性。

**用户2026-10-19决定**：终端判定不再强制 3 m/s。正式终端集合改为中等放宽版：法向速度 1.5—3.0 m/s，gap 0.20±0.04 m，切向位置 ≤0.04 m，切向速度 ≤0.5 m/s，完整姿态 ≤15°，‖ω‖ ≤3 rad/s；仍为板前虚拟平面、同一物理时刻全部满足，安全门槛、初态与 9 初态评价不变（参数见 DESIGN §10.2）。原 3 m/s 集合保留为历史配置，此前各轮结果不按新集合重判。3 m/s 等动量参考仍只作背景，不定义裁判触发。

装甲前表面、bumper 中心、朝板几何表面点和瞬时材料速度必须分别定义。0.20 m 为候选球面至前表面法向间隙，不是球心间距或全机最小净空。板厚关系、速度公式及其余候选参数集中记录在 DESIGN §10；不把工程提案写成硬件标定或已实现能力。

I4 使用独立 `task_id` 和 `observation_schema`；维度相同不代表语义兼容。旧 I3 checkpoint 继续只在旧 Contact 任务加载，新任务独立初始化。Actor 不读取评分阶段、碰撞锁存、未来状态或 Teacher 动作。第一阶段运动状态来自 MuJoCo 真值，标记为 State-Based Training；隐藏执行器状态时不声称严格全状态可观测。

复用 Physics、CTBR、PPO、归一化、Docker 和来源记录；两个任务共用同一物理区间推进协议，各自拥有任务评分。原 Contact 的默认配置、奖励、终止、首次碰撞分类及 recovery hold 不变。新任务的安全失败优先于同刻到达成功；无接触并不授权关闭实体板、rim、地面或机体碰撞。

I3 的训练预算和固定续训验收只约束该批工程闭环，不能作为新任务训练器的永久硬编码限制。I4已批准的首轮训练与评价口径见 DESIGN §10；一个CPU seed、524,288 transitions／15分钟上限及三组各9回合，达到上限或失败后不自动追加。工程接入、到达能力、敏捷性和鲁棒性分别报告；计划完成或写入正式文档不代表能力得到验证。

I4 第二轮（用户2026-10-17批准）为一次单因素有界运行：只改变 Arrival 训练的动作层初始分布
（悬停居中的归一化映射与初始 log_std），预算与首轮相同；奖励、初态、终端集合、安全门槛和评分不变。
I3 与首轮配置的原动作映射保持。名义可达性的离线见证不放宽或替代 ArrivalEvaluator 判定；
结果失败时同样停止，不自动追加预算、seed 或调参。
I4 第三轮（同日批准）在第二轮配置上只增加上述训练用 informed reset，预算与前两轮相同，达到上限或失败后停止。
I4 末端精度塑形运行（用户2026-10-18批准）只增加一次性回合末奖励 `B·min(1,1/ρ_best)`，不改变到达判定、终端集合、安全门槛或评价；
预算与运行 C 相同，结果失败时停止。
运行 E 之后用户要求继续改进：单次 Arrival 训练上限扩大为 2,097,152 transitions／60 分钟学习墙钟（单 seed、本机 CPU），
其余冻结；仍不含 GPU、服务器、多 seed 或硬件。



## 10. 两阶段 PPO 与固定表面实接触（USER-CONFIRMED；独立于旧 I4）

使用用户指定的 RMUC2027 场地原生 base 和 drone_v2。目标为蓝色 base 朝场地中心一侧、
绿灯正下方的装甲板；以当前资产的实际表面与碰撞体测量注册，不沿用旧场地坐标或另造替代 base。
视觉网格名只是来源标识；任务命中需要指定板面区域与实际接触对，不能将整个同色网格或整个 base
视为允许接触目标。场地引用的 XML、网格、飞机与底部接触件配置均须保存来源哈希。

**Stage 1 — State-Based PPO Pretraining**：传统系统负责机库起飞、一般导航和目标搜索；到基地前
指定位置并稳定悬停后，PPO 接管完整后续机动。平台名义位置与姿态固定。Actor 使用可靠的模拟自机／
平台相对状态，自主决定连续飞行、加减速和转姿，通过既有 CTBR 执行链完成首次受控实体接触。
不使用 Teacher action imitation，不给定完整飞行轨迹、参考前缀或转姿时间表；训练初态保留小范围偏差。
本阶段终点是指定接触件与注册测试表面发生符合几何、底部法向、速度和安全条件的真实接触，
不要求 PPO 同时学会分离与完整恢复。正式交付须包括学会动作的 policy、完整回放、独立成功率及失败分析；
loss／reward 变化或少量 optimizer 更新均不能代替。

**Stage 2 — Vision-Based Fine-Tuning**：从 Stage 1 已训练 policy 继续微调，保留同一飞行任务与控制接口，
逐步把理想相对状态替换为 Tracking、VIO／PX4 等处理后的估计信息，并模拟噪声、更新频率、延迟和必要的
短暂失视。Policy 不负责 RGB 识别；TCN、移动平台预测和主动感知仅在证据表明有需要时再引入。
本轮只建设 Stage 1 工程基础，Stage 2 观测适配及继续微调流程尚未实现。

此前“连续移动进场、RL 只接末端姿态、不在板前停车”的 approach 结果保留为历史工程基线，
不再约束新 Stage 1 的稳定悬停接管和完整机动学习。工程 reset 可以初始化悬停状态，但这不证明已实现
从机库连续飞行接入 policy；未来真实控制器交接仍须保留同一物理状态、电机 act/ctrl、上一 CTBR 和必要的
控制器／估计器记忆，禁止靠重置制造交接成功。巡航及有效接触速度分别待任务确认，不继承旧 I4 末速。

此任务的实体接触不能用旧 I4 距板约 20 cm 的无接触成功代替。保留旧任务、checkpoint、seed、
评分及证据语义；新任务不得静默复用旧 checkpoint 的控制或观测语义。Physics 保持独立，
通用场景组合属于 rm_uav_mujoco 的 rm27_sim；本仓库仅由冻结场景产生显式私有任务变体，
不修改权威场景、Physics 或旧消费者。RL 环境、reward 与新任务 scorer 属于本仓库。被动接触夹具检查只证明
指定初态下的几何/接触行为，不证明进场可控、速度范围、全场安全或学习成功；新训练须另获预算授权。

### 10.1 已接入的 Stage 1 工程契约

`fixed_surface_contact_v1`／`fixed_surface_state_v1` 使用 `stage1_env.py`、`stage1_evaluation.py` 和
`stage1_plant.py`，当前配置来源为 `configs/stage1/task_contact_geometry.json`。26 维 Actor／critic 输入为注册表面中心相对质心位置
3、外法向 3、world 速度 3、body-to-world 旋转矩阵 9、body rate 3、上一 physical CTBR 4、剩余时间 1。
时间来自物理时钟；电机内部状态、接触标签、评分阶段、未来状态不进入网络。相同维度不允许与旧 task/schema
静默混用 checkpoint。动作复用 `NormalizedCTBR` 的四维 normalized CTBR，随后经过物理输入限幅、共享 rate proxy、
混控和原电机动态；5 ms 控制周期内按实际完成的 1 ms 物理区间结算事件。

Reset 的 `initial_perturbation` 是 12 维：目标坐标位置扰动 3、world 速度扰动 3、body 旋转向量扰动 3、
body rate 扰动 3；名义姿态 world 对齐、名义速度与角速度为零，电机内部状态初始化为悬停推力。
新任务固定评价集为中心加位置／速度／姿态各轴正负边界，共 19 例；角速度扰动另由 reset 工程测试覆盖。
这 19 例不替换旧 I4 九初态，也不代表已完成统计泛化评价。

独立 scorer 不读取 reward 系数。首次事件必须来自实际 MuJoCo completed-interval 求解的指定接触对和正向压缩力，
同时检验注册二维区域、板面距离、表面法向、底部法向、接触点法向／切向速度、body rate 与穿透。
目前轴对称球夹具只约束底部法向，绕法向的 yaw 不受限，不能称为完整姿态门槛。
初始重叠失败；未指定接触、飞行体积退出、超速、超角速、非有限值和 MuJoCo warning 均失败；
同区间安全失败优先于合格接触，超时仍是任务终止。事件记录区间起止时间和接触前后状态，不宣称精确的步内撞击时刻。

### 10.2 提案参数与授权边界

用户已批准一次 Stage 1-A seed0 的 simulation-only PPO baseline：
`configs/stage1/task_stage1a_seed0.json` 以 `CONFIRMED`／`training_allowed=true` 记录本次仿真假设批准，
`confirmation_scope` 明确排除实机标定。所有数值沿用 Geometry 配置，不改变 reward／独立 scorer。
原 `task.json`、`task_contact_geometry.json` 和既有结果保留当时的提案状态；下列 Foundation／Geometry 记录不再构成本次批准的阻挡。

当前数值、初态范围、接触／安全门槛及 reward 权重存于 `configs/stage1/task_contact_geometry.json`，
数值逐项沿用受保护的 Foundation `task.json`；旧文件及 `ppo_foundation.json` 保留原 checkpoint／回放语义。
状态为 `PROPOSED_UNCONFIRMED`、`training_allowed=false`。现有 ray-audit 区域是工程注册范围，
底部球是已有 S1c 几何假设，均不升级为官方有效区或真实飞机标定。未确认参数前禁止正式 Stage 1 训练；
不能把论文容差、旧 I4 数值或校验器可接受的上限当成训练授权。

Foundation 当时仅执行 `configs/stage1/ppo_foundation.json` 的有界工程 smoke：一个 CPU seed、一个环境、64 transitions、
一个 PPO rollout／epoch，学习墙钟最多 60 s。整体验证另受控制步、实际物理区间和墙钟预算限制；
场景编译、夹具、回放及正式 19 初态评价成本分别记录。产物入口为
`artifacts/agile_flight/stage1_foundation_01/`。本批工程结果：首批 26 项新测试、另补 1 项零步回放检查、67 项旧接口回归通过；
一次 64-transition PPO 更新、checkpoint 往返及四类完整物理回放通过。固定 19 初态均越界，成功 0/19。
实际接触夹具同时碰到注册面 `35364` 与未注册面 `36293`，独立 scorer 按原契约拒绝成功；
另一个区域外夹具在开始时已有结构重叠，未覆盖预期的区域外首次接触。失败与原始轨迹全部保留，
没有缩小评价集合、删除几何或扩张合法接触集合。接口工程完成不等于实体接触任务验收通过。
上述 Foundation 原判保留。接触后续核对已证明 `35364` 来自 `visual_115` 既有已登记 skin/rim 修正代理、
`36293` 来自既有确认装甲 `visual_117` 正面，二者在相同目标正面重叠，不是未知支架误触。
任务自有 `contact_surface_registration.json` 仅补齐该装甲正面的八个源分片 `36286..36293`（17→25），
以原 mesh／生成器／XML／MJB 内容及顶点身份核对，不修改原资产、物理模型、90×100mm 区域或 scorer。
同 geom 的侧／背面、任一同时非法 pair 仍须失败。原无该可选注册字段的配置保留原17面分类。
当前接触工程验证通过：6ms 中心接受；across=0.06m、gap=0.03m 的非重叠初态在61ms真实首次接触，
仅报 `CONTACT_OUTSIDE_REGION`；错误 core-board 接触79ms拒绝；原重叠夹具仍0步失败。
五例完整 episode 重模拟逐值一致，旧 checkpoint 只读载入通过，未增加策略更新。
证据 `artifacts/agile_flight/stage1_contact_geometry_01/`；几何阻挡已消除，可以进入 seed0 参数／预算批准阶段，
当时尚未获准正式训练；本次 simulation-only 批准与预算见§10.4。实体接触工程 PASS 不等于 policy 学会任务。

### 10.3 方法比较、恢复支线与历史隔离

PerchRL 原文 `arXiv:2606.03441v1` §3.2、附录 A.3/A.5 提供两阶段 PPO、CTBR、密集／终端 reward 与初态扰动的研究依据；
本任务只采用其适合固定平台的部分，差异见 DESIGN §14，不宣称逐项复现或继承论文成功率。
E0／E1-A 的解析模型、梯度和 BPTT 工程成果保留；E1-B 是旧 I4 学习实验，BPTT／SuperMimic／Flightning
转为后续方法比较，不是新 Stage 1 的前置条件。G2、X9、X18 及其他 PPO／SAC 结果保持原任务身份与结论。

Collision Recovery 另设支线：先基于已有恢复工程能力建立独立控制与估计器重新准入基线，
研究参考 Faessler 2015 和 Reachability-Based Collision Recovery；之后再比较 RL Recovery。
Stage 1 保存接触前后状态，未来评估这些状态对恢复结果的影响；两控制器最终须在不中断物理状态下交接。
本轮不把旧 `contact_recovery_v1` 的接触→分离→recovery hold 成功改作新 Stage 1 判定，也不将现有恢复夹具当成整合完成。

### 10.4 Stage 1-A 单次批准实验

`ppo_stage1a_seed0.json` 固定seed0／单CPU线程／单环境、512 rollout／128 batch／4epochs；最多131072个训练transition
或1800s学习，先到即停。学习时钟包含rollout、optimizer、训练期记录和checkpoint保存；实际控制步／物理子步独立硬计数，
未完成rollout消耗也必须报告，最终只保留最后完整有效更新。初始模型／来源初始化及已有MJB加载成本另报，禁止重新编译场景。
训练前后各原19初态，评测共同预算600s／30400controls／152000物理区间，学习期间暂停评测时钟。
预算耗尽保存不完整轨迹，不伪造终止或减少声明的评估集合；初期／中途同名义初态行为比较优先级低于两组正式19例，
同样消耗剩余评测预算。初始、首个完整更新、首次到半transition或半学习时限的完整更新、最终有效更新均保留checkpoint。

全部训练曲线及Episode终止／接触记录保存，首个成功与最多12类代表失败另存完整随机训练轨迹；
随机训练Episode可能跨越policy更新，须与冻结checkpoint的确定性评测区分。检查位置／速度／姿态／CTBR、world yaw与旋转、
悬停／绕圈、裁切／饱和、真实接触前后状态及全部失败。工程更新、受控接触学习、敏捷连贯性和局部初态稳健性分别作结论。
本次预算执行后停止，不自动增加seed／训练轮次、修改reward／成功条件或进入Vision／Recovery。

已观察结果（2026-11-04）：唯一seed0完成131072 transitions／256次完整rollout更新，1501.227s学习；
原19初态前后均0/19，后评估全部角速度超限，接触能力未学会。工程有限更新PASS不等于任务、敏捷性或稳健性PASS。
完整证据见`artifacts/agile_flight/stage1a_seed0_01/analysis/index.html`；本次授权已消费完毕，不启动后续训练。

### 10.5 已批准的 Stage 1-A Action Scale 匹配比较

用户明确批准在原`fixed_surface_contact_v1`重训seed0，仅把policy rate映射beta从1改.1；不是旧I4无接触任务。
`ppo_stage1a_beta01_seed0.json`引用原task配置，必须从原零update initial actor/critic、optimizer和normalizer恢复，
禁止续训final。原19初态、reward/scorer/真实接触成功、env bounds、安全终止及PPO设置保持，预算同§10.4。
新结果独立保存，每次完整更新checkpoint/normalizer、所有训练episode和失败轨迹均保留；记录I/O计入学习预算。
比较学习曲线、正式成功率/失败、实际动作可行性、gamma=.999完整评估回报及initial/early/middle/final行为；
normalizer随不同rollout演变属于学习流程差异，不能宣称beta是全部失败的唯一原因。完成一次后停止，不自动扩展任务。

本次比较已完成并停止：预算内131072controls，正式前后0/19；受控接触能力未通过。该结果不修改上述成功契约，不授权追加训练；证据及后续提案统一见活动计划§0。

## 11. 可微分动力学 E0（独立工程核对，不授权训练）

用户确认研究自主策略训练的可微分动力学／BPTT，E0 只实现 RL 侧解析代理与
前向、梯度验证。权威 Physics、旧 I4 评分器、固定九初态、PPO/SAC checkpoint、
旧成功标准及实验结果不变；不引入强制 Teacher/Student 或 imitation。
不以 E0 通过宣称策略更快、更可靠或新赛季实接触成功。后续E1-A授权与边界见§12；
完整E1-B训练仍需明确预算与授权。

可复现入口为 `python -m rm27_contact_rl.diff_validation --output <fresh-artifact-dir>`，
配置为 `configs/agile_flight/diff_e0.json`。使用既有 train 环境，无新依赖；每次
使用新目录，已开始的运行拒绝覆盖。保护来源与旧轨迹在计算前后核对 SHA-256。
只读重放 G2 PPO seed0 与 X9 SAC seed0 各九条固定评测输入，不等于重新评估策略。

预定128状态覆盖 hover、maneuver、terminal approach、saturation，各32；
前向跨度为5/40/160/640ms，按时域、任务组和限幅区域分别报告P95／最大值。
根据用户最新调整，640ms P95位置≤5mm、速度≤0.05m/s、姿态≤1°、体角速≤0.1rad/s
作为研究目标；是否可用于BPTT取决于实际误差来源、任务覆盖与梯度可用性，不能只用
长时域目标判定可训练性。保留原审计的配置、数值和旧判定，补充分析另存新产物。

梯度区分内域光滑、严格饱和、限幅边界与不连续事件。严格饱和内部可有正确的零动作
导数；精确边界不假定唯一Jacobian，分别核对左右方向导数／小步割线。事件发生在
第五子步同样要排除，禁止跨终止或reset传播。至少按任务组及每个时域报告实际覆盖，
同时核对独立动作方向和独立初态方向，避免初态传递掩盖动作梯度错误。

float64光滑区自模型中心差分相对参考1e-4，完整MuJoCo链相对参考5%、有效cosine参考
0.95；近零导数结合绝对误差判断。补充分析固定归一化尺度为1m、四元数微分/0.5、
6m/s、20rad/s、1.103625电机N，判据 `absolute <= 1e-7 + rtol*reference_norm`。
范数≤1e-8时不计算relative／cosine；即使大于该值但接近差分噪声，也需报告量级及
epsilon敏感性，不把相对误差单独当失败。边界单侧割线不是高精度中心差分；自模型
单侧核对亦使用绝对加相对判据，并明确其有限步长含义。

E0总审计上限65536次reference控制转换、2000000个MuJoCo物理子步（含差分与旧输入
重放）、1800s计算时间，先到即停。补测继承已用预算，不重置额度；未插桩开发检查
成本单列，补测另作保守预算预留。预算内不能完成时，保存完成探针、当前探针起始
计数和实际时域覆盖，判为PARTIAL_BUDGET，不扩大预算补齐。非有限、时钟异常、
MuJoCo warning、来源变更即时失败。没有学习步骤，不将模拟控制转换称为训练步数。

原审计入口 `diff_validation.py` 保留原协议；最新补充分层入口为
`python -m rm27_contact_rl.diff_analysis --audit <existing-audit> --output <fresh-dir>`。
除HEAD／dirty清单外，还固定实际载入模块、必要未提交来源内容、配置／依赖锁及
编译模型参数；补充来源包只是本实验的可重建证据，不成为第二份源码权威。

代理限于 MuJoCo3.12.0、I4独立自由体、5×1ms、零流体力／关节阻尼、无外力输入、
filterexact 电机与 proxy CTBR 路径；不涵盖碰撞、PX4 rate loop 或任意 MJCF。
调用者必须由正式评分器在每个1ms端点判断事件；代理本身没有成功或碰撞判定。
当前既有运行时质量缩放可能使 `body_mass` 与 MuJoCo `dof_M0` 不一致；E0 明确拒绝
这种状态，不在代理中偷偷校正、不改权威实现、不重判旧 DR 结果。该不一致另待解决。

## 12. E1-A 可微分策略工程接入（2026-11-02 用户明确授权）

本批只实施80ms短窗口BPTT与有界最小更新验证。沿用E0冻结的名义MuJoCo3.12.0、
I4配置、26维 `arrival_world_v1`、hover-centered normalized CTBR和正式ArrivalEvaluator。
旧Physics、PPO/SAC、奖励／评分器、固定九初态、checkpoint和历史实验不得改动；
不使用质量缩放缓存不一致路径，不引入Teacher/Student、模仿、GPU或JAX。

Actor与critic都仅使用原26维观测。电机act保持在17维内部物理状态中；上一物理CTBR、
时间与评分器累计历史跨窗口保留，不能重建为新episode。窗口边界detach历史；窗口内
全部已执行控制保留计算图。原评分器逐1ms判断事件，包含第5子步；事件之后无推进、
奖励或reset梯度。任务逾时属于termination，和成功／安全失败一样取消bootstrap。
接触不属于解析模型支持范围，真实接触引起的前向差异必须拒绝，不能伪造接触梯度。

Actor目标为原逐控制奖励的折扣和，加尚未终止窗口的 `gamma^k V(obs_k)`。
事件／限幅分支选择不求导；连续奖励保留对应分支导数。Actor更新冻结critic参数，
但保留bootstrap对末端观测的梯度；critic拟合使用完整detached的窗口return。
观测保留原float32转换，随后CPUfloat64网络使用固定物理尺度归一化；转换的autograd
约定不等于精确量化函数可微。梯度检查结合绝对误差并避开小量差分噪声与事件切换。

验收必须包括完整actor→action→dynamics→objective→gradient链、有限非零梯度、
参数更新与目标方向、critic梯度隔离及固定target误差、饱和零梯度、逐子步事件遮罩、
原MuJoCo九初态评估接口和独立checkpoint保存／加载。工程PASS不能替代任务成功；
3次更新不证明critic足够准确或自主学习收敛。加载须核对任务、配置、动作映射、
观测尺度、来源、MuJoCo版本与编译参数，禁止伪装成旧SB3格式。

唯一smoke配置 `configs/agile_flight/diff_e1a.json`；入口
`python -m rm27_contact_rl.diff_smoke --output <fresh-artifact-dir>`。
单seed、单CPU线程、8个窗口／批、16控制／窗口；最多3次actor更新、6次critic更新，
总上限8192次reference控制、40960物理子步、300s，含差分、事件探针和原接口评估。
每个控制执行前保守预扣5子步，另外报告实际子步；先达即停并报告PARTIAL_BUDGET。
非有限、MuJoCo warning、前向不一致、来源变更立即失败。不自动扩大额度、重跑训练或
进入E1-B；单元／既有回归成本与smoke分开报告。原始E0证据保持只读，新产物不覆写。

## Simulation ownership update (2026-10-21)

User-authorized repository separation moves the unchanged Physics package to
`rm_uav_mujoco/packages/physics`. RL retains Gymnasium environments, reward,
training, checkpoint handling, training configurations and its Docker launcher.
Package names, version, third-party lock and scoring remain unchanged.
Complete mission integration belongs to parent `rm27/` and does not require RL.
Historical root mission implementations are preserved; this relocation does not
claim that their pending S3 algorithm adaptation has been completed.
Standalone RL installation requires the external Physics checkout or an installed
Physics wheel. Docker builds accept `RM27_PHYSICS_ROOT`; source runs mount both
checkouts read-only. Historical S1/S2 evidence describes the earlier layout.

## 13. Independent State Flight Control Probe (USER-AUTHORIZED pilot)

`state_flight_control_probe_v1` is an independent noncontact diagnostic, not contact or I4 success. Preserve the authoritative MuJoCo plant, original contact reward/scorer/checkpoints and dirty worktree. Hover and world ±x/±y 0.20 m references are fixed at reset; no reference trajectory, teacher or privileged motor observation. Four-second episodes retain original safety bounds, 5 ms CTBR / 1 ms integration, 40 ms motor response, beta=0.1 and the existing collective mapping.

Actor schema `flight_reference_state_v1` has 26 entries: reference-minus-position (3), world-up (3), world velocity (3), rotation matrix (9), body rates (3), previous physical CTBR (4), remaining time (1). Training reference distribution is 50% hover, 12.5% each shift with original small initial perturbations. Any physical contact fails. Success requires the full final 0.5 s continuously within 0.05 m position, 0.10 m/s speed, 5 deg tilt and 0.20 rad/s body rate, with no safety failure. All 19 hover perturbations plus four nominal shifts remain in the formal denominator.

A/B share original process reward forms (reference distance potential, near-reference upright alignment, action/smoothness, safety and time costs) and +20/-5 terminal events. Only B adds `-dt*C`, where C is the sum of capped squared position/.20 m, speed/.50 m/s and tilt/20 deg errors, cap 4 per term. These preregistered simulation assumptions are not hardware safety thresholds. Open-source comparisons inform design; no simultaneous upstream action, PPO or reward-coefficient migration is authorized.

Same seed0 actor/critic/optimizer/normalizer and reset RNG; CPU1/environment1; rollout512,batch128,4epochs. Each arm <=65,536 controls or900s; combined <=131,072 or1800s; independent evaluation <=900s and55,200 controls. Record actual complete episodes, initialization/compilation and all failures. Preserve Gaussian sample/mean/std, executed actions, actual interval motor forces, reward components and pre-minibatch GAE separately. Stop after this single-seed pilot. B support gate: hover>=18/19, shifts>=3/4, both families' mean integrated error >=30% lower than A with non-increased speed RMS. Failure within this budget does not universally refute dense reward.

Observed pilot outcome: engineering feasibility PASS (independent PD5/5), PPO learning NOT_DEMONSTRATED (both final0/23, all volume exits). Both arms completed65,536 controls and128 finite updates; no budget extension. B increased early failures and low-thrust behavior; lower censored error integrals do not establish improvement. Preserve this single-seed negative result and the separate contact0/19. Further training needs a new approved experiment; do not infer universal failure of dense rewards.

### C: approved fixed absorbing-failure cost diagnostic

User-approved C retains all B flight-task, action, PPO and scoring settings. Only early safety-failure terminal reward adds a fixed 12/s discounted cost for the remaining 4s horizon. Gamma=.999 per 5ms control, including the unfinished terminal slot at exponent zero and later slots at successive exponents. Physics stops immediately; no synthetic transitions, time-limit truncation or critic bootstrap after failure. Full-horizon failure/success adds zero. C starts from the saved B initial policy/critic/optimizer/RMS. One seed0 run <=65536 controls or900s; original23-case evaluation within prior900s ceiling. No extra training; compare actual flight coverage and coordination separately from success. Historical B and contact results stay unchanged.

C observed: latest complete update125/64000 optimized controls, external execution exit143 of UNKNOWN origin; no restart. All23 evaluations complete,0 successes and0 full-horizon cases. Speed/tilt and zero-thrust/clipping improve against saved B final, but position error and yaw do not; all cases exit the flight volume. Saved158 complete episodes and129 checkpoints; interrupted in-memory tail is unavailable. C remains a diagnostic configuration, not a validated flight capability or a replacement for B/contact scoring.


### Accelerated working baseline (separately USER-AUTHORIZED)

`state_flight_working_v1` is a new noncontact policy experiment against the unchanged flight-probe independent scorer: Hover>=18/19 and Shift>=3/4, full final0.5s at the existing position/speed/tilt/rate thresholds. Authoritative Physics, original A/B/C and contact benchmarks remain frozen. Joint observation/action/reward/PPO redesign is explicitly authorized; results are not a single-factor causal comparison. No teacher labels, prescribed action trajectory, hardware or publication.

Shared ceiling: at most two evidence-driven candidates, total1,000,000 training controls or3600s learning wall, whichever first; independent evaluation<=600s. Learn hover first and enable shift training only after the original19 hover gate passes. Increase allocation only with measured behavioral improvement. Configuration and source identities are recorded with each run. Success requires original physical flight evaluation, not reward, clipping or flight duration alone.


Observed working-baseline result: candidate2 final393216 controls passes all19 hover and4 shift cases under the unchanged scorer. Candidate1 remains0/23 after196608 controls. Shared run consumed589824 controls/1695.478s learning; formal/periodic/PD evaluation378.757s, with additional10.687s candidate2 engineering check. This establishes a single-seed nominal-Physics noncontact flight baseline, not contact, vision, hardware, multiseed robustness or agile-flight performance. Preserve all earlier negative outcomes.
