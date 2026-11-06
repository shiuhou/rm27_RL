# RM27 Agile Flight RL — DESIGN

当前主线为从稳定悬停状态学习完整机动与受控底部物理接触的两阶段 PPO；任务要求归 SPEC §10，
当前接入见 §14。E0／E1-A、旧 I4 和移动 approach 的设计保留各自适用范围。

## PDF season and historical interface batch (2026-10-30)

Current season source and component role are stated in the repository README.
Previous I0–I4 task contracts, checkpoints, scoring and training budgets are historical
experiment boundaries and are preserved. The new season goal is repeated contact
and recovery. The historical interface batch below does not supersede the current
two-stage PPO allocation. This interface batch itself authorizes no training.
`interface_worker.py` uses contact_world_v1, the existing
observation formula and NormalizedCTBR with an untrained fixed-seed actor. This is
an inference/transport test, not First Contact implementation or policy success.
Physics stays external and no parent implementation is imported by the worker.

日期：2026-10-13。状态：**I0／I1／I2／I3 ADOPTED**，依据用户对各批次的明确实施请求；I3为有界CPU PPO工程闭环，不代表学习成功。后续扩展研究／部署仍待批准。目标、工程规则与验收仅维护于 [SPEC.md](SPEC.md)，具体运行结果维护于对应任务的唯一活动计划。

现有 §1—§9 保留 I0—I3 与 S1/S2 的实现及兼容契约。I4为新增独立任务；用户2026-10-13明确批准以新活动计划实施，§10作为首轮 **ADOPTED** 设计。接口采用、工程通过和学习效果分别判断；附件中的阅读声明和性能目标不自动成为运行事实。

## 1. 来源与组织

独立仓库从 `578b09794407edd74e72327d5a89c5811c40e2a7` 保留历史建立，审阅分支 `refactor/repository-separation`。此前I0—I3源于ec9db048的接触研究；旧工作区及证据不迁移或清理。

研究代码位于 `src/rm27_contact_rl/`；共享物理、控制及历史 Contact／Arrival 评分实现位于 `../rm_uav_mujoco/packages/physics/src/rm27_contact_physics/`，新 Stage 1 的独立任务 scorer 位于 RL。原RM27消费者接入留待S3；旧分支实现是保留的历史/过渡来源。原生 MuJoCo 为参考后端，不预建 GPU、多后端工厂或仿真服务。PerchRL 提供研究启发，不复制其质量、时钟、成功率或附着终止条件。

```text
基线或策略 CTBR
  → 环境校验／裁剪
  → RateThrust／apply_proxy
  → 控制分配、单电机限幅
  → filterexact 执行器、六自由度和物理接触
  → 共享逐物理区间 evaluator
  → 事件 reward、终止及26维观测
```

## 2. 模型与参数来源

复用 `rm27_contact_physics/drone.py`、`rm27_contact_physics/model.py` 和 `configs/aggressive_contact.json` 的局部 armor／rim／bumper 场景。底部球形 bumper 的偏移及半径继续由原配置负责，其他机体禁碰几何保留。

`configs/contact_rl/minimal.json` 引用原场景、`configs/drone.json` 的机体几何和 `configs/dynamics_150g.json`，只增加 RL 特有值，不复制一套评分阈值。通过 `calibration.with_profile()` 应用质量、惯量、推力、lag、yaw override，并用 `verify_compiled()` 核对。`build_model(cfg, nominal_cfg, *, vehicle_config)` 必须显式接收机体配置；`load_config(path)`、`marker_center(target, marker_config)`和`Skill(..., marker_config=...)`不再寻找父仓库文件。Profile证据路径仍相对profile文件解析。其输出 yaw 系数与实际 override 一致。

150 g 质量有用户来源，其余为工程估计，未完成2S真机标定。四个旋翼总上限为 4.4145 N，不用旧推重比重新推导覆盖 profile。原180 g direct-motor 对照在原RM27保留；guard-tip 不属于首版允许接触集合。

## 3. 控制、基线与观测

### 控制链

动作顺序 `[collective_force_N, roll_rate, pitch_rate, yaw_rate]`，body FLU，推力沿 body +Z，rates rad/s。世界是局部右手 +Z up 场景坐标，原点沿用配置；不能把未标定场景轴称为地理北。旋转矩阵 R 为 body→world。

物理1 ms、控制／策略5 ms、rates ±20 rad/s、任务6 s来自配置，是研究设定，不是实机能力。一次策略调用计算 proxy 电机命令并保持到下一控制边界。`teacher_interface.apply_proxy()` 使用惯量、机体系角速度、名义 rate gain `2*damping*frequency`、共享 mixer inverse；限幅后命令送入 `data.ctrl`，MuJoCo actuator 保留40 ms工程估计电机响应。

共享控制分配函数在原 `control.py`，Skill 与环境都使用它；不复制电机编号、roll／pitch／yaw符号或评分逻辑。rate proxy 是近似研究低层，不是 PX4 PID。未来 PX4 映射需独立核实 thrust 与 FLU→FRD，不把历史 ArduPilot wire 当当前部署接口。

### 基线

原RM27中的 `Skill` direct-motor runner 保留回归与历史对照。新 CTBR 基线为 `Skill.command()` → `project()` → 同一个 `env.step()`，施加后将实际电机命令回写其电机观察器。它读取真实接触锁存，明确为 **privileged model-based baseline**；相同执行链不证明观测公平。基线自身投影／限幅诊断与环境实际命令饱和统计分别命名。

### 观测与reset

26维 float32：目标相对位置3、目标法向3、自身速度3、按行展开R9、body rates3、上一proxy输入CTBR4、剩余秒数1。上一动作在实际送入proxy后更新，为输入边界裁剪后的指令；不混作实际推力、测得角速度或电机效果。精确电机、contact bit及evaluator phase不进入actor观测。

reset沿原名义场景位置、单位姿态与零速度。`data.act` 和 `data.ctrl` 均为 profile 的 mg/4，初始上一动作 `[mg,0,0,0]` 是初始化约定而非真实历史。Skill电机观察器和previous motor command同为mg/4，specific thrust ratio=1，姿态／速度记忆取reset状态，时间记忆清空。

每个env独立持有MjData、evaluator、动作记忆、接触锁存、随机源及baseline可变状态。固定只读模型可复用，不要求新增模型共享机制；未来随机化动态参数时必须独立模型。reset不启用随机化，不自动调用现有随机场景。

## 4. 唯一逐物理区间采样协议

共享 `rm27_contact_physics/evaluation.py` 拥有本仓库唯一接触／恢复评价。旧runner不迁入本仓库；其消费者适配为S3，当前不改原仓库调用或评分。

现有模型为 `implicitfast`。每个 `[t_k,t_{k+1}]` 唯一执行顺序：

1. 起点保存S_k；控制边界先更新电机输入。起点快照已被消费，不能再次生成事件。
2. 仅调用一次 `mj_step()` 推进物理到t_{k+1}。
3. `mj_forward()`前读取本次积分内部求解留下的有效接触力；它关联该区间起点求解阶段，不能按已更新的data.time误标为终点力。
4. 按实际Δt结算一次F_k*Δt以及饱和／必要接触时长。
5. `mj_forward()`刷新终点S_{k+1}和全部有效接触几何；时间不变。
6. 消费终点事件；首次接触时间为t_{k+1}，全部同时碰撞对来自终点，接触前运动学来自S_k。按物理网格定位，不虚构插值碰撞时间。
7. 完成违规分类和恢复更新，再检查终止。失败优先；锁存结果后停止当前策略步剩余子步。

复用 `contact_metrics.py` 的真实接触测量。区间求解力用于冲量；现有终点快照力的峰值诊断明确来源，不额外累计冲量。reset快照没有完成区间，积分为零；额外查询、mj_forward或策略步末刷新不增加时间、冲量或hold。相邻步边界不能重复结算；返回读取已缓存评价。

终点t_s首次稳定时设置hold_start=t_s，之后按timestamp差计时；任一采样点不稳定立即清空。`recovery_time_s=hold_start-first_contact_time`，保留2 s内进入稳定再保持1 s的原含义。保持首次同时碰撞、短触锁存、二次碰撞及不可逆失败；历史separated不能代替当前无接触／间距要求。

旧runner此前循环顶部积分后才判断部分停止条件，终止边界／末端取样的实际区间归属会被修正，相关历史诊断数值不承诺逐项相同；阈值、事件定义和公开字段保留。旧runner保持完整回合或既有停止协议；完整回合轨迹仍止于最后控制tick，给后续任务阶段保留data.time交接点，避免重复时间戳；新env在成功／不可逆违规／6 s任务期限时结束。正常step五子步，终止step可更短；状态、时间、reward和原因对应停止时刻，不继续推进凑5 ms。中途快照不做最终TARGET_MISS结算。

首版reward按SPEC事件项，只用于最小环境，不宣称PPO可学，不进行调参。任务期限用terminated；没有额外运行器时间限制，首版truncated=False。

## 5. 包与运行环境

根 `pyproject.toml` 声明RL；`../rm_uav_mujoco/packages/physics/pyproject.toml` 声明独立Physics。
第三方版本仅由原 `requirements-contact-rl.lock` 固定，27个依赖版本不变。
Physics wheel可单独安装；RL当前支持完整checkout的editable安装，配置由
`rm27_contact_rl.paths.ROOT` 从安装源码定位，不依赖cwd或父工作区。
普通环境入口不加载Torch；train extra为SB3 2.9.0与官方CPU Torch 2.14.1+cpu，
限定Linux amd64/CPython3.12。Physics只需要MuJoCo/NumPy/SciPy及其传递依赖。

```bash
python3.12 -m venv .venv
uv pip sync --python .venv/bin/python --require-hashes requirements-contact-rl.lock
uv pip install --python .venv/bin/python --no-deps --no-build-isolation -e ../rm_uav_mujoco/packages/physics -e .
# 仅构建独立Physics；输出按需生成并忽略。
.venv/bin/python -m pip wheel --no-deps --no-build-isolation ../rm_uav_mujoco/packages/physics --wheel-dir artifacts/wheels
```

更新锁需独立理由；同时解析两个打包声明，并将单独安装的第一方Physics包排除在第三方哈希锁外。本次保留原锁字节及所有依赖版本。
从相邻 rm_uav_mujoco checkout 安装 Physics；未来原RM27用完整Git SHA和`subdirectory=packages/physics`
或固定wheel URL/SHA-256消费。S3未实施，版本0.1.0不代替内容标识。

生成锁只证明解析，安装只证明安装；行为证据另外报告。宿主登录环境可能设置ROS的PYTHONPATH；运行时使用 `env -u PYTHONPATH` 排除宿主附加包路径，测试另设 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。测试依原计划选定节点及根 `tests/test_contact_rl_env.py`，不自动执行全仓测试。

固定名义初态一次有界基线入口：

```bash
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl baseline --output artifacts/contact_rl/nominal_150g_ctbr
```

可通过 `--config` 指定实际配置文件。入口保存必要的配置、轨迹与summary；退出0表示任务成功，退出2表示已记录的基线任务失败，执行异常退出1。基线失败不是环境接口验收失败，也不能隐藏其退出状态。

## 6. Docker（I2 ADOPTED）与后续

单一 CPU headless 镜像定义于 `docker/Dockerfile`；根 `.dockerignore` 白名单纳入实际工作树的必要源码、打包声明、锁及所需配置（包括 `configs/base_target.json`），不按 Git 跟踪清单裁剪。排除 `.git`、宿主环境、缓存、测试、CAD、子模块及历史结果。使用官方 `ubuntu:24.04@sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517` 的 amd64 基础镜像，容器内仅安装 Python 3.12 venv 和 CA，随后按唯一完整哈希锁安装，不另维护容器依赖清单。I2已保存镜像使用原test/build集合、不包含Torch；I3在同一Dockerfile／锁加入获准CPU训练依赖，使用新镜像标识，不覆盖I2成果。CUDA及渲染栈仍不引入。`MUJOCO_GL=disable`，不创建 renderer。

editable 安装位于 `/opt/venv`，镜像自带的源码及配置在 `/workspace`，默认从 `/tmp` 运行。开发时可选择同路径只读源码挂载，不遮蔽依赖；改变依赖声明或锁时须重新构建。基础 digest 和 Python 锁固定输入；Ubuntu 软件包下载仍依赖构建时的软件源，最终构建镜像 ID 固定实际运行环境，不宣称仅凭基础 digest 可逐位复建全部系统包。

唯一启动入口 `docker/run.sh`：

```bash
# 在 checkout 根构建当前 I3；包含尚未提交文件，不覆盖保存的 I2 镜像。
bash docker/run.sh build rm27-contact-rl:separated-cpu

# dedicated-results 替换为宿主专用结果目录；run-id 必须尚未存在。
bash docker/run.sh run rm27-contact-rl:separated-cpu dedicated-results run-id --check

# 可选开发源码挂载；仍使用同一镜像和锁，输出使用另一个新标识。
bash docker/run.sh run rm27-contact-rl:separated-cpu dedicated-results new-run-id --source
```

入口从宿主采集 Git HEAD／dirty／status。构建时在 `/opt/contact-rl/build-source.json` 保存纳入上下文每个文件 SHA-256、聚合标识和锁标识；该记录位于源码挂载之外。CLI 复用原来源记录，运行时仍计算实际使用的代码及配置哈希，另记录基础镜像、构建快照、实际运行镜像 ID 和 IMAGE／BIND_MOUNT 来源。无 `.git` 的镜像不调用 Git；开发挂载从宿主传入当前 Git 身份，避免 checkout 指针失效。原生 venv 运行继续使用 Git。镜像自带源码是未提交实现的必要载体，HEAD 不足以还原它；镜像不自动推送。

结果仅写独立 `/data/<run-id>`，通过宿主专用目录 bind mount 持久化，原 `nominal_150g_ctbr` 不覆盖。入口解析镜像 tag 为 ID 后运行，使用调用者 UID:GID、只读根文件系统、限额 `/tmp`、无网络、无 capabilities，不使用 privileged。`--check` 在同一会话先检查导入／配置／reset／单次 step，再 exec 原 CLI 一次基线；失败状态继续用原 0／2／1 语义，不复制评分或基线。

I2仅检查镜像构建及默认镜像源码路径；可选源码挂载不额外运行一次基线。I3的新增检查范围见§7。使用非本机 UID 的服务器目录权限应由其调用者在获准阶段安排。迁移可通过获准 registry 或 `docker save/load` 保留镜像内容；I2／I3不推送、不操作服务器。

先Git＋独立venv，再Docker＋外部数据；I2／I3原生CPU MuJoCo及CPU PPO无需CUDA。未来采用GPU PyTorch时需要同时支持5060 Ti的sm_120与4090的sm_89，并检查宿主驱动／Toolkit。Docker不解决宿主驱动。服务器权限、驱动、CPU／内存、存储和调度UNKNOWN；先单设备迁移再扩独立实验，不预设七卡联合训练。

I3的成熟PPO接入见§7；后续课程、历史观测、视觉误差、延迟丢帧、动力学／接触随机化和PX4路线按独立任务进入。只有实际吞吐瓶颈与收益明确才评估MuJoCo Warp；原生接触参考保留独立数值对照职责，不为GPU先重写全部动力学。
MuJoCo Warp（GPU 并行仿真）＋mjlab（训练框架）＋viser（网页查看器）是候选加速手段（用户 2026-10-22 记录，未实施）。触发条件：
单轮实验等待超过约 1 小时，或需要 ≥5 个 seed 才能判断可靠性；或要做大规模随机化／超参数扫描。满足时由执行方主动提出可行性提案——
吞吐测量、与原生 MuJoCo 逐步数值对照、单 seed 训练后用原生评价器打分——经用户批准后才实施。原生 MuJoCo 始终是评分基准。

## 7. CPU PPO、Checkpoint与评价（I3 ADOPTED）

训练逻辑集中于研究包的`ppo.py`，现有CLI增加`train`与`evaluate`，原`baseline`保留。`configs/contact_rl/ppo_minimal.json`通过`env_config`引用原`minimal.json`；只持有训练配置，不复制机体、接触门槛或reward。设置为seed0、4个独立DummyVecEnv、Torch单计算线程、两层64单元MLP、CPU PPO；每环境rollout512步、batch256、5 epochs、学习率3e-4、gamma0.999、GAE lambda0.95，其余固定SB3默认值导出。

Gymnasium动作wrapper将四维[-1,1]按`low+(clip(u,-1,1)+1)*(high-low)/2`映射到原CTBR，仍通过原`env.step()`执行。悬停的归一化指令根据profile和实际action空间计算并记录；名义150 g时约为[-1/3,0,0,0]，零动作对应2.20725 N。wrapper不修改原观测上一动作字段。Monitor及VecNormalize置于环境外，obs归一化、reward不归一化；info只用于评分与记录，不作为actor输入。向量auto-reset的`terminal_observation`及原评分结果保留；rollout边界允许bootstrap，但不设置任务truncation。

所有学习阶段共用最多65,536 transitions／1800 s计时。前2个完整rollout/update（4,096 transitions）检查有限更新，保存`pre_resume`；关闭训练环境后新建环境，加载模型、优化器、归一化统计和训练计数。使用延续计数的学习调用再优化1个完整rollout（累计6,144），检查参数更新及步数延续；剩余预算最多59,392 transitions。保存／加载及阶段切换不重置计时器，不恢复进行中的Episode／rollout buffer。工程检查与续训均计入同一试跑；墙钟先到即停止并记录未完成项。

每个独立运行目录的`checkpoints/{initial,pre_resume,latest,final}/`按实际达到的阶段创建，每个有效checkpoint成套包含`model.zip`、`normalize.pkl`和`checkpoint.json`。SB3模型保存策略、优化器及计数；元数据记录配置、checkpoint标识和实际来源。`latest`仅保存最近有效状态；不以无效参数覆盖有效checkpoint。复用现有来源记录，算法标签按baseline／PPO实际入口填写，并纳入训练代码、配置、锁及模型标识，不建立第二套provenance框架。

checkpoint先在同一结果目录暂存完整文件组，再替换目的目录；写入失败保留上一有效组。加载校验文件哈希、策略／优化器／统计内容及计数，另用原始观测探针核对确定性动作。逐次learn请求一个完整rollout并延续计数；当前学习率与clip为固定常量，后续若引入动态schedule需重新审查进度语义。

评价通过同一函数和CLI入口，在新环境中加载checkpoint，以确定性动作运行一次最长6 s的原任务。训练前评价加载`initial`、训练后加载`final`；VecNormalize统计冻结且reward保持原事件项，评价不能反向修改训练统计。默认前后使用各自统计，只比较完整流程行为变化，不将差异归于Policy参数本身；单独参数比较才需同一归一化状态，本批不另跑此实验。

评价直接调用同一新建的Monitor／NormalizedCTBR／ContactRecoveryEnv实例，使用加载的`normalize_obs`向策略提供输入，保存环境实际返回的原始终止观测；不逆变换经过clip的terminal observation来重建轨迹。初始轨迹行标记`action_applied=false`，不表示零动作曾执行。初始评价在学习计时前、最终评价在学习计时后；学习过程中的保存／加载／切换均计入同一计时器。训练工程通过退出0，预算内工程项未完成退出2，工程异常记录原因／traceback并退出1；独立evaluate沿用任务成功0／任务失败2语义。

```bash
# 既有训练入口示例；S1/S2不运行训练。
bash docker/run.sh train rm27-contact-rl:separated-cpu artifacts/contact_rl/docker ppo_20261013_i3

# 独立评价入口示例；属于调用方式说明，不要求本批额外再跑一回合。
bash docker/run.sh evaluate rm27-contact-rl:separated-cpu artifacts/contact_rl/docker new-eval-id /path/to/old/checkpoints/final
```

原生入口为`env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl train --output <new-run-dir>`（可选`--config`）；评价为`evaluate --checkpoint <checkpoint-dir> --output <new-eval-dir>`。Docker的train／evaluate复用同一入口与`/opt/venv`，结果及checkpoint在独立`/data`挂载；evaluate接受现存checkpoint目录，在独立只读`/checkpoint`挂载；不得与可写结果目录重叠。运行标识必须新建；训练不附带额外baseline或全量pytest。

## 8. 证据和完成边界

工程检查、150 g特权CTBR基线及策略学习证据分开。基线失败保存必要摘要／轨迹，不放宽阈值或反复搜初态；新增环境具体错误只局部处理，不能扩成控制优化。Docker定义、构建及运行状态分别记录于活动计划。I3分开报告环境接入、PPO更新／续训、完整流程前后行为和原评分器的名义任务成功；参数改变、reward上升或单次成功各自不证明可靠性、能力提升或有效学习。稳定学习与泛化本轮未验证；GPU、服务器和硬件仍为NOT_RUN。预算耗尽或策略失败不自动追加训练／seed／调参，完成后停止。

实际命令、代码状态、依赖版本、PASS／FAIL／NOT_RUN／BLOCKED与必要负结果只更新对应任务的当前活动计划。不另建状态、handoff或清理报告。保留正式数据和唯一失败证据，清理本任务无用临时产物；不按年龄清空其他研究成果。

## 9. 分离后的来源与兼容

Docker白名单包含两个包、六份配置和同一锁；不访问父仓库、不初始化submodule。
从发布分支的干净独立clone构建。构建记录包含每个输入文件哈希、Git SHA/dirty；
运行provenance分别记录RL与实际导入Physics模块内容、Physics发行元数据、锁、
配置和镜像ID。没有Git时身份为UNKNOWN而不是虚构干净提交。

加载旧checkpoint按逻辑配置项比对内容哈希，不依赖旧绝对路径；继续校验模型、
优化器、归一化、计数与确定性探针。旧checkpoint不重写。评价统计冻结且输出
使用新标识。配置JSON保持原文，profile中的历史来源说明不是所需外部资产。

S1/S2保留全部旧工作区、历史分支、镜像及checkpoint。回退使用这些已有入口；
新分支修正通过普通提交或revert。原RM27适配、默认分支切换另行批准。

## 10. I4：State-Based Agile Terminal Arrival（I4 ADOPTED）

I4当前活动入口为仓库内 `.agents/plans/plan_agile_terminal_arrival_01.md`。
该计划是被Git忽略、不随Git分发的本地文件，不是理解正式规范或设计的必需文件。
它承接旧contact_rl_01 §12的任务范围和状态；任务参数、公式与设计仍以本文及SPEC为权威。
旧计划只保留历史；转接本身未授予实施权限，后续用户的明确实施请求授权本批范围。

本节保留此前收敛的I4设计及附件核对后的几何与步进决策。研究方向与3 m/s名义目标已由用户确认；其余数值与实验为本批批准的冻结首轮研究设定，不代表实机能力或学习有效性。Python包名、现有Physics、PPO和单一Docker保留；不重新初始化项目，也不将旧接触奖励对照作为本轮任务。

### 10.1 终端几何与速度语义

令 `c=target.center_m`，`n=unit(target.normal)`。当前 `model.py::build_model` 将 target body 原点置于 c，并将厚度 h 的 armor geom 中心置于局部 z=-h/2。因此 **c 是前表面中心**：geom 中心为 `c-(h/2)n`，后表面中心为 `c-hn`；当前 h=0.006 m，armor 占局部 z∈[-0.006,0]。

令 p、v 为质心世界位置和速度，R 为 body→world，ω 为 body 角速度，`r_b=[0,0,-0.042] m`，bumper 半径 `ρ=0.009 m`。位置单位 m，速度 m/s，角速度 rad/s：

```text
bumper 中心：                b = p + R r_b
bumper 中心速度：            v_b = v + (Rω) × (R r_b)
朝板几何表面点：             q = b - ρ n
到装甲前表面的法向间隙：     gap = nᵀ(q-c)
该位置瞬时材料速度：         v_material_at_q = v + (Rω) × (R r_b - ρ n)
```

q 是每个时刻选出的朝板几何点，不是固定在机体上的同一材料点。当前固定 n 下，几何轨迹导数 `dq/dt=v_b`，它不等于 `v_material_at_q`。无碰撞时 q 不命名为已发生的接触点；I4 公式不替换旧 ContactEvaluator 的真实碰撞点运动学。

候选 `gap=0.20 m`，因此 `b*=c+(0.20+ρ)n`、`p*=b*−R*r_b`，不再次增加 h/2。沿用 `R*=frame(n)` 的完整姿态目标，当前目标面内转角为默认 0；不降为仅推力轴对齐。名义 `c=[0,0,1.0758] m`、`n≈[0.4617148,0,0.8870284]`，对应 `p*≈[0.1158904,0,1.2984441] m`、`v*=−3n≈[−1.3851444,0,−2.6610853] m/s`、`ω*=0`。

记录字段区分 `bumper_center`、`facing_surface_point`、`bumper_center_velocity` 和 `material_velocity_at_facing_point`。gap 只描述 bumper 对前表面的几何关系，不替代完整机体对 armor/rim/ground 的碰撞检查，不证明后续碰撞可恢复。

### 10.2 初态、终端集合与安全

首轮只随机化初始位置。令 t₁、t₂ 为目标坐标系切向轴，`p₀=p*+Ln+η₁t₁+η₂t₂`，`L∼U[1.15,1.25] m`，`η₁,η₂∼U[−0.025,0.025] m`。名义 L=1.20、η=0，对应 `p₀≈[0.6699482,0,2.3628783] m`。初始 `v=ω=0`、`R=I`；四个 `data.act` 和 `data.ctrl` 均为 `mg/4=0.367875 N`，上一物理 CTBR 为 `[1.4715,0,0,0]`，是悬停合成初始化。各环境随机源和动态状态独立，不改变动力学或目标几何。

同一实际 1 ms 物理终点全部满足时记录最早 `arrival_time_s`：

| 条件 | 候选范围 |
|---|---|
| 朝板球面点法向间隙 | `nᵀ(q-c)=0.20±0.02 m` |
| 该点相对前表面中心的切向偏差 | `||(I−nnᵀ)(q−c)||≤0.015 m` |
| 质心速度及该点瞬时材料速度 | 各自向板法向速度 `3±0.3 m/s`，切向速度范数≤0.3 m/s |
| 完整姿态 | `acos(clip((trace(R*ᵀR)−1)/2,−1,1))≤10°`，包括偏航 |
| body 角速度 | `||ω||≤1 rad/s` |
| 安全 | 本回合无已锁存违规或有效外部机体碰撞 |

用户2026-10-19改用中等放宽集合（`configs/agile_flight/terminal_arrival_moderate_precision.json`）：法向速度以中心
2.25±0.75 m/s 表示 1.5—3.0 m/s，gap ±0.04 m，切向位置 ≤0.04 m，切向速度 ≤0.5 m/s，姿态 ≤15°，‖ω‖ ≤3 rad/s；评价器判定代码不变，
只换配置。v*、奖励速度误差与参考轨迹终端速度随中心取 2.25 m/s。上表为原 3 m/s 集合（`terminal_arrival.json`），保留为历史配置。
对新集合的保守全机净空（22 个 UAV geom 包围球、gap 最小 0.16 m、切向 0.04 m、姿态 15° 旋转界）：到装甲无限前表面 ≥0.1166 m，
到地面 ≥1.1254 m（同方法复算原集合为 0.1421／1.1601 m，与此前记录一致）。T=1.0 s 平坦参考开环在 9 初态全部到达。

不要求非零速度终端状态在固定位置保持；不跨时刻拼接位置、速度、姿态最优值。失败优先于同子步成功，成功优先于无违规的任务到期。任务期限 H=3 s，未到达使用有限时域 `terminated`；正常 `truncated=False`。外部学习墙钟停止仅记录训练未完成，不伪造物理任务成功、失败或 TimeLimit。

候选全程安全边界：质心 box `[−0.6,−0.5,0.5]` 至 `[1.4,0.5,3.0] m`；任一有效外部碰撞、非有限状态、MuJoCo warning、速度>6 m/s、相对竖直倾角>80°、body rate 范数>20 rad/s、连续 proxy 饱和>0.4 s 均失败。未成功而 gap<0.12 m 为 `TERMINAL_OVERSHOOT`。实体板、rim、地面和完整机体碰撞保留，虚拟终端不新增碰撞体；这些只属于 I4，不修改原恢复体积或接触阈值。

已做的解析判断仅说明终端倾角约 27.50°，竖直力平衡量 `mg/n_z≈1.659 N` 低于总推力上限；该倾斜姿态同时产生水平加速度，并非静态悬停。此前解析轨迹检查不包含 MuJoCo、rate proxy 或电机动态，不能作为完整执行链可达性通过。

### 10.3 独立环境与共享物理区间推进

首选新增轻量 `src/rm27_contact_rl/arrival_env.py::AgileTerminalArrivalEnv`；复用 RM27Plant、动作包装和现有配置入口。新增 `../rm_uav_mujoco/packages/physics/src/rm27_contact_physics/arrival.py::ArrivalEvaluator` 拥有独立到达评分；训练奖励仍归 RL，不复制 Contact 评分。原 ContactRecoveryEnv 的默认行为保留，不在其结果结算后补改 done 或 result。

分离基线的唯一生产步进原位于`evaluation.py::ContactEvaluator.advance`。I4已在现有
`model.py`的`truth_estimate`附近局部提取共享接口，Contact和Arrival均实际调用：

```python
advance_physics_interval(
    model, data, before, *, consume_interval, saturated=False
) -> VehicleEstimate
```

函数只执行：一次 `mj_step(model,data)` → 一次 `consume_interval(model,data,before.timestamp,saturated)` → `mj_forward(model,data)` → `truth_estimate(model,data)` 返回终点状态。区间消费者必须在 forward 覆盖求解数据前读取；实际 `dt=data.time−before.timestamp`。函数不计算任务成功、不发奖励、不调控制器，不建立 TaskManager、注册系统或新调度框架。

`ContactEvaluator.advance` 保留原签名、终止/时间校验与返回行为，传入现有 `integrate_interval` 后仍调用现有 `snapshot(state,before)`。区间积分、快照峰值、同时首触分类、短触、不可逆失败、hold 中断清零、`recovery_time_s`、公开字段和 `completed_interval_implicitfast_v1` 均保持。

`ArrivalEvaluator.advance` 调用同一函数；区间消费者只结算真实时间、饱和持续时间和必要安全数据，终点消费者计算新任务误差及到达条件。无接触要求检查求解阶段和刷新后的终点有效外部碰撞，任一违规锁存；有效接触沿用 `dist<=0 and efc_address>=0`，不能因终点已分离而清除违规。区间来源和终点来源分开标记：求解阶段只定位到本区间，不将更新后的 data.time 当作精确步内碰撞时刻。

每个环境只使用所属任务的一个 evaluator，不让两个 evaluator 各自推进同一 MjData。`ContactRecoveryEnv.step` 继续调用原 ContactEvaluator.advance；新 Arrival env 调用 ArrivalEvaluator.advance。`RM27Plant.apply_ctbr`、rate proxy、混控和电机动态保持；策略边界更新一次电机命令并保持，正常五子步，任一子步终止立即锁存并停止余下推进。重复查询和刷新不累计时间、积分、奖励或事件，返回结果都对应同一停止时刻。

### 10.4 Observation、Action 与奖励

候选 task_id 为 `agile_terminal_arrival_v1`，观测 schema 为 `arrival_world_v1`。保留 26 维 float32 顺序，只将前三维明确为质心到固定终端质心的世界相对位置 `p*−p`，原 Contact 的前三维仍为装甲目标 `c−p`：

| 字段 | 维度／坐标／单位 | 更新时间与来源 |
|---|---|---|
| `p*−p` | 3／world／m | reset 或实际 step 终点；状态真值与固定任务几何，未来需自身/目标定位 |
| n | 3／world／无量纲 | 固定目标先验，未来由目标姿态估计提供 |
| v | 3／world／m/s | 实际终点真值，未来需状态估计 |
| R 按行展开 | 9／body→world／无量纲 | 实际终点真值，未来需姿态估计 |
| ω | 3／body FLU／rad/s | 实际终点真值，未来需惯性/姿态估计 |
| 上一实际 proxy 输入 | 4／CTBR／N、rad/s | 指令经边界裁剪且实际送入 proxy 后更新；reset 为合成悬停 |
| 剩余时间 | 1／s | 由 H 与实际 episode 时间差计算 |

v*、R* 与 ω* 固定为本任务已声明先验，不跨回合改变隐藏目标。Actor 不读取 evaluator phase、碰撞、精确电机激活、Teacher 或未来状态；VecNormalize 仅归一化观测。状态真值训练与未来估计能力分开报告，隐藏电机滞后状态意味着输入未必严格 Markov。

物理动作仍为 collective 0—4.4145 N、body FLU rates 各±20 rad/s；归一化四维动作沿原仿射映射。归一化 0 对应 2.20725 N，悬停对应 `[-1/3,0,0,0]`。第二轮可由训练配置选择 `hover_centered_v1`，见§10.7；未声明时保持 `bounds_affine_v1` 原映射。1 ms 物理、5 ms 控制和 40 ms filterexact 电机响应保持；动作不是直接设置刚体位姿/速度，proxy 不等于 PX4。

保留上一版唯一候选奖励。令 `d=||b−b*||`、`Φ(d)=1/(1+(d/0.5m)²)`、`g(d)=exp(−(d/0.35m)²)`、`h(x;σ)=min((x/σ)²,1)`，`e_v=max(||v−v*||,||v_material_at_q−v*||)`、θ 为完整 SO(3) 角误差。每个真实完成物理区间结算：

```text
r_process = 2[Φ(d_next)−Φ(d_before)] − dt C(next)
C = 0.5
  + g(d)[0.5 h(e_v;1m/s) + 0.5 h(θ;30°)]
  + 0.25 h(||ω||;5rad/s) + 0.25 I_proxy_saturated
```

连续代价上界为 2 reward/s，按实际 dt；进度差分不再次乘 dt，也不裁掉倒退负项。成功事件一次 +20；安全失败或期限失败一次 `−20−2 max(0,H−t)`。剩余时间代价只是 reward，不能增加虚构物理区间。正常策略步求和五个真实区间，终止短步只求和实际完成区间；Contact 原事件奖励保持原位。

距离尺度提供接近信号，近端门控避免全程强迫终端大倾角；时间代价、有限进度收益和失败剩余时间项减少停留刷分与提前失败逃避代价的激励。权重是冻结的初次工程提案，并未验证有效；折扣可能改变循环/拖延收益，不宣称保持最优策略不变。奖励读取当前/前一状态真值用于训练，不因此扩大 Actor 输入。

### 10.5 PPO、评价与完成边界

复用唯一 SB3 训练器。将 task/schema、seed、环境数量、预算与协议合法性配置化；保留旧 I3 配置和默认续训验收，I4 正式训练不每次重演 4096→6144 工程检查。旧 checkpoint 缺少任务元数据时只走旧 Contact 兼容路径，不能仅凭26维加载新任务；新增 checkpoint 校验 task/schema、配置和实际来源。

首轮 seed=0，4个独立 DummyVecEnv，CPU Torch单线程，MLP两层各64；n_steps=512、batch=256、5 epochs、lr=3e-4、γ=0.999、GAE λ=0.95，VecNormalize仅观测，其余沿锁定版本默认值导出。每完整 rollout 为2048条 transition，采样段每环境2.56 s；γ在5 ms下折扣时间尺度约5 s，GAE组合尺度约0.097 s，均为继承的候选设置，不是论文参数。

总上限 **524,288 environment transitions（256个完整rollout/update）或15分钟学习墙钟，先到停止**。墙钟包括 checkpoint 与阶段处理，不保证学习收敛；中途被墙钟截断的采样单独计数，不冒充完成更新。初始、最近有效和最终 checkpoint 成套持久化。保留锁、基础镜像和一个 Docker，获批实施时仅将实际新增文件纳入现有白名单，不生成第二套容器或依赖。

独立评价固定9个不同初态：名义中心以及 L/η₁/η₂ 区间的8个角点。初始策略、最终重新加载策略、传统同任务对照各9回合，最多27回合；每回合沿3 s任务期限。评价确定性动作，环境独立、归一化冻结；前后各用对应统计只能比较完整训练流程。该集合用于局部邻域比较，不属于动力变化或分布外鲁棒性证明，不用于反复调参。

传统对照复用现有 Septic、Skill.track 与电机→CTBR project，固定1.2 s名义边界参考：起点悬停，终点 p*、v*、`a*=g/n_z·n−g·e_z`、jerk=0；到参考时域后保持连续终端外推，不切回悬停。使用同一 Arrival env、CTBR链和成功定义，不调用接触任务 FSM，不作为 PPO 标签；不搜索时域或增益。适配后是否到达是未知结果，旧4.003 s接触恢复不作为到达时间比较基准。

记录成功/失败及全部分母、到达时间、同刻位置/速度/姿态/rate误差、轨迹长度、plane gap、动作裁剪、proxy/电机命令饱和及失败分类。plane gap 不命名为完整机体最小净空；最小净空字段须声明覆盖的具体几何，不用质心距离代替。按 rollout 汇总训练回合事件与分项奖励，不保存全部训练轨迹或建立新日志框架。

分别验收工程链路、行为变化、终端到达、敏捷性与鲁棒性。候选敏捷性观察目标为9/9评价到达且各≤1.5 s，属于固定条件下研究目标，不是可靠性证明；物理/动力变化、真实状态估计及真机均未验证。预算内失败时保留必要结果并停止，不自动增加预算、seed、初态搜索、奖励权重或放宽门槛。

### 10.6 实现位置与最少检查

新增任务配置为`configs/agile_flight/{terminal_arrival,ppo_terminal_arrival}.json`，引用原150 g输入，不改六份旧配置。环境与评分落点见10.3；`rm27_adapter.py`只增加任务校验与显式初态输入，无参数reset的旧行为保持。`ppo.py`、`__main__.py`、`__init__.py`接入任务选择和同一训练/评价；`baselines.py`接入固定同任务参考，`provenance.py`记录新模块、配置和schema；`.dockerignore`、`docker/run.sh`扩展原入口，Dockerfile和哈希锁不变。

现有CLI实际使用：`train --config configs/agile_flight/ppo_terminal_arrival.json --output <新运行目录>`；
运行内按顺序完成初始9回合、预算内训练、最终9回合和传统9回合。
`evaluate --checkpoint <目录> --output <新评价目录> --task agile_terminal_arrival_v1`
只评价该任务的既定9个初态；`arrival-baseline --config <任务配置> --output <新目录>`
用于独立传统评价，首轮训练流程已经执行该组时不重复运行。
容器复用`docker/run.sh train ... --config configs/agile_flight/ppo_terminal_arrival.json`，
依赖仍位于`/opt/venv`，代码在`/workspace`、结果独立挂载。

I4资源可配置并有上限校验：CPU单线程、最多4环境、每环境rollout最多512、
完整rollout整除batch，seed为合法整数；旧I3协议仍按原固定seed/规模和续训验收。
本批实际配置仍为§10.5冻结值，不因此授权另跑seed或调参。
原始PPO归一化动作裁剪、环境物理输入裁剪、proxy饱和和电机命令饱和分别命名。

新任务行为放根 `tests/test_agile_terminal_arrival.py`，PPO接入扩展原测试，不按轮次造脚本。覆盖：前/中/后表面关系，bumper几何点与瞬时材料速度，所有终端条件同刻满足，安全优先、短步终止、一次区间/奖励及刷新幂等，reset隔离，Actor权限、CTBR执行、配置实际消费、task/schema拒绝及评价统计冻结。共享提取的旧行为复用现有 `test_interval_force_uses_completed_time_and_refresh_is_idempotent` 与 `test_hold_resets_and_recovery_time_marks_stable_start`，必要断言走真实 advance 路径，不复制评分算法。不默认全仓回归、迁移重验、PPO多seed、GPU benchmark或服务器操作。

I4接口、共享步进及上述检查已落地；本批完成一次CPU容器构建和有界训练/固定集合评价。
实际工程、任务失败、分项诊断和几何核对证据维护于本任务活动计划，不在此复制运行日志。
实现通过检查并不代表学会终端到达；3 m/s联合状态集合的完整执行链可达性和学习机制
仍须研究证据。静态终端容差净空不代替完整进场或接触恢复安全。
本批已停止，不自动追加训练、换seed、放宽阈值、调控制器或操作服务器。

### 10.7 第二轮：动作层初始分布（ADOPTED 配置项；效果按结果）

用户2026-10-17批准一次单因素有界运行。训练配置 `action_parameterization` 默认
`bounds_affine_v1`（原表达式，I3 与首轮配置、checkpoint 不变）；`hover_centered_v1` 仅限 Arrival：
归一化 0 为合成悬停 `[mg,0,0,0]`，逐维半幅 `max(high−hover, hover−low)`，即 `[2.943 N,20,20,20]`；
低于 0 N 的推力请求交给环境原输入裁剪并计数，不在 wrapper 内隐藏。`policy_kwargs.log_std_init`
校验于 [−3,0]，Contact 仅允许默认 0。配置与 checkpoint 一同保存，评价按 checkpoint 配置重建同一映射。

每 rollout 另记录逐维归一化裁剪计数，以及 rollout 观测上行为策略高斯均值和 `|μ|>1` 比例；
这些只用于归因，不进入奖励或观测。配置为
`configs/agile_flight/ppo_terminal_arrival_hover_centered.json`，与首轮配置只差上述映射和
`log_std_init=−1`；奖励、初态、终端集合、安全、评分、其他 PPO 超参、seed 与预算不变。

离线微分平坦见证（不属于训练或评分实现）表明名义初态在当前执行链约束下可进入终端集合，
开环前馈回放于 0.782 s 到达；它只证明名义可达，不提供 PPO 标签。实际运行结果与预先登记的预期
维护于活动计划；该配置项的采用不代表到达学习成功。

### 10.8 第三轮：Informed Reset／Reverse Curriculum（ADOPTED 配置项；效果按结果）

只读诊断（活动计划 §11）显示：奖励使成功比任何失败高约 43（折扣后约 30），但在训练噪声下即使均值完美，
离到达 ≥0.2 s 起步的成功率已≈0，从起点为 0；平坦开环前馈 T=1.0 在 9 初态均成功。故第三轮只改变训练起始状态分布。

- 参考：`baselines.py::FlatnessReference`。从当前 reset 状态到声明终端状态的 9 次平坦多项式（终端推力与传统参考
  相同、终端 jerk/snap 为 0），经 profile 电机滞后与名义 rate 增益求逆得到 CTBR。只用于生成起始状态。
- `ppo.py::InformedReset`／`InformedResetCurriculum`：只挂在训练向量（`make_vec_env(training=True)` 且配置声明
  `informed_reset`）。无选项的 reset 以概率 p 抽 lead∈[min_lead, frontier]，在真实 `step` 中按参考飞行
  `duration−lead` 秒（电机状态、上一动作、评价器历史与时钟自洽），然后 `start_scored_segment()` 清零奖励累计；
  前缀转移不返回学习器。前缀内若终止则改为完整起点并计数。带显式初态的 reset（评价）不受影响。
- 前沿：每个完整 rollout 末检查一次；最近 W 个 informed 回合成功率 ≥ 阈值时 frontier 增加一步，直到参考时长
  （此时等同完整起点）。每 rollout 记录 frontier、窗口成功率、informed／完整起点各自的完成与成功数。
- 配置 `configs/agile_flight/ppo_terminal_arrival_informed_reset.json`：在第二轮配置上只增加
  `informed_reset`（参考时长 1.0 s、p=0.5、初始 lead 0.1 s、最小 0.05 s、步长 0.1 s、阈值 0.5、窗口 32）。
- 评价、奖励、终端集合、安全、评分、动作参数化、PPO 超参、seed 与预算不变；结果维护于活动计划。
- 前缀复用（工程项，可选 `reference_reuse_episodes`=K，缺省 1 即第三轮行为）：K>1 时一次参考飞行到最长前缀，
  每 10 个策略步保存 `AgileTerminalArrivalEnv.episode_snapshot()`（完整 `MjData` 拷贝＋环境／评价器累计量），供本环境
  后续 K 个 informed 回合使用；恢复最后一个早于所需起点的快照，再用与新鲜前缀相同的飞行循环补足。所得状态与从相同
  初态新鲜飞行逐位一致（测试覆盖 MjData 主要字段、观测、累计量、评价器摘要及后续步）。代价是初态坐标在 K 回合内重复，
  边际分布不变；随机数消耗顺序与 K=1 相同。
- 前沿推进统计（可选 `advance_statistic`）：缺省 `all_informed_v1` 统计 [min_lead, frontier] 内全部 informed 回合
  （第三轮及批次 A/B）；`frontier_band_v1` 只统计 lead 位于最新一档 [frontier−step, frontier] 的回合，避免已掌握的短 lead
  撑过门槛。采样分布不变。

### 10.9 末端精度奖励塑形（ADOPTED 可选项；效果按结果）

用户2026-10-18批准一次单因素运行。`ArrivalEvaluator.terminal_violation_ratio` 给出 8 个终端条件误差／容差的最大值
（≤1 恰为同刻全部满足）；评价器在尚无安全违规且未到达的每个 1 ms 终点快照更新最小值，摘要字段
`best_terminal_violation_ratio`。到达判定 `qualifies` 与终止逻辑不变。

奖励可选 `terminal_precision_bonus`=B（缺省 0，原奖励逐位不变）：回合结束时 event 项一次性加 `B·min(1, 1/ρ_best)`，结果单列
`terminal_precision_bonus`，`reward_status` 标为塑形版本。B=15 由已有审计轨迹解析得出（使折扣后传统近失排在原地拖到超时之上的
最小整数档，需 B>12.3），成功仍比任何失败高 ≥40。配置为 `configs/agile_flight/terminal_arrival_precision.json`（原
`terminal_arrival.json` 不改）与 `ppo_terminal_arrival_informed_band_precision.json`（与运行 C 只差 `env_config`）。

### 10.10 长预算运行（2026-10-19）

用户要求在运行 E 后继续改进。Arrival 训练预算上限由 524,288 transitions／900 s 提为 `ARRIVAL_MAX_TRANSITIONS=2,097,152`／
`ARRIVAL_MAX_LEARNING_WALL_S=3600`（单 seed、本机 CPU；I3 原上限不变）。课程前沿每次推进时另存
`checkpoints/frontier_<lead>s/`，供离线 9 初态评价；不影响学习。运行 F 配置 `ppo_terminal_arrival_moderate_precision_long.json`
与运行 E 只差预算。

### 10.11 训练用动力学随机化（2026-10-19，ADOPTED 可选项）

运行 F 的只读稳健性评价显示，推重比下降（质量 +10%／+20%、推力增益 −10%）时完整起点成功率降到 5/9、1/9、5/9，失败为过冲；
其他单项偏差（质量 −10%、推力 +10%、电机时间常数 30／60 ms、惯量 ×0.8／×1.25、小观测噪声）均 9/9。
`AgileTerminalArrivalEnv.set_dynamics_scales(mass, thrust)` 按编译值缩放仿真机体质量与电机推力增益；rate proxy、混控、悬停命令和
参考轨迹保持名义 profile。缩放属于回合快照（恢复时重新施加），并写入结果 `dynamics_scales`。`ppo.py::DynamicsRandomization`
只挂在训练向量上，每回合从配置声明的均匀区间抽取；评价向量始终为名义机体。配置 `ppo_terminal_arrival_moderate_precision_long_dr.json`
= 运行 F 配置 + `dynamics_randomization={mass_scale:[0.9,1.15], thrust_scale:[0.85,1.15]}`。
与 informed reset 组合时，参考前缀在名义机体上飞（只用于生成参考上的起点），在交给学习器的时刻施加本回合抽到的缩放；
开环前缀若在随机化机体上飞会偏离参考（首次实现的运行 G：informed 成功 11%、前沿只到 0.4 s）。课程前沿累加以 9 位小数取整。

### 10.13 Weights & Biases 追踪（2026-10-19，用户要求；可选）

- 依赖：`pyproject.toml` 新增 extra `tracking = ["wandb==0.31.0"]`，用同一条 uv 命令（加 `--extra tracking` 与
  `--no-emit-package rm27-contact-physics`，后者沿用“第一方 Physics 不进第三方锁”的约定）以原锁为约束重新生成哈希锁：
  原 26 个版本不变，新增 wandb 及其 30 个传递依赖（含 W&B 官方的预发布客户端 `cwevals==0.0.1b1`，已核对 PyPI 来源与证明）。
  依赖变化需重建镜像；旧镜像保留。
- `rm27_contact_rl/tracking.py`：把每次更新记录（`updates.jsonl` 的同一份内容）展平成标量，供即时记录与事后导入共用；
  只有启用时才 import wandb。训练配置不含追踪项，同一配置可带或不带追踪运行；追踪失败只写入 `summary.json` 的
  `tracking.error`，不影响实验。
- 容差课程使用独立的 `tolerance_curriculum/{scale,window_episodes,window_success_rate,advanced}` 曲线；
  空窗口不记录成功率。课程标签同时识别 informed reset 和容差课程，后者另加 `tolerance-curriculum` 标签。
  既有 informed reset 的 `curriculum/*` 字段与历史无课程配置标签保持不变；即时与事后导入共用同一投影。
- 即时：`docker/run.sh train ... --wandb` 才开启容器网络（缺省仍 `--network none`），以环境变量名传入 `WANDB_API_KEY`
  （来自私有文件 `~/.config/rm27/wandb.env`，否则主机 `wandb login` 的 `~/.netrc`），并设 `RM27_WANDB=1`；W&B 本地文件写入
  运行目录，配置／缓存在容器 `/tmp`。项目缺省 `rm27-agile-flight-rl`，按环境配置分组，按算法／课程／历史／随机化打标签。
- 事后：`python -m rm27_contact_rl track-import <运行目录>... [--robustness-dir DIR]` 回放已存证据为一个 W&B 运行（步数为
  transitions），可附加稳健性结果为摘要。



## 11. 场地目标注册与历史移动进场接管基线

SPEC §10 的新任务先按当前场地测量目标。目标注册包含可见前表面、背板与绿灯的来源，板面坐标系、
有效区域、对应碰撞 geom 及全套资产哈希。碰撞面名称仅为候选集合；有效接触仍需检验接触点位于
注册板面区域。外观网格与碰撞壳的偏差独立记录，不因碰撞壳较厚而移动视觉目标或扩张命中范围。
原生绿灯、框架和底部接触件的碰撞覆盖不能由装甲板中心检查推定。

工程验证使用完整场地与 drone_v2 的既有碰撞体，不重新生成 base。原生飞机与已有配置附加的底部球
分列验证，明确球的几何来自工程假设。先做来源/几何检查，再用多位置射线核对板面和碰撞壳，最后
从无初始接触的固定初态通过 mj_step 检查首次接触对、有限状态及接触力。夹具初始化不作为飞行轨迹；
步进中不直接写姿态或速度。测试输出保留原始失败，不能据此调奖励或自动启动训练。

以下移动交接设计只描述此前 approach 工程检查，不是当前 Stage 1 的接管条件。
非接触连续进场工程检查由独立模拟器的 `rm27_sim.experiments.approach` 提供：复用位置/SO(3)
控制器，经 CTBR 投影和既有 rate proxy 驱动真实电机动态；5 ms 控制、1 ms 物理步进。
参考轨迹先向机库外侧离场并绕过中央结构，垂直参考从外围绕行起点开始平顺下降，至最后定速段前
结束。水平参考与交接速度连续；不在板前停车或重置，不以直接写状态代替飞行。

工程交接保存 MuJoCo 完整积分状态（含时间、电机 act/ctrl）、上一实际 CTBR、控制器垂直积分和
seed；从该快照恢复后，以相同闭环参考续飞验证一致性。当前基线不进行随机抽样，seed 是保存的
实验身份，并不代表已经实现随机源状态恢复；后续引入随机源时必须保存其真实状态。
轨迹中的正常结束状态只有在安全与移动交接窗口共同通过时才可标为交接状态，失败终态不可充当
有效训练初态。这些旧移动交接状态不自动成为新任务训练初态。新 Stage 1 的独立悬停 reset 已在 §14 接入；
从机库真实连续飞行到悬停控制器、再切换至 policy 的任务交接仍需实现，不能以 reset 冒充。
新增任务身份、快照及训练/评价配置显式区分旧 I4，无接触旧任务的公开行为不变。

## 12. RL 侧解析可微分代理（E0）

`diff_dynamics.py` 的 `DynamicsParameters.from_env(env)` 从当前编译模型提取固定
参数并校验支持范围；模型与 CTBR 控制器的惯量／mixer 分开保存。参数抽取不修改
MuJoCo，不复用旧文档中估计值替代实际数组。当前质量快取不一致会显式拒绝；
不要在这个接口内调用 `mj_setConst` 或绕过拒绝来改变旧执行语义。

`step_control(state, physical_ctbr, params)` 是可批量、保留 autograd 图的纯 Torch
计算。状态顺序为世界位置3、body→world的wxyz四元数4、世界速度3、body角速3、
四电机act4，共17标量（旋转只有3自由度）；输入为N及rad/s的物理CTBR，不能直接
传SB3归一化动作。输出含最终状态、全部五个1ms端点、raw／clipped motor command
及clipped CTBR。时间、上一CTBR、观测历史、reward／terminal history仍属环境／
未来训练层，不混入物理状态或丢弃后冒充可恢复episode。

先按原界限clip CTBR，再计算 `I_c*K*(omega_cmd-omega) + omega×(I_c*omega)`，
以原 inverse mixer 分配并clip电机；五个子步中指令保持不变。各步使用旧act产生
body力矩／推力，经四元数旋转加重力；电机更新是
`act_next = act + (1-exp(-dt/tau))*(command-act)`，与actearly=false一致。
半隐式平移用新速度更新位置。MuJoCo3.12.0独立自由体的implicitfast局部求解为
`(I + dt*J_gyro) * angular_acceleration = torque - omega×(I*omega)`，
`J_gyro = skew(omega)*I - skew(I*omega)`；新角速用于四元数右乘指数更新。
零角速处用解析级数保证一阶／二阶导数有限；不删除真实限幅来制造梯度。
这些是指定离散模型的适配，不是实机参数辨识或新物理权威。

`diff_validation.py` 使用原 `ArrivalEvaluator.advance` 逐1ms驱动参考，终止后
不再推进该窗口。先重放18条归档输入，再以128固定窗口核对四跨度；候选及参考
使用同一物理初态、同一开环CTBR序列。扰动方向同时包含初态和动作，四元数扰动
投影到切空间再归一化。整个640ms链均参与autograd，只有非光滑分支单列；这不是
把逐步局部导数当长期BPTT。输出包括逐样本数值、方向、覆盖、边界与来源hash。
窗口重建评分器历史只用于物理局部核对；归档全episode另行重放，不宣称任意中途
窗口重建了原episode的累计saturation／reward记忆。阈值与预算只由SPEC及配置定义。

按用户最新分层要求，`diff_analysis.py` 只读原审计并继承已用计算预算；原结果保持
原协议语义。它重用保存的MuJoCo正负扰动轨迹核对边界两侧，另以固定PPO／SAC
机动及末端窗口补测action-only、state-only与原hover-centered归一化动作链。
归一化映射的Torch组合在归档输入上与原 `NormalizedCTBR.action` 逐值核对；不改变
原wrapper或PPO／SAC行为。完全clip方向专门核对零导数；epsilon sweep区分有限步长
误差与微小导数差分噪声。单侧导数采用扰动侧的JVP对该侧割线，报告有限步长，
不以边界中心差分或Torch选定subgradient宣称存在唯一导数。

分类使用整段前缀的实际CTBR及raw motor距限幅面的距离，1e-10相对尺度以内列边界；
严格超限与内域分别报告。到达／安全／接触事件按真实事件时间屏蔽，包含第五子步
恰好结束的情况。预算中断时保存完成探针清单及活跃探针计数，不能用未完成样本
制造覆盖PASS。补充分析的COMPLETE只表示计划核对完成，E1适用范围需独立判断。

补充产物 `sources.json` 记录两仓HEAD／dirty、实际模块路径、依赖版本与编译参数；
`sources.tar.gz` 固定相关源码／配置／依赖锁的当前内容，包含必要未提交文件，
`model.xml` 固定生成的场景。旧轨迹不复制，其原始文件及摘要hash继续核对。
float32前向用同一冻结reference单独分析；没有把float32前向一致性当float32梯度
或策略吞吐证明。原始数据与补充分析分别记录，以保留旧判定和最新解释。

上述E0接口已在后续独立E1-A接入（§13），不接入PPO/SAC默认路径。优先以短窗口
解析模型验证自主学习；世界模型／残差模型与JAX迁移留到有偏差或吞吐证据后。

## 13. E1-A：80ms actor／critic 工程链

`diff_policy.py` 独立于原SB3训练器。`arrival_observation(state17, previous_action4,
elapsed_s, env)`只构造原26字段并保留float32观测转换；`ActorCritic`使用固定物理尺度，
actor／critic为独立26→64→64的tanh网络，actor输出4维tanh动作。`physical_action`
与原NormalizedCTBR表达式一致，包括normalized clip和后续物理CTBR clip。
电机、累计饱和、事件phase和best_violation_ratio不进入任一网络。

`rollout_window(env, policy, steps=16, gamma=.995, budget=...)`从专用环境当前完整状态
建立窗口，初始状态与历史为常数。每个控制用E0模型生成五端点，再用detached动作
执行原 `env.step`。临时包裹该评分器实例的advance调用以记录实际1ms区间与precision
更新有效性，finally恢复实例方法；没有替换评分器逻辑或修改Physics来源。按实际
子步数选取解析端点，只累计已执行区间；影子MuJoCo自然保存电机和完整评分历史。
严格核对state／observation／reward，接触差异或unsupported参数直接失败。

连续reward按原公式组合，包括material-point速度、动作正则、成功精度和历史
terminal-precision项。窗口前best_ratio为detached常数，窗口内有效端点累积可微min；
安全失效端点不得刷新best_ratio，timeout发生前的合法端点仍可刷新。姿态使用与原
acos同值的atan2表达式避免identity处无限导数；pi切面只保留有限选定次梯度，不宣称
全域光滑。饱和／成功／安全／逾时等离散选择由原评分器决定，不求事件时间导数。

`WindowRollout`返回return、detached critic_target、原初／末观测、内部末状态、
实际子步、逐控制奖励、bootstrap、前向误差及饱和诊断。bootstrap只在窗口仍存活时
启用；临时冻结critic权重但保持输入梯度，finally恢复参数标志。初始已终止窗口
返回零控制／零return／零bootstrap，不能自动reset或查询actor。

独立checkpoint包含actor、critic、尺度buffer及任务／配置／来源／编译参数契约；
`save_checkpoint`只创建新文件，`load_checkpoint`严格核对后使用原predict接口。
`diff_smoke.py`使用原 `_arrival_episode/_arrival_initials/_arrival_summary`评估九初态，
只是适配policy与观测归一化，不改正式评估函数。优化器状态单独保存并逐内容核对；
加载策略的中心初态轨迹结果与原策略核对。

Smoke用固定8个fresh resets验证3次SGD actor更新、6次SGD momentum critic更新，
包括actor参数方向有限差分、同批冻结critic目标前后比较、critic拟合下降与梯度隔离。
这是工程更新验证，不是覆盖全episode的E1-B采样器或收敛试验。原3s hover逾时探针
跨多个80ms窗口，另测第2／5子步到达、第5子步安全事件、初始接触及严格饱和。
计算上限与验收归SPEC §12，参数归唯一smoke配置；不创建完整E1-B入口。

## 14. 固定表面 Stage 1 PPO 工程基础

### 14.1 研究来源及有意简化

研究原文为 PerchRL `arXiv:2606.03441v1`，重点为 §3.2、附录 A.3/A.5。
本实现采用状态预训练后转估计观测微调的路线、CTBR、密集引导与终端事件奖励的组织方式，
并保留小范围初态扰动。此处的 reward 是适配固定平台的新提案，不逐项复制论文公式、权重、容差和训练预算。

RM27 平台固定，因此不引入论文的随机移动平台、TCN 历史编码、未来平台状态特权 critic 或主动感知机制。
现有 SB3 `MlpPolicy` 的 Actor 和 critic 使用同一 26 维输入；不使用 Teacher 标签或 imitation。
Stage 2 计划使用上游 Tracking、VIO／PX4 的处理后估计，不让 policy 识别 RGB；是否需要历史网络、
失视预测或额外感知机制由后续结果决定。论文性能和附着行为不能作为本任务能力或成功定义。

### 14.2 冻结场景、命名飞机和控制链

`configs/stage1/task_contact_geometry.json` 绑定独立模拟器保存的 `artifacts/2027-base-dock-ready` 场景及来源／摘要哈希，
并沿用 Foundation 的全部数值和已编译模型。旧 `task.json` 及 `ppo_foundation.json` 原内容保留。
`stage1_plant.py` 只生成有 manifest 的私有变体：固定名义平台运动关节，附加既有 S1c 底部球假设，
保留场地、原飞机、电机动态和禁碰几何。`stage1-prepare` 显式验证来源并编译；正常环境加载只消费已验证的 MJB，
不在每次 reset 编译。私有 XML、MJB、MuJoCo 版本和输入哈希都必须一致；不修改冻结源和 Physics。

源注册表包含目标侧和对侧的候选面。私有场景的目标集合依据冻结完整场地、目标位置及抽取区域解析，
保留目标侧真实表面，记录对侧排除来源，并对同一已注册板面范围做射线核对；不能把缺失的目标侧碰撞面
静默忽略或扩大允许区域。注册的 ray-audit 足迹仍是工程范围，不是官方计分区。

后续源重建证明原私有 manifest 的17面漏掉 `visual_117` 装甲正面壳。当前任务以可选
`contact_surface_registration`／`expected_contact_surface_registration_sha256` 在物理缓存验证后加载任务自有登记，
只追加 `36286..36293` 八面；没有该字段的旧配置仍走原17面、原来源集合。新增登记及其原来源进入 checkpoint
的 `sources`，使新旧语义可区分。校验覆盖原 target／MJB SHA、未变的中心／轴／90×100mm矩形、原XML属性与世界顶点、
正面源分组及 compiled float32 mesh 身份。该核对的微米级误差限是身份检查，不是新的接触／评分容差。
所有源场地、私有XML/MJB/manifest、材质、控制与 scorer 均不修改；不重新编译，也不改既有checkpoint。

全场景含其他关节，状态读取按 `drone_1_free` 的 qpos／dof 地址，动作按 `drone_1_motor_*` 的 actuator／act 地址。
不能沿用旧单刚体 `qpos[:3]` 或 `uav` 名称假设。物理 CTBR 调用共享 `RateThrust/apply_proxy` 和既有混控；
normalized CTBR 使用现有 `NormalizedCTBR`，负集体推力请求由环境明确限幅并计入诊断／成本。
本任务不使用 E0 发现缓存不一致的运行时质量缩放路径，不迁移 JAX／GPU。

### 14.3 独立观测、reset、物理事件与 reward

`Stage1ContactEnv` 的 task/schema 为 `fixed_surface_contact_v1`／`fixed_surface_state_v1`。
26 维字段和 12 维 `initial_perturbation` 契约归 SPEC §10.1。板面框架采用 `[-slope, across, normal]`，
把源注册的轴序转换为 SO(3)，矩形绝对坐标范围不变。固定评价中心和位置／速度／旋转各轴两侧形成 19 例。
Reset 的位置、速度、旋转向量和角速度均从声明范围独立抽样，电机 act／ctrl 为悬停值，
没有参考轨迹前缀。Actor 的剩余时间直接来自物理时钟，不能依赖 evaluator 的阶段或缓存。

`Stage1Evaluator` 完全独立于 reward。每个 1 ms 区间执行 `mj_step → 读取该区间 contacts/forces → mj_forward`，
之后读取命名飞机状态；控制步骤在第一个终止区间立即停止。共享旧 interval helper 的返回状态硬编码 `uav`，
因此此处保留相同顺序的局部任务适配，不修改 Physics。接触门槛使用区间前状态计算接触点速度与底部法向，
前后边界均核对安全；同区间安全违规优先。接触记录含实际 pair、force、位置、局部坐标、前后自机／电机状态和
区间起止时间。几何接近不是成功，初始重叠也不是有效接触。球夹具绕法向 yaw 不受限，未声称完整姿态对齐。

`_running_reward` 组合有界距离 potential 的差分、近目标法向对齐、相对悬停 CTBR 幅值与相邻动作变化、
速度／角速度、饱和／输入限幅和时间代价，再加一次终端事件。权重与尺度只来自任务配置，按实际完成区间的 dt 计费；
时间代价大于近目标对齐项最大收益，防止原地等待取得正 running reward。
这是飞行动作学习的候选塑形，不是能量最优目标，也不控制独立 scorer 的接触判定。

### 14.4 PPO 接入、工程验证与回放

原 `ppo.py` 只增加显式任务分支：配置校验、环境工厂、独立评测和 checkpoint 元数据；
旧 Contact、Arrival、SAC 和 E0／E1-A 路径保留。Stage 1 checkpoint 复用 SB3 保存／加载、VecNormalize、
optimizer／counter 与确定性 probe 检查，核对独立 task/schema 及实际 scene/config 来源。
这不表示已经实现 Stage 2 改观测后的继续微调接口。

`configs/stage1/ppo_foundation.json` 固定一个环境、64 transitions、一个 PPO rollout／epoch和最多 60 s 学习墙钟。
`stage1_runner.py` 负责编排独立 19 初态评价、有界 smoke、完整 episode 记录和按已记录命令重新模拟；
`stage1_replay.py` 另做已记录状态的 viewer／静态图，不把观看轨迹当成独立重模拟。
轨迹保留完整场景 qpos/qvel、motor act/ctrl、物理／归一化动作、观测、reward、实际区间数和初始完整积分状态；
回放必须保留输入限幅语义并核对逐步时间、终态、reward 和状态。

工程入口如下；命令说明不授予新运行、重复 smoke 或正式训练预算：

```bash
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-prepare --config configs/stage1/task.json --output artifacts/agile_flight/stage1_foundation_01/scene
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-smoke --config configs/stage1/ppo_foundation.json --output artifacts/agile_flight/stage1_foundation_01/smoke
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl evaluate --checkpoint <checkpoint-dir> --task fixed_surface_contact_v1 --output <new-evaluation-dir>
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir>
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir> --view
env -u PYTHONPATH .venv/bin/python -m rm27_contact_rl stage1-replay <episode-dir> --render <new-image-dir>
```

本轮产物目录为 `artifacts/agile_flight/stage1_foundation_01/`。工程结果、实际覆盖、成本和未通过项见 §14.5，
不宣称 64-step policy 已学会飞行。
`PROPOSED_UNCONFIRMED`／`training_allowed=false` 保留在任务配置；正式训练必须先确认任务参数并另获预算授权。
Recovery 控制器／估计器重新准入和不中断物理状态的 policy 交接属于独立后续支线。

### 14.5 本批实测、失败边界与下一轮提案

2026-11-04：原论文实际内容、版本和 SHA-256 在 `paper.json`；26 项新测试 6.08s、67 项相关旧回归 11.78s。
后续新增零步 frame0 回归，并与既有短回放共2项检查通过（4.04s）；初始qpos被篡改1mm会正确判失败。
私有场景仅编译一次，88.01s，4 个自由飞行体，28 qpos／24 qvel／16 motors／29102 geoms，MuJoCo 3.12.0。
`smoke` 和 `smoke_continued` 保留两次尚未进入策略更新的工程失败；`smoke_completed/summary.json`
累计扣除先前成本，并复用已经记录的夹具，不重复消费已完成物理检查。总计 8512 次控制／42505 个实际 1ms 区间／188.77s，
低于 20000／100000／900s 上限。编译、开发测试、图像渲染另计；没有额外 PPO 更新。

唯一 PPO rollout 为 seed0、64 transitions、1 epoch（batch32，共2个minibatch optimizer steps）、0.792s；26 次有限梯度 hook，actor／critic 参数变化 L2 为
0.026635／0.033446。critic explained variance 为 -12.16，不能据此判断长期估值充分。
原保存／加载接口逐内容恢复 policy、optimizer、归一化、计数器和确定性动作。
19 例独立评测全部 `FLIGHT_VOLUME_EXIT`，0/19；冻结归一化统计，所有失败保留。
hover timeout、指定接触夹具、限幅饱和夹具、policy 中心完整 Episode 的 MuJoCo 命令重模拟均为逐值零差，
包含 motor act/ctrl、qpos/qvel、观察、reward、物理时间、子步数及终止结果。

**实体接触夹具 FAIL，正式训练未就绪**。6ms 的同一求解同时出现底部球对 `rmuc2027_surface_35364` 的注册接触
及对 `rmuc2027_surface_36293` 的未注册接触，力约 7.31N／7.08N；前后状态和全部 pair 完整保存。
后者是原冻结 CAD 碰撞凸壳，来源几何最吻合 `visual_117` 的 5mm 置中壳。须区分“未登记”与“实际非目标”：
既有 `base_target.json` 明确将浅灰 `visual_117` 记作已确认装甲、`visual_115` 为周围暗框；因此不能把 36293
断言为非目标，也不能只因共面就直接扩大合法集合。同一 90×100mm 足迹内对其逐 mesh 射线有 389/441 命中，
与 `visual_117` 深度差约 -2.5mm，与 `visual_115` 近共面（-0.136～+0.291µm）。
需由 Sim 核对已有 corrected 面和原板壳的来源归属、外表面位置及登记完整性。
原射线审计只证明最前方注册面覆盖，不能证明没有后方重叠壳或目标语义正确。
区域外 across=0.06m 的夹具开始已有重叠，真实分类为 `INITIAL_OVERLAP`，没有继续平移寻找有利结果。
新 scorer、成功语义、所有原碰撞面和有效区均未为这两项失败改变。解决该资产／注册问题属于进入正式训练前的明确工作。

### 14.6 接触几何后续核对：工程 PASS，训练仍待批准

证据为 `artifacts/agile_flight/stage1_contact_geometry_01/`。来源不是仍保留STEP装配料号的CAD树，
可机械追溯到原runtime mesh、triangle group、分片生成器与碰撞XML，不能冒称具体STEP零件名。
`35364` 为 `visual_115` partition8，加既有向内2.5mm backing，重建世界顶点Hausdorff误差9.39e-10m；
`36293` 为确认装甲 `visual_117` partition7，误差5.53e-11m。该装甲全部22片中，仅最前8片对应朝向目标的
triangle group24..363（340 triangles）；另两方向的14片未登记。原17面来自corrected代理，未覆盖该原装甲壳，
这才是同一合法区域的同时接触被误拒绝的原因；不能因共面把整个visual115或visual117都放行。
八片完整重建最大差1.072µm来自原生成器10µm共用mesh网格，原XML与compiled身份检查通过；不涉及scorer放宽。

中心夹具原非重叠初态、gap1mm、vn0.2m/s，在6ms底部球同时碰35364/36293，压缩力7.311/7.077N，
两pair均被接受。旧配置相同状态／命令仍原判FAIL；qpos/qvel、motor act/ctrl、姿态、速度、时间及CTBR逐值0差，
改变仅为登记语义。原区域外1mm gap小于当地碰撞壳相对名义面的约2.22mm凸起，导致底部球预先穿入1.20/1.22mm，
不是已证明的其他机体结构重叠。保留across60mm，改gap30mm／vn0.5m/s，初始无活动飞机接触，61ms首次碰35369/35370，
局部across59.455/60.000mm，唯一失败 `CONTACT_OUTSIDE_REGION`。倒置错误部件夹具由非重叠起点79ms碰到core_board_29，
保留 `UNDESIGNATED_CONTACT` 及姿态／速度／区域等全部失败。原重叠夹具仍零步拒绝。
另外用小型真实MuJoCo fixture覆盖同geom背／侧面及同区间合法+非法pair，不替代全场资产来源证据。

5例完整记录及重模拟通过，物理主检查66controls／304个1ms区间／10.675s；前置夹具定位146区间／2.766s，
最終来源正例重模拟另2controls／6区间／1.479s，主工程累计456区间，低于4096controls／20000区间／300s上限。
针对性17tests通过、22 deselected、3.20s；来源重建、pytest、只读载入另计，无全场编译／PPO更新。
来源与最终载入防错补充的版本分别记录，不重写较早的证据。历史checkpoint只读载入仍原17面且内置状态／动作probe通过。
接触工程已可验收；Stage1自主飞行动作学习尚未执行，旧0/19不改判。前述seed0预算仍为提案；当前评测入口尚未传
共同预算，批准后须最小接入600s／30400controls／152000区间硬限，才可启动，不能声称此上限已经实施。
本轮保留双壳同时解算；所报压缩力是该未标定模型的模拟输出，不是真实硬件接触力测量，也不据此推导整组接触门槛。

此前 seed0 pilot 提案（现已由用户批准为单次simulation-only baseline，见§14.7）：单 CPU 线程、单环境，n_steps=512、batch_size=128、n_epochs=4，
最多 131072 transitions 或 1800s 学习，先到即停；使用同一 26 维／CTBR／无 DR 初态契约。
前后各一次完整 19 初态评价，独立上限 30400 controls／152000 区间／600s，总墙钟上限 2400s。
按当前短更新约 81 transitions/s 作部分成本估算，长运行含 checkpoint 等开销，可能先到墙钟而未满 transition。
保存最后一个完整有效更新；非有限值、MuJoCo warning、来源或观测／评分契约变化立即停止。不能凭 reward 上升或单例成功提前宣称学会。
评价记录完整成功率、失败类型、接触前相对速度／法向误差／body rate、到达时间、动作变化和饱和／裁切占比，
并保留完整 Episode。物理注册问题、接触件尺寸／材质、有效区、速度／姿态／穿入容差、悬停范围和4s时限须先确认。

### 14.7 Stage 1-A 单次seed0学习接入

当前批准配置为`task_stage1a_seed0.json`／`ppo_stage1a_seed0.json`，原参数已被用户接受为本轮仿真假设；没有实机标定声明。
复用原`ppo.train`／SB3更新／checkpoint，不建立另一个trainer。Stage1-A分支加独立训练与评测预算：
`SmokeBudget.activity`只累计活动阶段墙钟，评测时钟在学习期间暂停；`BudgetedTrainingEnv`在每次实际控制步前检查限额，
并记录实际1ms区间与不完整rollout消耗。原gradient hook在optimizer阶段检查学习deadline，控制步和梯度操作为检查边界；
不可再启动超时后的动作或更新。文件落盘／异常收尾属于不可拆分操作，若跨越边界须在实测墙钟中保留，不能删去成本。
初始前评测完成后才启动学习时钟；学习期的记录、保存及中点快照扣学习预算。后评测及早期／中期名义初态行为试验
共享剩余评测预算，不能把它们当免费额外rollout。参数、reward、scorer、原场地及Physics没有改变。

checkpoint初始0、early首个完整update、midpoint首次完成半transition或半学习墙钟额度时、final最后完整有效update。
正式前后各19例不缩减；`record_episode`遇预算耗尽保存`INCOMPLETE_BUDGET`与已执行完整状态，不能当作物理终止或完整回放。
每个训练episode的真实terminal/contact记录落盘；首个成功及最多12类代表失败保存全部状态／命令，可用既有MuJoCo重模拟。
这些随机轨迹可跨越rollout更新，不宣称来自单一checkpoint。确定性checkpoint回放另记录未裁切的Actor均值，
同时保留实际送入CTBR的裁切后动作，便于分辨Actor裁切和rate proxy饱和。
所有新增预算、部分记录、真实步进上限及代表轨迹往返检查均不执行optimizer；实际学习仅一次获批run。

已验证：seed0恰好131072步／1501.227s停止；38正式评估＋两个checkpoint行为例完成。两条完整记录命令重模拟
的状态／电机／观测／reward／终止零差，连同重模拟的共同评测成本131.082s。初始/512/65536/131072步checkpoint保存。
后评估0/19且均角速度超限；长期混控饱和与大幅旋转表明动作尚不受控，不能由优化曲线或较高reward认定学习成功。
每轴动作界限20rad/s与实际body-rate范数安全界限20rad/s是不同约束；此轮均沿用原契约，未事后修改。
原进程完整摘要已写出，但外层session退出143来源未知；完整性与可重放核对单独记录，未用重训掩盖退出状态。


### 14.8 Stage 1-A 执行链诊断与显式 Action Scale 对照

现有`proxy_saturated`就是共享`apply_proxy`的逆混控motor需求越界标志；逐motor clip到[0,maximum]同时改变collective及torque，
不能把它解释为独立的proxy torque限幅或实际data.act触顶。实际motor force经过40ms filterexact，ctrl/act必须分别记录；
本模型actearly=false、单位force gain，无另设act/force硬裁切。trajectory动作写于区间结束列，控制重建须使用前一列omega。
对保存的38条正式轨迹及四个checkpoint名义轨迹的只读重建验证映射/共享proxy/电机递推，未发现接线或尺度错误；
编译旋翼位置×推力方向＋gear独立构成的mixer完全相同。没有新MuJoCo步进或训练。

观察到final第一命令已经要求负motor；280ms独立裁切把所需负pitch torque变成正值，collective1.442→2.127N，
延迟后平均2.211N，随后实际pitch仍增速。动作域每轴±20与可执行wrench域／安全角速norm20不是同一约束。
原19例raw回报增加，但按γ=.999的完整确定性episode回报−3.40128→−4.46441；不能据raw曲线认定早死reward exploit。
该折扣回报不是PPO clipped surrogate或随机policy期望值。唯一性因果仍需独立对照。

提案只增加RL policy输出rate gain β=.1（有效每轴±2），保持env的±20 bounds和reward归一化尺度、推力映射、Physics、
scorer及门槛不变。直接修改env的body_rate_limit会连带改变_running_reward中的action尺度，不是此单因素提案。
静态hover-rest±2三轴box八角点可行，不能推广到任意collective／omega／motor状态；当时仅提出方案，后续授权实施结果如下。
先对原/候选做相同physical hover和±1rad/s pulse/release基线，再对冻结final policy做同初态≤1s闭环；
工程额度提案≤4096controls／20480个1ms区间／120s，0训练，结果保留全部失败。通过后才讨论新的有界PPO批准。
现有4s hover证据稳定，但缺此Stage1 proxy的专门±轴小rate阶跃证据，不能用其他PX4 inner-loop测试代替。
证据在`artifacts/agile_flight/stage1a_seed0_diagnosis_01/`；原实验与checkpoint保持。外层143原因仍UNKNOWN。

用户随后批准单因素控制验证。`NormalizedCTBR(..., rate_scale_beta=1.)`增加显式keyword-only诊断gain，
只缩放映射后的三个physical rate；默认1保留旧表达式及逐值结果，不写env bounds。`mapping_identity`记录parameterization/beta，
避免把相同checkpoint的反事实映射误记为原评估。`configs/stage1/action_scale_diagnostic.json`与
`tools/stage1_action_scale.py`只用于冻结策略控制诊断，不接入或启动PPO训练工厂。
单CPU/单环境共享4096controls/20480物理子步/120s硬预算，包含导入初始化，逐控制及原mj_step入口计数、SIGALRM截止。
复用原env/scorer及已有MJB，先完成相同physical命令的两映射4s hover和±轴1rad/s脉冲，再同初态冻结final各≤1s。
每1ms在原mj_step后且原mj_forward前记录实际actuator_force、motor activation/ctrl及物理状态；不修改积分器或安全检查。

OBSERVED：16窗口、3739controls/18692子步/45.740s，0训练、进程exit0；7组physical配对和原final历史重现均逐值零差。
小rate pulse无饱和，峰值1.1101rad/s、归零末端norm≤.001103rad/s；rate归零不代表恢复水平或位置。
冻结final beta1在.304s角速超限，beta.1裁切96.72%→0%、峰值角速20.00173→1.34859、yaw161.76°→4.19°，
但在.788s越界（目标第一轴.802176m > .8m），没有接触。原1s无安全失败联合检查FAIL，控制尺度假设支持和任务能力不得混为一谈。
后续匹配seed0重训只作为另行批准的学习研究建议，不因本对照改善而宣称稳定控制验收通过；全部旧参数、checkpoint和历史判定保持。
新证据`artifacts/agile_flight/stage1a_action_scale_01/index.html`，旧143来源仍UNKNOWN。


### 14.9 Stage 1-A beta 匹配训练接入

原训练工厂从config透传rate_scale_beta（缺省1）到NormalizedCTBR；checkpoint携带同一config，重载和正式评估因此保留映射。
训练可显式引用零更新initial_checkpoint。initialize_comparison仅接受相同任务/PPO/预算配置，差异限rate映射、来源引用及证据保留；
检查原文件hash、实际env来源、第一reset观测与normalization设置，加载原actor/critic/optimizer和统计后核对完整状态hash。
保持原seed0构造/reset顺序，不额外构造随机policy、不改变Torch/NumPy RNG。禁止final或已更新checkpoint作为该比较起点。
证据选项可保存每个完整更新及所有episode，末尾未结束片段独立标记INCOMPLETE_TRAINING_END，不伪造终止或丢弃失败；
学习期I/O仍消耗原预算。旧默认记录策略和历史任务算法不变。此轮为用户单独批准的beta=.1方法实验，不承诺接触学习成功。

本次实测工程更新通过、最终原19例0/19越界；低rate降低角速/混控失真，但未得到位置/姿态协调。原始初始状态精确匹配，后续normalizer随rollout分化；详细控制/回报/成本证据见活动计划§0及独立beta01结果，完成后不追加训练。

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

## 15. Independent flight probe implementation

`flight_probe.py` specializes Stage1's plant/step/safety extraction without modifying the original task. Reference-relative metrics and an independent continuous-tail scorer replace contact success; all contacts fail. A state-feedback PD baseline uses the same normalized CTBR path and is never used as PPO labels. `flight_probe_runner.py` uses the existing SB3 rollout/update/checkpoint implementation through an optional callback. Task/schema identity prevents cross-loading contact checkpoints. A paired runner owns exact common initialization and separate learning/evaluation clocks. The original trainer behavior remains the default.

Configuration is in `configs/flight_probe/`; the only A/B task difference is lambda_control. Actual per-1ms solver forces are captured before forward recomputation. Terminal episode recording detaches state before vector auto-reset; policy distributions are sampled/logged before clipping. Original safety bounds and collective clipping are deliberately shared. Evidence and recorded-state MuJoCo replay are kept under a fresh artifacts directory. Engineering baseline: all five nominal cases pass; shifts settle at1.224s;23 reset checks pass. Seven focused checks including two legacy regressions pass; no training was used for these gates.

Source review pins AirGym `bdaa61f6f1ce8df2e18ddec40762d08e16fa389c` and IsaacLab `207f319d9bcd4b70840001cbabf9af4b77686c05` (parent of the commit removing the supplied direct/quadcopter path). Read Hover/reset/observation/action/reward, PPO actor/loss/GAE/statistics and task PPO configurations. AirGym uses a separate rlPx4Controller rate controller, transforms collective from [-1,1] to[0,1], and rewards position/uprightness/spin/action smoothness. The external controller implementation is not in the AirGym tree and was not transplanted. IsaacLab applies collective/body moments directly, observes body velocity/rates/projected gravity/relative goal, and uses dt-scaled position proximity plus squared linear/angular velocity costs. Its PPO runner delegates optimization to RSL-RL; its internal optimizer is not vendored in the inspected task. No numerical equivalence with RM27 or ported controller is claimed. Retain original RM27 rate cost and the approved position/velocity/tilt ablation; do not copy upstream hover thrust, contact/reset tolerances, network, std or population size. Per-file SHA256 and exact source snapshots live in the experiment's upstream manifest.

The completed paired pilot did not learn stable flight. Reward/early-termination time consistency is the next proposed diagnosis; the unchanged collective many-to-one mapping remains a separate hypothesis. A has no clipping/zero-thrust in all final cases but still fails; B develops low collective means and earlier exits. Do not infer a proven reward exploit. That proposal is now authorized only for the separate C diagnostic below; original B remains unchanged.

Recorded-state playback resolves flight metadata to its declared original base plant, verifying all base values except the explicit task/observation/confirmation identities. This preserves contact asset registration and permits existing recordings without rewriting them. The replay adapter was fixed after the learning run; training hashes and the replay-only revision are recorded separately. Rendering performs mj_forward only, never mj_step. Original contact playback remains unchanged.

C is an explicitly selected flight_probe_failure_tail protocol, not a default B reward change. With terminal slot index k=start_completed_ms//5 and physical termination end_ms, delta=-12*((5*(k+1)-end_ms)*.001+.005*sum(.999**j,j=1..799-k)). Add delta to the existing event component while recording it separately in terminal metadata; original -5 failure event remains. Original step/scorer/Physics are unchanged. The runner loads the B initial model.zip parameters including optimizer and normalize.pkl RMS, verifies identity and initial physical/RNG hashes, and compares the first512 sampled actions/means/std/observations against saved B. Actual return is distinguished from GAE(lambda=.95) critic targets.

C evidence handling: external interruption occurred after complete update125. The original execution record is preserved incomplete; interruption.json distinguishes persisted training counts from unknown in-memory controls and exact stop time. Evaluation restores the last complete checkpoint without training and retains all23 cases. Common B/C objective returns are recomputed for both saved trajectory sets, and realized update counts125vs128 are reported as a comparison limitation.


### Working flight baseline

`flight_working.py` reuses the original plant, 1ms event checks and `FlightEvaluator`. The policy observes12 fixed-scale body-frame relative position, velocity, projected up relative to nominal, and body angular rates; no motor/contact state, full rotation-matrix constants, time or adaptive observation normalizer. `FeasibleCTBR` maps zero to nominal hover, collective to[.5mg,1.5mg], and rates to±2rad/s. Physics/proxy/mixer/motor dynamics are unchanged. Compact body observations follow the pinned IsaacLab design; positive proximity/upright/stability rewards follow the reviewed AirGym concepts with RM27-specific configurable scales. Original source pins remain above.

The new reward is bounded nonnegative during flight (including an alive floor), with fixed positive success/negative failure events. Earlier failure loses remaining positive rewards and brings the same negative event forward under discounting. This avoids the former stopped-negative-cost incentive without synthesizing physical steps. PPO uses SB3, CPU subprocess environments,64×64 ELU, small Gaussian exploration and zero initial action head; exact configuration resides in `configs/flight_working/baseline.json`. A zero head supplies nominal hover only, not learned disturbance rejection. No PD weights/actions are copied.

`flight_working_runner.py` separates learning/evaluation clocks, logs complete training episodes and sampled distribution evidence in batches, and records complete formal MuJoCo states for playback. PD remains an independent feasibility comparison. Old replay loads the explicitly recorded unchanged base plant; policy-task metadata separately identifies the new learning contract. This is an experimental candidate until strict acceptance is met.


The adopted working default is candidate2, `configs/flight_working/attitude_baseline.json`. PPO outputs collective plus small roll/pitch attitude setpoints (±.2rad) and yaw-rate correction (±.5rad/s). A fixed5/s attitude-error loop converts these into the existing CTBR, capped±2rad/s; no position/velocity PD runs inside the policy path. The actor uses bias-free Tanh layers, preserving the known zero-error hover equilibrium while learning feedback weights; no imitation. At the artificial4s limit, training removes the terminal event and bootstraps the critic; real safety/contact failures terminate immediately. The independent4s flight scorer remains unchanged and is recorded separately. Learning rate1e-4, initial std.15; remaining exact settings and the actual shared-budget allocation are in the run config.

Candidate2 first passed19/19 hover at262144 controls, then continued the same weights with50%hover/50%four-shift resets, finishing19/19+4/4 at393216. First shift checkpoint had±Y stable tails.453/.429s and remained failed; final shift mean settle2.4115s versus independentPD1.224s, max overshoot.05374m versusPD.000125m. Zero actor/mixer clips and zero-thrust in all final cases. The actual validated baseline includes the fixed attitude inner loop; do not describe it as a learned direct-body-rate controller. Old candidate1 config remains explicit reproducibility input, not the default. Checkpoint contract records fixed observation scales and bias-free actor requirements for any future separately authorized continuation.
