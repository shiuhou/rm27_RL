# RM27 Agile Flight RL — DESIGN

日期：2026-10-13。状态：**I0／I1／I2／I3 ADOPTED**，依据用户对各批次的明确实施请求；I3为有界CPU PPO工程闭环，不代表学习成功。后续扩展研究／部署仍待批准。目标、工程规则与验收仅维护于 [SPEC.md](SPEC.md)，具体运行结果维护于对应任务的唯一活动计划。

现有 §1—§9 保留 I0—I3 与 S1/S2 的实现及兼容契约。I4为新增独立任务；用户2026-10-13明确批准以新活动计划实施，§10作为首轮 **ADOPTED** 设计。接口采用、工程通过和学习效果分别判断；附件中的阅读声明和性能目标不自动成为运行事实。

## 1. 来源与组织

独立仓库从 `578b09794407edd74e72327d5a89c5811c40e2a7` 保留历史建立，审阅分支 `refactor/repository-separation`。此前I0—I3源于ec9db048的接触研究；旧工作区及证据不迁移或清理。

研究代码位于 `src/rm27_contact_rl/`；活动物理、控制和评分实现位于 `packages/physics/src/rm27_contact_physics/`。原RM27消费者接入留待S3；旧分支实现是保留的历史/过渡来源。原生 MuJoCo 为参考后端，不预建 GPU、多后端工厂或仿真服务。PerchRL 提供研究启发，不复制其质量、时钟、成功率或附着终止条件。

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

根 `pyproject.toml` 声明RL；`packages/physics/pyproject.toml` 声明独立Physics。
第三方版本仅由原 `requirements-contact-rl.lock` 固定，27个依赖版本不变。
Physics wheel可单独安装；RL当前支持完整checkout的editable安装，配置由
`rm27_contact_rl.paths.ROOT` 从安装源码定位，不依赖cwd或父工作区。
普通环境入口不加载Torch；train extra为SB3 2.9.0与官方CPU Torch 2.14.1+cpu，
限定Linux amd64/CPython3.12。Physics只需要MuJoCo/NumPy/SciPy及其传递依赖。

```bash
python3.12 -m venv .venv
uv pip sync --python .venv/bin/python --require-hashes requirements-contact-rl.lock
uv pip install --python .venv/bin/python --no-deps --no-build-isolation -e ./packages/physics -e .
# 仅构建独立Physics；输出按需生成并忽略。
.venv/bin/python -m pip wheel --no-deps --no-build-isolation packages/physics --wheel-dir artifacts/wheels
```

更新锁需独立理由；同时解析两个打包声明，并将单独安装的第一方Physics包排除在第三方哈希锁外。本次保留原锁字节及所有依赖版本。
同checkout安装本地Physics；未来原RM27用完整Git SHA和`subdirectory=packages/physics`
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

不要求非零速度终端状态在固定位置保持；不跨时刻拼接位置、速度、姿态最优值。失败优先于同子步成功，成功优先于无违规的任务到期。任务期限 H=3 s，未到达使用有限时域 `terminated`；正常 `truncated=False`。外部学习墙钟停止仅记录训练未完成，不伪造物理任务成功、失败或 TimeLimit。

候选全程安全边界：质心 box `[−0.6,−0.5,0.5]` 至 `[1.4,0.5,3.0] m`；任一有效外部碰撞、非有限状态、MuJoCo warning、速度>6 m/s、相对竖直倾角>80°、body rate 范数>20 rad/s、连续 proxy 饱和>0.4 s 均失败。未成功而 gap<0.12 m 为 `TERMINAL_OVERSHOOT`。实体板、rim、地面和完整机体碰撞保留，虚拟终端不新增碰撞体；这些只属于 I4，不修改原恢复体积或接触阈值。

已做的解析判断仅说明终端倾角约 27.50°，竖直力平衡量 `mg/n_z≈1.659 N` 低于总推力上限；该倾斜姿态同时产生水平加速度，并非静态悬停。此前解析轨迹检查不包含 MuJoCo、rate proxy 或电机动态，不能作为完整执行链可达性通过。

### 10.3 独立环境与共享物理区间推进

首选新增轻量 `src/rm27_contact_rl/arrival_env.py::AgileTerminalArrivalEnv`；复用 RM27Plant、动作包装和现有配置入口。新增 `packages/physics/src/rm27_contact_physics/arrival.py::ArrivalEvaluator` 拥有独立到达评分；训练奖励仍归 RL，不复制 Contact 评分。原 ContactRecoveryEnv 的默认行为保留，不在其结果结算后补改 done 或 result。

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

物理动作仍为 collective 0—4.4145 N、body FLU rates 各±20 rad/s；归一化四维动作沿原仿射映射。归一化 0 对应 2.20725 N，悬停对应 `[-1/3,0,0,0]`。1 ms 物理、5 ms 控制和 40 ms filterexact 电机响应保持；动作不是直接设置刚体位姿/速度，proxy 不等于 PX4。

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
