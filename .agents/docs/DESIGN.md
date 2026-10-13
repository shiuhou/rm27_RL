# RM27 Contact RL — DESIGN

日期：2026-10-13。状态：**I0／I1／I2／I3 ADOPTED**，依据用户对各批次的明确实施请求；I3为有界CPU PPO工程闭环，不代表学习成功。后续扩展研究／部署仍待批准。目标、工程规则与验收仅维护于 [SPEC.md](SPEC.md)，具体运行结果维护于唯一活动计划。

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

实际命令、代码状态、依赖版本、PASS／FAIL／NOT_RUN／BLOCKED与必要负结果只更新用户现有活动计划。不另建状态、handoff或清理报告。保留正式数据和唯一失败证据，清理本任务无用临时产物；不按年龄清空其他研究成果。

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
