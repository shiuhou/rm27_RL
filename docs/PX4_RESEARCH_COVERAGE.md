# PX4 遷移研究覆蓋與實機替換邊界

2026-10-11，依 [本輪研發完成標準](PX4_MIGRATION_SCOPE.md)核對。
來源清單為 `docs/migration/source_manifest.json`：9個來源狀態、319個
整合文件記錄及1個README衝突。固定 SLAM 補入版本為
`3ea1e16086965176db68b88e1c42990f92c5913e`；尚未提交的原研究成果也
按文件保留，不只依賴分支提交。原工作區沒有套用此次遷移變更。

## 研究 → 入口 → 證據

| 研究／功能 | 當前入口及輸入 | PX4處理與證據範圍 |
| --- | --- | --- |
| micro-flight／S0 | `python -m rm27 micro-flight ... --autopilot px4`；180／150 g dynamics | PX4 HIL、Offboard、正常LAND／停槳及exact replay已有證據；原10/10為合成定位診斷，不能當真VIO精度 |
| Corridor與低速接觸 | `route-flight --autopilot px4`；`configs/px4_corridor_mission_s2.json`、審核路徑 | 保留幾何／评分；1 ms接觸、恢復及回库20/20，無GPS數值flow／range短任務20/20；不是实测MTF表现 |
| 接觸分类／步長收敛 | `rm27.autopilot.px4.contact_audit`、`contact_convergence`；`contact_probe`及`contact_metrics` | 物理研究復用，front/back/grazing及1 ms對0.5 ms已有證據；不把局部探針當完整禁區證書 |
| Whole Maneuver／Teacher | `rm27.experiments.aggressive_contact.maneuver`、`teacher*`；原參考軌跡和评分器 | 直接MuJoCo研究保留；需要飛控時用`flight_control.sitl --autopilot px4 --policy teacher`，不另造同名研究 |
| Student／Phase3蒸馏 | `distillation.phase3`、`deployment`、`px4_reference_training`；原bundle與新PX4資料 | 既有策略經PX4 rate／attitude＋物理推力適配；新的保守hybrid訓練可运行，留出9/14為策略研究未合格 |
| Phase4飛控比較 | `flight_control.sitl --autopilot {px4,ardupilot}`、`batch`；`configs/px4_research_inputs.json` | 180 g固定物理來源、同评分器与显式控制接口；Teacher留出1/14，当前配对PX4 rate3/3，其餘三組0/3，不能宣稱等效 |
| Phase5估計／guard | `flight_control.quality`、`estimator`、`synthetic`、`error_budget`及PX4 batch study | PX4遙測與delay/dropout/bias注入、時間品質／commitment guard可用；合成或truth模式會標注；完整故障可靠性另列 |
| 完整150 g單機任務 | `python -m rm27 px4-sim run`；`configs/px4_simulation.json` | PX4估計與當前圖像控制，物理真值獨立评分；统一启动使用明确FlightPolicy；验收证据见收尾报告 |
| 機庫AprilTag／下視 | `dock_slots`、`return_camera`、`visual_return`及`flight` | 四槽IDs21–24；指定槽位图像时效、对准、重获、下降与LAND已有代码；下视只用于回库 |
| 定位／VIO／主相機 | `LocalizationEstimate` v2、PX4 `odometry`、既有native仿真adapter | 来源／发布时间、坐标、尺度、协方差、epoch、失效合约保留；算法／標定／真機研究归外部rm27_slam，本輪不續做 |
| Astra | `research.astra.campaign`；`research/astra/configs/campaign.json` | 直接推力向量簡化模型、觀測引導，沒有AP飛控依賴；复用研究，不宣称PX4闭环或实机拦截通过 |
| Darts | `python -m research.Darts ...`；原CAD／簡化軌跡 | 獨立飛鏢／解析軌跡／渲染，不啟動AP；保留相機時間與素材替換接口，與無人機遷移分開 |
| SE3 radio | `research/se3_radio_starter/`原入口與測試 | Bluetooth／pygame输入到作者SE3与MuJoCo电机混控；独立直接控制研究，不是PX4任务后端 |
| ULog／DataFlash | `px4-sim log`、`rm27.telemetry.ulog`；原日志只读 | 加入`.ulg`時間、模式、參數和實際fusion诊断，原`.BIN`分析保留；解析通过和任务通过分开 |
| M3C軟件／有限logging | `.maixpy/`、`logging_session`、`logging_replay` | 舊ODOMETRY／IMU／有限ULog重組工具已整合；假傳輸和完整保存ULog字節回放／掉包拒絕已有證據，沒有自动连UART |
| G1與legacy SITL | `rm27/autopilot/baselines/`、`rm27/experiments/legacy_sitl/` | 保留冻结AP和历史脚本；当前替代入口为micro／route／px4-sim，不重写历史证据 |

機器可讀的覆蓋、來源文件和本次核對hash在
`docs/migration/px4_coverage.json`。條目中的「已迁移」指軟件入口和
控制协议；策略成功率與硬件資格另述。

## 可重現的本地输入

完整任務會重建場景，來源為 `configs/scene.json`、機庫CAD、固定
field與root組合的base target，不修改上游field。
Phase4／5保持原180 g比較來源；`configs/px4_research_inputs.json`
明确引用主工作區原scene、原bundle及經審核的asset root搬遷表。
這些本地輸入已在本機驗證，Git clone不包含大型原始資料／bundle。
复现研究需先按hash準備同樣输入；不要改成150 g後沿用原比較結論。

## 替換實機資料時需保持的接口

| 邊界 | 當前仿真假設 | 實機替換内容 |
| --- | --- | --- |
| 機體 | 150 g、120 mm外廓、原估計質量分配與慣量；180 g比較另存 | 定型後全重、COM、惯量与CAD碰撞几何；下视2–3 g尚未计入150 g模型 |
| 推力与动态 | 每电机最大1.103625 N、40 ms lag、yaw torque 0.018 m | 2S實測推力曲線、电压影响、迟滞、旋向／电机位置；真实param里的allocator坐标不当CAD尺寸 |
| 控制 | PX4 NED／FRD、Hamilton wxyz、N映射到归一化推力；rate与attitude分别发送 | 保留MAVLink语义；以标定取代仿真ForceMapping；不可直接抄仿真gains或SITL参数到实机 |
| 定位 | 当前native仿真估计与数值flow／ray range，初次註冊用PX4估计及已知坐標 | 接入rm27_slam的metric schema v2；保留源時間、publication clock、外参、质量、reset／epoch及fusion诊断 |
| 主相機 | 800×600、70°、20 Hz、工程安裝角／理想模型 | 外部研究完成後提供OS04A10镜头内参、外参、相机／IMU时差及真实数据 |
| 回库 | 640×480下视、25 mm机腹偏移、70°、80 mm AprilTag、slot1／ID21 | 标记实际尺寸／ID、安装姿态、相机标定与来源时间；未知／错误目标及旧图仍拒绝 |
| UART／IMU | 单UART115200、8N1；观察HIGHRES_IMU约50 Hz | 带宽／loss／CPU／clock实测；完整200 Hz输入约13,940 B/s，超过11,520 B/s物理上限，不能用主机性能代替M3C性能 |

配置中的 `truth_position_gain_s_inv` 等名称來自旧研究；新的causal
任务实际对PX4估计计算误差，不读取truth补偿。仿真传感器与独立
评分器可以读取真值，任务不能以真值触发阶段或修正漂移。

待研究事项包括激进接触、未完成故障矩阵、狭槽失联落位／碰撞、
正式几何证明、多机调度和定型后的可靠性。它们的失败保留原状；
本轮接口迁移没有因此宣称这些项目合格。
