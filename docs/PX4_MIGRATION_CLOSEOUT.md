# PX4 v1.17.0 研發遷移收尾

2026-10-12。按使用者修訂的 [完成標準](PX4_MIGRATION_SCOPE.md)，
本輪控制／研究接口及完整仿真鏈路的軟件遷移已完成。
這個判定適用於原型研發，不代表原S0–S5所有嚴格可靠性門檻通過，
也不代表可以直接刷機、套參數或試飛。

工作區：`/home/shiuhou/Projects/rm27_drones_px4`，分支
`codex/px4-v1.17-migration`。主工作區未套用本輪變更。
操作入口見 [PX4_SIMULATION.md](PX4_SIMULATION.md)，研究索引與
實機替換邊界見 [PX4_RESEARCH_COVERAGE.md](PX4_RESEARCH_COVERAGE.md)。

## 本次完成的整合

`python -m rm27 px4-sim` 提供依賴檢查、場景重建、完整任務、
錄製電機輸入精確回放及只讀ULog分析。新配置為
`configs/px4_simulation.json` 和 `px4_prototype_*`；它們沿用已跑通
的控制數值，不為本次收尾反覆調整機體尺寸、增益或毫米精度。

完整任務原先靠執行時改寫控制函式，現在改成正式 `FlightPolicy`。
日常任務直接選用causal輸入、有限對準重試及遙測／range／停止電機
輸出確認。任務的姿態／導航與階段控制使用PX4估計和當前圖像；
碰撞、物理速度、非活動機體與實際落位由獨立評分器檢查。
既有感知算法不變，只把飛行runner、初始化准入及native player
作為明確接口傳入；不再全局替換飛行函式、Popen或檢測器。

場景從當前worktree的固定field、dock CAD及root target配置重建。
四個槽位有80 mm AprilTag／96 mm白底；機腹下視鏡頭專用於回庫。
新產物不繼承舊worktree場景中的失效絕對路徑。

## 本次實際檢查與全部嘗試

| 檢查 | 產物／結果 | 證據範圍 |
| --- | --- | --- |
| 依賴、PX4版本及hash | `px4-prototype-closeout-02/dependencies.json` 通過 | 固定v1.17.0及既有native安裝，沒有連接硬件 |
| 新場景、標記與原路徑 | `px4-prototype-closeout-01/scene/summary.json` 通過 | 重建150 g原型／目標／標記，靜態路徑審核；不是完整連續幾何證書 |
| 第一次入口嘗試 | `px4-prototype-closeout-01/summary.json` **失敗** | 事件記錄顯示飛行至complete及停槳，但新的物理評分返回NumPy布林值，JSON摘要寫入失敗；不補記為任務驗收成功 |
| 修正後完整入口 | `px4-prototype-closeout-02/summary.json` **通過** | headless閉環SITL；起飛→圖像識別／接近→低速接觸→恢復→AprilTag回庫→LAND→自動停槳 |
| 實際融合及停槳 | `px4-prototype-closeout-02/ulog-00/summary.json` 通過 | ULog有EKF2 EV位置與速度實際fusion；參數讀回和正常LAND／自動停槳確認 |
| 根目錄測試 | `px4-root-tests-closeout-02.log`：**459通過、5跳過** | 既有接口測試及新摘要序列化／變更輸入拒絕／缺fusion拒絕檢查 |
| 精確物理回放 | `px4-prototype-closeout-replay-01/summary.json` **通過** | 40,730個錄製電機輸入幀，所有比較狀態最大誤差0；不是新閉環飛行 |

機器可讀收尾紀錄與全部嘗試hash保存在
`artifacts/px4-migration-closeout-01/`。

本輪入口檢查包含2次嘗試，1次正式通過；修正後版本的代表性檢查為
1/1。這是整合與缺陷修正紀錄，不能當可靠性成功率。全部原始嘗試、
圖像、MAVLink、ULog、提交、差異與输入快照都保留。

通過試跑的仿真時間為162.952 s，物理接觸推斷延遲4 ms，恢復進入
門檻0.438 s後保持至确认；沒有二次或非指定碰撞。
獨立物理懸停評分的最大水平／高度誤差為3.263／18.047 mm；
物理停靠水平誤差1.190 mm。這些是MuJoCo真值評分，並非實測相機、
MTF、OS04A10或真機精度；本轮没有以这些数值继续选参数。

通過試跑的控制提交為
`8bf0d681a619f57d1bf33f98a9af70394e472811`。
PX4提交為`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，
二進制SHA-256為
`5646c151d002f988d73b533d6377685fa65ff5949b62f3cc869704075ec47c54`。

## P0–P7在修訂範圍下的判定

| 階段 | 收尾判定 | 保留的限制 |
| --- | --- | --- |
| P0 | 來源整合及12項研究覆蓋核對完成 | 大型CAD、原始日志及训练bundle是本地输入；不声称clone包含全部 |
| P1 | PX4固定版本／建置及180／150 g模型完成 | 除用户确认总重外，几何／惯量／推力／迟滞仍为工程估计 |
| P2 | HIL感测／电机闭环、坐标／时钟／停桨检查及录制回放已有证据 | 不把异步独立飞行描述成完全相同 |
| P3 | Offboard／导航／正常LAND／估计失效接口完成 | 无GPS flow／range为数值仿真；完整故障可靠性未获资格 |
| P4 | 接触、Teacher／Student、Phase4／5的PX4接口完成 | Teacher留出1/14、hybrid9/14和配对研究失败保留；激进策略性能另作研究 |
| P5 | 既有仿真感知接入完整PX4任务完成 | 新视觉／VIO算法、标定及真机研究由另一台电脑rm27_slam负责 |
| P6 | ULog／历史BIN与M3C离线软件接口完成 | 单UART带宽及真实高频IMU／设备性能尚未验证 |
| P7 | 正式入口、配置、研究索引、操作说明与证据报告完成 | 日常完整任务采用px4-sim；旧AP入口保留历史默认和显式后端选项 |

## 後續研究及硬件項

舊causal nominal／conditions／command-loss批次在暫停點已分別完成
10／6／5次；獨立重建的前兩組全部合格，第三組4次安全終止、
1次隔板碰撞。它们没有完成原先20／60次或完整故障计划，不自动
恢复。本轮不把未完成或失败摘要改为通过。
旧45/50变动工况、9任务／3终止／3未处理的回库故障集、扩展几何
证明及未完成相机保存sweep继续保留原证据范围。

机体定型后的碰撞间隙、狭槽失联落位、激进接触、完整故障矩阵、
多机调度与可靠性资格仍需后续研究。下视相机估重2–3 g尚未加入
150 g模型；实际总重、惯量、2S推力及光学标定不能由仿真通过推定。

视觉／VIO成果等待用户确认另一台电脑的
[`rm27_slam`](https://github.com/shiuhou/rm27_slam)研究就绪后再pull／merge。
本轮没有拉取该研究的新进度，也没有执行硬件连接、刷机、真实参数
写入或试飞。主机仿真表现与M3C实时资格分别判定。
