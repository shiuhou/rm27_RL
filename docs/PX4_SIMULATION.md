# PX4 v1.17.0 日常仿真入口

工作區：`/home/shiuhou/Projects/rm27_drones_px4`，分支
`codex/px4-v1.17-migration`。日常完整任務使用 `px4-sim`；此入口固定
PX4，不會連接硬件。原 `micro-flight`、`route-flight` 和研究入口仍
支援明確的 `--autopilot px4`／`--autopilot ardupilot`。它們原有的
ArduPilot 默認值保留作歷史重現；不要將其與新日常入口混用。

本輪完成標準見 [研發範圍](PX4_MIGRATION_SCOPE.md)。仿真能完成整段
鏈路，不代表機體已定型、可靠性合格或可以直接部署至真機。

## 依賴與版本

```bash
cd /home/shiuhou/Projects/rm27_drones_px4
source /home/shiuhou/venvs/mujoco/bin/activate
python -m rm27 px4-sim doctor
```

`doctor` 檢查 PX4 tag／提交、二進制 hash、Python 依賴、場地版本、
機庫 CAD 和固定仿真感知安裝。它不下載、不啟動容器或飛行。
需要安裝主機依賴時使用 `requirements-integration.txt` 和
`requirements-px4.txt`。固定 PX4 的建置入口為：

```bash
python -m rm27.autopilot.px4.build --output artifacts/px4-build-new
```

版本固定為 v1.17.0，提交
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。建置產物和子模組狀態保存
在新目錄；每次 SITL 有獨立參數、日誌和端口，只綁定本機。

本機完整圖像任務沿用既有 native OpenVINS 安裝：
`artifacts/openvins-native-evidence-build-01/installation.json`。該安裝
包含固定來源、庫、二進制和 Docker image hash；不由 `px4-sim`
重新研究或修改。新機器需要準備相同依賴並產生有效的本地 manifest，
相關建置來源見 `rm27/perception/sitl/instrument_openvins.py` 和
`tools/openvins/README.md`。缺少它時可先做
`doctor --without-perception` 及低速 PX4 控制實驗。

Git clone 不包含原始 CAD、場地大型本地素材、native 安裝及研究
訓練 bundle。`doctor` 報告實際缺項；不可把缺失輸入描述成已可重現。

## 完整任务

一次命令重建 150 g 機體、機庫、藍方目標、四個 AprilTag 和路徑審核，
再完成起飛 → 當前图像觀測／接近 → 低速接觸 → 恢復 → 返回指定
AprilTag → LAND → 自動停槳：

```bash
MUJOCO_GL=egl python -m rm27 px4-sim run \
  --output artifacts/px4-mission-new --instance 60
```

重建後的 XML 引用當前 worktree 素材，不依賴舊場景產物中的绝对路径。
每次使用新目錄。固定配置為 `configs/px4_simulation.json`，其中列出
模型、任務、相機與路徑文件；想調整原型時複製配置並用 `--config`
選用，不覆寫歷史研究配置。仿真控制參數不會寫入實機。

場景可以單獨建立後重用：

```bash
MUJOCO_GL=egl python -m rm27 px4-sim build --output artifacts/px4-scene-new
MUJOCO_GL=egl python -m rm27 px4-sim run \
  --workspace artifacts/px4-scene-new \
  --output artifacts/px4-mission-next --instance 61
```

場景輸入有變更時拒絕重用；移動 worktree 後重新建立。
靜態路徑審核不等於完整連續幾何證明。

## 觀看、回放和日誌

桌面環境去掉 `MUJOCO_GL=egl` 並加 `--view` 即可觀看同一完整任務；
預設 1 倍速，可用 `--speedup` 調整。關閉視窗會結束此次任務。
headless 預設要求 2 倍速，實際速度受主機感知處理限制。

```bash
python -m rm27 px4-sim run --workspace artifacts/px4-scene-new \
  --output artifacts/px4-mission-view-new --instance 62 --view --speedup 1
python -m rm27 view artifacts/px4-scene-new/tags/scene.xml
python -m rm27 px4-sim replay artifacts/px4-mission-new/flight \
  --output artifacts/px4-replay-new
python -m rm27 px4-sim log PATH_TO_ORIGINAL.ulg --output artifacts/px4-log-new
```

`view` 只看靜態場景。`replay` 重放錄製電機输入並比较所有物理狀態；
通过代表这些输入的确定性物理重现，不代表另一次異步閉環飛行會相同。
`log` 只讀分析 ULog，解碼成功不等於任務成功。

根目錄 `summary.json` 必須為 `passed: true`；它要求子任務成功、
參數讀回確認、正常自動停槳、有效 ULog 和 EKF2 實際 EV 位置／速度
融合，並非依賴進程退出碼。詳細資料位於：

- `dependencies.json`、`scene/summary.json`：依賴、場景與素材 hash。
- `flight/summary.json`：任務事件、獨立物理接觸／恢復／落位評分。
- `flight/metadata.json`、`source_diff.patch`、`image_inputs/`：提交、差異和輸入快照。
- `flight/frames.jsonl`、`mavlink.jsonl`、`physical_audit.jsonl`：控制、遙測和獨立物理記錄。
- `flight/vio/`、`flight/return_camera/`：本次飛行圖像、估計及 AprilTag 觀測。
- `flight/log/`、`ulog-*/summary.json`：PX4 原始 ULog 和实际融合旗標。

視覺／VIO的新算法與實機研究由另一台電腦的
[`rm27_slam`](https://github.com/shiuhou/rm27_slam)負責；待使用者通知
成果就緒後再 pull／整合。這裡只保存已存在的仿真依賴與 PX4接口。
