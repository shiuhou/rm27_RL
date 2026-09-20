# RM27 快速啟動

目前主線：先讓一架飛機穩定完成起飛、飛向基地、返回機庫及降落，再擴展至多機。基地高速接觸、碰撞後恢復與視覺導航尚未完成。

## 每次開機

```bash
cd /home/shiuhou/Projects/rm27_drones
source /home/shiuhou/venvs/mujoco/bin/activate
python -m rm27 --help
```

所有指令都在 repo 根目錄執行。不需要啟動 IsaacLab，也不需要設定 `PYTHONPATH`。

## 查看場景

新 STEP 飛機：單台放在機庫一號位（靜態外觀）：

```bash
python -m rm27 view artifacts/cad-drone-dock-20260916-v3/scene.xml
```

結構判讀、尺寸與重建指令：[CAD_DRONE.md](docs/CAD_DRONE.md)。

獨立測試場地（只有機庫、支柱、現用藍方基地與地面）：

```bash
python -m rm27 view artifacts/isolated-dock-base-20260916-v6/scene.xml
```

重建：`python -m rm27 build-isolated --output artifacts/isolated-new`
此場景沒有飛機；基地塔身仍是視覺模型，碰撞範圍見 [獨立場景說明](docs/ISOLATED_SCENE.md)。

完整場地、機庫與四架靜態飛機：

```bash
python -m rm27 view artifacts/dock-final/scene.xml
```

加入基地綠燈與測試接觸片的場景：

```bash
python -m rm27 view artifacts/base-target-v4/scene.xml
```

Viewer 只用來查看場景，不會自動開始飛行。

## 單機短程起降

```bash
RM27_RUN="artifacts/micro-flight-$(date +%Y%m%d-%H%M%S)"
python -m rm27 micro-flight artifacts/dock-final/scene.xml \
  --output "$RM27_RUN" --instance 31 --view
python -m json.tool "$RM27_RUN/summary.json"
```

流程：起飛、懸停、短距離平移、返回、降落及停槳。省略 `--view` 可在終端快速執行。這個任務使用 MuJoCo 真值輔助精準返回。

## 飛往基地前等待點並返回

```bash
RM27_ROUTE="artifacts/route-$(date +%Y%m%d-%H%M%S)"
python -m rm27 route-flight artifacts/base-target-v4/scene.xml \
  --audit artifacts/corridor-reviewed/summary.json \
  --output "$RM27_ROUTE" --instance 43 --view
python -m json.tool "$RM27_ROUTE/summary.json"
```

這是低速、真值輔助的完整往返，約 594 秒物理時間；目前不會撞擊裝甲板。省略 `--view` 可無視窗執行。

分析路線或驗證馬達輸入重播：

```bash
python -m rm27.experiments.analyze_route "$RM27_ROUTE" --output "${RM27_ROUTE}-analysis"
python -m rm27.simulation.replay_micro_flight "$RM27_ROUTE"
```

## 重建機庫場景

只有 CAD、場地版本或配置改變時才需要重建：

```bash
RM27_SCENE="artifacts/dock-$(date +%Y%m%d-%H%M%S)"
python -m rm27 build-dock --output "$RM27_SCENE"
python -m rm27.integration.check_scene "$RM27_SCENE/scene.xml"
python -m rm27 view "$RM27_SCENE/scene.xml"
```

重新生成基地目標場景：

```bash
RM27_BASE="artifacts/base-target-$(date +%Y%m%d-%H%M%S)"
python -m rm27.integration.base_target artifacts/dock-final/scene.xml --output "$RM27_BASE"
python -m rm27 view "$RM27_BASE/scene.xml"
```

## 測試

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

只在首次配置或依賴改變時安裝：

```bash
python -m pip install -r requirements-integration.txt
```

## 查看已有結果

```bash
python -m json.tool artifacts/s0-batch-01/summary.json
python -m json.tool artifacts/route-flight-reviewed/summary.json
xdg-open artifacts/route-analysis-03-fixed/route.png
```

不要按資料夾名稱判斷成功；以 `summary.json` 中的 `passed: true` 為準。

## 常見錯誤

| 問題 | 處理方式 |
|---|---|
| `No module named rm27` | 回到 repo 根目錄，重新執行「每次開機」三行指令。 |
| 輸出目錄已存在 | 使用新的時間戳目錄，保留舊實驗證據。 |
| `Address already in use` | 確認沒有同一測試仍在執行，或改用另一個 `--instance`。 |
| Viewer 無法開啟 | 在本機桌面終端執行；SSH 環境只跑無視窗測試。 |

## 詳細文件

| 內容 | 文件 |
|---|---|
| Milestone 與驗收條件 | [docs/MILESTONES.md](docs/MILESTONES.md) |
| 專案架構與責任邊界 | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| 單機短程飛行結果 | [docs/MICRO_FLIGHT.md](docs/MICRO_FLIGHT.md) |
| 基地路線與安全走廊 | [docs/CORRIDOR_REPORT.md](docs/CORRIDOR_REPORT.md) |
| 真機硬件與待確認資料 | [docs/REAL_HARDWARE.md](docs/REAL_HARDWARE.md) |
| `.BIN`、動力與接觸校準 | [docs/CALIBRATION_WORKFLOW.md](docs/CALIBRATION_WORKFLOW.md) |

`docs/archive/chatgptre.md` 只作歷史參考；當前規格以最新指示、原始 CAD／日誌、官方資料及可重現測試為準。
