# RM27 每日啟動指令速查

目前日常使用的是 **MuJoCo + ArduPilot SITL**，不需要先執行 `IsaacLab/isaaclab.sh`。
以下指令適用於這台電腦；每次開新終端，先執行第 1 節，再選擇要做的操作。

## 1. 每次開機／開新終端先執行

```bash
cd /home/shiuhou/Projects/rm27_drones
source /home/shiuhou/venvs/mujoco/bin/activate
python -m rm27 --help
```

保持在 repo 根目錄執行，下文的 `python` 就是 MuJoCo 虛擬環境裡的 Python。
不需要每次安裝依賴，也不需要額外設定 `PYTHONPATH`。

如果要在 VS Code 開啟整個專案（已安裝 `code` 指令時）：

```bash
code .
code START_HERE.md
```

## 2. 開啟場地、機庫與四架小飛機

使用目前已生成的整合場景，開啟互動視窗：

```bash
python -m rm27 view artifacts/dock-final/scene.xml
```

這是查看模型與無動力物理場景的 viewer，**不會自動執行飛行任務**。
關閉視窗即可結束。若想看實際測試的飛行結果，使用第 4 節。

只看上游原始場地：

```bash
python -m rm27 view rm_uav_lab_m600/rmuc2026_battlefield_mjcf/model.xml
```

只看 `base2.STEP` 已轉出的獨立 CAD 預覽：

```bash
python -m rm27 view artifacts/base-preview-v2/scene.xml
```

基地預覽包含兩組大型靜態視覺模型，載入較慢，沒有碰撞設定。

## 3. 即時觀看單機起飛、懸停、返回及降落

每次執行建立新的紀錄目錄；以下用時間命名。一次只執行一個此測試。

```bash
RM27_RUN="artifacts/micro-flight-$(date +%Y%m%d-%H%M%S)"
python -m rm27 micro-flight artifacts/dock-final/scene.xml --output "$RM27_RUN" --instance 31 --view
```

流程：1 號機正常解鎖 → 起飛至停機位底面上方約 0.6 m → 懸停 10 秒 →
平移 0.12 m → 返回原位 → 對準及下降 → 接觸停機位底面 → 自動停槳。
另外三架保持停放。精準返回使用模擬真值輔助定位。

指令會自行啟動及清理本次 ArduPilot SITL，**不需要另外啟動飛控**。
`--view` 會開啟 MuJoCo 視窗，直接顯示正在飛行的狀態，預設約 1× 速度。
鏡頭對準機庫，左上角顯示階段、物理時間、高度、水平偏差、模式與解鎖狀態。
初始化與等待飛控就緒通常佔前約 43 秒的物理時間，這段期間飛機停放是正常的。
完整流程約 100 秒物理時間；實際速度依渲染與電腦負載而異。

滑鼠左鍵拖曳旋轉視角、右鍵拖曳平移、滾輪縮放。視窗只觀察飛行，不接受
透過 viewer 改變飛行物理狀態。提早關閉視窗會取消本次測試並清理 SITL；
完成後視窗會自動關閉。結束時查看終端的 `passed: true`；失敗原因在 `failure`。

只想快速跑無視窗測試，省略 `--view` 即可，預設請求 5× 速度：

```bash
RM27_RUN="artifacts/micro-flight-$(date +%Y%m%d-%H%M%S)"
python -m rm27 micro-flight artifacts/dock-final/scene.xml --output "$RM27_RUN" --instance 31
```

兩種模式都可額外加上 `--speedup 2` 等正數指定請求速度；實際速度受電腦負載限制。

在同一個終端查看結果並產生圖像：

```bash
python -m json.tool "$RM27_RUN/summary.json"
python -m rm27.plot_micro_flight "$RM27_RUN"
MUJOCO_GL=egl python -m rm27.replay_micro_flight "$RM27_RUN" --render
xdg-open "$RM27_RUN/flight.png"
xdg-open "$RM27_RUN/replay-views.png"
```

`flight.png` 是量測曲線；`replay-views.png` 是六個階段的物理重播截圖，並非影片。
重播指令會核對記錄的馬達輸入是否重現原始物理狀態，不會再次啟動 SITL。
若要中途停止測試，在執行該測試的終端按 `Ctrl+C`；中止不算完成降落驗證。

## 4. 直接開啟已驗證的飛行結果

不用重新飛行即可查看：

```bash
xdg-open artifacts/micro-flight-alignment-1/replay-views.png
xdg-open artifacts/micro-flight-alignment-1/flight.png
python -m json.tool artifacts/micro-flight-alignment-1/summary.json
```

完整說明：[單機飛行驗證報告](docs/MICRO_FLIGHT.md)。
即時視窗：[使用方式與驗證](docs/LIVE_VIEW.md)。
已通過的兩次紀錄為 `micro-flight-alignment-1` 和 `micro-flight-alignment-2`。
`micro-flight-final` 是歷史失敗試驗，不能只看資料夾名稱判斷是否成功。

若要查看剛才另一個終端產生的結果，先把變數設成實際目錄，再用第 3 節的查看指令：

```bash
ls -dt artifacts/micro-flight-*/
RM27_RUN="artifacts/micro-flight-alignment-2"
```

## 5. 重新生成機庫整合場景

平常直接使用第 2 節的現有場景即可；需要重新生成時再執行：

```bash
RM27_SCENE="artifacts/dock-$(date +%Y%m%d-%H%M%S)"
python -m rm27 build-dock --output "$RM27_SCENE"
python -m rm27.check_scene "$RM27_SCENE/scene.xml"
python -m rm27 view "$RM27_SCENE/scene.xml"
```

生成靜態多角度預覽並開啟：

```bash
MUJOCO_GL=egl python -m rm27.render "$RM27_SCENE/scene.xml"
xdg-open "$RM27_SCENE/views.png"
```

如果要用新場景測試飛行，把第 3 節的 `artifacts/dock-final/scene.xml`
換成 `"$RM27_SCENE/scene.xml"`。場地版本或 CAD 雜湊不符時，建置會停止，
需要先審核新模型並更新設定。

## 6. 執行目前的整合測試

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests -q
```

目前是 14 項測試；包含視窗狀態隔離與更新頻率檢查。關閉外部 pytest 插件
自動載入，可避免本機 ROS 插件干擾。

只有初次配置或依賴有變動時，才需要安裝：

```bash
python -m pip install -r requirements-integration.txt
```

## 7. 常見狀況

| 狀況 | 處理方式 |
|---|---|
| `No module named rm27` | 重新執行第 1 節，確認目前在 repo 根目錄。 |
| 用錯 Python／MuJoCo 找不到 | 執行 `which python`，應顯示 `/home/shiuhou/venvs/mujoco/bin/python`。 |
| 輸出目錄已存在 | 換新的時間戳目錄；保留舊實驗紀錄。 |
| `Address already in use` | 先確認同一個測試是否仍在執行；或改用未使用的 `--instance 32`。 |
| viewer 無法開啟 | 在本機桌面終端執行；只有 SSH 無桌面時改用 `MUJOCO_GL=egl` 的渲染指令。 |
| 曾全域設定 `MUJOCO_GL=egl` | 互動 viewer 可改用 `env -u MUJOCO_GL python -m rm27 view artifacts/dock-final/scene.xml`。 |
| 關閉終端後 `$RM27_RUN`／`$RM27_SCENE` 消失 | 變數只在該終端有效，重新設成要查看的實際路徑。 |
| 新 clone 找不到 `artifacts/` 或 STEP | 生成物與原始 CAD 為本機檔案；先準備 CAD、對應子倉庫及依賴，再建置場景。 |

## 8. 常用檔案

| 想查看／修改 | 路徑 |
|---|---|
| 專案架構與目前進度 | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| 真機飛控、感測器與待確認資料 | [docs/REAL_HARDWARE.md](docs/REAL_HARDWARE.md) |
| 真機 `.BIN` 日誌放置方式 | [flight_logs/README.md](flight_logs/README.md) |
| 機庫位置、場地版本 | [configs/scene.json](configs/scene.json) |
| 暫定小飛機尺寸、重量與動力假設 | [configs/drone.json](configs/drone.json) |
| 本次小飛機 SITL 參數 | [configs/micro_hover.parm](configs/micro_hover.parm) |
| 機庫幾何驗證 | [docs/DOCK_VALIDATION.md](docs/DOCK_VALIDATION.md) |
| 單機飛行驗證 | [docs/MICRO_FLIGHT.md](docs/MICRO_FLIGHT.md) |
| 基地 CAD 審核 | [docs/BASE_AUDIT.md](docs/BASE_AUDIT.md) |

`chatgptre.md` 是舊對話紀錄，只供追溯歷史和尋找線索；目前規格以最新使用者
資訊、原始日誌／CAD、官方資料和本 repo 的可重現驗證為準。

模型或飛控參數修改後需要重新驗證；原本的 G1' 基準仍保留在子倉庫內。

## 基地綠燈與接觸片預覽

保留場地基地，新增暫定綠燈與藍色碰撞片：

```bash
python -m rm27 view artifacts/base-target-v3/scene.xml
```

重新生成（使用新的輸出目錄）：

```bash
python -m rm27.base_target artifacts/dock-final/scene.xml --output artifacts/my-base-target
python -m rm27 view artifacts/my-base-target/scene.xml
```

這是靜態預覽，不會執行基地接觸飛行；目前只新增接觸片碰撞，塔身仍是視覺模型。
綠燈位置／大小為可調近似，碰撞片是候選裝甲面內的測試區域。
詳見 [BASE_TARGET.md](docs/BASE_TARGET.md)。
