# 飛鏢簡化彈道實驗

在目前 `configs/scene.json` 指定、帶紅方機庫的場地，使用
`mech_darts.STEP`，從紅方飛鏢發射站射向藍方基地**綠燈上方**的表面。
這是獨立的小研究，不會啟動 ArduPilot 或執行無人機攔截。
程式、設定、說明和產物皆位於 `research/Darts/`；原始 CAD 與影片保持不變。

## 直接查看

在桌面終端執行：

```bash
cd /home/shiuhou/Projects/rm27_drones
/home/shiuhou/venvs/mujoco/bin/python -m research.Darts view research/Darts/artifacts/flight-04-apex-5m --camera follow
```

預設 0.25 倍速，自動重播；到達目標後停留兩秒。用 `--speed 1` 看原速。
滑鼠可旋轉、縮放視角；跟隨模式只自動更新觀察中心。

| 按鍵 | 功能 |
|---|---|
| `1` | 全場 |
| `2` | 跟隨飛鏢 |
| `3` | 目標近景 |
| `4` | 發射站近景 |
| `5` | 側面看拋物線 |
| 空白鍵 | 暫停／繼續 |
| `R` | 重新發射 |

直接播放已生成的影片：

```bash
xdg-open research/Darts/artifacts/render-04-apex-5m/flight.mp4
xdg-open research/Darts/artifacts/render-04-apex-5m/trajectory.png
```

`view` 是 MuJoCo 視窗中的運動學重播；不是只打開靜態 XML。
無桌面的環境使用下方 `render` 命令。

## 重新建立或調整

```bash
cd /home/shiuhou/Projects/rm27_drones
DART_RUN="research/Darts/artifacts/flight-$(date +%Y%m%d-%H%M%S)"
/home/shiuhou/venvs/mujoco/bin/python -m research.Darts build --output "$DART_RUN"
MUJOCO_GL=egl /home/shiuhou/venvs/mujoco/bin/python -m research.Darts render "$DART_RUN" \
  --output "${DART_RUN}-render" --video
/home/shiuhou/venvs/mujoco/bin/python -m research.Darts view "$DART_RUN" --camera follow
```

每次選新輸出目錄。`build` 會呼叫現有 root 場景建構器，按照目前已審閱的
場地版本與機庫設定重新生成，並檢查場地 Git 版本；不會自動更新第三方倉庫。
首次需要 repo 的 `requirements-integration.txt`；MP4 匯出另外使用本機已有的
OpenCV（`cv2`）。後續可加 `--cad-assets research/Darts/artifacts/cad-01` 重用相同
STEP SHA-256 的網格匯出，避免重複 CAD 轉換。

在 [config.json](config.json) 調整：

- `flight_time_s`：飛行時間；初速由起終點與重力反解。縮短時間通常使路徑更平、速度更高。
- `launch_offset_local_m`：相對 `launch_station_red` 的發射點。
- `target_ray_start_world_m`：在上方表面選取目標的射線起點。
- `target_width_height_m`：示範目標的有效矩形尺寸。
- `sample_dt_s`：CSV 取樣間隔。

座標沿用場地世界座標，公尺、Z 向上。發射口的精確位置、目標矩形尺寸及
飛行時間是可調假設；上方表面的座標／法向由現有場地網格射線求交得到。
它與無人機任務使用的**綠燈下方裝甲**獨立，不修改 root 的目標設定。

## 模型與驗證範圍

採用 `p(t) = p0 + v0*t + 0.5*g*t²`。被追蹤的是 CAD 前端面中心，並非質心。
完整 CAD 只做剛體座標變換及 mm→m 換算，沒有放大；鏢身朝向隨速度方向變化，
這是顯示假設。MuJoCo mocap body 接收解析位置與姿態，使用 `mj_forward` 顯示；
沒有藉由 `mj_step` 求解氣動、姿態穩定或接觸反作用。

到達有限大小的目標平面時記錄幾何命中並凍結，不模擬反彈／損壞／裁判扣血。
以連續線段射線分別檢查沿途原有碰撞幾何及視覺幾何，避免只看離散點漏掉薄牆。
這項檢查僅涵蓋中心路徑，不保證尾翼／整個鏢身的淨空，也不檢測起點已在實體內的情況。
MuJoCo 重播與新輸出的畫面不顯示黃色路徑；軌跡資料仍保留在 CSV。
四架原有無人機保持初始停放姿態。既有 MP4 是先前畫面，需重新 render 才會更新。

`summary.json` 的 `passed: true` 僅代表：目標平面命中、中心路徑未穿過原有
碰撞／視覺表面、原有場景幾何與自由度保留；不代表真實飛鏢或攔截控制已驗證。

## 本次结果

`flight-04-apex-5m/summary.json` 使用同一發射點與目標，把世界 Z 最高點
設定為 5.0 m。依照無阻力的上下行時間反解，結果如下：

| 項目 | 結果 |
|---|---|
| 場地版本 | `67c13270e213cf3f69da2619e349eed273432330` |
| 發射點 XYZ | `(12.4505, -6.2265, 1.05)` m |
| 上方目標 XYZ | `(-11.62324, 0.02, 1.26177)` m |
| 綠燈中心高度 | `1.17` m |
| 飛行時間 | 約 `1.77038` s |
| 反解初速 | 約 `16.5787` m/s |
| 最高高度 | `5.0` m |
| 命中總速度 | 約 `16.4530` m/s |
| 命中表面法向速度 | 約 `13.8753` m/s |
| 幾何命中 | 通過 |
| 中心路徑碰撞／視覺表面檢查 | 均無提前相交 |
| STEP | 87 個實體、156,360 個三角形，沿鏢身方向約 221.46 mm |

STEP SHA-256：`18c530bf2ec6dc860d9cfde36e0d66a678a8e7e0a4a9f482cd9fed96d57766fb`。
原影片 `simtohit.mp4` 用於理解展示方式，沒有从影片擬合飛行速度或軌跡。

與舊 `flight-02` 比較：飛行時間由 1.4 s 增加至 1.77038 s；最高點由
3.56050 m 增至 5.0 m；因為水平距離不變但可用時間增加，反解初速由
19.1010 m/s 降至 16.5787 m/s。這是固定起終點和指定最高點的數學結果，
不代表真實發射器會自動採用這組速度。

主要產物：

- `scene.xml`：完整有機庫場景，加上飛鏢及獨立上方目標。
- `trajectory.csv`：時間、前端 XYZ、速度 XYZ、姿態四元數 wxyz。
- `summary.json`：來源雜湊、假設、發射／命中數據與中心路徑檢查。
- `cad/audit.json`：CAD 單位、邊界、實體數；重用 CAD 時位於指定快取目錄。
- 渲染目錄：`flight.mp4`、`trajectory.png`、全場／發射／命中近景。

未來攔截實驗可直接呼叫 `BallisticPath.state(t)` 取得同一世界座標中的位置與速度，
或使用 CSV 重播；感測噪聲、飛鏢動力學及無人機攔截將分開加入。

驗證命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest research/Darts/tests -q
```

本次 8 項測試通過，涵蓋解析拋物線／能量、有效命中與偏靶、入射方向、
跨樣本薄牆檢測、視覺與碰撞分類，以及鏢身方向與取樣端點。
