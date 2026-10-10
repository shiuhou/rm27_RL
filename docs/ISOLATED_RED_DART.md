# 藍方基地＋紅方飛鏢發射區獨立場景

2026-09-27 依指定的 `artifacts/isolated-dock-base-20260916-v6/scene.xml`
新增場地，原 v6 保持不變。本次範圍是「只移除紅色機庫，加入紅色飛鏢區」。

```bash
cd /home/shiuhou/Projects/rm27_drones
/home/shiuhou/venvs/mujoco/bin/python -m rm27 view artifacts/isolated-blue-base-red-dart-20260927-v1/scene.xml
```

重建必須使用新輸出目錄：

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm27 build-isolated-dart --output artifacts/isolated-red-dart-new
```

`configs/isolated_red_dart.json` 固定兩份來源 XML 的 SHA-256。
建置器記錄來源與使用到的網格雜湊，複製所需資產到新目錄，並使用相對路徑。
整個輸出目錄可一起移動，不依賴原 artifact 或上游資產的絕對路徑。

## 物件範圍

- 移除 `field__visual__hangar_red_*`、`field__collision__hangar_red_*`
  及配套的 `test_column_footing`：共 13 個幾何物件。
- 保留其餘 v6 的 27 個幾何物件，包括藍方基地、綠燈、原裝甲測試片、
  地面與獨立的 `rm27_dock_*` 停機庫；停機位 site 也維持原位。
- 從現有場地匯入完整 `launch_station_red`：12 個外觀網格和 97 個碰撞片。
  此物件與 `research/Darts/config.json` 使用的紅方飛鏢發射站相同。
  世界位置維持 `(12.5905, -6.2265, 0)` m，未移動、縮放或加入旁邊的八角高地。

「紅方 H 機庫支柱」與 `rm27_dock` 是兩組獨立物件。
依本次只移除前者的要求，後者仍是固定在原高度的靜態停機庫，底下不再有支柱。
v6 本來就沒有飛機；新場景也沒有加入飛機、飛鏢本體或控制器。

## 證據與限制

本次輸出 `artifacts/isolated-blue-base-red-dart-20260927-v1/summary.json`
記錄 `passed: true`。驗證模式為 **headless 靜態 MuJoCo 檢查**：
編譯、100 步有限值檢查、逐物件世界位置／姿態／尺寸／接觸設定／顏色比較、
停機位姿態比較、移除清單與總數檢查，以及來源雜湊保持不變。
總計 136 個幾何物件，其中 107 個啟用碰撞。

`overview.png`、`red_dart.png`、`blue_base.png`、`retained_dock.png`
及 `views.png` 是本次 headless EGL 檢視圖；預覽關閉陰影及反射以檢查表面，
未修改 XML 的原有地面材質。沒有執行 live-view、SITL 或飛行任務。

根目錄回歸測試：
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/shiuhou/venvs/mujoco/bin/python -m pytest tests -q`
結果為 **107 passed in 7.28s**。新增的三項測試包含物件範圍與資產依賴、
來源異動拒絕建置、既有輸出保護；也確認複製輸出目錄後不需原始資產即可載入。

基地塔身仍為視覺模型，原有裝甲測試片保留碰撞。飛鏢站沿用場地現有的
主要面板碰撞片，細桿、螺絲與部分小面板僅供顯示；這不代表完整機械碰撞已驗證。
匯入來源為 `configs/scene.json` 指向的現用場地，沒有重新採用已退役的全場 STEP。
