# 獨立機庫與基地場景

```bash
python -m rm27 build-isolated --output artifacts/isolated-new
python -m rm27 view artifacts/isolated-new/scene.xml
```

預設讀取 `artifacts/dock-final/scene.xml`，也可用 `--scene` 指定來源。
來源 XML 雜湊已固定；更換內容前需要重新審核抽取範圍並更新配置。
輸出資料夾必須是新目錄。建置後查看 `summary.json` 的 `passed`。

保留現用紅方機庫、H 頂板及支柱，以及負 X 的藍方基地。
保留原世界位置與約 12 米間距，不包含飛機、其他障礙或飛控。
原支柱底部位於 Z=0.4 m；新場景加同截面柱腳延伸至平地，機庫高度不變。

基地是現有場地模型的抽取版本，並非重新轉換 `base2.STEP`。
`configs/isolated_scene.json` 定義抽取區域，只保留完全位於區域內的
連通曲面，不裁切三角形或修補網格。顏色沿用原材質。
綠燈與裝甲接觸片沿用 `configs/base_target.json`。

地面、柱腳、支柱、頂板、機庫及裝甲測試片有碰撞；基地塔身和綠燈
為視覺模型。抽取不保證 CAD 曲面完整性，也未建立基地塔身碰撞。
原全場路線審核和 SITL 成功紀錄不適用於這個新場景。

驗證方式：MuJoCo 編譯、靜態 100 步有限值檢查、保留幾何的位置比較，
以及 EGL 多角度畫面人工檢視。這是 headless 靜態場景驗證，沒有執行飛行。
