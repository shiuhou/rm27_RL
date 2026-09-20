# 驗證記錄

日期：2026-09-13

## 已執行

- `python -m compileall -q .`：通過。
- `python radio_se3.py --help`：正常退出，列出四個子命令。
- `python fetch_upstream.py --help`：正常退出。
- `python -m unittest discover -s tests -v`：**39 項通過，0 失敗**。

39 項包含：13 項校準／校準檔、15 項目標生成器、4 項輸入開關與四元數、3 項 ZIP 匯入、4 項控制器適配介面契約。

測試確認的例子包括：中心死區精確歸零、非對稱兩端量程正規化、方向反轉、重複軸與多軸混動拒絕、油門沒有放中間時拒絕校準／啟用、目標速度與位置限制、回中後停止漂移、斷線鎖定、四元數轉序、力／力矩單位與電機輸出範圍。

適配器契約測試使用假的 Controller、Mixer 與 MuJoCo `mj_step`；沒有模擬真實物理運動，也沒有驗證作者原始控制器的閉環穩定性。

本工作環境使用 Python 3.13 與預裝 NumPy；不是 `requirements.txt` 選定的 Windows/Python 3.11 完整環境。

## 未執行／待在使用者電腦驗證

- 作者完整 XML／mesh 載入與 MuJoCo 動力學四定點追蹤。
- MuJoCo passive viewer 與 pygame 雙視窗整合。
- 實體遙控器的藍牙配對、SDL 軸序、真實採樣與斷線事件。
- 指定版本依賴組合在 Windows 的完整安裝與運行。
- 所有真機測試；本包沒有真機通訊模組。

## 本機端到端驗證入口

```text
python radio_se3.py demo --headless --seconds 24
python radio_se3.py demo
python radio_se3.py probe
python radio_se3.py calibrate
python radio_se3.py fly
```

headless 模式的 `PASS` 只表示四個仿真定點區段末端誤差通過選定門檻；它不驗證 Bluetooth 或圖形介面，也不是全狀態穩定性證明。
