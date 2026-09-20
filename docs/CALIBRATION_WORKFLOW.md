# 可替換模型與真機日誌接入

這些入口供單機研究使用。`configs/dynamics_provisional.json` 是現有工程估計的
顯式版本，沒有宣稱已用真機辨識；預設建置與原 G1 基線維持原路徑。

## 接收 .BIN

```bash
python -m rm27.experiments.log_intake flight_logs/data/your-flight/original.BIN \
  --source-kind real --output artifacts/intake-your-flight
```

輸入只讀，解析前後比對 SHA-256；輸出訊息種類、感測器實例的數值範圍、
缺少的訊息、參數末值與變動紀錄。輸出目錄必須是新的。
`passed` 只代表有效日誌盤點且來源未改變，不表示飛行或校準通過。
SITL 日誌必須填 `--source-kind sitl`，不可當成真機量測。

數值範圍保留 DataFlash 解碼單位，尚未做飛行階段分段、同步或模型擬合。
日誌內的參數不是可直接寫回飛控的推薦配置。

## 替換動力模型

複製暫定 profile 到新的檔案，填入辨識結果與來源；不要覆寫原始輸入。
支持整機質量、三個主慣量、單馬達推力上限、馬達時間常數、推力對偏航力矩比。
frame 明確限制重心在模型原點、主慣量軸與機體軸一致；若不成立，需先修改
幾何／慣性表示，不能直接塞入三個數字。

`status` 可為 `engineering_estimate`、`fitted_candidate`、`validated`。
後兩者須附 `evidence`，每項包含相對 profile 的 `path` 與 `sha256`；載入時
核對檔案。`validated` 的實際成立仍須有獨立保留資料的驗證報告，檔案存在
本身不是物理有效性的證明。

```bash
python -m rm27 build-dock --dynamics configs/dynamics_provisional.json \
  --output artifacts/dock-profile-new
python -m rm27 micro-flight artifacts/dock-profile-new/scene.xml \
  --dynamics configs/dynamics_provisional.json --output artifacts/profile-flight-new --instance 42
python -m rm27.simulation.replay_micro_flight artifacts/profile-flight-new
```

新場景的 audit.json 保存 profile hash 和來源狀態。飛控參數不會由 profile
自動改寫；模型改變後必須重新驗收，不能沿用舊飛行成功。
飛行開始前比對 profile 與編譯模型，並保存 profile 快照供回放核對。
不指定 profile 時，原有 180 g 限制仍保留。

## 接觸量測

```bash
python -m rm27.experiments.contact_probe --output artifacts/contact-probes-new
```

獨立零重力球體探針驗證力的座標轉換、正背面法向投影、衝量／動量一致性
與時間步長收斂。輸出保留粗步長失敗以及細步長結果。
`accepted_timestep_s` 僅適用該探針，不是全場飛行或高速撞擊推薦步長。
contact_metrics.py 提供接觸點角速度及目標運動修正；沒有內建裁判傷害閾值。

## 資料真正能替換什麼

| 項目 | 可提供的證據 | 現在的邊界 |
|---|---|---|
| 質量、重心與慣量 | 稱重／結構量測，動態試驗輔助 | 單一 .BIN 不能唯一確定全部慣量 |
| 推力、反應與偏航 | .BIN 姿態／角速度／輸出與台架曲線共同約束 | 推力、慣量與延遲不可任意互相補償 |
| MTF-02P／羅盤 | .BIN 感測器、融合與外部運動參照 | 盤點不等於已完成無 GPS 融合 |
| 電池與推力下降 | 電壓／電流及不同負載的推力量測 | 目前 profile 為固定推力，尚無電壓查表模型 |
| 碰撞恢復與結構 | 接觸台架、形變、間隙和獨立碰撞試验 | 剛體仿真無破損不能證明真機存活 |
| OS04A10／返航標記 | 內外參、鏡頭、影像、端到端時間戳 | .BIN 不能替代相機標定及圖像 |

所有新參數先放在獨立 profile／場景，使用保留工況驗證。只有通過的版本
才能更新里程碑；原始 .BIN、原配置與失敗試驗保持可追溯。
