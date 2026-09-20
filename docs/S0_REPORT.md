# S0 批次驗證與可替換參數入口

2026-09-12：S0 通過。十次獨立啟動 ArduPilot SITL、使用同一現有場景與原
micro-flight 門檻，10/10 完成起飛、10 s 懸停、0.12 m 平移、返回、落位停槳。
全部 headless、請求 5×；導航仍有 MuJoCo 真值輔助。

## 證據

- 批次：[summary.json](../artifacts/s0-batch-01/summary.json)。全部嘗試與 stdout
  保留在同一批次目錄，沒有刪除／挑選失敗試驗。
- 每次 metadata 的 scene hash、SITL binary hash、runner hash、參數內容一致，
  以 `rm27.experiments.batch_micro_flight.audit_trials` 重新比對通過。
- 場景仍為 `artifacts/dock-final/scene.xml`，SHA-256
  `768545c221a1b9337ba1dbc5ed49dc5fa56dec15eed875c3d73812efbbf646cb`。
- 批次開始時 runner hash：
  `9e51c3ce6c83e400ec8a488249ca9dbd641103d50d5f63600d822d57ccadeb02`。
  此後新增了可選 dynamics 入口，另有下文的端到端回歸結果；不能把十次批次
  說成全部在新 runner 版本下重跑。
- trial-01 的完整錄製輸入回放精確重現 52,026 幀；這不是十次閉環位元一致。
- 十次 subprocess 壁鐘合計約 500.35 s；包含初始化與日誌輸出。

| 指標 | 最小 | p95 | 最大 |
|---|---:|---:|---:|
| 最終 XY 落位誤差，mm | 1.4441 | 2.5067 | 2.8347 |
| 接地速度，m/s | 0.029380 | 0.030092 | 0.030217 |

十次通訊均無 missing、duplicate、reset 或 invalid frame。
有限名義工況批次不證明任意擾動下可靠性，也不證明真機毫米級落位。

## 校準入口回歸

顯式 `dynamics_provisional.json` 建置
`artifacts/dock-profile-01/scene.xml` 成功；質量、慣量、馬達上限、時間常數與
偏航力矩係數由 profile 進入編譯模型，並有變更值與不匹配拒絕測試。
原路徑不指定 profile 時保留 180 g 檢查。

新入口的 [profile-flight-01](../artifacts/profile-flight-01/summary.json) 實際
SITL 起降往返通過；[回放](../artifacts/profile-flight-01/replay.json) 精確匹配。
profile 快照和 hash 保存於飛行目錄，未修改真機參數。

## S1 量測工具的有限結果

[contact-metrics-01](../artifacts/contact-metrics-01/summary.json) 保留首次失敗：
2 ms 接觸力取樣积分與球體動量變化差約 12.05%，1 ms 約 5.98%。
這是該取樣／數值積分實驗的誤差，不直接等同 MuJoCo 動量不守恆。

[contact-metrics-02](../artifacts/contact-metrics-02/summary.json) 增加
0.25、0.125 ms 探針後，最後兩級衝量差 2.086%、峰值差 0.664%；
0.125 ms 衝量對動量誤差 0.743%。前後面與力座標單元測試通過。
原飛行仍維持 2 ms，沒有用細步長探針結果宣稱飛行或高速撞擊已驗證。

S1 仍 partial：基地周邊／完整走廊、實際裝甲身份與結構模型尚未驗收；
球體探針不是 UAV 接觸恢復或裁判傷害試驗。S2–S5 尚未通過。

## .BIN 接入驗證

用本次 SITL 產生的 DataFlash 做 parser smoke test：
[log-intake-sitl-01](../artifacts/log-intake-sitl-01/summary.json)。原檔 hash
解析前後一致；訊息盤點成功，缺少 OF、RFND。此測試驗證解析入口，未使用
真機 .BIN，也不表示 MTF-02P 已進入飛控融合。

本次 root suite：24 passed。使用命令與資料需求見
[CALIBRATION_WORKFLOW.md](CALIBRATION_WORKFLOW.md)。
