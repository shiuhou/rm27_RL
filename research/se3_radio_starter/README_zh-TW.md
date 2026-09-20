# SE3 + MuJoCo + 藍牙遙控器：復現起步包

版本：0.1 · 2026-09-13

目標是復現這種互動：**遙控器進入藍牙手柄模式 → pygame 讀取搖桿 → 更新目標位置與朝向 → 作者的 SE3 控制器讓仿真飛機追蹤。** 不涉及真機飛控、強化學習、NMPC 或碰撞任務。

這是根據作者公開控制器介面新增的輸入層與啟動程式，**不是作者未公開的藍牙示範程式原檔**。作者的控制器、混控器、幾何運算和模型由 `fetch_upstream.py` 在你電腦下載，不包含在本壓縮包內；原始檔不會被本程式修改。

## 驗證範圍

本工作環境完成了 Python 語法檢查，以及 **39 項單元／介面契約測試**，結果見 `TEST_REPORT.md`。這些測試使用 NumPy 與假的模擬器介面；**不是 MuJoCo 動力學、pygame 視窗或實體藍牙遙控器的端到端測試**。本環境沒有安裝 MuJoCo／pygame，也無法取得作者完整模型到本機，因此沒有宣稱飛行已在這裡跑通。

你電腦上的驗證入口是 `demo` 與 `demo --headless`。先完成這個檢查，再接入手柄，便能把「控制器／模型」與「藍牙／映射」問題分開。

## 1. 安裝：以 Windows、Python 3.11 64-bit 為例

先解壓縮，在含有 `radio_se3.py` 的資料夾開啟 PowerShell。以下使用虛擬環境的完整 Python 路徑，不需要啟用 PowerShell 腳本執行權限。

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe fetch_upstream.py
```

`requirements.txt` 選定 NumPy 2.2.6、pygame 2.6.1、MuJoCo 3.3.7 作為版本基線；不是宣稱它們為最新版本，也不是宣稱這組版本已在本工作環境做整合測試。這個起步包不使用舊的 `mujoco-py`。

`fetch_upstream.py` 只下載與解壓作者的程式／模型；不安裝系統軟體、不更改遙控器韌體、不執行下載的程式。後續啟動仿真時才會匯入作者的 Python 模組。

來源是 `LGQWakkk/Quadrotor_SE3_Control`。下載器會優先把分支解析為確切 commit，再下載該 commit，並保存來源資料與 ZIP 的 SHA-256 到 `upstream/UPSTREAM_PROVENANCE.json`。若 GitHub API 不能解析 commit，會改下載指定分支並明確記錄 commit 未知。這是你執行下載當時的版本，不是本文件預先鎖死的 commit。

### GitHub 下載失敗

從作者 GitHub 頁面選 `Code > Download ZIP`，然後：

```powershell
.\.venv\Scripts\python.exe fetch_upstream.py --zip "C:\Users\你的帳號\Downloads\Quadrotor_SE3_Control-main.zip"
```

下載器拒絕覆寫已存在的 `upstream`。你已有完整作者專案時，可以不下載，啟動時使用：

```powershell
.\.venv\Scripts\python.exe radio_se3.py demo --repo "D:\projects\Quadrotor_SE3_Control"
```

請使用完整專案，不能只有三個 Python 檔；`crazyfile/scene.xml` 及其包含的模型／材質／網格檔案也需要保留。

## 2. 先不接遙控器，檢查 SE3 + 模型

```powershell
.\.venv\Scripts\python.exe radio_se3.py demo
```

程式會嘗試開啟 MuJoCo 與狀態面板。預期行為是先到 `(0, 0, 0.30)` 公尺，之後追蹤四段定點目標。每段持續 6 秒**仿真時間**，最後維持原點上方。按 `Esc` 結束。

畫面中大紅球代表目標位置，小黃球代表目標機頭方向，均為純視覺標記，不參與碰撞或產生力。狀態面板顯示真實位置、目標位置、位置誤差與四個電機的歸一化輸出。

沒有圖形介面的控制測試：

```powershell
.\.venv\Scripts\python.exe radio_se3.py demo --headless --seconds 24
```

會在四個定點區段末端量測誤差，寫入 `logs/` 的 CSV 與 JSON。四次位置誤差均小於 0.10 m 時列印 `PASS`。**0.10 m 是本起步包選定的初步驗收門檻，不是作者提供的性能保證。** 仿真時間不足 24 秒只算診斷，不算完成這組測試。

若這一步失敗，先處理模型、依賴版本、數值錯誤或圖形環境；不要用更換搖桿映射掩蓋問題。

## 3. 連接遙控器並檢查 pygame 讀值

在遙控器啟用你已確認的藍牙手柄模式，先由 Windows 完成配對。本程式不直接管理 Bluetooth 協定，而是讀取作業系統呈現給 pygame 的遊戲控制器。

```powershell
.\.venv\Scripts\python.exe radio_se3.py probe
```

視窗與終端會顯示裝置名稱、GUID、軸數，以及每一軸原始值。**一次只動一個方向**，確認各方向的讀值有改變。按 `Esc` 關閉後才執行下一個命令。

若有多個遊戲控制器，程式會列出編號。切換到其他裝置，例如：

```powershell
.\.venv\Scripts\python.exe radio_se3.py probe --device 1
```

校準與飛行也要使用相同 `--device`。不要只因為藍牙顯示已連線，就假設通道已經能正確讀取。之後改成 USB、改通道混控，或更換裝置時，重新校準。

## 4. 校準四個方向

```powershell
.\.venv\Scripts\python.exe radio_se3.py calibrate
```

校準介面採用 Mode 2 操作角色。畫面文字用英文，以免依賴額外中文字型；下列是完整操作順序。每一步都要讓搖桿保持不動，再在 pygame 視窗按 `Enter`。

1. **所有搖桿置中；油門也放在物理行程中間。**
2. 右搖桿向右到底 → 向左到底 → 回中。
3. 右搖桿向前推到底 → 向後拉到底 → 回中。
4. 左搖桿向上推到底 → 向下拉到底 → 回中。
5. 左搖桿向右到底 → 向左到底 → 回中。

每組正向、反向、回中，都各按一次 `Enter`。程式會推斷軸號、正負方向、中心與兩端量程，最後生成 `radio.json`；不需要手改軸號。如果多個通道一起變動、用了同一軸兩次、或中心不在兩端之間，會拒絕該次資料並要求重做。

若你的左右手習慣不同，可在校準時使用你希望對應該功能的實體軸；重點是 `right / forward / up / yaw_right` 四個功能各使用一個獨立軸。畫面給的是 Mode 2 提示，不是對軸序做硬編碼。

**沒有自動回中彈簧的油門，放手不會回中。** 本程式把這根軸當作「高度目標改變速度」：中間停止變高／變低，上半增加高度，下半降低高度。不要把最低油門當作這個位置控制模式的中立位置。

校準後再檢查一次：

```powershell
.\.venv\Scripts\python.exe radio_se3.py probe
```

此時會多顯示四個校準後的值。置中時應為 `0.000`，朝提示的正方向應為正值。校準檔若原本存在，舊檔備份為 `radio.json.bak`。

## 5. 啟動遙控位置跟蹤

```powershell
.\.venv\Scripts\python.exe radio_se3.py fly
```

**啟動後，仿真控制器就會嘗試追蹤 0.30 m 高度目標；遙控器更新目標的權限預設關閉。** 先置中四個方向（包含油門），按 `G` 開放遙控輸入。

| 操作角色 | 本程式的效果 |
|---|---|
| 右搖桿前後 | 改變世界座標 Y 方向的目標位置 |
| 右搖桿左右 | 改變世界座標 X 方向的目標位置 |
| 左搖桿上下 | 改變世界座標 Z 方向的目標高度 |
| 左搖桿左右 | 改變目標機頭朝向 |

這是**世界座標模式**：右是 +X、前是 +Y、上是 +Z。不隨機頭或相機方向旋轉。MuJoCo 相機轉了之後，螢幕左右不一定等於世界 X 方向，這不是軸映射自動反向。

搖桿幅度決定目標移動速度，不是直接的姿態角或電機油門。回中後，目標速度按設定的加速度限制減到零，接著保持新目標；飛機仍可能短暫追趕與制動。

### 按鍵

`G` 開／關遙控器對目標的更新；從關變開必須先置中。`H` 立即停止目標更新，凍結目前的**目標點**，飛機仍由 SE3 追到它。`R` 重置仿真到 XML 的初始狀態與 0.30 m 目標，並關閉遙控輸入。`P` 暫停／恢復仿真；恢復後須再次按 `G` 開放遙控輸入。`Esc` 結束程式。

按鍵可從 pygame 面板或 MuJoCo 視窗接收。`G` 不是實機解鎖；本程式完全沒有把指令送到真機的通訊接口。

### 斷線行為與限制

當 pygame 回報該裝置移除或讀取失敗時，程式立即凍結目標、停止接受遙控器更新。重新連線後需要重啟程式，不會自動熱重連。這樣可以避免裝置重新編號後誤讀其他手柄。

但是，**如果作業系統／驅動沒有回報斷線，反而持續提供最後的軸值，單靠一般 pygame 軸讀值無法保證識別這種靜默失聯。** 不以「搖桿沒有變動」作為斷線證據，因為使用者也可能有意保持固定幅度。發現異常時按 `H`、`P` 或關閉程式。這只是仿真輸入保護，不是真機失控保護方案。

## 6. 第一版預設限制

在 `settings.json` 修改目標生成器設定：

```json
{
  "start_position": [0.0, 0.0, 0.3],
  "minimum_position": [-1.5, -1.5, 0.15],
  "maximum_position": [1.5, 1.5, 1.5],
  "horizontal_speed": 0.4,
  "vertical_speed": 0.25,
  "acceleration": 1.0,
  "yaw_speed_deg": 45.0,
  "yaw_acceleration_deg": 120.0
}
```

距離單位 m、速度 m/s、線加速度 m/s²，偏航參數分別是 deg/s 與 deg/s²。水平對角輸入會限幅，避免同時推 X/Y 時速度變成單軸的 √2 倍。

**這些是目標位置、目標移動速度的限制，不是對實際飛機位置／速度的硬約束。** 飛機可能超調，也沒有避障功能。第一輪不要提高速度，不要加入碰撞、不同機型或傳感器失效。

## 7. 程式如何接到作者的控制器

```text
pygame 原始軸值
  → 校準、反向、中心死區
  → 目標位置變化速度與目標偏航速度
  → 限速／加速度限制／仿真步長積分
  → 目標位置 + 目標機頭方向
  → 作者 SE3Controller.control_update(...)
  → 作者 Mixer.calculate(...)
  → 四路歸一化 MuJoCo actuator 指令
  → MuJoCo 動力學 → 目前狀態
```

`upstream_adapter.py` 匯入作者的 `se3_controller.py`、`geometry.py`、`motor_mixer.py`，但不匯入會啟動另一個視窗的 `main.py`。保持作者示例的增益 `kx=0.6, kv=0.4, kR=6.0, kw=1.0`、33 g 控制器質量常數與 0.001 力矩縮放，控制／物理步長採 0.001 秒。

重要介面細節：

- 作者控制器使用 `xyzw` 四元數，MuJoCo 自由關節位置向量中使用 `wxyz`；適配器會轉序。
- 作者用機體 **+Y** 當 `forward`。本程式初始 `forward=[0,1,0]`，避免把另一種 +X 機頭慣例混進去；這個初始朝向與作者固定目標範例 `[1,0,0]` 不同，是明確的初值選擇。
- 本程式沿用原示例的**位置點追蹤**：傳給作者 `State` 的 `goal_vel` 與 `goal_omega` 保持零。搖桿產生的速度只用於積分目標；沒有直接送到作者非標準的目標速度附加項中。
- 本起步版讀取 MuJoCo 自由關節的真實位置、速度、四元數與機體角速度，未加入定位／狀態估計。這與真機的視覺或 IMU 完整閉環不是同一驗證範圍。
- 外部迴圈每個物理步計算一次電機控制量，然後 `mj_step`。不在渲染幀或多次 RK4 控制回調中積分搖桿目標，避免目標移動速度取決於渲染速度／回調次數。
- 多加一個總推力上限，避免作者混控器在集體推力極限附近的數值邊界；並對非有限結果報錯停止。這是適配層保護，沒有暗中修改作者檔案。它也不是保證原混控器在所有飽和狀態下都穩定。

實際機型與作者模型不同時，不能只替換外觀 XML 就沿用這些參數。此包先只針對公開示例模型的互動復現。

## 8. 檔案說明

| 檔案 | 用途 |
|---|---|
| `radio_se3.py` | `demo / probe / calibrate / fly` 四種入口，圖形迴圈與 CSV 日誌 |
| `core.py` | 軸校準、死區、目標生成器、輸入開關；不依賴 pygame/MuJoCo |
| `pygame_io.py` | 裝置檢測、逐軸校準與輸入監視視窗 |
| `upstream_adapter.py` | 調用作者控制器與混控器，讀寫 MuJoCo 狀態與電機 |
| `fetch_upstream.py` | 下載／匯入作者完整專案，記錄來源，不覆寫已有目錄 |
| `settings.json` | 目標速度、加速度與位置範圍 |
| `requirements.txt` | 選定依賴版本 |
| `tests/` | 39 項單元／介面契約測試 |
| `TEST_REPORT.md` | 實際已做與未做的驗證範圍 |

`upstream/`、`radio.json`、`logs/` 都是在你執行相關命令後才產生。

## 9. 常見問題

**`py -3.11` 找不到 Python**：先確認有安裝 Python 3.11 64-bit；或用該 Python 的完整路徑建立 `.venv`。不要把多套 Python 的套件混裝在一起。

**`Detected 0 joystick device(s)`**：確認系統辨認到的是遊戲控制器，而不只是藍牙配對項目。先重啟 `probe`；不用修改 SE3 控制增益。實際 Bluetooth 模式與韌體選項以你的遙控器介面為準，本文件沒有猜測特定機型的菜單位置。

**數值在 MuJoCo 視窗獲得焦點後停止更新**：本程式已在 pygame 初始化前設定 `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1`，並持續處理事件；但仍應在本機確認驅動行為。先把焦點切回輸入面板，看值是否恢復。

**持續下降或上升**：先看 `up` 校準值是否為零；非自回中油門需要手動放中間。這個模式的最低油門不是保持高度。

**`G` 按了沒反應**：看面板提示。四軸必須位於校準死區內；暫停時不能開放輸入；已偵測斷線的會話必須重啟。

**軸映射反了**：重跑校準，按提示指定的正／反方向擷取。若只是相機旋轉後看起來反了，先確認世界座標方向。

**`scene.xml`、mesh、texture 找不到**：確認下載的是完整作者專案；不要把 `crazyfile` 拼字改成另一個名稱。啟動使用腳本相對路徑，不需要切到作者資料夾。

**MuJoCo 視窗無法開啟**：先執行 `demo --headless` 區分渲染問題與控制問題。虛擬機、遠端桌面與圖形驅動差異要在本機確認。

**macOS**：官方 passive viewer 要用 `mjpython` 啟動，例如 `.venv/bin/mjpython radio_se3.py demo`。這份起步包的主要操作步驟以 Windows 編寫，pygame 與 MuJoCo 雙視窗在 macOS 的行為未驗證。

**仿真不穩、混控器報數值錯誤**：先保存完整錯誤與 `logs/*.source.json`，恢復原始模型和預設目標速度。原始示例的模型、混控器與控制器有自己的假設；本包不會偷偷用另一套控制器取代它來製造「通過」。

## 10. 測試命令

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe radio_se3.py demo --headless --seconds 24
```

第一行測試輸入與適配器介面邏輯；第二行才是本機 MuJoCo 動力學整合測試。它們不能互相替代。CSV 記錄的是數據快照，不是會隨後續仿真改變的 MuJoCo 陣列引用。

## 來源

以下為檢查介面與 API 時使用的主要來源；本包新增程式與說明不代表作者對它提供支持或驗證。

- 作者專案：`https://github.com/LGQWakkk/Quadrotor_SE3_Control`
- 原入口：`https://github.com/LGQWakkk/Quadrotor_SE3_Control/blob/main/main.py`
- SE3 介面：`https://github.com/LGQWakkk/Quadrotor_SE3_Control/blob/main/se3_controller.py`
- 混控器：`https://github.com/LGQWakkk/Quadrotor_SE3_Control/blob/main/motor_mixer.py`
- pygame joystick：`https://www.pygame.org/docs/ref/joystick.html`
- pygame event：`https://www.pygame.org/docs/ref/event.html`
- MuJoCo Python／passive viewer：`https://mujoco.readthedocs.io/en/stable/python.html`
- 選定 MuJoCo 版本：`https://pypi.org/project/mujoco/3.3.7/`
- 選定 pygame 版本：`https://pypi.org/project/pygame/2.6.1/`

第三方原始程式與模型的授權依各自來源文件為準；本壓縮包不重新打包它們。
