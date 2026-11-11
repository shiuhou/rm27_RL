RM27_ASM2_COLLISION_PACKAGE 可攜交付包

直接使用
Isaac Lab 載入 usd/drone.usdc；usd/drone.usda 為可閱讀來源。兩者網格均內嵌。
MuJoCo 載入 mujoco/model.xml，必須保留同層 meshes/、collision_meshes/。
metadata/ 中各 mesh 欄位相對於 mujoco/，原始 CAD 路徑欄位只作來源紀錄。
metadata/parts.json 為 CAD 提取的中間資料，含已知異常的原始體積積分值；正式物理參數以 mass_properties.json 為準。
preview/ 包含兩張檢查圖。cad/asm2.STEP 是未移電池的原始裝配。
工具已改成從交付包定位檔案，不依賴原電腦附件路徑。

驗證現有模型（在包根目錄執行）
python tools/validate_collisions.py
注意：嚴格空隙測試有已知 0.0653 mm 邊緣誤擋，程式預期回傳 exit code 1；這不是全項 PASS 模型。
MUJOCO_GL=egl python tools/render_collisions.py

重新建立（會更新交付包檔案，先另複製一份再執行）
python tools/build.py
python tools/export.py
python tools/decompose.py
python tools/apply_collisions.py
python tools/validate_collisions.py
python tools/preview.py
MUJOCO_GL=egl python tools/render_collisions.py
CoACD 參數見 metadata/coacd_parameters.json 與 tools/decompose.py。
依賴 OCP、trimesh、numpy、mujoco、coacd、shapely、usd-core、scipy、networkx、rtree、matplotlib、Pillow。
本機使用 MuJoCo 3.12.0、CoACD 1.0.14；Isaac Lab 執行由使用端完成。

檔案完整性（在包根目錄執行）
sha256sum -c MANIFEST_SHA256.txt
MANIFEST_SHA256.txt 覆蓋除自身以外的全部交付檔案。重新建立後雜湊會改變。

以下保留模型設定與驗證說明；單一檔名請按上述子目錄定位。

asm2 無人機模型使用說明（電池配平＋分解碰撞版）

目前版本
整機質量 142 g。電池已移位，估算重心的水平位置對齊四顆馬達中心。
機體原點設於更新後的合成重心，單位為公尺。
機體座標由 CAD 幾何推定：+X 朝相機、+Y 向左、+Z 向上。

SolidWorks 同步調整
以本次提供的原始 asm2.STEP 為起點，只平移 battery。
原始 CAD 座標位移（mm）：X +0.0414463，Y 0，Z -2.1427339。
相當於向後（遠離相機）2.1427339 mm、機身左側 0.0414463 mm，高度保持不變。
若 SolidWorks 使用不同座標系，請按機身方向換算。
交付的 MJCF、網格與 USD 已包含此位移，使用時避免再次平移。
請在 SolidWorks 檢查裝配干涉，並以實物平衡確認最終配置。

MuJoCo
載入 model.xml；meshes/ 與 collision_meshes/ 必須保留在 model.xml 同層。
Python 範例：
    import mujoco
    model = mujoco.MjModel.from_xml_path('/你的路徑/asm2_delivery/model.xml')
外觀 geom 位於 group 2，已關閉碰撞；碰撞 geom 位於 group 3。
一般觀看可關閉 group 3，檢查碰撞時只開啟 group 3。各零件的重心 site 位於 group 4。
這是單一自由剛體模型；獨立載入時沒有地板，重力下會自由落體。

Isaac Lab
以 RigidObjectCfg 搭配 sim_utils.UsdFileCfg(usd_path=模型絕對路徑) 載入 drone.usdc。
模型為一個剛體，採公尺和 Z-up。/Drone/part_* 為外觀，/Drone/Collision/* 為凸分解碰撞。
USD 碰撞網格預設隱藏，但物理碰撞啟用。檢視碰撞時可開啟 Physics debug visualization。
已設定載板與撞觸片的 PhysX contactOffset 為 0.01 mm，其餘零件為 0.1 mm，restOffset 為 0，以及剛體 contact report threshold 0。這些設定仍需 Isaac Lab 實跑驗證。
drone.usda 是可閱讀的 USD 來源，drone.usdc 是二進位交付檔。
旋翼推力、阻力、電機動態和飛行控制器需要接入任務程式。

重量與重心資料
mass_properties.json：各實例重量、重心、慣量、整機慣量、來源雜湊及電池位移。
mass_centers.png：更新後的俯視與側視重量／重心標註。
已知零件共 137.5 g；線材與螺絲共 4.5 g，以點質量放在更新後的合成重心，固有慣量設為零。
四組馬達連槳各 6.75 g；兩個 project2 各 3 g。

精度與驗證狀態
rk3576_body 的 STEP 體積積分異常，且一個面無法三角化。其 25 g 採三角網格表面分布近似。
其餘零件採均勻 CAD 體積密度估算，重心和慣量仍需實物量測驗證。
槳保、支撐、槳葉、相機與頂蓋使用 CoACD 凸分解，保留主要空隙。
電池、飛控、核心板、燈與連接件使用緊貼外形的凸包，會填平這些零件的小安裝孔。
兩邊接觸材質的滑動摩擦採 0.6 工程預設值；模型表示剛性接觸，未標定結構柔性、緩衝或真實摩擦。
細部誤差與空隙測試結果見 collision_validation.json。
已通過 MuJoCo 載入、142 g 質量核對、100 步自由落體有限值檢查。
配平後，合成重心相對四馬達中心的 XY 殘差小於 1e-9 m（計算殘差）。
USD 已通過 OpenUSD 解析與 usdc 匯出；Isaac Lab 執行尚待驗證。

兩組指定撞觸面與 Scorer 接口
contact_front_* 為既有前方折線撞觸面，contact_rear_* 為既有後方直線撞觸面。
平面輪廓由 rk3576_body 最低處的 CAD 網格抽出，共 15 個凸片。
總平面面積約 228.094 mm²，底面 Z=-19.86282 mm（相對目前整機重心）。
接觸片從實際底面向內延伸 0.05 mm，一般載板碰撞在此高度裁切，以避免同區域重複接觸。
這 0.05 mm 層內的底部微小邊緣圓角省略；其他高度的機體碰撞保留。
MuJoCo 與 USD 共用 collision_geometry.json 的網格、source_part、group 及名稱。
contact_front / contact_rear 表示指定面，body 表示其他機體碰撞。任務是否合法由 Scorer 根據目標、位置、法線等條件判定。
MuJoCo：contact_records.collect_contacts(model, data) 返回每個接觸的組別、雙方 geom、世界位置、指向無人機的法線、相對速度、力與單步衝量估計。
在需要取樣的狀態呼叫 mj_forward 後再 collect_contacts，可得到同一狀態下的接觸與求解力。衝量採接觸力乘物理步長，累積時每個物理步只記一次。
Isaac Lab：以 PhysX 接觸報告中的 collider prim 路徑對照 /Drone/Collision/contact_front_*、contact_rear_*，或讀取 prim customData 的 contact_group。
只取整個剛體合力無法區分兩條面；Isaac 的逐碰撞形狀事件接線與 Scorer 尚待任務端整合。

本機最終碰撞檢查
共 11,874 個碰撞片（含 15 個指定撞觸片）；全部 14 個零件實例都有碰撞。
底面平放能區分 contact_front 與 contact_rear；四個側向接觸測試均回報 body。
288 條槳保區域射線中，外觀網格有 86 條開放射線；碰撞網格保留其中 85 條。
剩下一條在頂蓋邊緣被凸片誤擋，該命中點到外觀網格最近距離約 0.0653 mm。嚴格的零誤擋檢查仍為 FAIL，collision_validation.json 保留 passed=false。
這是精細近似版本。實測 100 步單機接觸模擬約 0.023 s（不含載入），未測批量 RL 場景或 Isaac 執行效能。
collision_preview.png 由全部碰撞幾何渲染，藍色為前撞觸面，綠色為後撞觸面。
碰撞片數超過 MuJoCo Renderer 預設 10,000 上限，render_collisions.py 已擴大 max_geom；其他自訂 viewer 也需相應提高顯示上限。
USD 已內嵌網格，可單獨複製 drone.usdc 到另一台；MJCF 需連同 meshes/ 和 collision_meshes/ 複製。
PhysX 逐碰撞形狀接觸報告參考：
https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html
