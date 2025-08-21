# Lyapunov 與 Barrier 分析（Angle-Only CBDR 控制）

本文給出以角度量測（bearing 與 angular diameter）為主的 CBDR 避碰控制之穩定性/有界性論證與驗證方式。重點：
- 有界（boundedness）：狀態不發散。
- 終極有界（ultimate boundedness, UUB）：有限時間後進入並停留在有界集合。
- 安全（forward invariance）：若初始距離在安全集合內，控制使其前向不變或實用不變。

---

## 1) 假設與記號

- 平面運動（NED, z=0）。
- 自船：航向 ψ（deg），位置 p_o=(x_o,y_o)，速度 v_o，轉率 u（deg/s），|u|≤u_max。
- 目標：位置 p_c=(x_c,y_c)，速度 v_c（可常值或緩變），幾何直徑 D>0。
- 相對量：Δp=p_c−p_o，距離 R=||Δp||。
- 絕對方位 θ=atan2(Δy,Δx)∈[0,360)；相對方位 β=wrap(θ−ψ)∈(−180,180]。
- 角徑 α=2 arctan(D/(2R))（deg）。分析時以弧度 α_rad 便於微分，但實作保持度數。
- 程式中的導航門檻 α_nav=1.0°（`ALPHA_NAV`）。

備註：R 與 α 單調對應：R = D/(2 tan(α/2))。

---

## 2) 控制摘要（與程式一致）

- CBDR 偵測：以絕對或相對方位率 r（θ̇ 或 β̇）判斷 |r|Δt ≤ α。
- 若近似 CBDR 且 β<0（左舷）→ u=−u_max；β≥0（右舷）→ u=+u_max（強制破壞 CBDR）。
- 否則（非 CBDR 區）：以增益 g=α^2，用 u = ±sgn(r)·g，且依 |β|<90° 或 ≥90° 決定號誌以加速幾何分離。
- 若 α<α_nav：回到導航（u=目標相對方位），整體受限於飽和與速度上界。

---

## 3) 候選 Lyapunov/Barrier 函數

- 安全 Barrier：選 h(R)=R−R_safe，安全集合 S={h≥0}。可令 B_R(R)=1/h（h>0 有限），或以 α 表示之 B_α(α)=1/(D/(2 tan(α/2))−R_safe)。
- 幾何 Lyapunov：L(β)=1−cos β ∈[0,2]，於 β=0 最小。其導數 L̇=sin β·β̇。
- 複合函數：V(R,β)=w1·B_R(R)+w2·L(β)，w1,w2>0。

直觀：接近時（R↓、α↑），Barrier B_R 放大，控制以最大轉率與 g=α^2 破壞 CBDR；離開威脅（α<α_nav）後，V 不再上升且狀態受限於輸入飽和，故有界。

---

## 4) 關鍵不等式（概要）

在威脅區（α≥α_nav）且 Δt 足夠小，存在 c1,c2>0 與小 ε>0，使得：
- L̇ ≤ − c1·α^2·|sin β|/R + ε。
- 觸發 CBDR 分支時（|r|Δt ≤ α 且 |r|≈0），u=±u_max 於下一步令 |r| 增大，使 L̇ 更負。
- Ḃ_R = −ḣ/h^2。若幾何與控制使 ḣ 不致過度負向（接近時 α^2 主導），則 B_R 不爆增且 h 不穿越 0。

結論：
- 初值滿足 R(0)>R_safe 時，S 為實用前向不變（practical forward invariant），且系統實用終極有界（UUB）：存在 δ>0, T>0，使 t≥T 時 R(t)≥R_safe−δ，β 有界。離散/不確定性減小時 δ→0。
- α<α_nav 時回到導航，u,v 皆有界，位置/航向/速度因此有界。

---

## 5) 數值驗證（本庫腳本）

使用 `debug_tests/verify_uub.py`：
- 同時檢驗「絕對方位率」與「相對方位率」控制。
- 計算距離最小值、是否低於 R_safe、Barrier B_R 的行為，以及 CBDR 觸發比例。

執行：

```bash
python3 debug_tests/verify_uub.py
```

可調整 `R_SAFE` 與初始條件做靈敏度分析，或加入多案例蒙地卡羅測試。

---

## 6) 後續工作

- 以 Filippov/混合系統方法對 CBDR 切換面做嚴謹證明。
- 導入離散型 Control Barrier Function 條件（近似 ḣ+κh≥0）以提升形式保證。
- 對雜訊與不確定性加入濾波與遲滯，減少切換抖動。
