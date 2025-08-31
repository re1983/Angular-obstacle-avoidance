# Monte Carlo 船舶碰撞避免模擬 - 使用指南

## 完成的任務總結

✅ **已成功完成的功能：**

1. **模組化架構設計**
   - `simulation_core.py`: 從原始文件提取的核心模擬函數
   - `config.py`: 可配置的參數設定檔
   - `collision_geometry.py`: 智能碰撞場景生成系統
   - `monte_carlo_runner.py`: 蒙地卡羅模擬控制器
   - `analysis.py`: 結果分析和可視化工具
   - `demo.py`: 快速演示腳本

2. **智能場景生成**
   - 自動在Ownship路徑40%-60%位置隨機選擇碰撞點
   - 反推Ship A的合理起始位置
   - 隨機化Ship A的尺寸、速度、航向
   - 幾何有效性驗證（避免在起點/終點附近碰撞）

3. **雙控制方法支持**
   - 絕對方位角控制 (`adj_ownship_heading_absolute`)
   - 相對方位角控制 (`adj_ownship_heading_relative`)
   - 可在配置中切換或通過參數選擇

4. **統計分析功能**
   - 成功率統計（任務完成、碰撞避免、目標到達）
   - 距離分析（最小距離、危險遭遇、接近情況）
   - 參數敏感性分析
   - 失敗模式分析

5. **可視化功能**
   - 結果總覽圖表
   - 樣本軌跡繪製
   - 參數影響分析圖

## 快速開始

### 1. 基本演示
```bash
cd monte_carlo_simulation
python demo.py
```

### 2. 測試配置
```bash
python demo.py config    # 顯示當前配置
python demo.py test      # 測試場景生成
```

### 3. 完整模擬
```bash
python run_simulation.py --num_simulations 1000 --use_absolute
python run_simulation.py --num_simulations 1000 --use_relative
```

### 4. 比較研究
```bash
python run_simulation.py compare
```

## 核心特性

### 🎯 智能場景生成
系統會自動：
- 在Ownship路徑的40%-60%位置隨機選擇碰撞點
- 根據Ship A的速度和航向反推其起始位置
- 確保不在起點/終點附近發生碰撞
- 驗證場景的幾何合理性

### 📊 統計分析
每次模擬提供：
- 任務成功率（到達目標且避免碰撞）
- 碰撞避免率
- 最小距離統計
- 路徑效率分析
- 參數敏感性分析

### ⚙️ 可配置參數
在 `config.py` 中可調整：
- 模擬次數和時間參數
- Ship A的參數範圍（尺寸、速度、航向）
- 碰撞區域設定（40%-60%）
- 安全距離和排除區域

## 主要配置參數

```python
# 模擬參數
NUM_SIMULATIONS = 1000      # 模擬次數
USE_ABSOLUTE_BEARINGS = True # 控制方法選擇

# Ship A隨機化範圍
SHIP_A_SIZE_RANGE = [0.3, 1.0]      # 尺寸範圍 (m)
SHIP_A_VELOCITY_RANGE = [0.5, 3.0]  # 速度範圍 (m/s)
SHIP_A_HEADING_RANGE = [0.0, 360.0] # 航向範圍 (度)

# 碰撞幾何
COLLISION_ZONE_START_RATIO = 0.4  # 碰撞區域起始位置 (40%)
COLLISION_ZONE_END_RATIO = 0.6    # 碰撞區域結束位置 (60%)

# 安全參數
MIN_SAFE_DISTANCE = 2.0      # 最小安全距離 (m)
COLLISION_THRESHOLD = 1.0    # 碰撞判定距離 (m)
```

## 典型結果示例

運行50次模擬的演示結果：
```
Mission Success Rate: 0.0%      # 完整任務成功率
Collision Avoidance Rate: 90.0% # 碰撞避免率
Goal Reach Rate: 0.0%           # 目標到達率
Mean Minimum Distance: 6.23 ± 3.80 m # 平均最小距離
Dangerous Encounters: 5         # 危險遭遇次數
Close Calls: 2                  # 接近情況次數
```

## 輸出文件

模擬完成後會生成：
- `simulation_results_*.npz`: 原始數據（可用於後續分析）
- `overview_*.png`: 結果總覽圖表
- `trajectories_*.png`: 軌跡樣本圖
- `report_*.txt`: 詳細文字報告

## 應用場景

1. **算法性能比較**: 絕對 vs 相對方位角控制
2. **參數調優**: 找出最佳的系統參數組合
3. **風險評估**: 評估不同條件下的碰撞風險
4. **系統驗證**: 驗證避碰算法的可靠性

## 技術架構

### 模擬流程
1. **場景生成**: 隨機生成碰撞場景
2. **模擬執行**: 運行船舶動力學和控制邏輯
3. **結果評估**: 計算性能指標
4. **統計分析**: 生成統計報告和可視化

### 避碰邏輯
基於CBDR (Constant Bearing Decreasing Range) 原理：
- 當方位角變化率 ≈ 0 時，判定為碰撞威脅
- 根據目標相對位置執行規避動作
- 威脅解除後導航至目標點

## 擴展可能性

系統設計支持輕鬆擴展：
- 添加新的控制算法
- 修改場景生成策略
- 增加新的分析指標
- 支持多船舶場景
- 整合實際海況數據

## 注意事項

1. **計算時間**: 大量模擬需要較長時間，建議先小規模測試
2. **參數合理性**: 確保參數範圍設定合理，避免無效場景
3. **結果解讀**: 低成功率可能表示系統需要改進或參數需要調整
4. **隨機性**: 使用固定隨機種子可獲得可重現的結果

## 故障排除

### 常見問題
1. **場景生成失敗**: 檢查參數範圍是否合理
2. **成功率為0%**: 可能需要調整控制參數或安全距離
3. **模擬速度慢**: 減少模擬次數或時間步數

### 調試工具
```bash
python demo.py test      # 測試場景生成
python demo.py config    # 查看當前配置
```

這個系統現在已經完全準備好進行蒙地卡羅模擬分析了！
