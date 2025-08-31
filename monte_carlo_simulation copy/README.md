# Monte Carlo Collision Avoidance Simulation

這個模組提供了船舶碰撞避免系統的蒙地卡羅模擬工具，可以評估絕對方位角和相對方位角控制方法的性能。

## 功能特色

- **模組化設計**: 核心模擬、幾何計算、分析和可視化分離
- **隨機場景生成**: 自動產生多樣化的碰撞場景
- **雙控制方法**: 支援絕對和相對方位角控制策略
- **統計分析**: 全面的性能指標和敏感性分析
- **可視化**: 自動生成結果圖表和軌跡樣本
- **可配置參數**: 所有模擬參數均可在配置文件中調整

## 文件結構

```
monte_carlo_simulation/
├── __init__.py                # 包初始化
├── config.py                  # 配置參數
├── simulation_core.py         # 核心模擬函數
├── collision_geometry.py      # 碰撞幾何計算
├── monte_carlo_runner.py      # 蒙地卡羅模擬控制器
├── analysis.py               # 結果分析和可視化
├── run_simulation.py         # 主執行腳本
└── README.md                 # 說明文件
```

## 快速開始

### 1. 基本使用

運行預設的蒙地卡羅模擬：

```python
from monte_carlo_simulation.monte_carlo_runner import MonteCarloRunner
from monte_carlo_simulation.analysis import analyze_results, plot_results_overview

# 創建模擬器
runner = MonteCarloRunner()

# 運行模擬
results = runner.run_simulation()

# 分析結果
analysis = analyze_results(results)
plot_results_overview(results)
```

### 2. 命令行使用

```bash
# 運行預設模擬
python run_simulation.py

# 自定義參數
python run_simulation.py --num_simulations 500 --use_absolute --seed 42

# 比較研究（絕對 vs 相對方位角控制）
python run_simulation.py compare
```

### 3. 可用參數

- `--num_simulations`: 模擬次數 (預設: 1000)
- `--use_absolute`: 使用絕對方位角控制
- `--use_relative`: 使用相對方位角控制
- `--seed`: 隨機種子
- `--output_dir`: 輸出目錄
- `--no_plots`: 跳過圖表生成
- `--no_report`: 跳過報告生成

## 配置參數

主要配置參數在 `config.py` 中：

### 模擬參數
- `NUM_SIMULATIONS`: 模擬次數
- `DELTA_TIME`: 時間步長
- `TIME_STEPS`: 最大模擬步數

### 船舶參數
- `OWNSHIP_*`: 本船參數（位置、速度、尺寸等）
- `SHIP_A_*_RANGE`: Ship A的隨機化參數範圍
- `GOAL_POSITION`: 目標位置

### 碰撞幾何
- `COLLISION_ZONE_*_RATIO`: 碰撞區域（路徑的40%-60%）
- `*_EXCLUSION_DISTANCE`: 排除區域設定
- `SHIP_A_*_SPAWN_DISTANCE`: Ship A生成距離限制

### 安全參數
- `ALPHA_NAV`: 導航閾值
- `MIN_SAFE_DISTANCE`: 最小安全距離
- `COLLISION_THRESHOLD`: 碰撞閾值

## 模擬邏輯

### 1. 場景生成
1. 在本船路徑的40%-60%位置隨機選擇碰撞點
2. 隨機生成Ship A的參數（尺寸、速度、方位角）
3. 根據碰撞點和Ship A參數反推Ship A的起始位置
4. 驗證場景的幾何合理性

### 2. 模擬執行
1. 初始化兩艘船的狀態
2. 在每個時間步：
   - 計算方位角和角直徑
   - 應用避碰控制邏輯
   - 更新船舶位置
   - 檢查碰撞和目標到達
3. 記錄完整的模擬軌跡

### 3. 結果評估
- 任務成功率（到達目標且避免碰撞）
- 碰撞避免率
- 最小距離統計
- 路徑效率
- 參數敏感性分析

## 輸出結果

### 1. 數據文件
- `simulation_results_*.npz`: 原始模擬數據
- `report_*.txt`: 詳細文字報告

### 2. 可視化
- `overview_*.png`: 結果總覽圖表
- `trajectories_*.png`: 軌跡樣本圖

### 3. 統計指標
- 成功率統計
- 距離分佈分析
- 參數影響分析
- 失敗案例分析

## 依賴項

### 必需依賴
- numpy
- matplotlib

### 可選依賴
- tqdm (進度條顯示)
- seaborn (增強可視化)

如果沒有安裝可選依賴，系統會自動使用簡化版本。

## 應用場景

1. **算法評估**: 比較不同控制策略的性能
2. **參數調優**: 分析參數對系統性能的影響
3. **安全評估**: 評估在各種條件下的碰撞風險
4. **系統設計**: 為實際系統設計提供數據支持

## 擴展性

模組化設計允許輕鬆擴展：

- 添加新的控制算法到 `simulation_core.py`
- 修改場景生成邏輯在 `collision_geometry.py`
- 增加分析方法到 `analysis.py`
- 調整參數在 `config.py`

## 注意事項

1. 大量模擬可能需要較長時間，建議先用小樣本測試
2. 確保參數範圍合理，避免產生無效場景
3. 結果的可重現性依賴於隨機種子設定
4. 可視化功能需要 matplotlib 支持

## 技術細節

### 座標系統
使用NED座標系統：
- North (X+): 北方向
- East (Y+): 東方向  
- Down (Z+): 向下方向

### 方位角定義
- 絕對方位角: 相對於北方的真方位角
- 相對方位角: 相對於本船船頭方向的方位角
  - 正值: 目標在右舷（starboard）
  - 負值: 目標在左舷（port）

### CBDR邏輯
碰撞避免基於CBDR（Constant Bearing Decreasing Range）原理：
- 方位角變化率 ≈ 0: 真正的CBDR情況，執行規避動作
- 方位角變化率 ≠ 0: 根據目標位置和方位角變化調整轉向
