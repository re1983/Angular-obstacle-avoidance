# Monte Carlo Collision Avoidance Simulation

這是一個船舶碰撞避免的蒙地卡羅模擬系統，用於分析在不同場景下的避撞性能。

## 📁 文件結構

```
monte_carlo_simulation/
├── monte_carlo_runner.py      # 主要的蒙地卡羅模擬執行器
├── analyze_results.py         # 結果分析和可視化工具
├── run_complete_analysis.py   # 完整分析管道
├── config.py                  # 配置參數
├── BearingRateGraph_comparison.py  # 核心模擬函數（已存在）
└── results/                   # 結果輸出目錄
    └── results_YYYYMMDD_HHMMSS/
        ├── successful/        # 成功避碰的案例
        │   ├── 00001.txt     # 參數文件
        │   ├── 00001.png     # 軌跡圖
        │   └── ...
        ├── collision/         # 發生碰撞的案例
        ├── timeout/           # 超時的案例
        ├── overview_*.png     # 總覽分析圖
        ├── trajectories_*.png # 軌跡樣本圖
        ├── simulation_results_*.npz  # 原始數據
        └── report_*.txt       # 統計報告
```

## 🚀 快速開始

### 1. 執行單次蒙地卡羅模擬

```bash
python monte_carlo_runner.py
```

### 2. 分析已有結果

```bash
python analyze_results.py
```

### 3. 執行完整分析管道

```bash
python run_complete_analysis.py
```

### 4. 創建批次執行文件（Windows）

```bash
python run_complete_analysis.py --create-batch
```

然後雙擊生成的 `run_analysis.bat` 文件即可執行完整分析。

## ⚙️ 配置參數

在 `config.py` 中可以調整以下參數：

### 模擬參數
- `NUM_SIMULATIONS`: 蒙地卡羅運行次數
- `MAX_TIME_STEPS`: 最大模擬步數
- `DELTA_TIME`: 時間步長
- `USE_ABSOLUTE_BEARINGS`: 使用絕對/相對方位控制

### Ship A 隨機參數範圍
- `SHIP_A_VELOCITY_RANGE`: 速度範圍 [min, max] m/s
- `SHIP_A_HEADING_RANGE`: 航向範圍 [min, max] degrees  
- `SHIP_A_SIZE_RANGE`: 船隻大小範圍 [min, max] m

### 碰撞幾何參數
- `COLLISION_ZONE_START_RATIO`: 碰撞區域起始比例
- `COLLISION_ZONE_END_RATIO`: 碰撞區域結束比例
- `SHIP_A_MIN_SPAWN_DISTANCE`: Ship A 最小生成距離
- `SHIP_A_MAX_SPAWN_DISTANCE`: Ship A 最大生成距離

## 📊 輸出說明

### 個別結果文件（*.txt）
每個模擬的詳細參數和結果，包括：
- Ship A 隨機生成的參數
- 碰撞預測信息
- 模擬執行結果（成功/碰撞/超時）

### 軌跡圖（*.png）
每個模擬的船隻軌跡可視化，顯示：
- Ownship（藍線）和Ship A（紅線）的運動軌跡
- 起始位置、目標位置
- 最小距離點

### 總覽分析圖（overview_*.png）
包含6個子圖的統計分析：
1. 結果分佈餅圖
2. 最小距離分佈直方圖
3. Ship A 速度分佈
4. Ship A 航向分佈
5. 碰撞位置分佈
6. 結果類型vs最小距離散點圖

### 軌跡樣本圖（trajectories_*.png）
顯示每種結果類型的代表性軌跡樣本

### 統計報告（report_*.txt）
包含完整的統計分析結果：
- 成功率、碰撞率、超時率
- 距離統計（平均值、標準差、最值）
- 成功案例的到達時間統計

### 原始數據（simulation_results_*.npz）
所有模擬的原始數據，可用於後續深度分析

## 🔧 自定義使用

### 修改參數範圍
編輯 `config.py` 中的參數範圍來測試不同的場景：

```python
# 例如：測試更高速度的船隻
SHIP_A_VELOCITY_RANGE = [2.0, 5.0]  # 更高速度範圍

# 例如：測試特定航向範圍
SHIP_A_HEADING_RANGE = [90.0, 270.0]  # 只測試側面接近
```

### 增加模擬數量
```python
NUM_SIMULATIONS = 100  # 增加到100次模擬以獲得更好的統計
```

### 改變控制方法
```python
USE_ABSOLUTE_BEARINGS = False  # 測試相對方位控制
```

## 📈 結果解讀

- **成功率**: 表示避撞控制器的有效性
- **最小距離**: 越大表示安全裕度越高
- **到達時間**: 表示路徑效率
- **參數分佈**: 顯示哪些參數組合容易導致問題

## 🎯 使用案例

1. **性能評估**: 評估避撞算法在不同場景下的成功率
2. **參數調優**: 找到最佳的控制參數設置
3. **安全分析**: 分析最小安全距離的分佈
4. **場景測試**: 測試特定類型的遭遇場景

## 🔍 故障排除

- **所有模擬都超時**: 增加 `MAX_TIME_STEPS` 或檢查避撞算法
- **成功率太低**: 調整 `ALPHA_NAV` 參數或檢查碰撞閾值
- **內存不足**: 減少 `NUM_SIMULATIONS` 或關閉軌跡圖儲存

---

系統已完成開發，可以開始大規模的蒙地卡羅分析！🎉
