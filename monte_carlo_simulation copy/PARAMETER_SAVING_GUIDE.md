# 蒙地卡羅模擬參數保存與重現系統使用說明

## 📋 概述

這個系統現在可以保存每個模擬的完整初始參數，讓您能夠：
1. **重現特定的模擬結果** - 使用相同的初始條件重新運行
2. **分析失敗案例** - 詳細研究導致碰撞的參數組合
3. **驗證系統行為** - 確保控制算法的一致性

## 📂 文件結構

運行模擬後，`results/` 資料夾將包含：

```
results/
├── successful/          # 成功避碰的案例
│   ├── 00001.txt       # 參數文件
│   ├── 00001.png       # 軌跡圖（可選）
│   ├── 00002.txt
│   ├── 00002.png
│   └── ...
├── collision/           # 發生碰撞的案例
│   ├── 00008.txt
│   ├── 00008.png
│   └── ...
├── timeout/             # 超時的案例（如果有）
│   └── ...
├── overview_*.png       # 總覽分析圖
├── trajectories_*.png   # 軌跡樣本圖
├── simulation_results_*.npz  # 原始數據
└── report_*.txt         # 統計報告
```

## 📄 參數文件格式

每個 `.txt` 文件包含：

### 1. 船隻初始參數
- **Ownship**: 位置、速度、航向、轉向率、尺寸
- **Ship A**: 位置、速度、航向、轉向率、尺寸、速度限制
- **Goal**: 目標位置

### 2. 碰撞情景參數
- 計劃碰撞點位置
- 碰撞區域比例（路徑的40%-60%）
- 到碰撞點的距離

### 3. 模擬配置
- 控制方法（絕對/相對方位）
- 時間步長和最大步數
- 安全閾值設定

### 4. 結果摘要
- 最終結果（成功/碰撞/超時）
- 最小距離、模擬時間
- 最終船隻位置

### 5. 重現參數
- 隨機種子
- 模擬索引
- 情景ID

## 🔄 如何重現模擬

### 方法1：使用重現腳本
```bash
python reproduce_simulation.py
```

### 方法2：程式化重現
```python
from reproduce_simulation import reproduce_simulation

# 重現特定案例
result = reproduce_simulation('results/collision/00008.txt')
print(f"重現結果: {result['simulation_outcome']}")
```

### 方法3：手動重現
```python
from monte_carlo_simulation.simulation_core import ShipStatus, run_single_simulation

# 根據參數文件手動設置初始條件
ownship = ShipStatus(name="Ownship", velocity=1.0, ...)
ship_a = ShipStatus(name="Ship A", velocity=2.786, heading=120.399, ...)
goal = ShipStatus(name="Goal", position=[50, 0, 0], ...)

# 運行模擬
result = run_single_simulation(ownship, ship_a, goal, use_absolute_bearings=True)
```

## ⚙️ 配置選項

在 `config.py` 中調整保存選項：

```python
# 個別軌跡保存選項
SAVE_INDIVIDUAL_TRAJECTORIES = True   # 保存軌跡圖像
SAVE_INDIVIDUAL_PARAMETERS = True     # 保存參數文件
SHOW_INDIVIDUAL_PLOTS = False         # 是否顯示圖像（False=只保存）

# 模擬數量
NUM_SIMULATIONS = 10  # 增加數量以獲得更多樣的結果
```

## 📊 分析建議

### 分析碰撞案例
```bash
# 檢查所有碰撞案例的參數
ls results/collision/*.txt

# 分析特定碰撞案例
cat results/collision/00008.txt
```

### 比較成功與失敗案例
```python
# 比較船隻參數分佈
collision_params = []
success_params = []

for f in glob.glob('results/collision/*.txt'):
    params = parse_parameter_file(f)
    collision_params.append(params)

for f in glob.glob('results/successful/*.txt'):
    params = parse_parameter_file(f) 
    success_params.append(params)

# 統計分析
collision_velocities = [p['ship_a_velocity'] for p in collision_params]
success_velocities = [p['ship_a_velocity'] for p in success_params]
```

## 🧪 測試不同控制方法

```python
# 測試相對方位控制
from monte_carlo_simulation.monte_carlo_runner import MonteCarloRunner

runner = MonteCarloRunner(
    num_simulations=10,
    use_absolute_bearings=False,  # 使用相對方位
    random_seed=42
)
results = runner.run_simulation()
```

## 🔍 故障排除

1. **編碼問題**: 確保使用UTF-8編碼讀取參數文件
2. **路徑問題**: 使用絕對路徑或確認工作目錄
3. **重現不一致**: 檢查隨機種子是否正確設置

## 📈 進階分析

使用保存的參數可以進行：
- **敏感性分析**: 哪些參數最影響碰撞風險
- **臨界條件研究**: 成功與失敗的邊界條件
- **控制性能比較**: 絕對vs相對方位控制效果
- **統計模型建立**: 預測碰撞風險的機器學習模型
