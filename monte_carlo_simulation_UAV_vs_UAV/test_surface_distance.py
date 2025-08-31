"""測試表面距離計算"""
from BearingRateGraph_comparison import (
    run_single_simulation, 
    DEFAULT_OWNSHIP_CONFIG, 
    DEFAULT_SHIP_CONFIG, 
    DEFAULT_GOAL_CONFIG
)

# 運行一個簡單的測試
result = run_single_simulation(
    ownship_config=DEFAULT_OWNSHIP_CONFIG,
    ship_config=DEFAULT_SHIP_CONFIG,
    goal_config=DEFAULT_GOAL_CONFIG,
    time_steps=1000,
    delta_time=0.01,
    ALPHA_TRIG=2.0
)

print(f"表面距離範圍: {min(result['distances']):.3f}m 到 {max(result['distances']):.3f}m")
print(f"最小表面距離: {min(result['distances']):.3f}m")
print(f"物體大小: Ownship={result['ownship_size']:.1f}m, Ship A={result['ship_size']:.1f}m")
