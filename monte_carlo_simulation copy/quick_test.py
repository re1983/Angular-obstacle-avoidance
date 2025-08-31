"""
手動測試腳本
"""
from monte_carlo_simulation.cleaned_wrapper_v2 import run_collision_avoidance_simulation

# 運行單次模擬
result = run_collision_avoidance_simulation(
    ownship_position=[0.0, 0.0, 0.0],
    ownship_velocity=1.0,
    ship_a_position=[-16.249, -51.255, 0.000],
    ship_a_velocity=2.849,
    ship_a_heading=52.861,
    goal_position=[50.0, 0.0, 0.0],
    show_plot=True  # 顯示圖表
)

print(f"結果: {result['outcome']}")
print(f"最小距離: {result['min_distance']:.3f}m")
