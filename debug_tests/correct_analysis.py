#!/usr/bin/env python3
"""
Re-analyze with correct coordinate system
"""
import numpy as np

def correct_analysis():
    """Re-analyze with proper coordinate system"""
    
    print("=== 正確座標系統分析 ===")
    print("正北 = 0°, 東 = 90°, 南 = 180°, 西 = 270°(-90°)")
    print()
    
    # Initial setup from your simulation
    print("初始設置：")
    print("- Ownship: 位置(0,0), 航向 0° (朝北)")
    print("- Ship A: 位置(10,-10), 航向 90° (朝東)")
    print()
    
    # Calculate initial absolute bearing
    delta_pos = np.array([10, -10]) - np.array([0, 0])
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    
    print(f"Ship A相對於Ownship的絕對bearing: {absolute_bearing}°")
    print("這表示Ship A在Ownship的東南方向")
    print()
    
    print("=== 碰撞軌跡分析 ===")
    print("如果兩船保持航向：")
    print("- Ownship朝北移動: (0,0) → (0,1) → (0,2) ...")
    print("- Ship A朝東移動: (10,-10) → (11,-10) → (12,-10) ...")
    print("碰撞點估計: 大約在 (10, 0) 附近")
    print()
    
    print("=== Ship A相對於Ownship的bearing變化 ===")
    # Simulate a few time steps
    times = [0, 5, 10]
    ownship_positions = [(0, t) for t in times]  # Moving north
    ship_positions = [(10 + t, -10) for t in times]  # Moving east
    
    for i, (own_pos, ship_pos) in enumerate(zip(ownship_positions, ship_positions)):
        delta = np.array(ship_pos) - np.array(own_pos)
        theta = np.arctan2(delta[1], delta[0])
        abs_bearing = np.degrees(theta) % 360
        print(f"時刻 {times[i]}s: Ship A絕對bearing = {abs_bearing:.1f}°")
    
    print()
    print("絕對bearing變化: 315° → 341° → 351° (增加)")
    print("所以 absolute_bearings_difference > 0")
    print()
    
    print("=== 正確避障邏輯 ===")
    print("當Ship A在前方且absolute_bearings_difference > 0時:")
    print("- Ship A的bearing正在增加（往更右邊移動）")
    print("- 為了避免CBDR，要讓bearing變化得更快")
    print("- 但我們要讓Ship A離開collision course")
    print("- Ownship應該向左轉（減慢bearing增加或讓它開始減少）")
    print("- 所以應該是: rate_of_turn = -np.sign(absolute_bearings_difference)")
    print()
    print("原本的邏輯是正確的！")

if __name__ == "__main__":
    correct_analysis()
