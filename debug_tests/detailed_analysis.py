#!/usr/bin/env python3
"""
More detailed analysis of bearing change
"""
import numpy as np

def detailed_bearing_analysis():
    """More detailed bearing analysis"""
    
    print("=== 詳細bearing變化分析 ===")
    print()
    
    # Simulate several time steps
    times = [0, 1, 2, 3, 4, 5]
    ownship_positions = [(0, t) for t in times]  # Moving north
    ship_positions = [(10 + t, -10) for t in times]  # Moving east
    
    absolute_bearings = []
    
    for i, (own_pos, ship_pos) in enumerate(zip(ownship_positions, ship_positions)):
        delta = np.array(ship_pos) - np.array(own_pos)
        theta = np.arctan2(delta[1], delta[0])
        abs_bearing = np.degrees(theta) % 360
        absolute_bearings.append(abs_bearing)
        
        print(f"時刻 {times[i]}s:")
        print(f"  Ownship: {own_pos}")
        print(f"  Ship A: {ship_pos}")
        print(f"  絕對bearing: {abs_bearing:.2f}°")
        
        if i > 0:
            # Calculate bearing rate
            prev_bearing = absolute_bearings[i-1]
            current_bearing = abs_bearing
            
            # Handle 360° wrap
            bearing_diff = (current_bearing - prev_bearing) % 360
            if bearing_diff > 180:
                bearing_diff -= 360
            
            print(f"  Bearing變化: {bearing_diff:.2f}°/s")
            
            if bearing_diff > 0:
                print("  → bearing增加 (Ship A相對右移)")
            elif bearing_diff < 0:
                print("  → bearing減少 (Ship A相對左移)")
            else:
                print("  → bearing不變 (CBDR情況!)")
        print()
    
    print("=== 關鍵觀察 ===")
    print("在這個特殊案例中，由於兩船速度相同且垂直移動，")
    print("絕對bearing幾乎保持常數 → 這正是CBDR情況！")
    print()
    
    print("=== 正確避障策略 ===")
    print("當bearing保持常數時，需要主動打破這種平衡:")
    print("- Ship A在左舷(-45°)")
    print("- Ownship應該左轉，讓Ship A的bearing開始增加")
    print("- 這樣Ship A會從右舷通過，避免碰撞")
    print()
    
    print("=== 修正方案 ===")
    print("當absolute_bearings_difference ≈ 0時（CBDR），")
    print("應該根據相對bearing決定轉向:")
    print("- 如果Ship A在左舷 → 左轉")
    print("- 如果Ship A在右舷 → 右轉")

if __name__ == "__main__":
    detailed_bearing_analysis()
