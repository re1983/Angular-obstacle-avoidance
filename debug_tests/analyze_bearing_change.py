#!/usr/bin/env python3
"""
Analyze the absolute bearing change and turning logic
"""
import numpy as np

def analyze_bearing_change():
    """Analyze how absolute bearing changes when ship moves from west to east"""
    
    print("=== 分析Ship A從西向東移動時的absolute bearing變化 ===")
    print()
    
    # Ownship at origin, heading north
    ownship_pos = np.array([0, 0])
    ownship_heading = 0  # North
    
    # Ship A positions as it moves from west to east in front of ownship
    positions = [
        [-5, 5],   # West of ownship, ahead
        [0, 5],    # Directly ahead
        [5, 5],    # East of ownship, ahead
    ]
    
    print("Ownship位置: (0, 0), 航向: 0° (北)")
    print("Ship A移動路徑: 從西向東通過Ownship前方")
    print()
    
    absolute_bearings = []
    for i, pos in enumerate(positions):
        # Calculate absolute bearing
        delta_pos = np.array(pos) - ownship_pos
        theta = np.arctan2(delta_pos[1], delta_pos[0])
        absolute_bearing = np.degrees(theta) % 360
        absolute_bearings.append(absolute_bearing)
        
        print(f"時刻 {i+1}: Ship A位置 {pos}")
        print(f"  絕對bearing: {absolute_bearing:.1f}°")
        
        # Calculate relative bearing
        relative_bearing = (absolute_bearing - ownship_heading) % 360
        if relative_bearing > 180:
            relative_bearing -= 360
        print(f"  相對bearing: {relative_bearing:.1f}°")
        print()
    
    # Calculate bearing rate
    print("=== Absolute Bearing變化率分析 ===")
    for i in range(len(absolute_bearings) - 1):
        bearing_diff = absolute_bearings[i+1] - absolute_bearings[i]
        # Handle 360° wrap
        if bearing_diff > 180:
            bearing_diff -= 360
        elif bearing_diff < -180:
            bearing_diff += 360
            
        print(f"時刻 {i+1} → {i+2}: Δ絕對bearing = {bearing_diff:.1f}°")
        
        if bearing_diff > 0:
            print("  絕對bearing增加 → absolute_bearings_difference > 0")
        else:
            print("  絕對bearing減少 → absolute_bearings_difference < 0")
        print()
    
    print("=== 轉向邏輯分析 ===")
    print("如果Ship A從西向東通過前方:")
    print("- 絕對bearing從 135° → 45° (變化 -90°)")
    print("- absolute_bearings_difference < 0")
    print("- 前方扇區邏輯: rate_of_turn = -1 * np.sign(absolute_bearings_difference) * gain")
    print("- rate_of_turn = -1 * (-1) * gain = +gain (右轉)")
    print()
    print("但您期望左轉，這表示邏輯可能有問題...")

if __name__ == "__main__":
    analyze_bearing_change()
