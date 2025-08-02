#!/usr/bin/env python3
"""
重新分析船隻行為 - Goal和Ship A位置不同
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading, get_distance_3d

def reanalyze_behavior():
    """重新分析船隻行為"""
    
    print("=== 重新分析船隻行為 ===")
    print()
    
    # 當前設置（按照代碼中的實際值）
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 20, 0])
    
    print("正確的設置分析:")
    print(f"Ownship: {ownship.position} heading={ownship.heading}° (朝北)")
    print(f"Ship A: {ship_a.position} heading={ship_a.heading}° (朝南)")
    print(f"Goal: {goal.position} (靜止目標點)")
    print()
    
    print("初始幾何關係:")
    rel_bearing_ship = get_bearing(ownship, ship_a)
    abs_bearing_ship = get_absolute_bearing(ownship, ship_a)
    rel_bearing_goal = get_bearing(ownship, goal)
    abs_bearing_goal = get_absolute_bearing(ownship, goal)
    dist_ship = get_distance_3d(ownship.position, ship_a.position)
    dist_goal = get_distance_3d(ownship.position, goal.position)
    
    print(f"到Ship A: 相對bearing={rel_bearing_ship:.1f}°, 絕對bearing={abs_bearing_ship:.1f}°, 距離={dist_ship:.1f}m")
    print(f"到Goal: 相對bearing={rel_bearing_goal:.1f}°, 絕對bearing={abs_bearing_goal:.1f}°, 距離={dist_goal:.1f}m")
    print()
    
    print("Ship A在正東(0°)，Goal在正北(90°)")
    print("這是一個對開情況: Ownship朝北，Ship A朝南")
    print()
    
    # 模擬幾步看行為
    print("=== 前10步行為分析 ===")
    dt = 0.01
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    for step in range(10):
        # 當前狀態
        abs_bearing = get_absolute_bearing(ownship, ship_a)
        angular_size = get_angular_diameter(ownship, ship_a)
        distance = get_distance_3d(ownship.position, ship_a.position)
        
        absolute_bearings.append(abs_bearing)
        angular_sizes.append(angular_size)
        
        if step > 0:
            bearing_diff = abs_bearing - absolute_bearings[step-1]
            if bearing_diff > 180:
                bearing_diff -= 360
            elif bearing_diff < -180:
                bearing_diff += 360
            absolute_bearings_difference.append(bearing_diff / dt)
        
        # 避障決策
        rate_of_turn, velocity = adj_ownship_heading(
            absolute_bearings, absolute_bearings_difference, angular_sizes,
            ownship, goal, ship_a, dt
        )
        
        print(f"步驟 {step}:")
        print(f"  Ownship位置: ({ownship.position[0]:.2f}, {ownship.position[1]:.2f})")
        print(f"  Ship A位置: ({ship_a.position[0]:.2f}, {ship_a.position[1]:.2f})")
        print(f"  距離Ship A: {distance:.2f}m")
        print(f"  角直徑: {angular_size:.2f}°")
        if len(absolute_bearings_difference) > 0:
            print(f"  Bearing變化率: {absolute_bearings_difference[-1]:.2f}°/s")
        print(f"  決策: rate_of_turn={rate_of_turn:.2f}°/s")
        
        # 檢查是否觸發CBDR
        if len(absolute_bearings_difference) >= 1:
            cbdr_condition = (abs(absolute_bearings_difference[-1]*dt) <= angular_sizes[-1] and 
                             angular_sizes[-1] > 2.0 and 
                             distance < 25.0)
            print(f"  CBDR條件: {cbdr_condition}")
            if not cbdr_condition:
                goal_bearing = get_bearing(ownship, goal)
                print(f"  → 目標導航模式, 到Goal的bearing={goal_bearing:.1f}°")
        print()
        
        # 更新位置
        ownship.rate_of_turn = rate_of_turn
        ownship.heading += ownship.rate_of_turn * dt
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt
    
    print("=== 結論 ===")
    print("如果船隻仍然行為異常，可能的原因:")
    print("1. CBDR條件檢查過於寬鬆")
    print("2. 目標導航邏輯的實現問題")
    print("3. 船隻位置更新有問題")

if __name__ == "__main__":
    reanalyze_behavior()
