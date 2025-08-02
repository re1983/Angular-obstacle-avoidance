#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from BearingRateGraph_cleaned import *
import numpy as np

def test_initial_behavior():
    """測試前10步的初始行為"""
    print("=== 測試初始10步行為 ===\n")
    
    # 設置初始條件（與主代碼相同）
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=-0.0, position=[0, 0, 0])
    ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 20, 0])
    
    delta_time = 0.01
    
    # 初始化tracking arrays
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    print(f"初始設置:")
    print(f"Ownship: {ownship.position} heading={ownship.heading}°")
    print(f"Ship A: {ship.position} heading={ship.heading}°")
    print(f"Goal: {goal.position}")
    print()
    
    for step in range(10):
        print(f"步驟 {step}:")
        
        # 記錄當前狀態
        absolute_bearing = get_absolute_bearing(ownship, ship)
        angular_size = get_angular_diameter(ownship, ship)
        
        absolute_bearings.append(absolute_bearing)
        angular_sizes.append(angular_size)
        
        # 計算bearing rate（如果有歷史數據）
        if len(absolute_bearings) > 1:
            bearing_rate = angle_difference_in_deg(absolute_bearings[-2], absolute_bearings[-1]) / delta_time
            absolute_bearings_difference.append(bearing_rate)
        
        print(f"  位置: Ownship({ownship.position[0]:.3f},{ownship.position[1]:.3f}), Ship A({ship.position[0]:.3f},{ship.position[1]:.3f})")
        print(f"  絕對bearing: {absolute_bearing:.2f}°, 角直徑: {angular_size:.2f}°")
        
        if len(absolute_bearings_difference) > 0:
            print(f"  Bearing rate: {absolute_bearings_difference[-1]:.2f}°/s")
            
            # 檢查CBDR條件
            current_distance = get_distance_3d(ownship.position, ship.position)
            cbdr_condition = (abs(absolute_bearings_difference[-1]) < 1.0 and 
                            angular_size > 2.0 and 
                            current_distance < 25.0)
            print(f"  CBDR檢查: rate={abs(absolute_bearings_difference[-1]):.2f}<1.0? {abs(absolute_bearings_difference[-1]) < 1.0}")
            print(f"  角直徑檢查: {angular_size:.2f}>2.0? {angular_size > 2.0}")
            print(f"  距離檢查: {current_distance:.2f}<25.0? {current_distance < 25.0}")
            print(f"  CBDR結果: {cbdr_condition}")
            
            # 調用控制函數
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(
                absolute_bearings, absolute_bearings_difference, angular_sizes, 
                ownship, goal, ship, delta_time)
        else:
            # 第一步：沒有bearing歷史，應該使用目標導航
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(
                absolute_bearings, absolute_bearings_difference, angular_sizes, 
                ownship, goal, ship, delta_time)
            print(f"  無bearing歷史，使用目標導航")
        
        print(f"  控制輸出: rate_of_turn={ownship.rate_of_turn:.2f}°/s, velocity={ownship.velocity:.2f}m/s")
        
        # 更新船隻狀態
        ownship.heading += ownship.rate_of_turn * delta_time
        ownship.heading = ownship.heading % 360  # 保持在0-360度範圍
        
        # 更新位置
        ownship.position[0] += ownship.velocity * np.sin(np.deg2rad(ownship.heading)) * delta_time
        ownship.position[1] += ownship.velocity * np.cos(np.deg2rad(ownship.heading)) * delta_time
        
        ship.position[0] += ship.velocity * np.sin(np.deg2rad(ship.heading)) * delta_time  
        ship.position[1] += ship.velocity * np.cos(np.deg2rad(ship.heading)) * delta_time
        
        print()
        
        # 如果檢測到轉向，停止測試
        if abs(ownship.rate_of_turn) > 0.1:
            print("*** 檢測到轉向動作！***")
            break
    
    # 總結
    print("=== 測試結論 ===")
    final_heading_change = abs(ownship.heading - 0.0)  # 初始heading是0
    if final_heading_change > 1.0:
        print(f"✓ 船隻改變了航向 {final_heading_change:.2f}°")
    else:
        print(f"✗ 船隻沒有明顯改變航向 (變化僅 {final_heading_change:.2f}°)")
    
    if abs(ownship.rate_of_turn) > 0.1:
        print(f"✓ 船隻正在執行轉向 {ownship.rate_of_turn:.2f}°/s")
    else:
        print(f"✗ 船隻沒有轉向動作")

if __name__ == "__main__":
    test_initial_behavior()
