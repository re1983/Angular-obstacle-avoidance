#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from BearingRateGraph_cleaned import *
import numpy as np

def test_cbdr_logic():
    """測試實際的CBDR邏輯"""
    print("=== 測試CBDR邏輯 ===\n")
    
    # 設置初始條件
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=-0.0, position=[0, 0, 0])
    ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 20, 0])
    
    # 模擬前10步
    delta_time = 0.01
    for step in range(10):
        print(f"\n--- 步驟 {step} ---")
        print(f"Ownship位置: ({ownship.position[0]:.2f}, {ownship.position[1]:.2f})")
        print(f"Ship A位置: ({ship.position[0]:.2f}, {ship.position[1]:.2f})")
        
        # 調用避障函數
        rate_of_turn, velocity = adj_ownship_heading(ownship, ship, goal)
        
        print(f"返回的控制: rate_of_turn={rate_of_turn:.2f}°/s, velocity={velocity:.2f}m/s")
        
        # 更新位置
        ownship.rate_of_turn = rate_of_turn
        ownship.velocity = velocity
        update_ship_state(ownship, delta_time)
        update_ship_state(ship, delta_time)
        
        # 檢查距離
        distance = get_distance_3d(ownship.position, ship.position)
        goal_distance = get_distance_3d(ownship.position, goal.position)
        print(f"到Ship A距離: {distance:.2f}m, 到Goal距離: {goal_distance:.2f}m")
        
        # 如果船隻開始轉向就停止測試
        if abs(rate_of_turn) > 0.1:
            print("檢測到轉向動作!")
            break
    
    print("\n=== 測試結論 ===")
    if abs(ownship.rate_of_turn) > 0.1:
        print("✓ 船隻正確執行避障動作")
    else:
        print("✗ 船隻沒有執行避障動作")

if __name__ == "__main__":
    test_cbdr_logic()
