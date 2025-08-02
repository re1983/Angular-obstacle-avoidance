#!/usr/bin/env python3
"""
詳細分析為什麼自船不轉向和不前往目的地的問題
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading

def debug_ship_behavior():
    """調試船隻行為問題"""
    
    print("=== 調試船隻行為問題 ===")
    print()
    
    # 當前設置
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    
    print("=== 問題1: 目的地設置檢查 ===")
    print(f"Ownship初始位置: {ownship.position}")
    print(f"Goal位置: {goal.position}")
    print(f"Ship A位置: {ship_a.position}")
    print(f"Goal與Ship A位置相同！這可能是問題所在")
    print()
    
    print("=== 問題2: 初始條件分析 ===")
    # 計算初始狀態
    rel_bearing_to_ship = get_bearing(ownship, ship_a)
    abs_bearing_to_ship = get_absolute_bearing(ownship, ship_a)
    angular_size = get_angular_diameter(ownship, ship_a)
    rel_bearing_to_goal = get_bearing(ownship, goal)
    
    print(f"相對bearing to Ship A: {rel_bearing_to_ship:.2f}° ({'正前方' if abs(rel_bearing_to_ship) < 1 else '左舷' if rel_bearing_to_ship < 0 else '右舷'})")
    print(f"絕對bearing to Ship A: {abs_bearing_to_ship:.2f}°")
    print(f"角直徑: {angular_size:.2f}°")
    print(f"相對bearing to Goal: {rel_bearing_to_goal:.2f}°")
    print()
    
    print("=== 問題3: 避障邏輯測試 ===")
    # 測試避障邏輯的各個條件
    absolute_bearings = [abs_bearing_to_ship]
    absolute_bearings_difference = []
    angular_sizes = [angular_size]
    
    # 第一次調用 (len(absolute_bearings_difference) == 0)
    print("第一次調用避障函數:")
    rate_of_turn1, velocity1 = adj_ownship_heading(
        absolute_bearings, absolute_bearings_difference, angular_sizes,
        ownship, goal, ship_a, 0.01
    )
    print(f"返回: rate_of_turn={rate_of_turn1:.4f}, velocity={velocity1:.2f}")
    
    # 模擬一步更新
    ownship.rate_of_turn = rate_of_turn1
    ownship.heading += ownship.rate_of_turn * 0.01
    ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * 0.01
    ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * 0.01
    
    ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * 0.01
    ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * 0.01
    
    # 第二次調用 (len(absolute_bearings_difference) >= 1)
    new_abs_bearing = get_absolute_bearing(ownship, ship_a)
    bearing_diff = (new_abs_bearing - abs_bearing_to_ship)
    if bearing_diff > 180:
        bearing_diff -= 360
    elif bearing_diff < -180:
        bearing_diff += 360
    absolute_bearings_difference.append(bearing_diff / 0.01)
    absolute_bearings.append(new_abs_bearing)
    angular_sizes.append(get_angular_diameter(ownship, ship_a))
    
    print("\n第二次調用避障函數:")
    print(f"bearing_difference[-1]: {absolute_bearings_difference[-1]:.4f}°/s")
    print(f"angular_size[-1]: {angular_sizes[-1]:.4f}°")
    
    # 檢查CBDR條件
    cbdr_condition1 = abs(absolute_bearings_difference[-1] * 0.01) <= angular_sizes[-1]
    cbdr_condition2 = angular_sizes[-1] > 2.0
    print(f"CBDR條件1 (|bearing_rate * dt| <= angular_size): {cbdr_condition1}")
    print(f"CBDR條件2 (angular_size > 2.0): {cbdr_condition2}")
    print(f"CBDR檢測結果: {cbdr_condition1 and cbdr_condition2}")
    
    rate_of_turn2, velocity2 = adj_ownship_heading(
        absolute_bearings, absolute_bearings_difference, angular_sizes,
        ownship, goal, ship_a, 0.01
    )
    print(f"返回: rate_of_turn={rate_of_turn2:.4f}, velocity={velocity2:.2f}")
    print()
    
    print("=== 問題4: 目的地導航邏輯 ===")
    print("當沒有碰撞威脅時，應該導航到goal")
    print("但是goal與ship_a位置相同，這會造成混亂")
    print()
    
    print("=== 建議修正 ===")
    print("1. 將goal位置改為不同於ship_a的位置")
    print("2. 檢查goal導航邏輯是否正確執行")
    print("3. 確認避障條件不會永遠為真")

if __name__ == "__main__":
    debug_ship_behavior()
