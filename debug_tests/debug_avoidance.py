#!/usr/bin/env python3
"""
Debug the avoidance logic step by step
"""
import numpy as np

def analyze_avoidance_logic():
    """Analyze the current avoidance logic"""
    
    # Initial conditions from your simulation
    relative_bearing = -45.0  # Ship A在左舷
    absolute_bearings_difference = [0.0, -0.5]  # 假設一個小的absolute bearing rate
    angular_size = 6.0  # 假設angular size > 5.0觸發避障
    
    print("=== 避障邏輯分析 ===")
    print(f"Relative bearing: {relative_bearing}°")
    print(f"Absolute bearing rate: {absolute_bearings_difference[-1]}°/s")
    print(f"Angular size: {angular_size}°")
    print()
    
    # Check sector
    is_front_sector = abs(relative_bearing) <= 90
    print(f"扇區判斷: {'前方' if is_front_sector else '後方'}扇區")
    print()
    
    # Apply current logic
    if is_front_sector:
        # Target ahead: turn opposite to absolute bearing rate direction
        rate_of_turn = -1 * np.sign(absolute_bearings_difference[-1]) * angular_size
        direction = "左轉" if rate_of_turn < 0 else "右轉"
        print("前方扇區邏輯: 與absolute bearing rate反向轉")
        print(f"absolute_bearings_difference[-1] = {absolute_bearings_difference[-1]}")
        print(f"np.sign(absolute_bearings_difference[-1]) = {np.sign(absolute_bearings_difference[-1])}")
        print(f"rate_of_turn = -1 * {np.sign(absolute_bearings_difference[-1])} * {angular_size} = {rate_of_turn}")
        print(f"結果: {direction}")
    else:
        # Target behind: turn same direction as absolute bearing rate
        rate_of_turn = np.sign(absolute_bearings_difference[-1]) * angular_size
        direction = "左轉" if rate_of_turn < 0 else "右轉"
        print("後方扇區邏輯: 與absolute bearing rate同向轉")
        print(f"rate_of_turn = {np.sign(absolute_bearings_difference[-1])} * {angular_size} = {rate_of_turn}")
        print(f"結果: {direction}")
    
    print()
    print("=== 物理意義分析 ===")
    if relative_bearing < 0:
        print("Ship A在左舷，Ownship應該向右轉避開")
        expected_direction = "右轉"
    else:
        print("Ship A在右舷，Ownship應該向左轉避開")
        expected_direction = "左轉"
    
    print(f"期望方向: {expected_direction}")
    print(f"實際方向: {direction}")
    print(f"邏輯正確: {'✓' if expected_direction == direction else '✗'}")

if __name__ == "__main__":
    analyze_avoidance_logic()
