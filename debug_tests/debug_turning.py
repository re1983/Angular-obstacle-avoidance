#!/usr/bin/env python3
"""
Debug why the ship is turning east instead of west
"""
import numpy as np

def debug_turning_direction():
    """Debug the actual turning direction"""
    
    print("=== 調試轉向方向 ===")
    print()
    
    # Initial setup
    ownship_pos = np.array([0, 0])
    ship_pos = np.array([10, -10])
    ownship_heading = 0  # North
    
    print("初始設置:")
    print(f"Ownship: 位置{ownship_pos}, 航向{ownship_heading}° (朝北)")
    print(f"Ship A: 位置{ship_pos}")
    print()
    
    # Calculate initial absolute bearing
    delta_pos = ship_pos - ownship_pos
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    
    print(f"初始絕對bearing: {absolute_bearing}°")
    
    # Calculate relative bearing
    relative_bearing = (absolute_bearing - ownship_heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    
    print(f"初始相對bearing: {relative_bearing}°")
    print()
    
    # Simulate bearing change
    print("=== 模擬bearing變化 ===")
    
    # After 1 second: ownship moves north, ship moves east
    ownship_pos_t1 = np.array([0, 1])
    ship_pos_t1 = np.array([11, -10])
    
    delta_pos_t1 = ship_pos_t1 - ownship_pos_t1
    theta_t1 = np.arctan2(delta_pos_t1[1], delta_pos_t1[0])
    absolute_bearing_t1 = np.degrees(theta_t1) % 360
    
    print(f"1秒後絕對bearing: {absolute_bearing_t1}°")
    
    # Calculate bearing rate
    def angle_diff(a1, a2):
        diff = (a2 - a1) % 360
        if diff > 180:
            diff -= 360
        return diff
    
    bearing_change = angle_diff(absolute_bearing, absolute_bearing_t1)
    print(f"Bearing變化: {bearing_change}°")
    print(f"absolute_bearings_difference = {bearing_change}")
    print()
    
    # Apply avoidance logic
    print("=== 避障邏輯分析 ===")
    
    is_front_sector = abs(relative_bearing) <= 90
    print(f"扇區: {'前方' if is_front_sector else '後方'}")
    
    avoidance_gain = 6.0
    
    if is_front_sector:
        rate_of_turn = -np.sign(bearing_change) * avoidance_gain
        print(f"前方扇區邏輯: rate_of_turn = -np.sign({bearing_change}) * {avoidance_gain}")
        print(f"rate_of_turn = -({np.sign(bearing_change)}) * {avoidance_gain} = {rate_of_turn}")
    else:
        rate_of_turn = np.sign(bearing_change) * avoidance_gain
        print(f"後方扇區邏輯: rate_of_turn = np.sign({bearing_change}) * {avoidance_gain}")
        print(f"rate_of_turn = {np.sign(bearing_change)} * {avoidance_gain} = {rate_of_turn}")
    
    direction = "左轉(西)" if rate_of_turn < 0 else "右轉(東)"
    print(f"轉向結果: {direction}")
    print()
    
    print("=== 問題分析 ===")
    if direction == "右轉(東)":
        print("❌ 問題：船隻往東轉，但應該往西轉")
        print()
        print("可能原因:")
        print("1. bearing變化計算錯誤")
        print("2. 扇區判斷錯誤") 
        print("3. 避障邏輯錯誤")
        print()
        
        print("重新檢查相對bearing計算...")
        # Recalculate relative bearing more carefully
        angle_to_ship = np.degrees(np.arctan2(delta_pos[1], delta_pos[0]))
        print(f"Ship A相對於正東的角度: {angle_to_ship}°")
        print(f"這表示Ship A在東南方向 (315° = -45°)")
        print()
        print("相對於Ownship航向(0°)的相對bearing:")
        rel_bear = angle_to_ship - ownship_heading
        print(f"原始相對bearing: {rel_bear}°")
        if rel_bear > 180:
            rel_bear -= 360
        elif rel_bear < -180:
            rel_bear += 360
        print(f"調整後相對bearing: {rel_bear}°")
        
        if rel_bear < 0:
            print("Ship A在左舷，期望Ownship右轉避開")
        else:
            print("Ship A在右舷，期望Ownship左轉避開")

if __name__ == "__main__":
    debug_turning_direction()
