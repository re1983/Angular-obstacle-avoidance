#!/usr/bin/env python3
"""
測試修正後的邏輯：goal位置更改和距離閾值
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading, get_distance_3d

def test_corrected_logic():
    """測試修正後的邏輯"""
    
    print("=== 測試修正後的避障和目標導航邏輯 ===")
    print()
    
    # 新設置
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 20, 0])
    
    print("修正後的設置:")
    print(f"Ownship: position={ownship.position}, heading={ownship.heading}°")
    print(f"Ship A: position={ship_a.position}, heading={ship_a.heading}°")
    print(f"Goal: position={goal.position} (現在在北方)")
    print()
    
    # 模擬幾個時間步
    dt = 0.01
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    print("時間步模擬:")
    for i in range(10):
        # 計算當前狀態
        rel_bearing_ship = get_bearing(ownship, ship_a)
        abs_bearing_ship = get_absolute_bearing(ownship, ship_a)
        rel_bearing_goal = get_bearing(ownship, goal)
        angular_size = get_angular_diameter(ownship, ship_a)
        distance_ship = get_distance_3d(ownship.position, ship_a.position)
        distance_goal = get_distance_3d(ownship.position, goal.position)
        
        absolute_bearings.append(abs_bearing_ship)
        angular_sizes.append(angular_size)
        
        if i > 0:
            bearing_diff = abs_bearing_ship - absolute_bearings[i-1]
            if bearing_diff > 180:
                bearing_diff -= 360
            elif bearing_diff < -180:
                bearing_diff += 360
            absolute_bearings_difference.append(bearing_diff / dt)
        
        print(f"\n時間步 {i}:")
        print(f"  到Ship A: rel_bearing={rel_bearing_ship:.2f}°, distance={distance_ship:.2f}m")
        print(f"  到Goal: rel_bearing={rel_bearing_goal:.2f}°, distance={distance_goal:.2f}m")
        print(f"  角直徑: {angular_size:.2f}°")
        
        # 測試避障邏輯
        rate_of_turn, velocity = adj_ownship_heading(
            absolute_bearings, absolute_bearings_difference, angular_sizes,
            ownship, goal, ship_a, dt
        )
        
        print(f"  決策: rate_of_turn={rate_of_turn:.4f}°/s, velocity={velocity:.2f}m/s")
        
        # 判斷當前模式
        if len(absolute_bearings_difference) == 0:
            print(f"  模式: 初始頭撞檢測")
        elif len(absolute_bearings_difference) >= 1:
            cbdr_condition = (abs(absolute_bearings_difference[-1]*dt) <= angular_sizes[-1] and 
                             angular_sizes[-1] > 2.0 and 
                             distance_ship < 15.0)
            if cbdr_condition:
                print(f"  模式: CBDR避障")
            else:
                print(f"  模式: 目標導航")
        
        # 更新位置
        ownship.rate_of_turn = rate_of_turn
        ownship.heading += ownship.rate_of_turn * dt
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt
        
        # 如果距離夠遠，停止避障測試
        if distance_ship > 16:
            print(f"\n*** 距離足夠遠 ({distance_ship:.2f}m > 15m)，應該切換到目標導航 ***")
            break

if __name__ == "__main__":
    test_corrected_logic()
