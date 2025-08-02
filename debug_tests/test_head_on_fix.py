#!/usr/bin/env python3
"""
測試修正後的頭撞避障邏輯
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading

def test_corrected_head_on():
    """測試修正後的頭撞避障"""
    
    print("=== 測試修正後的頭撞避障邏輯 ===")
    print()
    
    # 對開設置：兩船相向而行
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    
    print("初始設置 (對開情況):")
    print(f"Ownship: heading={ownship.heading}° (朝北), position={ownship.position}")
    print(f"Ship A: heading={ship_a.heading}° (朝南), position={ship_a.position}")
    print()
    
    # 測試前3步的行為
    dt = 0.01
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    for i in range(3):
        # 計算當前狀態
        rel_bearing = get_bearing(ownship, ship_a)
        abs_bearing = get_absolute_bearing(ownship, ship_a)
        angular_size = get_angular_diameter(ownship, ship_a)
        distance = np.linalg.norm(ownship.position - ship_a.position)
        
        absolute_bearings.append(abs_bearing)
        angular_sizes.append(angular_size)
        
        if i > 0:
            bearing_diff = abs_bearing - absolute_bearings[i-1]
            # Handle 360° wrap
            if bearing_diff > 180:
                bearing_diff -= 360
            elif bearing_diff < -180:
                bearing_diff += 360
            absolute_bearings_difference.append(bearing_diff / dt)
        
        print(f"=== 時間步 {i} ===")
        print(f"相對bearing: {rel_bearing:.2f}° ({'右舷' if rel_bearing > 0 else '左舷' if rel_bearing < 0 else '正前方'})")
        print(f"絕對bearing: {abs_bearing:.2f}°")
        print(f"角直徑: {angular_size:.2f}°")
        print(f"距離: {distance:.2f}m")
        
        if len(absolute_bearings_difference) > 0:
            print(f"絕對bearing變化率: {absolute_bearings_difference[-1]:.4f}°/s")
        
        # 測試避障邏輯
        rate_of_turn, velocity = adj_ownship_heading(
            absolute_bearings,
            absolute_bearings_difference, 
            angular_sizes,
            ownship,
            goal,
            ship_a,
            dt
        )
        
        print(f"避障決策: rate_of_turn={rate_of_turn:.4f}°/s, velocity={velocity:.2f}m/s")
        
        if rate_of_turn != 0:
            direction = "右轉" if rate_of_turn > 0 else "左轉"
            print(f"✓ 船隻開始{direction}避障！")
        else:
            print("✗ 船隻沒有轉向")
        
        print()
        
        # 更新ownship狀態
        ownship.rate_of_turn = rate_of_turn
        ownship.heading += ownship.rate_of_turn * dt
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        # 更新ship_a位置（繼續朝南）
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt

if __name__ == "__main__":
    test_corrected_head_on()
