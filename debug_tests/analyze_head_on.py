#!/usr/bin/env python3
"""
分析對開情況下的初始條件和bearing變化
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading

def analyze_head_on_scenario():
    """分析對開情況"""
    
    print("=== 對開情況分析 ===")
    print()
    
    # 當前設置
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    
    print("初始設置:")
    print(f"Ownship: heading={ownship.heading}°, position={ownship.position}")
    print(f"Ship A: heading={ship_a.heading}°, position={ship_a.position}")
    print()
    
    # 分析前幾個時間步
    dt = 0.01
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    print("前10個時間步的分析:")
    for i in range(10):
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
        
        print(f"步驟 {i}:")
        print(f"  相對bearing: {rel_bearing:.2f}°")
        print(f"  絕對bearing: {abs_bearing:.2f}°") 
        print(f"  角直徑: {angular_size:.2f}°")
        print(f"  距離: {distance:.2f}")
        
        if len(absolute_bearings_difference) > 0:
            print(f"  絕對bearing變化率: {absolute_bearings_difference[-1]:.4f}°/s")
            
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
            print(f"  避障決策: rate_of_turn={rate_of_turn:.4f}, velocity={velocity:.2f}")
        else:
            print(f"  絕對bearing變化率: N/A (需要至少2個數據點)")
            print(f"  避障決策: N/A (insufficient data)")
        
        print()
        
        # 更新位置
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt
    
    print("=== 關鍵問題 ===")
    print("1. len(absolute_bearings_difference) > 1 條件:")
    print(f"   當前長度: {len(absolute_bearings_difference)}")
    print("2. 在前10秒內，avoid_logic需要至少2個bearing差異數據點")
    print("3. 如果沒有足夠數據，船隻會直接導航到目標點")
    print()
    
    print("=== 建議修正 ===")
    print("1. 修改條件為 len(absolute_bearings_difference) >= 1")
    print("2. 或者在初始時刻添加特殊處理邏輯")
    print("3. 對於對開情況，應該立即開始避障")

if __name__ == "__main__":
    analyze_head_on_scenario()
