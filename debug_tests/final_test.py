#!/usr/bin/env python3
"""
最終測試：確認船隻會避障然後前往目的地
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, get_angular_diameter, adj_ownship_heading, get_distance_3d

def final_test():
    """最終測試避障和目標導航"""
    
    print("=== 最終測試：避障 + 目標導航 ===")
    print()
    
    # 設置
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[20, 0, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 20, 0])
    
    print("設置:")
    print(f"Ownship: {ownship.position} → heading {ownship.heading}°")
    print(f"Ship A: {ship_a.position} → heading {ship_a.heading}°")
    print(f"Goal: {goal.position}")
    print()
    
    # 簡短仿真
    dt = 0.1  # 更大的時間步，看更明顯的效果
    steps = 200
    
    # 存儲數據
    positions = []
    modes = []
    turns = []
    distances_to_ship = []
    distances_to_goal = []
    
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    for i in range(steps):
        # 記錄當前狀態
        positions.append(ownship.position.copy())
        
        # 計算狀態
        abs_bearing = get_absolute_bearing(ownship, ship_a)
        angular_size = get_angular_diameter(ownship, ship_a)
        dist_ship = get_distance_3d(ownship.position, ship_a.position)
        dist_goal = get_distance_3d(ownship.position, goal.position)
        
        absolute_bearings.append(abs_bearing)
        angular_sizes.append(angular_size)
        distances_to_ship.append(dist_ship)
        distances_to_goal.append(dist_goal)
        
        if i > 0:
            bearing_diff = abs_bearing - absolute_bearings[i-1]
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
        turns.append(rate_of_turn)
        
        # 判斷模式
        if len(absolute_bearings_difference) == 0:
            mode = "初始檢測"
        elif len(absolute_bearings_difference) >= 1:
            cbdr_condition = (abs(absolute_bearings_difference[-1]*dt) <= angular_sizes[-1] and 
                             angular_sizes[-1] > 2.0 and 
                             dist_ship < 25.0)
            if cbdr_condition:
                mode = "CBDR避障"
            else:
                mode = "目標導航"
        modes.append(mode)
        
        # 更新狀態
        ownship.rate_of_turn = rate_of_turn
        ownship.velocity = velocity
        ownship.heading += ownship.rate_of_turn * dt
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt
        
        # 每20步報告一次
        if i % 20 == 0:
            print(f"步驟 {i}: 模式={mode}, 位置=({ownship.position[0]:.1f}, {ownship.position[1]:.1f}), "
                  f"到Ship A={dist_ship:.1f}m, 到Goal={dist_goal:.1f}m, 轉向={rate_of_turn:.1f}°/s")
        
        # 如果到達目標附近，停止
        if dist_goal < 2:
            print(f"\n✓ 到達目標！步驟 {i}, 最終位置: ({ownship.position[0]:.1f}, {ownship.position[1]:.1f})")
            break
    
    # 統計
    positions = np.array(positions)
    avoid_steps = sum(1 for m in modes if 'CBDR' in m or '初始' in m)
    nav_steps = sum(1 for m in modes if '目標' in m)
    
    print(f"\n=== 結果統計 ===")
    print(f"總步驟: {len(modes)}")
    print(f"避障步驟: {avoid_steps}")
    print(f"導航步驟: {nav_steps}")
    print(f"最終到Goal距離: {distances_to_goal[-1]:.2f}m")
    print(f"最小到Ship A距離: {min(distances_to_ship):.2f}m")
    
    # 簡單繪圖
    plt.figure(figsize=(10, 6))
    plt.subplot(1, 2, 1)
    plt.plot(positions[:, 1], positions[:, 0], 'b-', label='Ownship路徑')
    plt.plot(0, 0, 'bo', markersize=10, label='起點')
    plt.plot(goal.position[1], goal.position[0], 'go', markersize=10, label='目標')
    plt.plot(20, 0, 'ro', markersize=10, label='Ship A起點')
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('船隻路徑')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.subplot(1, 2, 2)
    plt.plot(np.arange(len(distances_to_goal)) * dt, distances_to_goal, 'g-', label='到Goal距離')
    plt.plot(np.arange(len(distances_to_ship)) * dt, distances_to_ship, 'r-', label='到Ship A距離')
    plt.xlabel('時間 (s)')
    plt.ylabel('距離 (m)')
    plt.title('距離變化')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    final_test()
