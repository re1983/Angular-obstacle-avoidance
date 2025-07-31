#!/usr/bin/env python3
"""
Debug version to analyze the collision avoidance behavior
"""
import numpy as np
import matplotlib.pyplot as plt

def get_bearing(ship1, ship2):
    """Calculate relative bearing from ship1 to ship2"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    angle_to_ship2 = np.degrees(theta)
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    return relative_bearing

def get_absolute_bearing(ship1, ship2):
    """Calculate absolute bearing (true bearing) from ship1 to ship2"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    return absolute_bearing

class ShipStatus:
    def __init__(self, name, position, heading):
        self.name = name
        self.position = np.array(position, dtype=float)
        self.heading = heading

def debug_initial_geometry():
    """Debug the initial collision geometry"""
    
    # Initial setup
    ownship = ShipStatus("Ownship", [0, 0, 0], heading=0.0)  # 朝北
    ship_a = ShipStatus("Ship A", [10, -10, 0], heading=90.0)  # 朝東
    
    # Calculate initial bearings
    relative_bearing = get_bearing(ownship, ship_a)
    absolute_bearing = get_absolute_bearing(ownship, ship_a)
    
    print("=== 初始幾何分析 ===")
    print(f"Ownship位置: {ownship.position[:2]}, 航向: {ownship.heading}° (朝北)")
    print(f"Ship A位置: {ship_a.position[:2]}, 航向: {ship_a.heading}° (朝東)")
    print()
    
    print("=== Bearing計算 ===")
    print(f"Relative bearing: {relative_bearing:.1f}°")
    print(f"Absolute bearing: {absolute_bearing:.1f}°")
    print()
    
    # Analyze the geometry
    delta_pos = ship_a.position - ownship.position
    print(f"位置差向量: {delta_pos[:2]}")
    print(f"Ship A相對於Ownship的方向: 東南方")
    print()
    
    # Determine sector
    is_front_sector = abs(relative_bearing) <= 90
    sector = "前方180°扇區" if is_front_sector else "後方180°扇區"
    print(f"Ship A位於Ownship的: {sector}")
    print()
    
    # Expected collision point analysis
    print("=== 碰撞分析 ===")
    print("如果兩船保持當前航向和速度:")
    print("- Ownship朝北移動")
    print("- Ship A朝東移動")
    print("- 碰撞點大約在 (10, 0) 附近")
    print()
    
    print("=== 預期避障行為 ===")
    if relative_bearing > 0:
        print(f"Ship A在Ownship右舷 (+{relative_bearing:.1f}°)")
        print("為避免碰撞，Ownship應該向左轉（往西）")
        print("期望的rate_of_turn: 負值")
    else:
        print(f"Ship A在Ownship左舷 ({relative_bearing:.1f}°)")
        print("為避免碰撞，Ownship應該向右轉（往東）")
        print("期望的rate_of_turn: 正值")
    
    return relative_bearing, absolute_bearing, is_front_sector

if __name__ == "__main__":
    debug_initial_geometry()
