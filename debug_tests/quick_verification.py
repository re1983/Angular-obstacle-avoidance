#!/usr/bin/env python3
"""
Quick verification of the corrected avoidance behavior
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, adj_ownship_heading, get_angular_diameter

def quick_verification():
    """Quick verification of turn direction"""
    
    print("=== 快速驗證修正後的避障行為 ===")
    
    # Set up scenario
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=90.0, rate_of_turn=0, position=[10, -10, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    
    # Track positions and decisions
    ownship_x, ownship_y = [], []
    ship_a_x, ship_a_y = [], []
    turn_decisions = []
    
    # Build initial bearing history
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    # Run for a short time to see initial behavior
    dt = 0.1
    for i in range(50):  # 5 seconds
        # Record positions
        ownship_x.append(ownship.position[0])
        ownship_y.append(ownship.position[1])
        ship_a_x.append(ship_a.position[0])
        ship_a_y.append(ship_a.position[1])
        
        # Calculate bearings
        abs_bearing = get_absolute_bearing(ownship, ship_a)
        angular_size = get_angular_diameter(ownship, ship_a)
        
        absolute_bearings.append(abs_bearing)
        angular_sizes.append(angular_size)
        
        if len(absolute_bearings) > 1:
            bearing_diff = abs_bearing - absolute_bearings[-2]
            # Handle wrap-around
            if bearing_diff > 180:
                bearing_diff -= 360
            elif bearing_diff < -180:
                bearing_diff += 360
            absolute_bearings_difference.append(bearing_diff)
        
        # Get avoidance decision
        if len(absolute_bearings_difference) > 0:
            rate_of_turn, velocity = adj_ownship_heading(
                absolute_bearings, 
                absolute_bearings_difference, 
                angular_sizes, 
                ownship, 
                goal, 
                ship_a,
                dt
            )
            turn_decisions.append(rate_of_turn)
            ownship.rate_of_turn = rate_of_turn
        else:
            turn_decisions.append(0)
        
        # Update ship positions
        # Ownship
        ownship.heading += ownship.rate_of_turn * dt
        ownship.position[0] += ownship.velocity * np.sin(np.radians(ownship.heading)) * dt
        ownship.position[1] += ownship.velocity * np.cos(np.radians(ownship.heading)) * dt
        
        # Ship A (continues straight east)
        ship_a.position[0] += ship_a.velocity * np.sin(np.radians(ship_a.heading)) * dt
        ship_a.position[1] += ship_a.velocity * np.cos(np.radians(ship_a.heading)) * dt
        
        # Early exit if no collision threat
        distance = np.sqrt((ownship.position[0] - ship_a.position[0])**2 + 
                          (ownship.position[1] - ship_a.position[1])**2)
        if distance > 20:
            break
    
    # Analysis
    print(f"仿真運行了 {len(turn_decisions)} 步 ({len(turn_decisions)*dt:.1f} 秒)")
    print(f"最終距離: {distance:.1f}")
    
    # Check turn behavior
    negative_turns = sum(1 for x in turn_decisions if x < -0.01)
    positive_turns = sum(1 for x in turn_decisions if x > 0.01)
    no_turns = sum(1 for x in turn_decisions if abs(x) <= 0.01)
    
    print(f"左轉決策: {negative_turns} 次")
    print(f"右轉決策: {positive_turns} 次") 
    print(f"不轉向: {no_turns} 次")
    
    if negative_turns > positive_turns:
        print("✓ 成功！主要是左轉 (往西避障)")
    elif positive_turns > negative_turns:
        print("✗ 仍有問題：主要是右轉 (往東)")
    else:
        print("? 無明確轉向傾向")
    
    # Check final positions
    print(f"Ownship最終位置: ({ownship.position[0]:.1f}, {ownship.position[1]:.1f})")
    print(f"Ship A最終位置: ({ship_a.position[0]:.1f}, {ship_a.position[1]:.1f})")
    
    if ownship.position[0] < -1:  # Moved west
        print("✓ Ownship向西移動，避障成功")
    elif ownship.position[0] > 1:   # Moved east  
        print("✗ Ownship向東移動，避障失敗")
    else:
        print("- Ownship基本保持直行")

if __name__ == "__main__":
    quick_verification()
