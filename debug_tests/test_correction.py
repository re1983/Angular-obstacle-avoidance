#!/usr/bin/env python3
"""
Test the corrected CBDR avoidance logic
"""
import numpy as np
import sys
sys.path.append('/home/re1983/extra/Work/Angular-obstacle-avoidance')

from BearingRateGraph_cleaned import ShipStatus, get_bearing, get_absolute_bearing, adj_ownship_heading, get_angular_diameter

def test_cbdr_correction():
    """Test the corrected CBDR avoidance logic"""
    
    print("=== 測試修正後的CBDR避障邏輯 ===")
    print()
    
    # Set up the scenario: Ship A approaching from southeast (-45° relative)
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[0, 0, 0])
    ship_a = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=90.0, rate_of_turn=0, position=[10, -10, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    
    # Simulate a few time steps to build up bearing history
    absolute_bearings = []
    absolute_bearings_difference = []
    angular_sizes = []
    
    print("建立bearing歷史...")
    for i in range(5):
        # Calculate current bearings
        rel_bearing = get_bearing(ownship, ship_a)
        abs_bearing = get_absolute_bearing(ownship, ship_a)
        angular_size = get_angular_diameter(ownship, ship_a)
        
        absolute_bearings.append(abs_bearing)
        angular_sizes.append(angular_size)
        
        if i > 0:
            bearing_diff = abs_bearing - absolute_bearings[i-1]
            # Handle 360° wrap
            if bearing_diff > 180:
                bearing_diff -= 360
            elif bearing_diff < -180:
                bearing_diff += 360
            absolute_bearings_difference.append(bearing_diff)
        
        print(f"時刻 {i}: 相對bearing={rel_bearing:.1f}°, 絕對bearing={abs_bearing:.1f}°, 角直徑={angular_size:.1f}°")
        
        # Update positions (minimal movement to simulate CBDR)
        ownship.position[1] += 0.01  # Small northward movement
        ship_a.position[0] += 0.01   # Small eastward movement
    
    print(f"\nBearing變化歷史: {[f'{diff:.3f}°' for diff in absolute_bearings_difference]}")
    print()
    
    # Test the avoidance function
    print("=== 測試避障決策 ===")
    current_rel_bearing = get_bearing(ownship, ship_a)
    current_abs_bearing = get_absolute_bearing(ownship, ship_a)
    current_angular_size = get_angular_diameter(ownship, ship_a)
    
    print(f"當前相對bearing: {current_rel_bearing:.1f}° (Ship A在左舷)")
    print(f"當前絕對bearing: {current_abs_bearing:.1f}°")
    print(f"當前角直徑: {current_angular_size:.1f}°")
    print(f"最新bearing變化: {absolute_bearings_difference[-1]:.3f}°")
    print()
    
    # Call the avoidance function
    rate_of_turn, velocity = adj_ownship_heading(
        absolute_bearings, 
        absolute_bearings_difference, 
        angular_sizes, 
        ownship, 
        goal, 
        ship_a
    )
    
    print(f"避障決策:")
    print(f"  Rate of turn: {rate_of_turn:.2f} (負值=左轉, 正值=右轉)")
    print(f"  Velocity: {velocity:.2f}")
    print()
    
    if rate_of_turn < 0:
        print("✓ 正確！船隻會左轉 (往西) 避開Ship A")
    elif rate_of_turn > 0:
        print("✗ 錯誤！船隻會右轉 (往東)，這會增加碰撞風險")
    else:
        print("✗ 錯誤！船隻不轉向，將發生碰撞")
    
    print()
    print("=== 理論驗證 ===")
    print("Ship A在左舷(-45°)，應該左轉讓Ship A從右舷通過")
    print("左轉會使Ship A的絕對bearing增加，打破CBDR狀態")

if __name__ == "__main__":
    test_cbdr_correction()
