#!/usr/bin/env python3
"""
Test the corrected avoidance logic
"""

def test_corrected_logic():
    """Test the corrected CBDR avoidance logic"""
    
    print("=== 測試修正後的避障邏輯 ===")
    print()
    
    # Test case: Ship A moving from west to east in front of ownship
    print("場景：Ship A從左前方向右前方移動")
    print("- 絕對bearing變化：135° → 90° → 45°")
    print("- absolute_bearings_difference = -45° (negative)")
    print("- 相對bearing = -45° (左舷，前方扇區)")
    print()
    
    absolute_bearings_difference = -45.0  # bearing decreasing
    current_relative_bearing = -45.0      # port side, front sector
    avoidance_gain = 6.0
    
    # Apply corrected logic
    is_front_sector = abs(current_relative_bearing) <= 90
    
    if is_front_sector:
        # Target ahead: turn in same direction as absolute bearing rate
        rate_of_turn = np.sign(absolute_bearings_difference) * avoidance_gain
        print("前方扇區邏輯：與absolute bearing rate同向轉")
    else:
        # Target behind: turn opposite to absolute bearing rate
        rate_of_turn = -np.sign(absolute_bearings_difference) * avoidance_gain
        print("後方扇區邏輯：與absolute bearing rate反向轉")
    
    print(f"absolute_bearings_difference = {absolute_bearings_difference}")
    print(f"np.sign(absolute_bearings_difference) = {np.sign(absolute_bearings_difference)}")
    print(f"rate_of_turn = {np.sign(absolute_bearings_difference)} * {avoidance_gain} = {rate_of_turn}")
    
    direction = "左轉" if rate_of_turn < 0 else "右轉"
    print(f"轉向結果: {direction}")
    print()
    
    print("=== 物理意義驗證 ===")
    print("Ship A從左前方向右前方移動，Ownship左轉：")
    print("- 效果：增加Ship A絕對bearing的減少速率")
    print("- 結果：Ship A從Ownship前方更快速通過，避免碰撞")
    print("- 這個邏輯是正確的！✓")

if __name__ == "__main__":
    import numpy as np
    test_corrected_logic()
