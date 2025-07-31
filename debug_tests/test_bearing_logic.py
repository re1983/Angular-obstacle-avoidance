#!/usr/bin/env python3
"""
Test script to verify bearing sector logic
"""

def test_bearing_sectors():
    """Test the bearing sector classification"""
    
    # Test cases: (bearing, expected_sector)
    test_cases = [
        (0, "front"),      # straight ahead
        (45, "front"),     # front right
        (-45, "front"),    # front left
        (90, "front"),     # right beam (edge case)
        (-90, "front"),    # left beam (edge case)
        (91, "rear"),      # just behind right beam
        (-91, "rear"),     # just behind left beam
        (135, "rear"),     # rear right
        (-135, "rear"),    # rear left
        (180, "rear"),     # straight behind
        (-180, "rear"),    # straight behind (negative)
    ]
    
    print("Bearing Sector Classification Test")
    print("=" * 40)
    print(f"{'Bearing':<10} {'Expected':<10} {'abs(bearing) <= 90':<20} {'Result':<10} {'Match'}")
    print("-" * 60)
    
    for bearing, expected in test_cases:
        is_front = abs(bearing) <= 90
        result = "front" if is_front else "rear"
        match = "✓" if result == expected else "✗"
        
        print(f"{bearing:<10} {expected:<10} {is_front:<20} {result:<10} {match}")
    
    print("\nConclusion:")
    print("The logic 'abs(current_relative_bearing) <= 90' correctly classifies:")
    print("- Front 180° sector: -90° to +90°")
    print("- Rear 180° sector: -180° to -90° and +90° to +180°")

if __name__ == "__main__":
    test_bearing_sectors()
