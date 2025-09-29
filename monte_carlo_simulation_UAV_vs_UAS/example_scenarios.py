"""
Example configuration for different test scenarios
不同測試場景的範例配置
"""

# =============================================================================
# 場景 1: 基本性能測試（預設配置）
# =============================================================================
BASIC_SCENARIO = {
    "NUM_SIMULATIONS": 50,
    "SHIP_A_VELOCITY_RANGE": [0.5, 3.0],
    "SHIP_A_HEADING_RANGE": [0.0, 360.0],
    "COLLISION_ZONE_START_RATIO": 0.4,
    "COLLISION_ZONE_END_RATIO": 0.6,
    "USE_ABSOLUTE_BEARINGS": True
}

# =============================================================================
# 場景 2: 高速船隻測試
# =============================================================================
HIGH_SPEED_SCENARIO = {
    "NUM_SIMULATIONS": 30,
    "SHIP_A_VELOCITY_RANGE": [3.0, 6.0],  # 更高速度
    "SHIP_A_HEADING_RANGE": [0.0, 360.0],
    "COLLISION_ZONE_START_RATIO": 0.3,    # 更早的碰撞點
    "COLLISION_ZONE_END_RATIO": 0.7,      # 更寬的碰撞區域
    "USE_ABSOLUTE_BEARINGS": True
}

# =============================================================================
# 場景 3: 正面接近測試
# =============================================================================
HEAD_ON_SCENARIO = {
    "NUM_SIMULATIONS": 25,
    "SHIP_A_VELOCITY_RANGE": [1.0, 4.0],
    "SHIP_A_HEADING_RANGE": [170.0, 190.0],  # 接近正面（180°±10°）
    "COLLISION_ZONE_START_RATIO": 0.4,
    "COLLISION_ZONE_END_RATIO": 0.6,
    "USE_ABSOLUTE_BEARINGS": True
}

# =============================================================================
# 場景 4: 側面接近測試
# =============================================================================
CROSSING_SCENARIO = {
    "NUM_SIMULATIONS": 40,
    "SHIP_A_VELOCITY_RANGE": [1.0, 3.0],
    "SHIP_A_HEADING_RANGE": [60.0, 120.0, 240.0, 300.0],  # 左右側面接近
    "COLLISION_ZONE_START_RATIO": 0.35,
    "COLLISION_ZONE_END_RATIO": 0.65,
    "USE_ABSOLUTE_BEARINGS": True
}

# =============================================================================
# 場景 5: 相對方位控制測試
# =============================================================================
RELATIVE_BEARING_SCENARIO = {
    "NUM_SIMULATIONS": 30,
    "SHIP_A_VELOCITY_RANGE": [0.5, 3.0],
    "SHIP_A_HEADING_RANGE": [0.0, 360.0],
    "COLLISION_ZONE_START_RATIO": 0.4,
    "COLLISION_ZONE_END_RATIO": 0.6,
    "USE_ABSOLUTE_BEARINGS": False  # 使用相對方位控制
}

# =============================================================================
# 場景 6: 壓力測試（密集碰撞區域）
# =============================================================================
STRESS_TEST_SCENARIO = {
    "NUM_SIMULATIONS": 100,
    "SHIP_A_VELOCITY_RANGE": [1.5, 4.0],
    "SHIP_A_HEADING_RANGE": [0.0, 360.0],
    "COLLISION_ZONE_START_RATIO": 0.45,   # 更小的碰撞區域
    "COLLISION_ZONE_END_RATIO": 0.55,     # 更集中的碰撞點
    "SHIP_A_MIN_SPAWN_DISTANCE": 5.0,     # 更近的生成距離
    "SHIP_A_MAX_SPAWN_DISTANCE": 50.0,    # 更近的生成距離
    "USE_ABSOLUTE_BEARINGS": True
}

def apply_scenario(scenario_name):
    """
    應用特定場景配置到 config.py
    
    使用方法:
    from example_scenarios import apply_scenario
    apply_scenario("HIGH_SPEED_SCENARIO")
    """
    import config
    
    scenarios = {
        "BASIC": BASIC_SCENARIO,
        "HIGH_SPEED": HIGH_SPEED_SCENARIO,
        "HEAD_ON": HEAD_ON_SCENARIO,
        "CROSSING": CROSSING_SCENARIO,
        "RELATIVE_BEARING": RELATIVE_BEARING_SCENARIO,
        "STRESS_TEST": STRESS_TEST_SCENARIO
    }
    
    if scenario_name not in scenarios:
        print(f"❌ Unknown scenario: {scenario_name}")
        print(f"Available scenarios: {list(scenarios.keys())}")
        return False
    
    scenario = scenarios[scenario_name]
    
    # 應用配置
    for key, value in scenario.items():
        if hasattr(config, key):
            setattr(config, key, value)
            print(f"✅ Set {key} = {value}")
        else:
            print(f"⚠️  Warning: {key} not found in config")
    
    print(f"🎯 Applied scenario: {scenario_name}")
    return True

if __name__ == "__main__":
    print("Available test scenarios:")
    print("=" * 40)
    
    scenarios_info = {
        "BASIC": "基本性能測試（預設配置）",
        "HIGH_SPEED": "高速船隻測試（3-6 m/s）", 
        "HEAD_ON": "正面接近測試（170-190°）",
        "CROSSING": "側面接近測試（60-120°, 240-300°）",
        "RELATIVE_BEARING": "相對方位控制測試",
        "STRESS_TEST": "壓力測試（密集碰撞區域）"
    }
    
    for name, description in scenarios_info.items():
        print(f"• {name:20}: {description}")
    
    print("\n使用方法:")
    print("from example_scenarios import apply_scenario")
    print("apply_scenario('HIGH_SPEED')")
