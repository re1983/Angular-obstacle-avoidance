"""
簡單回測工具 - 重新運行指定的模擬案例
現在完全從txt文件讀取配置，不依賴config.py
"""

import numpy as np
import sys
import re
from pathlib import Path

# 添加當前目錄到 Python 路徑
sys.path.append(str(Path(__file__).parent))

from BearingRateGraph_comparison import (
    create_collision_scenario_with_turning,
    run_single_simulation,
    plot_results
)

def parse_txt_file(txt_file):
    """解析 txt 結果文件，提取所有參數"""
    params = {}
    current_section = None
    
    with open(txt_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            
            # 識別各個區段
            if line == 'Ship A Parameters:':
                current_section = 'ship_params'
                params[current_section] = {}
            elif line == 'Collision Prediction:':
                current_section = 'collision_prediction'
                params[current_section] = {}
            elif line == 'Actual Collision Location:':
                current_section = 'actual_collision'
                params[current_section] = {}
            elif line == 'Ownship Configuration:':
                current_section = 'ownship_config'
                params[current_section] = {}
            elif line == 'Goal Configuration:':
                current_section = 'goal_config'
                params[current_section] = {}
            elif line == 'Simulation Parameters:':
                current_section = 'sim_params'
                params[current_section] = {}
            elif line == 'Simulation Results:':
                current_section = 'sim_results'  
                params[current_section] = {}
            elif ':' in line and current_section:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                # 解析數值和列表
                parsed_value = parse_value(value)
                params[current_section][key] = parsed_value
    
    return params

def parse_value(value_str):
    """解析字符串為適當的數據類型"""
    value_str = value_str.strip()
    
    if value_str.lower() == 'none':
        return None
    elif value_str.lower() == 'true':
        return True
    elif value_str.lower() == 'false':
        return False
    
    # 嘗試解析列表 [x, y, z]
    if value_str.startswith('[') and value_str.endswith(']'):
        try:
            # 提取數字
            numbers = re.findall(r'-?\d+\.?\d*', value_str)
            return [float(num) for num in numbers] if numbers else []
        except:
            pass
    
    # 嘗試解析數字
    try:
        if '.' in value_str:
            return float(value_str)
        else:
            return int(value_str)
    except ValueError:
        return value_str  # 保持為字符串

def backtest_single_case(txt_file, show_plot=True):
    """回測單一案例 - 完全從txt文件讀取配置"""
    print(f"回測文件: {Path(txt_file).name}")
    print("=" * 50)
    
    # 解析所有參數
    params = parse_txt_file(txt_file)
    ship_params = params.get('ship_params', {})
    ownship_config = params.get('ownship_config', {})
    goal_config = params.get('goal_config', {})
    sim_params = params.get('sim_params', {})
    original_results = params.get('sim_results', {})
    
    print("從txt文件讀取的配置:")
    print(f"  Ship A Parameters: {len(ship_params)} 項")
    print(f"  Ownship Config: {len(ownship_config)} 項") 
    print(f"  Goal Config: {len(goal_config)} 項")
    print(f"  Simulation Parameters: {len(sim_params)} 項")
    
    print(f"\n原始模擬結果:")
    print(f"  碰撞時間: {original_results.get('collision_time')}")
    print(f"  到達時間: {original_results.get('arrival_time')}")  
    print(f"  最小表面距離: {original_results.get('min_distance'):.6f}m")
    
    # 構建配置字典（從txt文件讀取，不依賴config.py）
    ownship_config_dict = {
        "name": "Ownship",
        "velocity": ownship_config.get('velocity', 1.0),
        "acceleration": 0,
        "heading": 0.0,
        "rate_of_turn": 0,
        "position": ownship_config.get('initial_position', [0, 0, 0]),
        "size": ownship_config.get('size', 0.5),
        "max_rate_of_turn": ownship_config.get('max_rate_of_turn', [12, 12])
    }
    
    goal_config_dict = {
        "name": "Goal",
        "velocity": 0.0,
        "acceleration": 0,
        "heading": 0,
        "rate_of_turn": 0,
        "position": goal_config.get('position', [50, 0, 0]),
        "size": 0.5,
        "max_rate_of_turn": [0, 0]
    }
    
    # 重新創建場景並運行（使用txt文件中的參數）
    print(f"\n重新運行模擬...")
    ship_config = create_collision_scenario_with_turning(
        ship_velocity=ship_params['velocity'],
        ship_heading=ship_params['heading'], 
        ship_rate_of_turn=ship_params.get('rate_of_turn', 0.0),
        ship_size=ship_params['size'],
        collision_ratio=ship_params['collision_ratio']
    )
    
    result = run_single_simulation(
        use_absolute_bearings=sim_params.get('use_absolute_bearings', True),
        ownship_config=ownship_config_dict,
        ship_config=ship_config,
        goal_config=goal_config_dict,
        time_steps=sim_params.get('max_time_steps', 20000),
        delta_time=sim_params.get('delta_time', 0.01), 
        ALPHA_TRIG=sim_params.get('alpha_nav', 1.0)
    )
    
    # 比較結果
    print(f"\n✅ 重測結果:")
    print(f"  碰撞時間: {result.get('collision_time')}")
    print(f"  到達時間: {result.get('arrival_time')}")
    print(f"  最小表面距離: {np.min(result['distances']):.6f}m")
    print(f"  模擬總時間: {result.get('simulation_time'):.2f}s")
    
    # 計算差異
    if original_results.get('min_distance') is not None:
        distance_diff = abs(np.min(result['distances']) - original_results['min_distance'])
        print(f"\n📊 距離差異: {distance_diff:.6f}m")
    
    # 顯示配置對比
    print(f"\n📋 配置對比:")
    print(f"  Delta time: {sim_params.get('delta_time', 0.01)}")
    print(f"  Max steps: {sim_params.get('max_time_steps', 20000)}")
    print(f"  Alpha nav: {sim_params.get('alpha_nav', 1.0)}")
    print(f"  Use absolute bearings: {sim_params.get('use_absolute_bearings', True)}")
    
    # 顯示圖表
    if show_plot:
        plot_results(result, sim_params.get('delta_time', 0.01), f"Backtest {Path(txt_file).stem} - ")
    
    return result

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python simple_backtest.py <txt文件路徑> [--no-plot]")
        print("範例: python simple_backtest.py results/collision/00092.txt")
        print("\\n現在完全從txt文件讀取配置，不需要config.py")
        sys.exit(1)
    
    txt_file = sys.argv[1]
    show_plot = "--no-plot" not in sys.argv
    
    if not Path(txt_file).exists():
        print(f"文件不存在: {txt_file}")
        sys.exit(1)
    
    backtest_single_case(txt_file, show_plot)
