"""
Monte Carlo Simulation Runner for Ship Collision Avoidance
使用 BearingRateGraph_comparison.py 的函數進行大量模擬
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import json
from datetime import datetime
import sys
from pathlib import Path

# 添加當前目錄到 Python 路徑
sys.path.append(str(Path(__file__).parent))

# 導入配置和模擬函數
from config import *
from BearingRateGraph_comparison import (
    create_collision_scenario_with_turning,
    run_single_simulation,
    DEFAULT_OWNSHIP_CONFIG,
    DEFAULT_GOAL_CONFIG,
    DEFAULT_SIM_PARAMS,
    print_simulation_summary
)

class MonteCarloRunner:
    """蒙地卡羅模擬運行器"""
    
    def __init__(self):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_dir = Path(f"results/results_{self.timestamp}")
        self.setup_directories()
        self.simulation_results = []
        
    def setup_directories(self):
        """建立結果目錄結構"""
        directories = [
            self.results_dir,
            self.results_dir / "successful",
            self.results_dir / "collision", 
            self.results_dir / "timeout"
        ]
        
        for directory in directories:
            directory.mkdir(parents=True, exist_ok=True)
        
        print(f"Results will be saved to: {self.results_dir}")
    
    def generate_random_ship_parameters(self):
        """根據配置隨機生成Ship A參數"""
        # 設定隨機種子（如果有指定）
        if RANDOM_SEED is not None:
            np.random.seed(RANDOM_SEED)
        
        return {
            'velocity': np.random.uniform(SHIP_A_VELOCITY_RANGE[0], SHIP_A_VELOCITY_RANGE[1]),
            'heading': np.random.uniform(SHIP_A_HEADING_RANGE[0], SHIP_A_HEADING_RANGE[1]),
            'size': np.random.uniform(SHIP_A_SIZE_RANGE[0], SHIP_A_SIZE_RANGE[1]),
            'collision_ratio': np.random.uniform(COLLISION_ZONE_START_RATIO, COLLISION_ZONE_END_RATIO),
            'rate_of_turn': 0.0  # 目前設為直線運動
        }
    
    def run_single_monte_carlo(self, sim_id):
        """執行單次蒙地卡羅模擬"""
        print(f"\n{'='*60}")
        print(f"Monte Carlo Simulation #{sim_id:05d}")
        print(f"{'='*60}")
        
        # 生成隨機參數
        ship_params = self.generate_random_ship_parameters()
        
        # 創建碰撞場景
        ship_config = create_collision_scenario_with_turning(
            ship_velocity=ship_params['velocity'],
            ship_heading=ship_params['heading'],
            ship_rate_of_turn=ship_params['rate_of_turn'],
            ship_size=ship_params['size'],
            collision_ratio=ship_params['collision_ratio']
        )
        
        # 執行模擬
        result = run_single_simulation(
            use_absolute_bearings=USE_ABSOLUTE_BEARINGS,
            ownship_config=DEFAULT_OWNSHIP_CONFIG,
            ship_config=ship_config,
            goal_config=DEFAULT_GOAL_CONFIG,
            time_steps=MAX_TIME_STEPS,
            delta_time=DELTA_TIME,
            ALPHA_TRIG=ALPHA_NAV
        )
        
        # 分析結果
        simulation_data = {
            'simulation_id': sim_id,
            'ship_parameters': ship_params,
            'collision_info': ship_config.get('_collision_info', {}),
            'result': {
                'collision_time': result.get('collision_time'),
                'arrival_time': result.get('arrival_time'),
                'timeout': result.get('timeout'),
                'simulation_time': result.get('simulation_time'),
                'min_distance': float(np.min(result['distances'])),
                'max_angular_size': float(np.max(result['angular_sizes'])),
                'success': self.classify_result(result)
            }
        }
        
        # 儲存個別結果
        self.save_individual_result(simulation_data, result)
        
        return simulation_data
    
    def classify_result(self, result):
        """分類模擬結果"""
        if result.get('collision_time') is not None:
            return 'collision'
        elif result.get('arrival_time') is not None:
            return 'successful'
        elif result.get('timeout'):
            return 'timeout'
        else:
            return 'unknown'
    
    def save_individual_result(self, simulation_data, full_result):
        """儲存個別模擬結果"""
        sim_id = simulation_data['simulation_id']
        result_type = simulation_data['result']['success']
        
        # 決定儲存目錄
        save_dir = self.results_dir / result_type
        
        # 儲存參數文件
        param_file = save_dir / f"{sim_id:05d}.txt"
        with open(param_file, 'w', encoding='utf-8') as f:
            f.write(f"Monte Carlo Simulation #{sim_id:05d}\n")
            f.write(f"Result Type: {result_type}\n")
            f.write(f"{'='*50}\n\n")
            
            f.write("Ship A Parameters:\n")
            for key, value in simulation_data['ship_parameters'].items():
                f.write(f"  {key}: {value}\n")
            
            f.write(f"\nCollision Prediction:\n")
            collision_info = simulation_data['collision_info']
            if collision_info:
                f.write(f"  Predicted collision point: {collision_info.get('predicted_collision_point', 'N/A')}\n")
                f.write(f"  Predicted collision time: {collision_info.get('predicted_collision_time', 'N/A')}\n")
                f.write(f"  Motion type: {collision_info.get('motion_type', 'N/A')}\n")
            
            f.write(f"\nSimulation Results:\n")
            for key, value in simulation_data['result'].items():
                if key != 'success':
                    f.write(f"  {key}: {value}\n")
        
        # 儲存軌跡圖（如果啟用）
        if SAVE_INDIVIDUAL_TRAJECTORIES:
            self.save_trajectory_plot(full_result, save_dir / f"{sim_id:05d}.png", sim_id, result_type)
    
    def save_trajectory_plot(self, result, filename, sim_id, result_type):
        """儲存個別軌跡圖"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 獲取物體大小信息
        ownship_size = result['ownship_size']
        ship_size = result['ship_size']
        
        # 繪製軌跡（圖例包含物體大小）
        ax.plot(result['ownship_positions'][:, 1], result['ownship_positions'][:, 0], 
                'b-', linewidth=1, label=f'Ownship (size: {ownship_size:.1f}m)', alpha=0.8)
        ax.plot(result['ship_positions'][:, 1], result['ship_positions'][:, 0], 
                'r-', linewidth=1, label=f'Ship A (size: {ship_size:.1f}m)', alpha=0.8)
        
        # 在軌跡末端添加箭頭
        self.add_trajectory_endpoint_arrows(ax, result['ownship_positions'], 'blue')
        self.add_trajectory_endpoint_arrows(ax, result['ship_positions'], 'red')
        
        # 添加軌跡箭頭（顯示運動方向
        self.add_trajectory_arrows(ax, result['ownship_positions'], 'blue')
        self.add_trajectory_arrows(ax, result['ship_positions'], 'red')
        
        # 標記起始和結束位置（Ownship start改成綠色）
        ax.plot(result['ownship_positions'][0, 1], result['ownship_positions'][0, 0], 
                'bo', markersize=5, label='Ownship Start')
        ax.plot(result['ship_positions'][0, 1], result['ship_positions'][0, 0], 
                'ro', markersize=5, label='Ship A Start')
        ax.plot(result['goal'].position[1], result['goal'].position[0], 
                'g*', markersize=10, label='Goal')
        
        # 添加最接近點（改小一點）
        min_dist_idx = np.argmin(result['distances'])
        ax.plot(result['ownship_positions'][min_dist_idx, 1], result['ownship_positions'][min_dist_idx, 0], 
                'ko', markersize=2, alpha=0.75)
        ax.plot(result['ship_positions'][min_dist_idx, 1], result['ship_positions'][min_dist_idx, 0], 
                'ko', markersize=2, alpha=0.75)
        
        # 連線顯示最小距離（現在是表面距離）
        ax.plot([result['ownship_positions'][min_dist_idx, 1], result['ship_positions'][min_dist_idx, 1]],
                [result['ownship_positions'][min_dist_idx, 0], result['ship_positions'][min_dist_idx, 0]],
                'k--', alpha=0.75, label=f'Min Surface Distance: {np.min(result["distances"]):.2f}m')
        
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title(f'Simulation #{sim_id:05d} - {result_type.upper()}')
        ax.legend(loc='upper right')  # 指定圖例位置避免警告
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
    
    def add_trajectory_endpoint_arrows(self, ax, positions, color):
        """在軌跡末端添加箭頭"""
        if len(positions) < 2:
            return
            
        # 計算末端方向向量
        end_pos = positions[-1]
        prev_pos = positions[-2]
        
        dx = end_pos[1] - prev_pos[1]  # East direction
        dy = end_pos[0] - prev_pos[0]  # North direction
        
        # 標準化向量長度
        length = np.sqrt(dx**2 + dy**2)
        if length > 0:
            # 箭頭長度設為1個單位
            arrow_length = 1.0
            dx_norm = dx / length * arrow_length
            dy_norm = dy / length * arrow_length
            
            # 在軌跡末端添加箭頭
            ax.annotate('', 
                       xy=(end_pos[1] + dx_norm, end_pos[0] + dy_norm),
                       xytext=(end_pos[1], end_pos[0]),
                       arrowprops=dict(arrowstyle='->', color=color, alpha=0.8, lw=1.0))
    
    def add_trajectory_arrows(self, ax, positions, color):
        """在軌跡上添加垂直小線段標記（每5秒一個）"""
        if len(positions) < 2:
            return
        
        # 計算每5秒對應的步數間隔（delta_time = 0.01s）
        time_interval_steps = int(5.0 / 0.01)  # 5秒 / 0.01秒 = 500步
        
        # 每隔time_interval_steps個點添加一個垂直線段
        for i in range(0, len(positions) - 1, time_interval_steps):
            if i + 1 < len(positions):
                # 計算軌跡方向向量
                dx = positions[i + 1, 1] - positions[i, 1]  # East direction
                dy = positions[i + 1, 0] - positions[i, 0]  # North direction
                
                # 標準化方向向量
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    # 計算垂直向量（逆時針旋轉90度）
                    perp_dx = -dy / length  # 垂直方向East
                    perp_dy = dx / length   # 垂直方向North
                    
                    # 小線段長度
                    line_length = 50.0
                    
                    # 計算線段兩端點
                    center_x = positions[i, 1]
                    center_y = positions[i, 0]
                    
                    start_x = center_x - perp_dx * line_length / 2
                    start_y = center_y - perp_dy * line_length / 2
                    end_x = center_x + perp_dx * line_length / 2
                    end_y = center_y + perp_dy * line_length / 2
                    
                    # 繪製垂直小線段
                    ax.plot([start_x, end_x], [start_y, end_y], 
                            color=color, linewidth=1, alpha=1.0)
    
    def run_monte_carlo_batch(self):
        """執行完整的蒙地卡羅批次模擬"""
        print(f"Starting Monte Carlo simulation with {NUM_SIMULATIONS} runs...")
        print(f"Using {'Absolute' if USE_ABSOLUTE_BEARINGS else 'Relative'} bearing control")
        
        self.simulation_results = []
        
        for i in range(NUM_SIMULATIONS):
            try:
                result = self.run_single_monte_carlo(i + 1)
                self.simulation_results.append(result)
                
                # 顯示進度
                progress = (i + 1) / NUM_SIMULATIONS * 100
                print(f"Progress: {progress:.1f}% ({i + 1}/{NUM_SIMULATIONS})")
                
            except Exception as e:
                print(f"Error in simulation {i + 1}: {str(e)}")
                continue
        
        # 生成總結報告
        self.generate_summary_report()
        self.save_raw_data()
        
        print(f"\nMonte Carlo simulation completed!")
        print(f"Results saved to: {self.results_dir}")
    
    def generate_summary_report(self):
        """生成統計報告"""
        if not self.simulation_results:
            return
        
        # 統計結果
        successful = [r for r in self.simulation_results if r['result']['success'] == 'successful']
        collisions = [r for r in self.simulation_results if r['result']['success'] == 'collision']
        timeouts = [r for r in self.simulation_results if r['result']['success'] == 'timeout']
        
        # 距離統計
        min_distances = [r['result']['min_distance'] for r in self.simulation_results]
        
        # 生成報告
        report_file = self.results_dir / f"report_{self.timestamp}.txt"
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(f"Monte Carlo Simulation Report\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*60}\n\n")
            
            f.write(f"Simulation Parameters:\n")
            f.write(f"  Number of simulations: {NUM_SIMULATIONS}\n")
            f.write(f"  Bearing control method: {'Absolute' if USE_ABSOLUTE_BEARINGS else 'Relative'}\n")
            f.write(f"  Max simulation time: {MAX_SIMULATION_TIME:.1f}s\n\n")
            
            f.write(f"Results Summary:\n")
            f.write(f"  Successful: {len(successful)} ({len(successful)/len(self.simulation_results)*100:.1f}%)\n")
            f.write(f"  Collisions: {len(collisions)} ({len(collisions)/len(self.simulation_results)*100:.1f}%)\n")
            f.write(f"  Timeouts: {len(timeouts)} ({len(timeouts)/len(self.simulation_results)*100:.1f}%)\n\n")
            
            f.write(f"Distance Statistics:\n")
            f.write(f"  Mean minimum distance: {np.mean(min_distances):.3f}m\n")
            f.write(f"  Std minimum distance: {np.std(min_distances):.3f}m\n")
            f.write(f"  Min minimum distance: {np.min(min_distances):.3f}m\n")
            f.write(f"  Max minimum distance: {np.max(min_distances):.3f}m\n\n")
            
            if successful:
                arrival_times = [r['result']['arrival_time'] for r in successful]
                f.write(f"Successful Cases Statistics:\n")
                f.write(f"  Mean arrival time: {np.mean(arrival_times):.2f}s\n")
                f.write(f"  Std arrival time: {np.std(arrival_times):.2f}s\n")
                f.write(f"  Min arrival time: {np.min(arrival_times):.2f}s\n")
                f.write(f"  Max arrival time: {np.max(arrival_times):.2f}s\n")
        
        print(f"\nStatistical Summary:")
        print(f"  Successful: {len(successful)}/{len(self.simulation_results)} ({len(successful)/len(self.simulation_results)*100:.1f}%)")
        print(f"  Collisions: {len(collisions)}/{len(self.simulation_results)} ({len(collisions)/len(self.simulation_results)*100:.1f}%)")
        print(f"  Timeouts: {len(timeouts)}/{len(self.simulation_results)} ({len(timeouts)/len(self.simulation_results)*100:.1f}%)")
        print(f"  Mean min distance: {np.mean(min_distances):.3f}m")
    
    def save_raw_data(self):
        """儲存原始數據"""
        data_file = self.results_dir / f"simulation_results_{self.timestamp}.npz"
        
        # 整理數據
        data_dict = {}
        for i, result in enumerate(self.simulation_results):
            prefix = f"sim_{i+1:05d}"
            for key, value in result.items():
                if isinstance(value, dict):
                    for subkey, subvalue in value.items():
                        data_dict[f"{prefix}_{key}_{subkey}"] = subvalue
                else:
                    data_dict[f"{prefix}_{key}"] = value
        
        # 儲存
        np.savez_compressed(data_file, **data_dict)
        print(f"Raw data saved to: {data_file}")

def main():
    """主函數"""
    print("Monte Carlo Collision Avoidance Simulation")
    print("=" * 50)
    
    # 創建並執行模擬
    runner = MonteCarloRunner()
    runner.run_monte_carlo_batch()

if __name__ == "__main__":
    main()
