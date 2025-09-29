"""
Monte Carlo Simulation Runner for Ship Collision Avoidance
使用 BearingRateGraph_comparison.py 的函數進行大量模擬
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
import os
import json
from datetime import datetime
import sys
from pathlib import Path
import time

# 添加當前目錄到 Python 路徑
sys.path.append(str(Path(__file__).parent))

# 導入配置和模擬函數
from config import *

from BearingRateGraph_comparison import (
    calculate_ship_initial_position_with_turning,
    run_single_simulation,
    print_simulation_summary
)

class MonteCarloRunner:
    """蒙地卡羅模擬運行器"""
    
    def __init__(self):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.results_dir = Path(f"results/results_{self.timestamp}")
        self.setup_directories()
        self.simulation_results = []
        
        # 從 config 創建 ownship 和 goal 配置
        self.ownship_config = {
            "name": "Ownship",
            "velocity": OWNSHIP_VELOCITY,
            "acceleration": 0,
            "heading": 0.0,
            "rate_of_turn": 0,
            "position": OWNSHIP_INITIAL_POSITION,
            "size": OWNSHIP_SIZE,
            "max_rate_of_turn": OWNSHIP_MAX_RATE_OF_TURN
        }
        
        self.goal_config = {
            "name": "Goal",
            "velocity": 0,
            "acceleration": 0,
            "heading": 0.0,
            "rate_of_turn": 0,
            "position": GOAL_POSITION,
            "size": 0.1  # 很小的虛擬大小
        }
        
        self.sim_params = {
            "delta_time": DELTA_TIME,
            "max_time_steps": MAX_TIME_STEPS,
            "use_absolute_bearings": USE_ABSOLUTE_BEARINGS,
            "alpha_nav": ALPHA_NAV
        }

    @staticmethod
    def compute_alpha_trigger_distance(ship_size_m: float, alpha_deg: float) -> float:
        """由角徑閾值與目標尺寸推回觸發距離（中心距離）。
        角徑公式：alpha = 2 * atan(size / (2*d)) => d = size / (2 * tan(alpha/2)).
        若 alpha<=0 則回傳 0（不過濾）。
        """
        if alpha_deg is None or alpha_deg <= 0:
            return 0.0
        alpha_rad = np.radians(alpha_deg)
        denom = np.tan(alpha_rad / 2.0)
        if abs(denom) < 1e-12:
            return float('inf')
        return ship_size_m / (2.0 * denom)
        
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
        
        print(f"Random seed set to: {RANDOM_SEED}")
        
        return {
            'velocity': np.random.uniform(SHIP_A_VELOCITY_RANGE[0], SHIP_A_VELOCITY_RANGE[1]),
            'heading': np.random.uniform(SHIP_A_HEADING_RANGE[0], SHIP_A_HEADING_RANGE[1]),
            'size': np.random.uniform(SHIP_A_SIZE_RANGE[0], SHIP_A_SIZE_RANGE[1]),
            'collision_ratio': np.random.uniform(COLLISION_ZONE_START_RATIO, COLLISION_ZONE_END_RATIO),
            'rate_of_turn': np.random.uniform(SHIP_A_MAX_RATE_OF_TURN[0], SHIP_A_MAX_RATE_OF_TURN[1])  # 隨機轉彎率
        }
    
    def run_single_monte_carlo(self, sim_id):
        """執行單次蒙地卡羅模擬"""
        print(f"\n{'='*60}")
        print(f"Monte Carlo Simulation #{sim_id:05d}")
        print(f"{'='*60}")
        
        # 生成隨機參數，若 Ship A 初始位置太靠近 Ownship 則重抽參數
        max_attempts = 50
        attempt = 0
        while True:
            attempt += 1
            ship_params = self.generate_random_ship_parameters()

            # 創建碰撞場景（使用正確的 ownship 和 goal 配置）
            initial_pos, collision_point, collision_time = calculate_ship_initial_position_with_turning(
                self.ownship_config,
                self.goal_config,
                ship_params['velocity'], 
                ship_params['heading'],
                ship_params['rate_of_turn'],
                ship_params['collision_ratio']
            )

            # 以角徑閾值估算觸發距離；若初始中心距離小於該距離（或最小生成距離）則跳過
            ownship_start = np.array(self.ownship_config["position"], dtype=float)
            ship_start = np.array(initial_pos, dtype=float)
            center_distance = float(np.linalg.norm(ship_start - ownship_start))

            alpha_trigger_d = self.compute_alpha_trigger_distance(ship_params['size'], ALPHA_NAV)
            required_min_d = max(alpha_trigger_d, SHIP_A_MIN_SPAWN_DISTANCE)

            if center_distance < required_min_d:
                print(
                    f"Skip init too close: d={center_distance:.2f} < min={required_min_d:.2f} "
                    f"(alpha-trigger={alpha_trigger_d:.2f}, min_spawn={SHIP_A_MIN_SPAWN_DISTANCE:.2f}) "
                    f"[attempt {attempt}/{max_attempts}]"
                )
                if attempt >= max_attempts:
                    print("[警告] 多次嘗試仍過近，放寬條件僅使用最小生成距離判定。")
                    # 只用最小生成距離再判一次，若仍過近就接受（避免無限循環）
                    if center_distance >= SHIP_A_MIN_SPAWN_DISTANCE:
                        break
                    else:
                        print("[警告] 距離仍過近，接受當前樣本以不中止批次（後續統計可剔除）。")
                        break
                continue
            else:
                break
        
        ship_config = {
            "name": "Ship A",
            "velocity": ship_params['velocity'],
            "acceleration": 0,
            "heading": ship_params['heading'],
            "rate_of_turn": ship_params['rate_of_turn'],
            "position": initial_pos,
            "size": ship_params['size'],
            "max_rate_of_turn": [12, 12]
        }
        
        # 添加碰撞預測資訊
        ship_config["_collision_info"] = {
            "predicted_collision_point": collision_point,
            "predicted_collision_time": collision_time,
            "collision_ratio": ship_params['collision_ratio'],
            "motion_type": "circular" if abs(ship_params['rate_of_turn']) > 1e-10 else "linear"
        }
        
        # 顯示碰撞預測信息（恢復原本的詳細輸出）
        print(f"=== Collision Prediction Info ===")
        print(f"Ship A Initial Position: [{initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f}]")
        print(f"Predicted Collision Point: [{collision_point[0]:.2f}, {collision_point[1]:.2f}, {collision_point[2]:.2f}]")
        print(f"Predicted Collision Time: {collision_time:.2f}s")
        print(f"Collision Location: {ship_params['collision_ratio']*100:.1f}% of Ownship Path")
        print(f"Ship A Motion Type: {ship_config['_collision_info']['motion_type']}")
        
        # 執行模擬
        result = run_single_simulation(
            use_absolute_bearings=USE_ABSOLUTE_BEARINGS,
            ownship_config=self.ownship_config,
            ship_config=ship_config,
            goal_config=self.goal_config,
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
        
        # 獲取實際碰撞位置（最小距離點）
        min_dist_idx = np.argmin(full_result['distances'])
        actual_collision_ownship_pos = full_result['ownship_positions'][min_dist_idx]
        actual_collision_ship_pos = full_result['ship_positions'][min_dist_idx]
        actual_collision_midpoint = (actual_collision_ownship_pos + actual_collision_ship_pos) / 2
        
        # 儲存參數文件
        param_file = save_dir / f"{sim_id:05d}.txt"
        with open(param_file, 'w', encoding='utf-8') as f:
            f.write(f"Monte Carlo Simulation #{sim_id:05d}\n")
            f.write(f"Result Type: {result_type}\n")
            f.write(f"{'='*50}\n\n")
            
            # Ship A 參數（包含初始位置）
            f.write("Ship A Parameters:\n")
            for key, value in simulation_data['ship_parameters'].items():
                f.write(f"  {key}: {value}\n")
            
            # 從 collision_info 取得初始位置
            collision_info = simulation_data['collision_info']
            if collision_info and 'predicted_collision_point' in collision_info:
                # 取得 Ship A 的初始位置（從模擬結果）
                ship_initial_pos = full_result['ship_positions'][0]
                f.write(f"  initial_position: [{ship_initial_pos[0]:.3f}, {ship_initial_pos[1]:.3f}, {ship_initial_pos[2]:.3f}]\n")
            
            f.write(f"\nCollision Prediction:\n")
            if collision_info:
                f.write(f"  Predicted collision point: {collision_info.get('predicted_collision_point', 'N/A')}\n")
                f.write(f"  Predicted collision time: {collision_info.get('predicted_collision_time', 'N/A')}\n")
                f.write(f"  Motion type: {collision_info.get('motion_type', 'N/A')}\n")
            
            # 實際碰撞位置
            f.write(f"\nActual Collision Location:\n")
            f.write(f"  Ownship position at min distance: [{actual_collision_ownship_pos[0]:.3f}, {actual_collision_ownship_pos[1]:.3f}, {actual_collision_ownship_pos[2]:.3f}]\n")
            f.write(f"  Ship A position at min distance: [{actual_collision_ship_pos[0]:.3f}, {actual_collision_ship_pos[1]:.3f}, {actual_collision_ship_pos[2]:.3f}]\n")
            f.write(f"  Midpoint at min distance: [{actual_collision_midpoint[0]:.3f}, {actual_collision_midpoint[1]:.3f}, {actual_collision_midpoint[2]:.3f}]\n")
            f.write(f"  Time at min distance: {min_dist_idx * DELTA_TIME:.3f}s\n")
            
            # Ownship 配置
            f.write(f"\nOwnship Configuration:\n")
            f.write(f"  initial_position: {OWNSHIP_INITIAL_POSITION}\n")
            f.write(f"  velocity: {OWNSHIP_VELOCITY}\n")
            f.write(f"  size: {OWNSHIP_SIZE}\n")
            f.write(f"  max_rate_of_turn: {OWNSHIP_MAX_RATE_OF_TURN}\n")
            
            # Goal 配置
            f.write(f"\nGoal Configuration:\n")
            f.write(f"  position: {GOAL_POSITION}\n")
            
            # 模擬參數
            f.write(f"\nSimulation Parameters:\n")
            f.write(f"  delta_time: {DELTA_TIME}\n")
            f.write(f"  max_time_steps: {MAX_TIME_STEPS}\n")
            f.write(f"  use_absolute_bearings: {USE_ABSOLUTE_BEARINGS}\n")
            f.write(f"  alpha_nav: {ALPHA_NAV}\n")
            
            f.write(f"\nSimulation Results:\n")
            for key, value in simulation_data['result'].items():
                if key != 'success':
                    f.write(f"  {key}: {value}\n")
        
        # 儲存軌跡圖（如果啟用）
        if SAVE_INDIVIDUAL_TRAJECTORIES:
            self.save_trajectory_plot(
                full_result,
                save_dir / f"{sim_id:05d}.png",
                sim_id,
                result_type,
                simulation_data['ship_parameters'],
                simulation_data.get('collision_info')
            )
    
    def save_trajectory_plot(self, result, filename, sim_id, result_type, ship_params, collision_info=None):
        """儲存個別軌跡圖"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 獲取物體大小信息
        ownship_size = result['ownship_size']
        ship_size = result['ship_size']
        
        # 獲取速度信息（用於動態箭頭大小）
        ownship_velocity = OWNSHIP_VELOCITY
        ship_velocity = ship_params['velocity']
        
        # 繪製軌跡（圖例包含物體大小和速度）
        ax.plot(result['ownship_positions'][:, 1], result['ownship_positions'][:, 0], 
                'b-', linewidth=1, label=f'Ownship (size: {ownship_size:.1f}m, v: {ownship_velocity:.1f}m/s)', alpha=0.8)
        ax.plot(result['ship_positions'][:, 1], result['ship_positions'][:, 0], 
                'r-', linewidth=1, label=f'Ship A (size: {ship_size:.1f}m, v: {ship_velocity:.1f}m/s)', alpha=0.8)
        
        # 在預測碰撞點畫一個紅色 X 標記
        if collision_info and 'predicted_collision_point' in collision_info and collision_info['predicted_collision_point'] is not None:
            try:
                p = collision_info['predicted_collision_point']
                # 轉為 (East, North) => (x, y)
                px_east = p[1]
                py_north = p[0]
                ax.plot(
                    px_east,
                    py_north,
                    marker='x',
                    color='red',
                    markersize=10,
                    markeredgewidth=2,
                    linestyle='None',
                    label='No-Action Collision Point'
                )
            except Exception:
                pass

        # 在軌跡末端添加箭頭
        self.add_trajectory_endpoint_arrows(ax, result['ownship_positions'], 'blue')
        self.add_trajectory_endpoint_arrows(ax, result['ship_positions'], 'red')
        
        # 添加軌跡箭頭（顯示運動方向，使用速度信息自動調整大小）
        self.add_trajectory_section(ax, result['ownship_positions'], 'blue', ownship_velocity)
        self.add_trajectory_section(ax, result['ship_positions'], 'red', ship_velocity)
        
        # 標記起始和結束位置（Ownship start改成綠色）
        ax.plot(result['ownship_positions'][0, 1], result['ownship_positions'][0, 0], 
                'bo', markersize=5, label='Ownship Start')
        ax.plot(
            result['ship_positions'][0, 1],
            result['ship_positions'][0, 0],
            'ro',
            markersize=5,
            label=f"Ship A Start (rate turn: {ship_params.get('rate_of_turn', 0.0):.2f} deg/s)"
        )
        ax.plot(result['goal'].position[1], result['goal'].position[0], 
                'g*', markersize=10, label='Goal')
        
        # 添加最接近點（改小一點）
        min_dist_idx = np.argmin(result['distances'])
        ax.plot(result['ownship_positions'][min_dist_idx, 1], result['ownship_positions'][min_dist_idx, 0], 
                'ko', markersize=2, alpha=0.5)
        ax.plot(result['ship_positions'][min_dist_idx, 1], result['ship_positions'][min_dist_idx, 0], 
                'ko', markersize=2, alpha=0.5)
        
        # 連線顯示最小距離（現在是表面距離）
        ax.plot([result['ownship_positions'][min_dist_idx, 1], result['ship_positions'][min_dist_idx, 1]],
                [result['ownship_positions'][min_dist_idx, 0], result['ship_positions'][min_dist_idx, 0]],
                'k--', alpha=0.5, label=f'Min Surface Distance: {np.min(result["distances"]):.2f}m')
        
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
    
    def add_trajectory_arrows_matplotlib_native(self, ax, positions, color, ship_velocity):
        """使用 matplotlib 原生的 FancyArrowPatch 添加方向箭頭（最佳方案）"""
        if len(positions) < 2:
            return
        
        # 計算每5秒對應的步數間隔
        time_interval_steps = int(5.0 / 0.01)  # 500步
        
        # 根據軸的數據範圍和船舶速度自動計算箭頭大小
        x_range = np.max(positions[:, 1]) - np.min(positions[:, 1])
        y_range = np.max(positions[:, 0]) - np.min(positions[:, 0])
        axis_range = max(x_range, y_range)
        
        # 動態箭頭長度：結合軸範圍和船舶速度
        # 基礎長度：軸範圍的1-2%
        base_length = axis_range * 0.015
        # 速度調整：速度越快，箭頭越長（但有上下限）
        velocity_factor = ship_velocity / 10.0  # 10 m/s為基準速度
        arrow_length = max(5.0, min(80.0, base_length * velocity_factor))
        
        # 箭頭頭部大小也根據長度調整
        head_width = arrow_length * 0.3
        head_length = arrow_length * 0.2
        
        # 在軌跡上每隔指定間隔添加箭頭
        for i in range(time_interval_steps, len(positions), time_interval_steps):
            start_pos = positions[i-1]
            end_pos = positions[i]
            
            # 計算方向向量
            dx = end_pos[1] - start_pos[1]
            dy = end_pos[0] - start_pos[0]
            
            # 檢查是否有足夠的移動
            movement = np.sqrt(dx**2 + dy**2)
            if movement < 0.1:  # 太小的移動跳過
                continue
            
            # 標準化方向並應用箭頭長度
            dx_norm = (dx / movement) * arrow_length
            dy_norm = (dy / movement) * arrow_length
            
            # 使用 FancyArrowPatch 創建箭頭
            arrow = mpatches.FancyArrowPatch(
                (start_pos[1], start_pos[0]),
                (start_pos[1] + dx_norm, start_pos[0] + dy_norm),
                arrowstyle='->', 
                color=color, 
                alpha=0.7,
                linewidth=1.0,
                mutation_scale=head_width
            )
            ax.add_patch(arrow)
        
        # 每隔time_interval_steps個點添加一個箭頭
        for i in range(0, len(positions) - 1, time_interval_steps):
            if i + time_interval_steps < len(positions):
                # 起點和終點
                start_pos = positions[i]
                end_pos = positions[i + time_interval_steps]
                
                # 計算方向向量
                dx = end_pos[1] - start_pos[1]  # East
                dy = end_pos[0] - start_pos[0]  # North
                length = np.sqrt(dx**2 + dy**2)
                
                if length > 0:
                    # 標準化並縮放到合適的長度
                    scale = arrow_length / length
                    dx_scaled = dx * scale
                    dy_scaled = dy * scale
                    
                    # 創建箭頭
                    arrow = FancyArrowPatch(
                        (start_pos[1], start_pos[0]),  # (East, North)
                        (start_pos[1] + dx_scaled, start_pos[0] + dy_scaled),
                        arrowstyle='->', 
                        color=color, 
                        alpha=0.8, 
                        linewidth=1.5,
                        mutation_scale=15  # 箭頭頭部大小
                    )
                    ax.add_patch(arrow)
    
    def add_trajectory_section(self, ax, positions, color, ship_velocity):
        """在軌跡上添加垂直小線段標記（每5秒一個）- 自動調整長度"""
        if len(positions) < 2:
            return
        
        # # 計算每5秒對應的步數間隔（delta_time = 0.01s）
        time_interval_steps = int(5.0 / 0.01)  # 5秒 / 0.01秒 = 500步
        
        # # 計算軌跡的總範圍來動態調整線段長度
        # x_range = np.max(positions[:, 1]) - np.min(positions[:, 1])  # East range
        # y_range = np.max(positions[:, 0]) - np.min(positions[:, 0])  # North range
        # total_range = max(x_range, y_range)
        
        # # 動態線段長度：總範圍的 1-3%，但限制在合理範圍內
        # line_length = max(5.0, min(100.0, total_range * 0.02))
        
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
                    line_length = ship_velocity / 3
                    
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
    start_ts = time.perf_counter()
    runner = MonteCarloRunner()
    runner.run_monte_carlo_batch()
    elapsed = time.perf_counter() - start_ts
    count = len(runner.simulation_results)
    print("=" * 50)
    print(f"Total elapsed time: {elapsed:.2f}s")
    if count > 0:
        print(f"Average per simulation: {elapsed / count:.2f}s ({count} runs)")

if __name__ == "__main__":
    main()
