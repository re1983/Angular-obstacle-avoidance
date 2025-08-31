"""
完整的蒙地卡羅模擬系統
包含參數保存、結果分析和可重現性功能
"""

import os
import sys
import time
import json
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from typing import Dict, List, Any

# 確保可以導入模組
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from monte_carlo_simulation import config
from monte_carlo_simulation.collision_geometry import create_collision_scenario
from monte_carlo_simulation.cleaned_wrapper_v2 import run_collision_avoidance_simulation
from monte_carlo_simulation.analysis import analyze_results, plot_results_overview, save_individual_trajectory, save_simulation_parameters

class FullMonteCarloRunner:
    """完整的蒙地卡羅模擬器，包含結果保存功能"""
    
    def __init__(self, num_simulations=None, use_absolute_bearings=True, random_seed=None, output_dir="results"):
        self.num_simulations = num_simulations or config.NUM_SIMULATIONS
        self.use_absolute_bearings = use_absolute_bearings
        self.random_seed = random_seed
        self.output_dir = output_dir
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 設定隨機種子
        if random_seed is not None:
            np.random.seed(random_seed)
            
        # 創建結果資料夾
        self.setup_output_directories()
        
    def setup_output_directories(self):
        """創建結果資料夾結構"""
        self.timestamped_dir = os.path.join(self.output_dir, f"run_{self.timestamp}")
        
        directories = [
            self.timestamped_dir,
            os.path.join(self.timestamped_dir, "successful"),
            os.path.join(self.timestamped_dir, "collision"), 
            os.path.join(self.timestamped_dir, "timeout")
        ]
        
        for directory in directories:
            os.makedirs(directory, exist_ok=True)
            
        print(f"📁 結果將保存到: {self.timestamped_dir}")
    
    def run_full_simulation(self):
        """運行完整的蒙地卡羅模擬"""
        print("🚀 開始完整蒙地卡羅模擬...")
        print(f"   模擬次數: {self.num_simulations}")
        print(f"   控制方法: {'絕對方位' if self.use_absolute_bearings else '相對方位'}")
        print(f"   隨機種子: {self.random_seed if self.random_seed else '隨機'}")
        
        start_time = time.time()
        results = []
        statistics = {
            'successful': 0,
            'collision': 0, 
            'timeout': 0,
            'min_distances': [],
            'simulation_times': []
        }
        
        # 運行所有模擬
        for i in range(self.num_simulations):
            print(f"\n[{i+1:04d}/{self.num_simulations}] ", end="")
            
            try:
                # 生成場景
                scenario, is_valid = create_collision_scenario()
                if not is_valid:
                    print("❌ 場景生成失敗")
                    continue
                
                # 添加模擬參數
                scenario['random_seed'] = self.random_seed
                scenario['simulation_index'] = i + 1
                
                # 運行模擬
                sim_result = run_collision_avoidance_simulation(
                    ownship_position=scenario['ownship_start'].tolist(),
                    ownship_velocity=config.OWNSHIP_VELOCITY,
                    ship_a_position=scenario['ship_a_start'].tolist(),
                    ship_a_velocity=scenario['ship_a_params']['velocity'],
                    ship_a_heading=scenario['ship_a_params']['heading'],
                    goal_position=scenario['goal_position'].tolist(),
                    alpha_nav=config.ALPHA_NAV,
                    ownship_size=config.OWNSHIP_SIZE,
                    ship_a_size=scenario['ship_a_params']['size'],
                    show_plot=False
                )
                
                # 處理結果
                result = self.process_simulation_result(sim_result, scenario, i + 1)
                results.append(result)
                
                # 更新統計
                outcome = result['simulation_result']['outcome']
                statistics[outcome] += 1
                statistics['min_distances'].append(result['metrics']['min_distance'])
                statistics['simulation_times'].append(result['metrics']['simulation_time'])
                
                # 保存個別結果
                if config.SAVE_INDIVIDUAL_PARAMETERS:
                    self.save_individual_result(result, i + 1)
                
                print(f"✅ {outcome} (距離: {result['metrics']['min_distance']:.1f}m)")
                
            except Exception as e:
                print(f"❌ 模擬 {i+1} 失敗: {e}")
                continue
        
        # 完成模擬
        total_time = time.time() - start_time
        
        # 生成總結
        summary = self.generate_summary(results, statistics, total_time)
        
        # 保存結果
        self.save_complete_results(results, summary)
        
        # 生成分析和圖表
        self.generate_analysis_and_plots(results, summary)
        
        print(f"\n🎉 模擬完成！")
        print(f"   總時間: {total_time:.1f}秒")
        print(f"   結果保存到: {self.timestamped_dir}")
        
        return results, summary
    
    def process_simulation_result(self, sim_result, scenario, simulation_id):
        """處理單個模擬結果"""
        # 計算評估指標
        min_distance = sim_result['min_distance']
        outcome = sim_result['outcome']
        
        # 判斷任務成功
        collision_occurred = outcome == 'collision'
        goal_reached = outcome == 'goal_reached'
        safe_passage = min_distance >= config.SUCCESS_MIN_DISTANCE_THRESHOLD
        mission_success = goal_reached and safe_passage and not collision_occurred
        
        # 計算路徑效率
        direct_distance = np.linalg.norm(np.array(scenario['goal_position']) - np.array(scenario['ownship_start']))
        actual_distance = len(sim_result['ownship_trajectory']) * config.OWNSHIP_VELOCITY * config.DELTA_TIME
        path_efficiency = direct_distance / actual_distance if actual_distance > 0 else 0
        
        metrics = {
            'simulation_id': simulation_id,
            'min_distance': min_distance,
            'collision_occurred': collision_occurred,
            'goal_reached': goal_reached,
            'safe_passage': safe_passage,
            'mission_success': mission_success,
            'simulation_time': sim_result['simulation_time'],
            'path_efficiency': path_efficiency,
            'ship_a_size': scenario['ship_a_params']['size'],
            'ship_a_velocity': scenario['ship_a_params']['velocity'],
            'ship_a_heading': scenario['ship_a_params']['heading'],
            'collision_ratio': scenario['collision_ratio'],
            'use_absolute_bearings': self.use_absolute_bearings,
            'final_ownship_position': sim_result['final_ownship_position'],
            'final_ship_a_position': sim_result['final_ship_a_position'],
            'timestamp': datetime.now().isoformat()
        }
        
        return {
            'simulation_result': sim_result,
            'scenario': scenario,
            'metrics': metrics
        }
    
    def save_individual_result(self, result, simulation_id):
        """保存個別模擬結果"""
        # 保存參數文件
        save_simulation_parameters(
            result['simulation_result'],
            result['scenario'], 
            result['metrics'],
            simulation_id,
            self.timestamped_dir
        )
        
        # 保存軌跡圖
        if config.SAVE_INDIVIDUAL_TRAJECTORIES:
            save_individual_trajectory(
                result['simulation_result'],
                result['scenario'],
                result['metrics'], 
                simulation_id,
                self.timestamped_dir
            )
    
    def generate_summary(self, results, statistics, total_time):
        """生成模擬總結"""
        total_sims = len(results)
        
        if total_sims == 0:
            return {'error': '沒有成功的模擬結果'}
        
        summary = {
            'run_info': {
                'timestamp': self.timestamp,
                'num_simulations': self.num_simulations,
                'successful_simulations': total_sims,
                'use_absolute_bearings': self.use_absolute_bearings,
                'random_seed': self.random_seed,
                'total_runtime': total_time,
                'output_directory': self.timestamped_dir
            },
            'statistics': {
                'mission_success_rate': statistics['successful'] / total_sims,
                'collision_rate': statistics['collision'] / total_sims,
                'timeout_rate': statistics['timeout'] / total_sims,
                'collision_avoidance_rate': 1.0 - (statistics['collision'] / total_sims),
                'goal_reach_rate': statistics['successful'] / total_sims,
                'mean_min_distance': np.mean(statistics['min_distances']),
                'std_min_distance': np.std(statistics['min_distances']),
                'mean_simulation_time': np.mean(statistics['simulation_times'])
            },
            'results': results,
            'num_simulations': total_sims
        }
        
        return summary
    
    def save_complete_results(self, results, summary):
        """保存完整結果到文件"""
        # 保存為 NPZ 格式 (數據)
        results_file = os.path.join(self.timestamped_dir, f"simulation_results_{self.timestamp}.npz")
        
        # 提取數據陣列
        min_distances = [r['metrics']['min_distance'] for r in results]
        collision_occurred = [r['metrics']['collision_occurred'] for r in results]
        goal_reached = [r['metrics']['goal_reached'] for r in results]
        ship_a_sizes = [r['metrics']['ship_a_size'] for r in results]
        ship_a_velocities = [r['metrics']['ship_a_velocity'] for r in results]
        
        np.savez(results_file,
                min_distances=min_distances,
                collision_occurred=collision_occurred,
                goal_reached=goal_reached,
                ship_a_sizes=ship_a_sizes,
                ship_a_velocities=ship_a_velocities,
                summary=summary)
        
        # 保存文字報告
        report_file = os.path.join(self.timestamped_dir, f"report_{self.timestamp}.txt")
        self.generate_text_report(summary, report_file)
        
        print(f"📊 結果已保存:")
        print(f"   數據文件: {results_file}")
        print(f"   報告文件: {report_file}")
    
    def generate_text_report(self, summary, output_file):
        """生成文字報告"""
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("🚢 蒙地卡羅碰撞避免模擬報告\n")
            f.write("=" * 60 + "\n\n")
            
            # 運行信息
            run_info = summary['run_info']
            f.write("📋 運行信息\n")
            f.write("-" * 20 + "\n")
            f.write(f"時間戳記: {run_info['timestamp']}\n")
            f.write(f"模擬總數: {run_info['num_simulations']}\n")
            f.write(f"成功模擬: {run_info['successful_simulations']}\n")
            f.write(f"控制方法: {'絕對方位' if run_info['use_absolute_bearings'] else '相對方位'}\n")
            f.write(f"隨機種子: {run_info['random_seed'] if run_info['random_seed'] else '隨機'}\n")
            f.write(f"運行時間: {run_info['total_runtime']:.1f} 秒\n\n")
            
            # 統計結果
            stats = summary['statistics']
            f.write("📊 統計結果\n")
            f.write("-" * 20 + "\n")
            f.write(f"任務成功率: {stats['mission_success_rate']:.1%}\n")
            f.write(f"碰撞率: {stats['collision_rate']:.1%}\n")
            f.write(f"超時率: {stats['timeout_rate']:.1%}\n")
            f.write(f"避碰成功率: {stats['collision_avoidance_rate']:.1%}\n")
            f.write(f"到達目標率: {stats['goal_reach_rate']:.1%}\n")
            f.write(f"平均最小距離: {stats['mean_min_distance']:.2f} ± {stats['std_min_distance']:.2f} m\n")
            f.write(f"平均模擬時間: {stats['mean_simulation_time']:.1f} s\n\n")
            
            # 文件結構說明
            f.write("📂 結果文件結構\n")
            f.write("-" * 20 + "\n")
            f.write("successful/     - 成功避碰的案例\n")
            f.write("collision/      - 發生碰撞的案例\n")
            f.write("timeout/        - 超時的案例\n")
            f.write("*.npz          - 原始數據文件\n")
            f.write("*.png          - 分析圖表\n")
            f.write("*.txt          - 參數和報告文件\n")
    
    def generate_analysis_and_plots(self, results, summary):
        """生成分析和圖表"""
        try:
            # 生成總覽圖
            overview_file = os.path.join(self.timestamped_dir, f"overview_{self.timestamp}.png")
            plot_results_overview(summary, save_path=overview_file)
            
            # 生成軌跡樣本圖
            trajectories_file = os.path.join(self.timestamped_dir, f"trajectories_{self.timestamp}.png")
            # 這需要額外的軌跡採樣功能
            
            print(f"📈 分析圖表已生成:")
            print(f"   總覽圖: {overview_file}")
            
        except Exception as e:
            print(f"⚠️ 圖表生成失敗: {e}")


def main():
    """主函數"""
    print("🚢 完整蒙地卡羅碰撞避免模擬系統")
    print("=" * 60)
    
    # 配置參數
    print("\n⚙️ 當前配置:")
    print(f"   模擬次數: {config.NUM_SIMULATIONS}")
    print(f"   控制方法: {'絕對方位' if config.USE_ABSOLUTE_BEARINGS else '相對方位'}")
    print(f"   保存軌跡圖: {config.SAVE_INDIVIDUAL_TRAJECTORIES}")
    print(f"   保存參數文件: {config.SAVE_INDIVIDUAL_PARAMETERS}")
    
    # 確認運行
    response = input("\n是否使用當前配置運行模擬？(y/n): ").strip().lower()
    if response != 'y':
        print("👋 取消運行")
        return
    
    # 創建並運行模擬
    runner = FullMonteCarloRunner(
        num_simulations=config.NUM_SIMULATIONS,
        use_absolute_bearings=config.USE_ABSOLUTE_BEARINGS,
        random_seed=config.RANDOM_SEED
    )
    
    try:
        results, summary = runner.run_full_simulation()
        
        # 顯示最終結果
        print("\n" + "=" * 60)
        print("🎯 最終結果總結:")
        stats = summary['statistics']
        print(f"   任務成功率: {stats['mission_success_rate']:.1%}")
        print(f"   平均最小距離: {stats['mean_min_distance']:.2f}m")
        print(f"   碰撞事件: {int(stats['collision_rate'] * len(results))} 次")
        print(f"   結果保存位置: {runner.timestamped_dir}")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 用戶中斷模擬")
    except Exception as e:
        print(f"\n❌ 模擬運行失敗: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
