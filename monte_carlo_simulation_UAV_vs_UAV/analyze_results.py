"""
Monte Carlo Results Analyzer and Visualizer
分析蒙地卡羅模擬結果並生成總覽圖表
"""

import numpy as np
import matplotlib.pyplot as plt
import json
from pathlib import Path
from datetime import datetime
import glob

class MonteCarloAnalyzer:
    """蒙地卡羅結果分析器"""
    
    def __init__(self, results_dir):
        self.results_dir = Path(results_dir)
        self.timestamp = self.results_dir.name.split('_', 1)[1]
        self.results = self.load_all_results()
        
    def load_all_results(self):
        """載入所有模擬結果"""
        results = {
            'successful': [],
            'collision': [],
            'timeout': []
        }
        
        for result_type in ['successful', 'collision', 'timeout']:
            result_dir = self.results_dir / result_type
            if result_dir.exists():
                for txt_file in result_dir.glob('*.txt'):
                    try:
                        data = self.parse_result_file(txt_file)
                        data['result_type'] = result_type
                        results[result_type].append(data)
                    except Exception as e:
                        print(f"Error loading {txt_file}: {e}")
        
        return results
    
    def parse_result_file(self, txt_file):
        """解析單一結果文件"""
        data = {}
        current_section = None
        
        with open(txt_file, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('='):
                    continue
                
                if line.startswith('Monte Carlo Simulation #'):
                    data['simulation_id'] = int(line.split('#')[1])
                elif line.startswith('Result Type:'):
                    data['result_type'] = line.split(':')[1].strip()
                elif line == 'Ship A Parameters:':
                    current_section = 'ship_params'
                    data['ship_params'] = {}
                elif line == 'Collision Prediction:':
                    current_section = 'collision_info'
                    data['collision_info'] = {}
                elif line == 'Actual Collision Location:':
                    current_section = 'actual_collision'
                    data['actual_collision'] = {}
                elif line == 'Ownship Configuration:':
                    current_section = 'ownship_config'
                    data['ownship_config'] = {}
                elif line == 'Goal Configuration:':
                    current_section = 'goal_config'
                    data['goal_config'] = {}
                elif line == 'Simulation Parameters:':
                    current_section = 'sim_params'
                    data['sim_params'] = {}
                elif line == 'Simulation Results:':
                    current_section = 'sim_results'
                    data['sim_results'] = {}
                elif ':' in line and current_section:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    # 嘗試轉換數值
                    try:
                        if value.lower() == 'none':
                            value = None
                        elif value.lower() == 'true':
                            value = True
                        elif value.lower() == 'false':
                            value = False
                        elif value.startswith('[') and value.endswith(']'):
                            # 處理列表格式 [x, y, z]
                            value = value[1:-1]  # 移除 []
                            if ',' in value:
                                # 分割並轉換為數字列表
                                value = [float(x.strip()) for x in value.split(',')]
                            else:
                                value = [float(value)]
                        else:
                            # 處理帶單位的數值（如 "21.59s"）
                            if value.endswith('s') and len(value) > 1:
                                try:
                                    value = float(value[:-1])  # 移除 's' 並轉換為數字
                                except ValueError:
                                    pass
                            else:
                                # 嘗試轉換為數字
                                if '.' in value:
                                    value = float(value)
                                else:
                                    try:
                                        value = int(value)
                                    except ValueError:
                                        pass  # 保持為字符串
                    except:
                        pass
                    
                    data[current_section][key] = value
        
        return data
    
    def generate_overview_plots(self):
        """生成總覽分析圖"""
        # 設定中文字體
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        fig, axes = plt.subplots(3, 3, figsize=(18, 18))  # 改為 3x3 布局
        fig.suptitle(f'Monte Carlo Simulation Overview - {self.timestamp}', fontsize=16)
        
        # 收集所有數據
        all_results = []
        for result_type, results_list in self.results.items():
            all_results.extend(results_list)
        
        if not all_results:
            print("No results to analyze!")
            return
        
        # 1. 結果分布餅圖
        ax1 = axes[0, 0]
        result_counts = {k: len(v) for k, v in self.results.items()}
        labels = []
        sizes = []
        colors = ['green', 'red', 'orange']
        
        for i, (result_type, count) in enumerate(result_counts.items()):
            if count > 0:
                labels.append(f'{result_type.title()} ({count})')
                sizes.append(count)
        
        if sizes:
            ax1.pie(sizes, labels=labels, autopct='%1.1f%%', colors=colors[:len(sizes)])
        ax1.set_title('Simulation Results Distribution')
        
        # 2. 最小距離分布直方圖
        ax2 = axes[0, 1]
        min_distances = []
        for result in all_results:
            if 'sim_results' in result and 'min_distance' in result['sim_results']:
                min_distances.append(result['sim_results']['min_distance'])
        
        if min_distances:
            ax2.hist(min_distances, bins=15, alpha=0.7, edgecolor='black')
            ax2.axvline(np.mean(min_distances), color='red', linestyle='--', 
                       label=f'Mean: {np.mean(min_distances):.2f}m')
            ax2.set_xlabel('Minimum Distance (m)')
            ax2.set_ylabel('Frequency')
            ax2.set_title('Minimum Distance Distribution')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        
        # 3. Ship A 參數分佈 - 速度
        ax3 = axes[0, 2]
        velocities = []
        for result in all_results:
            if 'ship_params' in result and 'velocity' in result['ship_params']:
                velocities.append(result['ship_params']['velocity'])
        
        if velocities:
            ax3.hist(velocities, bins=10, alpha=0.7, edgecolor='black', color='skyblue')
            ax3.set_xlabel('Ship A Velocity (m/s)')
            ax3.set_ylabel('Frequency')
            ax3.set_title('Ship A Velocity Distribution')
            ax3.grid(True, alpha=0.3)
        
        # 4. Ship A 參數分佈 - 航向
        ax4 = axes[1, 0]
        headings = []
        for result in all_results:
            if 'ship_params' in result and 'heading' in result['ship_params']:
                headings.append(result['ship_params']['heading'])
        
        if headings:
            ax4.hist(headings, bins=18, alpha=0.7, edgecolor='black', color='lightgreen')
            ax4.set_xlabel('Ship A Heading (degrees)')
            ax4.set_ylabel('Frequency')
            ax4.set_title('Ship A Heading Distribution')
            ax4.grid(True, alpha=0.3)
        
        # 5. Ship A 參數分佈 - 轉彎率 (新增)
        ax5 = axes[1, 1]
        rates_of_turn = []
        for result in all_results:
            if 'ship_params' in result and 'rate_of_turn' in result['ship_params']:
                rates_of_turn.append(result['ship_params']['rate_of_turn'])
        
        if rates_of_turn:
            ax5.hist(rates_of_turn, bins=15, alpha=0.7, edgecolor='black', color='orange')
            ax5.axvline(0, color='red', linestyle='--', alpha=0.8, label='Straight Line')
            ax5.set_xlabel('Ship A Rate of Turn (°/s)')
            ax5.set_ylabel('Frequency')
            ax5.set_title('Ship A Rate of Turn Distribution')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
        
        # 6. 碰撞位置分布
        ax6 = axes[1, 2]
        collision_ratios = []
        for result in all_results:
            if 'ship_params' in result and 'collision_ratio' in result['ship_params']:
                collision_ratios.append(result['ship_params']['collision_ratio'] * 100)  # 轉換為百分比
        
        if collision_ratios:
            ax6.hist(collision_ratios, bins=10, alpha=0.7, edgecolor='black', color='coral')
            ax6.set_xlabel('Collision Location (% of Ownship Path)')
            ax6.set_ylabel('Frequency')
            ax6.set_title('Collision Location Distribution')
            ax6.grid(True, alpha=0.3)
        
        # 7. Ship A 大小分布
        ax7 = axes[2, 0]
        sizes = []
        for result in all_results:
            if 'ship_params' in result and 'size' in result['ship_params']:
                sizes.append(result['ship_params']['size'])
        
        if sizes:
            ax7.hist(sizes, bins=10, alpha=0.7, edgecolor='black', color='purple')
            ax7.set_xlabel('Ship A Size (m)')
            ax7.set_ylabel('Frequency')
            ax7.set_title('Ship A Size Distribution')
            ax7.grid(True, alpha=0.3)
        
        # 8. 結果類型vs最小距離的散點圖
        ax8 = axes[2, 1]
        result_types = []
        min_dists_scatter = []
        colors_scatter = []
        color_map = {'successful': 'green', 'collision': 'red', 'timeout': 'orange'}
        
        for result in all_results:
            if 'sim_results' in result and 'min_distance' in result['sim_results']:
                result_types.append(result['result_type'])
                min_dists_scatter.append(result['sim_results']['min_distance'])
                colors_scatter.append(color_map.get(result['result_type'], 'gray'))
        
        if result_types:
            # 為每個結果類型創建數值
            type_to_num = {'successful': 2, 'timeout': 1, 'collision': 0}
            y_values = [type_to_num[rt] for rt in result_types]
            
            ax8.scatter(min_dists_scatter, y_values, c=colors_scatter, alpha=0.7, s=50)
            ax8.set_xlabel('Minimum Distance (m)')
            ax8.set_ylabel('Result Type')
            ax8.set_yticks([0, 1, 2])
            ax8.set_yticklabels(['Collision', 'Timeout', 'Successful'])
            ax8.set_title('Result Type vs Minimum Distance')
            ax8.grid(True, alpha=0.3)
        
        # 9. 轉彎率 vs 最小距離散點圖 (新增)
        ax9 = axes[2, 2]
        turn_rates_scatter = []
        min_dists_vs_turn = []
        colors_vs_turn = []
        
        for result in all_results:
            if ('ship_params' in result and 'rate_of_turn' in result['ship_params'] and 
                'sim_results' in result and 'min_distance' in result['sim_results']):
                turn_rates_scatter.append(result['ship_params']['rate_of_turn'])
                min_dists_vs_turn.append(result['sim_results']['min_distance'])
                colors_vs_turn.append(color_map.get(result['result_type'], 'gray'))
        
        if turn_rates_scatter:
            ax9.scatter(turn_rates_scatter, min_dists_vs_turn, c=colors_vs_turn, alpha=0.7, s=50)
            ax9.axvline(0, color='red', linestyle='--', alpha=0.5, label='Straight Line')
            ax9.set_xlabel('Ship A Rate of Turn (°/s)')
            ax9.set_ylabel('Minimum Distance (m)')
            ax9.set_title('Turn Rate vs Minimum Distance')
            ax9.legend()
            ax9.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 儲存圖表
        overview_file = self.results_dir / f"overview_{self.timestamp}.png"
        plt.savefig(overview_file, dpi=300, bbox_inches='tight')
        print(f"Overview plot saved to: {overview_file}")
        plt.close()
    
    def generate_trajectory_samples(self, max_samples=5):
        """生成軌跡樣本圖"""
        # 從每個類別選取樣本
        sample_results = {}
        for result_type, results_list in self.results.items():
            if results_list:
                # 選取前幾個樣本
                sample_results[result_type] = results_list[:min(max_samples, len(results_list))]
        
        if not any(sample_results.values()):
            print("No trajectory samples to plot!")
            return
        
        # 為每個樣本創建單獨的圖
        for result_type, samples in sample_results.items():
            if not samples:
                continue
                
            fig, axes = plt.subplots(1, len(samples), figsize=(5*len(samples), 4))
            if len(samples) == 1:
                axes = [axes]
            
            fig.suptitle(f'{result_type.title()} Trajectory Samples - {self.timestamp}', fontsize=14)
            
            for i, sample in enumerate(samples):
                ax = axes[i]
                sim_id = sample.get('simulation_id', i+1)
                
                # 載入對應的軌跡圖片（如果存在）
                png_file = self.results_dir / result_type / f"{sim_id:05d}.png"
                if png_file.exists():
                    # 顯示已存在的軌跡圖
                    img = plt.imread(png_file)
                    ax.imshow(img)
                    ax.axis('off')
                else:
                    # 如果沒有圖片，顯示參數信息
                    ax.text(0.5, 0.5, f'Simulation #{sim_id:05d}\n\n' + 
                           self.format_sample_info(sample), 
                           ha='center', va='center', transform=ax.transAxes,
                           fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"))
                    ax.set_xlim(0, 1)
                    ax.set_ylim(0, 1)
                
                ax.set_title(f'Sample {i+1} (#{sim_id:05d})')
            
            plt.tight_layout()
            
            # 儲存樣本圖
            sample_file = self.results_dir / f"trajectories_{result_type}_{self.timestamp}.png"
            plt.savefig(sample_file, dpi=300, bbox_inches='tight')
            print(f"Trajectory samples for {result_type} saved to: {sample_file}")
            plt.close()
    
    def format_sample_info(self, sample):
        """格式化樣本信息用於顯示"""
        info_parts = []
        
        if 'ship_params' in sample:
            ship_params = sample['ship_params']
            info_parts.append(f"Velocity: {ship_params.get('velocity', 'N/A'):.2f} m/s")
            info_parts.append(f"Heading: {ship_params.get('heading', 'N/A'):.1f}°")
            info_parts.append(f"Size: {ship_params.get('size', 'N/A'):.2f} m")
            
            # 顯示轉彎率
            rate_of_turn = ship_params.get('rate_of_turn', 0.0)
            if abs(rate_of_turn) < 0.01:
                info_parts.append("Motion: Straight line")
            else:
                direction = "Right" if rate_of_turn > 0 else "Left"
                info_parts.append(f"Turn Rate: {abs(rate_of_turn):.2f}°/s ({direction})")
            
            if 'initial_position' in ship_params:
                pos = ship_params['initial_position']
                if isinstance(pos, list) and len(pos) >= 2:
                    info_parts.append(f"Initial: ({pos[0]:.1f}, {pos[1]:.1f})")
        
        if 'actual_collision' in sample:
            actual = sample['actual_collision']
            if 'Time at min distance' in actual:
                info_parts.append(f"Min Dist Time: {actual['Time at min distance']:.1f}s")
        
        if 'sim_results' in sample:
            sim_results = sample['sim_results']
            info_parts.append(f"Min Distance: {sim_results.get('min_distance', 'N/A'):.2f} m")
            if sim_results.get('simulation_time'):
                info_parts.append(f"Sim Time: {sim_results.get('simulation_time', 'N/A'):.1f} s")
        
        return '\n'.join(info_parts)
    
    def print_summary(self):
        """列印詳細統計摘要"""
        print(f"\n{'='*60}")
        print(f"Monte Carlo Analysis Summary - {self.timestamp}")
        print(f"{'='*60}")
        
        total_sims = sum(len(results) for results in self.results.values())
        
        for result_type, results_list in self.results.items():
            count = len(results_list)
            percentage = (count / total_sims * 100) if total_sims > 0 else 0
            print(f"{result_type.title():>12}: {count:>3} ({percentage:>5.1f}%)")
        
        print(f"{'Total':>12}: {total_sims:>3}")
        
        # 統計信息
        all_results = []
        for results_list in self.results.values():
            all_results.extend(results_list)
        
        if all_results:
            # 最小距離統計
            min_distances = [r['sim_results']['min_distance'] for r in all_results 
                           if 'sim_results' in r and 'min_distance' in r['sim_results']]
            
            if min_distances:
                print(f"\nDistance Statistics:")
                print(f"  Mean minimum distance: {np.mean(min_distances):.3f} m")
                print(f"  Std minimum distance:  {np.std(min_distances):.3f} m")
                print(f"  Min minimum distance:  {np.min(min_distances):.3f} m")
                print(f"  Max minimum distance:  {np.max(min_distances):.3f} m")
            
            # Ship A 參數統計
            velocities = [r['ship_params']['velocity'] for r in all_results 
                         if 'ship_params' in r and 'velocity' in r['ship_params']]
            headings = [r['ship_params']['heading'] for r in all_results 
                       if 'ship_params' in r and 'heading' in r['ship_params']]
            sizes = [r['ship_params']['size'] for r in all_results 
                    if 'ship_params' in r and 'size' in r['ship_params']]
            rates_of_turn = [r['ship_params']['rate_of_turn'] for r in all_results 
                           if 'ship_params' in r and 'rate_of_turn' in r['ship_params']]
            
            if velocities:
                print(f"\nShip A Parameter Statistics:")
                print(f"  Velocity - Mean: {np.mean(velocities):.2f} m/s, Std: {np.std(velocities):.2f} m/s")
                print(f"           - Range: {np.min(velocities):.2f} - {np.max(velocities):.2f} m/s")
            if headings:
                print(f"  Heading  - Mean: {np.mean(headings):.1f}°, Std: {np.std(headings):.1f}°")
                print(f"           - Range: {np.min(headings):.1f}° - {np.max(headings):.1f}°")
            if sizes:
                print(f"  Size     - Mean: {np.mean(sizes):.2f} m, Std: {np.std(sizes):.2f} m")
                print(f"           - Range: {np.min(sizes):.2f} - {np.max(sizes):.2f} m")
            if rates_of_turn:
                print(f"  Rate of Turn - Mean: {np.mean(rates_of_turn):.2f}°/s, Std: {np.std(rates_of_turn):.2f}°/s")
                print(f"               - Range: {np.min(rates_of_turn):.2f}° - {np.max(rates_of_turn):.2f}°/s")
                # 統計直線運動的比例
                straight_line_count = sum(1 for rot in rates_of_turn if abs(rot) < 0.01)
                print(f"               - Straight line motion: {straight_line_count}/{len(rates_of_turn)} ({straight_line_count/len(rates_of_turn)*100:.1f}%)")
            
            # 新增：實際碰撞位置統計
            collision_times = [r['actual_collision']['Time at min distance'] for r in all_results 
                             if 'actual_collision' in r and 'Time at min distance' in r['actual_collision']]
            
            if collision_times:
                print(f"\nActual Collision Location Statistics:")
                print(f"  Time at min distance - Mean: {np.mean(collision_times):.2f}s, Std: {np.std(collision_times):.2f}s")
                print(f"                       - Range: {np.min(collision_times):.2f}s - {np.max(collision_times):.2f}s")
            
            # 新增：配置信息摘要
            if all_results and 'sim_params' in all_results[0]:
                sim_params = all_results[0]['sim_params']
                print(f"\nSimulation Configuration:")
                print(f"  Delta time: {sim_params.get('delta_time', 'N/A')}s")
                print(f"  Max time steps: {sim_params.get('max_time_steps', 'N/A')}")
                print(f"  Use absolute bearings: {sim_params.get('use_absolute_bearings', 'N/A')}")
                print(f"  Alpha nav: {sim_params.get('alpha_nav', 'N/A')}")
            
            if all_results and 'ownship_config' in all_results[0]:
                ownship_config = all_results[0]['ownship_config']
                print(f"\nOwnship Configuration:")
                print(f"  Velocity: {ownship_config.get('velocity', 'N/A')} m/s")
                print(f"  Size: {ownship_config.get('size', 'N/A')} m")
                if 'max_rate_of_turn' in ownship_config:
                    rot = ownship_config['max_rate_of_turn']
                    if isinstance(rot, list) and len(rot) >= 2:
                        print(f"  Max rate of turn: [{rot[0]}, {rot[1]}] deg/s")
                    else:
                        print(f"  Max rate of turn: {rot}")
            
            if all_results and 'goal_config' in all_results[0]:
                goal_config = all_results[0]['goal_config']
                if 'position' in goal_config:
                    pos = goal_config['position']
                    if isinstance(pos, list) and len(pos) >= 2:
                        print(f"\nGoal Position: ({pos[0]}, {pos[1]}, {pos[2] if len(pos) > 2 else 0})")
                    else:
                        print(f"\nGoal Position: {pos}")

def main():
    """主函數 - 分析最新的結果"""
    results_base_dir = Path("results")
    
    if not results_base_dir.exists():
        print("No results directory found!")
        return
    
    # 找到最新的結果目錄
    result_dirs = list(results_base_dir.glob("results_*"))
    if not result_dirs:
        print("No result directories found!")
        return
    
    latest_dir = max(result_dirs, key=lambda x: x.name)
    print(f"Analyzing results from: {latest_dir}")
    
    # 創建分析器並生成報告
    analyzer = MonteCarloAnalyzer(latest_dir)
    analyzer.print_summary()
    analyzer.generate_overview_plots()
    analyzer.generate_trajectory_samples()
    
    print(f"\nAnalysis complete! Check {latest_dir} for output files.")

if __name__ == "__main__":
    main()
