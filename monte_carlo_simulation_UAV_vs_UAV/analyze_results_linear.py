"""
Monte Carlo Results Analyzer and Visualizer
Analyze Monte Carlo simulation results and generate overview plots
"""

import numpy as np
import matplotlib.pyplot as plt
import json
from pathlib import Path
from datetime import datetime
import glob

class MonteCarloAnalyzer:
    """Monte Carlo results analyzer"""
    
    def __init__(self, results_dir):
        self.results_dir = Path(results_dir)
        self.timestamp = self.results_dir.name.split('_', 1)[1]
        self.results = self.load_all_results()
        
    def load_all_results(self):
        """Load all simulation results"""
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
        """Parse a single result file"""
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
                    
                    # Try converting values to appropriate types
                    try:
                        if value.lower() == 'none':
                            value = None
                        elif value.lower() == 'true':
                            value = True
                        elif value.lower() == 'false':
                            value = False
                        elif value.startswith('[') and value.endswith(']'):
                            # Handle list format [x, y, z]
                            value = value[1:-1]  # 移除 []
                            if ',' in value:
                                # Split and convert into a numeric list
                                value = [float(x.strip()) for x in value.split(',')]
                            else:
                                value = [float(value)]
                        else:
                            # Handle values with units (e.g., "21.59s")
                            if value.endswith('s') and len(value) > 1:
                                try:
                                    value = float(value[:-1])  # Remove 's' and convert to float
                                except ValueError:
                                    pass
                            else:
                                # Try converting to a number
                                if '.' in value:
                                    value = float(value)
                                else:
                                    try:
                                        value = int(value)
                                    except ValueError:
                                        pass  # Keep as string
                    except:
                        pass
                    
                    data[current_section][key] = value
        
        return data
    
    def generate_overview_plots(self):
        """Generate overview analysis plots"""
        # Set fonts (ensure minus sign renders correctly)
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        
        # Use a 3x2 layout (six plots)
        fig, axes = plt.subplots(3, 2, figsize=(8, 9))
        # Subtitle: show total simulations and percentages by result type
        counts = {k: len(self.results.get(k, [])) for k in ['successful', 'collision', 'timeout']}
        total = sum(counts.values())
        if total > 0:
            success_pct = counts['successful'] / total * 100
            collision_pct = counts['collision'] / total * 100
            timeout_pct = counts['timeout'] / total * 100
        else:
            success_pct = collision_pct = timeout_pct = 0.0
        subtitle = (
            f"Total: {total} | "
            f"Success: {counts['successful']} ({success_pct:.1f}%) | "
            f"Collision: {counts['collision']} ({collision_pct:.1f}%) | "
            f"Timeout: {counts['timeout']} ({timeout_pct:.1f}%)"
        )
        fig.suptitle(subtitle, fontsize=16)


        # Collect all results
        all_results = []
        for result_type, results_list in self.results.items():
            all_results.extend(results_list)
        
        if not all_results:
            print("No results to analyze!")
            return
        
        # Minimum distance distribution histogram
        ax_min_dist = axes[0, 0]
        min_distances = []
        for result in all_results:
            if 'sim_results' in result and 'min_distance' in result['sim_results']:
                min_distances.append(result['sim_results']['min_distance'])
        
        if min_distances:
            ax_min_dist.hist(min_distances, bins=50, alpha=0.7, edgecolor='black', color='orange')
            ax_min_dist.axvline(np.mean(min_distances), color='red', linestyle='--', 
                                label=f'Mean: {np.mean(min_distances):.2f}m')
            ax_min_dist.set_xlabel('Minimum Distance (m)')
            ax_min_dist.set_ylabel('Frequency')
            ax_min_dist.set_title('Minimum Distance Distribution')
            ax_min_dist.legend()
            ax_min_dist.grid(True, alpha=0.3)

        # Total simulation time distribution histogram (new)
        ax_sim_time = axes[0, 1]
        sim_times = []
        for result in all_results:
            if 'sim_results' in result and 'simulation_time' in result['sim_results']:
                sim_times.append(result['sim_results']['simulation_time'])

        if sim_times:
            ax_sim_time.hist(sim_times, bins=50, alpha=0.7, edgecolor='black', color='steelblue')
            ax_sim_time.axvline(np.mean(sim_times), color='red', linestyle='--',
                                label=f'Mean: {np.mean(sim_times):.2f}s')
            ax_sim_time.set_xlabel('Simulation Time (s)')
            ax_sim_time.set_ylabel('Frequency')
            ax_sim_time.set_title('Goal Reached Time Distribution')
            ax_sim_time.legend()
            ax_sim_time.grid(True, alpha=0.3)
        
        # Target Ship parameter distribution - velocity
        ax_vel = axes[1, 0]
        velocities = []
        for result in all_results:
            if 'ship_params' in result and 'velocity' in result['ship_params']:
                velocities.append(result['ship_params']['velocity'])
        
        if velocities:
            ax_vel.hist(velocities, bins=50, alpha=0.7, edgecolor='black', color='skyblue')
            ax_vel.set_xlabel('Target Ship Velocity (m/s)')
            ax_vel.set_ylabel('Frequency')
            ax_vel.set_title('Target Ship Velocity Distribution')
            ax_vel.grid(True, alpha=0.3)
        
        # Target Ship parameter distribution - heading
        ax_heading = axes[1, 1]
        headings = []
        for result in all_results:
            if 'ship_params' in result and 'heading' in result['ship_params']:
                headings.append(result['ship_params']['heading'])
        
        if headings:
            ax_heading.hist(headings, bins=50, alpha=0.7, edgecolor='black', color='lightgreen')
            ax_heading.set_xlabel('Target Ship Heading (degrees)')
            ax_heading.set_ylabel('Frequency')
            ax_heading.set_title('Target Ship Heading Distribution')
            ax_heading.grid(True, alpha=0.3)
        
        # Collision location distribution
        ax_collision_loc = axes[2, 0]
        collision_ratios = []
        for result in all_results:
            if 'ship_params' in result and 'collision_ratio' in result['ship_params']:
                collision_ratios.append(result['ship_params']['collision_ratio'] * 100)  # Convert to percentage
        
        if collision_ratios:
            ax_collision_loc.hist(collision_ratios, bins=50, alpha=0.7, edgecolor='black', color='coral')
            ax_collision_loc.set_xlabel('Collision Location (% of Ownship Path)')
            ax_collision_loc.set_ylabel('Frequency')
            ax_collision_loc.set_title('Collision Location Distribution')
            ax_collision_loc.grid(True, alpha=0.3)
        
        # Target Ship size distribution
        ax_size = axes[2, 1]
        sizes = []
        for result in all_results:
            if 'ship_params' in result and 'size' in result['ship_params']:
                sizes.append(result['ship_params']['size'])
        
        if sizes:
            ax_size.hist(sizes, bins=50, alpha=0.7, edgecolor='black', color='purple')
            ax_size.set_xlabel('Target Ship Size (m)')
            ax_size.set_ylabel('Frequency')
            ax_size.set_title('Target Ship Size Distribution')
            ax_size.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
    # Save overview plot
        overview_file = self.results_dir / f"overview_{self.timestamp}.png"
        plt.savefig(overview_file, dpi=600, bbox_inches='tight')
        print(f"Overview plot saved to: {overview_file}")
        plt.close()
    
    def generate_trajectory_samples(self, max_samples=5):
        """Generate trajectory sample plots"""
        # Select samples from each result category
        sample_results = {}
        for result_type, results_list in self.results.items():
            if results_list:
                # Take the first few samples
                sample_results[result_type] = results_list[:min(max_samples, len(results_list))]
        
        if not any(sample_results.values()):
            print("No trajectory samples to plot!")
            return
        
    # Create a figure for each category with its samples
        for result_type, samples in sample_results.items():
            if not samples:
                continue
                
            fig, axes = plt.subplots(1, len(samples), figsize=(5*len(samples), 4))
            if len(samples) == 1:
                axes = [axes]
            
            fig.suptitle(f'{result_type.title()} Trajectory Samples - {self.timestamp}', fontsize=16)
            
            for i, sample in enumerate(samples):
                ax = axes[i]
                sim_id = sample.get('simulation_id', i+1)
                
                # Load the corresponding trajectory image if present
                png_file = self.results_dir / result_type / f"{sim_id:05d}.png"
                if png_file.exists():
                    # Display the existing trajectory image
                    img = plt.imread(png_file)
                    ax.imshow(img)
                    ax.axis('off')
                else:
                    # If no image, show parameter summary text
                    ax.text(0.5, 0.5, f'Simulation #{sim_id:05d}\n\n' + 
                           self.format_sample_info(sample), 
                           ha='center', va='center', transform=ax.transAxes,
                           fontsize=10, bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray"))
                    ax.set_xlim(0, 1)
                    ax.set_ylim(0, 1)
                
                ax.set_title(f'Sample {i+1} (#{sim_id:05d})')
            
            plt.tight_layout()
            
            # Save sample figure
            sample_file = self.results_dir / f"trajectories_{result_type}_{self.timestamp}.png"
            plt.savefig(sample_file, dpi=300, bbox_inches='tight')
            print(f"Trajectory samples for {result_type} saved to: {sample_file}")
            plt.close()
    
    def format_sample_info(self, sample):
        """Format sample info for display"""
        info_parts = []
        
        if 'ship_params' in sample:
            ship_params = sample['ship_params']
            info_parts.append(f"Velocity: {ship_params.get('velocity', 'N/A'):.2f} m/s")
            info_parts.append(f"Heading: {ship_params.get('heading', 'N/A'):.1f}°")
            info_parts.append(f"Size: {ship_params.get('size', 'N/A'):.2f} m")
            
            # Show rate of turn
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
        """Print detailed statistics summary"""
        print(f"\n{'='*60}")
        print(f"Monte Carlo Analysis Summary - {self.timestamp}")
        print(f"{'='*60}")
        
        total_sims = sum(len(results) for results in self.results.values())
        
        for result_type, results_list in self.results.items():
            count = len(results_list)
            percentage = (count / total_sims * 100) if total_sims > 0 else 0
            print(f"{result_type.title():>12}: {count:>3} ({percentage:>5.1f}%)")
        
        print(f"{'Total':>12}: {total_sims:>3}")
        
    # Aggregated statistics
        all_results = []
        for results_list in self.results.values():
            all_results.extend(results_list)
        
        if all_results:
            # Minimum distance statistics
            min_distances = [r['sim_results']['min_distance'] for r in all_results 
                           if 'sim_results' in r and 'min_distance' in r['sim_results']]
            
            if min_distances:
                print(f"\nDistance Statistics:")
                print(f"  Mean minimum distance: {np.mean(min_distances):.3f} m")
                print(f"  Std minimum distance:  {np.std(min_distances):.3f} m")
                print(f"  Min minimum distance:  {np.min(min_distances):.3f} m")
                print(f"  Max minimum distance:  {np.max(min_distances):.3f} m")
            
            # Target Ship parameter statistics
            velocities = [r['ship_params']['velocity'] for r in all_results 
                         if 'ship_params' in r and 'velocity' in r['ship_params']]
            headings = [r['ship_params']['heading'] for r in all_results 
                       if 'ship_params' in r and 'heading' in r['ship_params']]
            sizes = [r['ship_params']['size'] for r in all_results 
                    if 'ship_params' in r and 'size' in r['ship_params']]
            rates_of_turn = [r['ship_params']['rate_of_turn'] for r in all_results 
                           if 'ship_params' in r and 'rate_of_turn' in r['ship_params']]
            
            if velocities:
                print(f"\nTarget Ship Parameter Statistics:")
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
                # Proportion of straight-line motion
                straight_line_count = sum(1 for rot in rates_of_turn if abs(rot) < 0.01)
                print(f"               - Straight line motion: {straight_line_count}/{len(rates_of_turn)} ({straight_line_count/len(rates_of_turn)*100:.1f}%)")
            
            # Actual collision location statistics
            collision_times = [r['actual_collision']['Time at min distance'] for r in all_results 
                             if 'actual_collision' in r and 'Time at min distance' in r['actual_collision']]
            
            if collision_times:
                print(f"\nActual Collision Location Statistics:")
                print(f"  Time at min distance - Mean: {np.mean(collision_times):.2f}s, Std: {np.std(collision_times):.2f}s")
                print(f"                       - Range: {np.min(collision_times):.2f}s - {np.max(collision_times):.2f}s")
            
            # Configuration summary
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
    """Main function - analyze the latest results"""
    results_base_dir = Path("results")
    
    if not results_base_dir.exists():
        print("No results directory found!")
        return
    
    # Find the latest results directory
    result_dirs = list(results_base_dir.glob("results_*"))
    if not result_dirs:
        print("No result directories found!")
        return
    
    latest_dir = max(result_dirs, key=lambda x: x.name)
    print(f"Analyzing results from: {latest_dir}")
    
    # Create analyzer and generate reports
    analyzer = MonteCarloAnalyzer(latest_dir)
    analyzer.print_summary()
    analyzer.generate_overview_plots()
    analyzer.generate_trajectory_samples()
    
    print(f"\nAnalysis complete! Check {latest_dir} for output files.")

if __name__ == "__main__":
    main()
