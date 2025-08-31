"""
Analysis and visualization tools for Monte Carlo simulation results.
Provides statistical analysis and plotting capabilities.
"""

import numpy as np
import matplotlib.pyplot as plt
import os
from . import config

try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False

def plot_single_row_original(result, delta_time, row=1, title_prefix="", save_path=None, show_plot=True):
    """
    Plot a single row of 5 subplots using the original visualization from BearingRateGraph_comparison.py
    This function is extracted from the original file for individual trajectory analysis.
    """
    fig = plt.figure(figsize=(25, 6))  # Adjusted height for single row
    
    # 1. Ship Positions
    plt.subplot(1, 5, 1)
    ownship_line, = plt.plot(result['ownship_positions'][:, 1], result['ownship_positions'][:, 0], label='Ownship')
    ship_line, = plt.plot(result['ship_positions'][:, 1], result['ship_positions'][:, 0], label='Ship A')
    
    # Add direction arrows at final positions
    if len(result['ownship_positions']) > 1:
        plt.annotate('', xy=(result['ownship_positions'][-1, 1], result['ownship_positions'][-1, 0]), 
                    xytext=(result['ownship_positions'][-2, 1], result['ownship_positions'][-2, 0]), 
                    arrowprops=dict(arrowstyle='->', color=ownship_line.get_color()))
        plt.annotate('', xy=(result['ship_positions'][-1, 1], result['ship_positions'][-1, 0]), 
                    xytext=(result['ship_positions'][-2, 1], result['ship_positions'][-2, 0]), 
                    arrowprops=dict(arrowstyle='->', color=ship_line.get_color()))
    
    # Add ship size circles and goal
    ownship_circle = plt.Circle((result['ownship_positions'][-1, 1], result['ownship_positions'][-1, 0]),
        result['ownship_size'] / 2, color=ownship_line.get_color(), fill=False, linestyle='--', alpha=1.0)
    ship_circle = plt.Circle((result['ship_positions'][-1, 1], result['ship_positions'][-1, 0]),
        result['ship_size'] / 2, color=ship_line.get_color(), fill=False, linestyle='--', alpha=1.0)
    goal_circle = plt.Circle((result['goal'].position[1], result['goal'].position[0]),
        1, color='Red', fill=False, linestyle='--', alpha=1.0)
    plt.gca().add_patch(goal_circle)
    plt.gca().add_patch(ownship_circle)
    plt.gca().add_patch(ship_circle)
    
    # Add arrows at 10-second intervals
    time_interval = 10.0
    point_interval = int(time_interval / delta_time)
    arrow_length = 5.0
    
    if point_interval > 0:
        for i in range(0, len(result['ownship_positions']), point_interval):
            # Plot position points
            plt.plot(result['ownship_positions'][i, 1], result['ownship_positions'][i, 0], 'o', 
                    color=ownship_line.get_color(), markersize=6, alpha=0.7)
            plt.plot(result['ship_positions'][i, 1], result['ship_positions'][i, 0], 'o', 
                    color=ship_line.get_color(), markersize=6, alpha=0.7)

            # Add three arrows for ownship
            ownship_pos = result['ownship_positions'][i]
            ownship_heading = result['ownship_headings'][i] if i < len(result['ownship_headings']) else result['ownship_headings'][-1]
            
            # North arrow (red)
            north_end_x = ownship_pos[1]
            north_end_y = ownship_pos[0] + arrow_length
            plt.annotate('', xy=(north_end_x, north_end_y), 
                        xytext=(ownship_pos[1], ownship_pos[0]), 
                        arrowprops=dict(arrowstyle='->', color='red', lw=1.5, alpha=0.7))
            
            # Heading arrow
            heading_end_x = ownship_pos[1] + arrow_length * np.sin(np.radians(ownship_heading))
            heading_end_y = ownship_pos[0] + arrow_length * np.cos(np.radians(ownship_heading))
            plt.annotate('', xy=(heading_end_x, heading_end_y), 
                        xytext=(ownship_pos[1], ownship_pos[0]), 
                        arrowprops=dict(arrowstyle='->', color=ownship_line.get_color(), lw=1.5, alpha=0.7))
            
            # Arrow to other ship
            ship_pos = result['ship_positions'][i]
            direction_to_ship = ship_pos - ownship_pos
            if np.linalg.norm(direction_to_ship) > 0:
                direction_to_ship_normalized = direction_to_ship / np.linalg.norm(direction_to_ship)
                ship_arrow_end = ownship_pos + arrow_length * direction_to_ship_normalized
                plt.annotate('', xy=(ship_arrow_end[1], ship_arrow_end[0]), 
                            xytext=(ownship_pos[1], ownship_pos[0]), 
                            arrowprops=dict(arrowstyle='->', color=ship_line.get_color(), lw=1.5, alpha=0.7))
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title(f'{title_prefix}Ship Positions')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # 2. Bearing Plot
    plt.subplot(1, 5, 2)
    plt.plot(result['bearings'], np.arange(len(result['bearings'])) * delta_time, label='Relative Bearing', alpha=1.0)
    absolute_bearings_normalized = result['absolute_bearings'].copy()
    absolute_bearings_normalized[absolute_bearings_normalized > 180] -= 360
    plt.plot(absolute_bearings_normalized, np.arange(len(absolute_bearings_normalized)) * delta_time, label='Absolute Bearing', linewidth=1)
    half_angular_sizes = result['angular_sizes'] / 2
    plt.plot(result['bearings'] - half_angular_sizes, np.arange(len(result['bearings'])) * delta_time, '--', alpha=0.9)
    plt.plot(result['bearings'] + half_angular_sizes, np.arange(len(result['bearings'])) * delta_time, '--', alpha=0.9)
    plt.ylabel('Time (s)')
    plt.xlabel('Bearing (degrees)')
    plt.title(f'{title_prefix}Bearing')
    plt.xlim(-181, 180)
    plt.axvline(x=0, color='r', linestyle='--')
    for angle in [45, 90, 135, 180, -45, -90, -135, -180]:
        plt.axvline(x=angle, color='g' if angle % 90 != 0 else 'b', linestyle='--')
    plt.legend()
    plt.grid(True)
    
    # 3. Angular Size Plot
    plt.subplot(1, 5, 3)
    plt.plot(np.arange(len(result['angular_sizes'])) * delta_time, result['angular_sizes'], label='Angular Size')
    
    max_angular_size = np.max(result['angular_sizes'])
    max_time_index = np.argmax(result['angular_sizes'])
    max_time = max_time_index * delta_time
    
    plt.axhline(y=max_angular_size, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Max: {max_angular_size:.3f}°')
    plt.axvline(x=max_time, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Time: {max_time:.1f}s')
    
    plt.annotate(f'Max: {max_angular_size:.3f}° at {max_time:.1f}s', 
                xy=(max_time, max_angular_size), 
                xytext=(max_time + 3, max_angular_size),
                arrowprops=dict(arrowstyle='->', color='red', alpha=0.8),
                fontsize=8, color='red', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8))
    
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Size (degrees)')
    plt.title(f'{title_prefix}Angular Size')
    plt.legend()
    plt.grid(True)
    
    # 4. Distance Plot
    plt.subplot(1, 5, 4)
    plt.plot(np.arange(len(result['distances'])) * delta_time, result['distances'], label='Distance')
    
    min_distance = np.min(result['distances'])
    min_distance_time_index = np.argmin(result['distances'])
    min_distance_time = min_distance_time_index * delta_time
    
    plt.axhline(y=min_distance, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Min: {min_distance:.1f}m')
    plt.axvline(x=min_distance_time, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Time: {min_distance_time:.1f}s')
    
    plt.annotate(f'Min: {min_distance:.1f}m at {min_distance_time:.1f}s', 
                xy=(min_distance_time, min_distance), 
                xytext=(min_distance_time + 3, min_distance),
                arrowprops=dict(arrowstyle='->', color='red', alpha=0.8),
                fontsize=8, color='red', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8))
    
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title(f'{title_prefix}Distance')
    plt.legend()
    plt.grid(True)
    
    # 5. Ship Velocities
    plt.subplot(1, 5, 5)
    plt.plot(np.arange(len(result['ownship_velocities'])) * delta_time, result['ownship_velocities'], 
             label='Ownship Velocity', color='blue')
    plt.plot(np.arange(len(result['ship_velocities'])) * delta_time, result['ship_velocities'], 
             label='Ship A Velocity', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title(f'{title_prefix}Velocities')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        if not show_plot:
            plt.close(fig)  # Close to save memory
            return
    
    if show_plot:
        plt.show()
    else:
        plt.close(fig)

def save_individual_trajectory(result, scenario, metrics, simulation_id, output_dir):
    """Save individual trajectory plot to categorized folders.
    
    Args:
        result: Simulation result dictionary
        scenario: Scenario parameters
        metrics: Performance metrics
        simulation_id: Unique simulation identifier
        output_dir: Base output directory
    """
    # Determine outcome category
    outcome = result.get('simulation_outcome', 'unknown')
    if outcome == 'success':
        category = 'successful'
    elif outcome == 'collision':
        category = 'collision'
    else:
        category = 'timeout'
    
    # Create category directory
    category_dir = os.path.join(output_dir, category)
    os.makedirs(category_dir, exist_ok=True)
    
    # Generate filename
    filename = f"{simulation_id:05d}.png"
    save_path = os.path.join(category_dir, filename)
    
    # Create title with key information
    control_method = "Absolute" if result['use_absolute_bearings'] else "Relative"
    title_prefix = (f"Sim {simulation_id:05d} - {control_method} - {outcome.title()} - "
                   f"MinDist: {metrics['min_distance']:.1f}m - ")
    
    # Plot using original function
    plot_single_row_original(
        result=result,
        delta_time=config.DELTA_TIME,
        row=1,
        title_prefix=title_prefix,
        save_path=save_path,
        show_plot=config.SHOW_INDIVIDUAL_PLOTS
    )
    
    return save_path

def save_simulation_parameters(result, scenario, metrics, simulation_id, output_dir):
    """Save simulation initial parameters to categorized text files.
    
    Args:
        result: Simulation result dictionary
        scenario: Scenario parameters
        metrics: Performance metrics
        simulation_id: Unique simulation identifier  
        output_dir: Base output directory
        
    Returns:
        str: Path to saved parameter file
    """
    # Determine outcome category
    outcome = result.get('simulation_outcome', 'unknown')
    if outcome == 'success':
        category = 'successful'
    elif outcome == 'collision':
        category = 'collision'
    else:
        category = 'timeout'
    
    # Create category directory
    category_dir = os.path.join(output_dir, category)
    os.makedirs(category_dir, exist_ok=True)
    
    # Create filename with zero-padded ID
    filename = f"{simulation_id:05d}.txt"
    filepath = os.path.join(category_dir, filename)
    
    # Compile all simulation parameters
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write(f"SIMULATION PARAMETERS - CASE {simulation_id:05d}\n")
        f.write("=" * 60 + "\n")
        f.write(f"Simulation Outcome: {outcome.upper()}\n")
        f.write(f"Timestamp: {metrics.get('timestamp', 'N/A')}\n\n")
        
        # Ownship Initial Parameters
        f.write("OWNSHIP INITIAL PARAMETERS\n")
        f.write("-" * 30 + "\n")
        ownship_start = scenario.get('ownship_start', [0, 0, 0])
        f.write(f"Name: Ownship\n")
        f.write(f"Initial Position: [{ownship_start[0]:.3f}, {ownship_start[1]:.3f}, {ownship_start[2]:.3f}] (North, East, Down)\n")
        f.write(f"Initial Velocity: {config.OWNSHIP_VELOCITY:.3f} m/s\n")
        f.write(f"Initial Heading: 0.000° (North)\n")
        f.write(f"Initial Rate of Turn: 0.000°/s\n")
        f.write(f"Ship Size: {config.OWNSHIP_SIZE:.3f} m\n")
        f.write(f"Max Rate of Turn: {config.OWNSHIP_MAX_RATE_OF_TURN} °/s\n\n")
        
        # Goal Parameters
        f.write("GOAL PARAMETERS\n")
        f.write("-" * 15 + "\n")
        goal_pos = scenario.get('goal_position', config.GOAL_POSITION)
        f.write(f"Goal Position: [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}] (North, East, Down)\n")
        goal_distance = np.sqrt((goal_pos[0] - ownship_start[0])**2 + (goal_pos[1] - ownship_start[1])**2)
        f.write(f"Distance to Goal: {goal_distance:.3f} m\n\n")
        
        # Ship A Initial Parameters
        f.write("SHIP A INITIAL PARAMETERS\n")
        f.write("-" * 25 + "\n")
        ship_a_start = scenario.get('ship_a_start', [0, 0, 0])
        ship_a_params = scenario.get('ship_a_params', {})
        f.write(f"Name: Ship A\n")
        f.write(f"Initial Position: [{ship_a_start[0]:.3f}, {ship_a_start[1]:.3f}, {ship_a_start[2]:.3f}] (North, East, Down)\n")
        f.write(f"Initial Velocity: {ship_a_params.get('velocity', 'N/A'):.3f} m/s\n")
        f.write(f"Initial Heading: {ship_a_params.get('heading', 'N/A'):.3f}°\n")
        f.write(f"Initial Rate of Turn: 0.000°/s\n")
        f.write(f"Ship Size: {ship_a_params.get('size', 'N/A'):.3f} m\n")
        f.write(f"Max Rate of Turn: {ship_a_params.get('max_rate_of_turn', 'N/A')} °/s\n")
        f.write(f"Velocity Limit: {ship_a_params.get('velocity_limit', 'N/A')} m/s\n\n")
        
        # Collision Scenario Parameters
        f.write("COLLISION SCENARIO PARAMETERS\n")
        f.write("-" * 30 + "\n")
        collision_point = scenario.get('collision_point', [0, 0, 0])
        f.write(f"Planned Collision Point: [{collision_point[0]:.3f}, {collision_point[1]:.3f}, {collision_point[2]:.3f}]\n")
        
        # Calculate collision zone ratio
        total_distance = goal_distance
        collision_distance = np.sqrt((collision_point[0] - ownship_start[0])**2 + (collision_point[1] - ownship_start[1])**2)
        collision_ratio = collision_distance / total_distance if total_distance > 0 else 0
        f.write(f"Collision Zone Ratio: {collision_ratio:.1%} of path to goal\n")
        f.write(f"Distance to Collision Point: {collision_distance:.3f} m\n\n")
        
        # Simulation Configuration
        f.write("SIMULATION CONFIGURATION\n")
        f.write("-" * 25 + "\n")
        f.write(f"Control Method: {'Absolute Bearings' if metrics.get('use_absolute_bearings', True) else 'Relative Bearings'}\n")
        f.write(f"Delta Time: {config.DELTA_TIME:.3f} s\n")
        f.write(f"Max Time Steps: {config.MAX_TIME_STEPS}\n")
        f.write(f"Max Simulation Time: {config.MAX_SIMULATION_TIME:.1f} s\n")
        f.write(f"Collision Threshold: {config.COLLISION_THRESHOLD:.3f} m\n")
        f.write(f"Success Distance Threshold: {config.SUCCESS_MIN_DISTANCE_THRESHOLD:.3f} m\n")
        f.write(f"Goal Reach Threshold: {config.SUCCESS_GOAL_REACH_THRESHOLD:.3f} m\n\n")
        
        # Simulation Results Summary
        f.write("SIMULATION RESULTS SUMMARY\n")
        f.write("-" * 27 + "\n")
        f.write(f"Final Outcome: {outcome.upper()}\n")
        f.write(f"Mission Success: {'YES' if metrics.get('mission_success', False) else 'NO'}\n")
        f.write(f"Collision Avoided: {'YES' if metrics.get('collision_avoided', False) else 'NO'}\n")
        f.write(f"Goal Reached: {'YES' if metrics.get('goal_reached', False) else 'NO'}\n")
        f.write(f"Minimum Distance: {metrics.get('min_distance', 'N/A'):.3f} m\n")
        f.write(f"Time to Complete: {metrics.get('simulation_time', 'N/A'):.1f} s\n")
        f.write(f"Final Ownship Position: {metrics.get('final_ownship_position', 'N/A')}\n")
        f.write(f"Final Ship A Position: {metrics.get('final_ship_a_position', 'N/A')}\n")
        f.write(f"Total Trajectory Points: {len(result.get('ownship_trajectory', []))}\n\n")
        
        # Distance Analysis
        f.write("DISTANCE ANALYSIS\n")
        f.write("-" * 17 + "\n")
        distances = result.get('distances', [])
        if len(distances) > 0:  # Use len() instead of direct bool evaluation
            f.write(f"Initial Distance: {distances[0]:.3f} m\n")
            f.write(f"Minimum Distance: {min(distances):.3f} m\n")
            f.write(f"Final Distance: {distances[-1]:.3f} m\n")
            f.write(f"Distance at Collision Zone: {distances[len(distances)//2]:.3f} m (estimated)\n\n")
        
        # Reproducibility Parameters
        f.write("REPRODUCIBILITY PARAMETERS\n")
        f.write("-" * 27 + "\n")
        f.write(f"Random Seed Used: {scenario.get('random_seed', 'N/A')}\n")
        f.write(f"Simulation Index: {simulation_id}\n")
        f.write(f"Scenario ID: {scenario.get('scenario_id', 'N/A')}\n")
        f.write("=" * 60 + "\n")
    
    return filepath

def analyze_results(results_summary):
    """Perform comprehensive analysis of simulation results.
    
    Args:
        results_summary: Results summary from MonteCarloRunner
        
    Returns:
        analysis: Dictionary containing detailed analysis
    """
    if not results_summary['results']:
        return {"error": "No simulation results to analyze"}
    
    analysis = {}
    results = results_summary['results']
    stats = results_summary['statistics']
    
    # Basic performance summary
    analysis['performance_summary'] = {
        'total_simulations': len(results),
        'mission_success_rate': stats.get('mission_success_rate', 0),
        'collision_avoidance_rate': stats.get('collision_avoidance_rate', 0),
        'goal_reach_rate': stats.get('goal_reach_rate', 0),
        'control_method': 'Absolute' if results[0]['metrics']['use_absolute_bearings'] else 'Relative'
    }
    
    # Distance analysis
    min_distances = [r['metrics']['min_distance'] for r in results]
    analysis['distance_analysis'] = {
        'mean_min_distance': np.mean(min_distances),
        'std_min_distance': np.std(min_distances),
        'median_min_distance': np.median(min_distances),
        'percentiles': {
            '5th': np.percentile(min_distances, 5),
            '25th': np.percentile(min_distances, 25),
            '75th': np.percentile(min_distances, 75),
            '95th': np.percentile(min_distances, 95)
        },
        'dangerous_encounters': sum(1 for d in min_distances if d <= config.COLLISION_THRESHOLD),
        'close_calls': sum(1 for d in min_distances if config.COLLISION_THRESHOLD < d <= config.SUCCESS_MIN_DISTANCE_THRESHOLD)
    }
    
    # Parameter sensitivity analysis
    analysis['parameter_sensitivity'] = analyze_parameter_sensitivity(results)
    
    # Failure analysis
    analysis['failure_analysis'] = analyze_failures(results)
    
    return analysis

def analyze_parameter_sensitivity(results):
    """Analyze how different parameters affect simulation outcomes.
    
    Args:
        results: List of simulation results
        
    Returns:
        sensitivity: Dictionary containing parameter sensitivity analysis
    """
    sensitivity = {}
    
    # Extract parameters and outcomes
    ship_a_sizes = [r['metrics']['ship_a_size'] for r in results]
    ship_a_velocities = [r['metrics']['ship_a_velocity'] for r in results]
    ship_a_headings = [r['metrics']['ship_a_heading'] for r in results]
    collision_ratios = [r['metrics']['collision_ratio'] for r in results]
    
    mission_success = [r['metrics']['mission_success'] for r in results]
    min_distances = [r['metrics']['min_distance'] for r in results]
    
    # Size sensitivity
    sensitivity['size_impact'] = {
        'correlation_with_success': np.corrcoef(ship_a_sizes, mission_success)[0,1],
        'correlation_with_min_distance': np.corrcoef(ship_a_sizes, min_distances)[0,1]
    }
    
    # Velocity sensitivity  
    sensitivity['velocity_impact'] = {
        'correlation_with_success': np.corrcoef(ship_a_velocities, mission_success)[0,1],
        'correlation_with_min_distance': np.corrcoef(ship_a_velocities, min_distances)[0,1]
    }
    
    # Heading sensitivity (convert to continuous metric)
    heading_complexity = [abs(np.sin(np.radians(h))) + abs(np.cos(np.radians(h))) for h in ship_a_headings]
    sensitivity['heading_impact'] = {
        'correlation_with_success': np.corrcoef(heading_complexity, mission_success)[0,1],
        'correlation_with_min_distance': np.corrcoef(heading_complexity, min_distances)[0,1]
    }
    
    # Collision position sensitivity
    sensitivity['collision_position_impact'] = {
        'correlation_with_success': np.corrcoef(collision_ratios, mission_success)[0,1], 
        'correlation_with_min_distance': np.corrcoef(collision_ratios, min_distances)[0,1]
    }
    
    return sensitivity

def analyze_failures(results):
    """Analyze failed simulations to identify common patterns.
    
    Args:
        results: List of simulation results
        
    Returns:
        failure_analysis: Dictionary containing failure pattern analysis
    """
    failed_results = [r for r in results if not r['metrics']['mission_success']]
    successful_results = [r for r in results if r['metrics']['mission_success']]
    
    if not failed_results:
        return {"message": "No failures to analyze"}
    
    failure_analysis = {
        'total_failures': len(failed_results),
        'failure_rate': len(failed_results) / len(results)
    }
    
    # Categorize failure types
    collision_failures = [r for r in failed_results if r['metrics']['collision_occurred']]
    goal_miss_failures = [r for r in failed_results if not r['metrics']['goal_reached']]
    unsafe_passage = [r for r in failed_results if not r['metrics']['safe_passage'] and not r['metrics']['collision_occurred']]
    
    failure_analysis['failure_types'] = {
        'collisions': len(collision_failures),
        'goal_misses': len(goal_miss_failures),
        'unsafe_passages': len(unsafe_passage)
    }
    
    # Compare parameter distributions for failures vs successes
    if successful_results:
        failure_analysis['parameter_comparison'] = compare_parameter_distributions(
            failed_results, successful_results
        )
    
    return failure_analysis

def compare_parameter_distributions(failed_results, successful_results):
    """Compare parameter distributions between failed and successful runs.
    
    Args:
        failed_results: List of failed simulation results
        successful_results: List of successful simulation results
        
    Returns:
        comparison: Dictionary containing distribution comparisons
    """
    comparison = {}
    
    # Extract parameters for both groups
    params = ['ship_a_size', 'ship_a_velocity', 'ship_a_heading', 'collision_ratio']
    
    for param in params:
        failed_values = [r['metrics'][param] for r in failed_results]
        success_values = [r['metrics'][param] for r in successful_results]
        
        comparison[param] = {
            'failed_mean': np.mean(failed_values),
            'success_mean': np.mean(success_values),
            'failed_std': np.std(failed_values),
            'success_std': np.std(success_values),
            'mean_difference': np.mean(failed_values) - np.mean(success_values)
        }
    
    return comparison

def plot_results_overview(results_summary, save_path=None):
    """Create overview plots of simulation results.
    
    Args:
        results_summary: Results summary from MonteCarloRunner
        save_path: Optional path to save the figure
    """
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(f'Monte Carlo Simulation Results Overview\n'
                f'Control Method: {"Absolute" if results_summary["results"][0]["metrics"]["use_absolute_bearings"] else "Relative"} Bearings',
                fontsize=16)
    
    results = results_summary['results']
    
    # 1. Success Rate Pie Chart
    mission_success = [r['metrics']['mission_success'] for r in results]
    success_rate = np.mean(mission_success)
    
    axes[0,0].pie([success_rate, 1-success_rate], 
                  labels=[f'Success\n{success_rate:.1%}', f'Failure\n{1-success_rate:.1%}'],
                  colors=['green', 'red'], autopct='%1.1f%%')
    axes[0,0].set_title('Mission Success Rate')
    
    # 2. Minimum Distance Distribution
    min_distances = [r['metrics']['min_distance'] for r in results]
    axes[0,1].hist(min_distances, bins=50, alpha=0.7, edgecolor='black')
    axes[0,1].axvline(config.COLLISION_THRESHOLD, color='red', linestyle='--', 
                      label=f'Collision Threshold ({config.COLLISION_THRESHOLD}m)')
    axes[0,1].axvline(config.SUCCESS_MIN_DISTANCE_THRESHOLD, color='orange', linestyle='--',
                      label=f'Safe Distance ({config.SUCCESS_MIN_DISTANCE_THRESHOLD}m)')
    axes[0,1].set_xlabel('Minimum Distance (m)')
    axes[0,1].set_ylabel('Frequency')
    axes[0,1].set_title('Minimum Distance Distribution')
    axes[0,1].legend()
    
    # 3. Parameter Impact - Ship A Size vs Success
    ship_a_sizes = [r['metrics']['ship_a_size'] for r in results]
    success_colors = ['green' if s else 'red' for s in mission_success]
    axes[0,2].scatter(ship_a_sizes, min_distances, c=success_colors, alpha=0.6)
    axes[0,2].set_xlabel('Ship A Size (m)')
    axes[0,2].set_ylabel('Minimum Distance (m)')
    axes[0,2].set_title('Ship A Size vs Minimum Distance')
    axes[0,2].axhline(config.SUCCESS_MIN_DISTANCE_THRESHOLD, color='orange', linestyle='--')
    
    # 4. Parameter Impact - Ship A Velocity vs Success  
    ship_a_velocities = [r['metrics']['ship_a_velocity'] for r in results]
    axes[1,0].scatter(ship_a_velocities, min_distances, c=success_colors, alpha=0.6)
    axes[1,0].set_xlabel('Ship A Velocity (m/s)')
    axes[1,0].set_ylabel('Minimum Distance (m)')
    axes[1,0].set_title('Ship A Velocity vs Minimum Distance')
    axes[1,0].axhline(config.SUCCESS_MIN_DISTANCE_THRESHOLD, color='orange', linestyle='--')
    
    # 5. Collision Position Analysis
    collision_ratios = [r['metrics']['collision_ratio'] for r in results]
    axes[1,1].scatter(collision_ratios, min_distances, c=success_colors, alpha=0.6)
    axes[1,1].set_xlabel('Collision Position Ratio')
    axes[1,1].set_ylabel('Minimum Distance (m)')
    axes[1,1].set_title('Collision Position vs Minimum Distance')
    axes[1,1].axhline(config.SUCCESS_MIN_DISTANCE_THRESHOLD, color='orange', linestyle='--')
    
    # 6. Path Efficiency vs Success
    path_efficiencies = [r['metrics']['path_efficiency'] for r in results]
    axes[1,2].scatter(path_efficiencies, min_distances, c=success_colors, alpha=0.6)
    axes[1,2].set_xlabel('Path Efficiency')
    axes[1,2].set_ylabel('Minimum Distance (m)')
    axes[1,2].set_title('Path Efficiency vs Minimum Distance')
    axes[1,2].axhline(config.SUCCESS_MIN_DISTANCE_THRESHOLD, color='orange', linestyle='--')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Overview plot saved to {save_path}")
    
    plt.show()

def plot_trajectory_samples(results_summary, num_samples=5, save_path=None):
    """Plot sample trajectories from simulation results.
    
    Args:
        results_summary: Results summary from MonteCarloRunner
        num_samples: Number of sample trajectories to plot
        save_path: Optional path to save the figure
    """
    results = results_summary['results']
    
    # Select sample cases: mix of successes and failures
    successful_results = [r for r in results if r['metrics']['mission_success']]
    failed_results = [r for r in results if not r['metrics']['mission_success']]
    
    num_success_samples = min(num_samples // 2, len(successful_results))
    num_failure_samples = min(num_samples - num_success_samples, len(failed_results))
    
    sample_results = []
    if successful_results:
        sample_results.extend(np.random.choice(successful_results, num_success_samples, replace=False))
    if failed_results:
        sample_results.extend(np.random.choice(failed_results, num_failure_samples, replace=False))
    
    fig, axes = plt.subplots(1, len(sample_results), figsize=(5*len(sample_results), 5))
    if len(sample_results) == 1:
        axes = [axes]
    
    fig.suptitle('Sample Trajectories', fontsize=16)
    
    for i, result in enumerate(sample_results):
        ax = axes[i]
        sim_result = result['simulation_result']
        scenario = result['scenario']
        metrics = result['metrics']
        
        # Plot trajectories
        ownship_pos = sim_result['ownship_positions']
        ship_a_pos = sim_result['ship_positions']
        
        ax.plot(ownship_pos[:, 1], ownship_pos[:, 0], 'b-', linewidth=2, label='Ownship')
        ax.plot(ship_a_pos[:, 1], ship_a_pos[:, 0], 'r-', linewidth=2, label='Ship A')
        
        # Mark start and end positions
        ax.plot(ownship_pos[0, 1], ownship_pos[0, 0], 'bo', markersize=8, label='Ownship Start')
        ax.plot(ownship_pos[-1, 1], ownship_pos[-1, 0], 'bs', markersize=8, label='Ownship End')
        ax.plot(ship_a_pos[0, 1], ship_a_pos[0, 0], 'ro', markersize=8, label='Ship A Start')
        ax.plot(ship_a_pos[-1, 1], ship_a_pos[-1, 0], 'rs', markersize=8, label='Ship A End')
        
        # Mark goal and collision point
        goal_pos = scenario['goal_position']
        collision_point = scenario['collision_point']
        
        ax.plot(goal_pos[1], goal_pos[0], 'g*', markersize=12, label='Goal')
        ax.plot(collision_point[1], collision_point[0], 'kx', markersize=10, label='Planned Collision')
        
        # Draw ship size circles at closest approach
        min_dist_idx = np.argmin(sim_result['distances'])
        ownship_closest = ownship_pos[min_dist_idx]
        ship_a_closest = ship_a_pos[min_dist_idx]
        
        ownship_circle = plt.Circle((ownship_closest[1], ownship_closest[0]), 
                                   sim_result['ownship_size']/2, fill=False, color='blue', linestyle='--')
        ship_a_circle = plt.Circle((ship_a_closest[1], ship_a_closest[0]),
                                  sim_result['ship_size']/2, fill=False, color='red', linestyle='--')
        ax.add_patch(ownship_circle)
        ax.add_patch(ship_a_circle)
        
        # Formatting
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title(f'Case {i+1}: {"Success" if metrics["mission_success"] else "Failure"}\n'
                    f'Min Dist: {metrics["min_distance"]:.1f}m')
        ax.grid(True)
        ax.axis('equal')
        ax.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Trajectory samples saved to {save_path}")
    
    plt.show()

def generate_report(results_summary, analysis, output_file="simulation_report.txt"):
    """Generate a comprehensive text report of simulation results.
    
    Args:
        results_summary: Results summary from MonteCarloRunner
        analysis: Analysis results from analyze_results()
        output_file: Output filename for the report
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("MONTE CARLO COLLISION AVOIDANCE SIMULATION REPORT\n")
        f.write("=" * 60 + "\n\n")
        
        # Run Information
        run_info = results_summary['run_info']
        f.write("RUN INFORMATION\n")
        f.write("-" * 20 + "\n")
        f.write(f"Total simulations: {results_summary['num_simulations']}\n")
        f.write(f"Control method: {'Absolute' if run_info['use_absolute_bearings'] else 'Relative'} bearings\n")
        f.write(f"Runtime: {run_info['total_runtime']:.1f} seconds\n")
        f.write(f"Random seed: {run_info.get('random_seed', 'None')}\n\n")
        
        # Performance Summary
        perf = analysis['performance_summary']
        f.write("PERFORMANCE SUMMARY\n")
        f.write("-" * 20 + "\n")
        f.write(f"Mission success rate: {perf['mission_success_rate']:.1%}\n")
        f.write(f"Collision avoidance rate: {perf['collision_avoidance_rate']:.1%}\n")
        f.write(f"Goal reach rate: {perf['goal_reach_rate']:.1%}\n\n")
        
        # Distance Analysis
        dist = analysis['distance_analysis']
        f.write("DISTANCE ANALYSIS\n")
        f.write("-" * 20 + "\n")
        f.write(f"Mean minimum distance: {dist['mean_min_distance']:.2f} ± {dist['std_min_distance']:.2f} m\n")
        f.write(f"Median minimum distance: {dist['median_min_distance']:.2f} m\n")
        f.write(f"Distance percentiles:\n")
        for k, v in dist['percentiles'].items():
            f.write(f"  {k}: {v:.2f} m\n")
        f.write(f"Dangerous encounters (≤{config.COLLISION_THRESHOLD}m): {dist['dangerous_encounters']}\n")
        f.write(f"Close calls ({config.COLLISION_THRESHOLD}m-{config.SUCCESS_MIN_DISTANCE_THRESHOLD}m): {dist['close_calls']}\n\n")
        
        # Parameter Sensitivity
        sens = analysis['parameter_sensitivity']
        f.write("PARAMETER SENSITIVITY\n")
        f.write("-" * 20 + "\n")
        f.write("Correlations with mission success:\n")
        f.write(f"  Ship A size: {sens['size_impact']['correlation_with_success']:.3f}\n")
        f.write(f"  Ship A velocity: {sens['velocity_impact']['correlation_with_success']:.3f}\n")
        f.write(f"  Ship A heading: {sens['heading_impact']['correlation_with_success']:.3f}\n")
        f.write(f"  Collision position: {sens['collision_position_impact']['correlation_with_success']:.3f}\n\n")
        
        # Failure Analysis
        if 'failure_analysis' in analysis and 'total_failures' in analysis['failure_analysis']:
            fail = analysis['failure_analysis']
            f.write("FAILURE ANALYSIS\n")
            f.write("-" * 20 + "\n")
            f.write(f"Total failures: {fail['total_failures']} ({fail['failure_rate']:.1%})\n")
            f.write("Failure types:\n")
            for failure_type, count in fail['failure_types'].items():
                f.write(f"  {failure_type}: {count}\n")
            f.write("\n")
        
        f.write("Report generated successfully.\n")
    
    print(f"Comprehensive report saved to {output_file}")
