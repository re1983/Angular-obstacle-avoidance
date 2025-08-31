"""
Wrapper for BearingRateGraph_cleaned.py - Authoritative Implementation
Preserves exact original algorithm while allowing parameter modification for Monte Carlo
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, Tuple, Any, List

# Add parent directory to path to import BearingRateGraph_cleaned
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

# Import the authoritative implementation
try:
    from BearingRateGraph_cleaned import (
        ShipStatus, adj_ownship_heading, get_distance_3d, get_bearing, 
        get_absolute_bearing, get_angular_diameter, angle_difference_in_deg
    )
    CLEANED_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import BearingRateGraph_cleaned.py: {e}")
    CLEANED_AVAILABLE = False

def run_collision_avoidance_simulation(
    ownship_position: List[float],
    ownship_velocity: float,
    ship_a_position: List[float], 
    ship_a_velocity: float,
    ship_a_heading: float,
    goal_position: List[float],
    delta_time: float = 0.01,
    max_time_steps: int = 5000,
    alpha_nav: float = 2.0,
    ownship_size: float = 0.5,
    ship_a_size: float = 0.5,
    ownship_max_rate_of_turn: List[float] = [12.0, 12.0],
    ship_a_max_rate_of_turn: List[float] = [0.0, 0.0],
    ship_a_velocity_limit: List[float] = [0.5, 10.0],
    use_absolute_bearings: bool = True,
    show_plot: bool = False,
    save_plot: bool = False,
    plot_filename: str = None
) -> Dict[str, Any]:
    """
    Run collision avoidance simulation using BearingRateGraph_cleaned.py
    
    This wrapper preserves the exact original algorithm while allowing parameter modification.
    """
    
    if not CLEANED_AVAILABLE:
        raise RuntimeError("BearingRateGraph_cleaned.py not available. Cannot run simulation.")
    
    # Create ship objects using ShipStatus class from original
    ownship = ShipStatus(
        name="Ownship",
        velocity=ownship_velocity,
        acceleration=0,
        heading=0,  # Will be updated by controller
        rate_of_turn=0,
        position=ownship_position,
        size=ownship_size,
        max_rate_of_turn=ownship_max_rate_of_turn,
        velocity_limit=[ownship_velocity, ownship_velocity]  # Constant velocity
    )
    
    ship_a = ShipStatus(
        name="Ship A",
        velocity=ship_a_velocity,
        acceleration=0,
        heading=ship_a_heading,
        rate_of_turn=0,  # Ship A maintains straight course
        position=ship_a_position,
        size=ship_a_size,
        max_rate_of_turn=ship_a_max_rate_of_turn,
        velocity_limit=ship_a_velocity_limit
    )
    
    goal = ShipStatus(
        name="Goal",
        velocity=0.0,
        acceleration=0,
        heading=0,
        rate_of_turn=0,
        position=goal_position,
        size=0.1  # Small goal size
    )
    
    # Initialize tracking lists (matching original structure)
    ownship_trajectory = [ownship.position.copy()]
    ship_a_trajectory = [ship_a.position.copy()]
    distances = []
    bearings = []
    absolute_bearings = []
    angular_sizes = []
    bearings_difference = []
    absolute_bearings_difference = []
    
    # Run simulation loop (matching original logic)
    min_distance = float('inf')
    collision_time = None
    outcome = 'timeout'
    
    # Override ALPHA_NAV in the original module temporarily
    import BearingRateGraph_cleaned as cleaned_module
    original_alpha_nav = cleaned_module.ALPHA_NAV
    cleaned_module.ALPHA_NAV = alpha_nav
    
    try:
        for step in range(max_time_steps):
            # Calculate current metrics (matching original order)
            bearing = get_bearing(ownship, ship_a)  # Relative bearing
            absolute_bearing = get_absolute_bearing(ownship, ship_a)  # Absolute bearing
            angular_size = get_angular_diameter(ownship, ship_a)
            current_distance = get_distance_3d(ownship.position, ship_a.position)
            
            # Store metrics
            bearings.append(bearing)
            absolute_bearings.append(absolute_bearing)
            angular_sizes.append(angular_size)
            distances.append(current_distance)
            min_distance = min(min_distance, current_distance)
            
            # Check for collision
            if current_distance <= (ownship_size + ship_a_size):
                outcome = 'collision'
                collision_time = step * delta_time
                break
                
            # Check if goal reached
            goal_distance = get_distance_3d(ownship.position, goal.position)
            if goal_distance <= 2.0:  # goal reach threshold
                outcome = 'goal_reached'
                break
            
            # Use original algorithm to adjust heading (matching original sequence)
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(
                absolute_bearings,
                absolute_bearings_difference,
                angular_sizes,
                ownship,
                goal,
                ship_a,
                delta_time
            )
            
            # Store trajectory BEFORE update (matching original sequence)
            ownship_trajectory.append(ownship.position.copy())
            ship_a_trajectory.append(ship_a.position.copy())
            
            # Update positions using original ShipStatus.update() method
            ownship.update(delta_time)
            ship_a.update(delta_time)
            
            # Calculate bearing rate differences (for next iteration) - AFTER update
            update_absolute_bearing = get_absolute_bearing(ownship, ship_a)
            update_bearing = get_bearing(ownship, ship_a)
            
            # Calculate bearing differences properly
            abs_diff = angle_difference_in_deg(absolute_bearing, update_absolute_bearing) / delta_time
            rel_diff = angle_difference_in_deg(bearing, update_bearing) / delta_time
            
            absolute_bearings_difference.append(abs_diff)
            bearings_difference.append(rel_diff)
        
        # If loop completes without break, step will be max_time_steps-1
        final_step = step if outcome != 'timeout' else max_time_steps - 1
        
    finally:
        # Restore original ALPHA_NAV
        cleaned_module.ALPHA_NAV = original_alpha_nav
    
    # Generate plot if requested
    if show_plot or save_plot:
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Convert trajectories to arrays for plotting
        ownship_traj = np.array(ownship_trajectory)
        ship_a_traj = np.array(ship_a_trajectory)
        
        # Plot trajectories
        ax.plot(ownship_traj[:, 1], ownship_traj[:, 0], 'b-', linewidth=2, label='Ownship')
        ax.plot(ship_a_traj[:, 1], ship_a_traj[:, 0], 'r-', linewidth=2, label='Ship A')
        
        # Mark start positions
        ax.plot(ownship_position[1], ownship_position[0], 'bo', markersize=8, label='Ownship Start')
        ax.plot(ship_a_position[1], ship_a_position[0], 'ro', markersize=8, label='Ship A Start')
        
        # Mark goal
        ax.plot(goal_position[1], goal_position[0], 'g*', markersize=15, label='Goal')
        
        # Mark final positions
        ax.plot(ownship_traj[-1, 1], ownship_traj[-1, 0], 'bs', markersize=8, label='Ownship End')
        ax.plot(ship_a_traj[-1, 1], ship_a_traj[-1, 0], 'rs', markersize=8, label='Ship A End')
        
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title(f'Collision Avoidance Simulation - {outcome.title()}\n'
                    f'Min Distance: {min_distance:.3f}m')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        if save_plot and plot_filename:
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            
        if show_plot:
            plt.show()
        else:
            plt.close()
    
    # Return comprehensive results
    return {
        'outcome': outcome,
        'min_distance': min_distance,
        'collision_time': collision_time,
        'final_ownship_position': ownship.position.tolist(),
        'final_ship_a_position': ship_a.position.tolist(),
        'total_time_steps': final_step,  # Use actual step count
        'ownship_trajectory': [pos.tolist() for pos in ownship_trajectory],
        'ship_a_trajectory': [pos.tolist() for pos in ship_a_trajectory],
        'distances': distances,
        'simulation_time': final_step * delta_time,
        'parameters': {
            'alpha_nav': alpha_nav,
            'delta_time': delta_time,
            'ownship_velocity': ownship_velocity,
            'ship_a_velocity': ship_a_velocity,
            'ship_a_heading': ship_a_heading,
            'ownship_size': ownship_size,
            'ship_a_size': ship_a_size
        }
    }
def validate_wrapper_consistency() -> bool:
    """
    Validate that wrapper produces identical results to original BearingRateGraph_cleaned.py
    """
    if not CLEANED_AVAILABLE:
        print("Cannot validate - BearingRateGraph_cleaned.py not available")
        return False
    
    # Use same test parameters as test_original_cleaned.py
    test_params = {
        'ownship_position': [0.0, 0.0, 0.0],
        'ownship_velocity': 1.0,
        'ship_a_position': [50.0, 0.0, 0.0],
        'ship_a_velocity': 1.0,
        'ship_a_heading': 180.0,  # head-on scenario
        'goal_position': [50.0, 0.0, 0.0],
        'alpha_nav': 2.0,
        'delta_time': 0.01,
        'max_time_steps': 5000
    }
    
    # Run wrapper
    wrapper_result = run_collision_avoidance_simulation(**test_params)
    
    print(f"Wrapper Results:")
    print(f"  Outcome: {wrapper_result['outcome']}")
    print(f"  Min Distance: {wrapper_result['min_distance']:.3f}m")
    print(f"  Total Steps: {wrapper_result['total_time_steps']}")
    
    # Expected results from baseline test
    expected_min_distance = 3.342
    expected_outcome = 'timeout'
    expected_steps = 4999
    
    # Check consistency
    distance_match = abs(wrapper_result['min_distance'] - expected_min_distance) < 0.01
    outcome_match = wrapper_result['outcome'] == expected_outcome
    steps_match = wrapper_result['total_time_steps'] == expected_steps
    
    consistent = distance_match and outcome_match and steps_match
    
    print(f"\nConsistency Check:")
    print(f"  Distance Match: {distance_match} ({wrapper_result['min_distance']:.3f} vs {expected_min_distance})")
    print(f"  Outcome Match: {outcome_match} ({wrapper_result['outcome']} vs {expected_outcome})")
    print(f"  Steps Match: {steps_match} ({wrapper_result['total_time_steps']} vs {expected_steps})")
    print(f"  Overall Consistent: {consistent}")
    
    return consistent


if __name__ == "__main__":
    print("Testing cleaned_wrapper.py consistency...")
    validate_wrapper_consistency()
