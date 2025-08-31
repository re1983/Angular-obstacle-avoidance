"""
Collision geometry calculations for Monte Carlo simulation.
Handles calculation of collision points and Ship A starting positions.
"""

import numpy as np
import math
from . import config

def calculate_collision_point(ownship_start, goal_position, collision_ratio=None):
    """Calculate collision point along Ownship's path.
    
    Args:
        ownship_start: Ownship starting position [N, E, D]
        goal_position: Goal position [N, E, D]  
        collision_ratio: Ratio along path (0.4-0.6), random if None
        
    Returns:
        collision_point: [N, E, D] coordinates of collision point
        collision_ratio: Actual ratio used
    """
    ownship_start = np.array(ownship_start)
    goal_position = np.array(goal_position)
    
    if collision_ratio is None:
        # Random collision point between 40%-60% of the path
        collision_ratio = np.random.uniform(
            config.COLLISION_ZONE_START_RATIO, 
            config.COLLISION_ZONE_END_RATIO
        )
    
    # Calculate collision point
    collision_point = ownship_start + collision_ratio * (goal_position - ownship_start)
    
    return collision_point, collision_ratio

def calculate_time_to_collision(ownship_start, collision_point, ownship_velocity):
    """Calculate time for ownship to reach collision point.
    
    Args:
        ownship_start: Ownship starting position [N, E, D]
        collision_point: Collision point [N, E, D]
        ownship_velocity: Ownship velocity (m/s)
        
    Returns:
        time_to_collision: Time in seconds
    """
    distance = np.linalg.norm(np.array(collision_point) - np.array(ownship_start))
    return distance / ownship_velocity

def generate_ship_a_trajectory(collision_point, ship_a_velocity, ship_a_heading, 
                             time_to_collision, min_distance=None, max_distance=None):
    """Generate Ship A starting position and trajectory to reach collision point.
    
    Args:
        collision_point: Target collision point [N, E, D]
        ship_a_velocity: Ship A velocity (m/s)
        ship_a_heading: Ship A heading (degrees)
        time_to_collision: Time available to reach collision point (s)
        min_distance: Minimum spawn distance from collision point
        max_distance: Maximum spawn distance from collision point
        
    Returns:
        ship_a_start: Starting position for Ship A [N, E, D]
        is_valid: Whether the trajectory is geometrically valid
    """
    if min_distance is None:
        min_distance = config.SHIP_A_MIN_SPAWN_DISTANCE
    if max_distance is None:
        max_distance = config.SHIP_A_MAX_SPAWN_DISTANCE
    
    collision_point = np.array(collision_point)
    
    # Calculate distance Ship A can travel in available time
    travel_distance = ship_a_velocity * time_to_collision
    
    # Check if travel distance is within reasonable bounds
    if travel_distance < min_distance or travel_distance > max_distance:
        return None, False
    
    # Calculate Ship A starting position
    # Move backwards from collision point along Ship A's heading direction
    heading_rad = np.radians(ship_a_heading)
    direction_vector = np.array([
        np.cos(heading_rad),  # North component
        np.sin(heading_rad),  # East component  
        0                     # Down component
    ])
    
    # Ship A starts at collision_point - travel_distance * direction
    ship_a_start = collision_point - travel_distance * direction_vector
    
    return ship_a_start, True

def validate_trajectory(ownship_start, goal_position, ship_a_start, collision_point):
    """Validate that the generated trajectory is reasonable.
    
    Args:
        ownship_start: Ownship starting position [N, E, D]
        goal_position: Goal position [N, E, D]
        ship_a_start: Ship A starting position [N, E, D]
        collision_point: Planned collision point [N, E, D]
        
    Returns:
        is_valid: Boolean indicating if trajectory is valid
        validation_info: Dictionary with validation details
    """
    ownship_start = np.array(ownship_start)
    goal_position = np.array(goal_position)
    ship_a_start = np.array(ship_a_start)
    collision_point = np.array(collision_point)
    
    validation_info = {}
    is_valid = True
    
    # Check if collision point is too close to start or goal
    dist_to_start = np.linalg.norm(collision_point - ownship_start)
    dist_to_goal = np.linalg.norm(collision_point - goal_position)
    
    validation_info['distance_to_start'] = dist_to_start
    validation_info['distance_to_goal'] = dist_to_goal
    
    if dist_to_start < config.START_EXCLUSION_DISTANCE:
        validation_info['start_exclusion_violation'] = True
        is_valid = False
    else:
        validation_info['start_exclusion_violation'] = False
        
    if dist_to_goal < config.GOAL_EXCLUSION_DISTANCE:
        validation_info['goal_exclusion_violation'] = True
        is_valid = False
    else:
        validation_info['goal_exclusion_violation'] = False
    
    # Check Ship A spawn distance constraints
    ship_a_to_collision = np.linalg.norm(ship_a_start - collision_point)
    validation_info['ship_a_spawn_distance'] = ship_a_to_collision
    
    if (ship_a_to_collision < config.SHIP_A_MIN_SPAWN_DISTANCE or 
        ship_a_to_collision > config.SHIP_A_MAX_SPAWN_DISTANCE):
        validation_info['spawn_distance_violation'] = True
        is_valid = False
    else:
        validation_info['spawn_distance_violation'] = False
    
    # Check if Ship A is too close to ownship start or goal
    ship_a_to_start = np.linalg.norm(ship_a_start - ownship_start)
    ship_a_to_goal = np.linalg.norm(ship_a_start - goal_position)
    
    validation_info['ship_a_to_start'] = ship_a_to_start
    validation_info['ship_a_to_goal'] = ship_a_to_goal
    
    min_separation = max(config.START_EXCLUSION_DISTANCE, config.GOAL_EXCLUSION_DISTANCE)
    if ship_a_to_start < min_separation or ship_a_to_goal < min_separation:
        validation_info['ship_a_proximity_violation'] = True
        is_valid = False
    else:
        validation_info['ship_a_proximity_violation'] = False
    
    return is_valid, validation_info

def generate_random_ship_a_parameters():
    """Generate random parameters for Ship A within configured ranges.
    
    Returns:
        ship_a_params: Dictionary containing randomized Ship A parameters
    """
    ship_a_params = {
        'size': np.random.uniform(*config.SHIP_A_SIZE_RANGE),
        'velocity': np.random.uniform(*config.SHIP_A_VELOCITY_RANGE), 
        'heading': np.random.uniform(*config.SHIP_A_HEADING_RANGE),
        'rate_of_turn': config.SHIP_A_MAX_RATE_OF_TURN[0],  # Use fixed rate of turn
        'max_rate_of_turn': config.SHIP_A_MAX_RATE_OF_TURN,
        'velocity_limit': config.SHIP_A_VELOCITY_LIMIT
    }
    
    return ship_a_params

def create_collision_scenario():
    """Create a complete collision scenario with randomized parameters.
    
    Returns:
        scenario: Dictionary containing all scenario parameters
        is_valid: Whether the scenario is geometrically valid
    """
    max_attempts = 100  # Prevent infinite loops
    
    for attempt in range(max_attempts):
        # Generate random Ship A parameters
        ship_a_params = generate_random_ship_a_parameters()
        
        # Calculate collision point
        collision_point, collision_ratio = calculate_collision_point(
            config.OWNSHIP_INITIAL_POSITION,
            config.GOAL_POSITION
        )
        
        # Calculate time to collision
        time_to_collision = calculate_time_to_collision(
            config.OWNSHIP_INITIAL_POSITION,
            collision_point,
            config.OWNSHIP_VELOCITY
        )
        
        # Generate Ship A trajectory  
        ship_a_start, trajectory_valid = generate_ship_a_trajectory(
            collision_point,
            ship_a_params['velocity'],
            ship_a_params['heading'],
            time_to_collision
        )
        
        if not trajectory_valid:
            continue
            
        # Validate the complete trajectory
        is_valid, validation_info = validate_trajectory(
            config.OWNSHIP_INITIAL_POSITION,
            config.GOAL_POSITION,
            ship_a_start,
            collision_point
        )
        
        if is_valid:
            scenario = {
                'ownship_start': np.array(config.OWNSHIP_INITIAL_POSITION),
                'goal_position': np.array(config.GOAL_POSITION),
                'collision_point': collision_point,
                'collision_ratio': collision_ratio,
                'time_to_collision': time_to_collision,
                'ship_a_start': ship_a_start,
                'ship_a_params': ship_a_params,
                'validation_info': validation_info,
                'attempt_number': attempt + 1,
                'random_seed': str(np.random.get_state()[1][0]) if len(np.random.get_state()[1]) > 0 else 'N/A',  # Current random state as string
                'scenario_id': f"scenario_{attempt+1:04d}"
            }
            
            return scenario, True
    
    # If we get here, couldn't create valid scenario
    return None, False

def calculate_closest_point_of_approach(pos1_start, vel1, heading1, pos2_start, vel2, heading2, max_time):
    """Calculate the closest point of approach between two ships.
    
    Args:
        pos1_start: Starting position of ship 1 [N, E, D]
        vel1: Velocity of ship 1 (m/s)
        heading1: Heading of ship 1 (degrees)
        pos2_start: Starting position of ship 2 [N, E, D] 
        vel2: Velocity of ship 2 (m/s)
        heading2: Heading of ship 2 (degrees)
        max_time: Maximum time to check (seconds)
        
    Returns:
        min_distance: Minimum distance between ships
        time_at_min_distance: Time when minimum distance occurs
    """
    pos1_start = np.array(pos1_start)
    pos2_start = np.array(pos2_start)
    
    # Convert headings to unit vectors
    heading1_rad = np.radians(heading1)
    heading2_rad = np.radians(heading2)
    
    vel1_vector = vel1 * np.array([np.cos(heading1_rad), np.sin(heading1_rad), 0])
    vel2_vector = vel2 * np.array([np.cos(heading2_rad), np.sin(heading2_rad), 0])
    
    # Relative position and velocity
    rel_pos = pos2_start - pos1_start
    rel_vel = vel2_vector - vel1_vector
    
    # If relative velocity is zero, ships maintain constant separation
    if np.linalg.norm(rel_vel) < 1e-10:
        return np.linalg.norm(rel_pos), 0.0
    
    # Time of closest approach: t = -(rel_pos · rel_vel) / |rel_vel|²
    t_closest = -np.dot(rel_pos, rel_vel) / np.dot(rel_vel, rel_vel)
    
    # Clamp to [0, max_time]
    t_closest = max(0, min(t_closest, max_time))
    
    # Calculate positions at closest approach
    pos1_closest = pos1_start + vel1_vector * t_closest
    pos2_closest = pos2_start + vel2_vector * t_closest
    
    min_distance = np.linalg.norm(pos2_closest - pos1_closest)
    
    return min_distance, t_closest
