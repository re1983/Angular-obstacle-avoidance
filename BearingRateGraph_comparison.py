import numpy as np
import matplotlib.pyplot as plt
import math

# Navigation threshold constant (degrees)
ALPHA_NAV = 0.5  # No collision threat below this angular diameter

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=1.0, max_rate_of_turn=[12, 12], velocity_limit=[0.5, 10.0]):
        self.name = name
        self.velocity = velocity
        self.acceleration = acceleration
        self.heading = heading
        self.rate_of_turn = rate_of_turn
        self.position = np.array(position, dtype=float)
        self.size = size
        self.max_rate_of_turn = max_rate_of_turn
        self.velocity_limit = velocity_limit

    def update(self, delta_time=0.01):
        self.heading += self.rate_of_turn * delta_time
        # NED coordinate system: North=X+, East=Y+, Down=Z+
        # Heading: 0°=North, 90°=East, 180°=South, 270°=West
        self.position += self.velocity * delta_time * np.array([
            np.cos(np.radians(self.heading)),
            np.sin(np.radians(self.heading)),
            0])
        self.velocity += self.acceleration

    def get_status(self):
        return {
            "name": self.name,
            "velocity": self.velocity,
            "heading": self.heading,
            "current_position": self.position
        }

    def copy(self):
        """Create a deep copy of the ship status"""
        return ShipStatus(
            name=self.name,
            velocity=self.velocity,
            acceleration=self.acceleration,
            heading=self.heading,
            rate_of_turn=self.rate_of_turn,
            position=self.position.copy(),
            size=self.size,
            max_rate_of_turn=self.max_rate_of_turn.copy(),
            velocity_limit=self.velocity_limit.copy()
        )

def get_distance_3d(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def get_bearing(ship1, ship2):
    """Calculate relative bearing from ship1 to ship2
    
    Returns:
        relative_bearing: Range -180° to +180°
        - Positive: target is to starboard (right) of ship1's heading
        - Negative: target is to port (left) of ship1's heading
        - 0°: target is directly ahead
        - ±180°: target is directly behind
    """
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    angle_to_ship2 = np.degrees(theta)
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    return relative_bearing

def get_absolute_bearing(ship1, ship2):
    """Calculate absolute bearing (true bearing) from ship1 to ship2"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    return absolute_bearing

def get_angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)

def angle_difference(angle1, angle2):
    angle_diff = (angle2 - angle1) % (2 * math.pi)
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    return angle_diff

def angle_difference_in_deg(angle1, angle2):
    angle_diff = (angle2 - angle1) % 360
    if angle_diff > 180:
        angle_diff -= 360
    return angle_diff

def adj_ownship_heading_absolute(absolute_bearings, absolute_bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01):
    """
    Adjust ownship heading based on CBDR principle using absolute bearings.
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn
    max_rate_of_turn = ship.max_rate_of_turn[0]
    current_relative_bearing = get_bearing(ship, target_ship)
    avoidance_gain = angular_sizes[-1]*2

    if len(absolute_bearings_difference) >= 1:
        if abs(absolute_bearings_difference[-1]*delta_time) <= angular_sizes[-1]:
            rounded_rate = np.round(absolute_bearings_difference[-1], 5)
            if abs(rounded_rate) <= 1e-5:  # True CBDR (bearing rate ≈ 0)
                if current_relative_bearing < 0:  # Ship is on port side (left)
                    rate_of_turn = -max_rate_of_turn  # Turn left (negative)
                else:  # Ship is on starboard side (right)
                    rate_of_turn = max_rate_of_turn   # Turn right (positive)
            else:
                # Non-zero bearing rate case
                if abs(current_relative_bearing) < 90:  # Target is ahead
                    rate_of_turn = -np.sign(absolute_bearings_difference[-1]) * avoidance_gain
                else:  # Target is behind
                    rate_of_turn = np.sign(absolute_bearings_difference[-1]) * avoidance_gain

        if angular_sizes[-1] < ALPHA_NAV:
            # Navigate to goal when no collision threat
            theta_goal = get_bearing(ship, goal)
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 1:
                velocity = distance
            else:
                velocity = 1.0
        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def adj_ownship_heading_relative(bearings, bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01):
    """
    Adjust ownship heading based on CBDR principle using relative bearings.
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn
    max_rate_of_turn = ship.max_rate_of_turn[0]
    current_relative_bearing = get_bearing(ship, target_ship)
    avoidance_gain = abs(angular_sizes[-1])*2

    if len(bearings_difference) >= 1:
        if abs(bearings_difference[-1]*delta_time) <= angular_sizes[-1]:
            rounded_rate = np.round(bearings_difference[-1], 5)
            if abs(rounded_rate) <= 1e-5:  # True CBDR (bearing rate ≈ 0)
                if current_relative_bearing < 0:  # Ship is on port side (left)
                    rate_of_turn = -max_rate_of_turn  # Turn left (negative)
                else:  # Ship is on starboard side (right)
                    rate_of_turn = max_rate_of_turn   # Turn right (positive)
            else:
                # Non-zero bearing rate case
                if abs(current_relative_bearing) < 90:  # Target is ahead
                    rate_of_turn = -np.sign(bearings_difference[-1]) * avoidance_gain
                else:  # Target is behind
                    rate_of_turn = np.sign(bearings_difference[-1]) * avoidance_gain

        if angular_sizes[-1] < ALPHA_NAV:
            # Navigate to goal when no collision threat
            theta_goal = get_bearing(ship, goal)
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 1:
                velocity = distance
            else:
                velocity = 1.0
        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def run_single_simulation(use_absolute_bearings=True):
    """Run a single simulation with either absolute or relative bearing control"""
    # Initialize ship statuses - use same initial positions as original files
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=0, position=[0, 0, 0], size=0.5)
    ship = ShipStatus("Ship A", velocity=2.0, acceleration=0, heading=180.0, rate_of_turn=0, position=[75, 0, 0], size=0.5)
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, rate_of_turn=0, position=[50, 0, 0])
    
    time_steps = 5000
    delta_time = 0.01
    
    # Initialize data storage
    ownship_positions, ship_positions = [], []
    bearings, angular_sizes, bearings_difference, distances = [], [], [], []
    absolute_bearings, absolute_bearings_difference = [], []
    ownship_velocities, ship_velocities, ownship_headings = [], [], []
    
    for _ in range(time_steps):
        # Calculate current bearings and measurements
        bearing = get_bearing(ownship, ship)
        absolute_bearing = get_absolute_bearing(ownship, ship)
        angular_size = get_angular_diameter(ownship, ship)
        
        # Store current measurements
        bearings.append(bearing)
        absolute_bearings.append(absolute_bearing)
        angular_sizes.append(angular_size)
        distances.append(get_distance_3d(ownship.position, ship.position))
        ownship_velocities.append(ownship.velocity)
        ship_velocities.append(ship.velocity)
        ownship_headings.append(ownship.heading)
        
        # Apply control logic based on method
        if use_absolute_bearings:
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_absolute(
                absolute_bearings, absolute_bearings_difference, angular_sizes, ownship, goal, ship, delta_time)
        else:
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_relative(
                bearings, bearings_difference, angular_sizes, ownship, goal, ship, delta_time)
        
        # Store positions
        ownship_positions.append(ownship.position.copy())
        ship_positions.append(ship.position.copy())
        
        # Update ship positions
        ownship.update(delta_time)
        ship.update(delta_time)
        
        # Calculate bearing differences
        update_absolute_bearing = get_absolute_bearing(ownship, ship)
        update_bearing = get_bearing(ownship, ship)
        
        abs_diff = angle_difference_in_deg(absolute_bearing, update_absolute_bearing) / delta_time
        rel_diff = angle_difference_in_deg(bearing, update_bearing) / delta_time
        
        absolute_bearings_difference.append(abs_diff)
        bearings_difference.append(rel_diff)
    
    # Convert to numpy arrays and clean up
    result = {
        'ownship_positions': np.array(ownship_positions),
        'ship_positions': np.array(ship_positions),
        'bearings': np.array(bearings),
        'absolute_bearings': np.array(absolute_bearings),
        'angular_sizes': np.array(angular_sizes),
        'bearings_difference': np.array(bearings_difference),
        'absolute_bearings_difference': np.array(absolute_bearings_difference),
        'distances': np.array(distances),
        'ownship_velocities': np.array(ownship_velocities),
        'ship_velocities': np.array(ship_velocities),
        'ownship_headings': np.array(ownship_headings),
        'ownship_size': ownship.size,
        'ship_size': ship.size,
        'goal': goal
    }
    
    # Clean up floating point errors
    result['bearings_difference'][np.abs(result['bearings_difference']) < 1e-10] = 0.0
    result['absolute_bearings_difference'][np.abs(result['absolute_bearings_difference']) < 1e-10] = 0.0
    
    return result

def plot_comparison_results(abs_result, rel_result, delta_time):
    """Plot comparison results in 2x5 grid"""
    fig = plt.figure(figsize=(25, 12))
    
    # Row 1: Absolute Bearings Results
    plot_single_row(abs_result, delta_time, row=1, title_prefix="Absolute Bearing Control - ")
    
    # Row 2: Relative Bearings Results  
    plot_single_row(rel_result, delta_time, row=2, title_prefix="Relative Bearing Control - ")
    
    plt.tight_layout()
    plt.show()

def plot_single_row(result, delta_time, row, title_prefix=""):
    """Plot a single row of 5 subplots"""
    
    # 1. Ship Positions
    plt.subplot(2, 5, (row-1)*5 + 1)
    ownship_line, = plt.plot(result['ownship_positions'][:, 1], result['ownship_positions'][:, 0], label='Ownship')
    ship_line, = plt.plot(result['ship_positions'][:, 1], result['ship_positions'][:, 0], label='Ship A')
    
    # Add direction arrows at final positions
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
    plt.subplot(2, 5, (row-1)*5 + 2)
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
    plt.subplot(2, 5, (row-1)*5 + 3)
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
    plt.subplot(2, 5, (row-1)*5 + 4)
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
    
    # 6. Ship Velocities (移到第5位)
    plt.subplot(2, 5, (row-1)*5 + 5)
    plt.plot(np.arange(len(result['ownship_velocities'])) * delta_time, result['ownship_velocities'], 
             label='Ownship Velocity', color='blue')
    plt.plot(np.arange(len(result['ship_velocities'])) * delta_time, result['ship_velocities'], 
             label='Ship A Velocity', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title(f'{title_prefix}Velocities')
    plt.legend()
    plt.grid(True)
    
    # 5. Relative Bearing Over Time (隱藏)
    # plt.subplot(2, 5, (row-1)*5 + 5)
    # plt.plot(np.arange(len(result['bearings'])) * delta_time, result['bearings'], label='Relative Bearing')
    # plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0° Line')
    # plt.axhline(y=-180, color='black', linestyle='--', alpha=0.5, label='-180° Line')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Bearing (degrees)')
    # plt.title(f'{title_prefix}Relative Bearing')
    # plt.legend()
    # plt.grid(True)
    
    # 6. Ship Velocities (已移到上方)
    # plt.subplot(2, 5, (row-1)*5 + 6)
    # plt.plot(np.arange(len(result['ownship_velocities'])) * delta_time, result['ownship_velocities'], 
    #          label='Ownship Velocity', color='blue')
    # plt.plot(np.arange(len(result['ship_velocities'])) * delta_time, result['ship_velocities'], 
    #          label='Ship A Velocity', color='red')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity (m/s)')
    # plt.title(f'{title_prefix}Velocities')
    # plt.legend()
    # plt.grid(True)
    
    # 7. Ownship Heading (隱藏)
    # plt.subplot(2, 8, (row-1)*8 + 7)
    # plt.plot(np.arange(len(result['ownship_headings'])) * delta_time, result['ownship_headings'], 
    #          label='Ownship Heading', color='blue')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Heading (degrees)')
    # plt.title(f'{title_prefix}Ownship Heading')
    # plt.legend()
    # plt.grid(True)
    
    # 8. Absolute Bearing Over Time (隱藏)
    # plt.subplot(2, 8, (row-1)*8 + 8)
    # absolute_bearings_normalized = result['absolute_bearings'].copy()
    # absolute_bearings_normalized[absolute_bearings_normalized > 180] -= 360
    # plt.plot(np.arange(len(absolute_bearings_normalized)) * delta_time, absolute_bearings_normalized, 
    #          label='Absolute Bearing', linewidth=1)
    # plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0° Line')
    # plt.axhline(y=-180, color='black', linestyle='--', alpha=0.5, label='-180° Line')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Bearing (degrees)')
    # plt.title(f'{title_prefix}Absolute Bearing')
    # plt.legend()
    # plt.grid(True)

def run_comparison():
    """Run both simulations and plot comparison"""
    print("Running simulation with absolute bearing control...")
    abs_result = run_single_simulation(use_absolute_bearings=True)
    
    print("Running simulation with relative bearing control...")
    rel_result = run_single_simulation(use_absolute_bearings=False)
    
    print("Plotting comparison results...")
    plot_comparison_results(abs_result, rel_result, delta_time=0.01)

if __name__ == "__main__":
    run_comparison()
