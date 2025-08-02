import numpy as np
import matplotlib.pyplot as plt
import math

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=0.5, max_rate_of_turn=[12, 12], velocity_limit=[0.5, 10.0]):
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

def adj_ownship_heading(absolute_bearings, absolute_bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01):
    """
    Adjust ownship heading based on CBDR (Constant Bearing, Decreasing Range) principle.
    Uses absolute bearings for proper CBDR detection.
    
    Args:
        absolute_bearings: List of absolute bearings to target ship
        absolute_bearings_difference: List of absolute bearing rate changes
        angular_sizes: List of angular sizes of target ship
        ship: Own ship object
        goal: Goal ship object
        target_ship: Target ship object
        delta_time: Time step
        
    Returns:
        tuple: (rate_of_turn, velocity)
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn  
    max_rate_of_turn = ship.max_rate_of_turn[0]
    
    # Early return if no bearing data available
    if len(absolute_bearings_difference) == 0:
        return rate_of_turn, velocity
    
    current_relative_bearing = get_bearing(ship, target_ship)
    avoidance_gain = angular_sizes[-1] ** 2  # Use angular size as urgency factor
    
    # Check for CBDR condition (Constant Bearing, Decreasing Range)
    if abs(absolute_bearings_difference[-1] * delta_time) <= angular_sizes[-1]:
        rounded_rate = np.round(absolute_bearings_difference[-1], 5)
        
        if abs(rounded_rate) <= 1e-5:  # True CBDR (bearing rate ≈ 0)
            # Turn away from ship based on its relative position
            if current_relative_bearing < 0:  # Ship is on port side (left)
                rate_of_turn = -max_rate_of_turn  # Turn left (negative)
            else:  # Ship is on starboard side (right)
                rate_of_turn = max_rate_of_turn   # Turn right (positive)
        else:
            # Non-zero bearing rate case
            if abs(current_relative_bearing) < 90:  # Target is ahead (front 180° sector)
                # Target ahead: turn opposite to absolute bearing rate direction
                rate_of_turn = -np.sign(absolute_bearings_difference[-1]) * avoidance_gain
            else:  # Target is behind (rear 180° sector)
                # Target behind: turn same direction as absolute bearing rate
                rate_of_turn = np.sign(absolute_bearings_difference[-1]) * avoidance_gain
    
    # Navigate to goal when no collision threat
    if angular_sizes[-1] < 2.0:
        theta_goal = get_bearing(ship, goal)
        rate_of_turn = theta_goal
        distance = get_distance_3d(ship.position, goal.position)
        velocity = min(distance, 1.0) if distance < 1 else 1.0
    
    # Clamp rate of turn to maximum limits
    rate_of_turn = np.clip(rate_of_turn, -max_rate_of_turn, max_rate_of_turn)
    
    return rate_of_turn, velocity

def run_simulation():
    """Run ship collision avoidance simulation."""
    # Initialize simulation parameters
    time_steps = 5000
    delta_time = 0.01
    
    # Initialize ships
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0, 
                        rate_of_turn=0, position=[0, 0, 0])
    target_ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=90.0, 
                           rate_of_turn=0, position=[25, -25, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, 
                     rate_of_turn=0, position=[50, 0, 0])
    
    # Initialize data storage
    simulation_data = {
        'ownship_positions': [], 'ship_positions': [],
        'bearings': [], 'angular_sizes': [], 'bearings_difference': [], 'distances': [],
        'absolute_bearings': [], 'absolute_bearings_difference': [],
        'ownship_velocities': [], 'ship_velocities': [], 'ownship_headings': []
    }
    
    # Run simulation loop
    for step in range(time_steps):
        # Calculate current measurements
        bearing = get_bearing(ownship, target_ship)
        absolute_bearing = get_absolute_bearing(ownship, target_ship)
        angular_size = get_angular_diameter(ownship, target_ship)
        distance = get_distance_3d(ownship.position, target_ship.position)
        
        # Store measurements
        simulation_data['bearings'].append(bearing)
        simulation_data['absolute_bearings'].append(absolute_bearing)
        simulation_data['angular_sizes'].append(angular_size)
        simulation_data['distances'].append(distance)
        simulation_data['ownship_velocities'].append(ownship.velocity)
        simulation_data['ship_velocities'].append(target_ship.velocity)
        simulation_data['ownship_headings'].append(ownship.heading)
        
        # Calculate control inputs
        ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(
            simulation_data['absolute_bearings'], 
            simulation_data['absolute_bearings_difference'], 
            simulation_data['angular_sizes'], 
            ownship, goal, target_ship, delta_time
        )
        
        # Store positions
        simulation_data['ownship_positions'].append(ownship.position.copy())
        simulation_data['ship_positions'].append(target_ship.position.copy())
        
        # Update ship states
        ownship.update(delta_time)
        target_ship.update(delta_time)
        
        # Calculate bearing rates
        update_absolute_bearing = get_absolute_bearing(ownship, target_ship)
        update_bearing = get_bearing(ownship, target_ship)
        
        abs_diff = angle_difference_in_deg(absolute_bearing, update_absolute_bearing) / delta_time
        rel_diff = angle_difference_in_deg(bearing, update_bearing) / delta_time
        
        simulation_data['absolute_bearings_difference'].append(abs_diff)
        simulation_data['bearings_difference'].append(rel_diff)
    
    # Convert to numpy arrays and clean up
    for key in simulation_data:
        simulation_data[key] = np.array(simulation_data[key])
    
    # Clean up floating point errors in bearing rates
    simulation_data['bearings_difference'][np.abs(simulation_data['bearings_difference']) < 1e-10] = 0.0
    simulation_data['absolute_bearings_difference'][np.abs(simulation_data['absolute_bearings_difference']) < 1e-10] = 0.0
    
    # Calculate jerk and plot results
    jerk = np.gradient(simulation_data['bearings_difference'], delta_time)
    plot_simulation_results(simulation_data, jerk, delta_time)

def plot_simulation_results(data, jerk, delta_time):
    """
    Plot comprehensive simulation results.
    
    Args:
        data: Dictionary containing simulation data arrays
        jerk: Jerk calculation array
        delta_time: Time step for simulation
    """
    fig = plt.figure(figsize=(24, 16))
    
    # Extract data for readability
    ownship_positions = data['ownship_positions']
    ship_positions = data['ship_positions']
    bearings = data['bearings']
    absolute_bearings = data['absolute_bearings']
    angular_sizes = data['angular_sizes']
    distances = data['distances']
    ownship_velocities = data['ownship_velocities']
    ship_velocities = data['ship_velocities']
    ownship_headings = data['ownship_headings']
    absolute_bearings_difference = data['absolute_bearings_difference']
    
    # 1. Ship Positions Plot
    plt.subplot(2, 4, 1)
    ownship_line, = plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
    ship_line, = plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
    
    # Add direction arrows
    if len(ownship_positions) > 1:
        plt.annotate('', xy=(ownship_positions[-1, 1], ownship_positions[-1, 0]), 
                    xytext=(ownship_positions[-2, 1], ownship_positions[-2, 0]), 
                    arrowprops=dict(arrowstyle='->', color=ownship_line.get_color()))
        plt.annotate('', xy=(ship_positions[-1, 1], ship_positions[-1, 0]), 
                    xytext=(ship_positions[-2, 1], ship_positions[-2, 0]), 
                    arrowprops=dict(arrowstyle='->', color=ship_line.get_color()))
    
    # Draw ship size circles
    ownship_circle = plt.Circle((ownship_positions[-1, 1], ownship_positions[-1, 0]), 
                               1.0, color=ownship_line.get_color(), fill=False, linestyle='--', alpha=0.7)
    ship_circle = plt.Circle((ship_positions[-1, 1], ship_positions[-1, 0]), 
                            1.0, color=ship_line.get_color(), fill=False, linestyle='--', alpha=0.7)
    plt.gca().add_patch(ownship_circle)
    plt.gca().add_patch(ship_circle)
    
    # Plot time markers every 10 seconds
    time_interval = 10.0
    point_interval = int(time_interval / delta_time)
    for i in range(0, len(ownship_positions), point_interval):
        plt.plot(ownship_positions[i, 1], ownship_positions[i, 0], 'o', 
                color=ownship_line.get_color(), markersize=6)
        plt.plot(ship_positions[i, 1], ship_positions[i, 0], 'o', 
                color=ship_line.get_color(), markersize=6)
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Ship Positions Over Time')
    plt.legend()
    plt.grid(True)
    
    xlims = plt.xlim()
    ylims = plt.ylim()
    x_ticks = np.arange(np.floor(xlims[0]/5)*5, np.ceil(xlims[1]/5)*5+1, 5)
    y_ticks = np.arange(np.floor(ylims[0]/5)*5, np.ceil(ylims[1]/5)*5+1, 5)
    plt.gca().set_xticks(x_ticks)
    plt.gca().set_yticks(y_ticks)
    plt.axis('equal')
    
    # 2. Bearing Plot
    plt.subplot(2, 4, 2)
    plt.plot(bearings, np.arange(len(bearings)) * delta_time, label='Relative Bearing', alpha=1.0)
    absolute_bearings_normalized = absolute_bearings.copy()
    absolute_bearings_normalized[absolute_bearings_normalized > 180] -= 360
    plt.plot(absolute_bearings_normalized, np.arange(len(absolute_bearings_normalized)) * delta_time, 
             label='Absolute Bearing', linewidth=1)
    
    half_angular_sizes = angular_sizes / 2
    plt.plot(bearings - half_angular_sizes, np.arange(len(bearings)) * delta_time, '--', alpha=0.9)
    plt.plot(bearings + half_angular_sizes, np.arange(len(bearings)) * delta_time, '--', alpha=0.9)
    
    plt.ylabel('Time (s)')
    plt.xlabel('Bearing (degrees)')
    plt.title('Bearing to Ship A Over Time')
    plt.xlim(-181, 180)
    plt.axvline(x=0, color='r', linestyle='--')
    for angle in [45, 90, 135, 180, -45, -90, -135, -180]:
        plt.axvline(x=angle, color='g' if angle % 90 != 0 else 'b', linestyle='--')
    plt.legend()
    plt.grid(True)
    
    # 3. Angular Size Plot
    plt.subplot(2, 4, 3)
    plt.plot(np.arange(len(angular_sizes)) * delta_time, angular_sizes, label='Angular Size')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Size (degrees)')
    plt.title('Angular Size Over Time')
    plt.legend()
    plt.grid(True)
    
    # 4. Distance Plot
    plt.subplot(2, 4, 4)
    plt.plot(np.arange(len(distances)) * delta_time, distances, label='Distance to Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Distance Over Time')
    plt.legend()
    plt.grid(True)
    
    # 5. Relative Bearing Plot
    plt.subplot(2, 4, 5)
    plt.plot(np.arange(len(bearings)) * delta_time, bearings, label='Relative Bearing')
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0° Line')
    plt.axhline(y=-180, color='black', linestyle='--', alpha=0.5, label='±180° Line')
    plt.axhline(y=180, color='black', linestyle='--', alpha=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Bearing (degrees)')
    plt.title('Relative Bearing Over Time')
    plt.legend()
    plt.grid(True)
    
    # 6. Ship Velocities Plot
    plt.subplot(2, 4, 6)
    plt.plot(np.arange(len(ownship_velocities)) * delta_time, ownship_velocities, 
             label='Ownship Velocity', color='blue')
    plt.plot(np.arange(len(ship_velocities)) * delta_time, ship_velocities, 
             label='Ship A Velocity', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Ship Velocities Over Time')
    plt.legend()
    plt.grid(True)
    
    # 7. Ownship Heading Plot
    plt.subplot(2, 4, 7)
    plt.plot(np.arange(len(ownship_headings)) * delta_time, ownship_headings, 
             label='Ownship Heading', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Ownship Heading Over Time')
    plt.legend()
    plt.grid(True)
    
    # 8. Absolute Bearing Plot
    plt.subplot(2, 4, 8)
    plt.plot(np.arange(len(absolute_bearings_normalized)) * delta_time, 
             absolute_bearings_normalized, label='Absolute Bearing', linewidth=1)
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0° Line')
    plt.axhline(y=-180, color='black', linestyle='--', alpha=0.5, label='±180° Line')
    plt.axhline(y=180, color='black', linestyle='--', alpha=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Absolute Bearing (degrees)')
    plt.title('Absolute Bearing for CBDR Detection')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()
