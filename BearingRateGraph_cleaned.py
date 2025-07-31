import numpy as np
import matplotlib.pyplot as plt
import math

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
    Now uses absolute bearings for proper CBDR detection.
    
    Args:
        absolute_bearings: List of absolute bearings to target ship
        absolute_bearings_difference: List of absolute bearing rate changes
        angular_sizes: List of angular sizes of target ship
        ship: Own ship object
        goal: Goal ship object
        target_ship: Target ship object (for relative bearing calculation)
        delta_time: Time step
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn

    if len(absolute_bearings_difference) > 1:
        # Check for CBDR (Constant Bearing, Decreasing Range) condition using absolute bearings
        # If absolute bearing rate is near zero AND angular size is increasing, we have CBDR
        if abs(absolute_bearings_difference[-1]*delta_time) <= angular_sizes[-1] and angular_sizes[-1] > 5.0:
            # Get current relative bearing to target ship to determine avoidance direction
            current_relative_bearing = get_bearing(ship, target_ship)
            
            # Collision avoidance: determine turn direction based on relative position
            avoidance_gain = angular_sizes[-1]  # Use angular size as urgency factor
            
            # For CBDR situation, decide turn direction based on relative bearing
            # Goal: turn away from the approaching ship to avoid collision
            if abs(absolute_bearings_difference[-1]) < 0.1:  # True CBDR (bearing rate ≈ 0)
                # Turn away from ship based on its relative position
                if current_relative_bearing < 0:  # Ship is on port side (left)
                    rate_of_turn = -avoidance_gain  # Turn left (negative)
                else:  # Ship is on starboard side (right)
                    rate_of_turn = avoidance_gain   # Turn right (positive)
            else:
                # Non-zero bearing rate case - use original logic
                # Determine turn direction based on relative bearing
                # relative_bearing range: -180° to +180°
                # Front sector: -90° to +90° (abs(relative_bearing) <= 90°)
                # Rear sector: +90° to +180° and -90° to -180° (abs(relative_bearing) > 90°)
                if abs(current_relative_bearing) <= 90:  # Target is ahead (front 180° sector)
                    # Target ahead: turn opposite to absolute bearing rate direction to accelerate avoidance
                    rate_of_turn = -np.sign(absolute_bearings_difference[-1]) * avoidance_gain
                else:  # Target is behind (rear 180° sector)
                    # Target behind: turn same direction as absolute bearing rate
                    rate_of_turn = np.sign(absolute_bearings_difference[-1]) * avoidance_gain
                
        else:
            # Navigate to goal when no collision threat
            theta_goal = get_bearing(ship, goal)  # Use relative bearing for goal navigation
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 1:
                velocity = distance
            else:
                velocity = 1.0
        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def run_simulation():
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=-0.0, position=[0, 0, 0])
    ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=90.0, rate_of_turn=0, position=[10, -10, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    time_steps = 8000
    delta_time = 0.01
    
    # Initialize data storage lists
    ownship_positions, ship_positions = [], []
    bearings, angular_sizes, bearings_difference, distances = [], [], [], []
    absolute_bearings, absolute_bearings_difference = [], []  # Add absolute bearings
    ownship_velocities, ship_velocities = [], []  # Add velocity tracking
    ownship_headings = []  # Add heading tracking
    
    for _ in range(time_steps):
        bearing = get_bearing(ownship, ship)  # Relative bearing for display
        absolute_bearing = get_absolute_bearing(ownship, ship)  # Absolute bearing for CBDR
        bearings.append(bearing)
        absolute_bearings.append(absolute_bearing)
        angular_size = get_angular_diameter(ownship, ship)
        angular_sizes.append(angular_size)
        distances.append(get_distance_3d(ownship.position, ship.position))
        
        # Record current states before update
        ownship_velocities.append(ownship.velocity)
        ship_velocities.append(ship.velocity)
        ownship_headings.append(ownship.heading)
        
        # Use absolute bearing difference for CBDR detection
        ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(absolute_bearings, absolute_bearings_difference, angular_sizes, ownship, goal, ship, delta_time)
        ownship_positions.append(ownship.position.copy())
        ship_positions.append(ship.position.copy())
        ownship.update(delta_time)
        ship.update(delta_time)
        
        # Calculate bearing rate using absolute bearings for proper CBDR detection
        update_absolute_bearing = get_absolute_bearing(ownship, ship)
        absolute_bearings_difference.append(angle_difference_in_deg(absolute_bearing, update_absolute_bearing) / delta_time)
        
        # Keep relative bearing difference for display purposes
        update_bearing = get_bearing(ownship, ship)
        bearings_difference.append(angle_difference_in_deg(bearing, update_bearing) / delta_time)
    
    # Convert to numpy arrays
    ownship_positions = np.array(ownship_positions)
    ship_positions = np.array(ship_positions)
    bearings = np.array(bearings)
    absolute_bearings = np.array(absolute_bearings)
    angular_sizes = np.array(angular_sizes)
    bearings_difference = np.array(bearings_difference)
    absolute_bearings_difference = np.array(absolute_bearings_difference)
    distances = np.array(distances)
    ownship_velocities = np.array(ownship_velocities)
    ship_velocities = np.array(ship_velocities)
    ownship_headings = np.array(ownship_headings)
    
    jerk = np.gradient(bearings_difference, delta_time)
    plot_simulation_results(ownship_positions, ship_positions, bearings, angular_sizes, 
                           bearings_difference, distances, jerk, delta_time, 
                           ownship_velocities, ship_velocities, ownship_headings, 
                           absolute_bearings, absolute_bearings_difference)

def plot_simulation_results(ownship_positions, ship_positions, bearings, angular_sizes, 
                           bearings_difference, distances, jerk, delta_time, 
                           ownship_velocities, ship_velocities, ownship_headings,
                           absolute_bearings, absolute_bearings_difference):
    fig = plt.figure(figsize=(24, 16))  # Adjust figure size for 2 rows
    
    # Row 1: Ship positions, bearings, angular sizes, distances
    # 1. Ship Positions
    plt.subplot(2, 4, 1)
    ownship_line, = plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
    ship_line, = plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
    
    # Add direction arrows
    plt.annotate('', xy=(ownship_positions[-1, 1], ownship_positions[-1, 0]), 
                xytext=(ownship_positions[-2, 1], ownship_positions[-2, 0]), 
                arrowprops=dict(arrowstyle='->', color=ownship_line.get_color()))
    plt.annotate('', xy=(ship_positions[-1, 1], ship_positions[-1, 0]), 
                xytext=(ship_positions[-2, 1], ship_positions[-2, 0]), 
                arrowprops=dict(arrowstyle='->', color=ship_line.get_color()))
    
    # Draw ship size circles (at final positions)
    ownship_circle = plt.Circle((ownship_positions[-1, 1], ownship_positions[-1, 0]), 
                               1.0, color=ownship_line.get_color(), fill=False, linestyle='--', alpha=0.7)
    ship_circle = plt.Circle((ship_positions[-1, 1], ship_positions[-1, 0]), 
                            1.0, color=ship_line.get_color(), fill=False, linestyle='--', alpha=0.7)
    plt.gca().add_patch(ownship_circle)
    plt.gca().add_patch(ship_circle)
    
    # Plot points every 10 seconds
    time_interval = 10.0  # 10 second intervals
    point_interval = int(time_interval / delta_time)  # Convert to time step intervals
    
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
    plt.axis('equal')
    
    # 2. Bearing Plot (both relative and absolute)
    plt.subplot(2, 4, 2)
    plt.plot(bearings, np.arange(len(bearings)) * delta_time, label='Relative Bearing to Ship A', alpha=0.7)
    
    # Convert absolute bearings to -180° to +180° range for consistent display
    absolute_bearings_normalized = absolute_bearings.copy()
    absolute_bearings_normalized[absolute_bearings_normalized > 180] -= 360
    plt.plot(absolute_bearings_normalized, np.arange(len(absolute_bearings_normalized)) * delta_time, label='Absolute Bearing to Ship A', linewidth=2)
    
    half_angular_sizes = angular_sizes / 2
    plt.plot(bearings - half_angular_sizes, np.arange(len(bearings)) * delta_time, '--', alpha=0.5)
    plt.plot(bearings + half_angular_sizes, np.arange(len(bearings)) * delta_time, '--', alpha=0.5)
    plt.ylabel('Time (s)')
    plt.xlabel('Bearing (degrees)')
    plt.title('Bearing to Ship A Over Time')
    plt.xlim(-180, 180)  # Set X-axis range to -180° to +180°
    plt.axvline(x=0, color='r', linestyle='--')
    for angle in [45, 90, 135, 180, -45, -90, -135, -180]:
        plt.axvline(x=angle, color='g' if angle % 90 != 0 else 'b', linestyle='--')
    plt.legend()
    plt.grid(True)
    
    # 3. Angular Size Plot
    plt.subplot(2, 4, 3)
    plt.plot(np.arange(len(angular_sizes)) * delta_time, angular_sizes, label='Angular Size of Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Size (degrees)')
    plt.title('Angular Size of Ship A Over Time')
    plt.legend()
    plt.grid(True)
    
    # 4. Distance Plot
    plt.subplot(2, 4, 4)
    plt.plot(np.arange(len(distances)) * delta_time, distances, label='Distance to Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Distance to Ship A Over Time')
    plt.legend()
    plt.grid(True)
    
    # Row 2: Bearing difference, ship velocities, ownship heading, absolute bearing rate
    # 5. Bearing Difference Plot (Relative)
    plt.subplot(2, 4, 5)
    plt.plot(np.arange(len(bearings_difference)) * delta_time, bearings_difference, label='Relative Bearing Rate')
    plt.xlabel('Time (s)')
    plt.ylabel('Bearing Rate (degrees/s)')
    plt.title('Relative Bearing Rate Over Time')
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
    
    # 8. Absolute Bearing Rate Plot (CBDR detection)
    plt.subplot(2, 4, 8)
    plt.plot(np.arange(len(absolute_bearings_difference)) * delta_time, absolute_bearings_difference, 
             label='Absolute Bearing Rate (CBDR)', color='red', linewidth=2)
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='CBDR Line (Rate = 0)')
    plt.xlabel('Time (s)')
    plt.ylabel('Absolute Bearing Rate (degrees/s)')
    plt.title('Absolute Bearing Rate for CBDR Detection')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()
