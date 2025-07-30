import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial import distance

# ================== Performance Timer ==================
class Timer:
    """Context manager for timing code blocks"""
    def __enter__(self):
        self.start = time.time()
        return self
    
    def __exit__(self, *args):
        self.end = time.time()
        print(f"Execution time: {self.end - self.start:.3f} seconds")

# ================== SO(2) Lie Group Implementation ==================
class SO2:
    """Optimized SO(2) rotation group implementation"""
    __slots__ = ('angle', 'cos_val', 'sin_val')
    
    def __init__(self, angle=0.0):
        self.angle = angle
        self.cos_val = np.cos(angle)
        self.sin_val = np.sin(angle)
    
    def rotate_vector(self, vector):
        """Rotate a 2D vector by the stored angle"""
        x, y = vector
        return np.array([
            x * self.cos_val - y * self.sin_val,
            x * self.sin_val + y * self.cos_val
        ])
    
    def get_heading_vector(self):
        """Get the heading vector (unit vector in the direction of angle)"""
        return np.array([self.cos_val, self.sin_val])

# ================== Ship Model ==================
class Ship:
    """Optimized ship model with physical constraints"""
    
    def __init__(self, name, position, velocity, heading, size=10.0, max_turn_rate=10.0, max_angular_accel=10.0):
        """
        Initialize a ship with physical parameters
        
        Parameters:
            name (str): Ship name
            position (list): Initial position [x, y]
            velocity (float): Initial velocity (m/s)
            heading (float): Initial heading (degrees)
            size (float): Ship size (meters)
            max_turn_rate (float): Maximum turn rate (deg/s)
            max_angular_accel (float): Maximum angular acceleration (deg/s²)
        """
        self.name = name
        self.position = np.array(position, dtype=np.float64)
        self.velocity = float(velocity)
        self.orientation = SO2(np.radians(heading))
        self.size = float(size)
        self.collision_radius = size * 1.2
        self.max_turn_rate = np.radians(max_turn_rate)  # Convert to rad/s
        self.max_angular_accel = np.radians(max_angular_accel)  # Convert to rad/s²
        self.current_angular_velocity = 0.0
        
        # Initialize history
        self.history = {
            'time': [],
            'position': [],
            'velocity': [],
            'heading': [],
            'angular_velocity': []
        }
    
    def update(self, dt, angular_velocity_command=None):
        """
        Update ship state for one time step
        
        Parameters:
            dt (float): Time step (seconds)
            angular_velocity_command (float, optional): Commanded angular velocity (rad/s)
        """
        # Apply angular acceleration limits if a command is provided
        if angular_velocity_command is not None:
            # Calculate maximum change in angular velocity based on max acceleration
            max_delta_omega = self.max_angular_accel * dt
            
            # Calculate desired change in angular velocity
            delta_omega = angular_velocity_command - self.current_angular_velocity
            
            # Apply acceleration limit
            if abs(delta_omega) > max_delta_omega:
                delta_omega = np.sign(delta_omega) * max_delta_omega
            
            # Update angular velocity with limit
            self.current_angular_velocity += delta_omega
            
            # Apply turn rate limit
            if abs(self.current_angular_velocity) > self.max_turn_rate:
                self.current_angular_velocity = np.sign(self.current_angular_velocity) * self.max_turn_rate
        
        # Update orientation
        new_angle = self.orientation.angle + self.current_angular_velocity * dt
        self.orientation = SO2(new_angle)
        
        # Update position based on velocity and heading
        heading_vector = self.orientation.get_heading_vector()
        self.position += self.velocity * dt * heading_vector
    
    def get_heading_degrees(self):
        """Get the current heading in degrees"""
        angle_rad = self.orientation.angle
        return np.degrees(angle_rad) % 360
    
    def get_relative_bearing(self, other_ship):
        """
        Calculate relative bearing to another ship (degrees)
        
        Parameters:
            other_ship (Ship): The other ship
        
        Returns:
            float: Relative bearing (degrees)
        """
        delta_pos = other_ship.position - self.position
        angle_to_other = np.arctan2(delta_pos[1], delta_pos[0])
        
        # Convert to relative bearing
        relative_bearing = angle_to_other - self.orientation.angle
        
        # Normalize to [-π, π]
        relative_bearing = (relative_bearing + np.pi) % (2 * np.pi) - np.pi
        
        # Convert to degrees
        return np.degrees(relative_bearing)
    
    def get_distance(self, other_ship):
        """Calculate distance to another ship"""
        return np.linalg.norm(self.position - other_ship.position)
    
    def get_angular_size(self, other_ship):
        """Calculate angular size of another ship (degrees)"""
        distance = self.get_distance(other_ship)
        if distance <= other_ship.size:  # Avoid division by zero or negative values
            return 180.0  # Maximum angular size
        
        # Angular diameter formula: 2 * arctan(size / (2 * distance))
        angular_size = 2 * np.arctan(other_ship.size / (2 * distance))
        return np.degrees(angular_size)
    
    def check_collision(self, other, threshold=1.0):
        """Check if this ship collides with another ship"""
        distance = self.get_distance(other)
        collision_distance = (self.collision_radius + other.collision_radius) * threshold
        return distance < collision_distance
    
    def record_history(self, time):
        """Record current state to history"""
        self.history['time'].append(time)
        self.history['position'].append(self.position.copy())
        self.history['velocity'].append(self.velocity)
        self.history['heading'].append(self.get_heading_degrees())
        self.history['angular_velocity'].append(np.degrees(self.current_angular_velocity))

# ================== Barrier Function Controller ==================
class BarrierController:
    """
    Optimized Barrier Function Controller for CBDR Avoidance
    """
    __slots__ = ('base_alpha', 'k_gain', 'omega_min', 'prev_bearing', 
                 'alpha_factor', 'adaptive_threshold', 'collision_warning',
                 'history_time', 'history_bearing', 'history_omega', 
                 'history_control', 'history_alpha_threshold', 'history_angular_size')
    
    def __init__(self, base_alpha=0.5, k_gain=0.3, omega_min=0.1):
        """
        Initialize the barrier function controller
        
        Parameters:
            base_alpha (float): Base safety threshold (deg/s)
            k_gain (float): Control gain
            omega_min (float): Minimum angular velocity (rad/s)
        """
        self.base_alpha = base_alpha
        self.k_gain = k_gain
        self.omega_min = omega_min
        self.prev_bearing = None
        self.alpha_factor = 0.1  # Adaptive factor for angular size
        self.adaptive_threshold = True  # Whether to use adaptive threshold
        self.collision_warning = False
        
        # Initialize history arrays
        self.history_time = []
        self.history_bearing = []
        self.history_omega = []
        self.history_control = []
        self.history_alpha_threshold = []
        self.history_angular_size = []
    
    def compute_control(self, current_bearing, angular_size, dt, current_time):
        """
        Compute control action based on barrier function
        
        Parameters:
            current_bearing (float): Current bearing to obstacle (deg)
            angular_size (float): Angular size of obstacle (deg)
            dt (float): Time step (s)
            current_time (float): Current simulation time (s)
            
        Returns:
            float: Angular velocity command (rad/s)
        """
        # Initialize on first call
        if self.prev_bearing is None:
            self.prev_bearing = current_bearing
            return 0.0
        
        # Compute LOS rate (ω = dθ/dt)
        omega = (current_bearing - self.prev_bearing) / dt
        self.prev_bearing = current_bearing
        
        # Calculate current alpha threshold
        if self.adaptive_threshold:
            alpha_threshold = self.base_alpha + self.alpha_factor * angular_size
        else:
            alpha_threshold = self.base_alpha
        
        # Barrier function constraint: |ω| > α_threshold
        constraint = abs(omega) - alpha_threshold
        
        # Control law: Apply correction when constraint is violated
        if constraint < 0:  # |ω| < α_threshold
            # Apply correction proportional to constraint violation, in opposite direction of ω
            angular_velocity = self.k_gain * abs(constraint) * -np.sign(omega)
            self.collision_warning = True
        else:
            angular_velocity = 0.0
            self.collision_warning = False
        
        # Save history for plotting
        self.history_time.append(current_time)
        self.history_bearing.append(current_bearing)
        self.history_omega.append(omega)
        self.history_control.append(np.degrees(angular_velocity))
        self.history_alpha_threshold.append(alpha_threshold)
        self.history_angular_size.append(angular_size)
        
        return angular_velocity

# ================== Simulation Environment ==================
class SimulationEnvironment:
    """Simulation environment for CBDR avoidance"""
    
    def __init__(self):
        """Initialize the simulation environment"""
        # Create ships
        self.ownship = Ship(
            name="Ownship",
            position=[0, 0],
            velocity=5.0,
            heading=0.0,
            size=20.0,
            max_turn_rate=10.0,
            max_angular_accel=5.0
        )
        
        self.ship_a = Ship(
            name="Ship A",
            position=[500, 100],
            velocity=5.0,
            heading=180.0,
            size=20.0,
            max_turn_rate=3.0,
            max_angular_accel=1.0
        )
        
        # Create controller
        self.controller = BarrierController(
            base_alpha=0.5,
            k_gain=0.3,
            omega_min=0.1
        )
        
        # Simulation parameters
        self.time = 0.0
        self.dt = 0.1  # Time step
        self.simulation_time = 120.0  # Total simulation time
        
        # History storage
        self.time_history = []
        self.distance_history = []
        self.bearing_history = []
        self.angular_size_history = []
        
        # Collision detection
        self.collision_detected = False
        self.min_distance = float('inf')
    
    def update(self):
        """Update the simulation for one time step"""
        # Record current state
        self.time_history.append(self.time)
        
        # Get bearing and angular size
        bearing = self.ownship.get_relative_bearing(self.ship_a)
        angular_size = self.ownship.get_angular_size(self.ship_a)
        distance = self.ownship.get_distance(self.ship_a)
        
        # Record for plotting
        self.bearing_history.append(bearing)
        self.angular_size_history.append(angular_size)
        self.distance_history.append(distance)
        
        # Update minimum distance
        self.min_distance = min(self.min_distance, distance)
        
        # Compute control command
        angular_velocity_command = self.controller.compute_control(
            bearing, angular_size, self.dt, self.time
        )
        
        # Apply control command and update ships
        self.ownship.update(self.dt, angular_velocity_command)
        self.ship_a.update(self.dt)
        
        # Record history
        self.ownship.record_history(self.time)
        self.ship_a.record_history(self.time)
        
        # Check for collision
        self.collision_detected = self.ownship.check_collision(self.ship_a)
        
        # Increment time
        self.time += self.dt
    
    def run_simulation(self):
        """Run the simulation"""
        print("Starting optimized CBDR Barrier Function Simulation...")
        print(f"Ownship max turn rate: {np.degrees(self.ownship.max_turn_rate):.1f} deg/s")
        print(f"Ownship max angular acceleration: {np.degrees(self.ownship.max_angular_accel):.1f} deg/s²")
        print(f"Ship A max turn rate: {np.degrees(self.ship_a.max_turn_rate):.1f} deg/s")
        
        with Timer():
            # Main simulation loop
            while self.time < self.simulation_time and not self.collision_detected:
                self.update()
        
        # Print results
        print("\n===== Simulation Results =====")
        print(f"Minimum distance between ships: {self.min_distance:.2f} m")
        collision_occurred = self.min_distance < (self.ownship.size + self.ship_a.size)
        print(f"Collision: {'Yes' if collision_occurred else 'No'}")
    
    def plot_trajectories(self, ax):
        """Plot ship trajectories"""
        if not self.ownship.history['time']:
            return
        
        # Extract trajectory data
        ownship_positions = np.array(self.ownship.history['position'])
        ship_a_positions = np.array(self.ship_a.history['position'])
        
        # Plot trajectories
        ax.plot(ownship_positions[:, 0], ownship_positions[:, 1], 'b-', label=self.ownship.name)
        ax.plot(ship_a_positions[:, 0], ship_a_positions[:, 1], 'r-', label=self.ship_a.name)
        
        # Mark starting positions
        ax.plot(ownship_positions[0, 0], ownship_positions[0, 1], 'bo', markersize=10)
        ax.plot(ship_a_positions[0, 0], ship_a_positions[0, 1], 'ro', markersize=10)
        
        # Add heading arrows at regular intervals
        step = max(1, len(ownship_positions) // 10)
        
        for i in range(0, len(ownship_positions), step):
            # Ownship heading arrow
            heading_rad = np.radians(self.ownship.history['heading'][i])
            ax.arrow(
                ownship_positions[i, 0], ownship_positions[i, 1],
                20 * np.cos(heading_rad), 20 * np.sin(heading_rad),
                head_width=5, head_length=10, fc='b', ec='b', alpha=0.5
            )
            
            # Ship A heading arrow
            heading_rad = np.radians(self.ship_a.history['heading'][i])
            ax.arrow(
                ship_a_positions[i, 0], ship_a_positions[i, 1],
                20 * np.cos(heading_rad), 20 * np.sin(heading_rad),
                head_width=5, head_length=10, fc='r', ec='r', alpha=0.5
            )
        
        # Add collision circles at end positions
        ownship_end = ownship_positions[-1]
        ship_a_end = ship_a_positions[-1]
        
        ownship_circle = plt.Circle(
            (ownship_end[0], ownship_end[1]), 
            self.ownship.collision_radius, 
            color='b', fill=False, linestyle='--', alpha=0.7
        )
        ship_a_circle = plt.Circle(
            (ship_a_end[0], ship_a_end[1]), 
            self.ship_a.collision_radius, 
            color='r', fill=False, linestyle='--', alpha=0.7
        )
        
        ax.add_patch(ownship_circle)
        ax.add_patch(ship_a_circle)
        
        # Set axis properties
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Ship Trajectories')
        ax.legend(loc='best')
        ax.grid(True)
        ax.axis('equal')
    
    def plot_bearing(self, ax):
        """Plot bearing to Ship A over time"""
        if self.time_history and self.bearing_history:
            ax.plot(self.time_history, self.bearing_history, 'g-', label='Bearing to Ship A')
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Bearing (deg)')
            ax.set_title('Bearing to Ship A Over Time')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_los_rate(self, ax):
        """Plot LOS rate and alpha threshold"""
        if self.controller.history_time:
            # Plot LOS rate
            ax.plot(self.controller.history_time, self.controller.history_omega, 'r-', label='LOS Rate (dθ/dt)')
            
            # Plot alpha threshold
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'g--', label='α threshold')
            ax.plot(self.controller.history_time, [-a for a in self.controller.history_alpha_threshold], 'g--')
            
            # Add explanation text
            ax.text(0.05, 0.95, f'α_threshold = safety threshold\n(not angular size)', 
                    transform=ax.transAxes, fontsize=10, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('dθ/dt (deg/s)')
            ax.set_title('LOS Rate and Safety Threshold')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_control(self, ax):
        """Plot control commands"""
        if self.controller.history_time:
            # Add turn rate limit lines
            ax.axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Max Turn Rate')
            ax.axhline(y=-10, color='r', linestyle='--', alpha=0.5)
            
            ax.plot(self.controller.history_time, self.controller.history_control, 'm-', label='Commanded Turn Rate')
            
            # Add actual turn rate
            if self.ownship.history['time'] and self.ownship.history['angular_velocity']:
                ax.plot(self.ownship.history['time'], self.ownship.history['angular_velocity'], 'c--', label='Actual Turn Rate')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angular Velocity (deg/s)')
            ax.set_title('Control Commands (Turn Rate)')
            ax.grid(True)
            ax.legend(loc='best')
    
    def plot_angular_size_and_threshold(self, ax):
        """Plot angular size and alpha threshold"""
        if self.time_history and self.angular_size_history and self.controller.history_alpha_threshold:
            # Plot angular size
            ax.plot(self.time_history, self.angular_size_history, 'c-', label='Angular Size')
            
            # Plot alpha threshold
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'm--', label='α threshold')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Degrees')
            ax.set_title('Angular Size and α Threshold')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_distance(self, ax):
        """Plot distance between ships"""
        if self.time_history and self.distance_history:
            ax.plot(self.time_history, self.distance_history, 'k-', label='Distance')
            
            # Add safe distance line
            safe_distance = self.ownship.size + self.ship_a.size
            ax.axhline(y=safe_distance, color='r', linestyle='--', alpha=0.7, label='Safe Distance')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Distance (m)')
            ax.set_title('Distance Between Ships')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_results(self):
        """Plot simulation results"""
        if not self.time_history:
            print("No simulation data to plot")
            return
        
        # Create figure
        fig, axs = plt.subplots(3, 2, figsize=(15, 18))
        fig.suptitle('CBDR Barrier Function Simulation Results', fontsize=16)
        
        # 1. Ship trajectories
        self.plot_trajectories(axs[0, 0])
        
        # 2. Bearing plot
        self.plot_bearing(axs[0, 1])
        
        # 3. LOS rate and threshold plot
        self.plot_los_rate(axs[1, 0])
        
        # 4. Control commands plot
        self.plot_control(axs[1, 1])
        
        # 5. Angular size and threshold plot
        self.plot_angular_size_and_threshold(axs[2, 0])
        
        # 6. Distance plot
        self.plot_distance(axs[2, 1])
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig('angular_obstacle_avoidance_simulation.png', dpi=150)
        plt.show()

# ================== Main Program ==================
if __name__ == "__main__":
    env = SimulationEnvironment()
    env.run_simulation()
    env.plot_results()
    print("Simulation completed.")