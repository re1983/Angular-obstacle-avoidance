import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial import distance
import math

# ================== Performance Timer ==================
# 性能计时器 - 用于测量代码块执行时间
class Timer:
    """Context manager for timing code blocks"""
    def __enter__(self):
        self.start = time.time()
        return self
    
    def __exit__(self, *args):
        self.end = time.time()
        print(f"Execution time: {self.end - self.start:.3f} seconds")

# ================== Angle Utilities ==================
# 角度工具 - 角度标准化和差值计算
def normalize_angle(angle):
    """Normalize angle to [-180, 180] degrees"""
    return ((angle + 180) % 360) - 180

def angle_difference(angle1, angle2):
    """Calculate the smallest difference between two angles"""
    diff = angle2 - angle1
    return normalize_angle(diff)

# ================== SO(2) Lie Group Implementation ==================
# SO(2) 李群实现 - 用于高效旋转计算
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
# 船舶模型 - 带物理约束的船舶模型
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
        Calculate relative bearing to another ship (degrees) - FIXED VERSION
        修正版：處理角度跳躍問題
        """
        delta_pos = other_ship.position - self.position
        angle_to_other = np.arctan2(delta_pos[1], delta_pos[0])
        
        # Convert to relative bearing
        relative_bearing = angle_to_other - self.orientation.angle
        
        # Normalize to [-π, π] then convert to degrees
        relative_bearing = ((relative_bearing + np.pi) % (2 * np.pi)) - np.pi
        bearing_degrees = np.degrees(relative_bearing)
        
        # Additional normalization to ensure [-180, 180]
        return normalize_angle(bearing_degrees)
    
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

# ================== Optimized Barrier Function Controller ==================
# 优化屏障函数控制器 - 带平滑过渡和CBDR避免
class OptimizedBarrierController:
    """Optimized Barrier Function Controller with smooth transitions and CBDR avoidance"""
    __slots__ = ('base_alpha', 'k_gain', 'smooth_factor', 'velocity_factor',
                 'prev_bearing', 'prev_control', 'cbdr_threshold', 
                 'history_time', 'history_bearing', 'history_omega', 
                 'history_control', 'history_alpha_threshold', 'history_angular_size')
    
    def __init__(self, base_alpha=0.5, k_gain=0.3, smooth_factor=0.2, velocity_factor=0.01):
        """
        Initialize optimized controller
        
        Parameters:
            base_alpha (float): Base safety threshold (deg/s)
            k_gain (float): Control gain
            smooth_factor (float): Smoothing factor for control transitions (0-1)
            velocity_factor (float): Velocity scaling factor for barrier function
        """
        self.base_alpha = base_alpha
        self.k_gain = k_gain
        self.smooth_factor = smooth_factor
        self.velocity_factor = velocity_factor
        self.prev_bearing = None
        self.prev_control = 0.0
        self.cbdr_threshold = 0.1  # CBDR detection threshold (deg/s^2)
        
        # Initialize history
        self.history_time = []
        self.history_bearing = []
        self.history_omega = []
        self.history_control = []
        self.history_alpha_threshold = []
        self.history_angular_size = []
    
    def compute_control(self, current_bearing, angular_size, dt, current_time, ownship, goal):
        """
        Compute optimized control action with smooth transitions and CBDR avoidance
        """
        if self.prev_bearing is None:
            self.prev_bearing = current_bearing
            return 0.0
        
        # Calculate bearing rate (LOS rate)
        bearing_diff = angle_difference(self.prev_bearing, current_bearing)
        omega = bearing_diff / dt
        self.prev_bearing = current_bearing
        
        # Adaptive threshold with velocity scaling
        velocity_scale = 1.0 + self.velocity_factor * ownship.velocity
        alpha_threshold = self.base_alpha * velocity_scale + 0.1 * angular_size
        
        # Check for CBDR situation (Constant Bearing, Decreasing Range)
        # CBDR is indicated by near-zero bearing rate combined with decreasing range
        is_cbdr = False
        if len(self.history_omega) > 1:
            # Calculate jerk (derivative of bearing rate)
            jerk = (omega - self.history_omega[-1]) / dt
            # CBDR condition: low bearing rate and negative jerk
            if abs(omega) < 1.0 and jerk < -self.cbdr_threshold:
                is_cbdr = True
        
        # Calculate barrier constraint
        constraint = abs(omega) - alpha_threshold
        
        # Navigation control: steer toward goal
        goal_bearing = ownship.get_relative_bearing(goal)
        goal_distance = ownship.get_distance(goal)
        
        # Calculate goal alignment control (more aggressive when far from goal)
        goal_heading_diff = angle_difference(ownship.get_heading_degrees(), 
                                            (goal_bearing + ownship.get_heading_degrees()) % 360)
        alignment_control = 0.15 * np.radians(goal_heading_diff) * min(1.0, goal_distance/200)
        
        # Navigation gain decreases as we approach goal
        nav_gain = 0.2 if goal_distance > 50 else 0.1
        navigation_control = nav_gain * np.radians(goal_bearing) + alignment_control
        
        # Collision avoidance control
        if constraint < 0 or is_cbdr:  # Collision threat or CBDR detected
            # Smooth control transition using sigmoid function
            transition = 1 / (1 + np.exp(-10 * constraint))
            
            # Avoidance control with smooth transition
            avoidance_control = -self.k_gain * (1 - transition) * np.sign(omega)
            
            # Blend with previous control for smoothness
            angular_velocity = (1 - self.smooth_factor) * avoidance_control + \
                              self.smooth_factor * self.prev_control
        else:  # No collision threat
            # Use dedicated navigation control
            angular_velocity = navigation_control
        
        # Apply low-pass filtering to reduce oscillations
        angular_velocity = (1 - self.smooth_factor) * angular_velocity + \
                          self.smooth_factor * self.prev_control
        
        # Save current control for next iteration smoothing
        self.prev_control = angular_velocity
        
        # Save history for analysis
        self.history_time.append(current_time)
        self.history_bearing.append(current_bearing)
        self.history_omega.append(omega)
        self.history_control.append(np.degrees(angular_velocity))
        self.history_alpha_threshold.append(alpha_threshold)
        self.history_angular_size.append(angular_size)
        
        return angular_velocity

# ================== Simulation Environment ==================
# 仿真环境 - 用于测试避障算法
class SimulationEnvironment:
    """Simulation environment for angular obstacle avoidance"""
    
    def __init__(self):
        """Initialize the simulation environment"""
        # Create ships
        self.ownship = Ship(
            name="Ownship",
            position=[0, 0],
            velocity=5.0,
            heading=90.0,
            size=20.0,
            max_turn_rate=10.0,
            max_angular_accel=5.0
        )
        
        self.ship_a = Ship(
            name="Ship A",
            position=[10, 500],
            velocity=5.0,
            heading=270.0,
            size=20.0,
            max_turn_rate=3.0,
            max_angular_accel=1.0
        )
        
        # Goal point
        self.goal = Ship(
            name="Goal",
            position=[0, 1000],
            velocity=0.0,
            heading=0.0,
            size=10.0
        )
        
        # Create optimized controller with increased smoothing
        self.controller = OptimizedBarrierController(
            base_alpha=0.5,
            k_gain=0.3,
            smooth_factor=0.5,  # Increased for smoother transitions
            velocity_factor=0.01
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
        self.time_history.append(self.time)
        
        bearing = self.ownship.get_relative_bearing(self.ship_a)
        angular_size = self.ownship.get_angular_size(self.ship_a)
        distance = self.ownship.get_distance(self.ship_a)
        
        self.bearing_history.append(bearing)
        self.angular_size_history.append(angular_size)
        self.distance_history.append(distance)
        
        self.min_distance = min(self.min_distance, distance)
        
        # Compute control
        angular_velocity_command = self.controller.compute_control(
            bearing, angular_size, self.dt, self.time, self.ownship, self.goal
        )
        
        # Update ship states
        self.ownship.update(self.dt, angular_velocity_command)
        self.ship_a.update(self.dt)
        
        # Record history
        self.ownship.record_history(self.time)
        self.ship_a.record_history(self.time)
        
        # Check for collision
        self.collision_detected = self.ownship.check_collision(self.ship_a)
        
        # Advance time
        self.time += self.dt
    
    def run_simulation(self):
        """Run the simulation"""
        print("Starting Optimized Barrier Function Simulation...")
        print(f"Ownship max turn rate: {np.degrees(self.ownship.max_turn_rate):.1f} deg/s")
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
        """Plot ship trajectories with goal"""
        if not self.ownship.history['time']:
            return
        
        ownship_positions = np.array(self.ownship.history['position'])
        ship_a_positions = np.array(self.ship_a.history['position'])
        
        # Plot trajectories
        ax.plot(ownship_positions[:, 0], ownship_positions[:, 1], 'b-', label=self.ownship.name)
        ax.plot(ship_a_positions[:, 0], ship_a_positions[:, 1], 'r-', label=self.ship_a.name)
        
        # Plot goal
        ax.plot(self.goal.position[0], self.goal.position[1], 'g*', markersize=15, label='Goal')
        
        # Starting positions
        ax.plot(ownship_positions[0, 0], ownship_positions[0, 1], 'bo', markersize=10)
        ax.plot(ship_a_positions[0, 0], ship_a_positions[0, 1], 'ro', markersize=10)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Ship Trajectories with Goal Navigation')
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
            ax.plot(self.controller.history_time, self.controller.history_omega, 'r-', 
                   label='LOS Rate (dθ/dt)', linewidth=2)
            
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'g--', 
                   label='α threshold')
            ax.plot(self.controller.history_time, [-a for a in self.controller.history_alpha_threshold], 'g--')
            
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('dθ/dt (deg/s)')
            ax.set_title('LOS Rate and Safety Threshold')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_control(self, ax):
        """Plot control commands"""
        if self.controller.history_time:
            ax.axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Max Turn Rate')
            ax.axhline(y=-10, color='r', linestyle='--', alpha=0.5)
            
            ax.plot(self.controller.history_time, self.controller.history_control, 'm-', 
                   label='Commanded Turn Rate')
            
            if self.ownship.history['time'] and self.ownship.history['angular_velocity']:
                ax.plot(self.ownship.history['time'], self.ownship.history['angular_velocity'], 
                       'c--', label='Actual Turn Rate')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angular Velocity (deg/s)')
            ax.set_title('Control Commands (Turn Rate)')
            ax.grid(True)
            ax.legend(loc='best')
    
    def plot_angular_size_and_threshold(self, ax):
        """Plot angular size and alpha threshold"""
        if self.time_history and self.angular_size_history and self.controller.history_alpha_threshold:
            ax.plot(self.time_history, self.angular_size_history, 'c-', label='Angular Size')
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'm--', 
                   label='α threshold')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Degrees')
            ax.set_title('Angular Size and α Threshold')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_distance(self, ax):
        """Plot distance between ships"""
        if self.time_history and self.distance_history:
            ax.plot(self.time_history, self.distance_history, 'k-', label='Distance')
            
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
        fig.suptitle('Optimized Barrier Function Simulation Results', fontsize=16)
        
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
        plt.savefig('optimized_angular_obstacle_avoidance.png', dpi=150)
        plt.show()

# ================== Main Program ==================
if __name__ == "__main__":
    # Create and run simulation
    env = SimulationEnvironment()
    env.run_simulation()
    env.plot_results()
    print("Optimized barrier function simulation completed successfully!")
