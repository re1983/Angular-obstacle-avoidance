import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial import distance

# ================== 性能优化计时器 ==================
class Timer:
    """Context manager for timing code blocks 用于计时代码块的上下文管理器"""
    def __enter__(self):
        self.start = time.time()
        return self
    
    def __exit__(self, *args):
        self.end = time.time()
        self.interval = self.end - self.start
        print(f"Operation took {self.interval:.4f} seconds")

# ================== SO(2) 李群实现 (优化版) ==================
class SO2:
    """
    Optimized SO(2) Special Orthogonal Group in 2D
    优化的二维特殊正交群实现
    """
    __slots__ = ('angle', 'matrix')  # 减少内存占用
    
    def __init__(self, angle=0.0):
        """
        Initialize with rotation angle (radians)
        使用旋转角度初始化（弧度）
        """
        self.angle = float(angle)
        self.update_matrix()
    
    def update_matrix(self):
        """Update rotation matrix from angle 从角度更新旋转矩阵"""
        c, s = np.cos(self.angle), np.sin(self.angle)
        self.matrix = np.array([[c, -s], [s, c]])
    
    def rotate_vector(self, vector):
        """
        Rotate a 2D vector (optimized)
        旋转二维向量（优化版）
        """
        c, s = np.cos(self.angle), np.sin(self.angle)
        x, y = vector
        return np.array([c*x - s*y, s*x + c*y])
    
    def inverse(self):
        """Return inverse rotation 返回逆旋转"""
        return SO2(-self.angle)
    
    def compose(self, other):
        """Compose two rotations 组合两个旋转"""
        return SO2(self.angle + other.angle)
    
    def __mul__(self, other):
        """Rotation composition 旋转组合"""
        if isinstance(other, SO2):
            return self.compose(other)
        elif isinstance(other, np.ndarray) and other.shape == (2,):
            return self.rotate_vector(other)
        else:
            raise TypeError("Unsupported operand type")
    
    def __repr__(self):
        return f"SO2(θ={np.degrees(self.angle):.2f}°)"

# ================== 船舶模型 (优化版) ==================
class Ship:
    """
    Optimized ship dynamics model with collision detection
    带碰撞检测的优化船舶动力学模型
    """
    __slots__ = ('name', 'position', 'velocity', 'orientation', 'size', 'history', 'collision_radius',
                 'max_turn_rate', 'max_angular_accel', 'current_angular_velocity')
    
    def __init__(self, name, position, velocity, heading, size=10.0, max_turn_rate=10.0, max_angular_accel=10.0):
        """
        Initialize ship state
        初始化船舶状态
        
        Parameters:
            name (str): Ship identifier 船舶标识
            position (np.array): [x, y] position (meters) 位置 [x, y] (米)
            velocity (float): Speed (m/s) 速度 (米/秒)
            heading (float): Initial heading (degrees) 初始航向 (度)
            size (float): Ship size (meters) for collision detection 船舶尺寸 (米) 用于碰撞检测
            max_turn_rate (float): Maximum turn rate (deg/s) 最大转向率 (度/秒)
            max_angular_accel (float): Maximum angular acceleration (deg/s²) 最大角加速度 (度/秒²)
        """
        self.name = name
        self.position = np.array(position, dtype=np.float64)
        self.velocity = float(velocity)
        self.orientation = SO2(np.radians(heading))
        self.size = float(size)
        self.collision_radius = size * 1.2  # 碰撞检测半径
        self.max_turn_rate = np.radians(max_turn_rate)  # 转换为弧度/秒
        self.max_angular_accel = np.radians(max_angular_accel)  # 转换为弧度/秒²
        self.current_angular_velocity = 0.0  # 当前角速度
        
        # 优化历史记录 - 存储每一步
        self.history = {
            'position': [self.position.copy()],
            'heading': [float(heading)],
            'velocity': [self.velocity],
            'angular_velocity': [0.0],
            'time': [0.0]
        }
    
    def update(self, dt, commanded_angular_velocity, current_time=None):
        """
        Update ship state (optimized)
        更新船舶状态（优化版）
        """
        # 角加速度限制
        angular_accel = (commanded_angular_velocity - self.current_angular_velocity) / dt
        max_accel = self.max_angular_accel
        
        # 限制角加速度
        if angular_accel > max_accel:
            angular_accel = max_accel
        elif angular_accel < -max_accel:
            angular_accel = -max_accel
        
        # 更新角速度
        self.current_angular_velocity += angular_accel * dt
        
        # 限制转向率在物理范围内
        if self.current_angular_velocity > self.max_turn_rate:
            self.current_angular_velocity = self.max_turn_rate
        elif self.current_angular_velocity < -self.max_turn_rate:
            self.current_angular_velocity = -self.max_turn_rate
        
        # Update orientation 更新方向
        self.orientation = SO2(self.orientation.angle + self.current_angular_velocity * dt)
        
        # Update position 更新位置
        # 修正：0°对应正北（y轴正方向），90°对应正东（x轴正方向）
        heading_vector = np.array([
            np.sin(self.orientation.angle),  # x分量 - 东
            np.cos(self.orientation.angle)   # y分量 - 北
        ])
        self.position += self.velocity * heading_vector * dt
        
        # 记录每一步的历史
        if current_time is not None:
            self.history['position'].append(self.position.copy())
            self.history['heading'].append(np.degrees(self.orientation.angle))
            self.history['velocity'].append(self.velocity)
            self.history['angular_velocity'].append(np.degrees(self.current_angular_velocity))
            self.history['time'].append(current_time)
    
    def get_relative_position(self, other):
        """
        Get relative position to another ship in ownship frame
        获取相对于另一艘船的位置（在本船坐标系中）
        """
        return self.orientation.inverse().rotate_vector(other.position - self.position)
    
    def get_bearing(self, other):
        """
        Calculate bearing to another ship (degrees) (optimized)
        计算到另一艘船的方位角 (度)（优化版）
        """
        rel_pos = other.position - self.position
        rel_pos_local = self.orientation.inverse().rotate_vector(rel_pos)
        return np.degrees(np.arctan2(rel_pos_local[1], rel_pos_local[0]))
    
    def get_distance(self, other):
        """Calculate distance to another ship (optimized) 计算到另一艘船的距离（优化版）"""
        return distance.euclidean(self.position, other.position)
    
    def get_angular_size(self, other):
        """Calculate angular size of another ship (degrees) (optimized) 计算另一艘船的角大小 (度)（优化版）"""
        dist = self.get_distance(other)
        if dist < 1e-6:  # Avoid division by zero 避免除以零
            return 90.0
        return np.degrees(2 * np.arctan(other.size / (2 * dist)))
    
    def check_collision(self, other, threshold=1.0):
        """
        Check collision with another ship
        检查与另一艘船的碰撞
        """
        dist = self.get_distance(other)
        return dist < (self.collision_radius + other.collision_radius) * threshold

# ================== 屏障函数控制器 (优化版) ==================
class BarrierController:
    """
    Optimized Barrier Function Controller for CBDR Avoidance
    用于CBDR避免的优化屏障函数控制器
    """
    __slots__ = ('base_alpha', 'k_gain', 'omega_min', 'prev_bearing', 
                 'alpha_factor', 'adaptive_threshold', 'collision_warning',
                 'history_time', 'history_bearing', 'history_omega', 
                 'history_control', 'history_alpha_threshold', 'history_angular_size')
    
    def __init__(self, base_alpha=0.5, k_gain=0.3, omega_min=0.1):
        """
        Initialize controller parameters
        初始化控制器参数
        
        Parameters:
            base_alpha (float): Base safety threshold for dθ/dt (deg/s) dθ/dt的基础安全阈值 (度/秒)
            k_gain (float): Controller gain 控制器增益
            omega_min (float): Minimum LOS rate to consider (deg/s) 考虑的最小LOS率 (度/秒)
        """
        self.base_alpha = base_alpha
        self.k_gain = k_gain
        self.omega_min = omega_min
        self.prev_bearing = None
        self.alpha_factor = 0.2  # Angular size influence factor 角大小影响因子
        self.adaptive_threshold = True  # Enable adaptive threshold 启用自适应阈值
        self.collision_warning = False
        
        # Initialize history arrays 初始化历史数组
        self.history_time = []
        self.history_bearing = []
        self.history_omega = []
        self.history_control = []
        self.history_alpha_threshold = []
        self.history_angular_size = []
    
    def compute_control(self, current_bearing, angular_size, dt, current_time):
        """
        Compute control action based on barrier function (optimized)
        基于屏障函数计算控制动作（优化版）
        
        Parameters:
            current_bearing (float): Current bearing to obstacle (deg) 到障碍物的当前方位角 (度)
            angular_size (float): Angular size of obstacle (deg) 障碍物的角大小 (度)
            dt (float): Time step (s) 时间步长 (秒)
            current_time (float): Current simulation time (s) 当前模拟时间 (秒)
            
        Returns:
            float: Angular velocity command (rad/s) 角速度指令 (弧度/秒)
        """
        # Initialize on first call 第一次调用时初始化
        if self.prev_bearing is None:
            self.prev_bearing = current_bearing
            return 0.0
        
        # Compute LOS rate (ω = dθ/dt) 计算LOS率 (ω = dθ/dt)
        omega = (current_bearing - self.prev_bearing) / dt
        self.prev_bearing = current_bearing
        
        # Calculate current alpha threshold 计算当前alpha阈值
        if self.adaptive_threshold:
            alpha_threshold = self.base_alpha + self.alpha_factor * angular_size
        else:
            alpha_threshold = self.base_alpha
        
        # Barrier function constraint: |ω| > α_threshold 屏障函数约束: |ω| > α_threshold
        constraint = abs(omega) - alpha_threshold
        
        # Control law: Apply correction when constraint is violated
        # 控制律: 当约束被违反时应用校正
        if constraint < 0:  # |ω| < α_threshold
            # 应用与约束违反成正比的校正角速度，方向与ω相反
            angular_velocity = self.k_gain * abs(constraint) * -np.sign(omega)
            self.collision_warning = True
        else:
            angular_velocity = 0.0
            self.collision_warning = False
        
        # Save history for plotting 保存历史用于绘图
        self.history_time.append(current_time)
        self.history_bearing.append(current_bearing)
        self.history_omega.append(omega)
        self.history_control.append(np.degrees(angular_velocity))
        self.history_alpha_threshold.append(alpha_threshold)
        self.history_angular_size.append(angular_size)
        
        return angular_velocity

# ================== 模拟环境 (优化版) ==================
class SimulationEnvironment:
    """
    Optimized simulation environment for ship navigation
    船舶导航优化模拟环境
    """
    def __init__(self):
        """Initialize simulation environment 初始化模拟环境"""
        # Create ships with sizes 创建带尺寸的船舶
        self.ownship = Ship("Ownship", position=[0, 0], velocity=10.0, heading=0.0, 
                           size=15.0, max_turn_rate=10.0, max_angular_accel=10.0)
        self.ship_a = Ship("Ship A", position=[1000, 1000], velocity=10.0, heading=270.0, 
                          size=20.0, max_turn_rate=5.0, max_angular_accel=5.0)
        self.goal = Ship("Goal", position=[0, 5000], velocity=0, heading=0, size=5.0)
        
        # Create controller 创建控制器
        self.controller = BarrierController(base_alpha=0.3, k_gain=0.4)
        
        # Simulation parameters 模拟参数
        self.dt = 0.5  # 时间步长 (秒)
        self.simulation_time = 120  # 总模拟时间 (秒)
        self.time = 0.0
        
        # 性能优化
        self.collision_detected = False
        self.min_distance = float('inf')
        self.collision_time = None
        self.distance_history = []
        self.time_history = []
        self.angular_size_history = []
    
    def update(self):
        """Update simulation for one time step 更新一个时间步的模拟"""
        # 更新时间
        current_time = self.time
        
        # 计算角大小
        angular_size = self.ownship.get_angular_size(self.ship_a)
        self.angular_size_history.append(angular_size)
        
        # 计算方位角
        bearing = self.ownship.get_bearing(self.ship_a)
        
        # 计算控制动作
        angular_velocity = self.controller.compute_control(bearing, angular_size, self.dt, current_time)
        
        # 更新本船状态（考虑角加速度限制）
        self.ownship.update(self.dt, angular_velocity, current_time)
        
        # 更新Ship A状态 (恒定速度和航向)
        self.ship_a.update(self.dt, 0.0, current_time=current_time)  # Ship A没有控制器，角速度指令为0
        
        # 检查碰撞
        dist = self.ownship.get_distance(self.ship_a)
        self.distance_history.append(dist)
        self.time_history.append(current_time)
        
        if dist < self.min_distance:
            self.min_distance = dist
        
        if not self.collision_detected and self.ownship.check_collision(self.ship_a):
            self.collision_detected = True
            self.collision_time = current_time
            print(f"! COLLISION DETECTED at t={current_time:.1f}s !")
        
        # 更新时间
        self.time += self.dt
    
    def run_simulation(self):
        """Run the simulation (optimized) 运行模拟（优化版）"""
        print("Starting optimized CBDR Barrier Function Simulation...")
        print(f"Ownship max turn rate: {np.degrees(self.ownship.max_turn_rate):.1f} deg/s")
        print(f"Ownship max angular acceleration: {np.degrees(self.ownship.max_angular_accel):.1f} deg/s²")
        print(f"Ship A max turn rate: {np.degrees(self.ship_a.max_turn_rate):.1f} deg/s")
        
        with Timer():
            # 主模拟循环
            while self.time < self.simulation_time and not self.collision_detected:
                self.update()
        
        # 打印结果
        print("\n===== Simulation Results =====")
        print(f"Minimum distance between ships: {self.min_distance:.2f} m")
        collision_occurred = self.min_distance < (self.ownship.size + self.ship_a.size)
        print(f"Collision occurred: {'Yes' if collision_occurred else 'No'}")
        if collision_occurred:
            print(f"Collision time: {self.collision_time:.1f}s")
        print(f"Final time: {self.time:.1f}s")
    
    def plot_results(self):
        """Plot simulation results 绘制模拟结果"""
        if not self.time_history:
            print("No simulation data to plot")
            return
        
        # 创建图表
        fig, axs = plt.subplots(3, 2, figsize=(15, 18))
        fig.suptitle('CBDR Barrier Function Simulation Results', fontsize=16)
        
        # 1. 船舶轨迹图
        self.plot_trajectories(axs[0, 0])
        
        # 2. 方位角图
        self.plot_bearing(axs[0, 1])
        
        # 3. LOS率与阈值图
        self.plot_los_rate(axs[1, 0])
        
        # 4. 控制指令图
        self.plot_control(axs[1, 1])
        
        # 5. 角大小与阈值图
        self.plot_angular_size_and_threshold(axs[2, 0])
        
        # 6. 距离图
        self.plot_distance(axs[2, 1])
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig('cbdr_barrier_simulation_optimized.png', dpi=150)
        plt.show()
    
    def plot_trajectories(self, ax):
        """Plot ship trajectories with collision zones 绘制船舶轨迹及碰撞区域"""
        # 确保有历史数据
        if not self.ownship.history['position'] or not self.ship_a.history['position']:
            return
        
        # 本船轨迹
        own_pos = np.array(self.ownship.history['position'])
        ax.plot(own_pos[:, 0], own_pos[:, 1], 'b-', label='Ownship Path', alpha=0.8, linewidth=2)
        ax.plot(own_pos[0, 0], own_pos[0, 1], 'bo', markersize=10, label='Ownship Start')
        ax.plot(own_pos[-1, 0], own_pos[-1, 1], 'bs', markersize=10, label='Ownship End')
        
        # Ship A轨迹
        ship_a_pos = np.array(self.ship_a.history['position'])
        ax.plot(ship_a_pos[:, 0], ship_a_pos[:, 1], 'r-', label='Ship A Path', alpha=0.8, linewidth=2)
        ax.plot(ship_a_pos[0, 0], ship_a_pos[0, 1], 'ro', markersize=10, label='Ship A Start')
        ax.plot(ship_a_pos[-1, 0], ship_a_pos[-1, 1], 'rs', markersize=10, label='Ship A End')
        
        # 目标位置
        ax.plot(self.goal.position[0], self.goal.position[1], 'g*', markersize=15, label='Goal')
        
        # 添加碰撞区域 - 使用历史位置中的最后一个
        own_last_pos = own_pos[-1]
        ship_a_last_pos = ship_a_pos[-1]
        
        own_circle = plt.Circle(own_last_pos, self.ownship.collision_radius, 
                               color='b', alpha=0.2, label='Ownship Collision Zone')
        ship_circle = plt.Circle(ship_a_last_pos, self.ship_a.collision_radius, 
                                color='r', alpha=0.2, label='Ship A Collision Zone')
        ax.add_patch(own_circle)
        ax.add_patch(ship_circle)
        
        # 添加航向箭头
        self.add_heading_arrow(ax, self.ownship)
        self.add_heading_arrow(ax, self.ship_a)
        
        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title('Ship Trajectories')
        ax.legend(loc='best')
        ax.grid(True)
        ax.axis('equal')
    
    def add_heading_arrow(self, ax, ship):
        """Add heading arrow for a ship 为船舶添加航向箭头"""
        # 使用当前位置和航向
        pos = ship.position
        heading_rad = ship.orientation.angle  # 弧度
        
        # 修正：0°对应正北（y轴正方向）
        dx = 50 * np.sin(heading_rad)  # 东方向分量
        dy = 50 * np.cos(heading_rad)  # 北方向分量
        
        color = 'b' if ship.name == "Ownship" else 'r'
        ax.arrow(pos[0], pos[1], dx, dy, 
                 head_width=15, head_length=20, fc=color, ec=color, alpha=0.8)
    
    def plot_bearing(self, ax):
        """Plot bearing to Ship A over time 绘制随时间变化的Ship A方位角"""
        if self.controller.history_time:
            ax.plot(self.controller.history_time, self.controller.history_bearing, 'b-')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Bearing to Ship A (deg)')
            ax.set_title('Bearing to Ship A')
            ax.grid(True)
    
    def plot_los_rate(self, ax):
        """Plot LOS rate and alpha threshold 绘制LOS率和alpha阈值"""
        if self.controller.history_time:
            # Plot LOS rate
            ax.plot(self.controller.history_time, self.controller.history_omega, 'r-', label='LOS Rate (dθ/dt)')
            
            # Plot alpha threshold
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'g--', label='α threshold')
            ax.plot(self.controller.history_time, [-a for a in self.controller.history_alpha_threshold], 'g--')
            
            # 添加说明文本
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
        """Plot control commands 绘制控制指令"""
        if self.controller.history_time:
            # 添加转向率限制线
            ax.axhline(y=10, color='r', linestyle='--', alpha=0.5, label='Max Turn Rate')
            ax.axhline(y=-10, color='r', linestyle='--', alpha=0.5)
            
            ax.plot(self.controller.history_time, self.controller.history_control, 'm-', label='Commanded Turn Rate')
            
            # 添加实际转向率
            if self.ownship.history['time'] and self.ownship.history['angular_velocity']:
                ax.plot(self.ownship.history['time'], self.ownship.history['angular_velocity'], 'c--', label='Actual Turn Rate')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Angular Velocity (deg/s)')
            ax.set_title('Control Commands (Turn Rate)')
            ax.grid(True)
            ax.legend(loc='best')
    
    def plot_angular_size_and_threshold(self, ax):
        """Plot angular size and alpha threshold 绘制角大小和alpha阈值"""
        if self.time_history and self.angular_size_history and self.controller.history_alpha_threshold:
            # 绘制角大小
            ax.plot(self.time_history, self.angular_size_history, 'c-', label='Angular Size')
            
            # 绘制alpha阈值
            # 注意：controller.history_time 和 time_history 可能不完全一致，但时间步相同
            ax.plot(self.controller.history_time, self.controller.history_alpha_threshold, 'm--', label='α threshold')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Degrees')
            ax.set_title('Angular Size and α Threshold')
            ax.legend(loc='best')
            ax.grid(True)
    
    def plot_distance(self, ax):
        """Plot distance between ships 绘制船舶间距离"""
        if self.time_history and self.distance_history:
            # 碰撞阈值
            collision_threshold = self.ownship.collision_radius + self.ship_a.collision_radius
            
            ax.plot(self.time_history, self.distance_history, 'm-', label='Distance')
            ax.axhline(y=collision_threshold, color='r', linestyle='--', label='Collision Threshold')
            
            # 标记最小距离
            min_dist = min(self.distance_history)
            min_time = self.time_history[np.argmin(self.distance_history)]
            ax.plot(min_time, min_dist, 'ro', markersize=8, label=f'Min Dist: {min_dist:.1f}m')
            
            # 标记碰撞时间（如果发生碰撞）
            if self.collision_detected:
                ax.axvline(x=self.collision_time, color='g', linestyle='--', label='Collision Time')
            
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Distance (m)')
            ax.set_title('Distance Between Ships')
            ax.legend(loc='best')
            ax.grid(True)

# ================== 主程序 ==================
if __name__ == "__main__":
    env = SimulationEnvironment()
    env.run_simulation()
    env.plot_results()
    print("Simulation completed.")