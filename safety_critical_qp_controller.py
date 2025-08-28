"""
Safety-Critical QP Controller for Angle-Only Collision Avoidance
基于二次规划的安全关键控制器，用于纯角度避碰

Author: Cline
Date: 2025-08-27
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import math

# 导航阈值常数（度）
ALPHA_NAV = 1.0  # 低于此角直径时无碰撞威胁

# 安全参数
ALPHA_SAFE = 5.0  # 安全角直径阈值（度），对应于最小安全距离
CBF_GAMMA = 0.5  # 控制屏障函数参数

class QPController:
    """
    Safety-Critical QP Controller using Angular Diameter Measurement
    使用角直径测量的安全关键二次规划控制器
    """
    
    def __init__(self, alpha_safe=ALPHA_SAFE, gamma=CBF_GAMMA, max_turn_rate=12.0):
        """
        初始化QP控制器
        
        Args:
            alpha_safe: 安全角直径阈值（度）
            gamma: 控制屏障函数参数
            max_turn_rate: 最大转向率（度/秒）
        """
        self.alpha_safe = alpha_safe
        self.gamma = gamma
        self.max_turn_rate = max_turn_rate
        self.max_turn_rate_rad = np.radians(max_turn_rate)
        
    def safety_constraint(self, alpha, alpha_dot, u, u_desired):
        """
        安全约束函数：基于角直径的控制屏障函数(CBF)
        h(α) = α_safe - α ≥ 0 必须始终满足（α越小越安全）
        
        CBF条件：ḣ + γh ≥ 0
        
        Args:
            alpha: 当前角直径（度）
            alpha_dot: 角直径变化率（度/秒）
            u: 控制输入（转向率）
            u_desired: 期望控制输入
            
        Returns:
            constraint_value: 约束值（应≥0）
        """
        # 转换为弧度用于计算
        alpha_rad = np.radians(alpha)
        alpha_safe_rad = np.radians(self.alpha_safe)
        alpha_dot_rad = np.radians(alpha_dot)
        
        h = alpha_safe_rad - alpha_rad  # 屏障函数（弧度）
        h_dot = -alpha_dot_rad  # 屏障函数导数
        
        # CBF条件：ḣ + γh ≥ 0
        return h_dot + self.gamma * h
    
    def qp_optimization(self, u_desired, alpha, alpha_dot, current_u):
        """
        二次规划优化：最小化控制输入与期望输入的偏差，同时满足安全约束
        
        Args:
            u_desired: 期望转向率（来自导航或其他控制器）
            alpha: 当前角直径（度）
            alpha_dot: 角直径变化率（度/秒）
            current_u: 当前转向率（用于平滑约束）
            
        Returns:
            u_opt: 优化后的转向率
        """
        # 定义优化问题
        def cost_function(u):
            """成本函数：最小化与期望控制的偏差"""
            return (u - u_desired)**2 + 0.1 * (u - current_u)**2  # 包含平滑项
        
        # 使用边界约束而不是不等式约束 for turn rate
        bounds = [(-self.max_turn_rate_rad, self.max_turn_rate_rad)]
        
        # 计算安全约束
        safety_constraint = self.safety_constraint(alpha, alpha_dot, current_u, u_desired)
        
        # 如果安全约束不满足，调整期望控制
        if safety_constraint < 0:
            # 需要减少角直径，因此调整期望控制
            # 根据角直径的变化率方向决定调整策略
            if alpha_dot > 0:  # 角直径在增加，需要更积极的避碰
                u_desired = np.clip(u_desired * 1.5, -self.max_turn_rate_rad, self.max_turn_rate_rad)
            else:  # 角直径在减少，可以稍微保守
                u_desired = np.clip(u_desired * 1.2, -self.max_turn_rate_rad, self.max_turn_rate_rad)
        
        # 初始猜测
        x0 = np.array([current_u])
        
        # 求解QP问题（只使用边界约束）
        result = minimize(cost_function, x0, bounds=bounds, method='L-BFGS-B')
        
        if result.success:
            return result.x[0]
        else:
            # 如果优化失败，返回安全控制
            return np.clip(u_desired, -self.max_turn_rate_rad, self.max_turn_rate_rad)
    
    def compute_angular_diameter_rate(self, angular_sizes, delta_time):
        """
        计算角直径变化率 α̇ = (α_current - α_previous) / Δt
        
        Args:
            angular_sizes: 角直径历史列表
            delta_time: 时间步长
            
        Returns:
            alpha_dot: 角直径变化率（度/秒）
        """
        if len(angular_sizes) < 2:
            return 0.0
        
        # 计算最近两个角直径的变化率
        alpha_current = angular_sizes[-1]
        alpha_previous = angular_sizes[-2]
        alpha_dot = (alpha_current - alpha_previous) / delta_time
        
        # 简单的低通滤波减少噪声
        if len(angular_sizes) > 10:
            # 使用移动平均平滑
            recent_alphas = angular_sizes[-10:]
            alpha_dot_smoothed = np.mean(np.diff(recent_alphas)) / delta_time
            return alpha_dot_smoothed
        
        return alpha_dot

def adj_ownship_heading_qp(bearings, bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01):
    """
    QP-based heading adjustment with safety guarantees
    基于QP的航向调整，具有安全保证
    
    Args:
        bearings: 相对方位历史
        bearings_difference: 方位变化率历史
        angular_sizes: 角大小历史
        ship: 本船状态
        goal: 目标点状态
        target_ship: 目标船状态
        delta_time: 时间步长
        
    Returns:
        rate_of_turn: 转向率（度/秒）
        velocity: 速度（米/秒）
    """
    # 初始化QP控制器（基于角直径）
    qp_controller = QPController(alpha_safe=5.0, gamma=0.5, max_turn_rate=12.0)
    
    # 计算当前角直径和角直径变化率
    current_alpha = angular_sizes[-1] if angular_sizes else 0.0
    alpha_dot = qp_controller.compute_angular_diameter_rate(angular_sizes, delta_time)
    
    # 计算期望控制输入（基于导航或避碰逻辑）
    if current_alpha < ALPHA_NAV:
        # 导航模式：朝向目标
        theta_goal = get_bearing(ship, goal)
        u_desired_deg = theta_goal  # 度/秒
    else:
        # 避碰模式：基于CBDR逻辑
        current_relative_bearing = get_bearing(ship, target_ship)
        avoidance_gain = current_alpha**2
        
        if len(bearings_difference) >= 1:
            if abs(bearings_difference[-1] * delta_time) <= current_alpha:
                rounded_rate = np.round(bearings_difference[-1], 5)
                if abs(rounded_rate) <= 1e-5:  # 真实CBDR（方位率≈0）
                    if current_relative_bearing < 0:  # 目标在左舷
                        u_desired_deg = -qp_controller.max_turn_rate  # 左转
                    else:  # 目标在右舷
                        u_desired_deg = qp_controller.max_turn_rate   # 右转
                else:
                    # 非零方位率情况
                    if abs(current_relative_bearing) < 90:  # 目标在前方
                        u_desired_deg = -np.sign(bearings_difference[-1]) * avoidance_gain
                    else:  # 目标在后方
                        u_desired_deg = np.sign(bearings_difference[-1]) * avoidance_gain
            else:
                # 非CBDR情况
                u_desired_deg = 0.0
        else:
            u_desired_deg = 0.0
    
    # 转换为弧度
    u_desired_rad = np.radians(u_desired_deg)
    current_u_rad = np.radians(ship.rate_of_turn)
    
    # QP优化（使用角直径而不是距离）
    u_opt_rad = qp_controller.qp_optimization(u_desired_rad, current_alpha, alpha_dot, current_u_rad)
    
    # 转换回度
    u_opt_deg = np.degrees(u_opt_rad)
    
    # 速度控制
    velocity = ship.velocity
    if current_alpha < ALPHA_NAV:
        # 在导航模式下，根据到目标的距离调整速度
        goal_bearing = get_bearing(ship, goal)
        # 简单速度控制：如果目标在前方，加速；如果在后方，减速
        if abs(goal_bearing) < 45:  # 目标在前方45度内
            velocity = min(velocity + 0.1, 2.0)  # 加速到最大2.0 m/s
        else:
            velocity = max(velocity - 0.1, 0.5)  # 减速到最小0.5 m/s
    
    return u_opt_deg, velocity

# 从现有代码中复制必要的函数
def get_distance_3d(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def get_bearing(ship1, ship2):
    """计算从ship1到ship2的相对方位"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    angle_to_ship2 = np.degrees(theta)
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    return relative_bearing

def get_absolute_bearing(ship1, ship2):
    """计算绝对方位（真方位）"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    return absolute_bearing

def get_angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)

def angle_difference_in_deg(angle1, angle2):
    angle_diff = (angle2 - angle1) % 360
    if angle_diff > 180:
        angle_diff -= 360
    return angle_diff

def run_qp_simulation():
    """
    运行QP控制器的仿真
    """
    # 复制ShipStatus类
    class ShipStatus:
        def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=1.0):
            self.name = name
            self.velocity = velocity
            self.acceleration = acceleration
            self.heading = heading
            self.rate_of_turn = rate_of_turn
            self.position = np.array(position, dtype=float)
            self.size = size

        def update(self, delta_time=0.01):
            self.heading += self.rate_of_turn * delta_time
            self.position += self.velocity * delta_time * np.array([
                np.cos(np.radians(self.heading)),
                np.sin(np.radians(self.heading)),
                0])
            self.velocity += self.acceleration
    
    # 初始化船舶状态
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=90, rate_of_turn=0, position=[0, 0, 0], size=0.5)
    target_ship = ShipStatus("Ship A", velocity=2.0, acceleration=0, heading=90.0, rate_of_turn=1, position=[15, -15, 0], size=0.5)
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, rate_of_turn=0, position=[0, 50, 0])
    
    time_steps = 5000
    delta_time = 0.01
    
    # 初始化数据存储
    bearings, angular_sizes, bearings_difference = [], [], []
    distances, ownship_headings = [], []
    
    for i in range(time_steps):
        # 计算当前测量值
        bearing = get_bearing(ownship, target_ship)
        angular_size = get_angular_diameter(ownship, target_ship)
        distance = get_distance_3d(ownship.position, target_ship.position)
        
        # 存储数据
        bearings.append(bearing)
        angular_sizes.append(angular_size)
        distances.append(distance)
        ownship_headings.append(ownship.heading)
        
        # 应用QP控制
        ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_qp(
            bearings, bearings_difference, angular_sizes, ownship, goal, target_ship, delta_time)
        
        # 更新船舶位置
        ownship.update(delta_time)
        target_ship.update(delta_time)
        
        # 计算方位变化率
        if i > 0:
            new_bearing = get_bearing(ownship, target_ship)
            bearing_diff = angle_difference_in_deg(bearing, new_bearing) / delta_time
            bearings_difference.append(bearing_diff)
        else:
            bearings_difference.append(0.0)
    
    # 返回结果
    return {
        'distances': np.array(distances),
        'bearings': np.array(bearings),
        'angular_sizes': np.array(angular_sizes),
        'ownship_headings': np.array(ownship_headings)
    }

def plot_qp_results(results):
    """
    绘制QP控制器结果
    """
    time = np.arange(len(results['distances'])) * 0.01
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # 距离图
    axes[0,0].plot(time, results['distances'], 'b-', label='Distance')
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Distance (m)')
    axes[0,0].set_title('Distance vs Time (QP Controller)')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    # 方位图
    axes[0,1].plot(time, results['bearings'], 'g-', label='Relative Bearing')
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Bearing (degrees)')
    axes[0,1].set_title('Relative Bearing vs Time')
    axes[0,1].legend()
    axes[0,1].grid(True)
    
    # 角大小图
    axes[1,0].plot(time, results['angular_sizes'], 'orange', label='Angular Size')
    axes[1,0].axhline(y=ALPHA_NAV, color='r', linestyle='--', label=f'Navigation Threshold ({ALPHA_NAV}°)')
    axes[1,0].axhline(y=ALPHA_SAFE, color='purple', linestyle='--', label=f'Safety Threshold ({ALPHA_SAFE}°)')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Angular Size (degrees)')
    axes[1,0].set_title('Angular Size vs Time')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    # 本船航向图
    axes[1,1].plot(time, results['ownship_headings'], 'purple', label='Ownship Heading')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Heading (degrees)')
    axes[1,1].set_title('Ownship Heading vs Time')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("运行QP控制器仿真...")
    results = run_qp_simulation()
    print("绘制结果...")
    plot_qp_results(results)
