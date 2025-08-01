import numpy as np
import matplotlib.pyplot as plt

# ======================
# 参数设定
# ======================
dt = 0.1                # 时间步长（秒）
total_time = 30          # 总模拟时间（秒）
steps = int(total_time / dt)

# 物体真实参数
true_s = 10.0            # 真实尺寸 (10-70m)
true_obj_pos = np.array([100.0, 50.0])  # 初始位置 (x, y)
true_obj_vel = np.array([-2.0, 1.0])    # 速度 (vx, vy) 等速运动

# 观察者运动参数（已知信息）
obs_acc = np.array([0.5, -0.5])        # 观察者加速度 (ax, ay)
obs_vel_init = np.array([1.0, 2.0])    # 观察者初始速度
obs_pos_init = np.array([0.0, 0.0])    # 观察者初始位置

# 初始估计值（带噪声）
init_obj_pos = true_obj_pos + np.random.normal(0, 20, 2)
init_obj_vel = true_obj_vel + np.random.normal(0, 1, 2)
init_s = np.clip(true_s + np.random.normal(0, 10), 10, 70)  # 初始尺寸估计

# 过程噪声协方差（物体位置、速度、尺寸）
Q = np.diag([0.5, 0.5, 0.1, 0.1, 0.001])  # [x, y, vx, vy, s]

# 观测噪声协方差
R = np.diag([0.00001, 0.000001])              # 方位角 (rad), 角直径 (rad)

# ======================
# IEKF 类实现
# ======================
class InvariantEKF:
    def __init__(self, init_state, init_cov, Q, R):
        self.state = init_state.copy()  # [x, y, vx, vy, s]
        self.cov = init_cov.copy()
        self.Q = Q
        self.R = R

    def predict(self):
        # 等速运动模型
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt
        
        self.state = F @ self.state
        self.cov = F @ self.cov @ F.T + self.Q
        
        # 应用尺寸约束
        self.state[4] = np.clip(self.state[4], 10, 70)

    def update(self, z, obs_pos):
        x, y, vx, vy, s = self.state
        dx = x - obs_pos[0]
        dy = y - obs_pos[1]
        d = np.sqrt(dx**2 + dy**2)
        
        # 预测观测值
        phi_pred = np.arctan2(dy, dx)
        theta_pred = s / d
        
        # 观测残差
        e = np.array([z[0] - phi_pred, z[1] - theta_pred])
        
        # 计算雅可比矩阵 H
        H = np.zeros((2, 5))
        # 对x的偏导
        H[0, 0] = -dy / (dx**2 + dy**2)  # d(phi)/dx
        H[0, 1] = dx / (dx**2 + dy**2)   # d(phi)/dy
        H[1, 0] = (-s * dx) / (d**3)     # d(theta)/dx
        H[1, 1] = (-s * dy) / (d**3)     # d(theta)/dy
        H[1, 4] = 1.0 / d                # d(theta)/ds
        
        # 卡尔曼增益
        S = H @ self.cov @ H.T + self.R
        K = self.cov @ H.T @ np.linalg.inv(S)
        
        # 更新状态与协方差
        self.state += K @ e
        self.cov = (np.eye(5) - K @ H) @ self.cov
        
        # 强制尺寸约束
        self.state[4] = np.clip(self.state[4], 10, 70)

# ======================
# 生成观察者轨迹与真实数据
# ======================
# 生成观察者轨迹（已知加速度）
obs_pos = np.zeros((steps, 2))
obs_vel = np.zeros((steps, 2))
obs_pos[0] = obs_pos_init
obs_vel[0] = obs_vel_init
for i in range(1, steps):
    obs_vel[i] = obs_vel[i-1] + obs_acc * dt
    obs_pos[i] = obs_pos[i-1] + obs_vel[i-1] * dt + 0.5 * obs_acc * dt**2

# 生成物体真实轨迹（等速）
obj_pos = np.zeros((steps, 2))
obj_pos[0] = true_obj_pos
for i in range(1, steps):
    obj_pos[i] = obj_pos[i-1] + true_obj_vel * dt

# 生成含噪声的观测数据
observations = []
for i in range(steps):
    dx = obj_pos[i, 0] - obs_pos[i, 0]
    dy = obj_pos[i, 1] - obs_pos[i, 1]
    d = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx) + np.random.normal(0, np.sqrt(R[0,0]))
    theta = (true_s / d) + np.random.normal(0, np.sqrt(R[1,1]))
    observations.append([phi, theta])

# ======================
# 初始化滤波器
# ======================
init_state = np.array([init_obj_pos[0], init_obj_pos[1], 
                      init_obj_vel[0], init_obj_vel[1], init_s])
init_cov = np.diag([20.0, 20.0, 2.0, 2.0, 10.0])
iekf = InvariantEKF(init_state, init_cov, Q, R)

# ======================
# 运行滤波器
# ======================
estimated_pos = []
estimated_vel = []
estimated_s = []
for i in range(steps):
    iekf.predict()
    iekf.update(observations[i], obs_pos[i])
    
    estimated_pos.append(iekf.state[:2].copy())
    estimated_vel.append(iekf.state[2:4].copy())
    estimated_s.append(iekf.state[4])

estimated_pos = np.array(estimated_pos)
estimated_vel = np.array(estimated_vel)
estimated_s = np.array(estimated_s)

# ======================
# 可视化结果
# ======================
plt.figure(figsize=(15, 10))

# 轨迹对比
plt.subplot(2, 2, 1)
plt.plot(obj_pos[:,0], obj_pos[:,1], 'b-', label='True Object Path')
plt.plot(estimated_pos[:,0], estimated_pos[:,1], 'r--', label='Estimated Path')
plt.plot(obs_pos[:,0], obs_pos[:,1], 'g-.', label='Observer Path')
plt.scatter(true_obj_pos[0], true_obj_pos[1], c='blue', marker='*', s=200, label='Object Start')
plt.scatter(obs_pos[0,0], obs_pos[0,1], c='green', marker='o', s=100, label='Observer Start')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison')
plt.legend()
plt.grid(True)

# 位置误差分析
pos_error = np.linalg.norm(estimated_pos - obj_pos, axis=1)
plt.subplot(2, 2, 2)
plt.plot(np.arange(steps)*dt, pos_error, 'm-')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Estimation Error')
plt.grid(True)

# 速度估计分析
plt.subplot(2, 2, 3)
plt.plot(np.arange(steps)*dt, estimated_vel[:,0], 'r--', label='Estimated Vx')
plt.plot(np.arange(steps)*dt, estimated_vel[:,1], 'b--', label='Estimated Vy')
plt.plot(np.arange(steps)*dt, [true_obj_vel[0]]*steps, 'r-', label='True Vx')
plt.plot(np.arange(steps)*dt, [true_obj_vel[1]]*steps, 'b-', label='True Vy')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Estimation')
plt.legend()
plt.grid(True)

# 尺寸估计分析
plt.subplot(2, 2, 4)
plt.plot(np.arange(steps)*dt, estimated_s, 'g-', label='Estimated Size')
plt.axhline(y=true_s, color='purple', linestyle='--', label='True Size')
plt.axhline(y=10, color='gray', linestyle=':', label='Size Bounds')
plt.axhline(y=70, color='gray', linestyle=':')
plt.xlabel('Time (s)')
plt.ylabel('Size (m)')
plt.title('Object Size Estimation')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()