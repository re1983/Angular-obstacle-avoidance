import numpy as np
import matplotlib.pyplot as plt

# ======================
# 參數設定
# ======================
dt = 0.1                # 時間步長（秒）
total_time = 30          # 總模擬時間（秒）
steps = int(total_time / dt)

# 物體真實參數
true_s = 15.0            # 真實尺寸 (10-70m)
true_obj_pos = np.array([10.0, 10.0])  # 初始位置 (x, y)
true_obj_vel = np.array([-1.0, 1.0])    # 速度 (vx, vy)

# 觀察者運動參數（等速轉向）
obs_speed = 1.50                        # 固定速度 (m/s)
obs_heading_init = np.deg2rad(0)       # 初始航向角（弧度）
obs_rate_of_turn = np.deg2rad(3)       # 轉向速率（弧度/秒）
obs_pos_init = np.array([0.0, 0.0])     # 初始位置

# 初始估計值（帶噪聲）
init_obj_pos = true_obj_pos + np.random.normal(0, 20, 2)
init_obj_vel = true_obj_vel + np.random.normal(0, 1, 2)
init_s = np.clip(true_s + np.random.normal(0, 10), 10, 70)

# 過程噪聲協方差
Q = np.diag([0.5, 0.5, 0.1, 0.1, 0.5])  # [x, y, vx, vy, s]

# 觀測噪聲協方差
R = np.diag([0.0, 0.0])              # 方位角 (rad), 角直徑 (rad)

# ======================
# IEKF 類實現（保持不變）
# ======================
class InvariantEKF:
    def __init__(self, init_state, init_cov, Q, R):
        self.state = init_state.copy()  # [x, y, vx, vy, s]
        self.cov = init_cov.copy()
        self.Q = Q
        self.R = R

    def predict(self):
        F = np.eye(5)
        F[0, 2] = dt
        F[1, 3] = dt
        self.state = F @ self.state
        self.cov = F @ self.cov @ F.T + self.Q
        self.state[4] = np.clip(self.state[4], 10, 20)

    def update(self, z, obs_pos):
        x, y, vx, vy, s = self.state
        dx = x - obs_pos[0]
        dy = y - obs_pos[1]
        d = np.sqrt(dx**2 + dy**2)
        
        phi_pred = np.arctan2(dy, dx)
        theta_pred = s / d
        
        e = np.array([z[0] - phi_pred, z[1] - theta_pred])
        
        H = np.zeros((2, 5))
        H[0, 0] = -dy / (dx**2 + dy**2)
        H[0, 1] = dx / (dx**2 + dy**2)
        H[1, 0] = (-s * dx) / (d**3)
        H[1, 1] = (-s * dy) / (d**3)
        H[1, 4] = 1.0 / d
        
        S = H @ self.cov @ H.T + self.R
        K = self.cov @ H.T @ np.linalg.inv(S)
        
        self.state += K @ e
        self.cov = (np.eye(5) - K @ H) @ self.cov
        self.state[4] = np.clip(self.state[4], 10, 70)

# ======================
# 生成觀察者軌跡（等速轉向）
# ======================
obs_pos = np.zeros((steps, 2))
obs_heading = np.zeros(steps)
obs_pos[0] = obs_pos_init
obs_heading[0] = obs_heading_init

for i in range(1, steps):
    obs_heading[i] = obs_heading[i-1] + obs_rate_of_turn * dt
    obs_pos[i, 0] = obs_pos[i-1, 0] + obs_speed * np.cos(obs_heading[i]) * dt
    obs_pos[i, 1] = obs_pos[i-1, 1] + obs_speed * np.sin(obs_heading[i]) * dt

# ======================
# 生成物體真實軌跡與觀測數據（保持不變）
# ======================
obj_pos = np.zeros((steps, 2))
obj_pos[0] = true_obj_pos
for i in range(1, steps):
    obj_pos[i] = obj_pos[i-1] + true_obj_vel * dt

observations = []
for i in range(steps):
    dx = obj_pos[i, 0] - obs_pos[i, 0]
    dy = obj_pos[i, 1] - obs_pos[i, 1]
    d = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx) + np.random.normal(0, np.sqrt(R[0,0]))
    theta = (true_s / d) + np.random.normal(0, np.sqrt(R[1,1]))
    observations.append([phi, theta])

# ======================
# 初始化濾波器與運行（保持不變）
# ======================
init_state = np.array([init_obj_pos[0], init_obj_pos[1], 
                      init_obj_vel[0], init_obj_vel[1], init_s])
init_cov = np.diag([20.0, 20.0, 2.0, 2.0, 10.0])
iekf = InvariantEKF(init_state, init_cov, Q, R)

estimated_pos = []
estimated_vel = []
estimated_s = []
for i in range(steps):
    iekf.predict()
    iekf.update(observations[i], obs_pos[i])
    
    estimated_pos.append(iekf.state[:2].copy())
    estimated_vel.append(iekf.state[2:4].copy())
    estimated_s.append(iekf.state[4])

# Convert lists to NumPy arrays for easier slicing
estimated_pos = np.array(estimated_pos)
estimated_vel = np.array(estimated_vel)
estimated_s = np.array(estimated_s)

# ======================
# 可視化（新增航向角）
# ======================
plt.figure(figsize=(15, 10))

# 軌跡對比
plt.subplot(2, 2, 1)
plt.plot(obj_pos[:,0], obj_pos[:,1], 'b-', label='True Object Path')
plt.plot(estimated_pos[:,0], estimated_pos[:,1], 'r--', label='Estimated Path')
plt.plot(obs_pos[:,0], obs_pos[:,1], 'g-.', label='Observer Path')
plt.quiver(obs_pos[::10, 0], obs_pos[::10, 1], 
           np.cos(obs_heading[::10]), np.sin(obs_heading[::10]),
           color='green', scale=20, label='Observer Heading')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Comparison with Observer Heading')
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