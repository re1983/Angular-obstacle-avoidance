import numpy as np
import matplotlib.pyplot as plt

# =============================================
# 參數設定
# =============================================
dt = 0.1               # 時間步長（秒）
total_time = 20         # 總模擬時間（秒）
steps = int(total_time / dt)

# 真實參數（Ground Truth）
true_obs = np.array([30, 40])   # 障礙物真實位置 (ox, oy)
true_s = 50.0                   # 障礙物真實尺寸 (s)
true_radius = 20.0              # 無人機圓弧半徑（米）
true_omega = 0.5                # 角速度（rad/s）

# 初始估計值（帶噪聲）
init_drone = np.array([0.0, 0.0]) + np.random.normal(0, 5, 2)
init_obs = true_obs + np.random.normal(0, 10, 2)
init_s = true_s + np.random.normal(0, 10)

# 過程噪聲協方差
Q = np.diag([0.1, 0.1, 0.1, 0.1])  # 無人機位置 (px, py), 障礙物位置 (ox, oy)

# 觀測噪聲協方差
R = np.diag([0.01, 0.005])          # 方位角 (rad), 角直徑 (rad)

# =============================================
# IEKF 類別
# =============================================
class InvariantEKF:
    def __init__(self, init_state, init_cov, Q, R):
        self.state = init_state.copy()  # [px, py, ox, oy]
        self.cov = init_cov.copy()
        self.Q = Q
        self.R = R

    def predict(self, control_input):
        # 控制輸入為無人機的圓周運動參數（此處簡化為已知運動模型）
        # 預測無人機位置（基於圓周運動）
        theta = true_omega * dt
        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]])
        drone_pos = self.state[:2]
        drone_pos = rot_mat @ (drone_pos - np.array([true_radius, 0])) + np.array([true_radius, 0])
        self.state[:2] = drone_pos
        # 障礙物位置保持不變（靜態模型）
        self.cov = self.cov + self.Q  # 簡單的協方差傳播（可改進）

    def update(self, z):
        # 觀測值 z = [phi, theta]
        px, py, ox, oy = self.state
        dx = ox - px
        dy = oy - py
        d = np.sqrt(dx**2 + dy**2)
        
        # 預測觀測
        phi_pred = np.arctan2(dy, dx)
        theta_pred = true_s / d
        
        # 觀測殘差
        e = np.array([z[0] - phi_pred, z[1] - theta_pred])
        
        # 計算雅可比矩陣 H
        H = np.zeros((2, 4))
        H[0, 0] = dy / (dx**2 + dy**2)    # d(phi)/dpx
        H[0, 1] = -dx / (dx**2 + dy**2)   # d(phi)/dpy
        H[0, 2] = -dy / (dx**2 + dy**2)   # d(phi)/dox
        H[0, 3] = dx / (dx**2 + dy**2)    # d(phi)/doy
        
        H[1, 0] = (true_s * dx) / (d**3)  # d(theta)/dpx
        H[1, 1] = (true_s * dy) / (d**3)  # d(theta)/dpy
        H[1, 2] = -H[1, 0]                # d(theta)/dox
        H[1, 3] = -H[1, 1]                # d(theta)/doy
        
        # 卡爾曼增益
        S = H @ self.cov @ H.T + self.R
        K = self.cov @ H.T @ np.linalg.inv(S)
        
        # 更新狀態與協方差
        self.state += K @ e
        self.cov = (np.eye(4) - K @ H) @ self.cov
        
        # 應用障礙物位置約束（可選）
        # self.state[2:] = np.clip(self.state[2:], [0, 0], [100, 100])

# =============================================
# 生成真實軌跡與觀測數據
# =============================================
# 無人機真實圓周運動軌跡
time = np.arange(0, total_time, dt)
drone_traj = np.zeros((steps, 2))
for i in range(steps):
    theta = true_omega * time[i]
    drone_traj[i, 0] = true_radius * np.cos(theta) + true_radius
    drone_traj[i, 1] = true_radius * np.sin(theta)

# 生成含噪聲的觀測數據
observations = []
for i in range(steps):
    dx = true_obs[0] - drone_traj[i, 0]
    dy = true_obs[1] - drone_traj[i, 1]
    phi = np.arctan2(dy, dx) + np.random.normal(0, np.sqrt(R[0,0]))
    theta = (true_s / np.sqrt(dx**2 + dy**2)) + np.random.normal(0, np.sqrt(R[1,1]))
    observations.append([phi, theta])

# =============================================
# 初始化濾波器
# =============================================
init_state = np.concatenate([init_drone, init_obs])
init_cov = np.diag([10.0, 10.0, 20.0, 20.0])
iekf = InvariantEKF(init_state, init_cov, Q, R)

# =============================================
# 運行濾波器
# =============================================
estimated_traj = []
estimated_obs = []
for i in range(steps):
    # 預測步驟（此處簡化為已知運動模型）
    iekf.predict(None)
    
    # 更新步驟
    z = observations[i]
    iekf.update(z)
    
    # 保存估計結果
    estimated_traj.append(iekf.state[:2].copy())
    estimated_obs.append(iekf.state[2:].copy())

estimated_traj = np.array(estimated_traj)
estimated_obs = np.array(estimated_obs)

# =============================================
# 可視化結果
# =============================================
plt.figure(figsize=(12, 6))

# 繪製真實與估計的無人機軌跡
plt.subplot(1, 2, 1)
plt.plot(drone_traj[:, 0], drone_traj[:, 1], 'b-', label='True Drone Path')
plt.plot(estimated_traj[:, 0], estimated_traj[:, 1], 'r--', label='Estimated Drone Path')
plt.scatter(true_obs[0], true_obs[1], c='green', s=100, marker='*', label='True Obstacle')
plt.scatter(estimated_obs[-1, 0], estimated_obs[-1, 1], c='purple', s=100, marker='x', label='Estimated Obstacle')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Estimation')
plt.legend()
plt.grid(True)

# 繪製障礙物位置估計誤差
plt.subplot(1, 2, 2)
obs_error = np.linalg.norm(estimated_obs - true_obs, axis=1)
plt.plot(time, obs_error, 'g-', label='Obstacle Position Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('Estimation Error Convergence')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()