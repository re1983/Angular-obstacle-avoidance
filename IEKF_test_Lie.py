import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm, logm

# ======================
# 參數設定
# ======================
dt = 0.1
total_time = 30
steps = int(total_time / dt)

# 物體真實參數（全局座標系）
true_s = 40.0
true_obj_global = np.array([80.0, 60.0])
true_obj_vel_global = np.array([-1.0, 0.5])

# 觀察者運動參數
obs_speed = 5.0
obs_heading_rate = np.deg2rad(10)  # 航向變化率
obs_pos_global = np.zeros((steps, 2))
obs_heading = np.zeros(steps)

# ======================
# IEKF 類實現
# ======================
class InvariantEKF:
    def __init__(self, init_state, init_cov, Q, R):
        self.state = init_state.copy()  # [rho, phi, phi_dot, theta, s]
        self.cov = init_cov
        self.Q = Q
        self.R = R

    def predict(self, delta_psi_obs):
        F = np.eye(5)
        F[1, 2] = dt
        self.state = F @ self.state
        
        # 補償觀察者航向變化
        self.state[1] -= delta_psi_obs
        
        self.cov = F @ self.cov @ F.T + self.Q
        self._apply_constraints()

    def update(self, z):
        rho, phi, phi_dot, theta, s = self.state
        H = np.array([
            [0, 1, 0, 0, 0],
            [s, 0, 0, rho, 0]
        ])
        z_pred = np.array([phi, s*rho])
        e = z - z_pred
        
        S = H @ self.cov @ H.T + self.R
        K = self.cov @ H.T @ np.linalg.inv(S)
        
        self.state += K @ e
        self.cov = (np.eye(5) - K @ H) @ self.cov
        self._apply_constraints()

    def _apply_constraints(self):
        # 尺寸約束
        self.state[4] = np.clip(self.state[4], 10, 70)
        # 物理合理性約束
        self.state[0] = max(1e-3, self.state[0])  # 避免負深度

    def get_global_pos(self, obs_pos, obs_heading):
        # 轉換到全局座標系
        rho, phi, *_ = self.state
        local_x = np.cos(phi) / rho
        local_y = np.sin(phi) / rho
        
        R = np.array([
            [np.cos(obs_heading), -np.sin(obs_heading)],
            [np.sin(obs_heading), np.cos(obs_heading)]
        ])
        global_pos = R @ np.array([local_x, local_y]) + obs_pos
        return global_pos

# ======================
# 生成數據
# ======================
# 初始化
obs_pos_global[0] = [0, 0]
obs_heading[0] = np.deg2rad(45)

for i in range(1, steps):
    obs_heading[i] = obs_heading[i-1] + obs_heading_rate * dt
    dx = obs_speed * dt * np.cos(obs_heading[i])
    dy = obs_speed * dt * np.sin(obs_heading[i])
    obs_pos_global[i] = obs_pos_global[i-1] + [dx, dy]

# 生成物體全局軌跡
obj_global = np.zeros((steps, 2))
obj_global[0] = true_obj_global
for i in range(1, steps):
    obj_global[i] = obj_global[i-1] + true_obj_vel_global * dt

# 生成觀測數據
observations = []
for i in range(steps):
    # 轉換到局部座標系
    rel_pos = obj_global[i] - obs_pos_global[i]
    local_x = np.cos(-obs_heading[i]) * rel_pos[0] - np.sin(-obs_heading[i]) * rel_pos[1]
    local_y = np.sin(-obs_heading[i]) * rel_pos[0] + np.cos(-obs_heading[i]) * rel_pos[1]
    
    rho = 1.0 / np.linalg.norm([local_x, local_y])
    phi = np.arctan2(local_y, local_x)
    theta = true_s * rho
    
    # 添加噪聲
    # phi_noisy = phi + np.random.normal(0, np.sqrt(R[0,0]))
    phi_noisy = phi
    theta_noisy = theta + np.random.normal(0, np.sqrt(R[1,1]))
    observations.append([phi_noisy, theta_noisy])

# ======================
# 初始化與運行濾波器
# ======================
init_rho = 1.0 / np.linalg.norm(obj_global[0] - obs_pos_global[0])
init_phi = np.arctan2(obj_global[0,1]-obs_pos_global[0,1], 
                      obj_global[0,0]-obs_pos_global[0,0]) - obs_heading[0]
init_state = np.array([init_rho, init_phi, 0, true_s*init_rho, true_s])
init_cov = np.diag([0.1, 0.1, 0.01, 0.05, 10.0])
Q = np.diag([1e-4, 1e-3, 1e-4, 1e-4, 0.1])
R = np.diag([0.005, 0.002])

iekf = InvariantEKF(init_state, init_cov, Q, R)

estimated_global = []
for i in range(steps):
    if i > 0:
        delta_psi = obs_heading[i] - obs_heading[i-1]
        iekf.predict(delta_psi)
    iekf.update(observations[i])
    global_pos = iekf.get_global_pos(obs_pos_global[i], obs_heading[i])
    estimated_global.append(global_pos)

estimated_global = np.array(estimated_global)

# ======================
# 可視化
# ======================
plt.figure(figsize=(12, 6))
plt.plot(obj_global[:,0], obj_global[:,1], 'b-', label='True Object Path')
plt.plot(estimated_global[:,0], estimated_global[:,1], 'r--', label='Estimated Path')
plt.plot(obs_pos_global[:,0], obs_pos_global[:,1], 'g-.', label='Observer Path')
plt.quiver(obs_pos_global[::10,0], obs_pos_global[::10,1], 
           np.cos(obs_heading[::10]), np.sin(obs_heading[::10]),
           color='green', scale=20, label='Observer Heading')
plt.xlabel('Global X (m)')
plt.ylabel('Global Y (m)')
plt.legend()
plt.grid(True)
plt.title('Global Coordinate Estimation with IEKF')
plt.show()