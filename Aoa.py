import numpy as np
import matplotlib.pyplot as plt

def compute_azimuth_angle_and_derivative(A_pos, B_pos, A_vel, B_vel):
    """
    計算 B 對 A 的方位角及其導數。

    Args:
        A_pos: ndarray, A 的位置 [x, y]
        B_pos: ndarray, B 的位置 [x, y]
        A_vel: ndarray, A 的速度 [vx, vy]
        B_vel: ndarray, B 的速度 [vx, vy]

    Returns:
        theta: float, 方位角（弧度）
        theta_dot: float, 方位角導數
    """
    rel_pos = B_pos - A_pos
    rel_vel = B_vel - A_vel

    dx, dy = rel_pos
    vx, vy = rel_vel

    theta = np.arctan2(dy, dx)
    theta_dot = (dx * vy - dy * vx) / (dx**2 + dy**2)

    return theta, theta_dot

def adjust_B_velocity(A_pos, B_pos, A_vel, B_speed):
    """
    調整 B 的速度方向，使方位角導數最大化。

    Args:
        A_pos: ndarray, A 的位置 [x, y]
        B_pos: ndarray, B 的位置 [x, y]
        A_vel: ndarray, A 的速度 [vx, vy]
        B_speed: float, B 的速度大小

    Returns:
        B_vel: ndarray, B 的新速度 [vx, vy]
    """
    rel_pos = B_pos - A_pos
    rel_pos_unit = rel_pos / np.linalg.norm(rel_pos)

    # A 的運動方向
    A_dir = A_vel / np.linalg.norm(A_vel)

    # 垂直於 A_dir 的方向
    perp_dir = np.array([-A_dir[1], A_dir[0]])

    # B 的速度應沿著垂直方向
    B_vel = B_speed * perp_dir

    return B_vel

# 模擬參數
A_pos = np.array([-5.0, 5.0])  # A 的初始位置
A_vel = np.array([1.0, 0.0])  # A 的速度

B_pos = np.array([0.0, 0.0])  # B 的初始位置
B_speed = 1.0                # B 的速度大小

num_steps = 100              # 模擬步數
dt = 0.1                     # 時間步長

# 存儲模擬結果
A_positions = [A_pos.copy()]
B_positions = [B_pos.copy()]
azimuth_angles = []
azimuth_derivatives = []

# 模擬運動
for _ in range(num_steps):
    theta, theta_dot = compute_azimuth_angle_and_derivative(A_pos, B_pos, A_vel, np.array([0.0, 0.0]))

    # 調整 B 的速度方向
    B_vel = adjust_B_velocity(A_pos, B_pos, A_vel, B_speed)

    # 更新位置
    A_pos += A_vel * dt
    B_pos += B_vel * dt

    # 存儲數據
    A_positions.append(A_pos.copy())
    B_positions.append(B_pos.copy())
    azimuth_angles.append(theta)
    azimuth_derivatives.append(theta_dot)

# 將結果轉換為 NumPy 陣列
A_positions = np.array(A_positions)
B_positions = np.array(B_positions)
azimuth_angles = np.array(azimuth_angles)
azimuth_derivatives = np.array(azimuth_derivatives)

# # 繪圖
# plt.figure(figsize=(10, 6))

# # 運動軌跡
# plt.subplot(2, 1, 1)
# plt.plot(A_positions[:, 0], A_positions[:, 1], label="Object A")
# plt.plot(B_positions[:, 0], B_positions[:, 1], label="Object B")
# plt.scatter(A_positions[0, 0], A_positions[0, 1], color="blue", label="A Start")
# plt.scatter(B_positions[0, 0], B_positions[0, 1], color="orange", label="B Start")
# plt.xlabel("X Position")
# plt.ylabel("Y Position")
# plt.title("Trajectories of A and B")
# plt.legend()
# plt.axis("equal")

# # 方位角導數
# plt.subplot(2, 1, 2)
# plt.plot(np.arange(num_steps) * dt, azimuth_derivatives, label="Azimuth Angle Derivative")
# plt.xlabel("Time (s)")
# plt.ylabel("Azimuth Angle Derivative (rad/s)")
# plt.title("Azimuth Angle Derivative vs Time")
# plt.legend()

# # 方位角
# plt.figure(figsize=(10, 6))
# plt.plot(np.arange(num_steps) * dt, azimuth_angles, label="Azimuth Angle")
# plt.xlabel("Time (s)")
# plt.ylabel("Azimuth Angle (rad)")
# plt.title("Azimuth Angle vs Time")
# plt.legend()
# 將方位角轉換為度
azimuth_angles_deg = np.degrees(azimuth_angles)

# 繪製三個圖表
plt.figure(figsize=(10, 12))

# 運動軌跡
plt.subplot(3, 1, 1)
plt.plot(A_positions[:, 0], A_positions[:, 1], label="Object A")
plt.plot(B_positions[:, 0], B_positions[:, 1], label="Object B")
plt.scatter(A_positions[0, 0], A_positions[0, 1], color="blue", label="A Start")
plt.scatter(B_positions[0, 0], B_positions[0, 1], color="orange", label="B Start")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Trajectories of A and B")
plt.legend()
plt.axis("equal")

# 方位角導數
plt.subplot(3, 1, 2)
plt.plot(np.arange(num_steps) * dt, azimuth_derivatives, label="Azimuth Angle Derivative")
plt.xlabel("Time (s)")
plt.ylabel("Azimuth Angle Derivative (rad/s)")
plt.title("Azimuth Angle Derivative vs Time")
plt.legend()

# 方位角（度）
plt.subplot(3, 1, 3)
plt.plot(np.arange(num_steps) * dt, azimuth_angles_deg, label="Azimuth Angle (deg)")
plt.xlabel("Time (s)")
plt.ylabel("Azimuth Angle (deg)")
plt.title("Azimuth Angle vs Time")
plt.legend()

plt.tight_layout()
plt.show()
