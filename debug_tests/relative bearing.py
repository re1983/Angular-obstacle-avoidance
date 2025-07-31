import numpy as np
import matplotlib.pyplot as plt

# 初始參數
dt = 0.5  # 時間間隔（秒）
T = 30  # 總模擬時間（秒）
steps = int(T / dt)

# 船速（單位距離/秒）
own_speed = 1.0
target_speed = 1.0

# 初始位置
own_pos = np.array([0.0, 0.0])
target_pos = np.array([0.0, 100.0])  # 目標船從北邊 100 單位遠的地方來

# 你原本朝北，但逐漸右轉（航向角從 0° 變到 30°）
headings_deg = np.linspace(0, 30, steps)  # 每步逐漸變航向
headings_rad = np.radians(headings_deg)

# 目標船始終朝南（180°）
target_heading_rad = np.radians(180)

# 記錄
own_positions = []
target_positions = []
relative_bearings_deg = []

for i in range(steps):
    # 更新位置
    own_dir = np.array([np.sin(headings_rad[i]), np.cos(headings_rad[i])])
    target_dir = np.array([np.sin(target_heading_rad), np.cos(target_heading_rad)])
    own_pos += own_dir * own_speed * dt
    target_pos += target_dir * target_speed * dt

    # 相對位置向量
    rel_vec = target_pos - own_pos
    rel_angle = np.arctan2(rel_vec[0], rel_vec[1])  # 相對於北方的方向
    rel_bearing = np.degrees(rel_angle - headings_rad[i])  # 減去自己當下的航向，得相對方位角
    rel_bearing = (rel_bearing + 360) % 360  # 轉成 0~360°
    
    # 儲存
    own_positions.append(own_pos.copy())
    target_positions.append(target_pos.copy())
    relative_bearings_deg.append(rel_bearing)

# 畫圖
own_positions = np.array(own_positions)
target_positions = np.array(target_positions)

plt.figure(figsize=(12, 5))

# 子圖 1: 位置路徑圖
plt.subplot(1, 2, 1)
plt.plot(own_positions[:, 0], own_positions[:, 1], label="Own ship")
plt.plot(target_positions[:, 0], target_positions[:, 1], label="Target ship")
plt.title("Ship Trajectories")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.axis('equal')

# 子圖 2: 相對方位角變化圖
plt.subplot(1, 2, 2)
plt.plot(np.arange(steps) * dt, relative_bearings_deg)
plt.title("Relative Bearing Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Relative Bearing (°)")
plt.grid(True)

plt.tight_layout()
plt.show()
