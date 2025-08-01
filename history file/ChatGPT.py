import numpy as np
import matplotlib.pyplot as plt

def so2_rotation(theta):
    """將角度 theta (以弧度表示) 轉換為 SO(2) 旋轉矩陣"""
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])

# 生成一組角度（跨越2pi附近，2pi約6.283）
angles = np.linspace(6.0, 6.5, 10)

# 將固定向量 [1, 0] 用每個旋轉矩陣旋轉得到新向量
vector = np.array([1, 0])
rotated_vectors = np.array([so2_rotation(theta) @ vector for theta in angles])

# 為每個箭頭定義起始位置，這裡所有箭頭都從原點開始
x_positions = np.zeros(len(rotated_vectors))
y_positions = np.zeros(len(rotated_vectors))

# 使用 quiver 繪製箭頭，X, Y, U, V 的維度必須一致
plt.figure(figsize=(6,6))
plt.quiver(x_positions, y_positions, rotated_vectors[:,0], rotated_vectors[:,1],
           angles, scale=5, cmap='viridis', width=0.005)
plt.plot(rotated_vectors[:,0], rotated_vectors[:,1], 'o-', label='Rotated Vectors')
plt.xlabel('X')
plt.ylabel('Y')
plt.title(f'使用 SO(2) 旋轉矩陣旋轉 [1, 0] 向量')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
