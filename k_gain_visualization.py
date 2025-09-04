import matplotlib.pyplot as plt
import numpy as np
from k_gain import k_gain, time, distance

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 创建一个包含多个子图的figure
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))

# 1. 直方图 - Time vs K值
ax1.bar(k_gain, time, alpha=0.7, color='skyblue', edgecolor='navy', width=2)
ax1.set_xlabel('K Gain', fontsize=12)
ax1.set_ylabel('Time', fontsize=12)
ax1.set_title('Time vs K Gain (Bar Chart)', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
# 在每个柱子上显示数值
for i, (k, t) in enumerate(zip(k_gain, time)):
    ax1.text(k, t + 0.5, f'{t:.2f}', ha='center', va='bottom', fontweight='bold')

# 2. 直方图 - Distance vs K值
ax2.bar(k_gain, distance, alpha=0.7, color='lightcoral', edgecolor='darkred', width=2)
ax2.set_xlabel('K Gain', fontsize=12)
ax2.set_ylabel('Distance', fontsize=12)
ax2.set_title('Distance vs K Gain (Bar Chart)', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
# 在每个柱子上显示数值
for i, (k, d) in enumerate(zip(k_gain, distance)):
    ax2.text(k, d + 1, f'{d:.3f}', ha='center', va='bottom', fontweight='bold')

# 3. 线性图 - 两个指标的比较
ax3.plot(k_gain, time, 'o-', label='Time', linewidth=2, markersize=8, color='blue')
ax3_twin = ax3.twinx()
ax3_twin.plot(k_gain, distance, 's-', label='Distance', linewidth=2, markersize=8, color='red')

ax3.set_xlabel('K Gain', fontsize=12)
ax3.set_ylabel('Time', color='blue', fontsize=12)
ax3_twin.set_ylabel('Distance', color='red', fontsize=12)
ax3.set_title('Time & Distance vs K Gain (Line Plot)', fontsize=14, fontweight='bold')
ax3.grid(True, alpha=0.3)
ax3.legend(loc='upper left')
ax3_twin.legend(loc='upper right')

# 4. 散点图 - Time vs Distance，用K值作为颜色编码
scatter = ax4.scatter(time, distance, c=k_gain, s=200, alpha=0.7, cmap='viridis', edgecolors='black')
ax4.set_xlabel('Time', fontsize=12)
ax4.set_ylabel('Distance', fontsize=12)
ax4.set_title('Distance vs Time (K Gain as Color)', fontsize=14, fontweight='bold')
ax4.grid(True, alpha=0.3)

# 添加颜色条
cbar = plt.colorbar(scatter, ax=ax4)
cbar.set_label('K Gain', fontsize=12)

# 在散点上标注K值
for i, (t, d, k) in enumerate(zip(time, distance, k_gain)):
    ax4.annotate(f'K={k}', (t, d), xytext=(5, 5), textcoords='offset points', 
                fontsize=10, fontweight='bold')

# 调整布局
plt.tight_layout()
plt.suptitle('K Gain Analysis Visualization', fontsize=16, fontweight='bold', y=0.98)

# 显示图表
plt.show()

# 打印数据摘要
print("Data Summary:")
print("=" * 40)
print(f"K Gain values: {k_gain}")
print(f"Time values: {time}")
print(f"Distance values: {distance}")
print("\nCorrelation Analysis:")
print(f"K-Time correlation: {np.corrcoef(k_gain, time)[0,1]:.4f}")
print(f"K-Distance correlation: {np.corrcoef(k_gain, distance)[0,1]:.4f}")
print(f"Time-Distance correlation: {np.corrcoef(time, distance)[0,1]:.4f}")
