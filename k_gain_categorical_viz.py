import matplotlib.pyplot as plt
import numpy as np
from k_gain import k_gain, mean_arrival_time, mean_minimum_distance

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 创建figure和子图 - 1行2列布局
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

# 将k_gain转换为字符串类别
k_categories = [f'K={k}' for k in k_gain]

# 1. 左侧图 - Mean Arrival Time
bars1 = ax1.bar(k_categories, mean_arrival_time, alpha=0.8, color='skyblue', 
                edgecolor='navy', width=0.6)
ax1.set_xlabel('K Gain Categories', fontsize=12, fontweight='bold')
ax1.set_ylabel('Mean Arrival Time (s)', fontsize=12, fontweight='bold')
ax1.set_title('Mean Arrival Time by K Gain', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3, axis='y')

# 在柱子上显示数值
for bar, time_val in zip(bars1, mean_arrival_time):
    height = bar.get_height()
    ax1.text(bar.get_x() + bar.get_width()/2., height + 0.3,
             f'{time_val:.2f}', ha='center', va='bottom', fontweight='bold')

# 设置y轴范围为100到120之间
ax1.set_ylim(100, 115)

# 2. 右侧图 - Mean Minimum Distance  
bars2 = ax2.bar(k_categories, mean_minimum_distance, alpha=0.8, color='lightcoral', 
                edgecolor='darkred', width=0.6)
ax2.set_xlabel('K Gain Categories', fontsize=12, fontweight='bold')
ax2.set_ylabel('Mean Minimum Distance (m)', fontsize=12, fontweight='bold')
ax2.set_title('Mean Minimum Distance by K Gain', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3, axis='y')

# 在柱子上显示数值
for bar, dist_val in zip(bars2, mean_minimum_distance):
    height = bar.get_height()
    ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
             f'{dist_val:.3f}', ha='center', va='bottom', fontweight='bold')

# 设置y轴范围，留出空间显示数值
ax2.set_ylim(0, max(mean_minimum_distance) * 1.15)

# 添加主标题
# plt.suptitle('K Gain Analysis and Comparison', fontsize=16, fontweight='bold')

# 调整布局，为主标题留出空间
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# 保存图表为IEEE论文推荐格式
# SVG: 矢量格式，最佳质量，推荐用于IEEE论文
plt.savefig('k_gain_analysis.svg', format='svg', dpi=300, bbox_inches='tight')

# EPS: 矢量格式，IEEE传统接受格式
plt.savefig('k_gain_analysis.eps', format='eps', dpi=300, bbox_inches='tight')

# PDF: 矢量格式，现代推荐格式
plt.savefig('k_gain_analysis.pdf', format='pdf', dpi=300, bbox_inches='tight')

# PNG: 高分辨率光栅格式，备用选择
plt.savefig('k_gain_analysis.png', format='png', dpi=600, bbox_inches='tight')

print("图表已保存为以下格式:")
print("- k_gain_analysis.svg (SVG - 矢量格式，IEEE论文推荐)")
print("- k_gain_analysis.eps (EPS - 矢量格式，IEEE传统格式)")
print("- k_gain_analysis.pdf (PDF - 矢量格式，现代推荐)")
print("- k_gain_analysis.png (PNG - 高分辨率光栅格式)")

# 显示图表
plt.show()

# 打印数据摘要
print("Categorical Data Summary:")
print("=" * 50)
for k, time_val, dist_val in zip(k_gain, mean_arrival_time, mean_minimum_distance):
    print(f"K={k:2d}: Time={time_val:7.2f}, Distance={dist_val:7.3f}")

print("\nStatistical Analysis:")
print("=" * 50)
print(f"Time - Min: {min(mean_arrival_time):.2f}, Max: {max(mean_arrival_time):.2f}, Range: {max(mean_arrival_time)-min(mean_arrival_time):.2f}")
print(f"Distance - Min: {min(mean_minimum_distance):.3f}, Max: {max(mean_minimum_distance):.3f}, Range: {max(mean_minimum_distance)-min(mean_minimum_distance):.3f}")

print("\nCorrelation Analysis:")
print("=" * 50)
print(f"K-Time correlation: {np.corrcoef(k_gain, mean_arrival_time)[0,1]:.4f}")
print(f"K-Distance correlation: {np.corrcoef(k_gain, mean_minimum_distance)[0,1]:.4f}")
print(f"Time-Distance correlation: {np.corrcoef(mean_arrival_time, mean_minimum_distance)[0,1]:.4f}")
