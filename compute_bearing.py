import numpy as np

def compute_bearing_angle(A_heading, B_pos, A_pos):
    """
    計算物體 A 的航向與物體 B 之間的夾角 (Bearing Angle)。

    Args:
        A_heading: ndarray，物體 A 的 Heading 向量 [hx, hy]
        B_pos: ndarray，物體 B 的位置 [bx, by]
        A_pos: ndarray，物體 A 的位置 [ax, ay]

    Returns:
        bearing_angle: float，夾角 (弧度)
    """
    # 計算目標方向向量
    d_AB = B_pos - A_pos
    d_AB_unit = d_AB / np.linalg.norm(d_AB)

    # 內積與外積
    cos_bearing = np.dot(A_heading, d_AB_unit)
    sin_bearing = np.cross(A_heading, d_AB_unit)

    # 使用 arctan2 計算夾角
    bearing_angle = np.arctan2(sin_bearing, cos_bearing)
    return bearing_angle

# 測試數據
A_heading = np.array([1.0, 0.0])  # A 的 Heading 向正北
A_pos = np.array([0.0, 0.0])      # A 的位置
B_pos = np.array([1.0, 1.0])      # B 的位置

bearing_angle = compute_bearing_angle(A_heading, B_pos, A_pos)
print(f"Bearing Angle: {bearing_angle:.2f} radians ({np.degrees(bearing_angle):.2f} degrees)")

import numpy as np

def compute_bearing_angle_3d(A_heading, B_pos, A_pos):
    """
    計算物體 A 的航向與物體 B 的方向之間的夾角 (Bearing Angle) in SO(3)。

    Args:
        A_heading: ndarray，物體 A 的 Heading 向量 [hx, hy, hz]
        B_pos: ndarray，物體 B 的位置 [bx, by, bz]
        A_pos: ndarray，物體 A 的位置 [ax, ay, az]

    Returns:
        bearing_angle: float，夾角 (弧度)
    """
    # 計算物體 B 的方向向量
    d_AB = B_pos - A_pos
    d_AB_unit = d_AB / np.linalg.norm(d_AB)

    # 內積
    cos_bearing = np.dot(A_heading, d_AB_unit)

    # 外積
    cross_product = np.cross(A_heading, d_AB_unit)
    sin_bearing = np.linalg.norm(cross_product)

    # 使用 arctan2 計算 Bearing Angle
    bearing_angle = np.arctan2(sin_bearing, cos_bearing)
    return bearing_angle

# 測試數據
A_heading = np.array([1.0, 0.0, 0.0])  # A 的 Heading 向正北
A_pos = np.array([0.0, 0.0, 0.0])      # A 的位置
B_pos = np.array([1.0, 1.0, 1.0])      # B 的位置

bearing_angle = compute_bearing_angle_3d(A_heading, B_pos, A_pos)
print(f"Bearing Angle: {bearing_angle:.2f} radians ({np.degrees(bearing_angle):.2f} degrees)")
