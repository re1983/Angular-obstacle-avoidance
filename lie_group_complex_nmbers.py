import math
import cmath

def so2_rotation(angle):
    return cmath.exp(1j * angle)

def compose_so2_rotations(angle1, angle2):
    z = so2_rotation(angle1) * so2_rotation(angle2)
    # print(f"z: {z}")
    result = math.atan2(z.imag, z.real)
    # print(f"result: {result}")
    return result
    # return math.atan2(z.imag, z.real)

def angle_difference(angle1, angle2):
    """
    計算兩個角度的差，範圍為 [-π, π]。
    Args:
        angle1: float, 第一個角度（弧度）
        angle2: float, 第二個角度（弧度）

    Returns:
        angle_diff: float, 角度差（弧度），範圍 [-π, π]
    """
    # 計算差值並映射到 [-π, π]
    angle_diff = (angle2 - angle1) % (2 * math.pi)
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    return angle_diff

# 測試
angle1 = math.radians(20)
angle2 = math.radians(20)

result1 = compose_so2_rotations(angle1, angle2)
print(f"Composition of {math.degrees(angle1)} and {math.degrees(angle2)}: {math.degrees(result1)} degrees")

# angle3 = math.radians(350)
# angle4 = math.radians(30)
# result2 = compose_so2_rotations(angle3, angle4)
# print(f"Composition of {math.degrees(angle3)} and {math.degrees(angle4)}: {math.degrees(result2)} degrees")
# # print(f"Composition of previous result and {math.degrees(angle3)}: {math.degrees(result2)} degrees")

# 測試
angle1 = math.radians(179)
angle2 = math.radians(-179)

angle_diff = angle_difference(angle1, angle2)
print(f"角度差 (弧度): {angle_diff}")
print(f"角度差 (度): {math.degrees(angle_diff)}")