import numpy as np

class ObjectIn3DSpace:
    def __init__(self, name, position, velocity, angular_velocity, rotation_matrix=None):
        """
        初始化物體的狀態。

        Args:
            name: str，物體名稱
            position: ndarray，物體初始位置 [x, y, z]
            velocity: ndarray，物體初始速度 [vx, vy, vz]
            angular_velocity: ndarray，物體的角速度 [ωx, ωy, ωz]（rad/s）
            rotation_matrix: ndarray，可選，初始旋轉矩陣 (3x3)
        """
        self.name = name
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.angular_velocity = np.array(angular_velocity, dtype=float)
        self.rotation_matrix = (
            np.eye(3) if rotation_matrix is None else np.array(rotation_matrix, dtype=float)
        )

    def skew_symmetric(self, omega):
        """
        計算角速度的反對稱矩陣。

        Args:
            omega: ndarray，角速度向量 [ωx, ωy, ωz]

        Returns:
            ndarray，反對稱矩陣 (3x3)
        """
        return np.array([
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ])

    def exp_so3(self, omega, delta_time):
        """
        使用羅德里格公式計算旋轉矩陣。

        Args:
            omega: ndarray，角速度向量 [ωx, ωy, ωz]
            delta_time: float，時間步長

        Returns:
            ndarray，旋轉矩陣 (3x3)
        """
        theta = np.linalg.norm(omega) * delta_time
        if theta < 1e-8:
            return np.eye(3)  # 當角速度接近零時，返回單位矩陣

        omega_unit = omega / np.linalg.norm(omega)
        omega_hat = self.skew_symmetric(omega_unit)
        return (
            np.eye(3)
            + np.sin(theta) * omega_hat
            + (1 - np.cos(theta)) * omega_hat @ omega_hat
        )

    def update(self, delta_time):
        """
        更新物體的狀態。

        Args:
            delta_time: float，時間步長
        """
        # 更新旋轉矩陣
        self.rotation_matrix = self.rotation_matrix @ self.exp_so3(self.angular_velocity, delta_time)

        # 更新位置
        self.position += self.velocity * delta_time

    def get_status(self):
        """
        獲取物體當前的狀態。

        Returns:
            dict，包含名稱、位置、速度、旋轉矩陣等信息
        """
        return {
            "name": self.name,
            "position": self.position,
            "velocity": self.velocity,
            "rotation_matrix": self.rotation_matrix
        }

# 測試 3D 運動
obj = ObjectIn3DSpace(
    name="3DObject",
    position=[0, 0, 0],
    velocity=[1, 0, 0],
    angular_velocity=[0, 0, np.radians(45)]  # 每秒繞 Z 軸旋轉 45 度
)

# 模擬運動
num_steps = 100
delta_time = 0.1  # 每步時間 (秒)

for step in range(num_steps):
    obj.update(delta_time)
    status = obj.get_status()
    print(f"Step {step + 1}: Position = {status['position']}, Rotation Matrix =\n{status['rotation_matrix']}")
