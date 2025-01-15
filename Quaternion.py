import numpy as np
from scipy.spatial.transform import Rotation as R

class ShipStatus:
    def __init__(self, name, speed, acceleration, heading, rate_of_turn, position, size=1.0, speed_limit=[0.5, 10.0]):
        self.name = name
        self.speed = speed
        self.acceleration = acceleration
        self.heading = R.from_euler('z', heading, degrees=False)  # 初始方向儲存為四元數
        self.rate_of_turn = rate_of_turn  # 航向角變化率 (rad/s)
        self.position = np.array(position, dtype=float)
        self.size = size
        self.speed_limit = speed_limit

    def update(self, delta_time=0.01):
        """
        使用四元數更新方向，並根據航向角更新位置。
        """
        # 計算旋轉增量的四元數
        delta_heading = R.from_euler('z', self.rate_of_turn * delta_time, degrees=False)
        self.heading = self.heading * delta_heading  # 更新方向（四元數乘法）

        # 獲取當前的航向角（只需要 z 軸分量）
        heading_angle = self.heading.as_euler('xyz', degrees=False)[2]

        # 根據當前方向更新位置
        direction = np.array([
            np.cos(heading_angle),
            np.sin(heading_angle),
            0
        ])
        self.position += self.speed * delta_time * direction

        # 更新速度
        self.speed += self.acceleration
        self.speed = np.clip(self.speed, self.speed_limit[0], self.speed_limit[1])

    def get_status(self):
        """
        返回物體的當前狀態。
        """
        heading_angle = self.heading.as_euler('xyz', degrees=False)[2]  # 獲取當前 z 軸方向
        return {
            "name": self.name,
            "speed": self.speed,
            "heading": heading_angle,
            "current_position": self.position
        }

# 測試 ShipStatus 類別
ship = ShipStatus(
    name="TestShip",
    speed=5.0,
    acceleration=0.1,
    heading=np.radians(30),  # 初始航向角
    rate_of_turn=np.radians(45),  # 每秒 45 度轉向
    position=[0, 0, 0]
)

# 模擬運動
for i in range(100):
    ship.update(delta_time=0.1)
    status = ship.get_status()
    print(f"Step {i + 1}: {status}")
