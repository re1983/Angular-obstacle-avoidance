import numpy as np

class ShipStatusLieGroup:
    def __init__(self, name, speed, acceleration, heading, rate_of_turn, position, size=1.0, speed_limit=[0.5, 10.0]):
        self.name = name
        self.speed = speed
        self.acceleration = acceleration
        self.R = np.array([[np.cos(heading), -np.sin(heading)],
                           [np.sin(heading),  np.cos(heading)]])  # 初始旋轉矩陣
        self.rate_of_turn = rate_of_turn  # 航向角速度 (rad/s)
        self.position = np.array(position, dtype=float)
        self.size = size
        self.speed_limit = speed_limit

    def exp_so2(self, omega, delta_time):
        """
        將李代數轉換為 SO(2) 群上的旋轉矩陣。
        """
        theta = omega * delta_time
        return np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

    def update(self, delta_time=0.01):
        """
        使用李群更新航向角，並根據航向更新位置。
        """
        # 更新旋轉矩陣
        self.R = self.R @ self.exp_so2(self.rate_of_turn, delta_time)

        # 計算航向方向
        direction = self.R @ np.array([1, 0])  # 航向的單位向量

        # 更新位置
        self.position += self.speed * delta_time * direction

        # 更新速度
        self.speed += self.acceleration
        self.speed = np.clip(self.speed, self.speed_limit[0], self.speed_limit[1])

    def get_status(self):
        """
        返回物體的當前狀態。
        """
        # 獲取當前的航向角（從旋轉矩陣計算）
        heading_angle = np.arctan2(self.R[1, 0], self.R[0, 0])
        return {
            "name": self.name,
            "speed": self.speed,
            "heading": heading_angle,
            "current_position": self.position
        }

# 測試 ShipStatusLieGroup 類別
ship = ShipStatusLieGroup(
    name="TestShip",
    speed=5.0,
    acceleration=0.1,
    heading=np.radians(30),  # 初始航向角
    rate_of_turn=np.radians(45),  # 每秒 45 度轉向
    position=[0, 0]
)

# 模擬運動
for i in range(100):
    ship.update(delta_time=0.1)
    status = ship.get_status()
    print(f"Step {i + 1}: {status}")
