import numpy as np
import matplotlib.pyplot as plt
import math

# 固定的目標機外形參數（先寫死在此檔案）
# 長度（機頭到機尾的總長，m）與翼展（左翼端到右翼端的總寬，m）
# DIAMOND_LENGTH_M = 8.28
# DIAMOND_WINGSPAN_M = 11.00

DIAMOND_LENGTH_M = 39.47
DIAMOND_WINGSPAN_M = 34.32

# Default simulation parameters
DEFAULT_OWNSHIP_CONFIG = {
    "name": "Ownship",
    "velocity": 1,
    "acceleration": 0,
    "heading": 0.0,
    "rate_of_turn": 0,
    "position": [0, 0, 0],
    "size": 0.5,
    "max_rate_of_turn": [22, 22]
}

DEFAULT_SHIP_CONFIG = {
    "name": "Ship A", 
    "velocity": 2.0,
    "acceleration": 0,
    "heading": 180.0,
    "rate_of_turn": 0,
    "position": [75, 0, 0],
    "size": 0.5,
    "max_rate_of_turn": [3, 3]
}

DEFAULT_GOAL_CONFIG = {
    "name": "Goal",
    "velocity": 0.0,
    "acceleration": 0,
    "heading": 0,
    "rate_of_turn": 0,
    "position": [50, 0, 0],
    "size": 10.0,
    "max_rate_of_turn": [0, 0]
}

DEFAULT_SIM_PARAMS = {
    "time_steps": 20000,  # 更大的數字以避免過早timeout
    "delta_time": 0.01,
    "ALPHA_TRIG": 0.5
}

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=1.0, max_rate_of_turn=[20, 20], velocity_limit=[0.5, 10.0]):
        self.name = name
        self.velocity = velocity
        self.acceleration = acceleration
        self.heading = heading
        self.rate_of_turn = rate_of_turn
        self.position = np.array(position, dtype=float)
        self.size = size
        self.max_rate_of_turn = max_rate_of_turn
        self.velocity_limit = velocity_limit

    def update(self, delta_time=0.01):
        self.heading += self.rate_of_turn * delta_time
        # NED coordinate system: North=X+, East=Y+, Down=Z+
        # Heading: 0°=North, 90°=East, 180°=South, 270°=West
        self.position += self.velocity * delta_time * np.array([
            np.cos(np.radians(self.heading)),
            np.sin(np.radians(self.heading)),
            0])
        self.velocity += self.acceleration

    def get_status(self):
        return {
            "name": self.name,
            "velocity": self.velocity,
            "heading": self.heading,
            "current_position": self.position
        }

    def copy(self):
        """Create a deep copy of the ship status"""
        return ShipStatus(
            name=self.name,
            velocity=self.velocity,
            acceleration=self.acceleration,
            heading=self.heading,
            rate_of_turn=self.rate_of_turn,
            position=self.position.copy(),
            size=self.size,
            max_rate_of_turn=self.max_rate_of_turn.copy(),
            velocity_limit=self.velocity_limit.copy()
        )

def calculate_ship_initial_position_with_turning(ownship_config, goal_config, 
                                               ship_velocity, ship_heading, ship_rate_of_turn,
                                               collision_ratio=0.5):
    """
    正確的圓弧運動反推算法：計算Ship A的初始位置
    
    考慮Ship A的圓弧運動軌跡（如果rate_of_turn ≠ 0）
    
    Args:
        ownship_config: Ownship配置字典
        goal_config: Goal配置字典  
        ship_velocity: Ship A的速度 (m/s)
        ship_heading: Ship A的初始航向 (度)
        ship_rate_of_turn: Ship A的轉彎速度 (度/秒)
        collision_ratio: 碰撞發生在Ownship路徑的比例 (0.0-1.0)
    
    Returns:
        tuple: (初始位置list, 碰撞點, 碰撞時間)
    """
    # 獲取Ownship和Goal的位置
    ownship_pos = np.array(ownship_config["position"])
    goal_pos = np.array(goal_config["position"])
    ownship_velocity = ownship_config["velocity"]
    
    # 計算Ownship的直線路徑
    path_vector = goal_pos - ownship_pos
    path_length = np.linalg.norm(path_vector)
    
    # 確定碰撞點
    collision_point = ownship_pos + collision_ratio * path_vector
    
    # 計算碰撞時間 (基於Ownship的等速運動)
    time_to_collision = (collision_ratio * path_length) / ownship_velocity
    
    # 轉換角度為弧度
    theta_0_rad = np.radians(ship_heading)
    omega_rad = np.radians(ship_rate_of_turn)
    
    if abs(ship_rate_of_turn) < 1e-10:
        # 情況1: 直線運動 (rate_of_turn ≈ 0)
        travel_distance = ship_velocity * time_to_collision
        direction = np.array([np.cos(theta_0_rad), np.sin(theta_0_rad), 0])
        initial_position = collision_point - travel_distance * direction
        
    else:
        # 情況2: 圓弧運動 (rate_of_turn ≠ 0)
        # 轉彎半徑
        R = ship_velocity / abs(omega_rad)
        
        # 轉過的角度
        delta_theta = omega_rad * time_to_collision
        
        # 最終航向
        theta_final_rad = theta_0_rad + delta_theta
        
        # 從碰撞點和最終航向反推圓心位置
        if omega_rad > 0:  # 右轉
            # 圓心在最終位置的左側
            center = collision_point + R * np.array([-np.sin(theta_final_rad), np.cos(theta_final_rad), 0])
            # 初始位置在圓心的右側
            initial_position = center + R * np.array([np.sin(theta_0_rad), -np.cos(theta_0_rad), 0])
        else:  # 左轉
            # 圓心在最終位置的右側  
            center = collision_point + R * np.array([np.sin(theta_final_rad), -np.cos(theta_final_rad), 0])
            # 初始位置在圓心的左側
            initial_position = center + R * np.array([-np.sin(theta_0_rad), np.cos(theta_0_rad), 0])
    
    return initial_position.tolist(), collision_point.tolist(), time_to_collision

def create_collision_scenario_with_turning(ship_velocity=2.0, ship_heading=180.0, ship_rate_of_turn=0, 
                                         ship_size=0.5, collision_ratio=0.5):
    """
    正確的碰撞場景生成函數（考慮圓弧運動）
    
    Args:
        ship_velocity: Ship A的速度 (m/s)
        ship_heading: Ship A的初始航向 (度)
        ship_rate_of_turn: Ship A的轉彎速度 (度/秒)
        ship_size: Ship A的大小 (m)
        collision_ratio: 碰撞發生在Ownship路徑的比例 (0.0-1.0)
    
    Returns:
        dict: Ship A的完整配置和碰撞預測資訊
    """
    # 計算正確的初始位置（考慮圓弧運動）
    initial_pos, collision_point, collision_time = calculate_ship_initial_position_with_turning(
        DEFAULT_OWNSHIP_CONFIG, 
        DEFAULT_GOAL_CONFIG,
        ship_velocity, 
        ship_heading,
        ship_rate_of_turn,
        collision_ratio
    )
    
    ship_config = {
        "name": "Ship A",
        "velocity": ship_velocity,
        "acceleration": 0,
        "heading": ship_heading,
        "rate_of_turn": ship_rate_of_turn,
        "position": initial_pos,
        "size": ship_size,
        "max_rate_of_turn": [12, 12]
    }
    
    # 添加碰撞預測資訊
    ship_config["_collision_info"] = {
        "predicted_collision_point": collision_point,
        "predicted_collision_time": collision_time,
        "collision_ratio": collision_ratio,
        "motion_type": "circular" if abs(ship_rate_of_turn) > 1e-10 else "linear"
    }
    
    print(f"=== Collision Prediction Info ===")
    print(f"Ship A Initial Position: [{initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f}]")
    print(f"Predicted Collision Point: [{collision_point[0]:.2f}, {collision_point[1]:.2f}, {collision_point[2]:.2f}]")
    print(f"Predicted Collision Time: {collision_time:.2f}s")
    print(f"Collision Location: {collision_ratio*100:.1f}% of Ownship Path")
    print(f"Ship A Motion Type: {ship_config['_collision_info']['motion_type']}")
    
    return ship_config

def get_distance_3d(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def get_bearing(ship1, ship2):
    """Calculate relative bearing from ship1 to ship2
    
    Returns:
        relative_bearing: Range -180° to +180°
        - Positive: target is to starboard (right) of ship1's heading
        - Negative: target is to port (left) of ship1's heading
        - 0°: target is directly ahead
        - ±180°: target is directly behind
    """
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    angle_to_ship2 = np.degrees(theta)
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    return relative_bearing

def get_absolute_bearing(ship1, ship2):
    """Calculate absolute bearing (true bearing) from ship1 to ship2"""
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    absolute_bearing = np.degrees(theta) % 360
    return absolute_bearing

def get_angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)

def _diamond_vertices_world(target_ship: "ShipStatus", length_m: float, wingspan_m: float) -> np.ndarray:
        """
        以「菱形」定義的機體外形，在世界座標下回傳四個頂點。
        定義（機體座標系，x 前、y 右、z 下）:
            - 機頭:  (+L/2, 0)
            - 機尾:  (-L/2, 0)
            - 右翼:  (0, +W/2)
            - 左翼:  (0, -W/2)
        然後依 target_ship.heading（度，0=北，90=東）旋轉，並平移到 target_ship.position。
        回傳 shape=(4,3) 的頂點陣列（N, E, D）。
        """
        L = float(length_m)
        W = float(wingspan_m)

        # 機體座標系下（N-x、E-y 對應為：北=+x，東=+y）
        body_pts = np.array([
                [ +L/2.0,  0.0, 0.0],  # nose
                [ -L/2.0,  0.0, 0.0],  # tail
                [  0.0,   +W/2.0, 0.0],# right wing tip (東側)
                [  0.0,   -W/2.0, 0.0] # left wing tip  (西側)
        ], dtype=float)

        # 依航向（度）建立旋轉矩陣：0°=北(+x)，90°=東(+y)
        theta = np.radians(target_ship.heading)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[ c, -s, 0.0],
                                    [ s,  c, 0.0],
                                    [0.0, 0.0, 1.0]], dtype=float)

        # 旋轉 + 平移到世界座標（N, E, D）
        world_pts = (R @ body_pts.T).T + np.asarray(target_ship.position, dtype=float)
        return world_pts

def get_diamond_angular_diameter(ownship: "ShipStatus", target_ship: "ShipStatus",
                                                                 length_m: float | None = None,
                                                                 wingspan_m: float | None = None) -> float:
        """
        計算在 2D 平面（N-E）下，以「菱形外形（長、翼展）」描述的目標機，
        就本機觀測到的「角直徑（Angular diameter，度）」。

        作法：
        - 以機頭、機尾、左右翼端四頂點形成凸多邊形（菱形）。
        - 轉到世界座標後，取各頂點相對於本機位置的方位角（atan2）。
        - 在圓周上找出覆蓋所有頂點角度的最短弧長，即為角直徑。

        參數:
            - ownship: 本機 ShipStatus
            - target_ship: 目標 ShipStatus（提供位置與航向）
            - length_m: 目標機長（可不填，預設使用 DIAMOND_LENGTH_M）
            - wingspan_m: 目標機翼展（可不填，預設使用 DIAMOND_WINGSPAN_M）

        回傳:
            - 角直徑（度）。
        """
        L = DIAMOND_LENGTH_M if length_m is None else float(length_m)
        W = DIAMOND_WINGSPAN_M if wingspan_m is None else float(wingspan_m)

        # 頂點（世界座標）
        verts = _diamond_vertices_world(target_ship, L, W)

        # 以本機為原點的向量，計算各頂點的方位角
        o = np.asarray(ownship.position, dtype=float)
        vecs = verts - o

        # 若本機在形狀內（極少見），定義角直徑為 360 度
        # 這裡用點到線段的最小距離檢查是否穿越，但簡化處理：如任何頂點與本機幾乎重合
        if np.any(np.linalg.norm(vecs[:, :2], axis=1) < 1e-6):
                return 360.0

        angles = np.arctan2(vecs[:, 1], vecs[:, 0])  # [-pi, pi]
        # 轉成 [0, 2pi) 便於找最小覆蓋弧
        ang = (angles + 2 * np.pi) % (2 * np.pi)
        ang.sort()

        # 計算相鄰角度間隙，包含尾首環狀差
        diffs = np.diff(ang, append=ang[0] + 2 * np.pi)
        max_gap = np.max(diffs)

        # 最小覆蓋弧長 = 2pi - 最大間隙
        arc = 2 * np.pi - max_gap
        return np.degrees(arc)

def angle_difference(angle1, angle2):
    angle_diff = (angle2 - angle1) % (2 * math.pi)
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    return angle_diff

def angle_difference_in_deg(angle1, angle2):
    angle_diff = (angle2 - angle1) % 360
    if angle_diff > 180:
        angle_diff -= 360
    return angle_diff

def adj_ownship_heading_absolute(headings_difference, absolute_bearings, bearings_difference, absolute_bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01, ALPHA_TRIG=1.0):
    """
    Adjust ownship heading based on CBDR principle using absolute bearings.
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn
    heading = ship.heading
    max_rate_of_turn = ship.max_rate_of_turn[0]
    current_relative_bearing = get_bearing(ship, target_ship)
    # noise = np.random.normal(1, 0.1)
    # print(noise)
    angular_noises = angular_sizes[-1] #+ np.random.normal(0, 0.088)
    # if angular_noises > ALPHA_TRIG:
    #     print(f"Angular Size: {angular_sizes[-1]:.4f}°, with noise: {angular_noises:.4f}°")

    K_GAIN = 32

    if len(absolute_bearings_difference) >= 1:
        if angular_sizes[-1] > ALPHA_TRIG:
            avoidance_gain = (angular_noises - ALPHA_TRIG) * K_GAIN
            # rounded_rate = np.round(absolute_bearings_difference[-1], 5)
            # rounded_rate = min(np.abs(absolute_bearings_difference[-1]), np.abs(bearings_difference[-1]))
            bearings_rate = absolute_bearings_difference[-1] #+ np.random.normal(0, 0.022)
            # rounded_rate = bearings_difference[-1]
            if abs(bearings_rate*delta_time) <= angular_noises: #or abs(bearings_difference[-1]*delta_time) <= angular_sizes[-1]:

                if abs(bearings_rate) <= 1e-5:  # True CBDR (bearing rate ≈ 0)
                    if current_relative_bearing <= 0:  # Ship is on port side (left)
                        if abs(current_relative_bearing) <= 90:
                            rate_of_turn = -avoidance_gain  # Turn left (negative)
                        else:
                            rate_of_turn = avoidance_gain   # Turn right (positive)
                    else:  # Ship is on starboard side (right)
                        if abs(current_relative_bearing) <= 90:
                            rate_of_turn = avoidance_gain   # Turn right (positive)
                        else:
                            rate_of_turn = -avoidance_gain  # Turn left (negative)

                else:
                    # Non-zero bearing rate case
                    # rate_of_turn = -np.sign(bearings_difference[-1]) * avoidance_gain
                    if abs(current_relative_bearing) <= 90:  # Target is ahead
                        rate_of_turn = -np.sign(bearings_rate) * avoidance_gain
                        # rate_of_turn = -np.sign(bearings_difference[-1]) * avoidance_gain

                    else:  # Target is behind
                        rate_of_turn = np.sign(bearings_rate) * avoidance_gain
                        # rate_of_turn = np.sign(bearings_difference[-1]) * avoidance_gain

        else:
            # if angular_sizes[-1] < ALPHA_TRIG:
            # Navigate to goal when no collision threat
            theta_goal = get_bearing(ship, goal)
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 30:
                velocity = distance / 2
            else:
                velocity = velocity

        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def adj_ownship_heading_relative(bearings, bearings_difference, angular_sizes, ship, goal, target_ship, delta_time=0.01, ALPHA_TRIG=1.0):
    """
    Adjust ownship heading based on CBDR principle using relative bearings.
    """
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn
    max_rate_of_turn = ship.max_rate_of_turn[0]
    current_relative_bearing = get_bearing(ship, target_ship)
    avoidance_gain = angular_sizes[-1]*2

    if len(bearings_difference) >= 1:
        if abs(bearings_difference[-1]*delta_time) <= angular_sizes[-1]:
            rounded_rate = np.round(bearings_difference[-1], 5)
            if abs(rounded_rate) <= 5e-2:  # True CBDR (bearing rate ≈ 0)
                if current_relative_bearing < 0:  # Ship is on port side (left)
                    rate_of_turn = -max_rate_of_turn  # Turn left (negative)
                else:  # Ship is on starboard side (right)
                    rate_of_turn = max_rate_of_turn   # Turn right (positive)
            else:
                # Non-zero bearing rate case
                if abs(current_relative_bearing) < 90:  # Target is ahead
                    rate_of_turn = -np.sign(bearings_difference[-1]) * avoidance_gain
                else:  # Target is behind
                    rate_of_turn = np.sign(bearings_difference[-1]) * avoidance_gain

        if angular_sizes[-1] < ALPHA_TRIG:
            # Navigate to goal when no collision threat
            theta_goal = get_bearing(ship, goal)
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 1:
                velocity = distance
            else:
                velocity = 1.0
        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def run_single_simulation(use_absolute_bearings=True, 
                            ownship_config=None, 
                            ship_config=None, 
                            goal_config=None,
                            time_steps=None,
                            delta_time=None,
                            ALPHA_TRIG=None):
    """Run a single simulation with either absolute or relative bearing control
    
    Args:
        use_absolute_bearings: Whether to use absolute bearing control
        ownship_config: Dict with ownship parameters (velocity, heading, position, size, etc.)
        ship_config: Dict with ship parameters
        goal_config: Dict with goal parameters
        time_steps: Number of simulation steps
        delta_time: Time step size
        ALPHA_TRIG: Navigation threshold (degrees)
    
    Returns:
        dict: Simulation results including collision/arrival times and timeout status
    """
    
    # # Use default configurations if not provided
    # if ownship_config is None:
    #     ownship_config = DEFAULT_OWNSHIP_CONFIG.copy()
    # if ship_config is None:
    #     ship_config = DEFAULT_SHIP_CONFIG.copy()
    # if goal_config is None:
    #     goal_config = DEFAULT_GOAL_CONFIG.copy()
    # if time_steps is None:
    #     time_steps = DEFAULT_SIM_PARAMS["time_steps"]
    # if delta_time is None:
    #     delta_time = DEFAULT_SIM_PARAMS["delta_time"]
    # if ALPHA_TRIG is None:
    #     ALPHA_TRIG = DEFAULT_SIM_PARAMS["ALPHA_TRIG"]
    
    # 清理ship_config，移除不屬於ShipStatus的參數
    clean_ship_config = {k: v for k, v in ship_config.items() if not k.startswith('_')}
    
    # Initialize ship statuses using configurations
    ownship = ShipStatus(**ownship_config)
    ship = ShipStatus(**clean_ship_config)
    goal = ShipStatus(**goal_config)    # Initialize data storage
    ownship_positions, ship_positions = [], []
    bearings, angular_sizes, bearings_difference, distances, headings_difference = [], [], [], [], []
    absolute_bearings, absolute_bearings_difference = [], []
    ownship_velocities, ship_velocities, ownship_headings = [], [], []
    
    # Initialize simulation end conditions
    collision_time = None
    arrival_time = None
    timeout = False
    simulation_ended = False
    actual_steps = 0
    
    for step in range(time_steps):
        actual_steps = step + 1
        current_time = step * delta_time
        
        # Check for collision (distance < sum of radii)
        center_distance = get_distance_3d(ownship.position, ship.position)
        collision_threshold = (ownship.size + ship.size) / 2
        surface_distance = center_distance - collision_threshold
        
        if center_distance <= collision_threshold:
            collision_time = current_time
            simulation_ended = True
            # print(f"Collision detected at time {collision_time:.2f}s!")
            break
        
        # Check for goal arrival
        distance_to_goal = get_distance_3d(ownship.position, goal.position)
        arrival_threshold = (ownship.size + goal.size) / 2
        
        if distance_to_goal <= arrival_threshold:
            arrival_time = current_time
            simulation_ended = True
            # print(f"Goal reached at time {arrival_time:.2f}s!")
            break
        
        # Calculate current bearings and measurements
        bearing = get_bearing(ownship, ship)
        absolute_bearing = get_absolute_bearing(ownship, ship)
        # 使用菱形外形的角直徑取代圓形模型
        angular_size = get_diamond_angular_diameter(ownship, ship)
        
        # Store current measurements (使用表面距離)
        bearings.append(bearing)
        absolute_bearings.append(absolute_bearing)
        angular_sizes.append(angular_size)
        distances.append(surface_distance)
        ownship_velocities.append(ownship.velocity)
        ship_velocities.append(ship.velocity)
        ownship_headings.append(ownship.heading)
        
        # Apply control logic based on method
        if use_absolute_bearings:
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_absolute(headings_difference, 
                absolute_bearings, bearings_difference, absolute_bearings_difference, angular_sizes, ownship, goal, ship, delta_time, ALPHA_TRIG)
        else:
            ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_relative(
                bearings, bearings_difference, angular_sizes, ownship, goal, ship, delta_time, ALPHA_TRIG)
        
        # Store positions
        ownship_positions.append(ownship.position.copy())
        ship_positions.append(ship.position.copy())
        
        # Update ship positions
        ownship.update(delta_time)
        ship.update(delta_time)
        
        # Calculate bearing differences
        update_absolute_bearing = get_absolute_bearing(ownship, ship)
        update_bearing = get_bearing(ownship, ship)

        abs_diff = angle_difference_in_deg(absolute_bearing , update_absolute_bearing) / delta_time
        rel_diff = angle_difference_in_deg(bearing, update_bearing) / delta_time
        heading_diff = angle_difference_in_deg(ownship_headings[-1], ownship.heading) / delta_time

        absolute_bearings_difference.append(abs_diff)
        bearings_difference.append(rel_diff)
        headings_difference.append(heading_diff)

    # Check for timeout
    if not simulation_ended:
        timeout = True
        print(f"Simulation timeout after {time_steps * delta_time:.2f}s")
    
    # Convert to numpy arrays and clean up
    result = {
        'ownship_positions': np.array(ownship_positions),
        'ship_positions': np.array(ship_positions),
        'bearings': np.array(bearings),
        'absolute_bearings': np.array(absolute_bearings),
        'angular_sizes': np.array(angular_sizes),
        'bearings_difference': np.array(bearings_difference),
        'absolute_bearings_difference': np.array(absolute_bearings_difference),
        'distances': np.array(distances),
        'ownship_velocities': np.array(ownship_velocities),
        'ship_velocities': np.array(ship_velocities),
        'ownship_headings': np.array(ownship_headings),
        'ownship_size': ownship.size,
        'ship_size': ship.size,
        'goal': goal,
        # Simulation end conditions
        'collision_time': collision_time,
        'arrival_time': arrival_time,
        'timeout': timeout,
        'actual_steps': actual_steps,
        'simulation_time': actual_steps * delta_time
    }
    
    # Clean up floating point errors
    result['bearings_difference'][np.abs(result['bearings_difference']) < 1e-9] = 0.0
    result['absolute_bearings_difference'][np.abs(result['absolute_bearings_difference']) < 1e-9] = 0.0
    
    return result

def print_simulation_summary(result, method_name=""):
    """Print a summary of simulation results"""
    print(f"\n--- {method_name} Simulation Summary ---")
    print(f"Simulation time: {result['simulation_time']:.2f}s ({result['actual_steps']} steps)")
    
    if result['collision_time'] is not None:
        print(f"❌ COLLISION occurred at {result['collision_time']:.2f}s")
        min_distance = np.min(result['distances'])
        print(f"   Minimum surface distance: {min_distance:.3f}m")
    elif result['arrival_time'] is not None:
        print(f"✅ GOAL REACHED at {result['arrival_time']:.2f}s")
        final_distance_to_goal = get_distance_3d(result['ownship_positions'][-1], result['goal'].position)
        print(f"   Final distance to goal: {final_distance_to_goal:.3f}m")
    elif result['timeout']:
        print(f"⏰ TIMEOUT after {result['simulation_time']:.2f}s")
        final_distance_to_goal = get_distance_3d(result['ownship_positions'][-1], result['goal'].position)
        min_distance = np.min(result['distances'])
        print(f"   Final distance to goal: {final_distance_to_goal:.3f}m")
        print(f"   Minimum surface distance to ship: {min_distance:.3f}m")
    
    # Additional statistics
    max_angular_size = np.max(result['angular_sizes'])
    max_angular_time = np.argmax(result['angular_sizes']) * result['simulation_time'] / len(result['angular_sizes'])
    print(f"Max angular size: {max_angular_size:.4f}° at {max_angular_time:.2f}s")
    
    min_distance = np.min(result['distances'])
    min_distance_time = np.argmin(result['distances']) * result['simulation_time'] / len(result['distances'])
    print(f"Minimum surface distance: {min_distance:.3f}m at {min_distance_time:.2f}s")

def plot_results(result, delta_time, title_prefix=""):
    """Plot results in a single 2x4 grid"""
    # Print simulation summary
    print_simulation_summary(result, title_prefix.rstrip(" - "))
    
    # Add simulation result info to title
    sim_info = ""
    if result['collision_time'] is not None:
        sim_info = f" - COLLISION at {result['collision_time']:.2f}s"
    elif result['arrival_time'] is not None:
        sim_info = f" - GOAL REACHED at {result['arrival_time']:.2f}s"
    elif result['timeout']:
        sim_info = f" - TIMEOUT after {result['simulation_time']:.2f}s"
    
    fig = plt.figure(figsize=(20, 10))
    fig.suptitle(f"{title_prefix}Simulation Results{sim_info}", fontsize=16, fontweight='bold')
    
    # Plot all 8 subplots for this result
    plot_all_subplots(result, delta_time, title_prefix)
    
    plt.tight_layout()
    plt.show()

def plot_all_subplots(result, delta_time, title_prefix=""):
    """Plot all 8 subplots in a 2x4 grid"""
    
    # 1. Ship Positions
    plt.subplot(2, 4, 1)
    ownship_line, = plt.plot(result['ownship_positions'][:, 1], result['ownship_positions'][:, 0], label='Ownship')
    ship_line, = plt.plot(result['ship_positions'][:, 1], result['ship_positions'][:, 0], label='Ship A')
    
    # Add direction arrows at final positions
    plt.annotate('', xy=(result['ownship_positions'][-1, 1], result['ownship_positions'][-1, 0]), 
                xytext=(result['ownship_positions'][-2, 1], result['ownship_positions'][-2, 0]), 
                arrowprops=dict(arrowstyle='->', color=ownship_line.get_color()))
    plt.annotate('', xy=(result['ship_positions'][-1, 1], result['ship_positions'][-1, 0]), 
                xytext=(result['ship_positions'][-2, 1], result['ship_positions'][-2, 0]), 
                arrowprops=dict(arrowstyle='->', color=ship_line.get_color()))
    
    # Add ship size circles and goal
    ownship_circle = plt.Circle((result['ownship_positions'][-1, 1], result['ownship_positions'][-1, 0]),
        result['ownship_size'] / 2, color=ownship_line.get_color(), fill=False, linestyle='--', alpha=1.0)
    ship_circle = plt.Circle((result['ship_positions'][-1, 1], result['ship_positions'][-1, 0]),
        result['ship_size'] / 2, color=ship_line.get_color(), fill=False, linestyle='--', alpha=1.0)
    goal_circle = plt.Circle((result['goal'].position[1], result['goal'].position[0]),
        1, color='Red', fill=False, linestyle='--', alpha=1.0)
    plt.gca().add_patch(goal_circle)
    plt.gca().add_patch(ownship_circle)
    plt.gca().add_patch(ship_circle)
    
    # Add arrows at 10-second intervals
    time_interval = 10.0
    point_interval = int(time_interval / delta_time)
    arrow_length = 5.0
    
    for i in range(0, len(result['ownship_positions']), point_interval):
        # Plot position points
        plt.plot(result['ownship_positions'][i, 1], result['ownship_positions'][i, 0], 'o', 
                color=ownship_line.get_color(), markersize=6, alpha=0.7)
        plt.plot(result['ship_positions'][i, 1], result['ship_positions'][i, 0], 'o', 
                color=ship_line.get_color(), markersize=6, alpha=0.7)

        # Add three arrows for ownship
        ownship_pos = result['ownship_positions'][i]
        ownship_heading = result['ownship_headings'][i] if i < len(result['ownship_headings']) else result['ownship_headings'][-1]
        
        # North arrow (red)
        north_end_x = ownship_pos[1]
        north_end_y = ownship_pos[0] + arrow_length
        plt.annotate('', xy=(north_end_x, north_end_y), 
                    xytext=(ownship_pos[1], ownship_pos[0]), 
                    arrowprops=dict(arrowstyle='->', color='red', lw=1.5, alpha=0.7))
        
        # Heading arrow
        heading_end_x = ownship_pos[1] + arrow_length * np.sin(np.radians(ownship_heading))
        heading_end_y = ownship_pos[0] + arrow_length * np.cos(np.radians(ownship_heading))
        plt.annotate('', xy=(heading_end_x, heading_end_y), 
                    xytext=(ownship_pos[1], ownship_pos[0]), 
                    arrowprops=dict(arrowstyle='->', color=ownship_line.get_color(), lw=1.5, alpha=0.7))
        
        # Arrow to other ship
        ship_pos = result['ship_positions'][i]
        direction_to_ship = ship_pos - ownship_pos
        if np.linalg.norm(direction_to_ship) > 0:
            direction_to_ship_normalized = direction_to_ship / np.linalg.norm(direction_to_ship)
            ship_arrow_end = ownship_pos + arrow_length * direction_to_ship_normalized
            plt.annotate('', xy=(ship_arrow_end[1], ship_arrow_end[0]), 
                        xytext=(ownship_pos[1], ownship_pos[0]), 
                        arrowprops=dict(arrowstyle='->', color=ship_line.get_color(), lw=1.5, alpha=0.7))
    
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title(f'{title_prefix}Ship Positions')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # 2. Bearing Plot
    plt.subplot(2, 4, 2)
    plt.plot(result['bearings'], np.arange(len(result['bearings'])) * delta_time, label='Relative Bearing', alpha=1.0)
    absolute_bearings_normalized = result['absolute_bearings'].copy()
    absolute_bearings_normalized[absolute_bearings_normalized > 180] -= 360
    plt.plot(absolute_bearings_normalized, np.arange(len(absolute_bearings_normalized)) * delta_time, label='Absolute Bearing', linewidth=1)
    half_angular_sizes = result['angular_sizes'] / 2
    plt.plot(result['bearings'] - half_angular_sizes, np.arange(len(result['bearings'])) * delta_time, '--', alpha=0.9)
    plt.plot(result['bearings'] + half_angular_sizes, np.arange(len(result['bearings'])) * delta_time, '--', alpha=0.9)
    plt.ylabel('Time (s)')
    plt.xlabel('Bearing (degrees)')
    plt.title(f'{title_prefix}Bearing')
    plt.xlim(-181, 180)
    plt.axvline(x=0, color='r', linestyle='--')
    for angle in [45, 90, 135, 180, -45, -90, -135, -180]:
        plt.axvline(x=angle, color='g' if angle % 90 != 0 else 'b', linestyle='--')
    plt.legend()
    plt.grid(True)
    
    # 3. Angular Size Plot
    plt.subplot(2, 4, 3)
    plt.plot(np.arange(len(result['angular_sizes'])) * delta_time, result['angular_sizes'], label='Angular Size')
    
    max_angular_size = np.max(result['angular_sizes'])
    max_time_index = np.argmax(result['angular_sizes'])
    max_time = max_time_index * delta_time
    
    plt.axhline(y=max_angular_size, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Max: {max_angular_size:.3f}°')
    plt.axvline(x=max_time, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Time: {max_time:.1f}s')
    
    plt.annotate(f'Max: {max_angular_size:.3f}° at {max_time:.1f}s', 
                xy=(max_time, max_angular_size), 
                xytext=(max_time + 3, max_angular_size),
                arrowprops=dict(arrowstyle='->', color='red', alpha=0.8),
                fontsize=8, color='red', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8))
    
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Size (degrees)')
    plt.title(f'{title_prefix}Angular Size')
    plt.legend()
    plt.grid(True)
    
    # 4. Distance Plot
    plt.subplot(2, 4, 4)
    plt.plot(np.arange(len(result['distances'])) * delta_time, result['distances'], label='Surface Distance')
    
    min_distance = np.min(result['distances'])
    min_distance_time_index = np.argmin(result['distances'])
    min_distance_time = min_distance_time_index * delta_time
    
    plt.axhline(y=min_distance, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Min: {min_distance:.1f}m')
    plt.axvline(x=min_distance_time, color='red', linestyle='--', alpha=0.8, linewidth=2, 
                label=f'Time: {min_distance_time:.1f}s')
    
    plt.annotate(f'Min: {min_distance:.1f}m at {min_distance_time:.1f}s', 
                xy=(min_distance_time, min_distance), 
                xytext=(min_distance_time + 3, min_distance),
                arrowprops=dict(arrowstyle='->', color='red', alpha=0.8),
                fontsize=8, color='red', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.8))
    
    plt.xlabel('Time (s)')
    plt.ylabel('Surface Distance (m)')
    plt.title(f'{title_prefix}Surface Distance')
    plt.legend()
    plt.grid(True)
    
    # 5. Relative Bearing Rate (d/dt)
    plt.subplot(2, 4, 5)
    t_rel_rate = np.arange(len(result['bearings_difference'])) * delta_time
    if len(t_rel_rate) > 0:
        plt.plot(t_rel_rate, result['bearings_difference'], label='Relative Bearing Rate')
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0 Line')
    plt.xlabel('Time (s)')
    plt.ylabel('Bearing rate (deg/s)')
    plt.title(f'{title_prefix}Relative Bearing Rate')
    plt.legend()
    plt.grid(True)
    
    # 6. Ship Velocities
    plt.subplot(2, 4, 6)
    plt.plot(np.arange(len(result['ownship_velocities'])) * delta_time, result['ownship_velocities'], 
             label='Ownship Velocity', color='blue')
    plt.plot(np.arange(len(result['ship_velocities'])) * delta_time, result['ship_velocities'], 
             label='Ship A Velocity', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title(f'{title_prefix}Velocities')
    plt.legend()
    plt.grid(True)
    
    # 7. Ownship Heading
    plt.subplot(2, 4, 7)
    plt.plot(np.arange(len(result['ownship_headings'])) * delta_time, result['ownship_headings'], 
             label='Ownship Heading', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title(f'{title_prefix}Ownship Heading')
    plt.legend()
    plt.grid(True)
    
    # 8. Absolute Bearing Rate (d/dt)
    plt.subplot(2, 4, 8)
    t_abs_rate = np.arange(len(result['absolute_bearings_difference'])) * delta_time
    if len(t_abs_rate) > 0:
        plt.plot(t_abs_rate, result['absolute_bearings_difference'], label='Absolute Bearing Rate', linewidth=1)
    plt.axhline(y=0, color='black', linestyle='--', alpha=0.5, label='0 Line')
    plt.xlabel('Time (s)')
    plt.ylabel('Bearing rate (deg/s)')
    plt.title(f'{title_prefix}Absolute Bearing Rate')
    plt.legend()
    plt.grid(True)

def run_comparison(ownship_config=None, ship_config=None, goal_config=None, 
                  time_steps=None, delta_time=None, ALPHA_TRIG=None):
    """Run both simulations and plot comparison
    
    Args:
        ownship_config: Dict with ownship parameters
        ship_config: Dict with ship parameters  
        goal_config: Dict with goal parameters
        time_steps: Number of simulation steps
        delta_time: Time step size
        ALPHA_TRIG: Navigation threshold (degrees)
    """
    # Use defaults if not provided
    if time_steps is None:
        time_steps = DEFAULT_SIM_PARAMS["time_steps"]
    if delta_time is None:
        delta_time = DEFAULT_SIM_PARAMS["delta_time"]
    if ALPHA_TRIG is None:
        ALPHA_TRIG = DEFAULT_SIM_PARAMS["ALPHA_TRIG"]
        
    print("Running simulation with absolute bearing control...")
    abs_result = run_single_simulation(
        use_absolute_bearings=True,
        ownship_config=ownship_config,
        ship_config=ship_config,
        goal_config=goal_config,
        time_steps=time_steps,
        delta_time=delta_time,
        ALPHA_TRIG=ALPHA_TRIG
    )
    
    print("Plotting absolute bearing control results...")
    plot_results(abs_result, delta_time=delta_time, title_prefix="Absolute Bearing Control - ")

    print("Running simulation with relative bearing control...")
    rel_result = run_single_simulation(
        use_absolute_bearings=False,
        ownship_config=ownship_config,
        ship_config=ship_config,
        goal_config=goal_config,
        time_steps=time_steps,
        delta_time=delta_time,
        ALPHA_TRIG=ALPHA_TRIG
    )
    
    print("Plotting relative bearing control results...")
    plot_results(rel_result, delta_time=delta_time, title_prefix="Relative Bearing Control - ")

def run_simulation(ownship_config=None, ship_config=None, goal_config=None,
                    time_steps=None, delta_time=None, ALPHA_TRIG=None):
    """Run single simulation with absolute bearing control
    
    Args:
        ownship_config: Dict with ownship parameters
        ship_config: Dict with ship parameters
        goal_config: Dict with goal parameters
        time_steps: Number of simulation steps
        delta_time: Time step size
        ALPHA_TRIG: Navigation threshold (degrees)
    """
    # Use defaults if not provided
    if time_steps is None:
        time_steps = DEFAULT_SIM_PARAMS["time_steps"]
    if delta_time is None:
        delta_time = DEFAULT_SIM_PARAMS["delta_time"]
    if ALPHA_TRIG is None:
        ALPHA_TRIG = DEFAULT_SIM_PARAMS["ALPHA_TRIG"]
        
    print("Running simulation with absolute bearing control...")
    abs_result = run_single_simulation(
        use_absolute_bearings=True,
        ownship_config=ownship_config,
        ship_config=ship_config,
        goal_config=goal_config,
        time_steps=time_steps,
        delta_time=delta_time,
        ALPHA_TRIG=ALPHA_TRIG
    )
    
    print("Plotting absolute bearing control results...")
    plot_results(abs_result, delta_time=delta_time, title_prefix="Absolute Bearing Control - ")   

if __name__ == "__main__":

    left_turn_ship = create_collision_scenario_with_turning(
        ship_velocity=2.0,
        ship_heading=-90.0,
        ship_rate_of_turn=1.0,
        collision_ratio=0.5,
    )
    left_turn_ship['size'] = 0.5
    run_simulation(
        ownship_config=DEFAULT_OWNSHIP_CONFIG,
        ship_config=left_turn_ship,
        goal_config=DEFAULT_GOAL_CONFIG
    )