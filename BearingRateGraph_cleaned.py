import numpy as np
import matplotlib.pyplot as plt
import math

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size=1.0, max_rate_of_turn=[6, 6], velocity_limit=[0.5, 10.0]):
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

def get_distance_3d(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

def get_bearing(ship1, ship2):
    delta_pos = ship2.position - ship1.position
    theta = np.arctan2(delta_pos[1], delta_pos[0])
    angle_to_ship2 = np.degrees(theta)
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    if relative_bearing > 180:
        relative_bearing -= 360
    return relative_bearing

def get_angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)

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

def adj_ownship_heading(bearings, bearings_difference, angular_sizes, ship, goal, delta_time=0.01):
    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn

    if len(bearings_difference) > 1:


        # angular = angular_sizes[-1]

        # if abs(bearings_difference[-1]) == 0.0:
        #     if bearings[-1] >= 0 and bearings[-1] < 90:
        #         rate_of_turn = angular
        #     elif bearings[-1] >= 90 and bearings[-1] < 180:
        #         rate_of_turn = -angular
        #     elif bearings[-1] >= -180 and bearings[-1] < -90:
        #         rate_of_turn = angular
        #     elif bearings[-1] >= -90 and bearings[-1] < 0:
        #         rate_of_turn = -angular
        if abs(bearings_difference[-1]*delta_time) <= angular_sizes[-1] and angular_sizes[-1] > 3.0:
            # print(f"Adjusting rate of turn: {rate_of_turn} degrees/s")
            # rate_of_turn = ship.rate_of_turn + np.sign(ship.rate_of_turn) * (angular_sizes[-1] - abs(bearings_difference[-1]*delta_time))
            rate_of_turn = -np.sign(bearings_difference[-1])* (angular_sizes[-1] - abs(bearings_difference[-1]*delta_time))
        else:
            theta_goal = get_bearing(ship, goal)
            rate_of_turn = theta_goal
            distance = get_distance_3d(ship.position, goal.position)
            if distance < 1:
                velocity = distance
            else:
                velocity = 1.0
        rate_of_turn = np.clip(rate_of_turn, -ship.max_rate_of_turn[0], ship.max_rate_of_turn[0])

    return rate_of_turn, velocity

def run_simulation():
    ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=-0.0, position=[0, 0, 0])
    ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=90.0, rate_of_turn=0, position=[10, -10, 0])
    goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])
    time_steps = 6000
    delta_time = 0.01
    ownship_positions, ship_positions = [], []
    bearings, angular_sizes, bearings_difference, distances = [], [], [], []
    for _ in range(time_steps):
        bearing = get_bearing(ownship, ship)
        bearings.append(bearing)
        angular_size = get_angular_diameter(ownship, ship)
        angular_sizes.append(angular_size)
        distances.append(get_distance_3d(ownship.position, ship.position))
        ownship.rate_of_turn, ownship.velocity = adj_ownship_heading(bearings, bearings_difference, angular_sizes, ownship, goal, delta_time)
        ownship_positions.append(ownship.position.copy())
        ship_positions.append(ship.position.copy())
        ownship.update(delta_time)
        ship.update(delta_time)
        update_bearing = get_bearing(ownship, ship)
        bearings_difference.append(angle_difference_in_deg(bearing, update_bearing) / delta_time)
    ownship_positions = np.array(ownship_positions)
    ship_positions = np.array(ship_positions)
    bearings = np.array(bearings)
    angular_sizes = np.array(angular_sizes)
    bearings_difference = np.array(bearings_difference)
    distances = np.array(distances)
    jerk = np.gradient(bearings_difference, delta_time)
    plot_simulation_results(ownship_positions, ship_positions, bearings, angular_sizes, bearings_difference, distances, jerk, delta_time)

def plot_simulation_results(ownship_positions, ship_positions, bearings, angular_sizes, bearings_difference, distances, jerk, delta_time):
    plt.figure(figsize=(20, 24))
    plt.subplot(6, 1, 1)
    ownship_line, = plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
    ship_line, = plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
    plt.annotate('', xy=(ownship_positions[-1, 1], ownship_positions[-1, 0]), xytext=(ownship_positions[-2, 1], ownship_positions[-2, 0]), arrowprops=dict(arrowstyle='->', color=ownship_line.get_color()))
    plt.annotate('', xy=(ship_positions[-1, 1], ship_positions[-1, 0]), xytext=(ship_positions[-2, 1], ship_positions[-2, 0]), arrowprops=dict(arrowstyle='->', color=ship_line.get_color()))
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.subplot(6, 1, 2)
    plt.plot(bearings, np.arange(len(bearings)) * delta_time, label='Bearing to Ship A')
    half_angular_sizes = angular_sizes / 2
    plt.plot(bearings - half_angular_sizes, np.arange(len(bearings)) * delta_time)
    plt.plot(bearings + half_angular_sizes, np.arange(len(bearings)) * delta_time)
    plt.ylabel('Time (s)')
    plt.xlabel('Bearing (degrees)')
    plt.title('Bearing to Ship A Over Time')
    plt.axvline(x=0, color='r', linestyle='--')
    plt.axvline(x=45, color='g', linestyle='--')
    plt.axvline(x=90, color='b', linestyle='--')
    plt.axvline(x=135, color='g', linestyle='--')
    plt.axvline(x=180, color='b', linestyle='--')
    plt.axvline(x=-45, color='g', linestyle='--')
    plt.axvline(x=-90, color='b', linestyle='--')
    plt.axvline(x=-135, color='g', linestyle='--')
    plt.axvline(x=-180, color='b', linestyle='--')
    plt.legend()
    plt.grid(True)
    plt.subplot(6, 1, 3)
    plt.plot(np.arange(len(angular_sizes)) * delta_time, angular_sizes, label='Angular Size of Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Size (degrees)')
    plt.title('Angular Size of Ship A Over Time')
    plt.legend()
    plt.grid(True)
    plt.subplot(6, 1, 4)
    plt.plot(np.arange(len(distances)) * delta_time, distances, label='Distance to Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.axvline(x=0, color='r', linestyle='--')
    plt.axhline(y=0, color='r', linestyle='--')
    plt.title('Distance to Ship A Over Time')
    plt.legend()
    plt.grid(True)
    plt.subplot(6, 1, 5)
    plt.plot(np.arange(len(bearings_difference)) * delta_time, bearings_difference, label='Bearing Difference')
    plt.xlabel('Time (s)')
    plt.ylabel('Bearing Difference (degrees/s)')
    plt.title('Bearing Difference Over Time')
    plt.legend()
    plt.grid(True)
    plt.subplot(6, 1, 6)
    plt.plot(np.arange(len(jerk)) * delta_time, jerk, label='Jerk to Ship A')
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (degrees/s^2)')
    plt.title('Jerk to Ship A Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    run_simulation()
