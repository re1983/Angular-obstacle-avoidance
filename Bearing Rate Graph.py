import numpy as np
import matplotlib.pyplot as plt

class ShipStatus:
    def __init__(self, name, speed, acceleration, heading, Rate_of_Turn, position, size = 1.0):
        self.name = name
        self.speed = speed
        self.acceleration = acceleration
        self.heading = heading
        self.Rate_of_Turn = Rate_of_Turn
        self.position = np.array(position, dtype=float)
        self.size = size

    def update(self, delta_time=0.01):
        self.heading = self.heading + self.Rate_of_Turn * delta_time
        self.position += self.speed * delta_time * np.array([
            np.cos(np.radians(self.heading)),
            np.sin(np.radians(self.heading)),
            0])
        self.speed = self.speed + self.acceleration

    def get_status(self):
        return {
            "name": self.name,
            "speed": self.speed,
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
    if(relative_bearing > 180):
        relative_bearing -= 360
    return relative_bearing

def get_Angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)

# Example usage
#name, speed, acceleration, heading, Rate_of_Turn, position
# ownship = ShipStatus("Ownship", speed=5.0, acceleration=0, heading=0.0, Rate_of_Turn=-0.0, position=[-50, 0, 0])
# ship = ShipStatus("Ship A", speed=7.07, acceleration=0, heading=135.0, Rate_of_Turn=-0.0, position=[50, -50, 0])

ownship = ShipStatus("Ownship", speed=0.0, acceleration=0, heading=0.0, Rate_of_Turn=-0.0, position=[0, 0, 0])
ship = ShipStatus("Ship A", speed=1.0, acceleration=0, heading=90.0, Rate_of_Turn=-0.0, position=[1, -5, 0])

# Simulation parameters
time_steps = 1000
delta_time = 0.01

# Lists to store positions for plotting
ownship_positions = []
ship_positions = []
bearings = []
angular_sizes = []
# goal = ShipStatus("Ship A", speed=0.0, acceleration=0, heading=0.0, Rate_of_Turn=0.0, position=[0, 0, 0])
# bearings_to_goal = []


# Simulation loop
for _ in range(time_steps):

    # print(ship.get_status())
    # print(ownship.get_status())
    bearing = get_bearing(ownship, ship)
    bearings.append(bearing)
    angular_size = get_Angular_diameter(ownship, ship)
    angular_sizes.append(angular_size)
    # bearing_to_goal = get_bearing(ownship, goal)
    # bearings_to_goal.append(bearing_to_goal)
    # print("Bearing to Ship A:", bearing)
    ownship_positions.append(ownship.position.copy())
    ship_positions.append(ship.position.copy())
    ownship.update(delta_time)
    ship.update(delta_time)

# Convert positions to numpy arrays for easier plotting
ownship_positions = np.array(ownship_positions)
ship_positions = np.array(ship_positions)

plt.figure(figsize=(10, 12))
plt.subplot(5, 1, 1)

# Plotting
# plt.figure(figsize=(10, 6))
# plt.plot(ownship_positions[:, 0], ownship_positions[:, 1], label='Ownship')
# plt.plot(ship_positions[:, 0], ship_positions[:, 1], label='Ship A')
# plt.xlabel('North (m)')
# plt.ylabel('East (m)')
# plt.title('Ship Positions Over Time in NED Coordinates')
# plt.legend()
# plt.grid(True)
# plt.show()

# # Rotate the plot by 90 degrees
# plt.figure(figsize=(10, 6))
plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
plt.legend()
plt.grid(True)
plt.axis('equal')  # Ensure the aspect ratio is equal


# Convert bearings to numpy array for easier plotting
bearings = np.array(bearings)
# print(len(bearings))
angular_sizes = np.array(angular_sizes)
# print(len(angular_sizes))
# print(angular_sizes)
# bearings = np.unwrap(bearings, period=360)  # Unwrap the bearings to remove jumps
# Plotting the bearing rate
plt.subplot(5, 1, 2)
plt.plot(bearings, np.arange(time_steps) * delta_time, label='Bearing to Ship A')
half_angular_sizes = angular_sizes / 2
plt.plot(bearings - half_angular_sizes, np.arange(time_steps) * delta_time)
plt.plot(bearings + half_angular_sizes, np.arange(time_steps) * delta_time)
plt.ylabel('Time (s)')
plt.xlabel('Bearing (degrees)')
plt.title('Bearing to Ship A Over Time')
plt.axvline(x=0, color='r', linestyle='--')  # Add a vertical line at x=0
# plt.axvline(x=180, color='b', linestyle='--')
# plt.axvline(x=-180, color='b', linestyle='--')
plt.legend()
plt.grid(True)
# # plt.gca().invert_yaxis()  # Invert the Y-axis

#ploting angular_sizes
plt.subplot(5, 1, 3)
plt.plot(np.arange(time_steps) * delta_time, angular_sizes, label='Angular Size of Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Angular Size (degrees)')
plt.title('Angular Size of Ship A Over Time')
plt.legend()
plt.grid(True)

bearing_rate = np.gradient(bearings, delta_time)
# Plotting the bearing rate
plt.subplot(5, 1, 4)
plt.plot(np.arange(time_steps) * delta_time, bearing_rate, label='Bearing Rate to Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Bearing Rate (degrees/s)')
plt.title('Bearing Rate to Ship A Over Time')
plt.legend()
plt.grid(True)


jerk = np.gradient(bearing_rate, delta_time)

plt.subplot(5, 1, 5)
plt.plot(np.arange(time_steps) * delta_time, jerk, label='Jerk to Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Jerk (degrees/s^2)')
plt.title('Jerk to Ship A Over Time')
plt.legend()
plt.grid(True)

plt.show()


# plt.subplot(3, 1, 3)
# plt.plot(np.arange(time_steps) * delta_time, bearings_to_goal, label='Bearing to Goal')
# #plot bearing to goal error
# plt.xlabel('Time (s)')
# plt.ylabel('Bearing (degrees)')
# plt.title('Bearing to Goal Over Time')
# plt.legend()
# plt.grid(True)
# plt.show()