import numpy as np
import matplotlib.pyplot as plt
import math
import cmath

class ShipStatus:
    def __init__(self, name, velocity, acceleration, heading, rate_of_turn, position, size = 1.0, max_rate_of_turn = [0, 0], velocity_limit = [ 0.5, 10.0]):
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
        self.heading = self.heading + self.rate_of_turn * delta_time
        self.position += self.velocity * delta_time * np.array([
            np.cos(np.radians(self.heading)),
            np.sin(np.radians(self.heading)),
            0])
        self.velocity = self.velocity + self.acceleration

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
    if(relative_bearing > 180):
        relative_bearing -= 360
    return relative_bearing

def get_Angular_diameter(ship1, ship2):
    distance = get_distance_3d(ship1.position, ship2.position)
    angular_diameter = 2 * np.arctan(ship2.size / (2 * distance))
    return np.degrees(angular_diameter)


# def adj_ownship_heading_velocity(bearings, angular_sizes, ship):

#     heading = (bearings[-1] - 90) % 360
#     # heading = ship.heading
#     velocity = ship.velocity

#     return heading, velocity

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


def adj_ownship_rate_of_turn(bearings, bearings_difference, angular_size, ship, goal, delta_time=0.01):

    velocity = ship.velocity
    rate_of_turn = ship.rate_of_turn

    if len(bearings_difference) > 1: # and angular_size > 0.25:
        theta_goal = get_bearing(ship, goal)
        rate_of_turn = theta_goal
        distance = get_distance_3d(ship.position, goal.position)
        if distance < 1:
            velocity = distance
        else:
            velocity = 1.0
        
        if abs(bearings_difference[-1]) <= 0.01 and angular_size > 0.1:
            if bearings[-1] < 0 and bearings[-1] > -90:
                rate_of_turn = -45 
            elif bearings[-1] < -90 and bearings[-1] > -180:
                rate_of_turn = 45
            elif bearings[-1] > 0 and bearings[-1] < 90:
                rate_of_turn = 45
            elif bearings[-1] > 90 and bearings[-1] < 180:
                rate_of_turn = -45

        # elif abs(bearings_difference[-1]) > 0.01 and angular_size > 0.1:
        else:
            rate_of_turn = -(1/bearings_difference[-1]) * angular_size
            # normalize=(bearings_difference[-1] - 1) / (100 - 1)

            # if bearings_difference[-1] > 0:
            #     rate_of_turn = - (normalize * angular_size)
            # else:
            #     rate_of_turn = normalize * angular_size
            # if bearings[-1] < 90 and bearings[-1] > -90:
            #     rate_of_turn = -(1/bearings_difference[-1]) * 5
            # else:
            #     rate_of_turn = 1/bearings_difference[-1] * 5


    # else:
    #     theta_goal = get_bearing(ship, goal)
    #     # print('sita_goal:', sita_goal)

    #     rate_of_turn = theta_goal * 0.1
    #     distance = get_distance_3d(ship.position, goal.position)
    #     if distance < 1:
    #         velocity = distance
    #     # rate_of_turn = angle_difference_in_deg(ship.heading, sita_goal) * 0.3
    #     # print('rate_of_turn:', rate_of_turn)
    return rate_of_turn, velocity

# Example usage
#name, velocity, acceleration, heading, Rate_of_Turn, position
# ownship = ShipStatus("Ownship", velocity=5.0, acceleration=0, heading=0.0, Rate_of_Turn=-0.0, position=[-50, 0, 0])
# ship = ShipStatus("Ship A", velocity=7.07, acceleration=0, heading=135.0, Rate_of_Turn=-0.0, position=[50, -50, 0])

ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=0.0, rate_of_turn=-0.0, position=[0, 0, 0])
ship = ShipStatus("Ship A", velocity=1.0, acceleration=0, heading=-90.0, rate_of_turn=0, position=[10, 10, 0])
goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0.0, rate_of_turn=0.0, position=[20, 0, 0])

# Simulation parameters
time_steps = 3000
delta_time = 0.01

# Lists to store positions for plotting
ownship_positions = []
ship_positions = []
bearings = []
angular_sizes = []
bearings_difference = []
distances = []
# goal = ShipStatus("Ship A", velocity=0.0, acceleration=0, heading=0.0, Rate_of_Turn=0.0, position=[0, 0, 0])
# bearings_to_goal = []


# Simulation loop
for _ in range(time_steps):

    # print(ship.get_status())
    # print(ownship.get_status())
    bearing = get_bearing(ownship, ship)
    bearings.append(bearing)
    angular_size = get_Angular_diameter(ownship, ship)
    angular_sizes.append(angular_size)
    distances.append(get_distance_3d(ownship.position, ship.position))
    # bearing_to_goal = get_bearing(ownship, goal)
    # bearings_to_goal.append(bearing_to_goal)
    # print("Bearing to Ship A:", bearing)
    ownship.rate_of_turn, ownship.velocity = adj_ownship_rate_of_turn(bearings, bearings_difference, angular_size, ownship, goal, delta_time)
    ownship_positions.append(ownship.position.copy())
    ship_positions.append(ship.position.copy())
    # adj_ownship_heading_velocity(bearings, angular_sizes, ship)
    # ownship.heading, ownship.velocity = adj_ownship_heading_velocity(bearings, angular_sizes, ownship)

    # ownship.velocity = time_steps * delta_time
    ownship.update(delta_time)
    ship.update(delta_time)

    update_bearing = get_bearing(ownship, ship)
    # angle_difference(bearing, update_bearing)
    # bearings_difference.append(angle_difference(math.radians(bearing), math.radians(update_bearing))/delta_time*180/math.pi)
    bearings_difference.append(angle_difference_in_deg(bearing, update_bearing)/delta_time)

# Convert positions to numpy arrays for easier plotting
ownship_positions = np.array(ownship_positions)
ship_positions = np.array(ship_positions)

plt.figure(figsize=(20, 24))

plt.subplot(6, 1, 1)

ownship_line, = plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
ship_line,   = plt.plot(ship_positions[:, 1],  ship_positions[:, 0],  label='Ship A')

# Add arrow for Ownship
plt.annotate(
    '',
    xy=(ownship_positions[-1, 1], ownship_positions[-1, 0]),
    xytext=(ownship_positions[-2, 1], ownship_positions[-2, 0]),
    arrowprops=dict(arrowstyle='->', color=ownship_line.get_color())
)

# Add arrow for Ship A
plt.annotate(
    '',
    xy=(ship_positions[-1, 1], ship_positions[-1, 0]),
    xytext=(ship_positions[-2, 1], ship_positions[-2, 0]),
    arrowprops=dict(arrowstyle='->', color=ship_line.get_color())
)

plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
plt.legend()
plt.grid(True)
plt.axis('equal')  # Ensure the aspect ratio is equal


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
# plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
# plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
# plt.xlabel('East (m)')
# plt.ylabel('North (m)')
# plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
# plt.legend()
# plt.grid(True)
# plt.axis('equal')  # Ensure the aspect ratio is equal


# Convert bearings to numpy array for easier plotting
bearings = np.array(bearings)
# print(len(bearings))
angular_sizes = np.array(angular_sizes)
bearing_difference = np.array(bearings_difference)
# print(len(angular_sizes))
# print(angular_sizes)
# bearings = np.unwrap(bearings, period=360)  # Unwrap the bearings to remove jumps
# Plotting the bearing rate
plt.subplot(6, 1, 2)
plt.plot(bearings, np.arange(time_steps) * delta_time, label='Bearing to Ship A')
half_angular_sizes = angular_sizes / 2
plt.plot(bearings - half_angular_sizes, np.arange(time_steps) * delta_time)
plt.plot(bearings + half_angular_sizes, np.arange(time_steps) * delta_time)
plt.ylabel('Time (s)')
plt.xlabel('Bearing (degrees)')
plt.title('Bearing to Ship A Over Time')
plt.axvline(x=0, color='r', linestyle='--')  # Add a vertical line at x=0
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
# # plt.gca().invert_yaxis()  # Invert the Y-axis

#ploting angular_sizes
plt.subplot(6, 1, 3)
plt.plot(np.arange(time_steps) * delta_time, angular_sizes, label='Angular Size of Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Angular Size (degrees)')
plt.title('Angular Size of Ship A Over Time')
plt.legend()
plt.grid(True)

#position distance
plt.subplot(6, 1, 4)
plt.plot(np.arange(time_steps) * delta_time, distances, label='Distance to Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.axvline(x=0, color='r', linestyle='--')
plt.axhline(y=0, color='r', linestyle='--')
plt.title('Distance to Ship A Over Time')
plt.legend()
plt.grid(True)

# bearing_rate = np.gradient(bearings, delta_time)
# # Plotting the bearing rate
# plt.subplot(5, 1, 4)
# plt.plot(np.arange(time_steps) * delta_time, bearing_rate, label='Bearing Rate to Ship A')
# plt.xlabel('Time (s)')
# plt.ylabel('Bearing Rate (degrees/s)')
# plt.title('Bearing Rate to Ship A Over Time')
# plt.legend()
# plt.grid(True)

plt.subplot(6, 1, 5)
plt.plot(np.arange(time_steps) * delta_time, bearings_difference, label='Bearing Difference')
plt.xlabel('Time (s)')
plt.ylabel('Bearing Difference (degrees/s)')
plt.title('Bearing Difference Over Time')
plt.legend()
plt.grid(True)


jerk = np.gradient(bearings_difference, delta_time)

plt.subplot(6, 1, 6)
plt.plot(np.arange(time_steps) * delta_time, jerk, label='Jerk to Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Jerk (degrees/s^2)')
plt.title('Jerk to Ship A Over Time')
plt.legend()
plt.grid(True)






# plt.subplot(3, 1, 3)
# plt.plot(np.arange(time_steps) * delta_time, bearings_to_goal, label='Bearing to Goal')
# #plot bearing to goal error
# plt.xlabel('Time (s)')
# plt.ylabel('Bearing (degrees)')
# plt.title('Bearing to Goal Over Time')
# plt.legend()
# plt.grid(True)
# plt.show()

# # Convert positions to numpy arrays for easier plotting
# ownship_positions = np.array(ownship_positions)
# ship_positions = np.array(ship_positions)

# # Plot ego-center map
# plt.figure(figsize=(10, 6))
# # Relative positions
# rel_x = ship_positions[:, 1] - ownship_positions[:, 1]
# rel_y = ship_positions[:, 0] - ownship_positions[:, 0]

# # Plotting ego-center map
# plt.plot(rel_x, rel_y, label='Ship A', color='red')

# # Add an arrow on the last segment
# plt.annotate(
#     '',
#     xy=(rel_x[-1], rel_y[-1]),
#     xytext=(rel_x[-2], rel_y[-2]),
#     arrowprops=dict(arrowstyle='->', color='red')
# )

# plt.xlabel('Relative East (m)')
# plt.ylabel('Relative North (m)')
# plt.title('Ego-Center Map of Ship Positions Over Time')
# plt.legend()
# plt.grid(True)
# plt.axis('equal')

# # Plotting
# plt.figure(figsize=(10, 6))
# ownship_line, = plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
# ship_line,   = plt.plot(ship_positions[:, 1],  ship_positions[:, 0],  label='Ship A')

# # Add arrow for Ownship
# plt.annotate(
#     '',
#     xy=(ownship_positions[-1, 1], ownship_positions[-1, 0]),
#     xytext=(ownship_positions[-2, 1], ownship_positions[-2, 0]),
#     arrowprops=dict(arrowstyle='->', color=ownship_line.get_color())
# )

# # Add arrow for Ship A
# plt.annotate(
#     '',
#     xy=(ship_positions[-1, 1], ship_positions[-1, 0]),
#     xytext=(ship_positions[-2, 1], ship_positions[-2, 0]),
#     arrowprops=dict(arrowstyle='->', color=ship_line.get_color())
# )

# plt.xlabel('East (m)')
# plt.ylabel('North (m)')
# plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
# plt.legend()
# plt.grid(True)
# plt.axis('equal')  # Ensure the aspect ratio is equal


plt.show()