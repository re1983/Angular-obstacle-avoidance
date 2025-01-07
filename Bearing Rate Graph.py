import numpy as np
import matplotlib.pyplot as plt

class ShipStatus:
    def __init__(self, name, speed, acceleration, heading, Rate_of_Turn, position):
        self.name = name
        self.speed = speed
        self.acceleration = acceleration
        self.heading = heading
        self.Rate_of_Turn = Rate_of_Turn
        self.position = np.array(position, dtype=float)

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
    angle_to_ship2 = np.degrees(np.arctan2(delta_pos[1], delta_pos[0]))
    relative_bearing = (angle_to_ship2 - ship1.heading) % 360
    return relative_bearing - 90

# Example usage
ownship = ShipStatus("Ownship", 0.0, 0, 0, 0.0, [0, 0, 0])
ship = ShipStatus("Ship A", 10.0, 0, 90.0, 0.0, [50, -50, 0])

# print(ship.get_status())
# print(ownship.get_status())
# ship.update(1.0)
# ownship.update(1.0)
# print(ship.get_status())
# print(ownship.get_status())

# distance = get_distance_3d(ownship.position, ship.position)
# print("Distance to Ship A:", distance)
# bearing = get_bearing(ownship, ship)
# print("Bearing to Ship A:", bearing)

# Simulation parameters
time_steps = 100
delta_time = 0.1

# Lists to store positions for plotting
ownship_positions = []
ship_positions = []

# Simulation loop
for _ in range(time_steps):
    ownship.update(delta_time)
    ship.update(delta_time)
    ownship_positions.append(ownship.position.copy())
    ship_positions.append(ship.position.copy())

# Convert positions to numpy arrays for easier plotting
ownship_positions = np.array(ownship_positions)
ship_positions = np.array(ship_positions)

# Plotting
# plt.figure(figsize=(10, 6))
# plt.plot(ownship_positions[:, 0], ownship_positions[:, 1], label='Ownship')
# plt.plot(ship_positions[:, 0], ship_positions[:, 1], label='Ship A')
# plt.xlabel('North (m)')
# plt.ylabel('East (m)')
# plt.title('Ship Positions Over Time in NED Coordinates')
# plt.legend()
# plt.grid(True)
plt.show()
# Rotate the plot by 90 degrees
plt.figure(figsize=(10, 6))
plt.plot(ownship_positions[:, 1], ownship_positions[:, 0], label='Ownship')
plt.plot(ship_positions[:, 1], ship_positions[:, 0], label='Ship A')
plt.xlabel('East (m)')
plt.ylabel('North (m)')
plt.title('Ship Positions Over Time in NED Coordinates (Rotated 90 Degrees)')
plt.legend()
plt.grid(True)
plt.show()

# Plotting the bearing rate
bearings = []

# Calculate bearings over time
for i in range(time_steps):
    print(ship.get_status())
    print(ownship.get_status())
    bearing = get_bearing(ownship, ship)
    print("Bearing to Ship A:", bearing)
    bearings.append(bearing)
    ownship.update(delta_time)
    ship.update(delta_time)


# Convert bearings to numpy array for easier plotting
bearings = np.array(bearings)

# Plotting the bearing rate
plt.figure(figsize=(10, 6))
plt.plot(np.arange(time_steps) * delta_time, bearings, label='Bearing to Ship A')
plt.xlabel('Time (s)')
plt.ylabel('Bearing (degrees)')
plt.title('Bearing to Ship A Over Time')
plt.legend()
plt.grid(True)
plt.show()

