import numpy as np
import matplotlib.pyplot as plt

# Example data: time and corresponding bearing values in degrees
time = np.linspace(0, 10, 100)
bearing = 0 + 10 * np.sin(time)

# Compute bearing rate (derivative of bearing with respect to time)
dt = time[1] - time[0]
bearing_rate = np.gradient(bearing, dt)

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, bearing, label='Bearing (deg)')
plt.xlabel('Time (s)')
plt.ylabel('Bearing (deg)')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, bearing_rate, label='Bearing Rate (deg/s)', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Bearing Rate (deg/s)')
plt.legend()

plt.tight_layout()
plt.show()