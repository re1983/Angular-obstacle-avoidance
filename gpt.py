import numpy as np
import matplotlib.pyplot as plt

def arctan2_angle(y, x):
    return np.arctan2(y, x)

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def compute_bearing(p_A, p_B):
    r = p_B - p_A
    return np.arctan2(r[1], r[0])

def compute_angular_diameter(p_A, p_B, obstacle_size):
    r = np.linalg.norm(p_B - p_A)
    return 2 * np.arctan2(obstacle_size / 2, r)

def compute_bearing_rate(p_A, v_A, p_B, v_B):
    r = p_B - p_A
    v_rel = v_B - v_A
    r2 = np.dot(r, r)
    return (r[0] * v_rel[1] - r[1] * v_rel[0]) / r2

def compute_dthetadpsi(p_A, p_B, psi, v):
    r = p_B - p_A
    r_norm2 = np.dot(r, r)
    d = r[0] * np.sin(psi) - r[1] * np.cos(psi)
    return v * d / r_norm2

# Parameters
v = 1.0  # Ownship speed (fixed)
dt = 0.1
total_time = 60
steps = int(total_time / dt)

k1 = 1.5  # goal tracking gain
k2 = 3.0  # avoidance gain
c = 5.0
delta_min = np.radians(5)  # angular diameter threshold
obstacle_size = 2.0  # meters

# Initial states
p_A = np.array([0.0, 0.0])
psi = np.radians(0.0)

p_G = np.array([30.0, 0.0])  # goal
p_B = np.array([30.0, 0.0])   # Same as goal point
v_B = np.array([-0.5, 0.0])   # Facing ownship


trajectory = []
obstacle_path = []

for t in range(steps):
    # Ownship velocity
    v_A = v * np.array([np.cos(psi), np.sin(psi)])

    # Goal tracking
    theta_G = compute_bearing(p_A, p_G)
    theta = compute_bearing(p_A, p_B)
    delta = compute_angular_diameter(p_A, p_B, obstacle_size)
    dot_theta = compute_bearing_rate(p_A, v_A, p_B, v_B)
    dtheta_dpsi = compute_dthetadpsi(p_A, p_B, psi, v)

    # Avoidance gain function
    alpha = np.tanh(c * (delta - delta_min)) if delta > delta_min else 0.0

    # Lyapunov-based control
    u_psi = -k1 * wrap_to_pi(psi - theta_G) - k2 * alpha * dot_theta * dtheta_dpsi

    # Integrate heading
    psi += u_psi * dt
    psi = wrap_to_pi(psi)

    # Update positions
    p_A = p_A + v * np.array([np.cos(psi), np.sin(psi)]) * dt
    p_B = p_B + v_B * dt

    # Record trajectory
    trajectory.append(p_A.copy())
    obstacle_path.append(p_B.copy())

    # Stop if goal is reached
    if np.linalg.norm(p_A - p_G) < 1.0:
        print(f"✅ Goal reached at t = {t*dt:.1f}s")
        break

# Visualization
trajectory = np.array(trajectory)
obstacle_path = np.array(obstacle_path)

plt.figure(figsize=(10, 6))
plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Ownship path')
plt.plot(obstacle_path[:, 0], obstacle_path[:, 1], 'r--', label='Obstacle path')
plt.plot(p_G[0], p_G[1], 'go', label='Goal')
plt.plot(obstacle_path[0, 0], obstacle_path[0, 1], 'ro', label='Obstacle start')
plt.plot(trajectory[0, 0], trajectory[0, 1], 'bo', label='Ownship start')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("CBDR Avoidance via Heading-only Lyapunov Control")
plt.show()
