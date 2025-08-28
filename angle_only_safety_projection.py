"""
Angle-only collision avoidance via bearing-rate safety projection

Purpose
- Demonstrate that with only two observables — bearing (beta) and angular diameter (alpha) —
  we can avoid CBDR and keep alpha below a chosen alpha_safe, without estimating range R.
- Controller uses a simple quadratic preference toward a desired turn and projects the control
  onto a safety set defined by |r_next| >= alpha/dt, with r_next ≈ r_meas - u. This enforces
  breaking CBDR directly from angle-only signals.

This script runs a simulation and saves plots to PNG (no GUI required).

Author: GitHub Copilot
Date: 2025-08-27
"""

import numpy as np
import math
import matplotlib
matplotlib.use('Agg')  # headless
import matplotlib.pyplot as plt


# ---------------------------- Config ----------------------------
DT = 0.01  # s
STEPS = 5000

# Turn-rate bound (deg/s)
U_MAX = 12.0

# Angle thresholds (deg)
ALPHA_NAV = 1.0   # below this, no threat -> navigate to goal
ALPHA_SAFE = 5.0  # desired safety bound for angular diameter

# Gains
K_NAV = 1.0       # proportional navigation to goal bearing
SMOOTH_W = 0.1    # smoothing weight for u_prev in quadratic preference
EPS_R = 1e-5      # near-zero bearing-rate threshold (deg/s)

# Target physical diameter (for alpha computation and verification only)
D_CONTACT = 0.5  # meters


# ---------------------------- Geometry utils ----------------------------
def wrap180(angle_deg: float) -> float:
    a = (angle_deg + 180.0) % 360.0 - 180.0
    return a


def bearing_deg(p_from: np.ndarray, p_to: np.ndarray) -> float:
    d = p_to - p_from
    theta = math.degrees(math.atan2(d[1], d[0]))
    return (theta + 360.0) % 360.0


def relative_bearing_deg(own_heading_deg: float, absolute_bearing_deg: float) -> float:
    return wrap180(absolute_bearing_deg - own_heading_deg)


def angular_diameter_deg(R: float, D: float = D_CONTACT) -> float:
    # alpha = 2 * arctan(D / (2R)) in degrees
    if R <= 1e-9:
        return 180.0  # degenerate; treat as full field
    return math.degrees(2.0 * math.atan(D / (2.0 * R)))


def distance(p1: np.ndarray, p2: np.ndarray) -> float:
    return float(np.linalg.norm(p2 - p1))


# ---------------------------- Kinematics ----------------------------
class Ship:
    def __init__(self, name: str, pos, heading_deg: float, speed_mps: float, rot_degps: float = 0.0, size: float = D_CONTACT):
        self.name = name
        self.p = np.array(pos, dtype=float)
        self.h = float(heading_deg)
        self.v = float(speed_mps)
        self.u = float(rot_degps)
        self.size = float(size)

    def step(self, dt: float = DT):
        # heading update (deg)
        self.h = (self.h + self.u * dt) % 360.0
        # position update (m)
        rad = math.radians(self.h)
        self.p += self.v * dt * np.array([math.cos(rad), math.sin(rad)])


# ---------------------------- Controller ----------------------------
def project_to_bearing_rate_safety(u_star: float, r_meas: float, alpha_deg: float, dt: float) -> float:
    """
    Safety set S: |r_next| >= alpha/dt with r_next ≈ r_meas - u.
    Forbidden band for u is (r_meas - alpha/dt, r_meas + alpha/dt).
    If u_star falls inside, snap to nearest boundary; clamp to [-U_MAX, U_MAX].
    All angles in deg, rates in deg/s.
    """
    if dt <= 0.0:
        return float(np.clip(u_star, -U_MAX, U_MAX))

    width = alpha_deg / dt
    lo = r_meas - width
    hi = r_meas + width

    u = u_star
    if lo < u < hi:
        # Snap to nearest boundary
        u = lo if abs(u - lo) <= abs(u - hi) else hi
    return float(np.clip(u, -U_MAX, U_MAX))


def angle_only_controller(beta_deg: float, alpha_deg: float, r_meas_degps: float,
                          u_prev: float, beta_goal_deg: float, dt: float) -> float:
    """
    Angle-only controller:
    - If alpha < ALPHA_NAV: navigate to goal (u_nav = K_NAV * beta_goal)
    - Else (threat region):
        * Compute a desire u_des from CBDR logic (break constant bearing, accelerate separation)
        * Smooth with previous u_prev via quadratic preference
        * Project onto safety set |r_meas - u| >= alpha/dt
    Returns bounded u in deg/s.
    """
    # Navigation when not threatened
    if alpha_deg < ALPHA_NAV:
        u_nav = K_NAV * beta_goal_deg
        return float(np.clip(u_nav, -U_MAX, U_MAX))

    # Threat region: CBDR-style desired action
    g = alpha_deg ** 2  # urgency gain
    # Default desired based on bearing-rate sign and relative geometry
    if abs(r_meas_degps) <= EPS_R:
        # Near true CBDR: force hard turn away from contact side
        u_des = -U_MAX if beta_deg < 0.0 else +U_MAX
    else:
        if abs(beta_deg) < 90.0:
            u_des = -math.copysign(g, r_meas_degps)  # target ahead
        else:
            u_des = +math.copysign(g, r_meas_degps)  # target behind

    # Quadratic preference toward u_des with smoothing to u_prev: minimize (u-u_des)^2 + w(u-u_prev)^2
    # Closed-form minimizer: (u_des + w*u_prev)/(1+w)
    u_star = (u_des + SMOOTH_W * u_prev) / (1.0 + SMOOTH_W)
    u_star = float(np.clip(u_star, -U_MAX, U_MAX))

    # Project to safety set (bearing-rate threshold)
    u_safe = project_to_bearing_rate_safety(u_star, r_meas_degps, alpha_deg, dt)
    return u_safe


# ---------------------------- Simulation ----------------------------
def run_sim():
    # Scenario similar to existing scripts
    own = Ship("Ownship", pos=[0.0, 0.0], heading_deg=90.0, speed_mps=1.0, rot_degps=0.0, size=D_CONTACT)
    tgt = Ship("Target", pos=[15.0, -15.0], heading_deg=90.0, speed_mps=2.0, rot_degps=1.0, size=D_CONTACT)
    goal = Ship("Goal", pos=[0.0, 50.0], heading_deg=0.0, speed_mps=0.0)

    # Logs
    ts = np.arange(STEPS) * DT
    dist = np.zeros(STEPS)
    alpha = np.zeros(STEPS)
    beta = np.zeros(STEPS)
    r_meas = np.zeros(STEPS)
    u_cmd = np.zeros(STEPS)
    own_h = np.zeros(STEPS)

    # init
    abs_bear_prev = None
    beta_prev = None

    for k in range(STEPS):
        # Measurements
        abs_bear = bearing_deg(own.p, tgt.p)          # theta
        beta_k = relative_bearing_deg(own.h, abs_bear)  # beta
        R = distance(own.p, tgt.p)
        alpha_k = angular_diameter_deg(R, D_CONTACT)

        # Bearing rate (relative) measurement
        if beta_prev is None:
            r_k = 0.0
        else:
            r_k = wrap180(beta_k - beta_prev) / DT

        # Goal relative bearing
        abs_goal = bearing_deg(own.p, goal.p)
        beta_goal = relative_bearing_deg(own.h, abs_goal)

        # Controller
        u = angle_only_controller(beta_k, alpha_k, r_k, own.u, beta_goal, DT)

        # Apply and step
        own.u = u
        own.step(DT)
        tgt.step(DT)

        # Log
        own_h[k] = own.h
        u_cmd[k] = u
        beta[k] = beta_k
        alpha[k] = alpha_k
        r_meas[k] = r_k
        dist[k] = R

        # Update prevs
        beta_prev = beta_k

    return {
        't': ts,
        'dist': dist,
        'alpha': alpha,
        'beta': beta,
        'r': r_meas,
        'u': u_cmd,
        'own_h': own_h,
    }


def plot_results(res, outfile='angle_only_projection_results.png'):
    t = res['t']
    fig, axes = plt.subplots(3, 2, figsize=(12, 9))

    axes[0, 0].plot(t, res['dist'], label='Range R (m)')
    axes[0, 0].set_title('True Range (verification only)')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('m')
    axes[0, 0].grid(True)

    axes[0, 1].plot(t, res['alpha'], color='orange', label='alpha (deg)')
    axes[0, 1].axhline(ALPHA_NAV, color='r', linestyle='--', label=f'alpha_nav={ALPHA_NAV}°')
    axes[0, 1].axhline(ALPHA_SAFE, color='purple', linestyle='--', label=f'alpha_safe={ALPHA_SAFE}°')
    axes[0, 1].set_title('Angular Diameter')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('deg')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].plot(t, res['beta'], color='green', label='beta (deg)')
    axes[1, 0].set_title('Relative Bearing beta')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('deg')
    axes[1, 0].grid(True)

    axes[1, 1].plot(t, res['r'], color='brown', label='r = d(beta)/dt (deg/s)')
    axes[1, 1].set_title('Measured Bearing Rate r')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('deg/s')
    axes[1, 1].grid(True)

    axes[2, 0].plot(t, res['u'], color='purple', label='u cmd (deg/s)')
    axes[2, 0].axhline(+U_MAX, color='k', linestyle='--', linewidth=0.8)
    axes[2, 0].axhline(-U_MAX, color='k', linestyle='--', linewidth=0.8)
    axes[2, 0].set_title('Turn Rate Command u')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('deg/s')
    axes[2, 0].grid(True)

    axes[2, 1].plot(t, res['own_h'], color='navy', label='heading (deg)')
    axes[2, 1].set_title('Ownship Heading')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('deg')
    axes[2, 1].grid(True)

    plt.tight_layout()
    plt.savefig(outfile, dpi=140)
    plt.close(fig)


def summarize(res):
    alpha_max = float(np.max(res['alpha']))
    alpha_over_safe = int(np.sum(res['alpha'] > ALPHA_SAFE))
    frac_below_safe = float(np.mean(res['alpha'] < ALPHA_SAFE))
    R_min = float(np.min(res['dist']))
    print(f"alpha_max = {alpha_max:.3f} deg, steps alpha>alpha_safe: {alpha_over_safe}/{len(res['alpha'])}, frac_below_safe={frac_below_safe:.3f}")
    print(f"R_min (verification only) = {R_min:.3f} m")


if __name__ == '__main__':
    res = run_sim()
    summarize(res)
    plot_results(res)
    print("Saved plot to angle_only_projection_results.png")
