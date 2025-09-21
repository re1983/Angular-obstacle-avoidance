"""
Bearing dynamics comparison for varying intruder turn rates (0–10 deg/s).

Scenario:
- Ownship: 15 m/s, straight north, starts at (0, 0).
- Intruder: 30 m/s, constant rate-of-turn ω in [0, 10] deg/s.
- Fixed collision point and time across all ω. Both reach P_c at time t_c.

Outputs:
- Trajectories (x vs y)
- NED bearing β(t) = atan2(x_i - x_o, y_i - y_o) [deg], 0°=North, clockwise+
- Bearing rate dβ/dt [deg/s]
- Bearing acceleration d²β/dt² [deg/s²]

This script is self-contained and does not modify or depend on scenario code.

Usage (customize ROT range and spacing):

    python3 bearing_rot_sweep.py --rot-start 0 --rot-stop 10 --rot-step 1

Defaults replicate [0, 10] deg/s with step 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
import argparse
from typing import Tuple, List
from pathlib import Path
import os

import numpy as np
import matplotlib
# Use non-interactive backend if no graphical display is available
if os.environ.get("DISPLAY", "") == "":
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Matplotlib defaults to avoid Type 3 fonts in PDF/EPS
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42
plt.rcParams["font.sans-serif"] = ["DejaVu Sans", "Arial", "Liberation Sans"]
plt.rcParams["text.usetex"] = False


@dataclass
class Params:
    v_own: float = 1.0  # m/s
    v_intr: float = 1.0  # m/s
    t_c: float = 30.0  # s, collision time
    dt: float = 0.01  # s, sampling interval
    psi_c_deg: float = 180.0  # intruder heading at collision [deg] (east=0, north=90)

    # Collision point determined by ownship travel to time t_c (northbound along y-axis)
    @property
    def P_c(self) -> Tuple[float, float]:
        return (0.0, self.v_own * self.t_c)


def ownship_state(params: Params, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    x_o = np.zeros_like(t)
    y_o = params.v_own * t
    return x_o, y_o


def intruder_state_constant_turn(params: Params, omega_deg_s: float, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute intruder trajectory with constant speed and turn rate such that it reaches
    the fixed collision point P_c at time t_c with heading psi_c.

    Coordinate convention:
    - x: East, y: North
    - Heading psi: 0 deg = East, 90 deg = North, increases CCW
    - omega > 0 means CCW (left) turn
    """
    v = params.v_intr
    t_c = params.t_c
    Pcx, Pcy = params.P_c
    psi_c = math.radians(params.psi_c_deg)

    if abs(omega_deg_s) < 1e-9:
        # Straight-line case: choose initial point so that after t_c at speed v along heading psi_c we reach P_c
        # For straight line, heading is constant psi0 = psi_c (since omega=0), so start position P0 = P_c - v*t_c*[cos psi0, sin psi0]
        psi0 = psi_c
        x0 = Pcx - v * t_c * math.cos(psi0)
        y0 = Pcy - v * t_c * math.sin(psi0)
        x = x0 + v * t * np.cos(psi0)
        y = y0 + v * t * np.sin(psi0)
        return x, y

    omega = math.radians(omega_deg_s)  # rad/s
    R = v / omega  # signed radius
    psi0 = psi_c - omega * t_c  # starting heading so that psi(t_c) = psi_c

    # Using standard unicycle model integration
    # x(t) = x0 + (v/omega)[sin(psi0 + omega t) - sin(psi0)]
    # y(t) = y0 - (v/omega)[cos(psi0 + omega t) - cos(psi0)]
    # Enforce x(t_c)=Pcx, y(t_c)=Pcy to determine (x0, y0)
    sin_psic = math.sin(psi_c)
    sin_psi0 = math.sin(psi0)
    cos_psic = math.cos(psi_c)
    cos_psi0 = math.cos(psi0)
    x0 = Pcx - (v / omega) * (sin_psic - sin_psi0)
    y0 = Pcy + (v / omega) * (cos_psic - cos_psi0)

    psi_t = psi0 + omega * t
    x = x0 + (v / omega) * (np.sin(psi_t) - sin_psi0)
    y = y0 - (v / omega) * (np.cos(psi_t) - cos_psi0)
    return x, y


def compute_bearing_series(xi: np.ndarray, yi: np.ndarray, xo: np.ndarray, yo: np.ndarray, dt: float):
    dx = xi - xo
    dy = yi - yo
    # NED bearing: 0° = North, clockwise positive
    beta_rad = np.arctan2(dx, dy)
    beta_unwrapped = np.unwrap(beta_rad)
    beta_deg = np.degrees(beta_unwrapped)

    # First derivative (deg/s)
    dbeta_dt_deg = np.gradient(beta_deg, dt)
    # Second derivative (deg/s^2)
    d2beta_dt2_deg = np.gradient(dbeta_dt_deg, dt)
    return beta_deg, dbeta_dt_deg, d2beta_dt2_deg


def run(params: Params, omega_list_deg: List[float] | np.ndarray, save_path: str | None = None, dpi: int = 600):
    # Time grid up to slightly before collision to avoid LOS singularity
    t = np.arange(0.0, params.t_c, params.dt)
    xo, yo = ownship_state(params, t)

    cmap = plt.get_cmap("viridis")

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax_traj, ax_beta = axes[0]
    ax_betar, ax_betaa = axes[1]

    for idx, omega in enumerate(omega_list_deg):
        xi, yi = intruder_state_constant_turn(params, omega, t)
        beta_deg, dbeta_dt_deg, d2beta_dt2_deg = compute_bearing_series(xi, yi, xo, yo, params.dt)

        color = cmap(float(idx) / max(1, len(omega_list_deg) - 1))
        label_val = (f"{omega:.3f}").rstrip('0').rstrip('.')
        label = f"ROT={label_val} deg/s"

        # Trajectories
        ax_traj.plot(xo, yo, color="black", linewidth=2, label="Ownship" if idx == 0 else None)
        ax_traj.plot(xi, yi, color=color, linewidth=1.5, label=label)

        # Bearing, rate, acceleration
        ax_beta.plot(t, beta_deg, color=color, label=label)
        ax_betar.plot(t, dbeta_dt_deg, color=color, label=label)
        ax_betaa.plot(t, d2beta_dt2_deg, color=color, label=label)

    # Mark collision point and time
    Pcx, Pcy = params.P_c
    ax_traj.scatter([Pcx], [Pcy], color="red", s=40, zorder=5, label="Collision point")
    ax_traj.set_title("Trajectories (Ownship northbound, Intruder varying ROT)")
    ax_traj.set_xlabel("x [m] (East)")
    ax_traj.set_ylabel("y [m] (North)")
    ax_traj.axis("equal")
    ax_traj.grid(True, linewidth=0.6)
    ax_traj.legend(loc="best", fontsize=8, ncol=2)

    ax_beta.set_title("NED Bearing β(t) [deg] (0°=N, CW+)")
    ax_beta.set_xlabel("t [s]")
    ax_beta.set_ylabel("β [deg]")
    ax_beta.grid(True, linewidth=0.6)

    ax_betar.set_title("Bearing Rate dβ/dt [deg/s]")
    ax_betar.set_xlabel("t [s]")
    ax_betar.set_ylabel("dβ/dt [deg/s]")
    ax_betar.grid(True, linewidth=0.6)

    ax_betaa.set_title("Bearing Acceleration d²β/dt² [deg/s²]")
    ax_betaa.set_xlabel("t [s]")
    ax_betaa.set_ylabel("d²β/dt² [deg/s²]")
    ax_betaa.grid(True, linewidth=0.6)

    fig.tight_layout()
    if save_path:
        # Ensure parent directory exists
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Bearing vs ROT sweep (intruder constant turn)")
    parser.add_argument("--rot-start", type=float, default=-10.0, help="Start ROT [deg/s]")
    parser.add_argument("--rot-stop", type=float, default=10.0, help="Stop ROT [deg/s] (inclusive)")
    parser.add_argument("--rot-step", type=float, default=2.0, help="ROT step [deg/s]")
    parser.add_argument("--save", type=str, default="save", help="Save figure to path (file or directory). If directory/no extension, a filename is generated.")
    parser.add_argument("--dpi", type=int, default=300, help="DPI when saving the figure")
    args = parser.parse_args()

    if args.rot_step <= 0:
        raise ValueError("--rot-step must be > 0")

    params = Params()
    # Inclusive stop handling for arange
    omega_list = np.arange(args.rot_start, args.rot_stop + 1e-9, args.rot_step)

    save_path = None
    if args.save:
        out = Path(args.save)
        # Determine if a filename with extension was provided
        is_file = out.suffix.lower() in (".png", ".pdf", ".svg")
        if not is_file:
            # Treat as directory or filename without extension -> create directory and generate name
            out_dir = out if out.suffix == "" else out.parent
            out_dir.mkdir(parents=True, exist_ok=True)
            def fmt(v: float) -> str:
                s = f"{v:.3f}".rstrip('0').rstrip('.')
                return s if s else "0"
            fname = f"bearing_rot_sweep_rot_{fmt(args.rot_start)}-{fmt(args.rot_stop)}_step_{fmt(args.rot_step)}.png"
            out = out_dir / fname
        save_path = str(out)

    run(params, omega_list, save_path=save_path, dpi=args.dpi)


if __name__ == "__main__":
    main()
