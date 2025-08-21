# Angle-Only Collision Avoidance via CBDR using Bearing and Angular Diameter

## Abstract

This note formalizes a simple, angle-only collision avoidance method for two-ship encounters. The controller relies solely on two angular observables: (i) the line-of-sight (LOS) bearing to the contact, and (ii) the apparent angular diameter of the contact (assuming a known physical size). The objective is to avoid Constant Bearing, Decreasing Range (CBDR) situations, which are classical indicators of collision risk. A waterfall-style bearing-versus-time visualization is used to support operator insight, inspired by submarine sonar displays.

## Coordinate frame and notation

- NED frame: North (x), East (y), Down (z). Motion is planar (z = 0).
- Ownship pose: heading (yaw) $\psi$ in degrees, position $\mathbf{p}_o = (x_o, y_o)$.
- Contact pose: position $\mathbf{p}_c = (x_c, y_c)$, physical size (diameter) $D$.
- Relative displacement: $\Delta\mathbf{p} = \mathbf{p}_c - \mathbf{p}_o$.

Absolute (true) bearing from ownship to the contact is

$$
\theta = \mathrm{atan2}(\Delta y, \Delta x) \;\;[\deg] \quad \text{wrapped to } [0,360) .
$$

Relative bearing (starboard-positive, port-negative) is

$$
\beta = ((\theta - \psi) \bmod 360) \in (-180, 180] .
$$

Euclidean range is $R = \|\Delta\mathbf{p}\|$. If the contact's apparent angular diameter is $\alpha$ and its physical diameter is $D$, then

$$
\alpha = 2\arctan\left(\frac{D}{2R}\right) \quad \Rightarrow \quad R = \frac{D}{2\tan(\alpha/2)} .
$$

In angle-only control, $R$ is not directly used by the controller; $\alpha$ is used as a proxy for encounter urgency.

## CBDR principle

A necessary condition for collision in straight-line motion is approximately constant LOS bearing with decreasing range:

$$
\dot{\theta} \approx 0 \quad \text{and} \quad \dot{R} < 0 .
$$

In discrete time with step $\Delta t$, the controller monitors a bearing-rate surrogate

$$
\dot{\theta} \;\approx\; \frac{\theta_{k+1} - \theta_k}{\Delta t}, \qquad
\dot{\beta} \;\approx\; \frac{\beta_{k+1} - \beta_k}{\Delta t} .
$$

To add robustness, the decision compares the magnitude of bearing change over one step to the current angular diameter:

$$
|\dot{\theta}|\,\Delta t \;\le\; \alpha \quad \text{or} \quad |\dot{\beta}|\,\Delta t \;\le\; \alpha .
$$

## Control policies

Two controllers share identical kinematics and differ only by which bearing-rate they evaluate.

### Kinematics (discrete time)
Heading and position update with step $\Delta t$:

$$
\psi_{k+1} = \psi_k + u_k\,\Delta t, \quad
\mathbf{p}_{o,k+1} = \mathbf{p}_{o,k} + v_k\,\Delta t
\begin{bmatrix}
\cos(\psi_k\,\pi/180)\\[2pt]
\sin(\psi_k\,\pi/180)
\end{bmatrix},
$$

where $u_k$ is rate-of-turn (deg/s) saturated by $u_{\text{max}}$, and $v_k$ the forward speed (m/s).

### Absolute-bearing controller (CBDR by $\dot{\theta}$)

Let $\dot{\theta}_k$ be the absolute bearing rate. Define an urgency gain $g_k = \alpha_k^2$. With thresholding $|\dot{\theta}_k|\,\Delta t \le \alpha_k$:

- If approximately CBDR ($|\dot{\theta}_k|$ very small), turn away from the contact's relative side:
  $$
    u_k = \begin{cases}
    -u_{\text{max}}, & \beta_k < 0 \quad (\text{contact on port})\\
    +u_{\text{max}}, & \beta_k \ge 0 \quad (\text{contact on starboard})
    \end{cases}
  $$
- Else, accelerate separation using the sign of $\dot{\theta}_k$:
  $$
  u_k = \begin{cases}
  -\text{sgn}(\dot{\theta}_k) \cdot g_k, & |\beta_k| < 90^\circ \\ 
  +\text{sgn}(\dot{\theta}_k) \cdot g_k, & |\beta_k| \ge 90^\circ
  \end{cases}
  $$
- If $\alpha_k < \alpha_{\text{nav}}$ (no threat), steer toward goal bearing $\beta^{\text{goal}}_k$: $u_k = \beta^{\text{goal}}_k$.

### Relative-bearing controller (CBDR by $\dot{\beta}$)

Same structure, but replace $\dot{\theta}$ with $\dot{\beta}$: threshold on $|\dot{\beta}_k|\,\Delta t \le \alpha_k$ and use $\text{sgn}(\dot{\beta}_k)$ in the non-CBDR branch.

## Visualization

- Waterfall plot (bearing vs. time) shows LOS evolution. A vertical line (constant bearing) with increasing angular size signals CBDR.
- Multi-panel comparison: 2×8 grid
  1. Plan view tracks with 10 s markers and direction arrows (north, heading, LOS-to-contact).
  2. Bearing waterfall (relative and absolute).
  3. Angular size with maximum annotated.
  4. Distance with minimum annotated (for analysis only).
  5. Relative bearing vs. time.
  6. Velocities.
  7. Ownship heading.
  8. Absolute bearing vs. time.

## Implementation overview

- Absolute-bearing controller: `BearingRateGraph_cleaned.py`
- Relative-bearing controller: `BearingRateGraph_cleaned_relative_bearing.py`
- Side-by-side comparison (both rows): `BearingRateGraph_comparison.py`

Discretization uses $\Delta t = 0.01\,\text{s}$ and typical simulation length of 5000 steps. The urgency gain uses $g_k = \alpha_k^2$ to increase turn rate as the contact's apparent size grows. The navigation threshold is set to $\alpha_{\text{nav}} = 2.0°$ (code constant: `ALPHA_NAV`) below which collision avoidance is not needed.

### Pseudo-code (controller core)

```text
if cbd_threshold_met:  # |bearing_rate| * dt <= alpha
    if abs(bearing_rate) <= 1e-5:  # True CBDR (bearing rate ≈ 0)
        if beta < 0:   u = -u_max    # turn left (port)
        else:          u = +u_max    # turn right (starboard)
    else:
        if abs(beta) < 90°:  u = -sign(bearing_rate) * alpha^2
        else:                u =  sign(bearing_rate) * alpha^2

if alpha < alpha_nav:  # no threat (alpha_nav = 2.0°, code: ALPHA_NAV)
    u = goal_relative_bearing
u = clamp(u, -u_max, u_max)
```

## Assumptions and limitations

- Target size $D$ is known to interpret $\alpha$ physically; the controller itself only uses $\alpha$ as an urgency proxy.
- Ownship dynamics are kinematic (no sway/surge dynamics or inertia).
- Bearing unwrapping and angle normalization are handled to avoid numerical spikes.
- Sensor noise and contact maneuvers are not modeled here; thresholds may require tuning in practice.

## How to run

- Absolute-bearing controller:
  ```bash
  python3 BearingRateGraph_cleaned.py
  ```
- Relative-bearing controller:
  ```bash
  python3 BearingRateGraph_cleaned_relative_bearing.py
  ```
- Side-by-side comparison (2×8 panels):
  ```bash
  python3 BearingRateGraph_comparison.py
  ```

## Math in GitHub README

GitHub renders LaTeX math in Markdown. Use inline math with `$ ... $` and display math with `$$ ... $$`. For multi-line alignment, use environments like `\begin{aligned} ... \end{aligned}` inside `$$` blocks. As an alternative for environments without MathJax (e.g., some PDF exports), pre-render equations to SVG/PNG or use KaTeX to statically render HTML.

---

© This document summarizes the methodology implemented in the repository for angle-only collision avoidance based on CBDR.
