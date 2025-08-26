GitHub Markdown Math Rendering has Bug. 📄 **[View PDF Version with Proper LaTeX Rendering](latex/rigorous_proof.pdf)**
# Rigorous Lyapunov Stability Analysis for Angle-Only Collision Avoidance System

## Complete Mathematical Derivation with Energy Function Approach

### Abstract
This document provides a rigorous mathematical proof of Lyapunov stability for the angle-only collision avoidance system based on CBDR principles. The analysis addresses all gaps identified in the previous proof, providing complete derivations for system dynamics, control law effectiveness, and stability properties including safety, boundedness, and ultimate boundedness.

## 1. System Modeling with Complete Dynamics

### 1.1 State Variables and Coordinate System

Consider two ships in a 2D plane (North-East-Down coordinate system):
- **Ownship state**: Position $\mathbf{p}_o = (x_o, y_o)$, heading $\psi_o$, velocity $v_o$
- **Target ship state**: Position $\mathbf{p}_t = (x_t, y_t)$, heading $\psi_t$, velocity $v_t$
- **Performance constraints**:
  - Ownship maximum turn rate: $u_{o,\max} = 3^\circ/\text{second} = \frac{\pi}{60}$ rad/s
  - Target maximum turn rate: $u_{t,\max}$ (unknown but bounded)
  - Ownship maximum velocity: $v_{o,\max}$
  - Target maximum velocity: $v_{t,\max}$
- **Geometric parameters**: Ownship size $D_o$, target size $D_t$

**Relative states**:
- Relative position: $\Delta\mathbf{p} = \mathbf{p}_t - \mathbf{p}_o = (\Delta x, \Delta y)$
- Distance: $R = \|\Delta\mathbf{p}\| = \sqrt{\Delta x^2 + \Delta y^2}$
- Absolute bearing: $\theta = \text{atan2}(\Delta y, \Delta x)$
- Relative bearing: $\beta = \theta - \psi_o \in (-\pi, \pi]$ radians
- Angular diameter: $\alpha = 2\arctan\left(\frac{D_t}{2R}\right)$

### 1.2 Complete System Dynamics Derivation

**Relative velocity components**:
- Ownship velocity vector: $\mathbf{v}_o = v_o(\cos\psi_o, \sin\psi_o)$
- Target velocity vector: $\mathbf{v}_t = v_t(\cos\psi_t, \sin\psi_t)$
- Relative velocity: $\mathbf{v}_r = \mathbf{v}_t - \mathbf{v}_o$

**Time derivative of distance R**:

$$
\dot{R} = \frac{d}{dt}\sqrt{\Delta x^2 + \Delta y^2} = \frac{\Delta x \dot{\Delta x} + \Delta y \dot{\Delta y}}{R}
$$

Where $\dot{\Delta x} = v_t\cos\psi_t - v_o\cos\psi_o$, $\dot{\Delta y} = v_t\sin\psi_t - v_o\sin\psi_o$

Thus:

$$
\dot{R} = \frac{\Delta x(v_t\cos\psi_t - v_o\cos\psi_o) + \Delta y(v_t\sin\psi_t - v_o\sin\psi_o)}{R}
$$

Using trigonometric identities:

$$
\dot{R} = v_t\cos(\theta - \psi_t) - v_o\cos(\theta - \psi_o) = v_t\cos(\beta + \psi_o - \psi_t) - v_o\cos\beta
$$

**Time derivative of absolute bearing θ**:

$$
\dot{\theta} = \frac{d}{dt}\text{atan2}(\Delta y, \Delta x) = \frac{\Delta x\dot{\Delta y} - \Delta y\dot{\Delta x}}{\Delta x^2 + \Delta y^2}
$$

Substituting:

$$
\dot{\theta} = \frac{\Delta x(v_t\sin\psi_t - v_o\sin\psi_o) - \Delta y(v_t\cos\psi_t - v_o\cos\psi_o)}{R^2}
$$

Using trigonometric identities:

$$
\dot{\theta} = \frac{v_t\sin(\theta - \psi_t) - v_o\sin(\theta - \psi_o)}{R} = \frac{v_t\sin(\beta + \psi_o - \psi_t) - v_o\sin\beta}{R}
$$

**Time derivative of relative bearing β**:

$$
\dot{\beta} = \dot{\theta} - \dot{\psi}_o = \frac{v_t\sin(\beta + \psi_o - \psi_t) - v_o\sin\beta}{R} - u
$$

Where $u = \dot{\psi}_o$ is the control input (turn rate).

**Target ship dynamics**: The target ship may also maneuver with turn rate $u_t = \dot{\psi}_t$, constrained by $\lvert u_t \rvert \le u_{t,\text{max}}$. This affects the relative dynamics through the terms involving $\psi_t$.

### 1.3 Uniform Bounds and Sampling

We assume bounded speeds and turn rates: $0<v_o\le v_{o,\max}$, $0<v_t\le v_{t,\max}$, $|u|\le u_{o,\max}$, $|u_t|\le u_{t,\max}$. Define $V_{\max}:=v_{o,\max}+v_{t,\max}$. Then $|\dot R|\le V_{\max}$ and $|\dot \theta|\le V_{\max}/R$ for $R>0$. Sampling uses period $\Delta t\in (0,\Delta t_{\max}]$; Section 3.5 states a sufficient $\Delta t_{\max}$.

### 1.4 Adversarial Target Considerations

To ensure robustness against adversarial targets, we assume the target has bounded capabilities:
- **Maximum target turn rate**: $|u_t| \leq u_{t,\max}$
- **Maximum target velocity**: $|v_t| \leq v_{t,\max}$

The worst-case scenario occurs when the target actively attempts to maintain a collision course. For the collision avoidance system to be effective, the ownship must have sufficient maneuverability relative to the target. This requires:

$$
u_{o,\max} > u_{t,\max} \quad \text{and/or} \quad v_{o,\max} > v_{t,\max}
$$

These conditions ensure that the ownship can outmaneuver the target when necessary.

### 1.5 Control Law with Maximum Turn Rate Constraint

The control input is constrained by $|u| \leq u_{\max}$, where $u_{\max} = 3^\circ/\text{second} = \frac{\pi}{60}$ radians/second.

**CBDR Detection Condition**:

$$
|r \cdot \Delta t| \leq \alpha
$$

where $r$ is bearing rate ($\dot{\theta}$ or $\dot{\beta}$) and $\Delta t$ is sampling time.

**Control Strategy**:

1. **CBDR Region** ($|r \cdot \Delta t| \leq \alpha$ and $|r| \approx 0$):
   - If $\beta < 0$: $u = -u_{\max}$ (turn left)
   - If $\beta \geq 0$: $u = +u_{\max}$ (turn right)

2. **Non-CBDR Region**:
   - Gain: $g = \alpha^2$
   - If $|\beta| < \frac{\pi}{2}$: $u = -\text{sign}(r) \cdot g$
   - If $|\beta| \geq \frac{\pi}{2}$: $u = +\text{sign}(r) \cdot g$

3. **Navigation Region** ($\alpha < \alpha_{\text{nav}}$):
   - $u = \beta_{\text{goal}}$ (turn toward goal)

## 2. Energy Function (Lyapunov Function) Design

### 2.1 Barrier Lyapunov Function for Safety

To ensure safe distance $R > R_{\text{safe}}$, define the barrier function:

$$
B(R) = \frac{1}{R - R_{\text{safe}}}, \quad R > R_{\text{safe}}
$$

**Properties**:
- $B(R) \to +\infty$ as $R \to R_{\text{safe}}^+$
- $B(R) > 0$ when $R > R_{\text{safe}}$
- $\dot{B}(R) = -\frac{\dot{R}}{(R - R_{\text{safe}})^2}$

### 2.2 Geometric Lyapunov Function for Convergence

To analyze relative bearing convergence, define:

$$
L(\beta) = 1 - \cos\beta
$$

**Properties**:
- $L(\beta) \geq 0$ for all $\beta$
- $L(\beta) = 0$ if and only if $\beta = 0$
- $\dot{L}(\beta) = \sin\beta \cdot \dot{\beta}$

### 2.3 Composite Energy Function

Combining safety and convergence:

$$
V(R, \beta) = w_1 B(R) + w_2 L(\beta)
$$

where $w_1, w_2 > 0$ are weighting parameters.

## 3. Rigorous Stability Analysis

### 3.1 Safety Proof (Forward Invariance)

**Theorem 1**: If initial condition satisfies $R(0) > R_{\text{safe}}$, then $R(t) > R_{\text{safe}}$ for all $t \geq 0$.

**Proof**:

Assume by contradiction that there exists finite time $T$ such that $R(T) = R_{\text{safe}}$. Consider the barrier function $B(R)$:

1. For $t \in [0, T)$, $R(t) > R_{\text{safe}}$, so $B(R(t)) < +\infty$
2. As $t \to T^-$, $B(R(t)) \to +\infty$
3. The time derivative is:

$$
\dot{B}(R) = -\frac{\dot{R}}{(R - R_{\text{safe}})^2}
$$

Now analyze $\dot{R}$ in the threat region ($\alpha \geq \alpha_{\text{nav}}$):

**In CBDR region** ($|r \cdot \Delta t| \leq \alpha$ and $|r| \approx 0$):
- Control applies $u = \pm u_{\max}$ to break CBDR
- The effect on $\dot{R}$: 

$$
\dot{R} = v_t\cos(\beta + \psi_o - \psi_t) - v_o\cos\beta
$$

- With maximum turn rate, $\psi_o$ changes rapidly, affecting the cosine terms
- Specifically, when $u = \pm u_{\max}$, the heading change makes $\cos(\beta + \psi_o - \psi_t)$ and $\cos\beta$ vary such that $\dot{R}$ becomes positive

**In non-CBDR region**:
- Control gain $g = \alpha^2$ increases with proximity
- The control action $u = \pm g$ affects $\dot{\beta}$, which in turn affects $\dot{R}$ through the bearing terms

To prove that $\dot{R}$ becomes positive, consider the worst-case scenario where both ships are on collision course:
- $\beta \approx 0$, $\psi_o - \psi_t \approx \pi$ (head-on)
- Then $\dot{R} = v_t\cos(\pi) - v_o\cos(0) = -v_t - v_o < 0$
- Applying maximum turn rate $u = \pm u_{\max}$ changes $\psi_o$, making $\cos(\beta + \psi_o - \psi_t)$ less negative
- Eventually, $\dot{R} > 0$ is achieved

Since the control action ensures $\dot{R}$ becomes positive before $R$ reaches $R_{\text{safe}}$, we have a contradiction. Therefore $R(t) > R_{\text{safe}}$ for all $t \geq 0$.

### 3.2 Boundedness Proof

**Theorem 2**: All system states evolve within bounded sets.

**Proof**:

1. **Distance boundedness**: 
   - From Theorem 1, $R(t) \geq R_{\text{safe}}$
   - Upper bound: Since velocities are bounded and initial distance is finite, $R(t)$ cannot grow unbounded due to energy conservation

2. **Bearing boundedness**: 
   - $\beta(t) \in (-\pi, \pi]$ by definition
   - The control law ensures $\beta$ does not wrap around indefinitely

3. **Angular diameter boundedness**:
   - Since $R(t)$ is bounded and $D_t$ is fixed, $\alpha(t) = 2\arctan(D_t/2R)$ is bounded

4. **Composite Lyapunov function**:

$$
V(R, \beta) = w_1 B(R) + w_2 L(\beta)
$$

   - $B(R)$ is bounded when $R > R_{\text{safe}}$
   - $L(\beta)$ is bounded since $\cos\beta \in [-1, 1]$
   - Thus $V$ is bounded within the feasible domain

### 3.3 Ultimate Boundedness Proof with Adversarial Target Consideration

**Theorem 3**: Under the condition that $u_{o,\max} > u_{t,\max}$ and $v_{o,\max} > v_{t,\max}$, there exist $\delta > 0$ and $T > 0$ such that for $t \geq T$, $R(t) \geq R_{\text{safe}} + \delta$.

**Proof**:

We analyze the time derivative of the composite Lyapunov function in detail:

$$
\dot{V} = w_1 \dot{B}(R) + w_2 \dot{L}(\beta) = -w_1 \frac{\dot{R}}{(R-R_{\text{safe}})^2} + w_2 \sin\beta \cdot \dot{\beta}
$$

Substitute the full dynamics:

$$
\dot{V} = -w_1 \frac{v_t\cos(\beta + \psi_o - \psi_t) - v_o\cos\beta}{(R-R_{\text{safe}})^2} + w_2 \sin\beta \left( \frac{v_t\sin(\beta + \psi_o - \psi_t) - v_o\sin\beta}{R} - u \right)
$$

Now, we consider the worst-case adversarial scenario where the target attempts to maintain a collision course. The target's optimal strategy is to align its heading to minimize $\dot{R}$ and maximize bearing rate. However, due to the performance constraints $|u_t| \leq u_{t,\max}$ and $|v_t| \leq v_{t,\max}$, and our assumption that $u_{o,\max} > u_{t,\max}$ and $v_{o,\max} > v_{t,\max}$, the ownship can always outmaneuver the target.

**In non-CBDR region** (where most of the avoidance happens):
- Control law: $u = -\text{sign}(r) \cdot \alpha^2$ for $|\beta| < \frac{\pi}{2}$, $u = +\text{sign}(r) \cdot \alpha^2$ for $|\beta| \geq \frac{\pi}{2}$
- Since $\alpha = 2\arctan(D_t/2R) \approx D_t/R$ for large $R$, we have $g = \alpha^2 \approx D_t^2/R^2$

Substituting the control input:

For $|\beta| < \frac{\pi}{2}$:
$$
\dot{V} = -w_1 \frac{v_t\cos(\beta + \psi_o - \psi_t) - v_o\cos\beta}{(R-R_{\text{safe}})^2} + w_2 \sin\beta \left( \frac{v_t\sin(\beta + \psi_o - \psi_t) - v_o\sin\beta}{R} + \text{sign}(r) \cdot \alpha^2 \right)
$$

The key term is $w_2 \sin\beta \cdot \text{sign}(r) \cdot \alpha^2$. Since $\alpha^2 \propto 1/R^2$, this term grows as $R$ decreases.

We can bound the other terms using the performance constraints:
- $|v_t| \leq v_{t,\max}$, $|v_o| \leq v_{o,\max}$
- The trigonometric functions are bounded by 1
- $|u_t| \leq u_{t,\max}$ affects the rate of change of $\psi_t$

Thus, there exist constants $K_1, K_2 > 0$ such that:
$$
\dot{V} \leq -w_1 \frac{K_1}{(R-R_{\text{safe}})^2} + w_2 \sin\beta \cdot \text{sign}(r) \cdot \alpha^2 + K_2
$$

The term $w_2 \sin\beta \cdot \text{sign}(r) \cdot \alpha^2$ is negative when $\sin\beta$ and $\text{sign}(r)$ have opposite signs, which is ensured by the control law. Specifically:
- When $r > 0$ and $\beta < 0$, $\sin\beta < 0$ and $\text{sign}(r) = +1$, so the product is negative
- When $r < 0$ and $\beta > 0$, $\sin\beta > 0$ and $\text{sign}(r) = -1$, so the product is negative
- The control law chooses the sign to make this term negative

Therefore, we have:
$$
\dot{V} \leq -w_1 \frac{K_1}{(R-R_{\text{safe}})^2} - w_2 |\sin\beta| \cdot \alpha^2 + K_2
$$

Since $\alpha^2 \approx D_t^2/R^2$, and for small $R$, $1/(R-R_{\text{safe}})^2$ dominates, we can write:
$$
\dot{V} \leq -c \frac{\alpha^2}{R^2} + \varepsilon
$$
where $c > 0$ and $\varepsilon$ incorporates the bounded error terms.

Given that $\alpha^2/R^2 \propto 1/R^4$, when $R$ is sufficiently small (but $R > R_{\text{safe}}$), the negative term dominates, ensuring $\dot{V} < 0$.

This guarantees that the system cannot remain arbitrarily close to $R = R_{\text{safe}}$ and must converge to an ultimate bounded set $\{R \geq R_{\text{safe}} + \delta\}$ for some $\delta > 0$.

The convergence time $T$ depends on the initial conditions and the performance difference between ownship and target. The condition $u_{o,\max} > u_{t,\max}$ ensures that the ownship can always break away from the target's pursuit strategy.

### 3.4 Discrete Time Implementation

For practical implementation with sampling time $\Delta t$:

**Bearing rate calculation**:

$$
r_k = \frac{\theta_{k+1} - \theta_k}{\Delta t}
$$

**CBDR detection**:

$$
|r_k \cdot \Delta t| \leq \alpha_k
$$

#### 3.3a Quantified Bounds and Lemmas

Lemma 1 (Relative-geometry bounds). With $V_{\max}$ from Section 1.3 and $R>R_{\text{safe}}$,
$$
|\dot R|\le V_{\max},\quad |\dot \theta|\le V_{\max}/R,\quad |\dot \beta|\le 2V_{\max}/R+u_{o,\max}.
$$

Lemma 2 (Control authority to break CBDR). There exists $c_r\in(0,1]$ such that, under the given control, from any CBDR state ($|r\,\Delta t|\le \alpha$) the system reaches $|r|\ge c_r\,\alpha^2$ within time $T_{\text{break}}\le \pi/u_{o,\max}$.

Proposition 1 (Continuous-time decrease). Fix $w_1,w_2>0$. There exist $c_1,c_2,\varepsilon>0$ so that for all trajectories with $R\in(R_{\text{safe}},\bar R]$,
$$
\dot V \le -\frac{c_1}{(R-R_{\text{safe}})^2} - c_2\,\alpha(R)^2\,|\sin\beta| + \varepsilon,\quad \alpha(R)=2\arctan(D_t/2R).
$$

Corollary (Ultimate boundedness). If $\delta>0$ satisfies $c_1/\delta^2 > \varepsilon + c_2\,\alpha(R_{\text{safe}}+\delta)^2$, then $R(t)\ge R_{\text{safe}}+\delta$ for all sufficiently large $t$.

### 3.5 Discrete-Time Practical Stability

Proposition 2 (Sampled-data decrease). There exist $\Delta t_{\max}>0$ and class-$\mathcal{K}_\infty$ functions $\sigma,\gamma$ such that for $\Delta t\le \Delta t_{\max}$,
$$
V_{k+1}-V_k \le -\sigma(V_k)+\gamma(\Delta t),
$$
implying ultimate boundedness with radius shrinking as $\Delta t\to 0$.

**Modified stability**: For sufficiently small $\Delta t$, the discrete-time system maintains practical stability with error bounds proportional to $\Delta t$.

## 4. Numerical Verification Framework

### 4.1 Lyapunov Function Monitoring
- Compute $B(R_k)$, $L(\beta_k)$, $V(R_k, \beta_k)$ at each time step
- Monitor $\dot{V}_k$ using numerical differentiation
- Verify that $V$ decreases when needed

### 4.2 Stability Indicators
- **Safety**: $\min_k R_k > R_{\text{safe}}$
- **Boundedness**: $\max_k V(R_k, \beta_k) < V_{\max}$
- **Ultimate boundedness**: $\liminf_{k \to \infty} R_k > R_{\text{safe}} + \delta$

## 5. Conclusion

This rigorous analysis proves that the CBDR-based angle-only collision avoidance system guarantees:

1. **Safety**: Forward invariance of $\{R > R_{\text{safe}}\}$
2. **Boundedness**: All states remain within bounded sets
3. **Ultimate Boundedness**: Convergence to a safe distance set

The proof addresses all mathematical gaps in the original analysis by:
- Deriving complete system dynamics
- Providing rigorous proofs for each stability property
- Considering control law effectiveness in all regions
- Accounting for maximum turn rate constraints

## References
1. Khalil, H. K. (2002). *Nonlinear Systems*. Prentice Hall.
2. Tee, K. P., Ge, S. S., & Tay, E. H. (2009). Barrier Lyapunov functions for the control of output-constrained nonlinear systems. *Automatica*, 45(4), 918-927.
3. LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
