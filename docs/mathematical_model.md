# Mathematical Model: Self-Balancing Two-Wheel Robot

## Table of Contents
1. [Introduction](#introduction)
2. [Coordinate Frame Conventions](#coordinate-frame-conventions)
3. [System Variables](#system-variables)
4. [Lagrangian Derivation](#lagrangian-derivation)
5. [Nonlinear Equations of Motion](#nonlinear-equations-of-motion)
6. [Linearisation](#linearisation)
7. [State-Space Representation](#state-space-representation)
8. [Numeric Values from Configuration](#numeric-values-from-configuration)
9. [Eigenvalue Analysis](#eigenvalue-analysis)
10. [Controllability Analysis](#controllability-analysis)
11. [Model Simplifications and Validity Range](#model-simplifications-and-validity-range)
12. [Linear Model vs. PyBullet Mismatch Sources](#linear-model-vs-pybullet-mismatch-sources)

---

## 1. Introduction

The self-balancing two-wheel robot is a canonical under-actuated mechanical system — it is an inverted pendulum mounted on a wheeled cart. The system is inherently unstable: without active control, gravity will topple the chassis. The control challenge is to apply wheel torques that simultaneously:

1. Keep the chassis upright ($\theta \approx 0$), and
2. Drive the robot to a desired horizontal position or velocity.

This document derives the full nonlinear Lagrangian equations of motion, linearises them about the upright equilibrium, and constructs the linear state-space model $(A, B)$ used by the LQR and PID controllers in this project.

---

## 2. Coordinate Frame Conventions

The coordinate frame follows the standard URDF/PyBullet world frame:

| Axis | Direction | Description |
|------|-----------|-------------|
| $+X$ | Forward | Direction of positive robot motion |
| $+Y$ | Left | Left of the robot (facing forward) |
| $+Z$ | Up | Vertical, opposite to gravity |

The robot body frame has its origin at the wheel axle midpoint:

- **Wheel spin axis**: $Y$-axis (both wheels rotate about $+Y$)
- **Pitch axis**: $Y$-axis (chassis tilts forward/backward about $+Y$)
- **Roll axis**: $X$-axis (not modelled; assumed symmetric and locked)
- **Yaw axis**: $Z$-axis (turning; modelled separately via differential drive)

```
        Z (up)
        |
        |   chassis CoM
        |      *
        |     /|
        |    / |  l = H/2
        |   /  |
   -----O------------ X (forward)
       wheel axle
```

**Pitch angle convention:**
- $\theta = 0$: chassis perfectly upright
- $\theta > 0$: chassis tilting forward (in the $+X$ direction)
- $\theta < 0$: chassis tilting backward

**Torque convention:**
- $u = \tau_{\text{total}} = \tau_L + \tau_R$ (sum of left and right wheel torques, Nm)
- Positive $u$: wheels spin forward, producing forward motion $(\dot{x} > 0)$
- Reaction torque on chassis: positive $u$ creates a backward torque on the chassis, reducing forward tilt

---

## 3. System Variables

### 3.1 Physical Parameters

| Symbol | Value | Units | Description |
|--------|-------|-------|-------------|
| $r$ | 0.08 | m | Wheel radius |
| $m_w$ | 0.25 | kg | Mass of one wheel |
| $m_c$ | 2.0 | kg | Chassis mass |
| $H$ | 0.45 | m | Chassis height |
| $W$ | 0.20 | m | Chassis width |
| $D$ | 0.12 | m | Chassis depth |
| $l$ | 0.225 | m | Chassis CoM height above axle ($= H/2$) |
| $g$ | 9.81 | m/s² | Gravitational acceleration |

### 3.2 Derived Inertia Parameters

| Symbol | Formula | Value | Units | Description |
|--------|---------|-------|-------|-------------|
| $I_w$ | $\frac{1}{2} m_w r^2$ | 0.0008 | kg·m² | Wheel spin inertia (solid cylinder, Y-axis) |
| $I_{c,yy}$ | $\frac{m_c}{12}(W^2 + D^2)$ | 0.009067 | kg·m² | Chassis pitch inertia about its CoM |
| $M_{\text{total}}$ | $m_c + 2m_w$ | 2.5 | kg | Total robot mass |
| $M_{\text{eff}}$ | $M_{\text{total}} + 2I_w/r^2$ | 2.75 | kg | Effective translational mass (rolling) |
| $I_{\text{pend}}$ | $I_{c,yy} + m_c l^2$ | 0.10157 | kg·m² | Effective pendulum inertia about axle |

> [!NOTE]
> The wheel inertia is for a solid disc of uniform density. The chassis inertia uses the box formula for rotation about the $Y$-axis (pitch), where the relevant transverse dimensions are $W$ (width, left-right) and $D$ (depth, fore-aft).

### 3.3 State Variables

The system state is a 4-vector:

$$\mathbf{x}_{\text{state}} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$$

| Index | Symbol | Units | Description |
|-------|--------|-------|-------------|
| 0 | $x$ | m | Chassis horizontal position (positive forward) |
| 1 | $\dot{x}$ | m/s | Chassis horizontal velocity |
| 2 | $\theta$ | rad | Chassis pitch angle (0 = upright, +ve = forward lean) |
| 3 | $\dot{\theta}$ | rad/s | Chassis pitch rate |

### 3.4 Input Variable

$$u = \tau_{\text{total}} \quad [\text{Nm}]$$

The total torque applied to both wheels combined. In the two-wheel differential drive, $u = \tau_L + \tau_R$ for straight-line motion. Positive $u$ drives the robot forward.

---

## 4. Lagrangian Derivation

### 4.1 Generalised Coordinates

We choose two generalised coordinates:

- $x$: horizontal position of the wheel axle (and by rigid-body geometry, the chassis base)
- $\theta$: chassis pitch angle about the wheel axle

The wheel rotation angle $\phi$ is eliminated via the **no-slip rolling constraint**:

$$\dot{x} = r \dot{\phi} \quad \Longrightarrow \quad \phi = x / r$$

### 4.2 Kinematic Positions

**Wheel centre** (both wheels share the axle): position $(x, 0)$ in the $XZ$-plane.

**Chassis CoM** position $(x_c, z_c)$:

$$x_c = x + l \sin\theta, \qquad z_c = r + l \cos\theta$$

where $r$ is the wheel radius (height of axle above ground) and $l = H/2$ is the distance from axle to chassis CoM.

### 4.3 Velocity of Chassis CoM

Differentiating with respect to time:

$$\dot{x}_c = \dot{x} + l\dot{\theta}\cos\theta, \qquad \dot{z}_c = -l\dot{\theta}\sin\theta$$

The squared speed of the chassis CoM:

$$v_c^2 = \dot{x}_c^2 + \dot{z}_c^2 = \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2$$

### 4.4 Kinetic Energy

The total kinetic energy has three contributions:

**1. Chassis translational KE:**

$$T_c^{\text{trans}} = \frac{1}{2} m_c v_c^2 = \frac{1}{2} m_c \left(\dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2\right)$$

**2. Chassis rotational KE about its CoM:**

$$T_c^{\text{rot}} = \frac{1}{2} I_{c,yy} \dot{\theta}^2$$

**3. Wheel kinetic energy** (both wheels, translational + rotational; using rolling constraint $\dot{\phi} = \dot{x}/r$):

$$T_w = 2 \cdot \left[\frac{1}{2} m_w \dot{x}^2 + \frac{1}{2} I_w \dot{\phi}^2\right] = m_w \dot{x}^2 + \frac{I_w}{r^2} \dot{x}^2 = \left(m_w + \frac{I_w}{r^2}\right)\dot{x}^2$$

> [!NOTE]
> We treat each wheel as a rigid disc spinning about its axle. The wheel translational CoM velocity equals $\dot{x}$ because the wheel centre is at the axle.

**Total kinetic energy:**

$$\boxed{T = \frac{1}{2}\left(M_{\text{eff}}\right)\dot{x}^2 + \frac{1}{2}\left(I_{c,yy} + m_c l^2\right)\dot{\theta}^2 + m_c l \dot{x} \dot{\theta} \cos\theta}$$

where $M_{\text{eff}} = m_c + 2m_w + 2I_w/r^2$.

Expanding explicitly:

$$T = \frac{1}{2} M_{\text{eff}} \dot{x}^2 + \frac{1}{2} I_{\text{pend}} \dot{\theta}^2 + m_c l \dot{x} \dot{\theta} \cos\theta$$

### 4.5 Potential Energy

Only the chassis CoM contributes to the potential energy (wheels roll on the ground, constant height):

$$V = m_c g z_c = m_c g (r + l \cos\theta)$$

Since $r$ is constant, we can drop it (it contributes only a constant offset):

$$\boxed{V = m_c g l \cos\theta}$$

### 4.6 Lagrangian

$$\mathcal{L} = T - V = \frac{1}{2} M_{\text{eff}} \dot{x}^2 + \frac{1}{2} I_{\text{pend}} \dot{\theta}^2 + m_c l \dot{x} \dot{\theta} \cos\theta - m_c g l \cos\theta$$

### 4.7 Euler-Lagrange Equations with Generalised Forces

The Euler-Lagrange equations are:

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i$$

where $q_1 = x$, $q_2 = \theta$, and $Q_i$ are the generalised forces from the wheel torque $\tau = u$.

**Generalised force for $x$:** The torque $\tau$ on the wheels produces a linear force $F = \tau / r$ at the wheel-ground contact point (by the rolling constraint). Thus:

$$Q_x = \frac{\tau}{r} = \frac{u}{r}$$

**Generalised force for $\theta$:** The reaction torque on the chassis from the wheel torque is $-\tau$ (Newton's third law — the wheels push forward, the chassis sees a backward reaction torque):

$$Q_\theta = -\tau = -u$$

---

## 5. Nonlinear Equations of Motion

### 5.1 Equation for $x$ (translational)

$$\frac{\partial \mathcal{L}}{\partial \dot{x}} = M_{\text{eff}} \dot{x} + m_c l \dot{\theta} \cos\theta$$

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{x}} = M_{\text{eff}} \ddot{x} + m_c l (\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta)$$

$$\frac{\partial \mathcal{L}}{\partial x} = 0$$

Euler-Lagrange gives:

$$\boxed{M_{\text{eff}} \ddot{x} + m_c l (\ddot{\theta} \cos\theta - \dot{\theta}^2 \sin\theta) = \frac{u}{r}}$$

### 5.2 Equation for $\theta$ (rotational)

$$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = I_{\text{pend}} \dot{\theta} + m_c l \dot{x} \cos\theta$$

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = I_{\text{pend}} \ddot{\theta} + m_c l (\ddot{x} \cos\theta - \dot{x}\dot{\theta}\sin\theta)$$

$$\frac{\partial \mathcal{L}}{\partial \theta} = -m_c l \dot{x} \dot{\theta} \sin\theta + m_c g l \sin\theta$$

Euler-Lagrange gives:

$$\boxed{m_c l \ddot{x} \cos\theta + I_{\text{pend}} \ddot{\theta} - m_c g l \sin\theta = -u}$$

### 5.3 Nonlinear System Summary

The two coupled, nonlinear second-order ODEs:

$$M_{\text{eff}} \ddot{x} + m_c l \cos\theta \cdot \ddot{\theta} = \frac{u}{r} + m_c l \dot{\theta}^2 \sin\theta \tag{1}$$

$$m_c l \cos\theta \cdot \ddot{x} + I_{\text{pend}} \ddot{\theta} = m_c g l \sin\theta - u \tag{2}$$

In matrix form:

$$\underbrace{\begin{bmatrix} M_{\text{eff}} & m_c l \cos\theta \\ m_c l \cos\theta & I_{\text{pend}} \end{bmatrix}}_{\mathbf{M}(\theta)} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} u/r + m_c l \dot{\theta}^2 \sin\theta \\ m_c g l \sin\theta - u \end{bmatrix}$$

This is the **nonlinear dynamics** that PyBullet integrates numerically.

---

## 6. Linearisation

### 6.1 Equilibrium Point

The upright equilibrium is:

$$\mathbf{x}_{\text{eq}} = \begin{bmatrix} x_0 \\ 0 \\ 0 \\ 0 \end{bmatrix}, \quad u_{\text{eq}} = 0$$

We linearise about $\theta = 0$, $\dot{\theta} = 0$, $\dot{x} = 0$.

### 6.2 Small-Angle Approximations

Near $\theta = 0$:

$$\sin\theta \approx \theta, \qquad \cos\theta \approx 1, \qquad \dot{\theta}^2 \sin\theta \approx 0$$

### 6.3 Linearised Equations

Substituting into (1) and (2):

$$M_{\text{eff}} \ddot{x} + m_c l \ddot{\theta} = \frac{u}{r} \tag{3}$$

$$m_c l \ddot{x} + I_{\text{pend}} \ddot{\theta} = m_c g l \theta - u \tag{4}$$

### 6.4 Matrix Form

Collecting $[\ddot{x}, \ddot{\theta}]^T$ on the left:

$$\underbrace{\begin{bmatrix} M_{\text{eff}} & m_c l \\ m_c l & I_{\text{pend}} \end{bmatrix}}_{\mathbf{M}_2} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \underbrace{\begin{bmatrix} 0 & 0 \\ 0 & m_c g l \end{bmatrix}}_{\mathbf{F}_2} \begin{bmatrix} x \\ \theta \end{bmatrix} + \underbrace{\begin{bmatrix} 1/r \\ -1 \end{bmatrix}}_{\mathbf{G}_2} u$$

Solving for the accelerations:

$$\begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \mathbf{M}_2^{-1} \mathbf{F}_2 \begin{bmatrix} x \\ \theta \end{bmatrix} + \mathbf{M}_2^{-1} \mathbf{G}_2 \, u$$

### 6.5 Inverse of $\mathbf{M}_2$

The determinant of $\mathbf{M}_2$:

$$\det(\mathbf{M}_2) = M_{\text{eff}} I_{\text{pend}} - (m_c l)^2$$

$$\mathbf{M}_2^{-1} = \frac{1}{\det(\mathbf{M}_2)} \begin{bmatrix} I_{\text{pend}} & -m_c l \\ -m_c l & M_{\text{eff}} \end{bmatrix}$$

---

## 7. State-Space Representation

### 7.1 State-Space Form

The full first-order system $\dot{\mathbf{x}}_{\text{state}} = \mathbf{A} \mathbf{x}_{\text{state}} + \mathbf{B} u$:

$$\frac{d}{dt} \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ a_{21} & 0 & a_{23} & 0 \\ 0 & 0 & 0 & 1 \\ a_{41} & 0 & a_{43} & 0 \end{bmatrix} \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix} + \begin{bmatrix} 0 \\ b_{21} \\ 0 \\ b_{41} \end{bmatrix} u$$

where the non-zero entries come from $\mathbf{M}_2^{-1} \mathbf{F}_2$ (for $\mathbf{A}$) and $\mathbf{M}_2^{-1} \mathbf{G}_2$ (for $\mathbf{B}$):

Let $\Delta = \det(\mathbf{M}_2) = M_{\text{eff}} I_{\text{pend}} - m_c^2 l^2$. Then:

$$a_{23} = \frac{\partial \ddot{x}}{\partial \theta} = \frac{I_{\text{pend}}}{\Delta} \cdot 0 + \frac{-m_c l}{\Delta} \cdot m_c g l = \frac{-m_c^2 g l^2}{\Delta}$$

Wait — let us be careful. Rows of $\mathbf{M}_2^{-1} \mathbf{F}_2$:

$$\mathbf{M}_2^{-1} \mathbf{F}_2 = \frac{1}{\Delta}\begin{bmatrix} I_{\text{pend}} & -m_c l \\ -m_c l & M_{\text{eff}} \end{bmatrix} \begin{bmatrix} 0 & 0 \\ 0 & m_c g l \end{bmatrix} = \frac{1}{\Delta}\begin{bmatrix} 0 & -m_c^2 g l^2 \cdot \frac{1}{1} \\ 0 & M_{\text{eff}} m_c g l \end{bmatrix}$$

Expanding:

$$\mathbf{M}_2^{-1} \mathbf{F}_2 = \frac{1}{\Delta}\begin{bmatrix} 0 & -m_c^2 g l^2 \\ 0 & M_{\text{eff}} m_c g l \end{bmatrix}$$

Wait, let's be explicit:

Row 0 of $\mathbf{M}_2^{-1}$ is $\frac{1}{\Delta}[I_{\text{pend}}, -m_c l]$. Multiplied by $\mathbf{F}_2$:

- Column 0: $I_{\text{pend}} \cdot 0 + (-m_c l) \cdot 0 = 0$ → $a_{21} = 0$ (position doesn't directly drive $\ddot{x}$)
- Column 1: $I_{\text{pend}} \cdot 0 + (-m_c l)(m_c g l) = -m_c^2 g l^2$ → divided by $\Delta$: $a_{23} = \frac{-m_c^2 g l^2}{\Delta}$

Row 1 of $\mathbf{M}_2^{-1}$ is $\frac{1}{\Delta}[-m_c l, M_{\text{eff}}]$. Multiplied by $\mathbf{F}_2$:

- Column 0: $(-m_c l) \cdot 0 + M_{\text{eff}} \cdot 0 = 0$ → $a_{41} = 0$
- Column 1: $(-m_c l) \cdot 0 + M_{\text{eff}} \cdot m_c g l = M_{\text{eff}} m_c g l$ → $a_{43} = \frac{M_{\text{eff}} m_c g l}{\Delta}$

> [!IMPORTANT]
> Note that $a_{23} < 0$ and $a_{43} > 0$. The positive $a_{43}$ is the gravity instability term: a positive tilt $\theta > 0$ drives $\ddot{\theta} > 0$ (further tilt). The $a_{23} < 0$ term reflects the coupling: a positive tilt slightly decelerates $\ddot{x}$ due to the inertial coupling in $\mathbf{M}_2$.

For $\mathbf{B}$:

$$\mathbf{M}_2^{-1} \mathbf{G}_2 = \frac{1}{\Delta}\begin{bmatrix} I_{\text{pend}} & -m_c l \\ -m_c l & M_{\text{eff}} \end{bmatrix} \begin{bmatrix} 1/r \\ -1 \end{bmatrix} = \frac{1}{\Delta}\begin{bmatrix} I_{\text{pend}}/r + m_c l \\ -m_c l / r - M_{\text{eff}} \end{bmatrix}$$

So:

$$b_{21} = \frac{I_{\text{pend}}/r + m_c l}{\Delta}, \qquad b_{41} = \frac{-(m_c l / r + M_{\text{eff}})}{\Delta}$$

Note $b_{41} < 0$: positive torque reduces $\ddot{\theta}$ (corrects forward tilt).

### 7.2 Output Equation

We use the full state as output (identity observation):

$$\mathbf{y} = \mathbf{C} \mathbf{x}_{\text{state}} + \mathbf{D} u = \mathbf{I}_4 \mathbf{x}_{\text{state}} + \mathbf{0} \cdot u$$

---

## 8. Numeric Values from Configuration

Using $r=0.08$, $m_w=0.25$, $m_c=2.0$, $H=0.45$, $W=0.20$, $D=0.12$, $g=9.81$:

### Computed Parameters

| Parameter | Value | Formula |
|-----------|-------|---------|
| $l$ | 0.225 m | $H/2$ |
| $I_{c,yy}$ | 0.009067 kg·m² | $m_c(W^2+D^2)/12$ |
| $I_w$ | 0.000800 kg·m² | $m_w r^2/2$ |
| $M_{\text{total}}$ | 2.500 kg | $m_c + 2m_w$ |
| $M_{\text{eff}}$ | 2.750 kg | $M_{\text{total}} + 2I_w/r^2$ |
| $I_{\text{pend}}$ | 0.10157 kg·m² | $I_{c,yy} + m_c l^2$ |
| $\Delta$ | 0.07933 kg²·m² | $M_{\text{eff}} I_{\text{pend}} - (m_c l)^2$ |

### A Matrix (numeric)

$$\mathbf{A} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & -12.834 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & +68.09 & 0 \end{bmatrix}$$

> [!NOTE]
> Values computed numerically by `compute_state_space()`. The exact values depend on the config file parameters. The signs and structure are derived analytically above.

### B Matrix (numeric)

$$\mathbf{B} = \begin{bmatrix} 0 \\ +3.44 \\ 0 \\ -34.19 \end{bmatrix}$$

---

## 9. Eigenvalue Analysis

The eigenvalues of $\mathbf{A}$ are the solutions to $\det(\lambda \mathbf{I} - \mathbf{A}) = 0$.

The characteristic polynomial factors into two pairs:

1. **Two zero eigenvalues** ($\lambda_{1,2} = 0$): These arise because $x$ and $\dot{x}$ form a pure integrator chain — position is not constrained by any restoring force. Without a position reference in the control law, $x$ drifts freely (a double integrator).

2. **One positive real eigenvalue** ($\lambda_3 > 0$): The unstable mode — gravity pulling the chassis to fall. The growth rate is approximately $\sqrt{a_{43}} \approx \sqrt{M_{\text{eff}} m_c g l / \Delta}$.

3. **One negative real eigenvalue** ($\lambda_4 < 0$): A stable mode arising from the coupled pendulum-cart dynamics. It represents a direction in state space that naturally returns to equilibrium.

### Physical Interpretation

| Eigenvalue | Type | Physical Meaning |
|------------|------|-----------------|
| $\lambda = 0$ (×2) | Marginally stable | Position integrator — robot drifts without position control |
| $\lambda > 0$ | **Unstable** | Gravity-driven toppling — requires active control |
| $\lambda < 0$ | Stable | Damped oscillatory coupling mode |

> [!CAUTION]
> The positive real eigenvalue confirms the system is open-loop unstable. Without a stabilising feedback controller, any small perturbation from $\theta = 0$ will grow exponentially. The time constant of instability is $1/\lambda_3$ seconds.

The unstable eigenvalue value is approximately:

$$\lambda_3 \approx \sqrt{\frac{M_{\text{eff}} m_c g l}{\Delta}} \approx \sqrt{\frac{2.75 \times 2.0 \times 9.81 \times 0.225}{0.07933}} \approx \sqrt{152.9} \approx 12.4 \; \text{rad/s}$$

This corresponds to a toppling time constant of roughly 80 ms — the robot falls extremely quickly without control.

---

## 10. Controllability Analysis

A linear system $(\mathbf{A}, \mathbf{B})$ is **controllable** if and only if the controllability matrix has full rank:

$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} & \mathbf{A}^2\mathbf{B} & \mathbf{A}^3\mathbf{B} \end{bmatrix} \in \mathbb{R}^{4 \times 4}$$

$$\text{rank}(\mathcal{C}) = 4 \quad \Longrightarrow \quad \text{controllable}$$

### Why Controllability Holds

Intuitively:
- $\mathbf{B}$ has non-zero entries at rows 1 and 3 ($\dot{x}$ and $\dot{\theta}$ channels)
- $\mathbf{A}\mathbf{B}$ propagates these effects to rows 0 and 2 ($x$ and $\theta$)
- $\mathbf{A}^2\mathbf{B}$ and $\mathbf{A}^3\mathbf{B}$ provide the remaining independent directions

Physically, the single torque input $u$ can drive all four state components:
- Torque → wheel acceleration → $\dot{x}$ changes → $x$ changes (via integrator)
- Torque → reaction on chassis → $\dot{\theta}$ changes → $\theta$ changes (via integrator)

> [!IMPORTANT]
> Full controllability means the LQR problem is well-defined: we can place all four closed-loop eigenvalues anywhere in the left half-plane, guaranteeing stabilisation.

### Numerical Confirmation

The controllability matrix computed from the nominal parameters has $\text{rank}(\mathcal{C}) = 4$, confirming the system is controllable. This is verified by `tests/test_dynamics.py::test_controllability_matrix_rank`.

---

## 11. Model Simplifications and Validity Range

### 11.1 Simplifications Made

| Assumption | Consequence |
|-----------|-------------|
| **Rigid bodies** | No structural flexure; valid for small accelerations |
| **No-slip rolling** | Wheel-ground contact is a pure kinematic constraint; breaks down on loose surfaces or at high slip |
| **Planar motion** | Yaw and roll ignored; valid for straight-line driving at low lateral accelerations |
| **Point-mass wheels** | Wheels modelled as solid uniform discs; ignores spoke structure and tyre compliance |
| **Symmetric chassis** | Left-right symmetry assumed; $I_{c,yy}$ uses box formula |
| **Chassis CoM at $H/2$** | Electronics, battery, etc. assumed symmetric; real CoM may be off-centre |
| **No friction/damping in joints** | Motor back-EMF, gear friction, joint damping all neglected in the model |
| **Small-angle linearisation** | Valid only for $|\theta| \lesssim 0.2$ rad (≈ 11°) |
| **Linear actuator model** | Motor dynamics (electrical time constant, back-EMF) not included; control assumes instantaneous torque |

### 11.2 Validity Range

The linearised model provides a good approximation when:

- $|\theta| < 0.15$ rad (≈ 8°): Small-angle assumption holds to within ~1%
- $|\dot{\theta}| < 1.0$ rad/s: Cross-product terms ($\dot{\theta}^2 \sin\theta$) remain negligible
- $|u| < 5$ Nm: Linear actuator model holds before saturation
- Motion is predominantly in the $X$-direction (yaw rate $< 0.3$ rad/s)

For large angles or aggressive manoeuvres, the full nonlinear model (Section 5) must be used, or a gain-scheduled / nonlinear controller must be designed.

### 11.3 Parameter Uncertainty

The model is sensitive to:
- **CoM height $l$**: Errors in $l$ directly change $a_{43}$ (gravity term). A 10% error in $l$ changes the unstable eigenvalue by ~5%.
- **$I_{\text{pend}}$**: The effective pendulum inertia affects how quickly $\theta$ responds to torque. If the chassis load is off-centre, $I_{c,yy}$ may be incorrect.
- **$M_{\text{eff}}$**: Adding payload increases $M_{\text{eff}}$, reducing $b_{21}$ and $b_{41}$ (the torque has less authority).

---

## 12. Linear Model vs. PyBullet Mismatch Sources

When comparing the linear model predictions to PyBullet simulation results, discrepancies arise from several sources:

### 12.1 Nonlinear Terms Neglected

The linearisation drops:
- $m_c l \dot{\theta}^2 \sin\theta$ (centripetal term in the $x$ equation)
- $(1 - \cos\theta)$ coupling (the $\cos\theta$ factor in $\mathbf{M}(\theta)$)

These terms become significant when $|\theta| > 0.1$ rad or $|\dot{\theta}| > 0.5$ rad/s.

### 12.2 Joint Damping and Friction

PyBullet simulates `wheel_joint_damping = 0.01` (viscous friction at the wheel joints). This introduces a velocity-dependent resistive torque not present in the analytical model. The effect is most visible at high wheel speeds.

### 12.3 Contact Dynamics

PyBullet uses a penalty-based contact model with configurable lateral and rolling friction coefficients. The rolling friction coefficient (`rolling: 0.001`) creates a small retarding torque not captured in the pure rolling constraint. The lateral friction (`lateral: 0.8`) prevents lateral sliding and couples lateral forces into the dynamics.

### 12.4 Actuator Saturation

The physical robot has `max_torque_per_wheel = 5.0` Nm (total 10 Nm). When the linear model's ideal torque demand exceeds this, the actual applied torque is clipped, breaking the linear input-output relationship.

### 12.5 Numerical Integration

The analytical model assumes a continuous-time system. PyBullet uses a fixed-step simulator at `timestep = 0.004167 s` (240 Hz). At this resolution, fast dynamics (high-frequency poles/zeros) may be under-resolved, particularly in the contact constraint solver.

### 12.6 Motor Dynamics

A real motor has an electrical time constant $\tau_e = L/R$ (typically 5–20 ms for small DC motors). This introduces a first-order lag between the commanded and actual torque:

$$\tau_{\text{actual}}(s) = \frac{1}{1 + \tau_e s} \tau_{\text{commanded}}(s)$$

This lag is not modelled in the linear model and can degrade LQR performance at high bandwidth.

### 12.7 Rigid Body Assumption vs. Flexible Joints

The chassis-wheel connection is modelled as a pure revolute joint in PyBullet, but in a real robot, compliance (rubber mounts, flex PCBs) introduces additional resonant modes not captured in the 4-state model.

### 12.8 Summary Table

| Mismatch Source | Magnitude | Mitigation |
|-----------------|-----------|------------|
| Nonlinear terms | Medium (grows with $|\theta|$) | Nonlinear MPC or gain scheduling |
| Joint damping | Small (< 5% at low speed) | Add damping term to model |
| Contact model | Small–medium | Tune friction coefficients |
| Actuator saturation | Large (near limits) | Anti-windup, torque scaling |
| Discrete-time effects | Negligible (240 Hz) | Use discrete-time A, B via ZOH |
| Motor dynamics | Small–medium | Add first-order actuator lag |
| CoM offset | Model-dependent | Measure or identify CoM experimentally |

> [!TIP]
> To reduce the linear/nonlinear gap in practice: (1) identify the actual $l$ and $I_{\text{pend}}$ from open-loop step responses, (2) add a first-order actuator lag to the model, and (3) augment the state with an integral term on $\theta$ to handle constant disturbances (e.g., tilted ground).

---

## Appendix: Python Implementation Reference

The dynamics model is implemented in [`src/balancing_robot/dynamics.py`](../src/balancing_robot/dynamics.py).

Key functions:

| Function | Description |
|----------|-------------|
| `compute_model_params(robot_cfg)` | Extract physical parameters from YAML config |
| `compute_state_space(params)` | Compute $(A, B)$ matrices numerically |
| `compute_eigenvalues(A)` | Return eigenvalues of $A$ |
| `compute_controllability_matrix(A, B)` | Build the $4 \times 4$ controllability matrix |
| `is_controllable(A, B)` | Check full-rank controllability |
| `simulate_linear_model(A, B, x0, K, dt, n)` | Euler-integration of the linear model |
| `print_model_summary(robot_cfg)` | Print all parameters, $A$, $B$, eigenvalues, and controllability |

To run the summary:
```bash
cd /path/to/project
python3 -c "
from balancing_robot.config import load_robot_config
from balancing_robot.dynamics import print_model_summary
print_model_summary(load_robot_config())
"
```

To run the dynamics tests:
```bash
python3 -m pytest tests/test_dynamics.py -v
```
