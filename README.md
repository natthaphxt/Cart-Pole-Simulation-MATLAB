# Cart-Pole Swing-Up and Stabilization System

**FRA333 Robot Kinematics**

**Student ID:**
- 66340500006
- 66340500074
- 66340500077

---

## Abstract

This project implements a hybrid control system for the cart-pole (inverted pendulum) using MATLAB/Simulink. The system combines an energy-based swing-up controller with LQR stabilization. The swing-up controller brings the pendulum from its downward position to near-upright, then automatically switches to LQR control for balancing. The system achieves stable inverted balance within approximately 5 seconds.

---

## Table of Contents

1. [Objectives](#1-objectives)
2. [Scope of Project](#2-scope-of-project)
3. [Related Theories](#3-related-theories)
4. [System Description](#4-system-description)
5. [System Architecture](#5-system-architecture)
6. [System Diagram](#6-system-diagram)
7. [Methodology](#7-methodology)
8. [Implementation](#8-implementation)
9. [Experimental Results](#9-experimental-results)
10. [Expected Study Results](#10-expected-study-results)
11. [Conclusion](#11-conclusion)
12. [Schedule](#12-schedule)
13. [References](#13-references)
14. [Appendix](#14-appendix-matlab-code)

---

## 1. Objectives

1. Develop an energy-based swing-up controller to bring the pendulum from the downward hanging position to the upright position
2. Design an LQR (Linear Quadratic Regulator) controller for stabilization at the inverted equilibrium point
3. Implement automatic mode switching between swing-up and stabilization controllers
4. Validate the complete hybrid control system through MATLAB/Simulink simulation
5. Analyze system performance and demonstrate successful swing-up and stabilization

---

## 2. Scope of Project

### 2.1 Mathematical Model
- Study limited to 2D model with cart movement along the x-axis only
- Study both Swing-up and Stabilization control aspects
- Define basic system parameters (cart mass, pendulum mass, pendulum length)
- Control force (u) applied to the cart has explicit magnitude constraints
- Create an ideal system model without considering friction and damping effects on the pendulum
- Include ground friction for the cart
- Pendulum angle constraint: within ±10 degrees (for stabilization)
- Cart travel range is limited
- Cart starting position is always at the center (x = 0)

### 2.2 Controller Design
- Use **Energy-Based Control** for Swing-up phase
- Use **LQR (Linear Quadratic Regulator)** for Stabilization phase

### 2.3 Simulation Environment
- Use MATLAB/Simulink for simulation
- Use Simscape Multibody for physical modeling
- Display results as state graphs and animation (simulation)

### 2.4 Control Constraints
- Control force (u) applied to the cart has explicit magnitude constraints:
  - Swing-up controller: |F| ≤ 3.5 N
  - LQR stabilization controller: |F| ≤ 10 N

### 2.5 Limitations
- This project is limited to simulation in MATLAB/Simulink; no physical hardware implementation
- External disturbances are not explicitly modeled

---

## 3. Related Theories

### 3.1 Lagrangian Method

The Lagrangian method is used to derive the equations of motion for the cart-pole system. The Lagrangian is defined as:

$$L = T - V$$

Where T is kinetic energy and V is potential energy.

**Kinetic Energy:**
$$T = \frac{1}{2}M\dot{x}^2 + \frac{1}{2}m(\dot{x}^2 + 2\dot{x}L\cos\theta\dot{\theta} + L^2\dot{\theta}^2)$$

**Potential Energy:**
$$V = mgL\cos\theta$$

Applying the Euler-Lagrange equations:
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}}\right) - \frac{\partial L}{\partial q} = Q$$

yields the nonlinear equations of motion for the cart-pole system.

### 3.2 Linearization

For LQR controller design, the nonlinear system must be linearized around the upright equilibrium point (θ = 0). Using Taylor series expansion and small angle approximations (sin θ ≈ θ, cos θ ≈ 1), the linearized state-space model is:

$$\dot{x} = Ax + Bu$$

With state vector **x** = [x, θ, ẋ, θ̇]ᵀ and:

$$A = \begin{bmatrix} 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 3.27 & -1.48 & 0 \\ 0 & 19.62 & -2.22 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 0 \\ 2.96 \\ 4.444 \end{bmatrix}$$

### 3.3 LQR (Linear Quadratic Regulator)

LQR is an optimal control method that finds feedback gains to minimize the cost function:

$$J = \int_0^\infty (x^TQx + u^TRu) \, dt$$

Where:
- **Q matrix**: Penalizes state deviations (how far from upright)
- **R value**: Penalizes control effort (how much force used)

The optimal feedback gain K is obtained by solving the Algebraic Riccati Equation. MATLAB's `lqr()` function computes this automatically.

**Control Law:**
$$u = -Kx$$

**Design Parameters:**
```matlab
Q = diag([1, 3, 1, 1]);  % State weights (angle prioritized)
R = 0.1;                  % Control effort weight
```

| Parameter | Effect |
|-----------|--------|
| Increase Q₂₂ (angle weight) | Tighter angle control, less wobble |
| Decrease R | More aggressive control, faster response |
| Increase Q₁₁ (position weight) | Keeps cart closer to center |

### 3.4 Energy-Based Control

Energy-based control manipulates the system's total mechanical energy to achieve swing-up. The principle is to pump energy into the system until it reaches the target energy level corresponding to the upright position.

**Target Energy (at upright position):**
$$E_{target} = mgL$$

**Current Energy:**
$$E_{current} = \frac{1}{2}m(\dot{x}^2 + 2\dot{x}L\cos\theta\dot{\theta} + L^2\dot{\theta}^2) + mgL\cos\theta$$

**Control Law:**
$$F = k \cdot \dot{\theta} \cdot (E_{current} - E_{target})$$

This strategy:
- Adds energy when pendulum swings toward upright (same direction as motion)
- Removes energy if overshooting (opposite direction)
- Naturally pumps the pendulum higher with each swing until reaching target energy

---

## 4. System Description

### 4.1 Physical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Cart Mass | M | 0.3 | kg |
| Pole Mass | m | 0.15 | kg |
| Pole Half-Length | L | 0.25 | m |
| Gravity | g | 9.81 | m/s² |
| Damping | b | 0.5 | N·s/m |

### 4.2 Angle Convention
- **θ = 0°:** Pendulum upright (target equilibrium)
- **θ = 180°:** Pendulum hanging down (initial position)

### 4.3 State Variables

| State | Symbol | Description |
|-------|--------|-------------|
| Cart Position | x | Horizontal position of cart (m) |
| Pendulum Angle | θ | Angle from vertical (rad) |
| Cart Velocity | ẋ | Cart linear velocity (m/s) |
| Angular Velocity | θ̇ | Pendulum angular velocity (rad/s) |

---

## 5. System Architecture
![System Architecture](images/System_Architecture.png)

### 5.1 Symbol Definitions

| Symbol | Description |
|--------|-------------|
| $\ddot{\theta}$ | Angular acceleration of pendulum |
| $\ddot{x}$ | Linear acceleration of cart |
| **A** | System matrix (state matrix) |
| **B** | Input matrix (describes how control signal affects system states) |
| **K** | LQR gain matrix used for pendulum stabilization |
| **F** | Force applied to cart |

### 5.2 Architecture Overview

The system architecture is divided into two main parts as shown in Figure 1:

1. **Controller Design Process**: Computes the optimal LQR gain matrix K
2. **Full State Feedback**: Uses the computed K to stabilize the pendulum

**Important Note:** The gain matrix **K** is computed only once and does not change over time. This is because K is derived from LQR calculation, which assumes the system is **Linear Time-Invariant (LTI)**. This assumption comes from linearizing the system around the equilibrium point where the pendulum angle θ ≈ 0. The computed K is then used to stabilize the pendulum at the upright position.

### 5.3 LQR Controller Design Process


*Figure 1: System Architecture - Controller Design Process and Full State Feedback*

### 5.4 Why K is Constant

The LQR gain matrix K remains constant because:

1. **Linearization Assumption**: The system is linearized around θ ≈ 0 (upright position)
2. **LTI System**: LQR assumes Linear Time-Invariant dynamics
3. **Offline Computation**: K is computed once using the Riccati equation before simulation
4. **Optimal for Equilibrium**: K is optimal only near the linearization point

This is why the LQR controller is only activated when the pendulum is close to upright (|θ| < 15°), where the linear approximation is valid.

---

## 6. Block Diagram

### 6.1 Overall Control Architecture
![description](images/Block_Diagram.png)
### 6.2 Simulink Block Diagram Structure
![description](images/Plant.png)

---

## 7. Methodology

### 7.1 Control Strategy Overview

The system uses a hybrid control approach with automatic switching between two controllers:

1. **Energy-Based Swing-Up Controller**: Active when pendulum is far from upright
2. **LQR Stabilization Controller**: Active when pendulum is near upright position

### 7.2 Energy-Based Swing-Up Control

**Principle:** Manipulate the total mechanical energy to swing the pendulum from hanging (θ = 180°) to upright (θ = 0°).

**Target Energy at Upright Position:**
```
E_target = m × g × L = 0.15 × 9.81 × 0.25 = 0.368 J
```

**Current Energy Calculation:**
```
E_current = 0.5 × m × (ẋ² + 2×ẋ×L×cos(θ)×ω + L²×ω²) + m×g×L×cos(θ)
```

**Control Law:**
```
F = k × ω × (E_current - E_target)
```
- Gain: k = 10
- Force saturation: |F| ≤ 3.5 N

### 7.3 LQR Stabilization Control

**Control Law:**
```
F = -K × [x, θ, ẋ, ω]ᵀ
```
- Force saturation: |F| ≤ 10 N
- K is computed using MATLAB's `lqr()` function

### 7.4 Angle Wrapping

The angle is wrapped to [-π, π] for continuous representation:
```matlab
θ_wrapped = atan2(sin(θ), cos(θ))
```

### 7.5 Switching Conditions

| Parameter | Threshold | Description |
|-----------|-----------|-------------|
| Angle | \|θ\| < 0.262 rad (15°) | Switch to LQR when pendulum is near upright |

---

## 8. Implementation

### 8.1 Requirements

| Software | Version | Purpose |
|----------|---------|---------|
| MATLAB | R2024b+ | Main programming environment |
| Simulink | R2024b+ | System simulation |
| Control System Toolbox | - | LQR controller design |
| Simscape Multibody | - | Physical modeling |

### 8.2 Project Files

| File | Description |
|------|-------------|
| `LQR_Pendulum.m` | MATLAB script defining system parameters and computing LQR gain |
| `Sim_CartPole_2024.slx` | Simulink model with controllers, switching logic, and cart-pole dynamics |
| `README.md` | Project documentation |

### 8.3 How to Run
**Step 1:** Download parameters and Simulink into MATLAB workspace

**Step 2:** Load parameters into MATLAB workspace
```matlab
run('LQR_Pendulum.m')
```

**Step 3:** Open the Simulink model
```matlab
open('Sim_CartPole_2024.slx')
```

**Step 4:** Run the simulation
- Click the "Run" button in Simulink
- Or use command: `sim('Sim_CartPole_2024')`

**Step 5:** Observe results
- View real-time animation in Mechanics Explorer
- Analyze state plots in Scope blocks

### 8.4 Parameter Configuration

The system parameters can be modified in `LQR_Pendulum.m`:

```matlab
% Physical Parameters
M = 0.3;        % Cart mass (kg)
m = 0.15;       % Pendulum mass (kg)
L = 0.25;       % Pole half-length (m)
g = 9.81;       % Gravity (m/s²)
b = 0.5;        % Damping coefficient (N·s/m)

% Initial Condition
theta0 = 180;   % Initial angle (degrees) - hanging down

% LQR Tuning
Q = diag([1, 3, 1, 1]);  % State weights
R = 0.1;                  % Control effort weight

% Switching Thresholds
theta_threshold = 0.3;   % Angle threshold (rad)
omega_threshold = 1.5;   % Angular velocity threshold (rad/s)
```

### 8.5 Simulink Model Components

**Main Subsystems:**
1. **Stateflow Chart**: Implements hybrid controller with mode switching
2. **Simscape Multibody**: Physical cart-pole model
3. **Sensor Blocks**: Extract state variables (x, θ, ẋ, θ̇)
4. **Scope Blocks**: Visualize system states and control input

**Key Blocks:**
- Prismatic Joint: Cart translation along x-axis
- Revolute Joint: Pendulum rotation
- Force Actuator: Applies control force to cart
- Transform Sensor: Measures pendulum angle

---

## 9. Experimental Results
![description](images/theta.png)
*Results: theta*


![description](images/omega.png)
*Results: omega*
### 9.1 Simulation Phases

| Phase | Time Period | Description |
|-------|-------------|-------------|
| Initial | 0 - 1 s | System starts with pendulum hanging down (θ = 180°) |
| Swing-Up | 1 - 5 s | Energy-based controller pumps pendulum with increasing oscillations |
| Transition | ~5 s | Automatic switch to LQR when \|θ\| < 15° |
| Stabilization | > 5 s | LQR maintains stable inverted balance |

### 9.2 Performance Metrics

| Metric | Result |
|--------|--------|
| Swing-up time | 3 - 5 seconds |
| Settling time (after LQR activation) | < 2 seconds |
| Steady-state angle error | < 0.01 rad (< 0.6°) |
| Maximum cart displacement during swing-up | ~0.5 m |
| Final cart position | Near origin (x ≈ 0) |

### 9.3 Controller Behavior Analysis

**Swing-Up Phase:**
- The energy-based controller successfully injects energy into the system
- Multiple swings required to reach target energy level
- Force saturation (±3.5 N) prevents excessive control action
- Cart moves back and forth to facilitate pendulum swinging

**Stabilization Phase:**
- Smooth transition from swing-up to LQR
- Rapid damping of oscillations after switch
- Cart returns toward center position
- Pendulum maintains stable upright position

### 9.4 Simulation Results

*Angle vs. Time Plot*
- Shows transition from θ = 180° (down) to θ = 0° (up)
- Oscillatory behavior during swing-up
- Rapid settling after LQR activation

*Cart Position vs. Time Plot*
- Cart oscillates during swing-up phase
- Stabilizes near origin after LQR takes over

*Control Force vs. Time Plot*
- Saturated oscillations during swing-up (±3.5 N)
- Reduced control effort during stabilization phase

---

## 10. Expected Study Results

### 10.1 Technical Achievements

1. **Successful Swing-Up**: The energy-based controller should reliably bring the pendulum from hanging position (180°) to near-upright position within 5 seconds

2. **Stable Balancing**: The LQR controller should maintain the inverted pendulum at the upright position with minimal oscillation (steady-state error < 1°)

3. **Smooth Transition**: The switching between controllers should be seamless without causing instability or large transients

4. **Constraint Satisfaction**: Control forces should remain within specified limits (±3.5 N for swing-up, ±10 N for stabilization)

### 10.2 Learning Outcomes

Through this project, students will gain practical experience in:

1. **System Modeling**: Applying Lagrangian mechanics to derive equations of motion for mechanical systems

2. **Linearization Techniques**: Understanding how to linearize nonlinear systems around equilibrium points for controller design

3. **Optimal Control**: Designing LQR controllers and understanding the role of Q and R matrices in performance tuning

4. **Energy-Based Control**: Implementing energy shaping concepts for nonlinear system control

5. **Hybrid Systems**: Designing switching logic for multi-mode control systems

6. **Simulation Tools**: Proficiency in MATLAB/Simulink and Simscape Multibody for control system simulation

### 10.3 Validation Criteria

| Criterion | Expected Result |
|-----------|-----------------|
| Swing-up success rate | 100% from θ₀ = 180° |
| Time to reach upright | ≤ 5 seconds |
| Stabilization time | ≤ 2 seconds after switch |
| Steady-state error | < 0.02 rad |
| Control saturation compliance | No violations |

---

## 11. Conclusion

This project successfully demonstrates a hybrid control system for the cart-pole swing-up and stabilization problem. The key achievements include:

### 11.1 Technical Accomplishments

1. **Energy-Based Swing-Up Control**: Successfully implemented an energy manipulation strategy that reliably swings the pendulum from the hanging position to near-upright. The controller effectively pumps energy into the system through coordinated force application.

2. **LQR Stabilization**: Designed and tuned an optimal LQR controller that maintains stable balance of the inverted pendulum with minimal steady-state error and quick settling time.

3. **Automatic Mode Switching**: Implemented robust switching logic based on angle threshold that ensures smooth transition between swing-up and stabilization modes.

4. **Simulation Validation**: Demonstrated complete system functionality through MATLAB/Simulink simulation with Simscape Multibody for realistic physical modeling.

### 11.2 Control System Insights

The project illustrates several important control engineering concepts:

- **Nonlinear vs. Linear Control**: Energy-based control handles the nonlinear swing-up regime, while LQR is effective for the linearized region near the equilibrium

- **Hybrid Control Architecture**: Combining multiple controllers with appropriate switching logic enables handling of control problems with multiple operating regimes

- **Energy Shaping**: Manipulating system energy provides an intuitive and effective approach for under-actuated mechanical systems

### 11.3 Future Work

Potential extensions of this project include:

1. Implementation on physical hardware
2. Adding position constraints for cart movement
3. Incorporating disturbance rejection capabilities
4. Exploring alternative swing-up strategies (e.g., optimal trajectory planning)
5. Implementing robust control for parameter uncertainties

---

## 12. Schedule

### 12.1 Project Timeline (Gantt Chart)

| No. | Task | Week 1 | Week 2 | Week 3 | Week 4 | Week 5 | Week 6 |
|:---:|------|:------:|:------:|:------:|:------:|:------:|:------:|
| 1 | Literature Review and Proposal | ██ | ██ | | | | |
| 2 | Linearization and LQR Design | | ██ | ██ | | | |
| 3 | Study and Design Energy-Based Swing-up | | | ██ | ██ | | |
| 4 | Build Simulation and Testing | | | | ██ | ██ | |
| 5 | Documentation and Report Writing | | | | | | ██ |

### 12.2 Detailed Task Breakdown

**Week 1-2: Literature Review and Proposal**
- Study cart-pole system dynamics
- Review Lagrangian mechanics
- Survey existing control approaches
- Write project proposal

**Week 2-3: Linearization and LQR Design**
- Derive equations of motion
- Linearize system around upright equilibrium
- Design LQR controller in MATLAB
- Tune Q and R matrices

**Week 3-4: Energy-Based Swing-up Design**
- Study energy-based control theory
- Implement swing-up controller
- Develop switching logic
- Integrate with LQR controller

**Week 4-5: Simulation and Testing**
- Build Simulink model with Simscape Multibody
- Integrate all control components
- Run simulation tests
- Parameter tuning and optimization
- Validate performance metrics

**Week 6: Documentation**
- Compile results and analysis
- Prepare final report
- Create presentation materials
- Video recording of simulation

---

## 13. References

1. Anderson, B. D., & Moore, J. B. (1990). *Optimal Control: Linear Quadratic Methods*. Prentice Hall.

2. Åström, K. J., & Furuta, K. (2000). Swinging up a pendulum by energy control. *Automatica*, 36(2), 287-295.

3. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.

4. Fantoni, I., & Lozano, R. (2002). *Non-linear Control for Underactuated Mechanical Systems*. Springer.

5. ME389 MEM04 PendulumGantry Guideline - Course Materials.

6. MathWorks Documentation - Control System Toolbox, Simscape Multibody.

---

## 14. Video testing

[![Watch the video](https://img.youtube.com/vi/dQTII64_-kc/maxresdefault.jpg)](https://www.youtube.com/watch?v=dQTII64_-kc)

*Animation: Cart-pole swing-up and stabilization*
---