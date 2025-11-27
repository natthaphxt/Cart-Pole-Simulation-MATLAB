# Cart-Pole Swing-Up and Stabilization System

**FRA333 Robot Kinematics**

**Student ID:**
66340500006
66340500074
66340500077

---

## Abstract

This project implements a hybrid control system for the cart-pole (inverted pendulum) using MATLAB/Simulink. The system combines an energy-based swing-up controller with LQR stabilization. The swing-up controller brings the pendulum from its downward position to near-upright, then automatically switches to LQR control for balancing. The system achieves stable inverted balance within approximately 5 seconds.

---

## 1. Objectives

1. Develop an energy-based swing-up controller to bring the pendulum to the upright position
2. Design an LQR controller for stabilization at the inverted equilibrium
3. Implement automatic mode switching between swing-up and stabilization
4. Validate the system through Simulink simulation

---

## 2. System Description

### Physical Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Cart Mass | M | 0.3 | kg |
| Pole Mass | m | 0.15 | kg |
| Pole Half-Length | L | 0.25 | m |
| Gravity | g | 9.81 | m/s² |
| Damping | b | 0.5 | N·s/m |

### Angle Convention
- **θ = 0°:** Pendulum upright (target)
- **θ = 180°:** Pendulum hanging down (initial)

---

## 3. Methodology

### Control Strategy

The system uses two controllers with automatic switching:

#### 1. Energy-Based Swing-Up Control

The swing-up controller uses the concept of energy manipulation to bring the pendulum from hanging down to upright position.

**Principle:** The total mechanical energy of the pendulum at the upright position is:

```
E_target = m × g × L (potential energy at top)
```

The controller calculates the current energy and compares it to the target:

```
E_current = (1/2) × m × L² × θ̇² + m × g × L × (1 - cos(θ))
E_error = E_current - E_target
```

**Control Law:**
```
u = k × E_error × sign(θ̇ × cos(θ))
```

This strategy:
- Adds energy when pendulum swings toward upright (same direction as motion)
- Removes energy if overshooting (opposite direction)
- Naturally pumps the pendulum higher with each swing until reaching the target energy

#### 2. LQR (Linear Quadratic Regulator) Stabilization

LQR is an optimal control method that finds the best feedback gains to minimize a cost function.

**Cost Function:**
```
J = ∫(xᵀQx + uᵀRu)dt
```

Where:
- **Q matrix** - penalizes state deviations (how far from upright)
- **R value** - penalizes control effort (how much force used)

**Control Law:**
```
u = -Kx = -[K₁, K₂, K₃, K₄] × [x, θ, ẋ, θ̇]ᵀ
```

The gain matrix K is computed by solving the Riccati equation, which MATLAB's `lqr()` function handles automatically.

**Tuning Interpretation:**
| Parameter | Effect |
|-----------|--------|
| Increase Q₂₂ (angle weight) | Tighter angle control, less wobble |
| Decrease R | More aggressive control, faster response |
| Increase Q₁₁ (position weight) | Keeps cart closer to center |

**Why LQR for stabilization?**
- Optimal balance between performance and control effort
- Works well for linear systems near equilibrium
- Easy to tune via Q and R matrices

### Linearized Model

State vector: **x = [x, θ, ẋ, θ̇]ᵀ**

```matlab
A = [0 0 1 0; 0 0 0 1; 0 3.27 -1.48 0; 0 19.62 -2.22 0];
B = [0; 0; 2.96; 4.444];
```

### LQR Design

```matlab
Q = diag([1, 3, 1, 1]);  % State weights (angle prioritized)
R = 0.1;                  % Control effort weight
K = lqr(A, B, Q, R);
```

### Switching Conditions

Mode switches to LQR when both conditions are met:
- |θ| < 0.3 rad (≈17.2°)
- |ω| < 1.5 rad/s

---

## 4. Implementation

### Requirements
- MATLAB R2024b+
- Simulink
- Control System Toolbox

### Files

| File | Description |
|------|-------------|
| `LQR_Pendulum.m` | Defines parameters and computes LQR gain |
| `CartPole.slx` | Simulink model with controllers and dynamics |

### How to Run

```matlab
run('LQR_Pendulum.m')    % Load parameters
open('CartPole.slx')      % Open model
% Click Run in Simulink
```

---

## 5. Experimental Results

### Simulation Phases

| Phase | Time | Behavior |
|-------|------|----------|
| Initial | 0-2s | Swing-up begins |
| Swing-Up | 2-5s | Increasing oscillations |
| Transition | ~5s | Switch to LQR |
| Stabilization | >5s | Balanced inverted |

### Performance

| Metric | Result |
|--------|--------|
| Swing-up time | ~3-5 s |
| Settling time | < 2 s |
| Steady-state error | < 0.01 rad |

### Simulation Video

[![Demo Video](./demo.webp)](https://youtu.be/dQTII64_-kc)

*Video: Cart-pole swing-up and stabilization demonstration*

### Simulation Plots

*[Insert plots: angle vs time, cart position vs time, control input vs time]*

---

## 6. Conclusion

The hybrid control system successfully demonstrates:
- Energy-based swing-up from hanging position
- Smooth automatic transition to LQR stabilization
- Stable inverted pendulum balance

This project illustrates key control concepts: energy-based control, optimal control (LQR), and hybrid switching systems.

---

## References

1. Anderson, B. D., & Moore, J. B. (1990). *Optimal Control: Linear Quadratic Methods*
2. Åström, K. J., & Furuta, K. (2000). Swinging up a pendulum by energy control. *Automatica*, 36(2), 287-295.
3. Ogata, K. (2010). *Modern Control Engineering*
4. ME389 MEM04 PendulumGantry Guideline
---

## Appendix: MATLAB Code

```matlab
% LQR_Pendulum.m
M = 0.3; m = 0.15; L = 0.25; g = 9.81; b = 0.5;

A = [0 0 1 0; 0 0 0 1; 0 3.27 -1.48 0; 0 19.62 -2.22 0];
B = [0; 0; 2.96; 4.444];

Q = diag([1, 3, 1, 1]);
R = 0.1;
K = lqr(A, B, Q, R);

theta_threshold = 0.3;  % rad
omega_threshold = 1.5;  % rad/s
```
