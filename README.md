# Cart-Pole Swing-Up and Stabilization System

## Overview

This project implements a **swing-up and stabilization controller** for an inverted pendulum on a cart (cart-pole system). The system combines two control strategies:

1. **Swing-up controller**: Energizes the pendulum from its downward hanging position to near the upright position
2. **LQR stabilization controller**: Maintains the pendulum in the upright inverted position once it reaches the target region

The controller automatically switches between these two modes based on the pendulum's angle and angular velocity, creating a complete solution for both bringing up and balancing the inverted pendulum.

---

## System Description

### Physical Parameters

| Parameter | Symbol | Value | Unit | Description |
|-----------|--------|-------|------|-------------|
| Cart Mass | M | 0.3 | kg | Mass of the cart |
| Pole Mass | m | 0.15 | kg | Mass of the pendulum |
| Pole Half-Length | L | 0.25 | m | Distance from pivot to center of mass |
| Gravity | g | 9.81 | m/s² | Gravitational acceleration |
| Damping Coefficient | b | 0.5 | N·s/m | Friction/damping in the system |

**Note**: The pole length parameter `L` represents the half-length (distance to center of mass). The full physical length of the pole is **0.5 m**.

### Angle Convention

- **θ = 0°**: Pendulum pointing upward (inverted, stable equilibrium target)
- **θ = 180°**: Pendulum hanging downward (initial position)

---

## Control Strategy

### 1. Swing-Up Phase

When the pendulum starts in the downward position, a swing-up controller applies energy-based control to pump energy into the system. This controller:
- Calculates the current energy of the pendulum
- Compares it to the target energy (upright position)
- Applies forces to increase/decrease energy as needed
- Continues until the pendulum approaches the upright position

### 2. Stabilization Phase (LQR Control)

Once the pendulum enters a region near the upright position (within threshold bounds), the system switches to an **LQR (Linear Quadratic Regulator)** controller designed to stabilize the system at the inverted position.

#### Switching Thresholds

The controller switches from swing-up to stabilization when:
- **|θ| < 0.3 rad** (≈ 17.2°) - pendulum is close to upright
- **|ω| < 1.5 rad/s** - angular velocity is sufficiently low

These thresholds define a "capture region" where the linear LQR controller can effectively stabilize the system.

---

## LQR Controller Design

### Linearized State-Space Model

The system is linearized around the upright equilibrium point (θ = 0). The state vector is:

**x = [x, θ, ẋ, θ̇]ᵀ**

Where:
- `x`: Cart position (m)
- `θ`: Pole angle from upright (rad)
- `ẋ`: Cart velocity (m/s)
- `θ̇`: Pole angular velocity (rad/s)

#### State-Space Matrices

```matlab
A = [0    0     1     0   ;
     0    0     0     1   ;
     0    3.27  -1.48 0   ;
     0    19.62 -2.22 0   ];

B = [0    ;
     0    ;
     2.96 ;
     4.444];
```

These matrices represent the linearized dynamics: **ẋ = Ax + Bu**

### Control Law

The LQR controller computes the control input (force on cart) as:

**u = -Kx**

Where **K** is the optimal feedback gain matrix computed by solving the Riccati equation with the specified weighting matrices.

### Tuning Parameters

#### Q Matrix (State Weighting)
```matlab
Q = diag([1, 3, 1, 1])
```

The Q matrix penalizes deviations in each state:
- **Q₁₁ = 1**: Cart position penalty
- **Q₂₂ = 3**: Angle penalty (higher weight → prioritize keeping upright)
- **Q₃₃ = 1**: Cart velocity penalty
- **Q₄₄ = 1**: Angular velocity penalty

#### R Scalar (Control Effort Weighting)
```matlab
R = 0.1
```

The R value penalizes control effort (force applied). Lower R allows more aggressive control action.

**Tuning Guideline**: 
- Increase Q₂₂ to improve angular stability (tighter angle control)
- Decrease R to allow stronger control forces
- Balance Q and R to achieve desired performance vs. control effort trade-off

---

## Files Description

### 1. `LQR_Pendulun.m`

MATLAB script that:
- Defines physical system parameters
- Specifies the linearized state-space model (A, B matrices)
- Designs the LQR controller by computing optimal gain K
- Sets switching thresholds for controller transition

**Usage**: Run this script before running the Simulink simulation to load parameters and compute the LQR gain matrix into the MATLAB workspace.

### 2. `CartPole_2024.slx`

Simulink model that:
- Implements the complete nonlinear cart-pole dynamics
- Contains swing-up controller logic
- Contains LQR stabilization controller
- Includes switching logic based on angle/velocity thresholds
- Provides visualization and data logging

**Usage**: Open in Simulink after running the MATLAB script, then run the simulation.

---

## Getting Started

### Prerequisites

- MATLAB R2024b or higher
- Simulink
- Control System Toolbox (for `lqr()` function)

### Running the Simulation

1. **Load Controller Parameters**:
   ```matlab
   run('LQR_Pendulun.m')
   ```
   
   This will:
   - Load all physical parameters into workspace
   - Compute the LQR gain matrix K
   - Set threshold values

2. **Open Simulink Model**:
   ```matlab
   open('CartPole_2024.slx')
   ```

3. **Run Simulation**:
   - Click the "Run" button in Simulink
   - Observe the pendulum swing up from the downward position
   - Watch the controller switch to stabilization mode
   - Monitor the cart position and pendulum angle

### Expected Behavior

1. **Initial Phase (t = 0-2s)**: Pendulum hangs downward, swing-up controller begins pumping energy
2. **Swing-Up Phase (t = 2-5s)**: Pendulum oscillates with increasing amplitude
3. **Transition (t ≈ 5s)**: Pendulum enters capture region, system switches to LQR control
4. **Stabilization Phase (t > 5s)**: LQR controller maintains inverted position with minimal oscillation

---

## Controller Tuning Guide

### Improving Stabilization Performance

**To reduce angle oscillations**:
```matlab
Q = diag([1, 10, 1, 1]);  % Increase Q₂₂
```

**To allow faster response**:
```matlab
R = 0.05;  % Decrease R (more aggressive control)
```

**To prioritize cart position**:
```matlab
Q = diag([5, 3, 1, 1]);  % Increase Q₁₁
```

### Adjusting Switching Thresholds

**For earlier switching (wider capture region)**:
```matlab
theta_threshold = 0.4;   % Increase angle threshold
omega_threshold = 2.0;   % Increase velocity threshold
```

**For later switching (more conservative)**:
```matlab
theta_threshold = 0.2;   % Decrease angle threshold
omega_threshold = 1.0;   % Decrease velocity threshold
```

**Trade-off**: Wider capture region may lead to LQR failure if pendulum enters with too much energy; narrower region may cause missed transitions.

---

## Technical Notes

### State-Space Derivation

The A and B matrices were derived using **Lagrangian mechanics**:

1. Define kinetic and potential energy of the system
2. Formulate the Lagrangian L = T - V
3. Apply Euler-Lagrange equations to get equations of motion
4. Linearize around the upright equilibrium (θ = 0, ẋ = 0)
5. Convert to state-space form

The specific numerical values in the matrices depend on the physical parameters (M, m, L, g, b).

### System Controllability

The system is **fully controllable** at the upright position, meaning the LQR controller can theoretically drive all states to zero from any initial condition (within the linear region). This can be verified by checking that the controllability matrix has full rank:

```matlab
Co = ctrb(A, B);
rank(Co)  % Should equal 4 (number of states)
```

### Stability Analysis

The open-loop system (without control) is **unstable** at the upright position, as indicated by positive eigenvalues in the A matrix. The LQR controller places the closed-loop poles in the left half-plane, ensuring exponential stability.

---

## Common Issues and Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Pendulum doesn't swing up | Swing-up gains too low | Increase energy control gain |
| Controller switches too early | Thresholds too large | Reduce theta/omega thresholds |
| Oscillations after stabilization | Q₂₂ too low or R too high | Increase Q₂₂ or decrease R |
| Cart drifts away | Q₁₁ too low | Increase cart position weight |
| Control saturates | R too low or Q too high | Increase R or decrease Q |

---

## Future Enhancements

Potential improvements to the system:

1. **Adaptive switching**: Use energy-based criteria instead of fixed thresholds
2. **Cart position control**: Add reference tracking to return cart to center
3. **Nonlinear control**: Implement nonlinear stabilization (e.g., sliding mode)
4. **Disturbance rejection**: Test robustness with external disturbances
5. **Experimental validation**: Implement on physical hardware

---

## References

- **LQR Theory**: Anderson, B. D., & Moore, J. B. (1990). Optimal Control: Linear Quadratic Methods
- **Cart-Pole Dynamics**: Ogata, K. (2010). Modern Control Engineering
- **Swing-Up Control**: Åström, K. J., & Furuta, K. (2000). Swinging up a pendulum by energy control

---

## License

This project is provided for educational and research purposes.

---

## Author Notes

This implementation demonstrates fundamental concepts in control theory:
- **Energy-based control** for swing-up
- **Optimal control (LQR)** for stabilization  
- **Hybrid control** combining multiple strategies
- **Linearization** of nonlinear systems

The cart-pole system is a classic benchmark in control engineering, illustrating the challenges of stabilizing unstable equilibria and the power of modern control techniques.