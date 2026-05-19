# 3-Link Planar Manipulator (RRR): Dynamic Modeling & Robust Control

This repository contains the complete framework for the dynamic modeling and robust control of a **3-DOF RRR planar robotic manipulator**. The project includes the analytical derivation of the robot's dynamic equations using the Lagrange formulation and the implementation of a robust control strategy in **MATLAB/Simulink**.


## Project Overview

Controlling a 3-link (RRR) robot presents significant challenges due to non-linearities, cross-coupling between joints, and unmodeled dynamics. This project addresses these challenges by deriving an exact dynamic model and synthesizing a robust controller capable of tracking trajectories despite parametric uncertainties or external disturbances.

### Key Features
* **Dynamic Derivation**: Analytical calculation of the robot's Inertia ($B$), Coriolis ($C$), and Gravity ($G$) matrices.
* **Robust Control Design**: Implementation of a robust control law (e.g., Sliding Mode Control or $H_\infty$ based approaches) to guarantee trajectory tracking stability under parameter variations.
* **Simulink Simulation**: A complete block-diagram simulation integrating the highly non-linear robot dynamics with the control loop.

---

## Technologies & Tools
* **MATLAB**: Used for script-based parameter initialization and symbolic/numerical derivation of the dynamic matrices.
* **Simulink**: Used for time-domain simulation, feedback loop implementation, and system performance evaluation.

---

## Repository Structure

```text
📁 3_arm_planar_manipulator_control/
│
├── 📄 matrici_modello.m     # Derives or defines the full analytical dynamic model
├── 📄 calc_B.m             # Computes the Inertia/Mass Matrix B(q)
├── 📄 calc_C.m             # Computes the Coriolis and Centrifugal Matrix C(q, q_dot)
├── 📄 calc_G.m             # Computes the Gravity Vector G(q)
│
├── 📄 init_robot.m         # Main setup script (loads robot parameters, gains, and trajectories)
├── 📄 controllo_v1.slx     # Core Simulink architecture combining robot dynamics and robust control
│
├── 📄 .gitignore           # Standard exclusions for MATLAB cache (slprj/, *.slxc)
└── 📄 README.md            # Project documentation
```

## Mathematical & Control Framework
### Dynamic Model
The equations of motion of the 3-link planar manipulator are expressed in the joint space as:

$$B(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau + \tau_d$$

Where:
* $q, \dot{q}, \ddot{q}$ represent the joint positions, velocities, and accelerations.
* $\tau$ is the control torque vector input.
* $\tau_d$ represents external disturbances and uncertainties.

### Robust Control Strategy
The synthesized controller ensures that the tracking error $e(t) = q_d(t) - q(t)$ asymptotically converges to zero or remains bounded within a strict threshold, even in the presence of bounds on parameter variations (e.g., payload mass uncertainty).

## Author
* **Paolo Danza**