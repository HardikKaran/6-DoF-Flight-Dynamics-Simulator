"""
Equations of Motion & RK4 Integrator
======================================
Implements the 6-DOF nonlinear body-axis equations of motion
(Newton–Euler) and a 4th-order Runge-Kutta time stepper.

State vector (12 elements)
--------------------------
    [U, V, W, p, q, r, Φ, Θ, Ψ, xE, yE, zE]

    U, V, W   — body-axis translational velocities  [m/s]
    p, q, r   — body-axis angular rates              [rad/s]
    Φ, Θ, Ψ   — Euler angles (roll, pitch, yaw)      [rad]
    xE, yE, zE — Earth-fixed position                 [m]
                 (zE positive DOWN; altitude h = −zE)

References
----------
- AERO50002 Lecture Notes, Chapter 3 Eq. 22 (force & moment eqs)
- AERO50002 Lecture Notes, Chapter 4 (Euler kinematic eqs, navigation)

TODO: Implement the following
------------------------------
1. compute_derivatives(state, controls) → 12-element derivative vector
   a. Translational:  U̇, V̇, Ẇ  (Eq. 22 force equations)
   b. Rotational:     ṗ, q̇, ṙ  (Eq. 22 moment equations, Ixz coupling)
   c. Euler rates:    Φ̇, Θ̇, Ψ̇  (kinematic equations)
   d. Navigation:     ẋE, ẏE, żE (body→Earth rotation)
2. rk4_step(state, controls, dt) → new state after one RK4 step
3. Trim solver — find (α_trim, δe_trim, T_trim) for level cruise
4. Gimbal-lock protection (clamp Θ near ±90° or quaternion option)
5. Ground collision check (zE ≥ 0 → clamp)
"""

import numpy as np
