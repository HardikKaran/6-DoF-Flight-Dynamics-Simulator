"""
Equations of Motion & RK4 Integrator  (Longitudinal)
======================================================
6-state longitudinal subset: [U, W, q, Θ, xE, zE].
Lateral states (V, p, r, Φ, Ψ, yE) are zero / ignored.

State vector
------------
  idx 0 : U     body-axis forward velocity   [m/s]
  idx 1 : W     body-axis vertical velocity   [m/s]  (positive down)
  idx 2 : q     pitch rate                    [rad/s] (positive nose-up)
  idx 3 : theta pitch angle                   [rad]
  idx 4 : xE    Earth x-position (forward)    [m]
  idx 5 : zE    Earth z-position (down)        [m]   (altitude h = −zE)

Control vector
--------------
  delta_e  : elevator deflection [rad]
  throttle : 0 … 1

References
----------
  AERO50002 Chapter 3 Eq. 22, Chapter 4 (kinematics & navigation).
"""

from __future__ import annotations

import math
import numpy as np
from scipy.optimize import fsolve

from . import aircraft as ac
from .aero import compute_aero

# ── State indices ─────────────────────────────────────────────────
I_U, I_W, I_Q, I_TH, I_XE, I_ZE = range(6)
STATE_SIZE = 6

# ── Default time step (≈60 Hz) ───────────────────────────────────
DT_DEFAULT = 1.0 / 60.0


# ══════════════════════════════════════════════════════════════════
#  Derivative function
# ══════════════════════════════════════════════════════════════════

def compute_derivatives(state: np.ndarray, delta_e: float,
                        throttle: float) -> np.ndarray:
    """
    Compute the 6 time-derivatives of the longitudinal state.

    Returns ndarray of [U̇, Ẇ, q̇, Θ̇, ẋE, żE].
    """
    U     = state[I_U]
    W     = state[I_W]
    q     = state[I_Q]
    theta = state[I_TH]
    xE    = state[I_XE]
    zE    = state[I_ZE]

    altitude = -zE      # h = −zE (positive up)

    # Get forces & moments from aero model
    fm = compute_aero(U, W, q, theta, delta_e, throttle, altitude)
    X = fm["X"]
    Z = fm["Z"]
    M = fm["M"]

    # ── Translational (longitudinal, V=0, p=0, r=0) ──────────────
    U_dot = X / ac.m - W * q
    W_dot = Z / ac.m + U * q

    # ── Rotational (Ixz negligible for longitudinal) ──────────────
    q_dot = M / ac.Iyy

    # ── Euler kinematics (longitudinal: Φ=0, Ψ=0) ────────────────
    theta_dot = q

    # ── Navigation (body → Earth, 2D plane) ───────────────────────
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    xE_dot =  U * cos_t + W * sin_t
    zE_dot = -U * sin_t + W * cos_t

    return np.array([U_dot, W_dot, q_dot, theta_dot, xE_dot, zE_dot])


# ══════════════════════════════════════════════════════════════════
#  RK4 integrator
# ══════════════════════════════════════════════════════════════════

def rk4_step(state: np.ndarray, delta_e: float, throttle: float,
             dt: float = DT_DEFAULT) -> np.ndarray:
    """
    Advance the state by one RK4 time step.

    Parameters
    ----------
    state    : (6,) ndarray — current state
    delta_e  : elevator deflection [rad]
    throttle : 0 … 1
    dt       : time step [s]

    Returns
    -------
    (6,) ndarray — new state after dt
    """
    k1 = compute_derivatives(state,              delta_e, throttle)
    k2 = compute_derivatives(state + 0.5*dt*k1,  delta_e, throttle)
    k3 = compute_derivatives(state + 0.5*dt*k2,  delta_e, throttle)
    k4 = compute_derivatives(state + dt*k3,       delta_e, throttle)

    new_state = state + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)

    # ── Gimbal-lock protection ────────────────────────────────────
    new_state[I_TH] = np.clip(new_state[I_TH],
                               -math.radians(85), math.radians(85))

    # ── Ground collision clamp ────────────────────────────────────
    altitude = -new_state[I_ZE]
    if altitude <= 0.0:
        new_state[I_ZE] = 0.0                       # on the ground
        new_state[I_W]  = min(new_state[I_W], 0.0)  # no downward vel
        new_state[I_U]  = max(new_state[I_U], 0.0)  # no backward vel
        new_state[I_Q]  = 0.0                        # stop rotating
        new_state[I_TH] = max(new_state[I_TH], 0.0) # level or nose-up

    return new_state


# ══════════════════════════════════════════════════════════════════
#  Trim solver
# ══════════════════════════════════════════════════════════════════

def find_trim(V_trim: float, h_trim: float,
              alpha_guess: float = 0.03,
              de_guess: float = 0.0,
              throttle_guess: float = 0.3) -> dict:
    """
    Solve for level-flight trim: U̇ = Ẇ = q̇ = 0.

    Three unknowns:  α,  δe,  throttle
    Three equations:  X = 0,  Z = 0,  M = 0

    Parameters
    ----------
    V_trim  : desired true airspeed [m/s]
    h_trim  : desired altitude [m]

    Returns
    -------
    dict with keys:
        alpha, delta_e, throttle — trim values
        state — (6,) ndarray of the trim state vector
        converged — bool
    """

    def residuals(x):
        alpha_t, de_t, thr_t = x
        U = V_trim * math.cos(alpha_t)
        W = V_trim * math.sin(alpha_t)
        theta = alpha_t                  # level flight: γ = 0 → Θ = α
        q = 0.0

        fm = compute_aero(U, W, q, theta, de_t, thr_t, h_trim)
        # For level flight we need X=0 (no acceleration), Z=0 (no sink),
        # M=0 (no pitch accel).  We evaluate the derivatives:
        U_dot = fm["X"] / ac.m - W * q     # = fm["X"]/m
        W_dot = fm["Z"] / ac.m + U * q     # = fm["Z"]/m
        q_dot = fm["M"] / ac.Iyy
        return [U_dot, W_dot, q_dot]

    x0 = np.array([alpha_guess, de_guess, throttle_guess])
    sol, info, ier, msg = fsolve(residuals, x0, full_output=True)

    alpha_t, de_t, thr_t = sol
    U = V_trim * math.cos(alpha_t)
    W = V_trim * math.sin(alpha_t)

    trim_state = np.array([
        U,               # U
        W,               # W
        0.0,             # q
        alpha_t,         # theta = alpha for level flight
        0.0,             # xE
        -h_trim,         # zE  (altitude = −zE)
    ])

    return {
        "alpha":    alpha_t,
        "delta_e":  de_t,
        "throttle": thr_t,
        "state":    trim_state,
        "converged": ier == 1,
        "message":  msg,
    }


# ══════════════════════════════════════════════════════════════════
#  Convenience: build initial state from trim
# ══════════════════════════════════════════════════════════════════

def make_initial_state(V: float = 180.0, h: float = 9144.0) -> tuple:
    """
    Return (state, delta_e_trim, throttle_trim) for level cruise.

    Defaults to M ≈ 0.62 at 30 000 ft (≈ 9 144 m).
    """
    trim = find_trim(V, h)
    if not trim["converged"]:
        print(f"[WARN] Trim solver did not converge: {trim['message']}")
    return trim["state"], trim["delta_e"], trim["throttle"]
