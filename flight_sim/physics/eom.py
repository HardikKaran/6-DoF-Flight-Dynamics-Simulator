"""
Equations of Motion & RK4 Integrator  (Full 6-DOF)
=====================================================
12-state vector: [U, V, W, p, q, r, Φ, Θ, Ψ, xE, yE, zE].

State vector
------------
  idx  0 : U     body-axis forward velocity   [m/s]
  idx  1 : V     body-axis lateral velocity    [m/s]  (positive starboard)
  idx  2 : W     body-axis vertical velocity   [m/s]  (positive down)
  idx  3 : p     roll rate                     [rad/s] (positive right-wing-down)
  idx  4 : q     pitch rate                    [rad/s] (positive nose-up)
  idx  5 : r     yaw rate                      [rad/s] (positive nose-right)
  idx  6 : phi   bank angle                    [rad]   (positive right-wing-down)
  idx  7 : theta pitch angle                   [rad]   (positive nose-up)
  idx  8 : psi   heading angle                 [rad]   (positive clockwise / nose-right)
  idx  9 : xE    Earth x-position (north)      [m]
  idx 10 : yE    Earth y-position (east)       [m]
  idx 11 : zE    Earth z-position (down)       [m]     (altitude h = −zE)

Control vector
--------------
  delta_e  : elevator deflection [rad]
  delta_a  : aileron  deflection [rad]
  delta_r  : rudder   deflection [rad]
  throttle : 0 … 1

References
----------
  Nelson, R.C., "Flight Stability and Automatic Control", 2nd Ed.,
    McGraw-Hill, 1998 — Ch. 2–4.
      Force equations:    Eq. (2.19a–c) → U̇, V̇, Ẇ
      Moment equations:   Eq. (2.22a–c) with Ixz coupling
      Euler kinematics:   Eq. (2.30a–c) → Φ̇, Θ̇, Ψ̇
      Navigation:         Eq. (2.33) → body-to-Earth DCM

  Stevens, B.L., Lewis, F.L. & Johnson, E.N., "Aircraft Control and
    Simulation", 3rd Ed., Wiley, 2016 — Ch. 1, Eqs. (1.3-17)–(1.3-22).

  Etkin, B. & Reid, L.D., "Dynamics of Flight: Stability and Control",
    3rd Ed., Wiley, 1996 — Ch. 4.

  Cook, M.V., "Flight Dynamics Principles", 3rd Ed., Butterworth-
    Heinemann, 2013 — Ch. 3–4.
"""

from __future__ import annotations

import math
import numpy as np
from scipy.optimize import fsolve

from . import aircraft as ac
from .aero import compute_aero

# ── State indices ─────────────────────────────────────────────────
I_U, I_V, I_W, I_P, I_Q, I_R = range(6)
I_PHI, I_TH, I_PSI, I_XE, I_YE, I_ZE = range(6, 12)
STATE_SIZE = 12

# ── Default time step (≈60 Hz) ───────────────────────────────────
DT_DEFAULT = 1.0 / 60.0


# ══════════════════════════════════════════════════════════════════
#  Derivative function  (full 6-DOF)
# ══════════════════════════════════════════════════════════════════

def compute_derivatives(state: np.ndarray, delta_e: float,
                        delta_a: float, delta_r: float,
                        throttle: float) -> np.ndarray:
    """
    Compute the 12 time-derivatives of the full 6-DOF state.

    Returns ndarray of shape (12,):
      [U̇, V̇, Ẇ, ṗ, q̇, ṙ, Φ̇, Θ̇, Ψ̇, ẋE, ẏE, żE]
    """
    U     = state[I_U]
    V     = state[I_V]
    W     = state[I_W]
    p     = state[I_P]
    q     = state[I_Q]
    r     = state[I_R]
    phi   = state[I_PHI]
    theta = state[I_TH]
    psi   = state[I_PSI]

    altitude = -state[I_ZE]      # h = −zE (positive up)

    # ── Get forces & moments from aero model ──────────────────────
    fm = compute_aero(U, V, W, p, q, r, phi, theta,
                      delta_e, delta_a, delta_r, throttle, altitude)
    X     = fm["X"]
    Y     = fm["Y"]
    Z     = fm["Z"]
    L_lat = fm["L_lat"]   # rolling moment
    M_pit = fm["M"]        # pitching moment
    N_yaw = fm["N"]        # yawing moment

    # ══════════════════════════════════════════════════════════════
    #  TRANSLATIONAL DYNAMICS  (Nelson Eq. 2.19a–c)
    #    X = m(U̇ + qW − rV)
    #    Y = m(V̇ + rU − pW)
    #    Z = m(Ẇ + pV − qU)
    # ══════════════════════════════════════════════════════════════
    U_dot = X / ac.m + r * V - q * W
    V_dot = Y / ac.m - r * U + p * W
    W_dot = Z / ac.m + q * U - p * V

    # ══════════════════════════════════════════════════════════════
    #  ROTATIONAL DYNAMICS with Ixz coupling
    #  (Nelson Eq. 2.22a–c, rearranged — see §5.6 of TODO)
    #
    #  The coupled roll/yaw equations (Ixy = Iyz = 0, Ixz ≠ 0):
    #    L = Ixx·ṗ − Ixz·ṙ  + qr(Izz − Iyy) − Ixz·pq
    #    M = Iyy·q̇           + rp(Ixx − Izz)  + Ixz(p² − r²)
    #    N = −Ixz·ṗ + Izz·ṙ + pq(Iyy − Ixx) + Ixz·qr
    #
    #  Solve the 2×2 system for ṗ, ṙ:
    #    Γ = Ixx·Izz − Ixz²
    #    L' = L − qr(Izz − Iyy) + Ixz·pq
    #    N' = N − pq(Iyy − Ixx) − Ixz·qr
    #    ṗ = (Izz·L' + Ixz·N') / Γ
    #    ṙ = (Ixx·N' + Ixz·L') / Γ
    # ══════════════════════════════════════════════════════════════
    Ixx = ac.Ixx
    Iyy = ac.Iyy
    Izz = ac.Izz
    Ixz = ac.Ixz

    Gamma = Ixx * Izz - Ixz * Ixz

    L_prime = L_lat - q * r * (Izz - Iyy) + Ixz * p * q
    N_prime = N_yaw - p * q * (Iyy - Ixx) - Ixz * q * r

    p_dot = (Izz * L_prime + Ixz * N_prime) / Gamma
    r_dot = (Ixx * N_prime + Ixz * L_prime) / Gamma

    # Pitch (uncoupled in the Ixy=Iyz=0 assumption):
    #   M = Iyy·q̇ + rp(Ixx − Izz) + Ixz(p² − r²)
    q_dot = (M_pit - r * p * (Ixx - Izz) - Ixz * (p * p - r * r)) / Iyy

    # ══════════════════════════════════════════════════════════════
    #  EULER KINEMATIC EQUATIONS  (Nelson Eq. 2.30a–c)
    #    Φ̇ = p + (q·sinΦ + r·cosΦ)·tanΘ
    #    Θ̇ = q·cosΦ − r·sinΦ
    #    Ψ̇ = (q·sinΦ + r·cosΦ) / cosΘ
    # ══════════════════════════════════════════════════════════════
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_th  = math.sin(theta)
    cos_th  = math.cos(theta)
    tan_th  = sin_th / max(abs(cos_th), 1e-10) * (1 if cos_th >= 0 else -1)

    q_sinPhi_r_cosPhi = q * sin_phi + r * cos_phi

    phi_dot   = p + q_sinPhi_r_cosPhi * tan_th
    theta_dot = q * cos_phi - r * sin_phi
    psi_dot   = q_sinPhi_r_cosPhi / max(abs(cos_th), 1e-10) * (1 if cos_th >= 0 else -1)

    # ══════════════════════════════════════════════════════════════
    #  NAVIGATION EQUATIONS  (Nelson Eq. 2.33)
    #  Body-to-Earth rotation using the Direction Cosine Matrix:
    #
    #  ┌ ẋE ┐   ┌ cΘcΨ   sΦsΘcΨ−cΦsΨ   cΦsΘcΨ+sΦsΨ ┐ ┌ U ┐
    #  │ ẏE │ = │ cΘsΨ   sΦsΘsΨ+cΦcΨ   cΦsΘsΨ−sΦcΨ │ │ V │
    #  └ żE ┘   └ −sΘ    sΦcΘ           cΦcΘ         ┘ └ W ┘
    # ══════════════════════════════════════════════════════════════
    sin_psi = math.sin(psi)
    cos_psi = math.cos(psi)

    xE_dot = (cos_th * cos_psi * U
              + (sin_phi * sin_th * cos_psi - cos_phi * sin_psi) * V
              + (cos_phi * sin_th * cos_psi + sin_phi * sin_psi) * W)

    yE_dot = (cos_th * sin_psi * U
              + (sin_phi * sin_th * sin_psi + cos_phi * cos_psi) * V
              + (cos_phi * sin_th * sin_psi - sin_phi * cos_psi) * W)

    zE_dot = (-sin_th * U
              + sin_phi * cos_th * V
              + cos_phi * cos_th * W)

    return np.array([U_dot, V_dot, W_dot, p_dot, q_dot, r_dot,
                     phi_dot, theta_dot, psi_dot,
                     xE_dot, yE_dot, zE_dot])


# ══════════════════════════════════════════════════════════════════
#  RK4 integrator
# ══════════════════════════════════════════════════════════════════

def rk4_step(state: np.ndarray, delta_e: float,
             delta_a: float, delta_r: float,
             throttle: float, dt: float = DT_DEFAULT) -> np.ndarray:
    """
    Advance the 12-state vector by one RK4 time step.

    Parameters
    ----------
    state    : (12,) ndarray — current state
    delta_e  : elevator deflection [rad]
    delta_a  : aileron  deflection [rad]
    delta_r  : rudder   deflection [rad]
    throttle : 0 … 1
    dt       : time step [s]

    Returns
    -------
    (12,) ndarray — new state after dt
    """
    k1 = compute_derivatives(state,              delta_e, delta_a, delta_r, throttle)
    k2 = compute_derivatives(state + 0.5*dt*k1,  delta_e, delta_a, delta_r, throttle)
    k3 = compute_derivatives(state + 0.5*dt*k2,  delta_e, delta_a, delta_r, throttle)
    k4 = compute_derivatives(state + dt*k3,       delta_e, delta_a, delta_r, throttle)

    new_state = state + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)

    # ── Angle wrapping ────────────────────────────────────────────
    # Keep Φ in [−π, π] and Ψ in [0, 2π]
    new_state[I_PHI] = _wrap_angle(new_state[I_PHI])
    new_state[I_PSI] = new_state[I_PSI] % (2.0 * math.pi)

    # ── Gimbal-lock protection ────────────────────────────────────
    new_state[I_TH] = np.clip(new_state[I_TH],
                               -math.radians(85), math.radians(85))

    # ── Ground collision clamp ────────────────────────────────────
    altitude = -new_state[I_ZE]
    if altitude <= 0.0:
        new_state[I_ZE]  = 0.0                        # on the ground
        new_state[I_W]   = min(new_state[I_W], 0.0)   # no downward vel
        new_state[I_U]   = max(new_state[I_U], 0.0)   # no backward vel
        new_state[I_V]   = 0.0                         # no sideslip on ground
        new_state[I_P]   = 0.0                         # stop rolling
        new_state[I_Q]   = 0.0                         # stop pitching
        new_state[I_R]   = 0.0                         # stop yawing
        new_state[I_PHI] = 0.0                         # wings level
        new_state[I_TH]  = max(new_state[I_TH], 0.0)  # level or nose-up

    return new_state


def _wrap_angle(a: float) -> float:
    """Wrap angle to [−π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ══════════════════════════════════════════════════════════════════
#  Trim solver  (longitudinal — lateral states set to zero)
# ══════════════════════════════════════════════════════════════════

def find_trim(V_trim: float, h_trim: float,
              alpha_guess: float = 0.03,
              de_guess: float = 0.0,
              throttle_guess: float = 0.3) -> dict:
    """
    Solve for level-flight trim: U̇ = Ẇ = q̇ = 0.

    Three unknowns:  α,  δe,  throttle
    Three equations:  X = 0,  Z = 0,  M = 0

    Lateral states are assumed zero (symmetric, wings-level flight).

    Parameters
    ----------
    V_trim  : desired true airspeed [m/s]
    h_trim  : desired altitude [m]

    Returns
    -------
    dict with keys:
        alpha, delta_e, throttle — trim values
        state — (12,) ndarray of the trim state vector
        converged — bool
    """

    def residuals(x):
        alpha_t, de_t, thr_t = x
        U = V_trim * math.cos(alpha_t)
        W = V_trim * math.sin(alpha_t)
        V_body = 0.0
        theta = alpha_t          # level flight: γ = 0 → Θ = α
        phi = 0.0
        p = q = r = 0.0

        fm = compute_aero(U, V_body, W, p, q, r, phi, theta,
                          de_t, 0.0, 0.0, thr_t, h_trim)
        U_dot = fm["X"] / ac.m
        W_dot = fm["Z"] / ac.m
        q_dot = fm["M"] / ac.Iyy
        return [U_dot, W_dot, q_dot]

    x0 = np.array([alpha_guess, de_guess, throttle_guess])
    sol, info, ier, msg = fsolve(residuals, x0, full_output=True)

    alpha_t, de_t, thr_t = sol
    U = V_trim * math.cos(alpha_t)
    W = V_trim * math.sin(alpha_t)

    trim_state = np.zeros(STATE_SIZE)
    trim_state[I_U]  = U
    trim_state[I_W]  = W
    trim_state[I_TH] = alpha_t   # theta = alpha for level flight
    trim_state[I_ZE] = -h_trim   # zE  (altitude = −zE)

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
