"""
Aerodynamic Force & Moment Model  (Longitudinal)
==================================================
Three-surface configuration: canard + wing + horizontal tail.
All outputs in BODY axes, consistent with the EOM sign conventions.

Coordinate system
-----------------
  x — forward (out the nose)
  z — downward (body axis, positive down)

Sign conventions
-----------------
  α   positive nose up
  q   positive nose up
  δe  positive trailing-edge down → increases tail lift → nose-down moment
  Θ   positive nose up

References
----------
  AERO50002 Lectures, Chapters 4 & 8; Tutorial 2 data sheets.
"""

from __future__ import annotations

import math
import numpy as np

from . import aircraft as ac
from .atmosphere import density, speed_of_sound

# ── tiny guard to avoid division by zero ─────────────────────────
_EPS = 1e-6


# ══════════════════════════════════════════════════════════════════
#  Precomputed geometry  (moment arms from CG)
# ══════════════════════════════════════════════════════════════════
_l_C = ac.canard.x_ac - ac.x_CG   # canard arm (negative → forward of CG)
_l_W = ac.wing.x_ac   - ac.x_CG   # wing arm   (small positive)
_l_H = ac.tail.x_ac   - ac.x_CG   # tail arm   (positive → aft of CG)

# Vertical offsets of thrust line and surfaces from CG
_dz_T = ac.z_T  - ac.z_CG         # thrust line above CG (positive → above)
_dz_C = ac.canard.z_ac - ac.z_CG
_dz_W = ac.wing.z_ac   - ac.z_CG
_dz_H = ac.tail.z_ac   - ac.z_CG

# Tail volume ratio (useful for pitch damping estimate)
_V_H = (ac.tail.S * _l_H) / (ac.S_ref * ac.c_ref)


# ══════════════════════════════════════════════════════════════════
#  Helper: local effective angle of attack for each surface
# ══════════════════════════════════════════════════════════════════
def _surface_alpha(alpha_body: float, surface, upwash_sign: float = 0.0) -> float:
    """
    Effective AoA seen by a lifting surface.

    α_eff = α_body + i − α_0 + upwash_sign · (dε/dα) · α_body

    upwash_sign:
      -1 for surfaces in downwash   (tail)
      +1 for surfaces in upwash     (canard, in the wing's upwash field)
       0 for the wing itself
    """
    return (alpha_body * (1.0 + upwash_sign * surface.de_da)
            + surface.i
            - surface.alpha_0)


# ══════════════════════════════════════════════════════════════════
#  Public API
# ══════════════════════════════════════════════════════════════════

def compute_aero(U: float, W: float, q: float, theta: float,
                 delta_e: float, throttle: float,
                 altitude: float) -> dict:
    """
    Compute aerodynamic + thrust forces and moments (longitudinal).

    Parameters
    ----------
    U, W      : body-axis velocities [m/s]
    q         : pitch rate [rad/s]
    theta     : pitch angle [rad]
    delta_e   : elevator deflection [rad], positive TED
    throttle  : 0 … 1
    altitude  : geometric altitude h [m]  (= −zE)

    Returns
    -------
    dict with keys:
        X, Z      — total body-axis forces [N]
        M         — total pitching moment about CG [N·m]
        CL, CD    — total lift / drag coefficients
        alpha     — angle of attack [rad]
        V_T       — true airspeed [m/s]
        q_bar     — dynamic pressure [Pa]
        T         — thrust [N]
        L_aero    — total lift force [N]
        D_aero    — total drag force [N]
    """

    # ── Airspeed & angle of attack ────────────────────────────────
    V_T   = math.sqrt(U * U + W * W + _EPS)
    alpha = math.atan2(W, U)

    # ── Atmosphere ────────────────────────────────────────────────
    rho   = density(max(altitude, 0.0))
    q_bar = 0.5 * rho * V_T * V_T

    # ── Per-surface lift coefficients ─────────────────────────────
    # Wing
    alpha_W = _surface_alpha(alpha, ac.wing, upwash_sign=0.0)
    CL_W    = ac.wing.a * alpha_W

    # Canard  (in upwash field of wing → upwash_sign = +1)
    alpha_C = _surface_alpha(alpha, ac.canard, upwash_sign=+1.0)
    CL_C    = ac.canard.a * alpha_C

    # Horizontal tail  (in downwash of wing → upwash_sign = -1)
    # Also receives elevator contribution
    alpha_H = _surface_alpha(alpha, ac.tail, upwash_sign=-1.0)
    CL_H    = ac.tail.a * alpha_H + ac.a_E * delta_e

    # ── Pitch-damping contribution to tail lift ───────────────────
    # The tail sees an additional angle due to pitch rate:
    #   Δα_H = −q · l_H / V_T   (tail aft of CG, pitch-up → reduced AoA at tail)
    if V_T > _EPS:
        delta_alpha_q = -q * _l_H / V_T
        CL_H += ac.tail.a * delta_alpha_q

    # ── Total lift coefficient (area-weighted) ────────────────────
    CL_total = (CL_W * ac.wing.S
                + CL_C * ac.canard.S
                + CL_H * ac.tail.S) / ac.S_ref

    # ── Drag (component build-up) ────────────────────────────────
    CD_W = ac.wing.k   * CL_W * CL_W / (math.pi * ac.wing.AR)
    CD_C = ac.CD0_C + ac.canard.k * CL_C * CL_C / (math.pi * ac.canard.AR)
    CD_H = ac.CD0_H + ac.tail.k   * CL_H * CL_H / (math.pi * ac.tail.AR)

    CD_total = (ac.CD0
                + CD_W * ac.wing.S / ac.S_ref
                + CD_C * ac.canard.S / ac.S_ref
                + CD_H * ac.tail.S / ac.S_ref)

    # ── Lift & drag forces [N] in wind axes ───────────────────────
    L_aero = q_bar * ac.S_ref * CL_total
    D_aero = q_bar * ac.S_ref * CD_total

    # ── Rotate wind → body axes ───────────────────────────────────
    sin_a = math.sin(alpha)
    cos_a = math.cos(alpha)
    Xa =  L_aero * sin_a - D_aero * cos_a
    Za = -L_aero * cos_a - D_aero * sin_a

    # ── Thrust ────────────────────────────────────────────────────
    T = _compute_thrust(throttle, V_T)

    # ── Gravity in body axes ──────────────────────────────────────
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    Xg = -ac.m * ac.g * sin_t
    Zg =  ac.m * ac.g * cos_t

    # ── Total body-axis forces ────────────────────────────────────
    X = Xa + Xg + T       # thrust along body x-axis
    Z = Za + Zg

    # ── Pitching moment about CG ─────────────────────────────────
    # Each surface: M_surf = (lift_surf · l_surf)  (positive nose-up)
    # Plus zero-lift moment contributions.
    M_W = (q_bar * ac.wing.S   * ac.wing.c_bar   * ac.wing.CM0
            + q_bar * ac.wing.S   * CL_W * _l_W)
    M_C = (q_bar * ac.canard.S * ac.canard.c_bar * ac.canard.CM0
            + q_bar * ac.canard.S * CL_C * _l_C)
    M_H = (q_bar * ac.tail.S   * ac.tail.c_bar   * ac.tail.CM0
            + q_bar * ac.tail.S   * CL_H * _l_H)

    # Fuselage contribution
    M_fus = q_bar * ac.S_ref * ac.c_ref * ac.dCM_da_fus * alpha

    # Thrust moment (z-offset of thrust line from CG)
    M_thrust = T * _dz_T

    # Pitch damping (dimensional):  M_q ≈ −(1/2)ρV · a_H · S_H · l_H² · (q/V)
    # Simplified from non-dimensional CM_q:
    if V_T > _EPS:
        CM_q_eff = -2.0 * ac.tail.a * _V_H * (_l_H / ac.c_ref)
        M_q_damp = q_bar * ac.S_ref * ac.c_ref * CM_q_eff * (q * ac.c_ref / (2.0 * V_T))
    else:
        M_q_damp = 0.0

    M_total = M_W + M_C + M_H + M_fus + M_thrust + M_q_damp

    return {
        "X":       X,
        "Z":       Z,
        "M":       M_total,
        "CL":      CL_total,
        "CD":      CD_total,
        "alpha":   alpha,
        "V_T":     V_T,
        "q_bar":   q_bar,
        "T":       T,
        "L_aero":  L_aero,
        "D_aero":  D_aero,
    }


# ══════════════════════════════════════════════════════════════════
#  Thrust model  (turboprop)
# ══════════════════════════════════════════════════════════════════
_ETA_PROP = 0.85          # propeller efficiency (typical for cruise)
_V_MIN_THRUST = 30.0      # m/s — below this, cap T at static value

def _compute_thrust(throttle: float, V: float) -> float:
    """
    Turboprop thrust: T = η · P / V.

    At very low speeds the 1/V term would blow up, so we cap thrust
    at the value corresponding to V = _V_MIN_THRUST.
    """
    P = throttle * ac.P_max_total      # shaft power [W]
    V_eff = max(V, _V_MIN_THRUST)
    return _ETA_PROP * P / V_eff
