"""
Aerodynamic Force & Moment Model  (Full 6-DOF)
================================================
Three-surface configuration: canard + wing + horizontal tail.
All outputs in BODY axes, consistent with the EOM sign conventions.

Coordinate system
-----------------
  x — forward (out the nose)
  y — starboard (right wing)
  z — downward (body axis, positive down)

Sign conventions
-----------------
  α   positive nose up
  β   positive wind from starboard (positive V_body)
  q   positive nose up
  p   positive right-wing-down roll
  r   positive nose-right yaw
  δe  positive TED → increases tail lift → nose-down moment
  δa  positive → right roll  (right aileron up, left aileron down)
  δr  positive → trailing edge to port → yaw nose-left  (Nelson convention)
  Θ   positive nose up
  Φ   positive right wing down

References
----------
  Nelson, R.C., "Flight Stability and Automatic Control", 2nd Ed.,
    McGraw-Hill, 1998 — Chapters 2–4, lateral derivatives Ch. 3 §3.5.
  Cook, M.V., "Flight Dynamics Principles", 3rd Ed., 2013 — Ch. 3–4.
  Etkin, B. & Reid, L.D., "Dynamics of Flight", 3rd Ed., 1996 — Ch. 4–5.
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
_l_C = 0.0; _l_W = 0.0; _l_H = 0.0
_dz_T = 0.0; _dz_C = 0.0; _dz_W = 0.0; _dz_H = 0.0
_V_H = 0.0

def _precompute():
    """Recompute derived geometry from current aircraft params."""
    global _l_C, _l_W, _l_H, _dz_T, _dz_C, _dz_W, _dz_H, _V_H
    _l_C = ac.canard.x_ac - ac.x_CG   # canard arm (negative → forward of CG)
    _l_W = ac.wing.x_ac   - ac.x_CG   # wing arm   (small positive)
    _l_H = ac.tail.x_ac   - ac.x_CG   # tail arm   (positive → aft of CG)
    _dz_T = ac.z_T  - ac.z_CG         # thrust line above CG
    _dz_C = ac.canard.z_ac - ac.z_CG
    _dz_W = ac.wing.z_ac   - ac.z_CG
    _dz_H = ac.tail.z_ac   - ac.z_CG
    _V_H = (ac.tail.S * _l_H) / (ac.S_ref * ac.c_ref)

_precompute()  # run once at import


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

def compute_aero(U: float, V_body: float, W: float,
                 p: float, q: float, r: float,
                 phi: float, theta: float,
                 delta_e: float, delta_a: float, delta_r: float,
                 throttle: float, altitude: float) -> dict:
    """
    Compute aerodynamic + thrust forces and moments (full 6-DOF).

    Parameters
    ----------
    U, V_body, W : body-axis velocities [m/s]  (forward, right, down)
    p, q, r      : body-axis angular rates [rad/s]  (roll, pitch, yaw)
    phi          : bank angle [rad]
    theta        : pitch angle [rad]
    delta_e      : elevator deflection [rad], positive TED
    delta_a      : aileron deflection [rad], positive right-roll
    delta_r      : rudder deflection [rad], positive TE-to-port
    throttle     : 0 … 1
    altitude     : geometric altitude h [m]  (= −zE)

    Returns
    -------
    dict with keys:
        X, Y, Z       — total body-axis forces [N]
        L_lat, M, N   — total body-axis moments [N·m] (roll, pitch, yaw)
        CL, CD, CY    — force coefficients
        alpha, beta   — aerodynamic angles [rad]
        V_T, q_bar    — airspeed [m/s] and dynamic pressure [Pa]
        T             — thrust [N]
        L_aero, D_aero, Y_aero — aero lift/drag/side forces [N]
    """

    # ── Airspeed & aerodynamic angles ─────────────────────────────────────
    V_T   = math.sqrt(U * U + V_body * V_body + W * W + _EPS)
    alpha = math.atan2(W, U)
    beta  = math.atan2(V_body, math.sqrt(U * U + W * W + _EPS))

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
    #   Δα_H = +q · l_H / V_T   (tail aft of CG, pitch-up → increased AoA at tail)
    if V_T > _EPS:
        delta_alpha_q = q * _l_H / V_T
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

    # ── Gravity in body axes (full 3D) ────────────────────────────
    # Nelson Eq. (2.11): body-axis gravity components
    sin_t = math.sin(theta)
    cos_t = math.cos(theta)
    sin_p = math.sin(phi)
    cos_p = math.cos(phi)
    Xg = -ac.m * ac.g * sin_t
    Yg =  ac.m * ac.g * cos_t * sin_p
    Zg =  ac.m * ac.g * cos_t * cos_p

    # ── Lateral aerodynamic forces & moments ──────────────────────
    # Nelson §3.5 / Cook §4.3: linearised lateral-directional model
    b      = ac.b_ref
    V_safe = max(V_T, _EPS)
    p_hat  = p * b / (2.0 * V_safe)   # normalised roll rate  p̂ = pb/(2V)
    r_hat  = r * b / (2.0 * V_safe)   # normalised yaw rate   r̂ = rb/(2V)

    # Side-force coefficient
    CY = (ac.CY_beta * beta
          + ac.CY_p * p_hat
          + ac.CY_r * r_hat
          + ac.CY_da * delta_a
          + ac.CY_dr * delta_r)
    Y_aero = q_bar * ac.S_ref * CY

    # Rolling-moment coefficient  (about body x-axis)
    Cl = (ac.Cl_beta * beta
          + ac.Cl_p * p_hat
          + ac.Cl_r * r_hat
          + ac.Cl_da * delta_a
          + ac.Cl_dr * delta_r)
    L_lat = q_bar * ac.S_ref * b * Cl

    # Yawing-moment coefficient  (about body z-axis)
    Cn = (ac.Cn_beta * beta
          + ac.Cn_p * p_hat
          + ac.Cn_r * r_hat
          + ac.Cn_da * delta_a
          + ac.Cn_dr * delta_r)
    N_yaw = q_bar * ac.S_ref * b * Cn

    # ── Total body-axis forces ────────────────────────────────────
    X = Xa + Xg + T       # thrust along body x-axis
    Y = Y_aero + Yg       # lateral force
    Z = Za + Zg           # normal force

    # ── Pitching moment about CG ─────────────────────────────────
    # Each surface: M_surf = CM0_term − CL · l  (positive nose-up;
    #   l measured positive-aft from CG, so −CL·l gives nose-down for
    #   upward lift behind the CG — standard sign convention).
    M_W = (q_bar * ac.wing.S   * ac.wing.c_bar   * ac.wing.CM0
            - q_bar * ac.wing.S   * CL_W * _l_W)
    M_C = (q_bar * ac.canard.S * ac.canard.c_bar * ac.canard.CM0
            - q_bar * ac.canard.S * CL_C * _l_C)
    M_H = (q_bar * ac.tail.S   * ac.tail.c_bar   * ac.tail.CM0
            - q_bar * ac.tail.S   * CL_H * _l_H)

    # Fuselage contribution
    M_fus = q_bar * ac.S_ref * ac.c_ref * ac.dCM_da_fus * alpha

    # Thrust moment (z-offset of thrust line from CG)
    M_thrust = -T * _dz_T

    # NOTE: Pitch damping is already captured by the Δα_H = q·l_H/V_T
    # modification to CL_H above; no separate CM_q term is needed.

    M_total = M_W + M_C + M_H + M_fus + M_thrust

    return {
        "X":       X,
        "Y":       Y,
        "Z":       Z,
        "L_lat":   L_lat,        # rolling moment about body x
        "M":       M_total,      # pitching moment about body y
        "N":       N_yaw,        # yawing moment about body z
        "CL":      CL_total,
        "CD":      CD_total,
        "CY":      CY,
        "Cl":      Cl,           # rolling-moment coefficient
        "Cn":      Cn,           # yawing-moment coefficient
        "alpha":   alpha,
        "beta":    beta,
        "V_T":     V_T,
        "q_bar":   q_bar,
        "T":       T,
        "L_aero":  L_aero,
        "D_aero":  D_aero,
        "Y_aero":  Y_aero,
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
