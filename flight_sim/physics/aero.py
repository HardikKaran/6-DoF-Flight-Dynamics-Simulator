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
#  Stall model — smooth CL cap with post-stall drop-off
#  Reference: Viterna & Corrigan (1982), flat-plate analogy
# ══════════════════════════════════════════════════════════════════
def _stall_CL(CL_linear: float, alpha: float,
              alpha_stall: float = 0.26,  # ~15 deg
              CL_max: float = 1.6) -> float:
    """Apply post-stall CL reduction using a smooth blend.

    Below alpha_stall the linear CL is returned unchanged.
    Above alpha_stall a Viterna-type flat-plate model blends in,
    giving a smooth drop-off and returning ~0 near 90 deg.
    """
    a = abs(alpha)
    if a < alpha_stall:
        return CL_linear
    sign = 1.0 if CL_linear >= 0 else -1.0
    # flat-plate: CL_fp = 2 sin(a) cos(a)
    CL_fp = 2.0 * math.sin(a) * math.cos(a)
    # blend factor (sigmoid)
    k = min((a - alpha_stall) / 0.10, 1.0)  # linear blend over ~6 deg
    CL_blended = CL_max * (1.0 - k) + CL_fp * k
    return sign * min(abs(CL_blended), abs(CL_max))


# ══════════════════════════════════════════════════════════════════
#  Compressibility corrections
#  — Prandtl-Glauert below M_DD, wave drag above
#  Reference: Anderson, "Fundamentals of Aerodynamics", Ch. 11
# ══════════════════════════════════════════════════════════════════
def _prandtl_glauert(M: float) -> float:
    """Prandtl-Glauert compressibility correction factor (>=1).

    Returns 1/sqrt(1 - M^2) for M < M_crit, capped at M_crit=0.70.
    """
    M_crit = 0.70
    M_eff = min(abs(M), M_crit)
    return 1.0 / math.sqrt(max(1.0 - M_eff * M_eff, 0.09))


def _wave_drag(M: float, M_DD: float = None) -> float:
    """Lock-type wave drag increment above drag-divergence Mach.

    CD_wave = 20 * (M - M_DD)^4  for M > M_DD, else 0.
    Reference: Raymer, "Aircraft Design", Ch. 12.
    """
    if M_DD is None:
        M_DD = ac.M_DD
    if M <= M_DD:
        return 0.0
    dM = M - M_DD
    return 20.0 * dM * dM * dM * dM


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
                 throttle: float, altitude: float,
                 flaps: float = 0.0, W_dot_prev: float = 0.0) -> dict:
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
    flaps        : wing flap deflection fraction 0..1 (1 = full 30 deg)
    W_dot_prev   : previous-step Wdot for downwash-lag derivatives

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
    a_snd = speed_of_sound(max(altitude, 0.0))
    Mach  = V_T / max(a_snd, 1.0)
    q_bar = 0.5 * rho * V_T * V_T

    # ── Compressibility correction factor ─────────────────────────
    PG = _prandtl_glauert(Mach)

    # ── Per-surface lift coefficients ─────────────────────────────
    # Wing  (with Prandtl-Glauert correction and optional flap increment)
    alpha_W = _surface_alpha(alpha, ac.wing, upwash_sign=0.0)
    CL_W_lin = ac.wing.a * PG * alpha_W + flaps * ac.delta_CL0_W
    CL_W     = _stall_CL(CL_W_lin, alpha)

    # Canard  (in upwash field of wing → upwash_sign = +1)
    alpha_C = _surface_alpha(alpha, ac.canard, upwash_sign=+1.0)
    CL_C    = ac.canard.a * PG * alpha_C

    # Horizontal tail  (in downwash of wing → upwash_sign = -1)
    # Also receives elevator contribution
    alpha_H = _surface_alpha(alpha, ac.tail, upwash_sign=-1.0)
    CL_H    = ac.tail.a * PG * alpha_H + ac.a_E * delta_e

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

    CD0_eff = ac.CD0 + flaps * ac.delta_CD0_flap   # flap drag increment
    CD_total = (CD0_eff
                + CD_W * ac.wing.S / ac.S_ref
                + CD_C * ac.canard.S / ac.S_ref
                + CD_H * ac.tail.S / ac.S_ref
                + _wave_drag(Mach))                 # wave drag above M_DD

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

    # ── Wdot (downwash lag) derivatives — Nelson Ch. 8 ────────────
    # The rate of change of downwash lags behind alpha-dot,
    # producing additional pitching moment and normal force.
    #   CM_alpha_dot approx = -2 * a_H * V_H_bar * (de/da) * l_H / c_ref
    #   CZ_alpha_dot approx = -2 * a_H * V_H_bar * (de/da)
    # We approximate alpha_dot ~ Wdot / V using previous-step Wdot.
    if V_T > _EPS and abs(W_dot_prev) > 0.0:
        alpha_dot_est = W_dot_prev / V_T
        alpha_dot_hat = alpha_dot_est * ac.c_ref / (2.0 * V_T)  # normalised
        CZ_adot = -2.0 * ac.tail.a * _V_H * ac.tail.de_da
        CM_adot = -2.0 * ac.tail.a * _V_H * ac.tail.de_da * _l_H / ac.c_ref
        Z_wdot = q_bar * ac.S_ref * CZ_adot * alpha_dot_hat
        M_wdot = q_bar * ac.S_ref * ac.c_ref * CM_adot * alpha_dot_hat
    else:
        Z_wdot = 0.0
        M_wdot = 0.0

    # ── Total body-axis forces ────────────────────────────────────
    X = Xa + Xg + T       # thrust along body x-axis
    Y = Y_aero + Yg       # lateral force
    Z = Za + Zg + Z_wdot  # normal force (includes Wdot derivative)

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

    # NOTE: Pitch damping is already captured by the delta_alpha_H = q*l_H/V_T
    # modification to CL_H above; no separate CM_q term is needed.

    M_total = M_W + M_C + M_H + M_fus + M_thrust + M_wdot

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
        "Mach":    Mach,
        "stall":   abs(alpha) > 0.26,
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
