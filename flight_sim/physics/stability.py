"""
Linear Stability Analysis
===========================
Compute the linearised state-space matrices (A, B) at a trim point,
extract eigenvalues, and identify classical flight dynamic modes.

References
----------
  Nelson, R.C., "Flight Stability and Automatic Control", 2nd Ed.,
    McGraw-Hill, 1998 — Ch. 4 (longitudinal), Ch. 5 (lateral).
  Etkin, B. & Reid, L.D., "Dynamics of Flight", 3rd Ed., 1996 — Ch. 6.
  Cook, M.V., "Flight Dynamics Principles", 3rd Ed., 2013 — Ch. 5.
"""

from __future__ import annotations

import math
import numpy as np

from .eom import compute_derivatives, find_trim, STATE_SIZE
from .eom import I_U, I_V, I_W, I_P, I_Q, I_R, I_PHI, I_TH, I_PSI
from . import aircraft as ac
from .atmosphere import speed_of_sound


# ══════════════════════════════════════════════════════════════════
#  Numerical linearisation via central differences
# ══════════════════════════════════════════════════════════════════

def linearise(state: np.ndarray, delta_e: float, delta_a: float,
              delta_r: float, throttle: float,
              dx: float = 1e-4, du: float = 1e-4) -> dict:
    """
    Compute A (12x12) and B (12x4) matrices by central finite differences.

    A = df/dx,  B = df/du  evaluated at the given operating point.
    Control vector u = [delta_e, delta_a, delta_r, throttle].

    Returns dict with 'A', 'B' arrays.
    """
    n = STATE_SIZE
    m = 4  # number of controls
    A = np.zeros((n, n))
    B = np.zeros((n, m))

    f0 = compute_derivatives(state, delta_e, delta_a, delta_r, throttle)

    # -- A matrix (state perturbations) --
    for j in range(n):
        x_plus = state.copy()
        x_minus = state.copy()
        x_plus[j] += dx
        x_minus[j] -= dx
        fp = compute_derivatives(x_plus, delta_e, delta_a, delta_r, throttle)
        fm = compute_derivatives(x_minus, delta_e, delta_a, delta_r, throttle)
        A[:, j] = (fp - fm) / (2.0 * dx)

    # -- B matrix (control perturbations) --
    controls = [delta_e, delta_a, delta_r, throttle]
    for j in range(m):
        cp = list(controls)
        cm = list(controls)
        cp[j] += du
        cm[j] -= du
        fp = compute_derivatives(state, *cp)
        fm = compute_derivatives(state, *cm)
        B[:, j] = (fp - fm) / (2.0 * du)

    return {"A": A, "B": B}


# ══════════════════════════════════════════════════════════════════
#  Eigenvalue analysis & mode identification
# ══════════════════════════════════════════════════════════════════

def get_eigenvalues(A: np.ndarray) -> np.ndarray:
    """Return eigenvalues of the system matrix A."""
    return np.linalg.eigvals(A)


def identify_modes(eigenvalues: np.ndarray) -> list:
    """
    Identify classical flight dynamic modes from eigenvalues.

    Returns a list of dicts, each with:
        name       : mode name (short period, phugoid, dutch roll, etc.)
        eigenvalue : complex eigenvalue
        wn         : natural frequency [rad/s]
        zeta       : damping ratio
        period     : period [s] (inf for real roots)
        t_half     : half-life [s] (time to halve amplitude)
        category   : 'longitudinal' or 'lateral'
    """
    modes = []
    used = set()

    # Sort by imaginary part magnitude (largest first for oscillatory)
    idx_sorted = np.argsort(-np.abs(eigenvalues.imag))

    for i in idx_sorted:
        if i in used:
            continue
        ev = eigenvalues[i]

        # Find conjugate pair
        partner = None
        if abs(ev.imag) > 1e-6:
            for j in idx_sorted:
                if j != i and j not in used:
                    if abs(eigenvalues[j] - ev.conjugate()) < 1e-6:
                        partner = j
                        break

        sigma = ev.real
        omega = abs(ev.imag)
        wn = abs(ev)
        zeta = -sigma / wn if wn > 1e-10 else (1.0 if sigma < 0 else -1.0)
        period = (2.0 * math.pi / omega) if omega > 1e-6 else float('inf')
        t_half = (math.log(2.0) / abs(sigma)) if abs(sigma) > 1e-10 else float('inf')

        mode_info = {
            "eigenvalue": complex(ev),
            "wn": float(wn),
            "zeta": float(zeta),
            "period": float(period),
            "t_half": float(t_half),
            "stable": sigma < 0,
        }

        modes.append(mode_info)
        used.add(i)
        if partner is not None:
            used.add(partner)

    # Classify modes by frequency and damping
    _classify_modes(modes)
    return modes


def _classify_modes(modes: list):
    """Assign names to modes based on their frequency characteristics."""
    # Separate into oscillatory and aperiodic
    osc = [m for m in modes if m["period"] < 1e6]
    aper = [m for m in modes if m["period"] >= 1e6]

    # Sort oscillatory by natural frequency
    osc.sort(key=lambda m: m["wn"], reverse=True)

    # Longitudinal oscillatory: Short period (high freq) and Phugoid (low freq)
    lon_osc_assigned = 0
    for m in osc:
        if lon_osc_assigned == 0 and m["wn"] > 0.5:
            m["name"] = "Short Period"
            m["category"] = "longitudinal"
            lon_osc_assigned += 1
        elif lon_osc_assigned == 1 and m["wn"] < 0.5:
            m["name"] = "Phugoid"
            m["category"] = "longitudinal"
            lon_osc_assigned += 1
        elif "name" not in m:
            if m["wn"] > 0.3:
                m["name"] = "Dutch Roll"
                m["category"] = "lateral"
            else:
                m["name"] = "Oscillatory"
                m["category"] = "unknown"

    for m in aper:
        if "name" not in m:
            ev = m["eigenvalue"]
            if abs(ev.real) < 0.01:
                m["name"] = "Spiral"
                m["category"] = "lateral"
            elif abs(ev.real) > 0.5:
                m["name"] = "Roll Subsidence"
                m["category"] = "lateral"
            else:
                m["name"] = "Aperiodic"
                m["category"] = "unknown"

    # Fill in any unclassified
    for m in modes:
        if "name" not in m:
            m["name"] = "Mode"
            m["category"] = "unknown"


# ══════════════════════════════════════════════════════════════════
#  Trim map  (how trim varies with speed / altitude)
# ══════════════════════════════════════════════════════════════════

def compute_trim_map(V_range: np.ndarray = None,
                     h_range: np.ndarray = None) -> dict:
    """
    Compute a map of trim conditions over a range of speeds and altitudes.

    Returns dict with 2D arrays: alpha, delta_e, throttle, converged.
    """
    if V_range is None:
        V_range = np.linspace(60, 220, 9)
    if h_range is None:
        h_range = np.array([0, 3000, 6000, 9144])

    nV = len(V_range)
    nH = len(h_range)
    alpha_map = np.zeros((nH, nV))
    de_map = np.zeros((nH, nV))
    thr_map = np.zeros((nH, nV))
    conv_map = np.zeros((nH, nV), dtype=bool)

    for ih, h in enumerate(h_range):
        for iv, V in enumerate(V_range):
            try:
                trim = find_trim(V, h)
                alpha_map[ih, iv] = math.degrees(trim["alpha"])
                de_map[ih, iv] = math.degrees(trim["delta_e"])
                thr_map[ih, iv] = trim["throttle"] * 100
                conv_map[ih, iv] = trim["converged"]
            except Exception:
                conv_map[ih, iv] = False

    return {
        "V_range": V_range.tolist(),
        "h_range": h_range.tolist(),
        "alpha": alpha_map.tolist(),
        "delta_e": de_map.tolist(),
        "throttle": thr_map.tolist(),
        "converged": conv_map.tolist(),
    }


# ══════════════════════════════════════════════════════════════════
#  Elevator hinge moment  (Nelson §7.4)
# ══════════════════════════════════════════════════════════════════

def compute_hinge_moment(alpha_H: float, delta_e: float,
                         q_bar: float) -> dict:
    """
    Compute elevator hinge moment and equivalent stick force.

    H_e = q_bar * S_e * c_e * (b_0 + b_H * alpha_H + b_E * delta_e)
    F_s = H_e * G   (G = gearing ratio, ~0.5 for typical linkage)

    Parameters
    ----------
    alpha_H  : effective tail AoA [rad]
    delta_e  : elevator deflection [rad]
    q_bar    : dynamic pressure [Pa]

    Returns
    -------
    dict with H_e [N*m], F_s [N], C_H (hinge moment coefficient)
    """
    S_e = ac.tail.S * ac.cE_cH           # elevator planform area
    c_e = ac.tail.c_bar * ac.cE_cH       # elevator chord
    C_H = ac.b_0 + ac.b_H * alpha_H + ac.b_E * delta_e
    H_e = q_bar * S_e * c_e * C_H
    G = 0.5  # gearing ratio
    F_s = H_e * G
    return {"C_H": float(C_H), "H_e": float(H_e), "F_s": float(F_s)}


# ══════════════════════════════════════════════════════════════════
#  Convenience: full stability report for a trim point
# ══════════════════════════════════════════════════════════════════

def stability_report(V: float = 180.0, h: float = 9144.0) -> dict:
    """
    Compute full stability analysis at a trim point.

    Returns dict with trim info, eigenvalues, modes, A/B matrices.
    """
    trim = find_trim(V, h)
    if not trim["converged"]:
        return {"error": "Trim did not converge", "trim": trim}

    state = trim["state"]
    de = trim["delta_e"]
    thr = trim["throttle"]

    lin = linearise(state, de, 0.0, 0.0, thr)
    eigs = get_eigenvalues(lin["A"])
    modes = identify_modes(eigs)

    # Extract longitudinal and lateral subsets
    lon_idx = [I_U, I_W, I_Q, I_TH]
    lat_idx = [I_V, I_P, I_R, I_PHI]
    A_lon = lin["A"][np.ix_(lon_idx, lon_idx)]
    A_lat = lin["A"][np.ix_(lat_idx, lat_idx)]

    eigs_lon = np.linalg.eigvals(A_lon)
    eigs_lat = np.linalg.eigvals(A_lat)
    modes_lon = identify_modes(eigs_lon)
    modes_lat = identify_modes(eigs_lat)

    return {
        "trim": {
            "alpha_deg": math.degrees(trim["alpha"]),
            "delta_e_deg": math.degrees(de),
            "throttle_pct": thr * 100,
            "V": V,
            "h": h,
        },
        "A": lin["A"].tolist(),
        "B": lin["B"].tolist(),
        "eigenvalues_full": [{"real": e.real, "imag": e.imag} for e in eigs],
        "eigenvalues_lon": [{"real": e.real, "imag": e.imag} for e in eigs_lon],
        "eigenvalues_lat": [{"real": e.real, "imag": e.imag} for e in eigs_lat],
        "modes_lon": [{k: v for k, v in m.items() if k != "eigenvalue"} for m in modes_lon],
        "modes_lat": [{k: v for k, v in m.items() if k != "eigenvalue"} for m in modes_lat],
        "modes_full": [{k: v for k, v in m.items() if k != "eigenvalue"} for m in modes],
    }
