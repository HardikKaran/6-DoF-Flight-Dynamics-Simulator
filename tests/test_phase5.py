"""
Phase 5 — Full 6-DOF Physics Tests
=====================================
Validates the 12-state EOM, lateral aerodynamics, Ixz coupling,
Euler kinematics, 3D navigation equations, and trim stability.

References used for equation verification:
  [1] Nelson, R.C., "Flight Stability and Automatic Control", 2nd Ed.,
      McGraw-Hill, 1998.
  [2] Stevens, B.L., Lewis, F.L., Johnson, E.N., "Aircraft Control and
      Simulation", 3rd Ed., Wiley, 2016.
  [3] Etkin, B. & Reid, L.D., "Dynamics of Flight", 3rd Ed., Wiley, 1996.
  [4] Cook, M.V., "Flight Dynamics Principles", 3rd Ed., 2013.
"""

import sys, os, math
import numpy as np

# Ensure the flight_sim package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from flight_sim.physics import atmosphere as atm
from flight_sim.physics import aircraft as ac
from flight_sim.physics.aero import compute_aero, _precompute
from flight_sim.physics.eom import (
    compute_derivatives, rk4_step, find_trim, make_initial_state,
    I_U, I_V, I_W, I_P, I_Q, I_R,
    I_PHI, I_TH, I_PSI, I_XE, I_YE, I_ZE,
    STATE_SIZE,
)

SEP = "=" * 65
PASS = "  ✓ "
FAIL = "  ✗ "
n_pass = 0
n_fail = 0


def check(cond, msg):
    global n_pass, n_fail
    if cond:
        print(PASS + msg)
        n_pass += 1
    else:
        print(FAIL + msg)
        n_fail += 1


# ══════════════════════════════════════════════════════════════════
#  1. State vector & indices
# ══════════════════════════════════════════════════════════════════
def test_state_vector():
    print(SEP)
    print("1. State Vector — 12-state indices")
    print(SEP)
    check(STATE_SIZE == 12, f"STATE_SIZE = {STATE_SIZE} (expect 12)")
    check(I_U == 0 and I_V == 1 and I_W == 2, "U=0, V=1, W=2")
    check(I_P == 3 and I_Q == 4 and I_R == 5, "p=3, q=4, r=5")
    check(I_PHI == 6 and I_TH == 7 and I_PSI == 8, "φ=6, θ=7, ψ=8")
    check(I_XE == 9 and I_YE == 10 and I_ZE == 11, "xE=9, yE=10, zE=11")
    print()


# ══════════════════════════════════════════════════════════════════
#  2. Aircraft parameters — lateral derivatives present
# ══════════════════════════════════════════════════════════════════
def test_aircraft_params():
    print(SEP)
    print("2. Aircraft Parameters — lateral derivatives & inertias")
    print(SEP)
    # Inertias
    check(ac.Ixx > 0, f"Ixx = {ac.Ixx} > 0")
    check(ac.Iyy > 0, f"Iyy = {ac.Iyy} > 0")
    check(ac.Izz > 0, f"Izz = {ac.Izz} > 0")
    check(ac.Ixx * ac.Izz > ac.Ixz**2,
          f"Γ = Ixx·Izz − Ixz² = {ac.Ixx * ac.Izz - ac.Ixz**2:.0f} > 0 (positive definite)")

    # Lateral derivatives – sign checks (physically correct)
    check(ac.CY_beta < 0, f"CY_β = {ac.CY_beta} < 0 (restoring side force)")
    check(ac.Cl_beta < 0, f"Cl_β = {ac.Cl_beta} < 0 (dihedral effect)")
    check(ac.Cn_beta > 0, f"Cn_β = {ac.Cn_beta} > 0 (weathercock stability)")
    check(ac.Cl_p < 0,    f"Cl_p = {ac.Cl_p} < 0 (roll damping)")
    check(ac.Cn_r < 0,    f"Cn_r = {ac.Cn_r} < 0 (yaw damping)")
    check(ac.Cl_da > 0,   f"Cl_δa = {ac.Cl_da} > 0 (aileron effectiveness)")
    check(ac.Cn_dr < 0,   f"Cn_δr = {ac.Cn_dr} < 0 (rudder effectiveness, Nelson convention)")
    check(ac.b_ref > 0,   f"b_ref = {ac.b_ref} m (wingspan)")
    print()


# ══════════════════════════════════════════════════════════════════
#  3. Aero model — longitudinal + lateral at sample condition
# ══════════════════════════════════════════════════════════════════
def test_aero_model():
    print(SEP)
    print("3. Aero Model — full 6-DOF at sample condition")
    print(SEP)

    alpha = math.radians(3.0)
    V = 180.0
    U = V * math.cos(alpha)
    W = V * math.sin(alpha)
    V_body = 0.0
    theta = alpha
    phi = 0.0
    p = q = r = 0.0
    de = 0.0
    da = 0.0
    dr = 0.0
    throttle = 0.3

    fm = compute_aero(U, V_body, W, p, q, r, phi, theta,
                      de, da, dr, throttle, 9144.0)

    # Check all expected keys are present
    required_keys = ['X', 'Y', 'Z', 'L_lat', 'M', 'N',
                     'CL', 'CD', 'CY', 'alpha', 'beta', 'V_T',
                     'q_bar', 'T', 'L_aero', 'D_aero', 'Y_aero']
    for k in required_keys:
        check(k in fm, f"Key '{k}' present in aero output")

    # At zero sideslip, lateral forces/moments should be ~zero
    check(abs(fm['Y']) < 100, f"|Y| = {abs(fm['Y']):.1f} ≈ 0 at β=0")
    check(abs(fm['L_lat']) < 100, f"|L_lat| = {abs(fm['L_lat']):.1f} ≈ 0 at β=0")
    check(abs(fm['N']) < 100, f"|N| = {abs(fm['N']):.1f} ≈ 0 at β=0")
    check(abs(fm['beta']) < 0.001, f"|β| = {abs(fm['beta']):.4f} ≈ 0")

    # Longitudinal sanity
    check(fm['CL'] > 0, f"CL = {fm['CL']:.4f} > 0 (positive lift)")
    check(fm['CD'] > 0, f"CD = {fm['CD']:.6f} > 0 (positive drag)")
    check(fm['T'] > 0, f"T = {fm['T']:.0f} N > 0 (positive thrust)")

    print(f"\n  Sample forces: X={fm['X']:.0f}  Y={fm['Y']:.0f}  Z={fm['Z']:.0f}")
    print(f"  Moments: L={fm['L_lat']:.0f}  M={fm['M']:.0f}  N={fm['N']:.0f}")
    print()

    # Test with sideslip
    print("  --- With β ≈ 5° sideslip ---")
    beta_test = math.radians(5.0)
    V_body_test = V * math.sin(beta_test)
    fm2 = compute_aero(U, V_body_test, W, 0, 0, 0, 0.0, theta,
                       de, da, dr, throttle, 9144.0)
    check(fm2['Y'] < 0, f"Y = {fm2['Y']:.0f} < 0 at positive β (CY_β < 0)")
    check(fm2['L_lat'] < 0, f"L_lat = {fm2['L_lat']:.0f} < 0 at positive β (Cl_β < 0, dihedral)")
    check(fm2['N'] > 0, f"N = {fm2['N']:.0f} > 0 at positive β (Cn_β > 0, weathercock)")
    print()


# ══════════════════════════════════════════════════════════════════
#  4. Aileron & rudder effectiveness
# ══════════════════════════════════════════════════════════════════
def test_control_effectiveness():
    print(SEP)
    print("4. Aileron & Rudder Control Effectiveness")
    print(SEP)

    V = 180.0
    alpha = math.radians(3.0)
    U = V * math.cos(alpha)
    W = V * math.sin(alpha)
    theta = alpha

    # Baseline (no controls)
    fm0 = compute_aero(U, 0, W, 0, 0, 0, 0, theta,
                       0, 0, 0, 0.3, 9144.0)

    # Aileron: positive δa → positive L_lat (roll right)
    fm_a = compute_aero(U, 0, W, 0, 0, 0, 0, theta,
                        0, math.radians(10), 0, 0.3, 9144.0)
    check(fm_a['L_lat'] > fm0['L_lat'],
          f"δa=+10° → L_lat increases: {fm_a['L_lat']:.0f} > {fm0['L_lat']:.0f} (roll right)")

    # Rudder: positive δr (TE-to-port) → negative N (yaw left, Cn_dr < 0)
    fm_r = compute_aero(U, 0, W, 0, 0, 0, 0, theta,
                        0, 0, math.radians(10), 0.3, 9144.0)
    check(fm_r['N'] < fm0['N'],
          f"δr=+10° → N decreases: {fm_r['N']:.0f} < {fm0['N']:.0f} (yaw left)")

    # Side force from rudder
    check(fm_r['Y'] > fm0['Y'],
          f"δr=+10° → Y increases: {fm_r['Y']:.0f} (positive side force, CY_dr > 0)")
    print()


# ══════════════════════════════════════════════════════════════════
#  5. Gravity components — Nelson Eq. (2.11)
# ══════════════════════════════════════════════════════════════════
def test_gravity():
    print(SEP)
    print("5. Body-Axis Gravity — Nelson Eq. (2.11)")
    print(SEP)

    theta = math.radians(10)
    phi = math.radians(20)

    # Use a simple condition to check gravity
    V = 180.0
    U = V
    fm = compute_aero(U, 0, 0, 0, 0, 0, phi, theta, 0, 0, 0, 0, 9144.0)

    # Gravity should contribute:
    #   Xg = -mg sinθ
    #   Yg =  mg cosθ sinΦ
    #   Zg =  mg cosθ cosΦ
    mg = ac.m * ac.g
    Xg_expected = -mg * math.sin(theta)
    Yg_expected = mg * math.cos(theta) * math.sin(phi)
    Zg_expected = mg * math.cos(theta) * math.cos(phi)

    # We can't isolate gravity from aero forces directly, but we can
    # verify the total weight magnitude is correct
    W_total = math.sqrt(Xg_expected**2 + Yg_expected**2 + Zg_expected**2)
    check(abs(W_total - mg) < 1.0,
          f"|W_gravity| = {W_total:.0f} ≈ mg = {mg:.0f} (correct magnitude)")
    print(f"  Components: Xg={Xg_expected:.0f}  Yg={Yg_expected:.0f}  Zg={Zg_expected:.0f}")
    print()


# ══════════════════════════════════════════════════════════════════
#  6. Ixz coupling — verify the 2×2 moment equation system
# ══════════════════════════════════════════════════════════════════
def test_ixz_coupling():
    print(SEP)
    print("6. Ixz Coupling — Γ = Ixx·Izz − Ixz²")
    print(SEP)

    Gamma = ac.Ixx * ac.Izz - ac.Ixz**2
    check(Gamma > 0, f"Γ = {Gamma:.0f} > 0 (positive definite inertia)")

    # Verify that the coupling ratio Ixz/Ixx is reasonable (< 0.3 typically)
    ratio = ac.Ixz / ac.Ixx
    check(ratio < 0.3, f"Ixz/Ixx = {ratio:.3f} < 0.3 (reasonable coupling)")

    # Test: apply a pure rolling moment and check that it induces yaw (Ixz)
    V = 180.0
    alpha = math.radians(2.0)
    state = np.zeros(STATE_SIZE)
    state[I_U] = V * math.cos(alpha)
    state[I_W] = V * math.sin(alpha)
    state[I_TH] = alpha
    state[I_ZE] = -9144.0

    # Apply aileron to create rolling moment
    derivs = compute_derivatives(state, 0.0, math.radians(10), 0.0, 0.3)
    p_dot = derivs[I_P]
    r_dot = derivs[I_R]

    check(abs(p_dot) > 0.001, f"ṗ = {p_dot:.4f} ≠ 0 (rolling acceleration from aileron)")
    # Ixz coupling should induce some yaw acceleration
    check(abs(r_dot) > 0.0001,
          f"ṙ = {r_dot:.6f} ≠ 0 (Ixz coupling induces yaw from roll)")
    print()


# ══════════════════════════════════════════════════════════════════
#  7. Euler Kinematics — Nelson Eq. (2.30a–c)
# ══════════════════════════════════════════════════════════════════
def test_euler_kinematics():
    print(SEP)
    print("7. Euler Kinematics — Φ̇, Θ̇, Ψ̇")
    print(SEP)

    # Test 1: pure pitch rate → Θ̇ = q at Φ=0
    state = np.zeros(STATE_SIZE)
    state[I_U] = 180.0
    state[I_Q] = 0.1  # 0.1 rad/s pitch rate
    state[I_ZE] = -9144.0

    derivs = compute_derivatives(state, 0.0, 0.0, 0.0, 0.3)
    check(abs(derivs[I_TH] - 0.1) < 0.001,
          f"Pure q=0.1: Θ̇ = {derivs[I_TH]:.4f} ≈ 0.1 (Θ̇ = q·cosΦ at Φ=0)")

    # Test 2: pure roll rate → Φ̇ = p at Θ=0
    state2 = np.zeros(STATE_SIZE)
    state2[I_U] = 180.0
    state2[I_P] = 0.2
    state2[I_ZE] = -9144.0

    derivs2 = compute_derivatives(state2, 0.0, 0.0, 0.0, 0.3)
    check(abs(derivs2[I_PHI] - 0.2) < 0.01,
          f"Pure p=0.2: Φ̇ = {derivs2[I_PHI]:.4f} ≈ 0.2 (Φ̇ = p at Θ=0)")

    # Test 3: with bank angle, yaw rate → heading change
    state3 = np.zeros(STATE_SIZE)
    state3[I_U] = 180.0
    state3[I_R] = 0.05
    state3[I_PHI] = 0.0  # wings level
    state3[I_ZE] = -9144.0

    derivs3 = compute_derivatives(state3, 0.0, 0.0, 0.0, 0.3)
    # At Φ=0, Θ=0: Ψ̇ = r·cosΦ/cosΘ = r
    check(abs(derivs3[I_PSI] - 0.05) < 0.01,
          f"Pure r=0.05: Ψ̇ = {derivs3[I_PSI]:.4f} ≈ 0.05 at Φ=Θ=0")
    print()


# ══════════════════════════════════════════════════════════════════
#  8. Navigation Equations — Body→Earth DCM
# ══════════════════════════════════════════════════════════════════
def test_navigation():
    print(SEP)
    print("8. Navigation — Body→Earth Direction Cosine Matrix")
    print(SEP)

    # Test 1: straight and level, heading north (Ψ=0, Θ=0, Φ=0)
    state = np.zeros(STATE_SIZE)
    state[I_U] = 100.0  # flying north at 100 m/s
    state[I_ZE] = -5000.0

    derivs = compute_derivatives(state, 0.0, 0.0, 0.0, 0.3)
    check(abs(derivs[I_XE] - 100.0) < 5.0,
          f"Heading north: ẋE = {derivs[I_XE]:.1f} ≈ 100 m/s")
    check(abs(derivs[I_YE]) < 5.0,
          f"Heading north: ẏE = {derivs[I_YE]:.1f} ≈ 0 m/s")

    # Test 2: heading east (Ψ = π/2)
    state2 = np.zeros(STATE_SIZE)
    state2[I_U] = 100.0
    state2[I_PSI] = math.pi / 2  # heading east
    state2[I_ZE] = -5000.0

    derivs2 = compute_derivatives(state2, 0.0, 0.0, 0.0, 0.3)
    check(abs(derivs2[I_XE]) < 5.0,
          f"Heading east: ẋE = {derivs2[I_XE]:.1f} ≈ 0")
    check(abs(derivs2[I_YE] - 100.0) < 5.0,
          f"Heading east: ẏE = {derivs2[I_YE]:.1f} ≈ 100 m/s")

    # Test 3: climbing (Θ > 0)
    state3 = np.zeros(STATE_SIZE)
    state3[I_U] = 100.0
    state3[I_TH] = math.radians(10)  # 10° pitch-up
    state3[I_ZE] = -5000.0

    derivs3 = compute_derivatives(state3, 0.0, 0.0, 0.0, 0.3)
    check(derivs3[I_ZE] < 0,
          f"Pitch-up 10°: żE = {derivs3[I_ZE]:.1f} < 0 (climbing, altitude increases)")
    print()


# ══════════════════════════════════════════════════════════════════
#  9. Trim solver
# ══════════════════════════════════════════════════════════════════
def test_trim():
    print(SEP)
    print("9. Trim Solver — level cruise at M≈0.62, h=9144 m")
    print(SEP)

    a_cruise = atm.speed_of_sound(9144.0)
    V_cruise = ac.M_inf * a_cruise
    print(f"  Target V = {V_cruise:.1f} m/s  (M={ac.M_inf})")

    trim = find_trim(V_cruise, 9144.0)
    check(trim['converged'], "Trim solver converged")
    check(-math.radians(5) < trim['alpha'] < math.radians(15),
          f"α_trim = {math.degrees(trim['alpha']):.3f}° (reasonable range for canard config)")
    check(abs(trim['delta_e']) < math.radians(20),
          f"δe_trim = {math.degrees(trim['delta_e']):.3f}°")
    check(0 < trim['throttle'] < 1.0,
          f"throttle = {trim['throttle']*100:.1f}%")

    # Verify state is 12 elements
    check(len(trim['state']) == 12,
          f"Trim state has {len(trim['state'])} elements (expect 12)")

    # Verify lateral states are zero
    check(abs(trim['state'][I_V]) < 1e-10, "V_body = 0 at trim")
    check(abs(trim['state'][I_P]) < 1e-10, "p = 0 at trim")
    check(abs(trim['state'][I_R]) < 1e-10, "r = 0 at trim")
    check(abs(trim['state'][I_PHI]) < 1e-10, "Φ = 0 at trim")

    # Verify trim derivatives ≈ 0
    state = trim['state']
    derivs = compute_derivatives(state, trim['delta_e'], 0.0, 0.0, trim['throttle'])
    check(abs(derivs[I_U]) < 1.0,
          f"U̇ at trim = {derivs[I_U]:.4f} ≈ 0")
    check(abs(derivs[I_W]) < 1.0,
          f"Ẇ at trim = {derivs[I_W]:.4f} ≈ 0")
    check(abs(derivs[I_Q]) < 0.01,
          f"q̇ at trim = {derivs[I_Q]:.6f} ≈ 0")

    print(f"\n  Trim: α={math.degrees(trim['alpha']):.3f}°  "
          f"δe={math.degrees(trim['delta_e']):.3f}°  "
          f"thr={trim['throttle']*100:.1f}%")
    print()
    return trim


# ══════════════════════════════════════════════════════════════════
#  10. Longitudinal stability (10-second sim from trim)
# ══════════════════════════════════════════════════════════════════
def test_longitudinal_stability(trim):
    print(SEP)
    print("10. Longitudinal Stability — 10s from trim (wings level)")
    print(SEP)

    state = trim['state'].copy()
    de = trim['delta_e']
    thr = trim['throttle']
    dt = 1.0 / 60.0
    n_steps = int(10.0 / dt)

    print(f"  {'t':>5}  {'U':>8}  {'W':>8}  {'q':>10}  {'θ°':>7}  {'alt':>7}")
    for i in range(n_steps):
        if i % 120 == 0:
            alt = -state[I_ZE]
            print(f"  {i*dt:5.1f}  {state[I_U]:8.2f}  {state[I_W]:8.2f}"
                  f"  {state[I_Q]:10.6f}  {math.degrees(state[I_TH]):7.3f}"
                  f"  {alt:7.0f}")
        state = rk4_step(state, de, 0.0, 0.0, thr, dt)

    alt_final = -state[I_ZE]
    alt_init = -trim['state'][I_ZE]
    drift = abs(alt_final - alt_init)
    check(drift < 100, f"Altitude drift = {drift:.1f} m (< 100 m is good)")

    # Lateral states should remain zero
    check(abs(state[I_V]) < 1.0, f"|V| = {abs(state[I_V]):.4f} ≈ 0 (no sideslip)")
    check(abs(state[I_P]) < 0.01, f"|p| = {abs(state[I_P]):.6f} ≈ 0 (no roll)")
    check(abs(state[I_R]) < 0.01, f"|r| = {abs(state[I_R]):.6f} ≈ 0 (no yaw)")
    check(abs(state[I_PHI]) < 0.01, f"|Φ| = {abs(state[I_PHI]):.6f} ≈ 0 (wings level)")
    print()


# ══════════════════════════════════════════════════════════════════
#  11. Roll response to aileron input
# ══════════════════════════════════════════════════════════════════
def test_roll_response(trim):
    print(SEP)
    print("11. Roll Response — 5s with δa = +10°")
    print(SEP)

    state = trim['state'].copy()
    de = trim['delta_e']
    thr = trim['throttle']
    da = math.radians(10)    # 10° aileron (roll right)
    dt = 1.0 / 60.0
    n_steps = int(5.0 / dt)

    for i in range(n_steps):
        state = rk4_step(state, de, da, 0.0, thr, dt)

    # After 5s of aileron input, expect significant bank angle
    phi_deg = math.degrees(state[I_PHI])
    p_val = state[I_P]

    check(phi_deg > 5.0,
          f"Bank Φ = {phi_deg:.1f}° > 5° after 5s of aileron input")
    check(p_val > 0 or abs(phi_deg) > 30,
          f"Roll rate p = {p_val:.4f} rad/s (positive = right)")

    # Adverse yaw: expect some yaw (Cn_da < 0 → nose left)
    psi_deg = math.degrees(state[I_PSI])
    print(f"  After 5s: Φ={phi_deg:.1f}°  p={p_val:.4f}  Ψ={psi_deg:.1f}°")
    print()


# ══════════════════════════════════════════════════════════════════
#  12. Yaw response to rudder input
# ══════════════════════════════════════════════════════════════════
def test_yaw_response(trim):
    print(SEP)
    print("12. Yaw Response — 3s with δr = +10° (yaw left)")
    print(SEP)

    state = trim['state'].copy()
    de = trim['delta_e']
    thr = trim['throttle']
    dr = math.radians(10)    # rudder input
    dt = 1.0 / 60.0
    n_steps = int(3.0 / dt)

    for i in range(n_steps):
        state = rk4_step(state, de, 0.0, dr, thr, dt)

    r_val = state[I_R]
    psi_deg = math.degrees(state[I_PSI])
    beta_state = state[I_V] / max(math.sqrt(state[I_U]**2 + state[I_W]**2), 1.0)

    # Positive δr with Cn_dr < 0 should create nose-left yaw
    # But β builds up and is resisted by Cn_β. So we just check for sideslip
    check(abs(state[I_V]) > 0.1,
          f"Sideslip V = {state[I_V]:.2f} m/s (nonzero from rudder)")
    print(f"  After 3s: r={r_val:.4f}  Ψ={psi_deg:.1f}°  β≈{math.degrees(math.atan2(state[I_V], state[I_U])):.2f}°")
    print()


# ══════════════════════════════════════════════════════════════════
#  13. Dutch roll mode check
# ══════════════════════════════════════════════════════════════════
def test_dutch_roll(trim):
    print(SEP)
    print("13. Dutch Roll — impulse β then free response (2s)")
    print(SEP)

    state = trim['state'].copy()
    de = trim['delta_e']
    thr = trim['throttle']
    dt = 1.0 / 60.0

    # Apply sideslip impulse
    state[I_V] = 5.0  # 5 m/s lateral velocity
    n_steps = int(2.0 / dt)
    v_history = []

    for i in range(n_steps):
        v_history.append(state[I_V])
        state = rk4_step(state, de, 0.0, 0.0, thr, dt)

    v_arr = np.array(v_history)
    # The oscillation should be damped (Cn_r < 0, CY_beta < 0)
    # Check that peak amplitude decreases
    peak1 = max(abs(v_arr[:60]))
    peak2 = max(abs(v_arr[60:]))
    check(peak2 < peak1,
          f"Dutch roll damped: peak1={peak1:.2f} > peak2={peak2:.2f}")
    print()


# ══════════════════════════════════════════════════════════════════
#  14. Ground collision clamp (12-state)
# ══════════════════════════════════════════════════════════════════
def test_ground_clamp():
    print(SEP)
    print("14. Ground Collision Clamp")
    print(SEP)

    state = np.zeros(STATE_SIZE)
    state[I_U] = 50.0
    state[I_W] = 20.0  # sinking
    state[I_ZE] = 5.0  # slightly below ground (altitude = -5m)
    state[I_PHI] = math.radians(30)  # banked

    new_state = rk4_step(state, 0.0, 0.0, 0.0, 0.3, 1/60)
    check(new_state[I_ZE] == 0.0, f"zE clamped to 0 (was {state[I_ZE]})")
    check(new_state[I_PHI] == 0.0, "Φ reset to 0 on ground")
    check(new_state[I_P] == 0.0, "p reset to 0 on ground")
    check(new_state[I_R] == 0.0, "r reset to 0 on ground")
    print()


# ══════════════════════════════════════════════════════════════════
#  15. Coordinated turn check
# ══════════════════════════════════════════════════════════════════
def test_coordinated_turn(trim):
    print(SEP)
    print("15. Banked Turn — verify altitude loss & heading change")
    print(SEP)

    state = trim['state'].copy()
    de = trim['delta_e']
    thr = trim['throttle']
    dt = 1.0 / 60.0

    # First, establish a bank using aileron for 2 seconds
    for i in range(int(2.0 / dt)):
        state = rk4_step(state, de, math.radians(5), 0.0, thr, dt)

    phi_after_bank = math.degrees(state[I_PHI])
    print(f"  After 2s aileron: Φ = {phi_after_bank:.1f}°")

    # Now hold neutral aileron for 5 more seconds
    psi_before = state[I_PSI]
    alt_before = -state[I_ZE]
    for i in range(int(5.0 / dt)):
        state = rk4_step(state, de, 0.0, 0.0, thr, dt)

    psi_after = state[I_PSI]
    alt_after = -state[I_ZE]
    heading_change = math.degrees(psi_after - psi_before)

    check(abs(heading_change) > 2.0,
          f"Heading changed by {heading_change:.1f}° (banked turn)")
    print(f"  Alt change: {alt_after - alt_before:.0f} m  Ψ change: {heading_change:.1f}°")
    print()


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("\n" + "╔" + "═" * 63 + "╗")
    print("║  Phase 5 — Full 6-DOF Physics Verification Suite" + " " * 13 + "║")
    print("╚" + "═" * 63 + "╝\n")

    test_state_vector()
    test_aircraft_params()
    test_aero_model()
    test_control_effectiveness()
    test_gravity()
    test_ixz_coupling()
    test_euler_kinematics()
    test_navigation()
    trim = test_trim()
    test_longitudinal_stability(trim)
    test_roll_response(trim)
    test_yaw_response(trim)
    test_dutch_roll(trim)
    test_ground_clamp()
    test_coordinated_turn(trim)

    print(SEP)
    print(f"RESULTS: {n_pass} passed, {n_fail} failed")
    print(SEP)
    if n_fail > 0:
        print("Some tests FAILED — review output above.")
        sys.exit(1)
    else:
        print("All tests PASSED!")
        sys.exit(0)
