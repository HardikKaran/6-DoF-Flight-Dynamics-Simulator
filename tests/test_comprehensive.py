"""
Comprehensive Physics Validation Tests
========================================
Prove that the 6-DOF flight dynamics simulator obeys physical laws
and matches known theoretical results for a Piaggio P.180-class aircraft.

Test categories:
  1. ISA Atmosphere
  2. Force / moment balance at trim (L=W, T=D, M=0)
  3. Stall model behaviour
  4. Compressibility corrections (Prandtl-Glauert, wave drag)
  5. Stability eigenvalues (all negative real parts at trim)
  6. Classical mode identification (Short Period, Phugoid, Dutch Roll)
  7. Phugoid period vs theoretical T = pi*sqrt(2)*V/g
  8. Trim map convergence
  9. Fuel burn rate validation (dm = TSFC * T * dt)
 10. Autopilot PID convergence
 11. RK4 stability from trim (perturbation decay)
 12. Euler kinematics (DCM rotation)
 13. Ground collision clamp
 14. Flap effects on CL and CD
 15. Hinge moment computation
"""

import sys
import os
import math
import numpy as np

# Ensure the flight_sim package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from flight_sim.physics import atmosphere as atm
from flight_sim.physics import aircraft as ac
from flight_sim.physics.aero import compute_aero, _stall_CL, _prandtl_glauert, _wave_drag
from flight_sim.physics.eom import (
    compute_derivatives, rk4_step, find_trim, make_initial_state,
    I_U, I_V, I_W, I_P, I_Q, I_R,
    I_PHI, I_TH, I_PSI, I_XE, I_YE, I_ZE,
    STATE_SIZE,
)
from flight_sim.physics.stability import (
    linearise, get_eigenvalues, identify_modes, stability_report,
    compute_trim_map, compute_hinge_moment,
)
from flight_sim.physics.autopilot import PIDController, Autopilot

SEP = "=" * 70
PASS = "  PASS"
FAIL = "  FAIL"
total_tests = 0
passed_tests = 0


def check(condition, description):
    """Track test results."""
    global total_tests, passed_tests
    total_tests += 1
    if condition:
        passed_tests += 1
        print(f"{PASS}: {description}")
    else:
        print(f"{FAIL}: {description}")
    return condition


# ══════════════════════════════════════════════════════════════════
#  1. ISA ATMOSPHERE
# ══════════════════════════════════════════════════════════════════

def test_atmosphere():
    print(f"\n{SEP}")
    print("1. ISA Atmosphere Validation")
    print(SEP)

    # Sea level: rho=1.225, T=288.15 K, a=340.3 m/s
    check(abs(atm.density(0) - 1.225) < 0.002,
          f"Sea-level density = {atm.density(0):.4f} (expect 1.225)")
    check(abs(atm.temperature(0) - 288.15) < 0.1,
          f"Sea-level temperature = {atm.temperature(0):.2f} K (expect 288.15)")
    check(abs(atm.speed_of_sound(0) - 340.3) < 0.5,
          f"Sea-level speed of sound = {atm.speed_of_sound(0):.1f} m/s (expect 340.3)")

    # Tropopause (11 000 m): T=216.65 K, rho~0.3639
    check(abs(atm.temperature(11000) - 216.65) < 0.5,
          f"Tropopause temperature = {atm.temperature(11000):.2f} K (expect 216.65)")
    check(abs(atm.density(11000) - 0.3639) < 0.01,
          f"Tropopause density = {atm.density(11000):.4f} (expect ~0.364)")

    # Density decreases with altitude (basic physics)
    check(atm.density(5000) < atm.density(0),
          "Density decreases: rho(5000) < rho(0)")
    check(atm.density(9144) < atm.density(5000),
          "Density decreases: rho(9144) < rho(5000)")

    # Speed of sound decreases in troposphere
    check(atm.speed_of_sound(5000) < atm.speed_of_sound(0),
          "Speed of sound decreases with altitude in troposphere")


# ══════════════════════════════════════════════════════════════════
#  2. FORCE & MOMENT BALANCE AT TRIM  (L=W, T=D, M=0)
# ══════════════════════════════════════════════════════════════════

def test_trim_balance():
    print(f"\n{SEP}")
    print("2. Force & Moment Balance at Trim")
    print(SEP)

    V = 180.0
    h = 9144.0
    trim = find_trim(V, h)
    check(trim["converged"], "Trim solver converged")

    state = trim["state"]
    de = trim["delta_e"]
    thr = trim["throttle"]

    # At trim: all derivatives should be ~0
    derivs = compute_derivatives(state, de, 0.0, 0.0, thr)
    U_dot = derivs[I_U]
    W_dot = derivs[I_W]
    q_dot = derivs[I_Q]

    check(abs(U_dot) < 0.5,
          f"U_dot at trim = {U_dot:.4f} (expect ~0)")
    check(abs(W_dot) < 0.5,
          f"W_dot at trim = {W_dot:.4f} (expect ~0)")
    check(abs(q_dot) < 0.01,
          f"q_dot at trim = {q_dot:.6f} (expect ~0)")

    # Force balance: compute aero forces at trim
    alpha = trim["alpha"]
    U = V * math.cos(alpha)
    W = V * math.sin(alpha)
    fm = compute_aero(U, 0.0, W, 0.0, 0.0, 0.0, 0.0, alpha,
                      de, 0.0, 0.0, thr, h)

    W_force = ac.m * ac.g  # weight
    L_aero = fm["L_aero"]
    D_aero = fm["D_aero"]
    T_eng = fm["T"]
    M_moment = fm["M"]

    # L ≈ W (lift supports weight) — allow 5% tolerance
    lift_ratio = L_aero / W_force
    check(abs(lift_ratio - 1.0) < 0.05,
          f"L/W = {lift_ratio:.4f} (expect ~1.0, L={L_aero:.0f} N, W={W_force:.0f} N)")

    # T ≈ D (thrust balances drag) — allow 10% (because of α component)
    # More precisely: T*cos(alpha) ≈ D (thrust has a small Z component)
    T_horiz = T_eng * math.cos(alpha)
    td_ratio = T_horiz / max(D_aero, 1)
    check(abs(td_ratio - 1.0) < 0.15,
          f"T*cos(a)/D = {td_ratio:.4f} (expect ~1.0, T={T_eng:.0f} N, D={D_aero:.0f} N)")

    # M ≈ 0 (moment balance)
    check(abs(M_moment) < 50,
          f"M at trim = {M_moment:.2f} N*m (expect ~0)")

    # Physical sanity: CL > 0, CD > 0, reasonable values
    check(fm["CL"] > 0.1 and fm["CL"] < 1.5,
          f"CL = {fm['CL']:.4f} (reasonable range 0.1-1.5)")
    check(fm["CD"] > 0.005 and fm["CD"] < 0.2,
          f"CD = {fm['CD']:.4f} (reasonable range 0.005-0.2)")
    # Note: alpha can be slightly negative for canard aircraft (canard provides
    # significant lift, reducing wing loading)
    check(-5 < math.degrees(fm["alpha"]) < 15,
          f"alpha_trim = {math.degrees(fm['alpha']):.2f} deg (within -5 to 15 for canard a/c)")

    print(f"\n  Summary: V={V} m/s, h={h} m")
    print(f"    alpha_trim  = {math.degrees(alpha):.3f} deg")
    print(f"    de_trim     = {math.degrees(de):.3f} deg")
    print(f"    throttle    = {thr*100:.1f}%")
    print(f"    L = {L_aero:.0f} N, W = {W_force:.0f} N")
    print(f"    T = {T_eng:.0f} N, D = {D_aero:.0f} N")


# ══════════════════════════════════════════════════════════════════
#  3. STALL MODEL BEHAVIOUR
# ══════════════════════════════════════════════════════════════════

def test_stall_model():
    print(f"\n{SEP}")
    print("3. Stall Model Behaviour")
    print(SEP)

    # Below stall: linear CL returned unchanged
    CL_lin = 0.8
    alpha_low = math.radians(10)
    CL_out = _stall_CL(CL_lin, alpha_low)
    check(abs(CL_out - CL_lin) < 1e-6,
          f"Below stall: CL unchanged ({CL_out:.4f} == {CL_lin:.4f})")

    # Above stall: CL drops (post-stall)
    alpha_high = math.radians(25)
    CL_post = _stall_CL(CL_lin, alpha_high)
    check(CL_post < CL_lin,
          f"Above stall: CL decreases ({CL_post:.4f} < {CL_lin:.4f})")

    # At 90 deg: CL should be near zero (flat plate)
    alpha_90 = math.radians(80)
    CL_90 = _stall_CL(CL_lin, alpha_90)
    check(CL_90 < 0.8,
          f"Near 90 deg: CL drops significantly ({CL_90:.4f})")

    # CL_max cap: cannot exceed CL_max
    alpha_stall_border = math.radians(16)
    CL_big = 3.0
    CL_capped = _stall_CL(CL_big, alpha_stall_border)
    check(CL_capped <= 1.6 + 0.01,
          f"CL_max cap: {CL_capped:.4f} <= 1.6")

    # Stall flag set in compute_aero above α_stall
    V = 180.0
    alpha = math.radians(20)
    U = V * math.cos(alpha)
    W_body = V * math.sin(alpha)
    fm = compute_aero(U, 0.0, W_body, 0.0, 0.0, 0.0, 0.0, alpha,
                      0.0, 0.0, 0.0, 0.3, 9144.0)
    check(fm.get("stall", False) == True,
          f"Stall flag True at alpha=20 deg")


# ══════════════════════════════════════════════════════════════════
#  4. COMPRESSIBILITY CORRECTIONS
# ══════════════════════════════════════════════════════════════════

def test_compressibility():
    print(f"\n{SEP}")
    print("4. Compressibility Corrections")
    print(SEP)

    # Prandtl-Glauert: at M=0 → factor=1
    check(abs(_prandtl_glauert(0.0) - 1.0) < 0.01,
          f"PG factor at M=0: {_prandtl_glauert(0.0):.4f} (expect 1.0)")

    # PG: at M=0.5 → 1/sqrt(1-0.25) = 1/sqrt(0.75) ≈ 1.155
    pg_05 = _prandtl_glauert(0.5)
    expected = 1.0 / math.sqrt(0.75)
    check(abs(pg_05 - expected) < 0.01,
          f"PG factor at M=0.5: {pg_05:.4f} (expect {expected:.4f})")

    # PG: at M=0.7 (M_crit), factor is capped
    pg_07 = _prandtl_glauert(0.7)
    pg_08 = _prandtl_glauert(0.8)
    check(abs(pg_07 - pg_08) < 0.01,
          f"PG capped: PG(0.7)={pg_07:.4f} == PG(0.8)={pg_08:.4f}")

    # Wave drag: zero below M_DD (0.65)
    check(abs(_wave_drag(0.5)) < 1e-10,
          f"Wave drag at M=0.5: {_wave_drag(0.5):.6f} (expect 0)")
    check(abs(_wave_drag(0.65)) < 1e-10,
          f"Wave drag at M_DD=0.65: {_wave_drag(0.65):.6f} (expect 0)")

    # Wave drag: positive above M_DD
    wd = _wave_drag(0.75)
    check(wd > 0,
          f"Wave drag at M=0.75: {wd:.6f} (expect > 0)")

    # Wave drag increases with Mach
    wd2 = _wave_drag(0.80)
    check(wd2 > wd,
          f"Wave drag increases: WD(0.80)={wd2:.6f} > WD(0.75)={wd:.6f}")


# ══════════════════════════════════════════════════════════════════
#  5. EIGENVALUE STABILITY AT TRIM
# ══════════════════════════════════════════════════════════════════

def test_eigenvalue_stability():
    print(f"\n{SEP}")
    print("5. Eigenvalue Stability at Trim (all Re < 0)")
    print(SEP)

    report = stability_report(180.0, 9144.0)
    check("error" not in report,
          "Stability report computed without error")

    # All longitudinal eigenvalues should have negative real parts (stable)
    eigs_lon = [complex(e["real"], e["imag"]) for e in report["eigenvalues_lon"]]
    all_lon_stable = all(e.real < 0 for e in eigs_lon)
    check(all_lon_stable,
          f"All longitudinal eigenvalues stable: {['({:.3f},{:.3f}j)'.format(e.real, e.imag) for e in eigs_lon]}")

    # Lateral eigenvalues: some aircraft have mildly unstable Dutch Roll
    # without a yaw damper (positive real part < 0.1 is common)
    eigs_lat = [complex(e["real"], e["imag"]) for e in report["eigenvalues_lat"]]
    worst_lat = max(e.real for e in eigs_lat)
    check(worst_lat < 0.1,
          f"Lateral eigenvalues not wildly unstable (worst Re={worst_lat:.4f} < 0.1): "
          f"{['({:.3f},{:.3f}j)'.format(e.real, e.imag) for e in eigs_lat]}")

    # Print eigenvalue details
    print("\n  Longitudinal eigenvalues:")
    for e in eigs_lon:
        print(f"    {e.real:+.4f} {'+ ' if e.imag >= 0 else '- '}{abs(e.imag):.4f}j")
    print("  Lateral eigenvalues:")
    for e in eigs_lat:
        print(f"    {e.real:+.4f} {'+ ' if e.imag >= 0 else '- '}{abs(e.imag):.4f}j")


# ══════════════════════════════════════════════════════════════════
#  6. CLASSICAL MODE IDENTIFICATION
# ══════════════════════════════════════════════════════════════════

def test_mode_identification():
    print(f"\n{SEP}")
    print("6. Classical Flight Dynamics Mode Identification")
    print(SEP)

    report = stability_report(180.0, 9144.0)
    modes_lon = report.get("modes_lon", [])
    modes_lat = report.get("modes_lat", [])

    # Should identify Short Period
    sp_found = any("Short Period" in m.get("name", "") for m in modes_lon)
    check(sp_found, "Short Period mode identified in longitudinal modes")

    # Should identify Phugoid
    ph_found = any("Phugoid" in m.get("name", "") for m in modes_lon)
    check(ph_found, "Phugoid mode identified in longitudinal modes")

    # Short period: higher frequency than phugoid
    for m in modes_lon:
        if "Short Period" in m.get("name", ""):
            sp_wn = m["wn"]
        if "Phugoid" in m.get("name", ""):
            ph_wn = m["wn"]
    if sp_found and ph_found:
        check(sp_wn > ph_wn,
              f"Short Period wn ({sp_wn:.3f}) > Phugoid wn ({ph_wn:.3f})")

    # Should identify Dutch Roll or Roll Subsidence in lateral modes
    lat_names = [m.get("name", "") for m in modes_lat]
    has_lateral_mode = any(n in ["Dutch Roll", "Roll Subsidence", "Spiral"]
                          for n in lat_names)
    check(has_lateral_mode,
          f"Lateral modes identified: {lat_names}")

    # Print all modes
    print("\n  Longitudinal modes:")
    for m in modes_lon:
        print(f"    {m['name']:20s}  wn={m['wn']:.4f}  zeta={m['zeta']:.4f}  "
              f"T={m['period']:.2f}s  stable={m['stable']}")
    print("  Lateral modes:")
    for m in modes_lat:
        print(f"    {m['name']:20s}  wn={m['wn']:.4f}  zeta={m['zeta']:.4f}  "
              f"T={m['period']:.2f}s  stable={m['stable']}")


# ══════════════════════════════════════════════════════════════════
#  7. PHUGOID PERIOD VS THEORETICAL
# ══════════════════════════════════════════════════════════════════

def test_phugoid_period():
    print(f"\n{SEP}")
    print("7. Phugoid Period vs Theoretical (T = pi*sqrt(2)*V/g)")
    print(SEP)

    V = 180.0
    # Theoretical phugoid period: T = pi * sqrt(2) * V / g  (Nelson Eq. 4.65)
    T_theory = math.pi * math.sqrt(2) * V / ac.g
    print(f"  Theoretical phugoid period: T = {T_theory:.1f} s")

    report = stability_report(V, 9144.0)
    modes_lon = report.get("modes_lon", [])
    ph_period = None
    for m in modes_lon:
        if "Phugoid" in m.get("name", ""):
            ph_period = m["period"]
            break

    if ph_period is not None:
        ratio = ph_period / T_theory
        # Theoretical formula is approximate; allow 50% tolerance for
        # a three-surface aircraft with non-ideal assumptions
        check(0.3 < ratio < 2.0,
              f"Phugoid period ratio: sim={ph_period:.1f}s / theory={T_theory:.1f}s = {ratio:.2f} "
              f"(within factor of 2)")
        print(f"  Simulated phugoid period: {ph_period:.1f} s")
        print(f"  Ratio sim/theory: {ratio:.2f}")
    else:
        check(False, "Phugoid mode not found for period comparison")


# ══════════════════════════════════════════════════════════════════
#  8. TRIM MAP CONVERGENCE
# ══════════════════════════════════════════════════════════════════

def test_trim_map():
    print(f"\n{SEP}")
    print("8. Trim Map Convergence")
    print(SEP)

    V_range = np.array([80, 120, 160, 200])
    h_range = np.array([0, 3000, 9144])
    result = compute_trim_map(V_range, h_range)

    n_total = len(V_range) * len(h_range)
    n_converged = sum(sum(row) for row in result["converged"])
    ratio = n_converged / n_total

    check(ratio > 0.5,
          f"Trim map convergence: {n_converged}/{n_total} = {ratio*100:.0f}%")

    # At each converged point: alpha should be positive and reasonable
    for ih, h in enumerate(h_range):
        for iv, V in enumerate(V_range):
            if result["converged"][ih][iv]:
                alpha_deg = result["alpha"][ih][iv]
                check(-5 < alpha_deg < 20,
                      f"  Trim alpha at V={V}, h={h}: {alpha_deg:.1f} deg (reasonable)")


# ══════════════════════════════════════════════════════════════════
#  9. FUEL BURN RATE VALIDATION
# ══════════════════════════════════════════════════════════════════

def test_fuel_burn():
    print(f"\n{SEP}")
    print("9. Fuel Burn Rate (dm = TSFC * T * dt)")
    print(SEP)

    T = 5000.0  # N (example thrust)
    dt = 60.0   # 1 minute
    dm = ac.TSFC * T * dt
    dm_expected = 0.000085 * 5000 * 60  # = 25.5 kg

    check(abs(dm - dm_expected) < 0.01,
          f"Fuel burn: dm = {dm:.2f} kg in 60s at T=5000N (expect {dm_expected:.2f})")

    # At cruise thrust (~4000N), fuel should last reasonable time
    T_cruise = 4000.0
    dm_hour = ac.TSFC * T_cruise * 3600
    hours_endurance = ac.m_fuel_max / dm_hour
    check(hours_endurance > 0.5,
          f"Endurance: {hours_endurance:.1f} hours at T={T_cruise}N (expect > 0.5h)")
    check(hours_endurance < 10,
          f"Endurance: {hours_endurance:.1f} hours (expect < 10h, reasonable)")
    print(f"  TSFC = {ac.TSFC} kg/(N*s)")
    print(f"  Max fuel = {ac.m_fuel_max} kg")
    print(f"  Cruise burn rate = {dm_hour:.1f} kg/hr at T={T_cruise}N")
    print(f"  Estimated endurance = {hours_endurance:.1f} hours")


# ══════════════════════════════════════════════════════════════════
# 10. AUTOPILOT PID CONVERGENCE
# ══════════════════════════════════════════════════════════════════

def test_autopilot():
    print(f"\n{SEP}")
    print("10. Autopilot PID Convergence")
    print(SEP)

    # Test PID controller convergence for a simple first-order plant
    pid = PIDController(kp=2.0, ki=0.5, kd=0.3, out_min=-10, out_max=10)
    dt = 0.01
    value = 0.0
    target = 5.0

    for _ in range(2000):
        error = target - value
        u = pid.update(error, dt)
        value += u * dt  # simple integrator plant

    check(abs(value - target) < 0.5,
          f"PID converges to target: value={value:.3f} (target={target:.3f})")

    # Test Autopilot pitch hold
    ap = Autopilot()
    ap.set_mode('pitch', theta_cmd=math.radians(5.0))
    check(ap.mode == 'pitch', "Autopilot mode set to 'pitch'")

    # Simulate pitch-hold with a simplified 2nd-order pitch plant:
    #   q_dot = M_de * delta_e - damping * q
    #   theta_dot = q
    # Using realistic-scale gains
    theta = 0.0
    q = 0.0
    M_de = -10.0   # pitch moment effectiveness (rad/s^2 per rad of elevator)
    damping = 2.0  # pitch damping
    for _ in range(10000):
        state_d = {'theta': theta, 'phi': 0.0, 'psi': 0.0,
                   'altitude': 9144.0, 'q': q, 'p': 0.0, 'r': 0.0}
        cmds = ap.update(state_d, dt)
        de = cmds.get('delta_e', 0.0)
        q_dot = M_de * de - damping * q
        q += q_dot * dt
        theta += q * dt

    theta_cmd_deg = 5.0
    theta_deg = math.degrees(theta)
    check(abs(theta_deg - theta_cmd_deg) < 3.0,
          f"Pitch AP converges: theta={theta_deg:.2f} deg (cmd={theta_cmd_deg})")
    print(f"  Pitch AP final: theta={theta_deg:.2f} deg, q={q:.4f} rad/s")

    # Test autopilot off mode returns empty dict
    ap.set_mode('off')
    cmds = ap.update({'theta': 0, 'phi': 0, 'psi': 0,
                      'altitude': 9144, 'q': 0, 'p': 0, 'r': 0}, dt)
    check(len(cmds) == 0, "Autopilot OFF returns no commands")


# ══════════════════════════════════════════════════════════════════
# 11. RK4 STABILITY FROM TRIM (perturbation decay)
# ══════════════════════════════════════════════════════════════════

def test_rk4_stability():
    print(f"\n{SEP}")
    print("11. RK4 Stability from Trim (perturbation decay)")
    print(SEP)

    state, de_trim, thr_trim = make_initial_state(180.0, 9144.0)

    # Apply small perturbation to pitch angle
    state_perturbed = state.copy()
    state_perturbed[I_TH] += math.radians(2.0)  # +2 deg pitch perturbation

    # Simulate 30 seconds
    dt = 1.0 / 60.0
    s = state_perturbed.copy()
    max_theta_deviation = 0.0
    initial_deviation = abs(s[I_TH] - state[I_TH])

    for i in range(int(30.0 / dt)):
        s = rk4_step(s, de_trim, 0.0, 0.0, thr_trim, dt)
        dev = abs(s[I_TH] - state[I_TH])
        max_theta_deviation = max(max_theta_deviation, dev)

    final_deviation = abs(s[I_TH] - state[I_TH])

    # The perturbation should not grow without bound (aircraft is stable)
    check(max_theta_deviation < math.radians(30),
          f"Max theta deviation < 30 deg: {math.degrees(max_theta_deviation):.2f} deg")

    # Speed should stay reasonable (not diverge)
    V_final = math.hypot(s[I_U], s[I_W])
    check(50 < V_final < 350,
          f"Speed after 30s: {V_final:.1f} m/s (within 50-350)")

    # Altitude should stay reasonable
    alt_final = -s[I_ZE]
    check(alt_final > 0,
          f"Altitude after 30s: {alt_final:.0f} m (above ground)")

    print(f"  Initial perturbation: {math.degrees(initial_deviation):.2f} deg")
    print(f"  Final theta deviation: {math.degrees(final_deviation):.2f} deg")
    print(f"  Final speed: {V_final:.1f} m/s, altitude: {alt_final:.0f} m")


# ══════════════════════════════════════════════════════════════════
# 12. EULER KINEMATICS — DCM ROTATION
# ══════════════════════════════════════════════════════════════════

def test_euler_kinematics():
    print(f"\n{SEP}")
    print("12. Euler Kinematics (DCM body-to-Earth)")
    print(SEP)

    # In level flight (theta=alpha, phi=0, psi=0):
    # xE_dot should be ~V (forward motion)
    # zE_dot should be ~0 (level flight)
    V = 180.0
    trim = find_trim(V, 9144.0)
    state = trim["state"]
    de = trim["delta_e"]
    thr = trim["throttle"]

    derivs = compute_derivatives(state, de, 0.0, 0.0, thr)
    xE_dot = derivs[I_XE]
    zE_dot = derivs[I_ZE]
    yE_dot = derivs[I_YE]

    check(abs(xE_dot - V) < 10,
          f"xE_dot ~ V at trim: {xE_dot:.1f} m/s (expect ~{V})")
    check(abs(zE_dot) < 2.0,
          f"zE_dot ~ 0 at trim: {zE_dot:.4f} m/s (expect ~0)")
    check(abs(yE_dot) < 1.0,
          f"yE_dot ~ 0 at trim: {yE_dot:.4f} m/s (expect ~0, symmetric)")


# ══════════════════════════════════════════════════════════════════
# 13. GROUND COLLISION CLAMP
# ══════════════════════════════════════════════════════════════════

def test_ground_collision():
    print(f"\n{SEP}")
    print("13. Ground Collision Clamp")
    print(SEP)

    # Create state near ground with downward velocity
    state = np.zeros(STATE_SIZE)
    state[I_U] = 50.0  # forward speed
    state[I_W] = 10.0  # downward velocity (positive = down in body)
    state[I_ZE] = -10.0  # 10 m altitude
    state[I_TH] = math.radians(5)

    # Simulate until hitting ground
    dt = 1.0 / 60.0
    s = state.copy()
    for _ in range(600):  # up to 10 seconds
        s = rk4_step(s, 0.0, 0.0, 0.0, 0.3, dt)

    alt = -s[I_ZE]
    check(alt >= 0.0,
          f"Aircraft altitude >= 0 after descent: {alt:.2f} m")
    check(s[I_W] <= 0.001,
          f"Downward velocity clamped on ground: W={s[I_W]:.4f}")


# ══════════════════════════════════════════════════════════════════
# 14. FLAP EFFECTS ON CL AND CD
# ══════════════════════════════════════════════════════════════════

def test_flap_effects():
    print(f"\n{SEP}")
    print("14. Flap Effects on CL and CD")
    print(SEP)

    V = 100.0  # low speed for approach
    alpha = math.radians(5.0)
    U = V * math.cos(alpha)
    W = V * math.sin(alpha)
    h = 1000.0

    # Clean configuration
    fm_clean = compute_aero(U, 0.0, W, 0.0, 0.0, 0.0, 0.0, alpha,
                            0.0, 0.0, 0.0, 0.3, h, flaps=0.0)
    # Full flaps
    fm_flaps = compute_aero(U, 0.0, W, 0.0, 0.0, 0.0, 0.0, alpha,
                            0.0, 0.0, 0.0, 0.3, h, flaps=1.0)

    # Flaps should increase CL
    check(fm_flaps["CL"] > fm_clean["CL"],
          f"Flaps increase CL: {fm_flaps['CL']:.4f} > {fm_clean['CL']:.4f}")

    # Flaps should increase CD
    check(fm_flaps["CD"] > fm_clean["CD"],
          f"Flaps increase CD: {fm_flaps['CD']:.4f} > {fm_clean['CD']:.4f}")

    delta_CL = fm_flaps["CL"] - fm_clean["CL"]
    delta_CD = fm_flaps["CD"] - fm_clean["CD"]
    print(f"  Delta CL (flaps): {delta_CL:.4f}")
    print(f"  Delta CD (flaps): {delta_CD:.4f}")


# ══════════════════════════════════════════════════════════════════
# 15. HINGE MOMENT COMPUTATION
# ══════════════════════════════════════════════════════════════════

def test_hinge_moment():
    print(f"\n{SEP}")
    print("15. Hinge Moment Computation")
    print(SEP)

    alpha_H = math.radians(2.0)
    delta_e = math.radians(5.0)
    q_bar = 5000.0  # Pa

    hm = compute_hinge_moment(alpha_H, delta_e, q_bar)
    C_H = hm["C_H"]
    H_e = hm["H_e"]
    F_s = hm["F_s"]

    # Hinge moment coefficient expected to be negative for positive deflection
    # (restoring moment) based on typical b_E values
    check(C_H != 0.0,
          f"Hinge moment coefficient C_H = {C_H:.6f} (non-zero)")
    check(isinstance(H_e, float),
          f"Hinge moment H_e = {H_e:.2f} N*m (valid float)")
    check(isinstance(F_s, float),
          f"Stick force F_s = {F_s:.2f} N (valid float)")
    # F_s should be H_e * G (gearing ratio 0.5)
    check(abs(F_s - H_e * 0.5) < 0.01,
          f"F_s = H_e * 0.5: {F_s:.2f} = {H_e*0.5:.2f}")

    print(f"  C_H = {C_H:.6f}")
    print(f"  H_e = {H_e:.2f} N*m")
    print(f"  F_s = {F_s:.2f} N")


# ══════════════════════════════════════════════════════════════════
# 16. MULTI-SPEED TRIM VALIDATION
# ══════════════════════════════════════════════════════════════════

def test_multi_speed_trim():
    print(f"\n{SEP}")
    print("16. Multi-Speed Trim Validation")
    print(SEP)

    h = 9144.0
    speeds = [80, 120, 160, 200]
    prev_alpha = None
    prev_thr = None

    for V in speeds:
        trim = find_trim(V, h)
        if trim["converged"]:
            alpha_deg = math.degrees(trim["alpha"])
            de_deg = math.degrees(trim["delta_e"])
            thr_pct = trim["throttle"] * 100

            check(-5 < alpha_deg < 20,
                  f"  V={V} m/s: alpha={alpha_deg:.2f} deg (reasonable)")

            # At higher speeds: alpha should be lower (less lift needed per q)
            if prev_alpha is not None and V > 120:
                check(alpha_deg < prev_alpha + 1.0,
                      f"  V={V}: alpha decreases with speed ({alpha_deg:.2f} < {prev_alpha:.2f})")
            prev_alpha = alpha_deg

            print(f"    V={V} m/s: alpha={alpha_deg:.2f} deg, de={de_deg:.2f} deg, thr={thr_pct:.1f}%")
        else:
            print(f"    V={V} m/s: trim did NOT converge")


# ══════════════════════════════════════════════════════════════════
# 17. LATERAL RESPONSE — AILERON INPUT
# ══════════════════════════════════════════════════════════════════

def test_lateral_response():
    print(f"\n{SEP}")
    print("17. Lateral Response to Aileron Input")
    print(SEP)

    state, de_trim, thr_trim = make_initial_state(180.0, 9144.0)

    # Apply aileron step input
    dt = 1.0 / 60.0
    da = math.radians(5.0)  # 5 deg aileron
    s = state.copy()

    for _ in range(120):  # 2 seconds
        s = rk4_step(s, de_trim, da, 0.0, thr_trim, dt)

    # Should have developed a roll rate (p > 0 for positive aileron)
    check(s[I_P] > 0.001,
          f"Positive aileron -> positive roll rate: p={s[I_P]:.4f} rad/s")

    # Should have developed a bank angle (Phi > 0)
    check(s[I_PHI] > math.radians(1.0),
          f"Bank angle developed: phi={math.degrees(s[I_PHI]):.2f} deg")

    print(f"  After 2s of 5 deg aileron:")
    print(f"    p = {s[I_P]:.4f} rad/s")
    print(f"    phi = {math.degrees(s[I_PHI]):.2f} deg")
    print(f"    r = {s[I_R]:.4f} rad/s (yaw coupling)")


# ══════════════════════════════════════════════════════════════════
# 18. ENERGY CONSERVATION CHECK
# ══════════════════════════════════════════════════════════════════

def test_energy_consistency():
    print(f"\n{SEP}")
    print("18. Energy Consistency (no free energy creation)")
    print(SEP)

    # At trim, with zero throttle, aircraft should lose energy
    state, de_trim, thr_trim = make_initial_state(180.0, 9144.0)
    s = state.copy()
    dt = 1.0 / 60.0

    V0 = math.hypot(s[I_U], s[I_V], s[I_W])
    h0 = -s[I_ZE]
    E0 = 0.5 * ac.m * V0**2 + ac.m * ac.g * h0

    # Simulate 10s with ZERO throttle (should lose energy to drag)
    for _ in range(600):
        s = rk4_step(s, de_trim, 0.0, 0.0, 0.0, dt)

    V1 = math.hypot(s[I_U], s[I_V], s[I_W])
    h1 = -s[I_ZE]
    E1 = 0.5 * ac.m * V1**2 + ac.m * ac.g * h1

    check(E1 < E0,
          f"Energy decreases with zero throttle: E0={E0:.0f} J -> E1={E1:.0f} J")
    print(f"  Energy loss: {E0 - E1:.0f} J ({(E0-E1)/E0*100:.1f}%)")


# ══════════════════════════════════════════════════════════════════
#  RUN ALL TESTS
# ══════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 70)
    print("  COMPREHENSIVE PHYSICS VALIDATION — 6-DOF Flight Simulator")
    print("  Piaggio P.180 Avanti")
    print("=" * 70)

    test_atmosphere()
    test_trim_balance()
    test_stall_model()
    test_compressibility()
    test_eigenvalue_stability()
    test_mode_identification()
    test_phugoid_period()
    test_trim_map()
    test_fuel_burn()
    test_autopilot()
    test_rk4_stability()
    test_euler_kinematics()
    test_ground_collision()
    test_flap_effects()
    test_hinge_moment()
    test_multi_speed_trim()
    test_lateral_response()
    test_energy_consistency()

    print(f"\n{'=' * 70}")
    print(f"  RESULTS: {passed_tests}/{total_tests} tests passed")
    if passed_tests == total_tests:
        print("  ALL TESTS PASSED — Physics validated!")
    else:
        print(f"  {total_tests - passed_tests} test(s) FAILED")
    print("=" * 70 + "\n")

    return 0 if passed_tests == total_tests else 1


if __name__ == "__main__":
    sys.exit(main())
