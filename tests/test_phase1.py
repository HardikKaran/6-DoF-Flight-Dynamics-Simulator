"""
Quick smoke-test for Phase 1 physics.

Runs:
  1. ISA atmosphere spot-checks
  2. Aero model at a sample condition
  3. Trim solver at cruise
  4. 10-second RK4 simulation from trim → verify stability
"""

import sys, os, math
import numpy as np

# Ensure the flight_sim package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from flight_sim.physics import atmosphere as atm
from flight_sim.physics import aircraft as ac
from flight_sim.physics.aero import compute_aero
from flight_sim.physics.eom import (
    compute_derivatives, rk4_step, find_trim, make_initial_state,
    I_U, I_W, I_Q, I_TH, I_XE, I_ZE,
)

SEP = "=" * 60


def test_atmosphere():
    print(SEP)
    print("1. ISA Atmosphere Spot-Checks")
    print(SEP)
    for h in [0, 1000, 5000, 9144, 11000, 15000]:
        rho = atm.density(h)
        T = atm.temperature(h)
        a = atm.speed_of_sound(h)
        print(f"  h={h:>6} m  |  ρ={rho:.4f} kg/m³  |  T={T:.1f} K  |  a={a:.1f} m/s")
    # Sea-level check
    assert abs(atm.density(0) - 1.225) < 0.001, "Sea-level density mismatch"
    assert abs(atm.temperature(0) - 288.15) < 0.01, "Sea-level temperature mismatch"
    print("  ✓ Sea-level values correct\n")


def test_aero_sample():
    print(SEP)
    print("2. Aero Model — sample condition (V=180 m/s, α=3°, h=9144 m)")
    print(SEP)
    alpha = math.radians(3.0)
    V = 180.0
    U = V * math.cos(alpha)
    W = V * math.sin(alpha)
    theta = alpha  # level flight approximation
    q = 0.0
    de = 0.0
    throttle = 0.3

    fm = compute_aero(U, W, q, theta, de, throttle, 9144.0)
    for k in ["alpha", "V_T", "q_bar", "CL", "CD", "L_aero", "D_aero",
              "T", "X", "Z", "M"]:
        val = fm[k]
        unit = {"alpha": "rad", "V_T": "m/s", "q_bar": "Pa",
                "L_aero": "N", "D_aero": "N", "T": "N",
                "X": "N", "Z": "N", "M": "N·m"}.get(k, "")
        print(f"  {k:>8} = {val:>12.2f}  {unit}")
    print()


def test_trim():
    print(SEP)
    print("3. Trim Solver — cruise at M≈0.62, h=9144 m (30 000 ft)")
    print(SEP)
    a_cruise = atm.speed_of_sound(9144.0)
    V_cruise = ac.M_inf * a_cruise
    print(f"  Speed of sound at 30 000 ft: {a_cruise:.1f} m/s")
    print(f"  Target V_cruise: {V_cruise:.1f} m/s  (M={ac.M_inf})")

    trim = find_trim(V_cruise, 9144.0)
    print(f"  Converged: {trim['converged']}")
    print(f"  α_trim   = {math.degrees(trim['alpha']):.3f}°")
    print(f"  δe_trim  = {math.degrees(trim['delta_e']):.3f}°")
    print(f"  throttle = {trim['throttle']:.4f}  ({trim['throttle']*100:.1f}%)")
    print(f"  State    = {trim['state']}")
    print()
    return trim


def test_simulation(trim):
    print(SEP)
    print("4. 10-second RK4 Simulation from Trim")
    print(SEP)
    state = trim["state"].copy()
    de = trim["delta_e"]
    thr = trim["throttle"]
    dt = 1.0 / 60.0
    t = 0.0
    n_steps = int(10.0 / dt)

    print(f"  {'t':>6}  {'U':>8}  {'W':>8}  {'q':>10}  {'θ(°)':>8}  {'alt(m)':>8}")
    for i in range(n_steps):
        if i % 60 == 0:  # print every second
            alt = -state[I_ZE]
            print(f"  {t:6.2f}  {state[I_U]:8.2f}  {state[I_W]:8.2f}"
                  f"  {state[I_Q]:10.6f}  {math.degrees(state[I_TH]):8.3f}"
                  f"  {alt:8.1f}")
        state = rk4_step(state, de, thr, dt)
        t += dt

    alt_final = -state[I_ZE]
    alt_init = trim["state"][I_ZE] * -1
    drift = abs(alt_final - alt_init)
    print(f"\n  Final altitude: {alt_final:.1f} m  (drift: {drift:.2f} m)")
    if drift < 50.0:
        print("  ✓ Aircraft holds approximately level — trim looks good!\n")
    else:
        print(f"  ⚠ Altitude drifted by {drift:.1f} m — trim may need tuning.\n")


if __name__ == "__main__":
    test_atmosphere()
    test_aero_sample()
    trim = test_trim()
    test_simulation(trim)
    print("All Phase 1 tests complete.")
