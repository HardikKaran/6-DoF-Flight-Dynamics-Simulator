"""
flight_sim.physics — Physics sub-package
=========================================
Contains:
    aircraft.py    — Piaggio P.180 Avanti parameters (dataclass + constants)
    atmosphere.py  — ISA atmosphere model  rho(h), T(h), a(h)
    aero.py        — Aerodynamic force & moment model (full 6-DOF)
    eom.py         — Equations of motion + RK4 integrator + trim solver
    stability.py   — Linearisation, eigenvalue analysis, trim map
    autopilot.py   — PID autopilot (pitch-hold, alt-hold, heading-hold)
"""
