"""
flight_sim.physics — Physics sub-package
=========================================
Contains:
    aircraft.py    — Piaggio P.180 Avanti parameters (dataclass + constants)
    atmosphere.py  — ISA atmosphere model  ρ(h), T(h), a(h)
    aero.py        — Aerodynamic force & moment model (longitudinal)
    eom.py         — Equations of motion + RK4 integrator + trim solver
"""
