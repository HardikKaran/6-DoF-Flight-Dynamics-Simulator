"""
Piaggio P.180 Avanti — Aircraft Parameters
===========================================
Three-surface configuration: canard + wing + horizontal tail.
Data sourced from AERO50002 lecture notes and tutorial 2.

All angles stored in RADIANS unless noted otherwise.
All SI units (m, kg, s, rad).
"""

import math
from dataclasses import dataclass, field


# ── helper ────────────────────────────────────────────────────────
def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


# ══════════════════════════════════════════════════════════════════
#  GEOMETRY 
# ══════════════════════════════════════════════════════════════════
length        = 14.408 # m   
wingspan      = 14.033 # m   
height        =  3.980 # m   
canardSpan    =  4.259 # m   
tailplaneSpan =  3.356 # m   
engineSpacing =  2.844 # m between engine centrelines
propDiameter  =  2.159 # m 
dihedral      =  2.0   # degrees


# ══════════════════════════════════════════════════════════════════
#  LIFTING-SURFACE DATA 
# ══════════════════════════════════════════════════════════════════

@dataclass
class LiftingSurface:
    """Geometric and aerodynamic parameters for one lifting surface."""
    name: str

    # Geometry
    S: float            # planform area [m²]
    c_bar: float        # mean aerodynamic chord [m]
    x_ac: float         # longitudinal AC position (from datum) [m]
    z_ac: float         # vertical AC position (above baseline) [m]
    AR: float           # aspect ratio
    sweep_qc: float     # quarter-chord sweep [rad]

    # Aerodynamics
    a: float            # lift-curve slope [1/rad]
    alpha_0: float      # zero-lift angle of attack [rad]
    i: float            # setting (incidence) angle [rad]
    CM0: float          # zero-lift pitching-moment coefficient
    k: float            # induced-drag k-factor

    # Downwash / upwash
    de_da: float = 0.0  # dε/dα  (downwash or upwash gradient)


wing = LiftingSurface(
    name     = "Wing",
    S        = 16.00,
    c_bar    = 1.18,
    x_ac     = 7.59,
    z_ac     = 0.87,
    AR       = 12.31,
    sweep_qc = deg2rad(-0.50),
    a        = 8.17, # 1/rad
    alpha_0  = deg2rad(-2.40),
    i        = deg2rad(1.60),
    CM0      = -0.46,
    k        = 1.1,
    de_da    = 0.0, # not applicable for wing itself
)

canard = LiftingSurface(
    name     = "Canard",
    S        = 2.19,
    c_bar    = 0.65,
    x_ac     = 0.62,
    z_ac     = 0.07,
    AR       = 5.14,
    sweep_qc = deg2rad(0.00),
    a        = 3.40, # 1/rad
    alpha_0  = deg2rad(-1.40),
    i        = deg2rad(0.00),
    CM0      = -0.0429,
    k        = 2.21,
    de_da    = 0.15, # upwash gradient  dε_U/dα
)

tail = LiftingSurface(
    name     = "Horizontal Tail",
    S        = 3.83,
    c_bar    = 0.92,
    x_ac     = 13.00,
    z_ac     = 2.99,
    AR       = 4.74,
    sweep_qc = deg2rad(30.00),
    a        = 4.23, # 1/rad
    alpha_0  = deg2rad(2.50),
    i        = deg2rad(0.00),
    CM0      = 0.04,
    k        = 2.5,
    de_da    = 0.32, # downwash gradient  dε/dα
)


# ══════════════════════════════════════════════════════════════════
#  FUSELAGE EFFECTS 
# ══════════════════════════════════════════════════════════════════
dCM_da_fus = 0.76 # fuselage pitching moment coeff [1/rad]


# ══════════════════════════════════════════════════════════════════
#  DRAG 
# ══════════════════════════════════════════════════════════════════
CD0   = 0.022   # aircraft zero-lift drag coefficient
M_DD  = 0.65    # wave drag divergence Mach number
CD0_C = 0.00685 # canard zero-lift drag coefficient
CD0_H = 0.00951 # tailplane zero-lift drag coefficient


# ══════════════════════════════════════════════════════════════════
#  WING FLAPS — Approach Configuration (δ_fW = 30°)  
# ══════════════════════════════════════════════════════════════════
flap_Sf_S_W    = 0.55  # flapped area ratio  (S_f / S)_W
flap_cprime_c  = 1.25  # flap extension ratio (c'/c)_W
delta_CL0_W    = 0.932 # flap zero-lift increment  ΔC_L0W
delta_CD0_flap = 0.017 # flap zero-lift drag increment  ΔC_D0


# ══════════════════════════════════════════════════════════════════
#  CANARD FLAPS 
# ══════════════════════════════════════════════════════════════════
dCL_C_dflap_C = 1.463 # canard flap lift increment  ΔC_LC / δ_fC  [1/rad]


# ══════════════════════════════════════════════════════════════════
#  ELEVATOR
# ══════════════════════════════════════════════════════════════════
cE_cH       = 0.4           # elevator-to-tailplane chord ratio  c_E / c_H
delta_E_max = deg2rad(20.0) # max elevator deflection  ±20°
a_E         = 3.08          # elevator lift-curve slope  [1/rad]
b_0         = 0.0           # zero-lift hinge moment coeff  [1/rad]
b_H         = -0.502        # tailplane hinge moment slope  [1/rad]
b_E         = -0.879        # elevator hinge moment slope   [1/rad]
b_T         = 0.0           # trim-tab hinge moment slope   [1/rad]


# ══════════════════════════════════════════════════════════════════
#  PROPULSION 
# ══════════════════════════════════════════════════════════════════
z_T = 0.97                     # thrust-line position above baseline [m]

P_max       = 634_000          # W  (each engine)
engines     = 2
P_max_total = P_max * engines  # 1 268 000 W total


# ══════════════════════════════════════════════════════════════════
#  INERTIAL PROPERTIES — mid-cruise 
# ══════════════════════════════════════════════════════════════════
m       = 4836.0               # aircraft mass [kg]
z_CG    = 0.85                 # vertical CoG above baseline [m]
Iyy     = 75_290               # pitching 2nd moment of inertia [kg m²]
I_E     = 0.261                # elevator polar moment of inertia [kg m²]

Ixx     = 26_781               # rolling 2nd moment of inertia [kg m²]
Izz     = 95_725               # yawing 2nd moment of inertia [kg m²]
Ixz     = 3_936                # cross-product 2nd moment of inertia [kg m²]


# ══════════════════════════════════════════════════════════════════
#  OPERATING CONDITIONS — cruise 
# ══════════════════════════════════════════════════════════════════
h_cruise_ft  = 30_000                # cruise altitude [ft]
h_cruise_m   = h_cruise_ft * 0.3048  # ≈ 9 144 m
M_inf        = 0.62                  # cruise Mach number


# ══════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════
g = 9.81 # m/s²  gravitational acceleration


# ══════════════════════════════════════════════════════════════════
#  REFERENCE VALUES 
# ══════════════════════════════════════════════════════════════════
S_ref  = wing.S                # 16.00 m²
c_ref  = wing.c_bar            # 1.18  m
AR_ref = wing.AR               # 12.31