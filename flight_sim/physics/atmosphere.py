"""
International Standard Atmosphere (ISA) Model
===============================================
Provides density ρ(h), temperature T(h), pressure p(h), and speed of
sound a(h) as functions of geometric altitude h [m].

Valid from sea level to 20 km (troposphere + lower stratosphere).
"""

import math

# ── Sea-level constants ───────────────────────────────────────────
T0   = 288.15      # K     sea-level temperature
p0   = 101_325.0   # Pa    sea-level pressure
rho0 = 1.225       # kg/m³ sea-level density
a0   = 340.294     # m/s   sea-level speed of sound
g0   = 9.80665     # m/s²  gravitational acceleration
R    = 287.0528    # J/(kg·K)  specific gas constant for dry air
gamma_air = 1.4    # ratio of specific heats

# ── Troposphere (0 – 11 000 m) ───────────────────────────────────
LAPSE_RATE = -0.0065   # K/m  temperature lapse rate
H_TROP     = 11_000.0  # m    tropopause altitude

# Temperature at tropopause
T_TROP = T0 + LAPSE_RATE * H_TROP   # ≈ 216.65 K
# Pressure at tropopause
p_TROP = p0 * (T_TROP / T0) ** (-g0 / (LAPSE_RATE * R))
# Density at tropopause
rho_TROP = p_TROP / (R * T_TROP)


def temperature(h: float) -> float:
    """ISA temperature [K] at altitude h [m]."""
    if h <= H_TROP:
        return T0 + LAPSE_RATE * h
    else:
        # Stratosphere (isothermal up to ~20 km)
        return T_TROP


def pressure(h: float) -> float:
    """ISA pressure [Pa] at altitude h [m]."""
    if h <= H_TROP:
        T = T0 + LAPSE_RATE * h
        return p0 * (T / T0) ** (-g0 / (LAPSE_RATE * R))
    else:
        # Stratosphere: exponential decay at constant temperature
        return p_TROP * math.exp(-g0 * (h - H_TROP) / (R * T_TROP))


def density(h: float) -> float:
    """ISA density ρ [kg/m³] at altitude h [m]."""
    T = temperature(h)
    p = pressure(h)
    return p / (R * T)


def speed_of_sound(h: float) -> float:
    """Speed of sound a [m/s] at altitude h [m]."""
    T = temperature(h)
    return math.sqrt(gamma_air * R * T)
