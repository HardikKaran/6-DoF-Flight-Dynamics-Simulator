"""
PID Autopilot
===============
Simple PID controllers for pitch-hold, altitude-hold, and heading-hold.

References
----------
  Nelson, R.C., "Flight Stability and Automatic Control", 2nd Ed.,
    McGraw-Hill, 1998 — Ch. 12.
  Stevens, B.L. & Lewis, F.L., "Aircraft Control and Simulation",
    3rd Ed., Wiley, 2016 — Ch. 7.
"""

from __future__ import annotations
import math


class PIDController:
    """Generic PID controller with anti-windup and output clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float = -1.0, out_max: float = 1.0,
                 imax: float = 10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.imax = imax
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, error: float, dt: float) -> float:
        """Compute PID output given current error and timestep."""
        if dt <= 0:
            return 0.0
        # Proportional
        P = self.kp * error
        # Integral with anti-windup
        self._integral += error * dt
        self._integral = max(-self.imax, min(self.imax, self._integral))
        I = self.ki * self._integral
        # Derivative (skip on first call)
        if self._first:
            D = 0.0
            self._first = False
        else:
            D = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        # Total with clamping
        output = P + I + D
        return max(self.out_min, min(self.out_max, output))


class Autopilot:
    """
    Flight autopilot with selectable modes.

    Modes:
        'off'      — manual control
        'pitch'    — hold pitch angle (theta_cmd in rad)
        'altitude' — hold altitude (h_cmd in metres) via cascaded alt→pitch→elevator
        'heading'  — hold heading (psi_cmd in rad) via bank-angle command
    """

    def __init__(self):
        # Pitch hold: error = theta_cmd - theta → delta_e
        self.pitch_pid = PIDController(
            kp=-2.0, ki=-0.5, kd=-0.8,
            out_min=-0.35, out_max=0.35  # ~±20 deg elevator
        )
        # Altitude hold (outer loop): error = h_cmd - h → theta_cmd
        # Deliberately PI-only (no D): derivative of altitude error is
        # climb-rate, which the inner pitch loop already controls.
        # Max theta command ±5 deg for smooth, realistic response.
        self.alt_pid = PIDController(
            kp=0.001, ki=0.0001, kd=0.0,
            out_min=-0.087, out_max=0.087,  # ~±5 deg theta command
            imax=100.0
        )
        # Heading hold: error = psi_cmd - psi → phi_cmd
        self.heading_pid = PIDController(
            kp=1.5, ki=0.1, kd=0.3,
            out_min=-0.52, out_max=0.52  # ~±30 deg bank
        )
        # Bank-angle hold (inner loop for heading): error = phi_cmd - phi → delta_a
        self.bank_pid = PIDController(
            kp=-1.5, ki=-0.2, kd=-0.5,
            out_min=-0.35, out_max=0.35  # ~±20 deg aileron
        )

        self.mode = 'off'
        self.theta_cmd = 0.0    # pitch command [rad]
        self.h_cmd = 9144.0     # altitude command [m]
        self.psi_cmd = 0.0      # heading command [rad]

    def reset(self):
        """Reset all PID integrators."""
        self.pitch_pid.reset()
        self.alt_pid.reset()
        self.heading_pid.reset()
        self.bank_pid.reset()

    def set_mode(self, mode: str, **kwargs):
        """Set autopilot mode and command values."""
        if mode != self.mode:
            self.reset()          # only reset PIDs when mode actually changes
        self.mode = mode
        if 'theta_cmd' in kwargs:
            self.theta_cmd = float(kwargs['theta_cmd'])
        if 'h_cmd' in kwargs:
            self.h_cmd = float(kwargs['h_cmd'])
        if 'psi_cmd' in kwargs:
            self.psi_cmd = float(kwargs['psi_cmd'])

    def update(self, state_dict: dict, dt: float) -> dict:
        """
        Compute autopilot commands.

        Parameters
        ----------
        state_dict : dict with keys theta, phi, psi, altitude, q, p, r
        dt         : timestep [s]

        Returns
        -------
        dict with optional keys: delta_e, delta_a, delta_r
        (only present for active axes)
        """
        if self.mode == 'off':
            return {}

        cmds = {}
        theta = state_dict.get('theta', 0.0)
        phi = state_dict.get('phi', 0.0)
        psi = state_dict.get('psi', 0.0)
        altitude = state_dict.get('altitude', 0.0)

        if self.mode == 'pitch':
            err = self.theta_cmd - theta
            cmds['delta_e'] = self.pitch_pid.update(err, dt)

        elif self.mode == 'altitude':
            # Outer loop: altitude → pitch command
            alt_err = self.h_cmd - altitude
            theta_cmd = self.alt_pid.update(alt_err, dt)
            # Inner loop: pitch hold
            pitch_err = theta_cmd - theta
            cmds['delta_e'] = self.pitch_pid.update(pitch_err, dt)

        elif self.mode == 'heading':
            # Heading error with wrap-around
            h_err = _wrap_angle(self.psi_cmd - psi)
            # Outer: heading → bank command
            phi_cmd = self.heading_pid.update(h_err, dt)
            # Inner: bank hold
            bank_err = phi_cmd - phi
            cmds['delta_a'] = self.bank_pid.update(bank_err, dt)

        return cmds


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
