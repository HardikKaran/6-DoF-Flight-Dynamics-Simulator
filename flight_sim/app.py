"""
Flask + Flask-SocketIO Application
====================================
Real-time 2D flight dynamics simulator for the Piaggio P.180 Avanti.

Data flow
---------
    Browser keypress → SocketIO → Flask → physics loop updates controls
    Python physics loop (60 Hz) → SocketIO emit(state) → JS renders frame
"""

from __future__ import annotations

import math
import time
import threading

import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

from physics.eom import (rk4_step, make_initial_state, compute_derivatives,
                          I_U, I_V, I_W, I_P, I_Q, I_R,
                          I_PHI, I_TH, I_PSI, I_XE, I_YE, I_ZE,
                          DT_DEFAULT)
from physics.aero import compute_aero
from physics import aircraft as ac
from physics.autopilot import Autopilot
from physics.stability import stability_report, compute_trim_map

# ══════════════════════════════════════════════════════════════════
#  App & SocketIO
# ══════════════════════════════════════════════════════════════════
app = Flask(__name__)
app.config['SECRET_KEY'] = 'piaggio-p180-sim'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ══════════════════════════════════════════════════════════════════
#  Shared simulation state (protected by lock)
# ══════════════════════════════════════════════════════════════════
sim_lock = threading.Lock()
sim = {
    'state':     None,   # np.ndarray (12,)
    'delta_e':   0.0,
    'delta_a':   0.0,    # aileron [rad]
    'delta_r':   0.0,    # rudder  [rad]
    'throttle':  0.5,
    'i_H':       0.0,    # tail setting angle [rad]
    'flaps':     0.0,    # wing flap fraction 0..1
    'paused':    False,
    'running':   False,
    'trim_de':   0.0,
    'trim_thr':  0.5,
    'fuel':      1.0,    # fuel fraction 0-1 (affects mass)
    'base_mass': 4836.0, # full-fuel mass
    'W_dot_prev': 0.0,   # previous-step Wdot for downwash lag
    'fuel_burn_enabled': True,
}
autopilot = Autopilot()


def _init_sim(V: float = 180.0, h: float = 9144.0):
    """(Re-)initialise from trim."""
    # Apply fuel-dependent mass
    ac.m = sim['base_mass'] * (0.6 + 0.4 * sim['fuel'])  # min 60% mass at empty fuel
    state, de_trim, thr_trim = make_initial_state(V=V, h=h)
    sim['state']     = state.copy()
    sim['delta_e']   = de_trim
    sim['delta_a']   = 0.0
    sim['delta_r']   = 0.0
    sim['throttle']  = thr_trim
    sim['trim_de']   = de_trim
    sim['trim_thr']  = thr_trim
    sim['paused']    = False
    sim['flaps']     = 0.0
    sim['W_dot_prev'] = 0.0
    autopilot.set_mode('off')
    print(f"[sim] Trim -> alpha={math.degrees(math.atan2(state[I_W], state[I_U])):.2f} deg  "
          f"de={math.degrees(de_trim):.2f} deg  thr={thr_trim*100:.1f}%  "
          f"V={math.hypot(state[I_U], state[I_W]):.1f} m/s  "
          f"h={-state[I_ZE]:.0f} m  mass={ac.m:.0f} kg")


def _build_state_dict() -> dict:
    """Pack current sim state into a JSON-safe dict for the browser."""
    s = sim['state']
    U, V, W = s[I_U], s[I_V], s[I_W]
    p, q, r = s[I_P], s[I_Q], s[I_R]
    phi, theta, psi = s[I_PHI], s[I_TH], s[I_PSI]
    xE, yE, zE = s[I_XE], s[I_YE], s[I_ZE]
    altitude = -zE

    # Aero snapshot (for force arrows / readouts)
    fm = compute_aero(U, V, W, p, q, r, phi, theta,
                      sim['delta_e'], sim['delta_a'], sim['delta_r'],
                      sim['throttle'], altitude,
                      flaps=sim['flaps'])

    V_T = fm['V_T']
    alpha = fm['alpha']
    beta  = fm.get('beta', 0.0)
    mach  = fm.get('Mach', V_T / 303.0)

    W_force = ac.m * ac.g  # weight in N

    return {
        'U': float(U),
        'V': float(V),
        'W': float(W),
        'p': float(p),
        'q': float(q),
        'r': float(r),
        'phi':   float(phi),
        'theta': float(theta),
        'psi':   float(psi),
        'xE': float(xE),
        'yE': float(yE),
        'zE': float(zE),
        'altitude': float(altitude),
        'V_T':   float(V_T),
        'alpha': float(alpha),
        'beta':  float(beta),
        'mach':  float(mach),
        'stall': bool(fm.get('stall', False)),
        'fuel':  float(sim['fuel']),
        'mass':  float(ac.m),
        'flaps': float(sim['flaps']),
        'ap_mode': autopilot.mode,
        # Current & trim control values so the browser can sync sliders
        'delta_e':  float(sim['delta_e']),
        'delta_a':  float(sim['delta_a']),
        'delta_r':  float(sim['delta_r']),
        'throttle': float(sim['throttle']),
        'trim_de':  float(sim['trim_de']),
        'trim_thr': float(sim['trim_thr']),
        'forces': {
            'X': float(fm['X']),
            'Y': float(fm['Y']),
            'Z': float(fm['Z']),
            'L_lat': float(fm['L_lat']),
            'M': float(fm['M']),
            'N': float(fm['N']),
            'CL': float(fm['CL']),
            'CD': float(fm['CD']),
            'CY': float(fm.get('CY', 0.0)),
            'T': float(fm['T']),
            'L_aero': float(fm['L_aero']),
            'D_aero': float(fm['D_aero']),
            'Y_aero': float(fm.get('Y_aero', 0.0)),
            'W_force': float(W_force),
        },
    }


# ══════════════════════════════════════════════════════════════════
#  Physics loop (background thread)
# ══════════════════════════════════════════════════════════════════

def physics_loop():
    """Run at ~60 Hz, advance state via RK4, emit to clients."""
    dt = DT_DEFAULT
    print(f"[sim] Physics loop started  (dt={dt*1000:.1f} ms)")
    while sim['running']:
        t0 = time.perf_counter()

        if not sim['paused']:
            with sim_lock:
                # Autopilot corrections
                if autopilot.mode != 'off':
                    s = sim['state']
                    ap_state = {
                        'theta': float(s[I_TH]),
                        'phi': float(s[I_PHI]),
                        'psi': float(s[I_PSI]),
                        'altitude': float(-s[I_ZE]),
                        'q': float(s[I_Q]),
                        'p': float(s[I_P]),
                        'r': float(s[I_R]),
                    }
                    ap_cmds = autopilot.update(ap_state, dt)
                    if 'delta_e' in ap_cmds:
                        sim['delta_e'] = ap_cmds['delta_e']
                    if 'delta_a' in ap_cmds:
                        sim['delta_a'] = ap_cmds['delta_a']

                # RK4 step with flaps and Wdot tracking
                W_before = sim['state'][I_W]
                sim['state'] = rk4_step(sim['state'],
                                        sim['delta_e'],
                                        sim['delta_a'],
                                        sim['delta_r'],
                                        sim['throttle'],
                                        dt,
                                        flaps=sim['flaps'],
                                        W_dot_prev=sim['W_dot_prev'])
                sim['W_dot_prev'] = (sim['state'][I_W] - W_before) / dt

                # Fuel burn (variable mass)
                if sim['fuel_burn_enabled'] and sim['fuel'] > 0.01:
                    from physics.aero import compute_aero as _ca
                    s = sim['state']
                    alt = float(-s[I_ZE])
                    fm = _ca(s[I_U], s[I_V], s[I_W],
                             s[I_P], s[I_Q], s[I_R],
                             s[I_PHI], s[I_TH],
                             sim['delta_e'], sim['delta_a'], sim['delta_r'],
                             sim['throttle'], alt, flaps=sim['flaps'])
                    T = fm['T']
                    dm = ac.TSFC * T * dt
                    m_fuel = sim['fuel'] * ac.m_fuel_max
                    m_fuel = max(0.0, m_fuel - dm)
                    sim['fuel'] = m_fuel / ac.m_fuel_max
                    ac.m = ac.m_empty + m_fuel

            try:
                payload = _build_state_dict()
                socketio.emit('state', payload)
            except Exception as exc:
                print(f"[sim] emit error: {exc}")

        elapsed = time.perf_counter() - t0
        sleep_time = max(0, dt - elapsed)
        time.sleep(sleep_time)

    print("[sim] Physics loop stopped")


# ══════════════════════════════════════════════════════════════════
#  Routes
# ══════════════════════════════════════════════════════════════════

@app.route('/')
def index():
    return render_template('index.html')


# ══════════════════════════════════════════════════════════════════
#  SocketIO events
# ══════════════════════════════════════════════════════════════════

@socketio.on('connect')
def on_connect():
    print("[ws] client connected")
    # Send initial state immediately
    if sim['state'] is not None:
        emit('state', _build_state_dict())


@socketio.on('disconnect')
def on_disconnect():
    print("[ws] client disconnected")


@socketio.on('controls')
def on_controls(data):
    """Receive pilot inputs from the browser."""
    with sim_lock:
        # Only accept manual inputs for axes not under autopilot control
        ap_mode = autopilot.mode
        if 'delta_e' in data and ap_mode not in ('pitch', 'altitude'):
            sim['delta_e'] = float(data['delta_e'])
        if 'delta_a' in data and ap_mode != 'heading':
            sim['delta_a'] = float(data['delta_a'])
        if 'delta_r' in data:
            sim['delta_r'] = float(data['delta_r'])
        if 'throttle' in data:
            sim['throttle'] = float(data['throttle'])
        if 'i_H' in data:
            ac.tail.i = float(data['i_H'])


@socketio.on('reset')
def on_reset(data=None):
    """Re-initialise to trim condition with optional altitude/velocity/fuel."""
    data = data or {}
    h = float(data.get('altitude', 9144))
    v = float(data.get('velocity', 180))
    fuel = float(data.get('fuel', 1.0))
    sim['fuel'] = max(0.1, min(1.0, fuel))
    print(f"[ws] reset requested  V={v} m/s  h={h} m  fuel={sim['fuel']*100:.0f}%")
    with sim_lock:
        _init_sim(V=v, h=h)
    emit('state', _build_state_dict())


@socketio.on('pause')
def on_pause(data):
    sim['paused'] = bool(data.get('paused', False))
    print(f"[ws] paused = {sim['paused']}")


@socketio.on('update_params')
def on_update_params(data):
    """Update aircraft parameters from the browser editor."""
    print(f"[ws] update_params: {data}")
    with sim_lock:
        if 'm' in data:          ac.m = float(data['m']); sim['base_mass'] = ac.m
        if 'x_CG' in data:       ac.x_CG = float(data['x_CG'])
        if 'S_ref' in data:      ac.wing.S = float(data['S_ref']); ac.S_ref = ac.wing.S
        if 'AR_wing' in data:    ac.wing.AR = float(data['AR_wing']); ac.AR_ref = ac.wing.AR
        if 'P_max' in data:      ac.P_max = float(data['P_max']); ac.P_max_total = ac.P_max * ac.engines
        if 'CD0' in data:        ac.CD0 = float(data['CD0'])
        if 'Iyy' in data:        ac.Iyy = float(data['Iyy'])
        if 'c_bar' in data:      ac.wing.c_bar = float(data['c_bar']); ac.c_ref = ac.wing.c_bar
        if 'a_wing' in data:     ac.wing.a = float(data['a_wing'])
        if 'a_canard' in data:   ac.canard.a = float(data['a_canard'])
        if 'a_tail' in data:     ac.tail.a = float(data['a_tail'])
        if 'a_E' in data:        ac.a_E = float(data['a_E'])
        if 'de_da' in data:      ac.tail.de_da = float(data['de_da'])
        if 'deu_da' in data:     ac.canard.de_da = float(data['deu_da'])
        if 'CM0_W' in data:      ac.wing.CM0 = float(data['CM0_W'])
        if 'dCM_da_fus' in data: ac.dCM_da_fus = float(data['dCM_da_fus'])
        # Re-init aero precomputed values
        try:
            from physics.aero import _precompute
            _precompute()
        except Exception:
            pass
    print(f"[ws] params updated - m={ac.m}, Iyy={ac.Iyy}, CD0={ac.CD0}")


@socketio.on('set_flaps')
def on_set_flaps(data):
    """Set wing flap deflection (0..1)."""
    val = max(0.0, min(1.0, float(data.get('flaps', 0.0))))
    sim['flaps'] = val


@socketio.on('set_autopilot')
def on_set_autopilot(data):
    """Set autopilot mode and commands."""
    mode = data.get('mode', 'off')
    kwargs = {}
    if 'theta_cmd' in data:
        kwargs['theta_cmd'] = math.radians(float(data['theta_cmd']))
    if 'h_cmd' in data:
        kwargs['h_cmd'] = float(data['h_cmd'])
    if 'psi_cmd' in data:
        kwargs['psi_cmd'] = math.radians(float(data['psi_cmd']))
    autopilot.set_mode(mode, **kwargs)
    print(f"[ws] autopilot mode={mode}")


@socketio.on('get_stability')
def on_get_stability(data=None):
    """Compute and return stability analysis at current trim."""
    data = data or {}
    V = float(data.get('V', 180.0))
    h = float(data.get('h', 9144.0))
    try:
        report = stability_report(V, h)
        emit('stability_result', report)
    except Exception as exc:
        emit('stability_result', {"error": str(exc)})


@socketio.on('get_trim_map')
def on_get_trim_map(data=None):
    """Compute and return trim map data."""
    try:
        result = compute_trim_map()
        emit('trim_map_result', result)
    except Exception as exc:
        emit('trim_map_result', {"error": str(exc)})


# ══════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    _init_sim()
    sim['running'] = True
    physics_thread = threading.Thread(target=physics_loop, daemon=True)
    physics_thread.start()
    print("[app] Starting server on http://127.0.0.1:5000")
    socketio.run(app, host='127.0.0.1', port=5000, debug=False)
