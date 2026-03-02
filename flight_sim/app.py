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
                          I_U, I_W, I_Q, I_TH, I_XE, I_ZE, DT_DEFAULT)
from physics.aero import compute_aero
from physics import aircraft as ac

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
    'state':     None,   # np.ndarray (6,)
    'delta_e':   0.0,
    'throttle':  0.5,
    'paused':    False,
    'running':   False,
    'trim_de':   0.0,
    'trim_thr':  0.5,
}


def _init_sim():
    """(Re-)initialise from trim."""
    state, de_trim, thr_trim = make_initial_state()
    sim['state']     = state.copy()
    sim['delta_e']   = de_trim
    sim['throttle']  = thr_trim
    sim['trim_de']   = de_trim
    sim['trim_thr']  = thr_trim
    sim['paused']    = False
    print(f"[sim] Trim → α={math.degrees(math.atan2(state[I_W], state[I_U])):.2f}°  "
          f"δe={math.degrees(de_trim):.2f}°  thr={thr_trim*100:.1f}%  "
          f"V={math.hypot(state[I_U], state[I_W]):.1f} m/s  "
          f"h={-state[I_ZE]:.0f} m")


def _build_state_dict() -> dict:
    """Pack current sim state into a JSON-safe dict for the browser."""
    s = sim['state']
    U, W, q, theta = s[I_U], s[I_W], s[I_Q], s[I_TH]
    xE, zE = s[I_XE], s[I_ZE]
    altitude = -zE

    # Aero snapshot (for force arrows / readouts)
    fm = compute_aero(U, W, q, theta, sim['delta_e'], sim['throttle'], altitude)

    V_T = fm['V_T']
    alpha = fm['alpha']
    mach = V_T / max(1.0, fm.get('a', 303.0))  # approximate
    # Compute mach from atmosphere if available
    try:
        from physics.atmosphere import speed_of_sound
        mach = V_T / speed_of_sound(altitude)
    except Exception:
        pass

    W_force = ac.m * ac.g  # weight in N

    return {
        'U': float(U),
        'W': float(W),
        'q': float(q),
        'theta': float(theta),
        'xE': float(xE),
        'zE': float(zE),
        'altitude': float(altitude),
        'V_T': float(V_T),
        'alpha': float(alpha),
        'mach': float(mach),
        'forces': {
            'X': float(fm['X']),
            'Z': float(fm['Z']),
            'M': float(fm['M']),
            'CL': float(fm['CL']),
            'CD': float(fm['CD']),
            'T': float(fm['T']),
            'L_aero': float(fm['L_aero']),
            'D_aero': float(fm['D_aero']),
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
                sim['state'] = rk4_step(sim['state'],
                                        sim['delta_e'],
                                        sim['throttle'],
                                        dt)
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
        if 'delta_e' in data:
            sim['delta_e'] = float(data['delta_e'])
        if 'throttle' in data:
            sim['throttle'] = float(data['throttle'])


@socketio.on('reset')
def on_reset():
    """Re-initialise to trim condition."""
    print("[ws] reset requested")
    with sim_lock:
        _init_sim()
    emit('state', _build_state_dict())


@socketio.on('pause')
def on_pause(data):
    sim['paused'] = bool(data.get('paused', False))
    print(f"[ws] paused = {sim['paused']}")


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
