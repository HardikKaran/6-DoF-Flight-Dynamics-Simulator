"""
Flask + Flask-SocketIO Application
====================================
Real-time 2D flight dynamics simulator for the Piaggio P.180 Avanti.

Data flow
---------
    Browser keypress → SocketIO → Flask → physics loop updates controls
    Python physics loop (60 Hz) → SocketIO emit(state) → JS renders frame

TODO: Implement the following
------------------------------
1. Flask app factory / route for serving index.html
2. SocketIO event handlers:
   a. 'connect'      — initialise state, start physics loop
   b. 'controls'     — receive {de, da, dr, throttle} from browser
   c. 'reset'        — re-initialise state to trim condition
3. Background thread running the physics loop at ~60 Hz
   a. Read latest controls
   b. Call rk4_step()
   c. Emit state dict to all connected clients
4. Serve static files (sim.js, renderer.js)
"""

from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app)
