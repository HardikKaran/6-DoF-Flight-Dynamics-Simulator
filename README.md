# 6-DOF Flight Dynamics Simulator — Piaggio P.180 Avanti

A real-time, browser-based **full 6-degrees-of-freedom** flight dynamics simulator for the Piaggio P.180 Avanti three-surface turboprop aircraft. Built with Python (Flask + SocketIO) for the physics engine and HTML5 Canvas for rendering.

## Features

### Physics Engine
- **Full 6-DOF EOM** — 12-state vector `[U, V, W, p, q, r, Phi, Theta, Psi, xE, yE, zE]` with RK4 integration at 60 Hz
- **Three-surface aerodynamics** — canard, wing, and horizontal tail with individual lift/drag/moment contributions
- **ISA atmosphere** — troposphere + stratosphere model for density, temperature, speed of sound
- **Stall model** — Viterna & Corrigan post-stall CL drop-off with smooth blend
- **Compressibility** — Prandtl-Glauert correction + Lock-type wave drag above M_DD
- **Ixz coupling** — full coupled roll/yaw dynamics via the 2x2 inertia system
- **Fuel burn** — TSFC-based variable mass with real-time mass tracking
- **Flap effects** — wing flap CL/CD increments for approach configuration
- **Downwash lag (Wdot) derivatives** — CZ_alpha_dot and CM_alpha_dot (Nelson Ch. 8)
- **Trim solver** — `fsolve`-based Newton method for level-flight trim (alpha, delta_e, throttle)

### Stability Analysis
- **Linearisation** — numerical A (12x12) and B (12x4) matrices via central finite differences
- **Eigenvalue analysis** — identifies Short Period, Phugoid, Dutch Roll, Roll Subsidence, Spiral modes
- **Interactive s-plane plot** — eigenvalue visualisation in the browser
- **Trim map** — sweep of trim conditions over speed and altitude ranges
- **Hinge moment** — elevator hinge moment and stick force computation (Nelson Section 7.4)

### Autopilot
- **Pitch hold** — PID theta-command to elevator
- **Altitude hold** — cascaded altitude -> pitch -> elevator PID loops
- **Heading hold** — cascaded heading -> bank -> aileron PID loops
- Anti-windup and output clamping on all controllers

### Visualisation
- **2D Canvas renderer** — wireframe P.180 with sky/ground/clouds
- **Force arrows** — lift, drag, weight, thrust vectors on the aircraft
- **Flow arrows** — freestream velocity field with parallax
- **Attitude indicator** — artificial horizon with pitch ladder and bank
- **HUD overlay** — real-time flight data readouts
- **Stall warning** — flashing red banner + canvas border

### Controls
| Key | Action |
|-----|--------|
| Arrow Up/Down | Elevator |
| W / S | Throttle |
| A / D | Aileron |
| Q / E | Rudder |
| F | Toggle flaps (0 / 50 / 100%) |
| R | Reset to trim |
| P | Pause / Resume |

## Architecture

```
Browser (JS)                          Server (Python)
=============                         ===============
sim.js          <-- SocketIO -->      app.py
  controls(30Hz)  ----------->          physics_loop(60Hz)
  <-----------  state(60Hz)             eom.py (RK4)
renderer.js                              aero.py (forces/moments)
  Canvas 2D                              aircraft.py (P.180 params)
                                         atmosphere.py (ISA)
                                         stability.py (linearisation)
                                         autopilot.py (PID)
```

## Setup

```bash
# Clone
git clone https://github.com/<user>/6-DoF-Flight-Dynamics-Simulator.git
cd 6-DoF-Flight-Dynamics-Simulator

# Install dependencies
pip install -r requirements.txt

# Run
python flight_sim/app.py
```

Open http://127.0.0.1:5000 in your browser.

## Requirements

- Python 3.9+
- NumPy, SciPy, Flask, Flask-SocketIO

## Physics Validation

81 comprehensive tests validate the physics engine against known theoretical results:

```bash
python tests/test_comprehensive.py
```

Test categories:
1. ISA atmosphere (sea level, tropopause benchmarks)
2. Force/moment balance at trim (L=W, T=D, M=0)
3. Stall model (post-stall CL drop-off, CL_max cap)
4. Compressibility (Prandtl-Glauert, wave drag)
5. Eigenvalue stability (all longitudinal modes stable)
6. Mode identification (Short Period, Phugoid, Dutch Roll, Roll Subsidence, Spiral)
7. Phugoid period vs theoretical $T = \pi\sqrt{2}\,V/g$
8. Trim map convergence across speed/altitude envelope
9. Fuel burn rate validation
10. Autopilot PID convergence
11. RK4 perturbation stability (decay from trim)
12. Euler kinematics (DCM rotation)
13. Ground collision clamp
14. Flap CL/CD increments
15. Hinge moment computation
16. Multi-speed trim (alpha decreases with speed)
17. Lateral response to aileron
18. Energy conservation (no free energy creation)

## References

- Nelson, R.C., *Flight Stability and Automatic Control*, 2nd Ed., McGraw-Hill, 1998
- Cook, M.V., *Flight Dynamics Principles*, 3rd Ed., 2013
- Etkin, B. & Reid, L.D., *Dynamics of Flight*, 3rd Ed., 1996
- Stevens, B.L. & Lewis, F.L., *Aircraft Control and Simulation*, 3rd Ed., 2016
- AERO50002 Lecture Notes, Chapters 4 & 8
