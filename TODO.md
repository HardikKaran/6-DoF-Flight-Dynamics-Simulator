# TODO — 2D Flight Dynamics Simulator (Piaggio P.180 Avanti)

## Legend
- [ ] Not started
- [~] In progress
- [x] Completed

---

## Phase 0 — Project Setup
- [x] Create `flight_sim/` directory structure
- [x] Initialise `aircraft.py` with all Piaggio P.180 Avanti parameters (Tables 1 & 2)
- [x] Scaffold `eom.py`, `aero.py`, `app.py`, `sim.js`, `renderer.js`, `index.html`
- [x] Deprecate old `flight_dynamics/physics_engine.py` (Cessna 172 params)
- [x] Add `requirements.txt` (flask, flask-socketio, numpy, eventlet/gevent)
- [x] Estimate or source missing inertia values (Ixx, Izz, Ixz) for the Avanti

---

## Phase 1 — Longitudinal-Only Physics (6 states: U, W, q, Θ, xE, zE)
> Goal: a working pitch / climb / dive sim with elevator + throttle.

- [x] **1.1 ISA atmosphere model** — `ρ(h)`, `T(h)`, `a(h)` → `atmosphere.py`
- [x] **1.2 Aerodynamic model (aero.py)** — longitudinal only
  - [x] Dynamic pressure `q̄ = ½ρV²`
  - [x] Wing lift `CL_W = a_W · (α − α_0W + i_W)` with zero-lift AoA & setting angle
  - [x] Canard lift `CL_C` (with upwash correction `dε_U/dα`)
  - [x] Tail lift `CL_H` (with downwash correction `dε/dα`)
  - [x] Total aircraft CL (wing + canard + tail, area-weighted)
  - [x] Drag: `CD = CD0 + k_W·CL_W²/(π·AR_W) + CD0_C + k_C·CL_C²/(π·AR_C) + CD0_H + k_H·CL_H²/(π·AR_H)`
  - [x] Pitching moment about CG: wing `CM0_W` + canard contribution + tail contribution + fuselage `(dCM/dα)_F · α`
  - [x] Elevator effectiveness: `a_E · δe` acting on tail lift → moment arm from tail AC to CG
  - [x] Pitch damping term: `CM_q · (q · c̄ / 2V)`
  - [x] Rotate wind-axis forces → body axes: `Xa = L·sin(α) − D·cos(α)`, `Za = −L·cos(α) − D·sin(α)`
- [x] **1.3 Thrust model**
  - [x] Turboprop: `T = η_prop · P / V` (with prop efficiency η_prop ≈ 0.85)
  - [x] Throttle maps to fraction of P_max_total
  - [x] Thrust line offset `z_T` → contributes to pitching moment
- [x] **1.4 Gravity in body axes** — `Xg = −mg·sin(Θ)`, `Zg = mg·cos(Θ)`
- [x] **1.5 Equations of motion (eom.py)** — 6-state longitudinal
  - [x] `U̇ = (X/m) − W·q`  (V=0, r=0 simplification)
  - [x] `Ẇ = (Z/m) + U·q`  (V=0, p=0 simplification)
  - [x] `q̇ = M / Iyy`       (Ixz=0 or negligible for longitudinal)
  - [x] `Θ̇ = q`
  - [x] `ẋE = U·cos(Θ) + W·sin(Θ)`
  - [x] `żE = −U·sin(Θ) + W·cos(Θ)`
- [x] **1.6 RK4 integrator** — `rk4_step(state, controls, dt)` with `dt ≈ 1/60 s`
- [x] **1.7 Trim solver** — find `(α_trim, δe_trim, T_trim)` so all derivatives ≈ 0
  - [x] Verify: aircraft holds level flight with zero control input at trim
- [x] **1.8 Ground collision clamp** — if `altitude ≤ 0`, freeze vertical motion

---

## Phase 2 — Rendering (Canvas 2D)
> Goal: see the aircraft moving on screen before the physics are perfect.

- [x] **2.1 Canvas setup** — sizing, DPI scaling, `requestAnimationFrame` loop
- [x] **2.2 Sky background** — gradient (blue → light blue), darkens with altitude
- [x] **2.3 Ground / horizon line** — scrolls vertically with altitude
- [x] **2.4 Cloud layers** — parallax scrolling decorations
- [x] **2.5 Aircraft drawing**
  - [x] Option A: wireframe Piaggio shape (canard, swept mid-wing, T-tail, pusher nacelles)
  - [ ] Option B: load aircraft sprite/image (as shown in screenshot)
  - [x] Rotate by pitch angle Θ; aircraft centred on screen
- [x] **2.6 Force arrows** — draw vectors for lift, drag, weight, thrust on the aircraft
- [x] **2.7 Flow arrows** — freestream velocity field (green arrows, left side of screen)
- [x] **2.8 Attitude indicator** — artificial horizon dial (bottom-centre)
- [x] **2.9 HUD text** — speed (m/s), altitude (m), α (deg), Θ (deg), q (rad/s)
- [x] **2.10 Force readout panel** — ΣX, ΣZ, ΣM values (bottom)

---

## Phase 3 — Flask + SocketIO Integration
> Goal: connect Python physics ↔ browser rendering in real-time.

- [x] **3.1 Flask route** — serve `index.html` at `/`
- [x] **3.2 Static file serving** — `sim.js`, `renderer.js`
- [x] **3.3 SocketIO events**
  - [x] `connect` → initialise state to trim, start physics thread
  - [x] `controls` → receive `{de, throttle}` from browser
  - [x] `state` → emit `{U, W, q, theta, xE, zE, forces, …}` to browser
  - [x] `reset` → re-initialise to trim
- [x] **3.4 Physics background thread** — 60 Hz loop using `socketio.start_background_task`
- [x] **3.5 sim.js client** — connect, send controls, receive state, call renderer

---

## Phase 4 — UI Controls & Interactivity
> Goal: match the control panel shown in the design screenshots.

- [x] **4.1 Right panel — States section** (read-only)
  - [x] Pitch Rate display + slider indicator
  - [x] Initial Γ + Change readout
  - [x] Velocity (m/s) slider + value
  - [x] Angle of Attack (α) slider + value (deg)
- [x] **4.2 Right panel — Pilot Inputs section** (interactive)
  - [x] Elevator slider (±20°)
  - [x] Tail Setting Angle slider
  - [x] Throttle (%) slider (0–100%)
- [x] **4.3 Right panel — Parameters section**
  - [x] Altitude slider / readout
  - [x] Fuel (%) slider → adjusts aircraft mass dynamically
- [x] **4.4 Mode buttons**
  - [x] "Static Flow / Moving Body" — aircraft moves, flow arrows fixed
  - [x] "Static Body / Moving Flow" — aircraft centred, flow arrows scroll
- [x] **4.5 Bottom bar — Axis Settings**
  - [x] Flight Path / Aircraft Datum toggle
  - [x] X-Earth / Z-Earth axis labels + display
  - [x] Body Velocity vector indicator
- [x] **4.6 Bottom bar — Force display**
  - [x] ΣX, ΣZ, ΣM numerical readouts
  - [x] Earth Axis / Body Axis dropdown to change reference frame
- [x] **4.7 Keyboard controls** — Arrow Up/Down (elevator), W/S (throttle), R (reset), P (pause)
- [x] **4.8 Aircraft parameter editor overlay** (as in screenshot 3)
  - [x] Editable fields: AR, Pmax, ebar, sref, weights, xcg, aero coefficients…
  - [x] "Load" button to apply & "Exit" to dismiss

---

## Phase 5 — Full 6-DOF (12 states)
> Goal: add lateral/directional dynamics (bank, yaw, sideslip).

- [ ] **5.1 Extend state vector** — add V, p, r, Φ, Ψ, yE
- [ ] **5.2 Lateral aero derivatives** — Yv, Lv, Nv, Lp, Np, Lr, Nr, Yδa, Lδa, Nδa, Yδr, Lδr, Nδr
- [ ] **5.3 Full Euler kinematic equations** — Φ̇, Θ̇, Ψ̇ with all trig terms
- [ ] **5.4 Full navigation equations** — 3D body→Earth rotation matrix
- [ ] **5.5 Aileron + rudder controls** — A/D keys, sliders
- [ ] **5.6 Ixz coupling** — solve 2×2 system for ṗ, ṙ using Γ = Ixx·Izz − Ixz²
- [ ] **5.7 Estimate Ixx, Izz, Ixz** — use geometric/statistical methods or literature

---

## Phase 6 — Atmosphere & Refinements
> Goal: realism improvements.

- [ ] **6.1 ISA atmosphere** — ρ, T, a as functions of altitude (troposphere + stratosphere)
- [ ] **6.2 Stall model** — cap CL above α_stall ≈ 15°, post-stall CL drop-off
- [ ] **6.3 Compressibility corrections** — Prandtl-Glauert below M_DD, wave drag above
- [ ] **6.4 Flap effects** — ΔCL0_W, ΔCD0, canard flap ΔCL_C/δ_fC for approach config
- [ ] **6.5 Gimbal-lock protection** — clamp Θ at ±85° or implement quaternion attitude
- [ ] **6.6 Variable mass** — fuel burn reduces m and shifts CG over time
- [ ] **6.7 Ẇ-derivatives** — XẆ, ZẆ, MẆ (downwash lag, Ch. 8)

---

## Phase 7 — Stability Analysis & Autopilot (stretch)
> Goal: educational tools for flight dynamics course.

- [ ] **7.1 Linearisation** — compute stability matrices A, B at trim
- [ ] **7.2 Eigenvalue display** — show short-period & phugoid modes
- [ ] **7.3 PID pitch autopilot** — hold altitude or pitch angle (Ch. 12)
- [ ] **7.4 Trim map** — show how trim α, δe, T vary with speed and altitude
- [ ] **7.5 Elevator hinge moment** — compute stick force using b_0, b_H, b_E, b_T

---

## Phase 8 — Polish & Deployment
- [ ] **8.1 Responsive layout** — handle window resize
- [ ] **8.2 Loading screen** — show while SocketIO connects
- [ ] **8.3 Unit tests** — verify EOM derivatives against hand-calc at trim
- [ ] **8.4 README** — setup instructions, screenshots, architecture docs
- [ ] **8.5 Docker / deployment** — optional containerised deployment
