/**
 * sim.js — Game Loop, SocketIO Client & Input Handling
 * =====================================================
 * Connects to the Flask-SocketIO backend, sends pilot control inputs,
 * receives the physics state at ~60 Hz, and drives the renderer.
 */

/* ================================================================
   CONTROL STATE
   ================================================================ */
const controls = {
  delta_e:  0.0,   // elevator deflection, rad (positive = nose-up)
  throttle: 0.5,   // 0-1
  delta_a:  0.0,   // aileron (future)
  delta_r:  0.0,   // rudder  (future)
  i_H:      0.0,   // tail setting angle, rad
};

/* keyboard rate (rad per held-frame for δe, fraction for throttle) */
const DE_RATE = 0.005;    // ~0.29 deg/frame → ~17 deg/s at 60fps
const TH_RATE = 0.008;
const DE_MAX  =  0.35;    // ~20°
const DE_MIN  = -0.35;

/* key state map */
const keys = {};
window.addEventListener('keydown', e => {
  keys[e.key] = true;
  /* single-press shortcuts */
  if (e.key === 'r' || e.key === 'R') resetSim();
  if (e.key === 'p' || e.key === 'P') togglePause();
});
window.addEventListener('keyup', e => { keys[e.key] = false; });

/* ================================================================
   VIEW / DISPLAY OPTIONS  (shared with renderer)
   ================================================================ */
const viewOpts = {
  mode:        'flow',     // 'flow' = static body / moving flow, 'body' = static flow / moving body
  showForces:  true,
  showFlow:    true,
  showVelVec:  true,
  showHud:     true,
  datum:       'aircraft', // 'aircraft' or 'flightpath'
  forceFrame:  'body',     // 'body' or 'earth'
};

/* ================================================================
   CONTROL READ (keyboard + sliders)
   ================================================================ */
function readControls() {
  /* elevator */
  if (keys['ArrowUp'])   controls.delta_e = Math.min(controls.delta_e + DE_RATE, DE_MAX);
  if (keys['ArrowDown']) controls.delta_e = Math.max(controls.delta_e - DE_RATE, DE_MIN);

  /* throttle */
  if (keys['w'] || keys['W']) controls.throttle = Math.min(controls.throttle + TH_RATE, 1);
  if (keys['s'] || keys['S']) controls.throttle = Math.max(controls.throttle - TH_RATE, 0);

  /* sync slider display values with keyboard changes */
  syncSliderDisplay();
}

function syncSliderDisplay() {
  const sliderDe = document.getElementById('slider-de');
  const sliderTh = document.getElementById('slider-throttle');
  if (sliderDe) sliderDe.value = (controls.delta_e * 180 / Math.PI).toFixed(1);
  if (sliderTh) sliderTh.value = (controls.throttle * 100).toFixed(0);

  /* slider value labels */
  const set = (id, v) => { const el = document.getElementById(id); if (el) el.textContent = v; };
  set('sv-de',  (controls.delta_e * 180 / Math.PI).toFixed(1) + '°');
  set('sv-thr', (controls.throttle * 100).toFixed(0) + '%');
  set('sv-ih',  (controls.i_H * 180 / Math.PI).toFixed(1) + '°');
}

/* ================================================================
   UI BINDINGS  (sliders → controls)
   ================================================================ */
function bindSliders() {
  const bind = (id, cb) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('input', cb);
  };
  bind('slider-de', () => {
    controls.delta_e = parseFloat(document.getElementById('slider-de').value) * Math.PI / 180;
    syncSliderDisplay();
  });
  bind('slider-throttle', () => {
    controls.throttle = parseFloat(document.getElementById('slider-throttle').value) / 100;
    syncSliderDisplay();
  });
  bind('slider-ih', () => {
    controls.i_H = parseFloat(document.getElementById('slider-ih').value) * Math.PI / 180;
    syncSliderDisplay();
  });

  /* parameter sliders — display only, applied on reset */
  bind('slider-alt', () => {
    document.getElementById('sv-alt').textContent = document.getElementById('slider-alt').value + ' m';
  });
  bind('slider-vel', () => {
    document.getElementById('sv-vel').textContent = document.getElementById('slider-vel').value + ' m/s';
  });
  bind('slider-fuel', () => {
    document.getElementById('sv-fuel').textContent = document.getElementById('slider-fuel').value + '%';
  });
}

/* ================================================================
   MODE & TOGGLE BUTTONS
   ================================================================ */
function bindButtons() {
  /* view mode */
  const btnFlow = document.getElementById('btn-mode-flow');
  const btnBody = document.getElementById('btn-mode-body');
  if (btnFlow) btnFlow.addEventListener('click', () => {
    viewOpts.mode = 'flow';
    btnFlow.classList.add('active'); btnBody.classList.remove('active');
  });
  if (btnBody) btnBody.addEventListener('click', () => {
    viewOpts.mode = 'body';
    btnBody.classList.add('active'); btnFlow.classList.remove('active');
  });

  /* display toggles */
  const tog = (id, key) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('click', () => {
      viewOpts[key] = !viewOpts[key];
      el.classList.toggle('active', viewOpts[key]);
    });
  };
  tog('btn-toggle-forces', 'showForces');
  tog('btn-toggle-flow',   'showFlow');
  tog('btn-toggle-vel',    'showVelVec');
  tog('btn-toggle-hud',    'showHud');

  /* datum */
  const btnFP = document.getElementById('btn-datum-fp');
  const btnAC = document.getElementById('btn-datum-ac');
  if (btnFP) btnFP.addEventListener('click', () => {
    viewOpts.datum = 'flightpath';
    btnFP.classList.add('active'); btnAC.classList.remove('active');
  });
  if (btnAC) btnAC.addEventListener('click', () => {
    viewOpts.datum = 'aircraft';
    btnAC.classList.add('active'); btnFP.classList.remove('active');
  });

  /* force frame dropdown */
  const ff = document.getElementById('force-frame');
  if (ff) ff.addEventListener('change', () => { viewOpts.forceFrame = ff.value; });

  /* pause / reset */
  const btnReset = document.getElementById('btn-reset');
  if (btnReset) btnReset.addEventListener('click', resetSim);
  const btnPause = document.getElementById('btn-pause');
  if (btnPause) btnPause.addEventListener('click', togglePause);

  /* parameter editor */
  const btnParams = document.getElementById('btn-params');
  if (btnParams) btnParams.addEventListener('click', openParamEditor);
  const peExit = document.getElementById('pe-exit');
  if (peExit) peExit.addEventListener('click', closeParamEditor);
  const peLoad = document.getElementById('pe-load');
  if (peLoad) peLoad.addEventListener('click', loadParams);
}

/* ================================================================
   HUD — update the right-panel & bottom-bar readouts
   ================================================================ */
function updatePanel(state) {
  const set = (id, val) => {
    const el = document.getElementById(id);
    if (el) el.textContent = val;
  };
  const alt = state.altitude != null ? state.altitude : -(state.zE || 0);
  const V_T = state.V_T || 0;
  const alpha = (state.alpha || 0);
  const theta = (state.theta || 0);
  const gamma = theta - alpha;  // flight path angle

  /* right panel — states */
  set('val-speed',    V_T.toFixed(1) + ' m/s');
  set('val-altitude', alt.toFixed(0) + ' m');
  set('val-alpha',    (alpha * 180 / Math.PI).toFixed(2) + '°');
  set('val-theta',    (theta * 180 / Math.PI).toFixed(2) + '°');
  set('val-q',        (state.q || 0).toFixed(4) + ' rad/s');
  set('val-mach',     (state.mach || 0).toFixed(3));
  set('val-gamma',    (gamma * 180 / Math.PI).toFixed(2) + '°');

  /* force readouts - respecting frame selection */
  if (state.forces) {
    if (viewOpts.forceFrame === 'body') {
      set('val-fx', state.forces.X.toFixed(0) + ' N');
      set('val-fz', state.forces.Z.toFixed(0) + ' N');
    } else {
      /* earth-axis: rotate body forces */
      const ct = Math.cos(theta), st = Math.sin(theta);
      const Xe =  state.forces.X * ct + state.forces.Z * st;
      const Ze = -state.forces.X * st + state.forces.Z * ct;
      set('val-fx', Xe.toFixed(0) + ' N');
      set('val-fz', Ze.toFixed(0) + ' N');
    }
    set('val-fm', state.forces.M.toFixed(0) + ' N·m');
    set('val-lift', state.forces.L_aero.toFixed(0) + ' N');
    set('val-drag', state.forces.D_aero.toFixed(0) + ' N');
    set('val-thrust', state.forces.T.toFixed(0) + ' N');

    /* bottom bar */
    set('bar-lift',   state.forces.L_aero.toFixed(0) + ' N');
    set('bar-drag',   state.forces.D_aero.toFixed(0) + ' N');
    set('bar-thrust', state.forces.T.toFixed(0) + ' N');
    set('bar-weight', state.forces.W_force.toFixed(0) + ' N');
  }
  /* bottom bar flight data */
  set('bar-vt',    V_T.toFixed(1));
  set('bar-alt',   alt.toFixed(0));
  set('bar-alpha', (alpha * 180 / Math.PI).toFixed(1) + '°');
  set('bar-theta', (theta * 180 / Math.PI).toFixed(1) + '°');
  set('bar-xe',    (state.xE || 0).toFixed(0) + ' m');
  set('bar-ze',    (state.zE || 0).toFixed(0) + ' m');
}

/* ================================================================
   SOCKET.IO CONNECTION
   ================================================================ */
let socket = null;
let lastState = null;
let trimLoaded = false;   // true once we've synced controls to server trim

function connectSocket() {
  socket = io();

  socket.on('connect', () => {
    console.log('[sim] connected to server');
    trimLoaded = false;  // re-sync on reconnect
    const cs = document.getElementById('conn-status');
    if (cs) { cs.textContent = 'Connected'; cs.style.color = '#4af7b0'; }
  });

  socket.on('disconnect', () => {
    console.log('[sim] disconnected');
    const cs = document.getElementById('conn-status');
    if (cs) { cs.textContent = 'Disconnected'; cs.style.color = '#ff4444'; }
  });

  socket.on('state', (state) => {
    /* On first state (or after reset), adopt the server's trim controls
       so the 30 Hz control loop doesn't override trim values. */
    if (!trimLoaded && state.trim_de != null) {
      controls.delta_e  = state.trim_de;
      controls.throttle = state.trim_thr;
      controls.i_H      = 0.0;
      const sliderDe = document.getElementById('slider-de');
      const sliderTh = document.getElementById('slider-throttle');
      const sliderIh = document.getElementById('slider-ih');
      if (sliderDe) sliderDe.value = (controls.delta_e * 180 / Math.PI).toFixed(1);
      if (sliderTh) sliderTh.value = (controls.throttle * 100).toFixed(0);
      if (sliderIh) sliderIh.value = 0;
      syncSliderDisplay();
      trimLoaded = true;
      console.log(`[sim] trim synced: δe=${(controls.delta_e*180/Math.PI).toFixed(2)}°  thr=${(controls.throttle*100).toFixed(1)}%`);
    }

    lastState = state;
    state._viewOpts = viewOpts;  // pass display options to renderer
    render(state);
    updatePanel(state);
  });
}

/* ================================================================
   CONTROL SEND LOOP (30 Hz)
   ================================================================ */
let controlInterval = null;

function startControlLoop() {
  controlInterval = setInterval(() => {
    readControls();
    if (socket && socket.connected) {
      socket.emit('controls', {
        delta_e:  controls.delta_e,
        throttle: controls.throttle,
        delta_a:  controls.delta_a,
        delta_r:  controls.delta_r,
        i_H:      controls.i_H,
      });
    }
  }, 1000 / 30);
}

/* ================================================================
   RESET  (sends init velocity / altitude / fuel from sliders)
   ================================================================ */
function resetSim() {
  /* Tell the server to re-trim.  The next 'state' event will carry the
     new trim_de / trim_thr values; trimLoaded=false makes the state
     handler adopt them automatically. */
  trimLoaded = false;

  if (socket && socket.connected) {
    const alt = parseFloat(document.getElementById('slider-alt')?.value || 9144);
    const vel = parseFloat(document.getElementById('slider-vel')?.value || 180);
    const fuel = parseFloat(document.getElementById('slider-fuel')?.value || 100);
    socket.emit('reset', { altitude: alt, velocity: vel, fuel: fuel / 100 });
  }
}

/* ================================================================
   PAUSE / RESUME
   ================================================================ */
let paused = false;
function togglePause() {
  paused = !paused;
  if (socket && socket.connected) {
    socket.emit('pause', { paused });
  }
  const btn = document.getElementById('btn-pause');
  if (btn) btn.textContent = paused ? '▶ Resume' : '⏸ Pause';
}

/* ================================================================
   PARAMETER EDITOR
   ================================================================ */
function openParamEditor() {
  document.getElementById('param-overlay').classList.add('open');
}

function closeParamEditor() {
  document.getElementById('param-overlay').classList.remove('open');
}

function loadParams() {
  const pf = (id) => parseFloat(document.getElementById(id)?.value || 0);
  const params = {
    m:          pf('pe-m'),
    x_CG:      pf('pe-xcg'),
    S_ref:     pf('pe-sref'),
    AR_wing:   pf('pe-ar'),
    P_max:     pf('pe-pmax'),
    CD0:       pf('pe-cd0'),
    Iyy:       pf('pe-iyy'),
    c_bar:     pf('pe-cbar'),
    a_wing:    pf('pe-aw'),
    a_canard:  pf('pe-ac'),
    a_tail:    pf('pe-ah'),
    a_E:       pf('pe-ae'),
    de_da:     pf('pe-deda'),
    deu_da:    pf('pe-deuda'),
    CM0_W:     pf('pe-cm0w'),
    dCM_da_fus:pf('pe-cmfus'),
  };
  if (socket && socket.connected) {
    socket.emit('update_params', params);
  }
  closeParamEditor();
}

/* ================================================================
   INIT
   ================================================================ */
window.addEventListener('DOMContentLoaded', () => {
  const cvs = document.getElementById('sim-canvas');
  if (!cvs) { console.error('Canvas #sim-canvas not found'); return; }

  initCanvas(cvs);
  bindSliders();
  bindButtons();
  connectSocket();
  startControlLoop();
});