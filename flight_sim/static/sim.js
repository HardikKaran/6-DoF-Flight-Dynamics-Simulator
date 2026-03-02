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
};

/* keyboard rate (rad per held-frame for δe, fraction for throttle) */
const DE_RATE = 0.005;    // ~0.29 deg/frame → ~17 deg/s at 60fps
const TH_RATE = 0.008;
const DE_MAX  =  0.35;    // ~20°
const DE_MIN  = -0.35;

/* key state map */
const keys = {};
window.addEventListener('keydown', e => { keys[e.key] = true; });
window.addEventListener('keyup',   e => { keys[e.key] = false; });

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

  /* sync slider values to match keyboard changes */
  const sliderDe = document.getElementById('slider-de');
  const sliderTh = document.getElementById('slider-throttle');
  if (sliderDe) sliderDe.value = (controls.delta_e * 180 / Math.PI).toFixed(1);
  if (sliderTh) sliderTh.value = (controls.throttle * 100).toFixed(0);
}

/* ================================================================
   UI BINDINGS  (sliders → controls)
   ================================================================ */
function bindSliders() {
  const sliderDe = document.getElementById('slider-de');
  const sliderTh = document.getElementById('slider-throttle');
  if (sliderDe) {
    sliderDe.addEventListener('input', () => {
      controls.delta_e = parseFloat(sliderDe.value) * Math.PI / 180;
    });
  }
  if (sliderTh) {
    sliderTh.addEventListener('input', () => {
      controls.throttle = parseFloat(sliderTh.value) / 100;
    });
  }
}

/* ================================================================
   HUD — update the right-panel readouts
   ================================================================ */
function updatePanel(state) {
  const set = (id, val) => {
    const el = document.getElementById(id);
    if (el) el.textContent = val;
  };
  const alt = state.altitude != null ? state.altitude : -(state.zE || 0);
  set('val-speed',    (state.V_T || 0).toFixed(1) + ' m/s');
  set('val-altitude', alt.toFixed(0) + ' m');
  set('val-alpha',    ((state.alpha || 0) * 180 / Math.PI).toFixed(2) + '°');
  set('val-theta',    ((state.theta || 0) * 180 / Math.PI).toFixed(2) + '°');
  set('val-q',        (state.q || 0).toFixed(4) + ' rad/s');
  set('val-mach',     (state.mach || 0).toFixed(3));

  /* control readouts */
  set('val-de',       (controls.delta_e * 180 / Math.PI).toFixed(1) + '°');
  set('val-throttle', (controls.throttle * 100).toFixed(0) + '%');

  /* force readouts */
  if (state.forces) {
    set('val-fx', state.forces.X.toFixed(0) + ' N');
    set('val-fz', state.forces.Z.toFixed(0) + ' N');
    set('val-fm', state.forces.M.toFixed(0) + ' N·m');
    set('val-lift', state.forces.L_aero.toFixed(0) + ' N');
    set('val-drag', state.forces.D_aero.toFixed(0) + ' N');
    set('val-thrust', state.forces.T.toFixed(0) + ' N');
  }
}

/* ================================================================
   SOCKET.IO CONNECTION
   ================================================================ */
let socket = null;
let lastState = null;

function connectSocket() {
  socket = io();

  socket.on('connect', () => {
    console.log('[sim] connected to server');
    document.getElementById('conn-status').textContent = 'Connected';
    document.getElementById('conn-status').style.color = '#4af7b0';
  });

  socket.on('disconnect', () => {
    console.log('[sim] disconnected');
    document.getElementById('conn-status').textContent = 'Disconnected';
    document.getElementById('conn-status').style.color = '#ff4444';
  });

  socket.on('state', (state) => {
    lastState = state;
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
      });
    }
  }, 1000 / 30);
}

/* ================================================================
   RESET
   ================================================================ */
function resetSim() {
  controls.delta_e  = 0;
  controls.throttle = 0.5;
  if (socket && socket.connected) {
    socket.emit('reset');
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
   INIT
   ================================================================ */
window.addEventListener('DOMContentLoaded', () => {
  const cvs = document.getElementById('sim-canvas');
  if (!cvs) { console.error('Canvas #sim-canvas not found'); return; }

  initCanvas(cvs);
  bindSliders();
  connectSocket();
  startControlLoop();

  /* reset button */
  const btnReset = document.getElementById('btn-reset');
  if (btnReset) btnReset.addEventListener('click', resetSim);

  /* pause button */
  const btnPause = document.getElementById('btn-pause');
  if (btnPause) btnPause.addEventListener('click', togglePause);
});