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
  delta_a:  0.0,   // aileron deflection, rad (positive = roll right)
  delta_r:  0.0,   // rudder deflection, rad  (positive = TE-to-port → yaw left)
  i_H:      0.0,   // tail setting angle, rad
};

/* keyboard rate (rad per held-frame for δe/δa/δr, fraction for throttle) */
const DE_RATE = 0.005;    // ~0.29 deg/frame → ~17 deg/s at 60fps
const DA_RATE = 0.005;    // aileron rate
const DR_RATE = 0.005;    // rudder rate
const TH_RATE = 0.008;
const DE_MAX  =  0.35;    // ~20°
const DE_MIN  = -0.35;
const DA_MAX  =  0.35;    // ~20° aileron
const DA_MIN  = -0.35;
const DR_MAX  =  0.44;    // ~25° rudder
const DR_MIN  = -0.44;

/* key state map */
const keys = {};
window.addEventListener('keydown', e => {
  keys[e.key] = true;
  /* single-press shortcuts */
  if (e.key === 'r' || e.key === 'R') resetSim();
  if (e.key === 'p' || e.key === 'P') togglePause();
  if (e.key === 'f' || e.key === 'F') toggleFlaps();
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

  /* aileron  (A = roll left → δa < 0,  D = roll right → δa > 0) */
  if (keys['a'] || keys['A']) controls.delta_a = Math.max(controls.delta_a - DA_RATE, DA_MIN);
  if (keys['d'] || keys['D']) controls.delta_a = Math.min(controls.delta_a + DA_RATE, DA_MAX);
  /* aileron auto-centre when released */
  if (!keys['a'] && !keys['A'] && !keys['d'] && !keys['D']) {
    controls.delta_a *= 0.85;
    if (Math.abs(controls.delta_a) < 0.001) controls.delta_a = 0;
  }

  /* rudder  (Q = left yaw → δr > 0,  E = right yaw → δr < 0) */
  if (keys['q'] || keys['Q']) controls.delta_r = Math.min(controls.delta_r + DR_RATE, DR_MAX);
  if (keys['e'] || keys['E']) controls.delta_r = Math.max(controls.delta_r - DR_RATE, DR_MIN);
  /* rudder auto-centre when released */
  if (!keys['q'] && !keys['Q'] && !keys['e'] && !keys['E']) {
    controls.delta_r *= 0.85;
    if (Math.abs(controls.delta_r) < 0.001) controls.delta_r = 0;
  }

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
  set('sv-da',  (controls.delta_a * 180 / Math.PI).toFixed(1) + '°');
  set('sv-dr',  (controls.delta_r * 180 / Math.PI).toFixed(1) + '°');
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
  bind('slider-da', () => {
    controls.delta_a = parseFloat(document.getElementById('slider-da').value) * Math.PI / 180;
    syncSliderDisplay();
  });
  bind('slider-dr', () => {
    controls.delta_r = parseFloat(document.getElementById('slider-dr').value) * Math.PI / 180;
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
  const beta  = (state.beta  || 0);
  const theta = (state.theta || 0);
  const phi   = (state.phi   || 0);
  const psi   = (state.psi   || 0);
  const gamma = theta - alpha;  // flight path angle

  /* right panel — states */
  set('val-speed',    V_T.toFixed(1) + ' m/s');
  set('val-altitude', alt.toFixed(0) + ' m');
  set('val-alpha',    (alpha * 180 / Math.PI).toFixed(2) + '°');
  set('val-beta',     (beta  * 180 / Math.PI).toFixed(2) + '°');
  set('val-theta',    (theta * 180 / Math.PI).toFixed(2) + '°');
  set('val-phi',      (phi   * 180 / Math.PI).toFixed(2) + '°');
  set('val-psi',      (psi   * 180 / Math.PI).toFixed(1) + '°');
  set('val-q',        (state.q || 0).toFixed(4) + ' rad/s');
  set('val-p',        (state.p || 0).toFixed(4) + ' rad/s');
  set('val-r',        (state.r || 0).toFixed(4) + ' rad/s');
  set('val-mach',     (state.mach || 0).toFixed(3));
  set('val-gamma',    (gamma * 180 / Math.PI).toFixed(2) + '°');

  /* force readouts - respecting frame selection */
  if (state.forces) {
    if (viewOpts.forceFrame === 'body') {
      set('val-fx', state.forces.X.toFixed(0) + ' N');
      set('val-fy', (state.forces.Y || 0).toFixed(0) + ' N');
      set('val-fz', state.forces.Z.toFixed(0) + ' N');
    } else {
      /* earth-axis: rotate body forces */
      const ct = Math.cos(theta), st = Math.sin(theta);
      const Xe =  state.forces.X * ct + state.forces.Z * st;
      const Ze = -state.forces.X * st + state.forces.Z * ct;
      set('val-fx', Xe.toFixed(0) + ' N');
      set('val-fy', (state.forces.Y || 0).toFixed(0) + ' N');
      set('val-fz', Ze.toFixed(0) + ' N');
    }
    set('val-fl', (state.forces.L_lat || 0).toFixed(0) + ' N·m');
    set('val-fm', state.forces.M.toFixed(0) + ' N·m');
    set('val-fn', (state.forces.N || 0).toFixed(0) + ' N·m');
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
  set('bar-phi',   (phi   * 180 / Math.PI).toFixed(1) + '°');
  set('bar-psi',   (psi   * 180 / Math.PI).toFixed(1) + '°');
  set('bar-xe',    (state.xE || 0).toFixed(0) + ' m');
  set('bar-ye',    (state.yE || 0).toFixed(0) + ' m');
  set('bar-ze',    (state.zE || 0).toFixed(0) + ' m');

  /* stall warning */
  const stallEl = document.getElementById('stall-warn');
  if (stallEl) stallEl.style.display = state.stall ? 'block' : 'none';

  /* autopilot mode indicator */
  set('val-ap-mode', (state.ap_mode || 'OFF').toUpperCase());

  /* fuel & mass */
  set('val-fuel', ((state.fuel || 1) * 100).toFixed(0) + '%');
  set('val-mass', (state.mass || 0).toFixed(0) + ' kg');

  /* flap display */
  set('val-flaps', ((state.flaps || 0) * 100).toFixed(0) + '%');
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
    /* hide loading screen */
    const ls = document.getElementById('loading-screen');
    if (ls) ls.classList.add('hidden');
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
      controls.delta_a  = 0.0;
      controls.delta_r  = 0.0;
      controls.i_H      = 0.0;
      const sliderDe = document.getElementById('slider-de');
      const sliderTh = document.getElementById('slider-throttle');
      const sliderIh = document.getElementById('slider-ih');
      const sliderDa = document.getElementById('slider-da');
      const sliderDr = document.getElementById('slider-dr');
      if (sliderDe) sliderDe.value = (controls.delta_e * 180 / Math.PI).toFixed(1);
      if (sliderTh) sliderTh.value = (controls.throttle * 100).toFixed(0);
      if (sliderIh) sliderIh.value = 0;
      if (sliderDa) sliderDa.value = 0;
      if (sliderDr) sliderDr.value = 0;
      syncSliderDisplay();
      trimLoaded = true;
      console.log(`[sim] trim synced: δe=${(controls.delta_e*180/Math.PI).toFixed(2)}°  thr=${(controls.throttle*100).toFixed(1)}%`);
    }

    lastState = state;
    state._viewOpts = viewOpts;  // pass display options to renderer
    render(state);
    updatePanel(state);
  });

  socket.on('stability_result', (data) => {
    console.log('[sim] stability result:', data);
    showStabilityOverlay(data);
  });

  socket.on('trim_map_result', (data) => {
    console.log('[sim] trim map result:', data);
    showTrimMapOverlay(data);
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
   FLAPS TOGGLE  (F key cycles 0% → 50% → 100% → 0%)
   ================================================================ */
let flapSetting = 0;
function toggleFlaps() {
  flapSetting = (flapSetting + 1) % 3;
  const val = [0, 0.5, 1.0][flapSetting];
  if (socket && socket.connected) {
    socket.emit('set_flaps', { flaps: val });
  }
  const el = document.getElementById('val-flaps');
  if (el) el.textContent = (val * 100).toFixed(0) + '%';
}

/* ================================================================
   AUTOPILOT
   ================================================================ */
function setAutopilot(mode) {
  const data = { mode };
  if (mode === 'pitch') {
    data.theta_cmd = parseFloat(document.getElementById('ap-theta-cmd')?.value || 0);
  } else if (mode === 'altitude') {
    data.h_cmd = parseFloat(document.getElementById('ap-h-cmd')?.value || 9144);
  } else if (mode === 'heading') {
    data.psi_cmd = parseFloat(document.getElementById('ap-psi-cmd')?.value || 0);
  }
  if (socket && socket.connected) {
    socket.emit('set_autopilot', data);
  }
  /* update button states */
  document.querySelectorAll('.ap-btn').forEach(b => b.classList.remove('active'));
  const btn = document.getElementById('ap-btn-' + mode);
  if (btn) btn.classList.add('active');
  _currentApMode = mode;
}
let _currentApMode = 'off';

/* live-update AP command when input values change while mode is active */
function _bindApInputs() {
  const hInput = document.getElementById('ap-h-cmd');
  const thetaInput = document.getElementById('ap-theta-cmd');
  const psiInput = document.getElementById('ap-psi-cmd');
  if (hInput) hInput.addEventListener('input', () => {
    if (_currentApMode === 'altitude' && socket && socket.connected)
      socket.emit('set_autopilot', { mode: 'altitude', h_cmd: parseFloat(hInput.value || 9144) });
  });
  if (thetaInput) thetaInput.addEventListener('input', () => {
    if (_currentApMode === 'pitch' && socket && socket.connected)
      socket.emit('set_autopilot', { mode: 'pitch', theta_cmd: parseFloat(thetaInput.value || 0) });
  });
  if (psiInput) psiInput.addEventListener('input', () => {
    if (_currentApMode === 'heading' && socket && socket.connected)
      socket.emit('set_autopilot', { mode: 'heading', psi_cmd: parseFloat(psiInput.value || 0) });
  });
}

/* ================================================================
   STABILITY ANALYSIS
   ================================================================ */
function requestStability() {
  if (socket && socket.connected) {
    const V = parseFloat(document.getElementById('slider-vel')?.value || 180);
    const h = parseFloat(document.getElementById('slider-alt')?.value || 9144);
    socket.emit('get_stability', { V, h });
  }
}

function requestTrimMap() {
  if (socket && socket.connected) {
    socket.emit('get_trim_map');
  }
}

/* ================================================================
   STABILITY RESULTS OVERLAY
   ================================================================ */
function showStabilityOverlay(data) {
  let overlay = document.getElementById('stability-overlay');
  if (!overlay) return;
  if (data.error) {
    overlay.querySelector('.overlay-body').innerHTML = '<p style="color:#f85149">Error: ' + data.error + '</p>';
    overlay.classList.add('open');
    return;
  }
  let html = '<h3>Trim Condition</h3>';
  const t = data.trim;
  html += `<p>V=${t.V.toFixed(1)} m/s, h=${t.h.toFixed(0)} m, alpha=${t.alpha_deg.toFixed(2)} deg, `
        + `de=${t.delta_e_deg.toFixed(2)} deg, thr=${t.throttle_pct.toFixed(1)}%</p>`;

  html += '<h3>Longitudinal Modes</h3><table><tr><th>Mode</th><th>wn</th><th>zeta</th><th>Period</th><th>T_half</th><th>Stable</th></tr>';
  (data.modes_lon || []).forEach(m => {
    html += `<tr><td>${m.name}</td><td>${m.wn.toFixed(3)}</td><td>${m.zeta.toFixed(3)}</td>`
          + `<td>${m.period < 1e5 ? m.period.toFixed(2)+'s' : 'inf'}</td>`
          + `<td>${m.t_half < 1e5 ? m.t_half.toFixed(2)+'s' : 'inf'}</td>`
          + `<td style="color:${m.stable?'#4af7b0':'#f85149'}">${m.stable?'YES':'NO'}</td></tr>`;
  });
  html += '</table>';

  html += '<h3>Lateral Modes</h3><table><tr><th>Mode</th><th>wn</th><th>zeta</th><th>Period</th><th>T_half</th><th>Stable</th></tr>';
  (data.modes_lat || []).forEach(m => {
    html += `<tr><td>${m.name}</td><td>${m.wn.toFixed(3)}</td><td>${m.zeta.toFixed(3)}</td>`
          + `<td>${m.period < 1e5 ? m.period.toFixed(2)+'s' : 'inf'}</td>`
          + `<td>${m.t_half < 1e5 ? m.t_half.toFixed(2)+'s' : 'inf'}</td>`
          + `<td style="color:${m.stable?'#4af7b0':'#f85149'}">${m.stable?'YES':'NO'}</td></tr>`;
  });
  html += '</table>';

  html += '<h3>Eigenvalues (s-plane)</h3>';
  html += '<canvas id="eig-canvas" width="300" height="200" style="background:#0d1117;border:1px solid #30363d;border-radius:4px"></canvas>';

  overlay.querySelector('.overlay-body').innerHTML = html;
  overlay.classList.add('open');

  /* draw eigenvalue plot */
  requestAnimationFrame(() => drawEigenPlot(data));
}

function drawEigenPlot(data) {
  const cvs = document.getElementById('eig-canvas');
  if (!cvs) return;
  const ctx = cvs.getContext('2d');
  const w = cvs.width, h = cvs.height;
  ctx.clearRect(0, 0, w, h);

  const allEigs = [...(data.eigenvalues_lon||[]), ...(data.eigenvalues_lat||[])];
  if (!allEigs.length) return;

  const maxR = Math.max(...allEigs.map(e => Math.abs(e.real)), 0.1);
  const maxI = Math.max(...allEigs.map(e => Math.abs(e.imag)), 0.1);
  const scale = Math.max(maxR, maxI) * 1.3;

  const cx = w/2, cy = h/2;
  const sx = (w/2-10)/scale, sy = (h/2-10)/scale;

  /* axes */
  ctx.strokeStyle = '#30363d';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();

  /* labels */
  ctx.fillStyle = '#8b949e';
  ctx.font = '9px sans-serif';
  ctx.fillText('Re', w-18, cy-4);
  ctx.fillText('Im', cx+4, 12);

  /* stability boundary */
  ctx.strokeStyle = '#f8514955';
  ctx.setLineDash([4,4]);
  ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, h); ctx.stroke();
  ctx.setLineDash([]);

  /* plot longitudinal eigs */
  (data.eigenvalues_lon||[]).forEach(e => {
    const px = cx + e.real * sx;
    const py = cy - e.imag * sy;
    ctx.fillStyle = '#58a6ff';
    ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI*2); ctx.fill();
  });

  /* plot lateral eigs */
  (data.eigenvalues_lat||[]).forEach(e => {
    const px = cx + e.real * sx;
    const py = cy - e.imag * sy;
    ctx.fillStyle = '#f7c948';
    ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI*2); ctx.fill();
  });

  /* legend */
  ctx.fillStyle = '#58a6ff'; ctx.fillRect(8, 8, 8, 8);
  ctx.fillStyle = '#8b949e'; ctx.fillText('Lon', 20, 16);
  ctx.fillStyle = '#f7c948'; ctx.fillRect(8, 20, 8, 8);
  ctx.fillStyle = '#8b949e'; ctx.fillText('Lat', 20, 28);
}

function showTrimMapOverlay(data) {
  let overlay = document.getElementById('stability-overlay');
  if (!overlay) return;
  if (data.error) {
    overlay.querySelector('.overlay-body').innerHTML = '<p style="color:#f85149">Error: ' + data.error + '</p>';
    overlay.classList.add('open');
    return;
  }
  let html = '<h3>Trim Map</h3>';
  html += '<table><tr><th>h (m) \\ V (m/s)</th>';
  data.V_range.forEach(v => html += `<th>${v.toFixed(0)}</th>`);
  html += '</tr>';

  html += '<tr><td colspan="' + (data.V_range.length+1) + '"><b>alpha (deg)</b></td></tr>';
  data.h_range.forEach((h, ih) => {
    html += `<tr><td>${h.toFixed(0)}</td>`;
    data.alpha[ih].forEach((a, iv) => {
      const ok = data.converged[ih][iv];
      html += `<td style="color:${ok?'#e6edf3':'#f85149'}">${a.toFixed(1)}</td>`;
    });
    html += '</tr>';
  });

  html += '<tr><td colspan="' + (data.V_range.length+1) + '"><b>delta_e (deg)</b></td></tr>';
  data.h_range.forEach((h, ih) => {
    html += `<tr><td>${h.toFixed(0)}</td>`;
    data.delta_e[ih].forEach((d, iv) => {
      const ok = data.converged[ih][iv];
      html += `<td style="color:${ok?'#e6edf3':'#f85149'}">${d.toFixed(1)}</td>`;
    });
    html += '</tr>';
  });

  html += '<tr><td colspan="' + (data.V_range.length+1) + '"><b>throttle (%)</b></td></tr>';
  data.h_range.forEach((h, ih) => {
    html += `<tr><td>${h.toFixed(0)}</td>`;
    data.throttle[ih].forEach((t, iv) => {
      const ok = data.converged[ih][iv];
      html += `<td style="color:${ok?'#e6edf3':'#f85149'}">${t.toFixed(0)}</td>`;
    });
    html += '</tr>';
  });

  html += '</table>';
  overlay.querySelector('.overlay-body').innerHTML = html;
  overlay.classList.add('open');
}

function closeStabilityOverlay() {
  const overlay = document.getElementById('stability-overlay');
  if (overlay) overlay.classList.remove('open');
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
  _bindApInputs();
  connectSocket();
  startControlLoop();
});