/**
 * renderer.js — HTML Canvas 2D Drawing
 * ======================================
 * Draws the 2D side-view of the Piaggio P.180 Avanti.
 * The aircraft is always centred on screen; the world scrolls behind it.
 */

/* ================================================================
   CONSTANTS
   ================================================================ */
const SKY_TOP    = '#0a1a3a';
const SKY_BOT    = '#4a90d9';
const GROUND_COL = '#3a7a3a';
const GROUND_DARK= '#2a5a2a';
const AC_COLOUR  = '#4af7b0';
const FORCE_LIFT = '#00ccff';
const FORCE_DRAG = '#ff4444';
const FORCE_WT   = '#ffaa00';
const FORCE_THR  = '#44ff44';
const FLOW_COL   = 'rgba(74, 247, 176, 0.5)';
const HUD_FONT   = '13px "IBM Plex Mono", monospace';
const HUD_COL    = '#eef2ff';

/* pixels-per-metre for the aircraft wireframe */
const AC_SCALE = 6;

/* ================================================================
   CLOUD DATA — random-ish parallax layers (generated once)
   ================================================================ */
const CLOUDS = [];
(function initClouds() {
  const rng = (a, b) => a + Math.random() * (b - a);
  for (let i = 0; i < 18; i++) {
    CLOUDS.push({
      x: rng(0, 2000),
      y: rng(0.05, 0.45),       // fraction of canvas height
      w: rng(80, 220),
      h: rng(30, 70),
      depth: rng(0.2, 1.0),     // parallax factor
    });
  }
})();

/* ================================================================
   P.180 WIREFRAME  (local frame: x forward, y up, in metres)
   ================================================================ */
const PIAGGIO = {
  /* fuselage centreline */
  fuselage: [[-6.0, 0], [-5.2, 0.55], [-3, 0.7], [2, 0.7],
             [5.5, 0.6], [7.2, 0.3], [7.2, -0.3], [5.5, -0.5],
             [2, -0.6], [-3, -0.6], [-5.2, -0.45], [-6.0, 0]],
  /* canard (forward small wing) */
  canardU: [[-4.8, 0.35], [-5.2, 0.4], [-5.6, 0.25]],
  canardL: [[-4.8, -0.35], [-5.2, -0.4], [-5.6, -0.25]],
  /* main wing */
  wingU:  [[0.5, 0.7], [-0.2, 0.75], [-0.5, 3.8], [0.3, 3.8], [0.6, 0.75]],
  wingL:  [[0.5, -0.7], [-0.2, -0.75], [-0.5, -3.8], [0.3, -3.8], [0.6, -0.75]],
  /* T-tail horizontal stabiliser */
  tailHU: [[6.2, 2.0], [5.6, 2.05], [5.2, 3.2], [5.9, 3.2], [6.3, 2.1]],
  tailHL: [[6.2, -2.0], [5.6, -2.05], [5.2, -3.2], [5.9, -3.2], [6.3, -2.1]],
  /* vertical fin */
  fin:    [[5.5, 0.6], [5.8, 2.0], [6.5, 2.0], [7.0, 0.3]],
  /* engine nacelles */
  engU:   [[2.2, 1.5], [3.3, 1.5], [3.3, 1.15], [2.2, 1.15]],
  engL:   [[2.2, -1.5], [3.3, -1.5], [3.3, -1.15], [2.2, -1.15]],
};

/* ================================================================
   CANVAS INIT
   ================================================================ */
let canvas, ctx, W, H, dpr;

function initCanvas(el) {
  canvas = el;
  ctx    = canvas.getContext('2d');
  resize();
  window.addEventListener('resize', resize);
}

function resize() {
  dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  W = rect.width;
  H = rect.height;
  canvas.width  = W * dpr;
  canvas.height = H * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

/* ================================================================
   SKY
   ================================================================ */
function drawSky(altitude) {
  /* shift gradient blue → darker as altitude rises */
  const f = Math.min(altitude / 15000, 1);
  const grad = ctx.createLinearGradient(0, 0, 0, H);
  grad.addColorStop(0, lerpColour(SKY_TOP, '#020814', f));
  grad.addColorStop(1, lerpColour(SKY_BOT, '#1a3a6a', f));
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, W, H);
}

/* ================================================================
   GROUND
   ================================================================ */
function drawGround(altitude, xE) {
  /* ground line Y: at low altitude → near bottom, scrolls down as h ↑ */
  const groundY = H * 0.8 + (altitude / 500) * H * 0.3;
  if (groundY > H) return;  // off-screen

  /* scrolling stripe pattern */
  ctx.save();
  ctx.fillStyle = GROUND_COL;
  ctx.fillRect(0, groundY, W, H - groundY);
  ctx.strokeStyle = GROUND_DARK;
  ctx.lineWidth = 1;
  const stripeW = 80;
  const offset = (xE * 0.5) % stripeW;
  for (let x = -stripeW + offset; x < W + stripeW; x += stripeW) {
    ctx.beginPath();
    ctx.moveTo(x, groundY);
    ctx.lineTo(x + stripeW * 0.5, H);
    ctx.stroke();
  }
  /* horizon line */
  ctx.strokeStyle = '#5aaa5a';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, groundY);
  ctx.lineTo(W, groundY);
  ctx.stroke();
  ctx.restore();
}

/* ================================================================
   CLOUDS
   ================================================================ */
function drawClouds(xE, altitude) {
  ctx.save();
  for (const c of CLOUDS) {
    const px = ((c.x - xE * 0.02 * c.depth) % (W + c.w * 2)) ;
    const x = ((px % (W + c.w * 2)) + (W + c.w * 2)) % (W + c.w * 2) - c.w;
    const y = c.y * H - altitude * 0.005 * c.depth;
    const a = 0.25 + 0.15 * c.depth;
    ctx.fillStyle = `rgba(255,255,255,${a})`;
    ctx.beginPath();
    ctx.ellipse(x + c.w / 2, y + c.h / 2, c.w / 2, c.h / 2, 0, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

/* ================================================================
   AIRCRAFT WIREFRAME
   ================================================================ */
function drawAircraft(cx, cy, theta) {
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-theta);               // canvas y inverted
  ctx.scale(AC_SCALE, -AC_SCALE);   // flip y so up is positive

  ctx.strokeStyle = AC_COLOUR;
  ctx.lineWidth = 1.5 / AC_SCALE;
  ctx.lineJoin = 'round';

  for (const key of Object.keys(PIAGGIO)) {
    const pts = PIAGGIO[key];
    ctx.beginPath();
    ctx.moveTo(pts[0][0], pts[0][1]);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
    ctx.stroke();
  }
  ctx.restore();
}

/* ================================================================
   FORCE ARROWS
   ================================================================ */
function drawForceArrows(cx, cy, theta, forces) {
  if (!forces) return;
  const scale = 0.003;  // N → pixels
  ctx.save();
  ctx.translate(cx, cy);

  /* Lift — up (perpendicular to velocity, in screen coords roughly up) */
  drawArrow(0, 0, 0, -forces.L_aero * scale, FORCE_LIFT, 'L');
  /* Drag — backward (along negative flight path) */
  drawArrow(0, 0, -forces.D_aero * scale, 0, FORCE_DRAG, 'D');
  /* Weight — down */
  drawArrow(0, 0, 0, forces.W_force * scale, FORCE_WT, 'W');
  /* Thrust — forward */
  drawArrow(0, 0, forces.T * scale * 3, 0, FORCE_THR, 'T');

  ctx.restore();
}

function drawArrow(x1, y1, dx, dy, colour, label) {
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 2) return;
  const angle = Math.atan2(dy, dx);
  const headLen = Math.min(12, len * 0.3);

  ctx.save();
  ctx.strokeStyle = colour;
  ctx.fillStyle   = colour;
  ctx.lineWidth   = 2.5;
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 + dx, y1 + dy);
  ctx.stroke();

  /* arrowhead */
  ctx.beginPath();
  ctx.moveTo(x1 + dx, y1 + dy);
  ctx.lineTo(x1 + dx - headLen * Math.cos(angle - 0.35),
             y1 + dy - headLen * Math.sin(angle - 0.35));
  ctx.lineTo(x1 + dx - headLen * Math.cos(angle + 0.35),
             y1 + dy - headLen * Math.sin(angle + 0.35));
  ctx.closePath();
  ctx.fill();

  /* label */
  ctx.font = '11px sans-serif';
  ctx.fillText(label, x1 + dx * 0.5 + 6, y1 + dy * 0.5 - 6);
  ctx.restore();
}

/* ================================================================
   FLOW ARROWS (freestream velocity field)
   ================================================================ */
function drawFlowArrows(alpha, V_T, xE, mode) {
  if (V_T < 1) return;
  const arrowLen = Math.min(60, V_T * 0.3);
  const cosA = Math.cos(alpha);
  const sinA = Math.sin(alpha);

  ctx.save();
  ctx.strokeStyle = FLOW_COL;
  ctx.fillStyle   = FLOW_COL;
  ctx.lineWidth   = 1.5;

  const rows = Math.floor(H / 60);
  /* In "body" mode arrows scroll with xE; in "flow" mode arrows are fixed */
  const xOffset = (mode === 'body') ? -(xE * 0.3) % 80 : 0;
  for (let r = 0; r < rows; r++) {
    const x0 = 40 + xOffset;
    const y0 = 40 + r * 60;
    const dx = arrowLen * cosA;
    const dy = -arrowLen * sinA;  // screen y inverted
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x0 + dx, y0 + dy);
    ctx.stroke();
    /* head */
    const a = Math.atan2(dy, dx);
    ctx.beginPath();
    ctx.moveTo(x0 + dx, y0 + dy);
    ctx.lineTo(x0 + dx - 8 * Math.cos(a - 0.4), y0 + dy - 8 * Math.sin(a - 0.4));
    ctx.lineTo(x0 + dx - 8 * Math.cos(a + 0.4), y0 + dy - 8 * Math.sin(a + 0.4));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

/* ================================================================
   VELOCITY VECTOR (body velocity arrow from aircraft CG)
   ================================================================ */
function drawVelocityVector(cx, cy, alpha, theta, V_T) {
  if (V_T < 1) return;
  /* velocity direction in screen coords: flight path angle γ = θ − α */
  const gamma = theta - alpha;
  const len = Math.min(80, V_T * 0.35);
  const dx = len * Math.cos(-gamma);   // screen: right = positive x
  const dy = len * Math.sin(-gamma);   // screen: down  = positive y (negated for proper direction)

  ctx.save();
  ctx.strokeStyle = '#ffee58';
  ctx.fillStyle   = '#ffee58';
  ctx.lineWidth   = 2;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + dx, cy + dy);
  ctx.stroke();
  ctx.setLineDash([]);

  /* arrowhead */
  const a = Math.atan2(dy, dx);
  const hl = 10;
  ctx.beginPath();
  ctx.moveTo(cx + dx, cy + dy);
  ctx.lineTo(cx + dx - hl * Math.cos(a - 0.35), cy + dy - hl * Math.sin(a - 0.35));
  ctx.lineTo(cx + dx - hl * Math.cos(a + 0.35), cy + dy - hl * Math.sin(a + 0.35));
  ctx.closePath();
  ctx.fill();

  /* label */
  ctx.font = '10px sans-serif';
  ctx.fillText('V', cx + dx + 8, cy + dy - 4);
  ctx.restore();
}

/* ================================================================
   ATTITUDE INDICATOR  (artificial horizon mini-dial)
   ================================================================ */
function drawAttitudeIndicator(cx, cy, radius, theta, phi, q) {
  ctx.save();
  ctx.translate(cx, cy);

  /* outer ring */
  ctx.strokeStyle = '#556';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, Math.PI * 2);
  ctx.stroke();

  /* clipped interior */
  ctx.beginPath();
  ctx.arc(0, 0, radius - 2, 0, Math.PI * 2);
  ctx.clip();

  /* sky / ground split: bank rotates, pitch translates */
  ctx.rotate(phi);                           // bank (roll) tilts horizon
  const pxPerRad = radius / 0.52;            // ~30 deg fills the radius
  ctx.translate(0, theta * pxPerRad);        // pitch moves horizon down when nose-up
  const big = radius * 4;
  ctx.fillStyle = '#3a6abf';
  ctx.fillRect(-big, -big, big * 2, big);   // sky (top half)
  ctx.fillStyle = '#8B6914';
  ctx.fillRect(-big, 0, big * 2, big);      // ground (bottom half)
  /* horizon line */
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-big, 0); ctx.lineTo(big, 0); ctx.stroke();
  /* pitch ladder — every 10° */
  ctx.strokeStyle = 'rgba(255,255,255,0.5)';
  ctx.lineWidth = 0.8;
  ctx.font = '8px sans-serif';
  ctx.fillStyle = '#fff';
  const pxPerDeg = pxPerRad * Math.PI / 180;
  for (let d = -30; d <= 30; d += 10) {
    if (d === 0) continue;
    const y = -d * pxPerDeg;
    const hw = d % 20 === 0 ? 18 : 10;
    ctx.beginPath(); ctx.moveTo(-hw, y); ctx.lineTo(hw, y); ctx.stroke();
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);  // reset
  ctx.translate(cx, cy);

  /* fixed aircraft symbol (small wings + dot) */
  ctx.strokeStyle = '#ffaa00';
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(-18, 0); ctx.lineTo(-6, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(6, 0);  ctx.lineTo(18, 0); ctx.stroke();
  ctx.fillStyle = '#ffaa00';
  ctx.beginPath(); ctx.arc(0, 0, 3, 0, Math.PI * 2); ctx.fill();

  ctx.restore();
}

/* ================================================================
   HUD OVERLAY  (text readouts on the canvas)
   ================================================================ */
function drawHUD(state) {
  if (!state) return;
  const alt   = state.altitude != null ? state.altitude : -state.zE;
  const V_T   = state.V_T || 0;
  const alpha  = (state.alpha || 0) * 180 / Math.PI;
  const beta   = (state.beta  || 0) * 180 / Math.PI;
  const theta  = (state.theta || 0) * 180 / Math.PI;
  const phi    = (state.phi   || 0) * 180 / Math.PI;
  const psi    = (state.psi   || 0) * 180 / Math.PI;
  const q_val  = state.q || 0;
  const p_val  = state.p || 0;
  const r_val  = state.r || 0;

  ctx.save();
  ctx.font = HUD_FONT;
  ctx.fillStyle = HUD_COL;
  ctx.textAlign = 'right';
  const x = W - 16;
  let y = 28;
  const dy = 20;
  ctx.fillText(`V  ${V_T.toFixed(1)} m/s`,        x, y); y += dy;
  ctx.fillText(`Alt ${alt.toFixed(0)} m`,          x, y); y += dy;
  ctx.fillText(`\u03b1  ${alpha.toFixed(2)}\u00b0`,          x, y); y += dy;
  ctx.fillText(`\u03b2  ${beta.toFixed(2)}\u00b0`,           x, y); y += dy;
  ctx.fillText(`\u03b8  ${theta.toFixed(2)}\u00b0`,          x, y); y += dy;
  ctx.fillText(`\u03a6  ${phi.toFixed(2)}\u00b0`,            x, y); y += dy;
  ctx.fillText(`\u03a8  ${psi.toFixed(1)}\u00b0`,            x, y); y += dy;
  ctx.fillText(`q  ${q_val.toFixed(4)} rad/s`,     x, y); y += dy;
  ctx.fillText(`p  ${p_val.toFixed(4)} rad/s`,     x, y); y += dy;
  ctx.fillText(`r  ${r_val.toFixed(4)} rad/s`,     x, y); y += dy;
  ctx.restore();
}

/* ================================================================
   FORCE READOUT PANEL  (bottom bar)
   ================================================================ */
function drawForceReadout(forces) {
  if (!forces) return;
  ctx.save();
  ctx.font = '14px "IBM Plex Mono", monospace';
  ctx.fillStyle = HUD_COL;
  ctx.textAlign = 'left';
  const x = W * 0.33;
  const y = H - 16;
  ctx.fillText(`ΣX = ${forces.X.toFixed(0)} N`, x, y - 36);
  ctx.fillText(`ΣZ = ${forces.Z.toFixed(0)} N`, x, y - 18);
  ctx.fillText(`ΣM = ${forces.M.toFixed(0)} N·m`, x, y);
  ctx.restore();
}

/* ================================================================
   MASTER RENDER
   ================================================================ */
function render(state) {
  if (!ctx) return;
  ctx.clearRect(0, 0, W, H);

  const opts     = state._viewOpts || {};
  const altitude = state.altitude != null ? state.altitude : -(state.zE || 0);
  const xE       = state.xE || 0;
  const theta    = state.theta || 0;
  const phi      = state.phi   || 0;
  const alpha    = state.alpha || 0;
  const beta     = state.beta  || 0;
  const V_T      = state.V_T || 0;
  const mode     = opts.mode || 'flow';

  /* 1 — background */
  drawSky(altitude);
  drawClouds(xE, altitude);
  drawGround(altitude, xE);

  /* 2 — flow arrows */
  if (opts.showFlow !== false) {
    drawFlowArrows(alpha, V_T, xE, mode);
  }

  /* 3 — aircraft position */
  let cx, cy;
  if (mode === 'body') {
    /* "Static Flow / Moving Body" — aircraft shifts based on xE */
    cx = W * 0.2 + (xE * 0.01) % (W * 0.6);
    cy = H * 0.42 + (altitude < 500 ? (500 - altitude) * 0.15 : 0);
  } else {
    /* "Static Body / Moving Flow" — aircraft centred */
    cx = W * 0.42;
    cy = H * 0.42;
  }
  drawAircraft(cx, cy, theta);

  /* 4 — force arrows */
  if (opts.showForces !== false) {
    drawForceArrows(cx, cy, theta, state.forces);
  }

  /* 5 — velocity vector */
  if (opts.showVelVec !== false) {
    drawVelocityVector(cx, cy, alpha, theta, V_T);
  }

  /* 6 — attitude indicator (now with bank angle) */
  drawAttitudeIndicator(W * 0.35, H - 80, 55, theta, phi, state.q || 0);

  /* 7 — HUD text + force readout */
  if (opts.showHud !== false) {
    drawHUD(state);
    drawForceReadout(state.forces);
  }

  /* 8 — stall warning border flash */
  if (state.stall) {
    ctx.save();
    ctx.strokeStyle = '#f85149';
    ctx.lineWidth = 6;
    ctx.strokeRect(3, 3, W - 6, H - 6);
    ctx.restore();
  }
}

/* ================================================================
   COLOUR UTIL
   ================================================================ */
function lerpColour(a, b, t) {
  const pa = hexToRgb(a), pb = hexToRgb(b);
  const r = Math.round(pa.r + (pb.r - pa.r) * t);
  const g = Math.round(pa.g + (pb.g - pa.g) * t);
  const bl = Math.round(pa.b + (pb.b - pa.b) * t);
  return `rgb(${r},${g},${bl})`;
}
function hexToRgb(hex) {
  const v = parseInt(hex.replace('#', ''), 16);
  return { r: (v >> 16) & 255, g: (v >> 8) & 255, b: v & 255 };
}
