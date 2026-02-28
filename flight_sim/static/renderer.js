/**
 * renderer.js — HTML Canvas 2D Drawing
 * ======================================
 * Draws the 2D side-view of the Piaggio P.180 Avanti:
 *   - Sky gradient background
 *   - Ground / horizon line (scrolls with altitude)
 *   - Clouds (parallax decoration)
 *   - Wireframe or sprite aircraft rotated by pitch Θ
 *   - Force arrows (lift, drag, weight, thrust)
 *   - Flow arrows (freestream velocity vectors)
 *   - Artificial horizon / attitude indicator
 *   - HUD text overlay (speed, altitude, α, pitch, q, forces)
 *
 * The aircraft is always centred on screen; the world scrolls
 * behind it using xE (forward) and zE (altitude = −zE).
 *
 * TODO: Implement the following
 * ------------------------------
 *  1. initCanvas(canvasElement) — sizing, DPI scaling
 *  2. drawSky(ctx, altitude)
 *  3. drawGround(ctx, altitude, xE) — with scrolling texture
 *  4. drawClouds(ctx, xE, altitude) — parallax layers
 *  5. drawAircraft(ctx, cx, cy, theta, scale)
 *       - Piaggio shape: canard + mid-wing + T-tail + pusher props
 *       - OR load aircraft sprite image (see UI screenshot)
 *  6. drawForceArrows(ctx, state, forces) — ΣX, ΣZ, ΣM vectors
 *  7. drawFlowArrows(ctx, state) — freestream velocity field
 *  8. drawAttitudeIndicator(ctx, theta, q) — bottom-centre dial
 *  9. drawHUD(ctx, telemetry) — text overlays
 * 10. render(state) — master draw call (clears + calls all above)
 */
