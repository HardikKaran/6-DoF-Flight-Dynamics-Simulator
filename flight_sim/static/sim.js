/**
 * sim.js — Game Loop, SocketIO Client & Input Handling
 * =====================================================
 * Connects to the Flask-SocketIO backend, sends pilot control inputs,
 * receives the physics state at ~60 Hz, and drives the renderer.
 *
 * Controls (keyboard + UI sliders)
 * --------------------------------
 *   Arrow Up / Down   → elevator  δe  (rad, ±20°)
 *   W / S             → throttle  (0–1)
 *   A / D             → aileron   δa  (future)
 *   Q / E             → rudder    δr  (future)
 *
 * TODO: Implement the following
 * ------------------------------
 *  1. SocketIO connection to Flask backend
 *  2. Keyboard event listeners (keydown / keyup state map)
 *  3. Slider oninput bindings for δe, δa, δr, throttle
 *  4. readControls() — merge keyboard + slider state
 *  5. On 'state' event from server → call renderer.render(state)
 *  6. HUD value updates (speed, altitude, α, Θ, q, forces)
 *  7. Reset button handler
 *  8. "Static Flow / Moving Body" & "Static Body / Moving Flow" toggle
 *  9. Earth-axis / body-axis force display toggle
 */
