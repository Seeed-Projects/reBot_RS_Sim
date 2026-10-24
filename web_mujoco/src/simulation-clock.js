/** Convert monotonic wall time (ms) to fixed MuJoCo steps (seconds). */
export function createSimulationClock(timestep, { maxFrameSeconds = 0.25 } = {}) {
  if (!(timestep > 0) || !Number.isFinite(timestep)) throw new RangeError('Invalid physics timestep');
  if (!(maxFrameSeconds >= timestep) || !Number.isFinite(maxFrameSeconds)) throw new RangeError('Invalid catch-up limit');
  let previous = null;
  let remainder = 0;

  function reset(now = null) {
    previous = Number.isFinite(now) ? now : null;
    remainder = 0;
  }

  function steps(now, { speed = 1, paused = false } = {}) {
    if (!(speed > 0) || !Number.isFinite(speed)) throw new RangeError('Invalid simulation speed');
    if (!Number.isFinite(now)) return 0;
    if (paused) { reset(now); return 0; }
    if (previous === null) {
      previous = now;
      return 0;
    }
    // Bound work after a long main-thread stall. Hidden tabs explicitly reset
    // the clock so returning to the page never fast-forwards the vehicle.
    const elapsed = Math.min(maxFrameSeconds, Math.max(0, (now - previous) / 1000));
    previous = now;
    remainder += elapsed * speed;
    const count = Math.floor((remainder + timestep * 1e-9) / timestep);
    remainder = Math.max(0, remainder - count * timestep);
    return count;
  }

  return { steps, reset };
}

// Preserve the grasp controller's existing 12-ms-or-faster update cadence even
// when a low render rate requires many physics substeps in one animation frame.
export function advanceSimulation(physics, steps, updateControls) {
  for (let remaining = steps; remaining > 0;) {
    updateControls();
    const count = Math.min(12, remaining);
    physics.step(count);
    remaining -= count;
  }
}
