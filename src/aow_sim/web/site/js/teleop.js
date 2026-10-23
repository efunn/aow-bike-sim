// The operator's command, as teleop builds it (run_drive.py, `apply` and
// `turn`): a speed that steps on a fresh press, builds while held and coasts
// back to zero when released, and a heading SETPOINT that slews while held
// and stays where it is left, at most 35 deg ahead of the bike. Same
// constants as teleop, with one difference: a fresh turn press does NOT jump
// the heading 10 deg here (teleop's does), which reads as a glitch to someone
// who has not driven it before.

const STEP_V = 0.25;        // m/s on a fresh press
const ACCEL = 1.5;          // m/s^2 while held
const ACCEL_REV = 1.1;      // m/s^2 building reverse speed
const BRAKE = 2.5;          // m/s^2 with the opposite key held
const DECAY = 0.6;          // m/s^2 coast-down once released
const TURN_RATE = 1.2;      // rad/s while held
const LEAD_MAX = 35 * Math.PI / 180;

const wrapPi = (x) => Math.atan2(Math.sin(x), Math.cos(x));

export class Teleop {
  constructor(vMax) {
    this.vMax = vMax;
    this.v = 0.0;
    this.psi = 0.0;          // heading setpoint, world, unwrapped
    this.held = new Set();   // "up" | "down" | "left" | "right"
    this.armed = true;       // the lead clamp; a snap disarms it
  }

  reset(psi) { this.v = 0.0; this.psi = psi; this.held.clear(); this.armed = true; }

  // A fresh press: the speed keys step before they ramp; the turn keys only
  // start their slew.
  press(name) {
    if (this.held.has(name)) return;
    this.held.add(name);
    if (name === "up" || name === "down") {
      const dir = name === "up" ? 1 : -1;
      this.v = Math.max(-this.vMax, Math.min(this.vMax, this.v + dir * STEP_V));
    }
  }

  release(name) { this.held.delete(name); }

  // A 90 deg heading snap (teleop's 6 / 7).
  snap(dir, bikePsi) { this.turn(dir * Math.PI / 2, bikePsi, false); }

  stop() { this.v = 0.0; }

  // Move the heading setpoint. A held turn (`clamp`) is refused only in the
  // direction that would GROW a lead already at LEAD_MAX -- it can always
  // come back. A snap leads on purpose: it disarms the clamp, both
  // directions are then free, and `update` re-arms it once the bike is back
  // inside the band (run_drive.lead_blocks and teleop's `turn`).
  turn(delta, bikePsi, clamp = true) {
    const lead = wrapPi(this.psi - bikePsi);
    if (clamp && this.armed
        && ((delta > 0 && lead >= LEAD_MAX) || (delta < 0 && lead <= -LEAD_MAX))) return;
    if (!clamp) this.armed = false;
    this.psi += delta;
  }

  update(dt, bikePsi) {
    if (!this.armed && Math.abs(wrapPi(this.psi - bikePsi)) <= LEAD_MAX) this.armed = true;
    const h = this.held;
    const rampV = (h.has("up") ? 1 : 0) - (h.has("down") ? 1 : 0);
    if (rampV) {
      const rate = this.v * rampV < -1e-9 ? BRAKE : rampV > 0 ? ACCEL : ACCEL_REV;
      this.v = Math.max(-this.vMax, Math.min(this.vMax, this.v + rampV * rate * dt));
    } else {
      this.v -= Math.sign(this.v) * Math.min(Math.abs(this.v), DECAY * dt);
    }
    const rampPsi = (h.has("left") ? 1 : 0) - (h.has("right") ? 1 : 0);
    if (rampPsi) this.turn(rampPsi * TURN_RATE * dt, bikePsi);
  }

  // The world-frame velocity command: the speed along the commanded heading.
  vCmdWorld() { return [this.v * Math.cos(this.psi), this.v * Math.sin(this.psi)]; }
}
