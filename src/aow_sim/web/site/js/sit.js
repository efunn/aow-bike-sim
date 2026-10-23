// control/swing_sit.SwingSit: lying down and getting up on the righting
// module, scripted. Same phases, same defaults; the Python docstring has the
// reasoning and analysis/moves/swing_sit.py the sweep the numbers came from.
// Crank +deploy puts the RIGHT blade down; roll is +ve leaning right.

const DEG = Math.PI / 180;
const sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);

// control/recovery.ready_for_policy
export function readyForPolicy(roll, rollRate, recoverDeg, handoffRate) {
  return Math.abs(roll) < recoverDeg * DEG && Math.abs(rollRate) < handoffRate;
}

const cmd = (crank, current = 1.0, policy = true, diff = 0.0) => ({ crank, current, policy, diff });

export class SwingSit {
  static ACTIVE = new Set(["stop", "deploy", "kick", "settle", "lower", "limp",
                           "lift", "balance", "retract"]);

  constructor(deploy, recoverDeg, handoffRate) {
    this.deploy = deploy;
    this.recoverDeg = recoverDeg; this.handoffRate = handoffRate;
    this.lean = 110 * DEG; this.deployS = 0.6; this.kick = 20.0;
    this.settleS = 0.4; this.lower = 30 * DEG; this.limpS = 1.0;
    this.liftTimeoutS = 3.0; this.retractAfter = 1.0;
    this.fallenRad = 60 * DEG; this.fallenS = 0.5; this.stillRate = 2.0;
    this.leanOn = 3 * DEG; this.kickMaxS = 1.5; this.abort = 8 * DEG;
    this.release = 95 * DEG; this.releaseRate = 30 * DEG;
    this.upright = 3 * DEG; this.balanceMaxS = 4.0;
    this.retractRate = 120 * DEG; this.stillSpeed = 0.05; this.stopMaxS = 4.0;
    this.liftPolicy = false; this.liftRate = 120 * DEG;
    this.retryS = 1.0;        // a lift that timed out tries again after this
    this.reset();
  }

  reset() {
    this.retractFrom = 0.0; this.liftFrom = null; this.hold = 0.0;
    this.aborted = false; this.fell = false;
    this.phase = "riding"; this.side = 1.0; this.t0 = 0.0;
    this.crank = 0.0; this.downSince = null;
    this.retry = false;
  }

  get active() { return SwingSit.ACTIVE.has(this.phase); }

  go(phase, t) {
    if (phase === "lift" || phase === "stop") this.fell = false;
    this.phase = phase; this.t0 = t;
  }

  lieDown(t, roll, side = null) {
    this.side = side !== null ? side : (sign(roll) || 1.0);
    this.aborted = false;
    this.retry = false;
    this.go("stop", t);
  }

  getUp(t, roll) {
    this.side = sign(roll) || 1.0;
    this.liftFrom = null;
    this.downSince = null;
    this.retry = false;
    this.go("lift", t);
  }

  abortTo(t, crank) { this.aborted = true; this.retractFrom = crank; this.go("retract", t); }

  fallen(t, roll, rollRate) {
    if (Math.abs(roll) > this.fallenRad && Math.abs(rollRate) < this.stillRate) {
      if (this.downSince === null) this.downSince = t;
      return t - this.downSince >= this.fallenS;
    }
    this.downSince = null;
    return false;
  }

  step(t, roll, rollRate, crank, speed = 0.0) {
    let dt = t - this.t0, s = this.side;
    let p = this.phase;
    if (p === "stop") {
      if (speed < this.stillSpeed || dt >= this.stopMaxS) this.go("deploy", t);
      return cmd(crank);
    }
    if (p === "deploy") {
      if (dt >= this.deployS) this.go("kick", t);
      return cmd(s * this.lean);
    }
    if ((p === "kick" || p === "settle") && s * roll < -this.abort) {
      this.abortTo(t, crank);
      return cmd(crank);
    }
    if (p === "kick") {
      if (s * roll >= this.leanOn || dt >= this.kickMaxS) this.go("settle", t);
      return cmd(s * this.lean, 1.0, false, -s * this.kick);
    }
    if (p === "settle") {
      if (dt >= this.settleS) { this.go("lower", t); this.crank = s * this.lean; }
      return cmd(s * this.lean, 1.0, false);
    }
    if (p === "lower") {
      const mag = Math.max(0.0, this.lean - this.lower * dt);
      if (mag === 0.0) this.go("limp", t);
      return cmd(s * mag, 1.0, false);
    }
    if (p === "limp") {
      const frac = this.limpS > 0 ? Math.max(0.0, 1.0 - dt / this.limpS) : 0.0;
      if (frac === 0.0) this.go("lying", t);
      return cmd(0.0, frac, false);
    }
    if (p === "lying") {
      if (!(this.retry && dt >= this.retryS)) return cmd(crank, 0.0, false);
      this.getUp(t, roll);      // the side it lies on NOW
      dt = 0.0; s = this.side; p = this.phase;
    }
    if (p === "lift") {
      if (this.liftFrom === null) this.liftFrom = crank;
      let goal = s * this.deploy;
      if (this.liftRate !== null) {
        const lim = this.liftRate * dt;
        goal = this.liftFrom + Math.max(-lim, Math.min(lim, goal - this.liftFrom));
      }
      if (readyForPolicy(roll, rollRate, this.recoverDeg, this.handoffRate)) {
        this.go("balance", t);
        this.hold = goal;
        return cmd(goal);
      }
      if (dt > this.liftTimeoutS) {
        this.go("limp", t);
        this.retry = this.retryS !== null;
      }
      return cmd(goal, 1.0, this.liftPolicy);
    }
    if ((p === "balance" || p === "retract") && Math.abs(roll) > this.fallenRad && !this.fell) {
      this.fell = true;
      this.retractFrom = crank;
      this.go("retract", t);
      return cmd(crank);
    }
    if (p === "balance") {
      const mag = Math.max(Math.min(Math.abs(this.hold), this.release),
                           Math.abs(this.hold) - this.releaseRate * dt);
      const eased = mag <= this.release + 1e-9;
      if (dt >= this.balanceMaxS || (dt >= this.retractAfter && eased && Math.abs(roll) < this.upright)) {
        this.retractFrom = s * mag;
        this.go("retract", t);
      }
      return cmd(s * mag);
    }
    if (p === "retract") {
      const goal = this.retractFrom - sign(this.retractFrom)
        * Math.min(Math.abs(this.retractFrom), this.retractRate * dt);
      if (Math.abs(goal) < 1e-9 && (Math.abs(crank) < 5 * DEG || this.fell)) this.go("riding", t);
      return cmd(goal);
    }
    return cmd(0.0);
  }
}
