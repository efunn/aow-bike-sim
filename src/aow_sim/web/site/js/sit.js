// control/swing_sit.SwingSit: lying down and getting up on the righting
// module, scripted. Same phases, same defaults; the Python docstring has the
// reasoning and analysis/moves/swing_sit.py the sweep the numbers came from.
// Crank +deploy puts the RIGHT blade down; roll is +ve leaning right.
// Options are the Python keywords in camelCase, in degrees where it takes
// degrees (`leanDeg`, `lowerMap`, ...).

const DEG = Math.PI / 180;
const sign = (x) => (x > 0 ? 1 : x < 0 ? -1 : 0);
const clip = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

// np.interp: xp increasing, clamped at the ends.
function interp(x, xp, fp) {
  const n = xp.length;
  if (x <= xp[0]) return fp[0];
  if (x >= xp[n - 1]) return fp[n - 1];
  let i = 1;
  while (xp[i] < x) i++;
  const t = (x - xp[i - 1]) / (xp[i] - xp[i - 1]);
  return fp[i - 1] + t * (fp[i] - fp[i - 1]);
}

// control/recovery.ready_for_policy
export function readyForPolicy(roll, rollRate, recoverDeg, handoffRate) {
  return Math.abs(roll) < recoverDeg * DEG && Math.abs(rollRate) < handoffRate;
}

const cmd = (crank, { current = 1.0, policy = true, diff = 0.0, limp = false } = {}) =>
  ({ crank, current, policy, diff, limp });
const off = (crank, o = {}) => cmd(crank, { ...o, policy: false });

const DEFAULTS = {
  leanDeg: 110.0, deployS: 0.6, kick: 20.0, settleS: 0.4,
  leanOnDeg: 3.0, kickMaxS: 1.5, abortDeg: 8.0,
  releaseDeg: 95.0, releaseDps: 30.0, uprightDeg: 3.0, balanceMaxS: 4.0,
  retractDps: 120.0, stillSpeed: 0.05, stopMaxS: 4.0,
  lowerDps: 30.0, limpS: 1.0, restDeg: 0.0, lowerMap: null,
  lowerRollDps: 60.0, lowerRollAcc: 240.0,
  liftTimeoutS: 3.0, retractAfter: 1.0, liftPolicy: false, liftDps: 120.0,
  fallenDeg: 60.0, fallenS: 0.5, stillRate: 2.0, retryS: 1.0,
  handOff: true, blockDeg: 20.0, blockMoveDeg: 3.0, blockS: 0.5,
  propTolDeg: 20.0, pumpS: 0.0, pumpEase: 0.0,
};

export class SwingSit {
  static ACTIVE = new Set(["stop", "deploy", "kick", "settle", "lower", "limp",
                           "lift", "pump", "balance", "retract"]);

  constructor(deploy, recoverDeg, handoffRate, opts = {}) {
    const o = { ...DEFAULTS, ...opts };
    this.deploy = deploy;
    this.recoverDeg = recoverDeg; this.handoffRate = handoffRate;
    this.lean = o.leanDeg * DEG; this.deployS = o.deployS; this.kick = o.kick;
    this.settleS = o.settleS; this.lower = o.lowerDps * DEG;
    this.rest = o.restDeg * DEG;
    // (crank, roll) in the side's frame, crank falling, roll rising
    this.lowerMap = o.lowerMap && o.lowerMap.map((a) => a.map((x) => x * DEG));
    this.lowerRollRate = o.lowerRollDps * DEG;
    this.lowerRollAcc = o.lowerRollAcc * DEG;
    this.limpS = o.limpS; this.liftTimeoutS = o.liftTimeoutS;
    this.retractAfter = o.retractAfter;
    this.fallenRad = o.fallenDeg * DEG; this.fallenS = o.fallenS;
    this.stillRate = o.stillRate;
    this.retryS = o.retryS;
    this.handOff = o.handOff;
    this.block = o.blockDeg * DEG; this.blockS = o.blockS;
    this.blockMove = o.blockMoveDeg * DEG;
    this.pumpS = o.pumpS; this.pumpEase = o.pumpEase;
    this.propTol = o.propTolDeg * DEG;
    this.leanOn = o.leanOnDeg * DEG; this.kickMaxS = o.kickMaxS;
    this.abort = o.abortDeg * DEG;
    this.release = o.releaseDeg * DEG; this.releaseRate = o.releaseDps * DEG;
    this.upright = o.uprightDeg * DEG; this.balanceMaxS = o.balanceMaxS;
    this.retractRate = o.retractDps * DEG;
    this.stillSpeed = o.stillSpeed; this.stopMaxS = o.stopMaxS;
    this.liftPolicy = o.liftPolicy;
    this.liftRate = o.liftDps === null ? null : o.liftDps * DEG;
    this.reset();
  }

  reset() {
    this.plan = null;
    this.blockC = 0.0; this.blockBest = Infinity; this.blockT = null;
    this.pumpFrom = 0.0; this.pumpUsed = 0.0; this.push = true;
    this.failed = null;
    this.retry = false;
    this.retractFrom = 0.0; this.liftFrom = null; this.hold = 0.0;
    this.aborted = false; this.fell = false;
    this.phase = "riding"; this.side = 1.0; this.t0 = 0.0;
    this.crank = 0.0; this.downSince = null;
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

  getUp(t, roll, side = null) {
    this.side = side !== null ? side : (sign(roll) || 1.0);
    this.liftFrom = null;
    this.blockT = null;
    this.blockBest = Infinity;
    this.pumpUsed = 0.0;
    this.failed = null;
    this.downSince = null;
    this.retry = false;
    this.go("lift", t);
  }

  lieFromProp(t, crank) {
    this.side = sign(crank) || 1.0;
    this.aborted = false;
    this.retry = false;
    this.startLower(t, crank);
  }

  startLower(t, crank) {
    this.crank = crank;
    this.plan = null;
    if (this.lowerMap !== null) {
      const [uc, wc] = this.lowerMap;
      const u0 = this.side * crank;
      if (u0 > uc[uc.length - 1]) {
        const w0 = interp(-u0, uc.map((x) => -x), wc);
        const d = wc[wc.length - 1] - w0;
        const v = this.lowerRollRate, a = this.lowerRollAcc;
        if (d < v * v / a) {                    // never reaches cruise
          const ta = Math.sqrt(d / a);
          this.plan = [w0, wc[wc.length - 1], 2 * ta, ta, a * ta];
        } else {
          const ta = v / a;
          this.plan = [w0, wc[wc.length - 1], 2 * ta + (d - v * ta) / v, ta, v];
        }
      }
    }
    this.go("lower", t);
  }

  lowerCrank(dt) {
    if (this.plan === null) return Math.max(-this.rest, this.side * this.crank - this.lower * dt);
    const [w0, w1, T, ta, v] = this.plan;
    const [uc, wc] = this.lowerMap;
    if (dt >= T) return Math.max(-this.rest, uc[uc.length - 1] - this.lower * (dt - T));
    const a = ta > 0 ? v / ta : 0.0;
    let w;
    if (dt < ta) w = w0 + 0.5 * a * dt * dt;
    else if (dt < T - ta) w = w0 + 0.5 * v * ta + v * (dt - ta);
    else w = w1 - 0.5 * a * (T - dt) ** 2;
    return Math.min(this.side * this.crank, interp(w, wc, uc));
  }

  engage(t) {
    if (this.phase !== "propped") throw new Error(`engage from ${this.phase}, not propped`);
    this.hold = this.side * this.lean;
    this.go("balance", t);
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
    if (this.phase === "stop") {
      if (speed < this.stillSpeed || dt >= this.stopMaxS) this.go("deploy", t);
      return cmd(crank);
    }
    if (this.phase === "deploy") {
      if (dt >= this.deployS) this.go("kick", t);
      return cmd(s * this.lean);
    }
    if ((this.phase === "kick" || this.phase === "settle") && s * roll < -this.abort) {
      this.abortTo(t, crank);
      return cmd(crank);
    }
    if (this.phase === "kick") {
      if (s * roll >= this.leanOn || dt >= this.kickMaxS) this.go("settle", t);
      return off(s * this.lean, { diff: -s * this.kick });
    }
    if (this.phase === "settle") {
      if (dt >= this.settleS) this.startLower(t, s * this.lean);
      return off(s * this.lean);
    }
    if (this.phase === "lower") {
      const u = this.lowerCrank(dt);
      if (u === -this.rest) this.go("limp", t);
      return off(s * u);
    }
    if (this.phase === "limp") {
      const frac = this.limpS > 0 ? Math.max(0.0, 1.0 - dt / this.limpS) : 0.0;
      if (frac === 0.0) this.go("lying", t);
      return off(-s * this.rest, { current: frac });
    }
    if (this.phase === "lying") {
      if (!(this.retry && this.retryS !== null && dt >= this.retryS)) return off(crank, { current: 0.0 });
      this.getUp(t, roll);      // the side it lies on NOW
      dt = 0.0; s = this.side;
    }
    if (this.phase === "lift") {
      if (this.liftFrom === null) this.liftFrom = crank;
      const target = s * (this.handOff ? this.deploy : this.lean);
      let goal = target;
      const rate = this.liftRate;
      if (rate !== null) goal = this.liftFrom + clip(goal - this.liftFrom, -rate * dt, rate * dt);
      if (!this.handOff) {
        // BLOCKED: behind its ramp and making no forward progress -- neither
        // the crank nor a new best roll -- for `blockS`.
        const best = s * roll < this.blockBest - 1.0 * DEG;
        if (s * (goal - crank) > this.block) {
          if (this.blockT === null || best || s * (crank - this.blockC) > this.blockMove) {
            this.blockT = t; this.blockC = crank;
            this.blockBest = Math.min(this.blockBest, s * roll);
          }
        } else {
          this.blockT = null;
          this.blockBest = s * roll;
        }
        if (this.blockT !== null && t - this.blockT >= this.blockS) {
          if (this.pumpUsed < this.pumpS) {
            this.push = true;
            this.pumpFrom = crank;
            this.go("pump", t);
            return off(target);
          }
          this.failed = "blocked";
          this.startLower(t, crank);
          return off(crank);
        }
        const ramp = rate === null ? 0.0 : Math.abs(target - this.liftFrom) / rate;
        if (Math.abs(goal - target) < 1e-6 && (
          readyForPolicy(roll, rollRate, this.recoverDeg, this.handoffRate)
          || (dt >= ramp + this.settleS
              && Math.abs(crank - target) <= this.propTol
              && s * roll > 0.0 && s * roll < this.fallenRad     // onto ITS blade
              && Math.abs(rollRate) < 10.0 * DEG))) {
          this.go("propped", t);
          return off(goal, { limp: true });
        }
        if (dt > this.liftTimeoutS) {
          this.failed = "timeout";
          this.startLower(t, crank);
          return off(crank);
        }
      } else if (readyForPolicy(roll, rollRate, this.recoverDeg, this.handoffRate)) {
        this.go("balance", t);
        this.hold = goal;
        return cmd(goal);
      }
      if (dt > this.liftTimeoutS) {
        this.go("limp", t);
        this.retry = this.retryS !== null;
      }
      return cmd(goal, { policy: this.liftPolicy });
    }
    if (this.phase === "pump") {
      if (s * crank >= this.lean - this.propTol) {
        this.pumpUsed += dt;
        this.liftFrom = s * this.lean; this.blockT = null;
        this.blockBest = Infinity;
        this.go("lift", t);
        return off(s * this.lean);
      }
      if (this.pumpUsed + dt >= this.pumpS) {
        this.failed = "blocked";
        this.startLower(t, crank);
        return off(crank);
      }
      const up = s * rollRate;
      if (up < -5.0 * DEG) this.push = true;
      else if (up > 5.0 * DEG) this.push = false;
      return this.push ? off(s * this.lean) : off(crank, { current: this.pumpEase });
    }
    if (this.phase === "propped") return off(s * this.lean, { limp: true });
    if ((this.phase === "balance" || this.phase === "retract") && Math.abs(roll) > this.fallenRad && !this.fell) {
      this.fell = true;
      this.retractFrom = crank;
      this.go("retract", t);
      return cmd(crank);
    }
    if (this.phase === "balance") {
      const mag = Math.max(Math.min(Math.abs(this.hold), this.release),
                           Math.abs(this.hold) - this.releaseRate * dt);
      const eased = mag <= this.release + 1e-9;
      if (dt >= this.balanceMaxS || (dt >= this.retractAfter && eased && Math.abs(roll) < this.upright)) {
        this.retractFrom = s * mag;
        this.go("retract", t);
      }
      return cmd(s * mag);
    }
    if (this.phase === "retract") {
      const goal = this.retractFrom - sign(this.retractFrom)
        * Math.min(Math.abs(this.retractFrom), this.retractRate * dt);
      if (Math.abs(goal) < 1e-9 && (Math.abs(crank) < 5 * DEG || this.fell)) this.go("riding", t);
      return cmd(goal);
    }
    return cmd(0.0);
  }
}
