// The bike's control loop: GeneralEnv.step, ported line for line.
//
// Each class names the Python it mirrors. The constants come from
// assets/config.json, which export.py reads off the live Python objects, so a
// number here is never re-derived from bike_params. check.mjs replays a Python
// trace through this file; change the arithmetic and re-run it.
//
// Plain ES module, no DOM: the page and node both import it.
//
// Views into WASM memory (data.qpos and friends) are fetched afresh at each
// use, never cached: a view goes stale if the heap grows, and efc_force moves
// with the arena every step.

import { SwingSit } from "./sit.js";

// Python's round(): half to even. Ties are rare on simulator floats, but the
// RateFilter tap count is round(25 / 10) = round(2.5), which is 2, not 3.
export function roundHalfEven(x) {
  const r = Math.round(x);
  return Math.abs(x % 1) === 0.5 ? 2 * Math.round(x / 2) : r;
}

const clip = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
const wrapPi = (x) => Math.atan2(Math.sin(x), Math.cos(x));
const sigmoid = (x) => 0.5 * (1.0 + Math.tanh(0.5 * x));

// control/balance.extract_state, the parts the observation reads.
export function chassisState(d) {
  const q = d.qpos, v = d.qvel;
  let w = q[3], x = q[4], y = q[5], z = q[6];
  const n = Math.hypot(w, x, y, z);
  w /= n; x /= n; y /= n; z /= n;
  const R00 = 1 - 2 * (y * y + z * z), R10 = 2 * (x * y + z * w);
  const R20 = 2 * (x * z - y * w), R21 = 2 * (y * z + x * w);
  const R22 = 1 - 2 * (x * x + y * y);
  const yaw = Math.atan2(R10, R00);
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return {
    roll: Math.atan2(R21, R22), roll_rate: v[3], yaw,
    v_lon: c * v[0] + s * v[1], v_lat: -s * v[0] + c * v[1],
    pitch: Math.asin(clip(R20, -1, 1)), pitch_rate: -v[4], yaw_rate: v[5],
  };
}

// drivetrain_model.DrivetrainSim: the XC430 firmware loop, sticking Coulomb
// friction on the input shafts, and the differential's detent.
class Drivetrain {
  constructor(cfg) {
    const t = cfg.drivetrain;
    Object.assign(this, t);
    this.cmdId = [cfg.actuators.drive_a, cfg.actuators.drive_b];
    this.motorId = [cfg.actuators.drive_a_motor, cfg.actuators.drive_b_motor];
    this.qadr = [cfg.joints.input_a_spin.qpos, cfg.joints.input_b_spin.qpos];
    this.dadr = [cfg.joints.input_a_spin.dof, cfg.joints.input_b_spin.dof];
    this.phase = 0.0;
    this.ready = false;
  }

  reset(d, phase = null) {
    const q = d.qpos, v = d.qvel, ctrl = d.ctrl;
    this.kC = 0; this.kM = 0;
    this.ref = [0, 0]; this.wf = [0, 0]; this.duty = [0, 0];
    this.cmd = [new Array(this.nc).fill(0), new Array(this.nc).fill(0)];
    this.th = [[], []]; this.w = [[], []];
    for (let i = 0; i < 2; i++) {
      const th = q[this.qadr[i]] / this.belt, w = v[this.dadr[i]] / this.belt;
      this.ref[i] = th; this.wf[i] = w;
      this.th[i] = new Array(this.nm).fill(th);
      this.w[i] = new Array(this.nm).fill(w);
      ctrl[this.motorId[i]] = 0.0;
    }
    this.fa = [q[this.qadr[0]], q[this.qadr[1]]];
    if (this.detent) {
      if (phase !== null) this.phase = phase;
      const g = this.detent;
      this.anchor = g.ga * q[this.qadr[0]] + g.gb * q[this.qadr[1]];
    }
    const f = d.qfrc_applied;
    f[this.dadr[0]] = 0.0; f[this.dadr[1]] = 0.0;
    this.tLast = d.time;
    this.ready = true;
    this.n = this.every - 1;
  }

  preStep(d) {
    const t = d.time;
    if (!this.ready || t < this.tLast) this.reset(d);
    this.tLast = t;
    this.n += 1;
    const fw = this.n >= this.every;
    if (fw) this.n = 0;
    const q = d.qpos, v = d.qvel;
    const fric = [0, 0];
    if (fw) {
      const ctrl = d.ctrl, h = this.h_fw, belt = this.belt, vlim = this.vlim;
      const kp = this.kp, ki = this.ki, alpha = this.alpha;
      const kc = this.kC, kc1 = (this.kC + 1) % this.nc;
      const km = this.kM, km1 = (this.kM + 1) % this.nm;
      for (let i = 0; i < 2; i++) {
        let vr = ctrl[this.cmdId[i]] / belt;
        vr = vr > vlim ? vlim : vr < -vlim ? -vlim : vr;
        const buf = this.cmd[i];
        buf[kc] = vr;
        const vRef = buf[kc1];
        const th = q[this.qadr[i]] / belt, w = v[this.dadr[i]] / belt;
        const bt = this.th[i], bw = this.w[i];
        bt[km] = th; bw[km] = w;
        const wf = this.wf[i] + alpha * (bw[km1] - this.wf[i]);
        this.wf[i] = wf;
        let ref = this.ref[i] + vRef * h;
        let duty = kp * (vRef - wf) + ki * (ref - bt[km1]);
        if (duty > 1.0) {
          if (ki > 0.0) ref -= (duty - 1.0) / ki;
          duty = 1.0;
        } else if (duty < -1.0) {
          if (ki > 0.0) ref -= (duty + 1.0) / ki;
          duty = -1.0;
        }
        this.ref[i] = ref;
        this.duty[i] = duty;
        ctrl[this.motorId[i]] = this.stall * (this.supply * duty - w / this.w0) / belt;
      }
      this.kC = kc1; this.kM = km1;
    }
    for (let i = 0; i < 2; i++) {
      const x = q[this.qadr[i]];
      let f = -this.k_in * (x - this.fa[i]) - this.c_in * v[this.dadr[i]];
      const fc = this.fc_in[i];
      if (f > fc || f < -fc) {
        f = f > 0.0 ? fc : -fc;
        this.fa[i] = x + f / this.k_in;
      }
      fric[i] = f;
    }
    if (this.detent) {
      const g = this.detent;
      const phi = g.ga * q[this.qadr[0]] + g.gb * q[this.qadr[1]];
      const P = g.period;
      const x = ((((phi - this.phase) % P) + P) % P) - 0.5 * P;   // Python %
      let qf = 0.0;
      if (Math.abs(x) < 0.5 * g.width) {
        const c = Math.cos(Math.PI * x / g.width);
        const limit = g.fmax * c * c;
        const vphi = g.ga * v[this.dadr[0]] + g.gb * v[this.dadr[1]];
        qf = -g.k_stick * (phi - this.anchor) - g.c_stick * vphi;
        if (qf > limit || qf < -limit) {
          qf = qf > 0.0 ? limit : -limit;
          this.anchor = phi + qf / g.k_stick;
        }
      } else {
        this.anchor = phi;
      }
      fric[0] += g.ga * qf;
      fric[1] += g.gb * qf;
    }
    const fa = d.qfrc_applied;
    fa[this.dadr[0]] = fric[0];
    fa[this.dadr[1]] = fric[1];
  }
}

// gearbox_friction.GearboxFriction: load-proportional friction on the steer,
// written into the model's dof_frictionloss before every step.
class Gearbox {
  constructor(g, model) { Object.assign(this, g); this.m = model; this.t = -1.0; }

  reset(d) {
    const fl = this.m.dof_frictionloss;
    let below = 0;
    for (let i = 0; i < this.dof; i++) if (fl[i] > 0) below++;
    this.below = below;
    fl[this.dof] = this.base + this.s0;
    this.load = 0.0;
    this.t = d.time;
  }

  // For a joint driven by a MuJoCo actuator (the steer): the motor torque is
  // the actuator's force on the dof minus the joint's damping.
  preStep(d) {
    const dof = this.dof;
    this.update(d, d.qfrc_actuator[dof] - this.damp * d.qvel[dof]);
  }

  // This step's friction limit from the motor torque `tau` at the joint.
  update(d, tau) {
    const t = d.time;
    if (t < this.t) this.reset(d);
    this.t = t;
    const dof = this.dof, w = d.qvel[dof];
    const a = tau >= 0.0 ? tau : -tau;
    let f0, c, helping;
    if (-this.w_stick < w && w < this.w_stick) {
      f0 = this.s0; c = this.sc;
      const r = d.ne + this.below;
      helping = r >= d.nefc || d.efc_force[r] * tau >= 0.0;
    } else {
      f0 = this.r0; c = this.rc;
      helping = w * tau < 0.0;
    }
    let F;
    if (helping) { F = (f0 + c * a) / (1.0 - c); this.load = a + F; }
    else { F = (f0 + c * a) / (1.0 + c); this.load = a - F; }
    const fl = this.m.dof_frictionloss;
    if (this.touch === null) {
      let b = this.base;
      if (this.base_q2) { const qq = d.qpos[this.qadr]; b += this.base_q2 * qq * qq; }
      fl[dof] = b + F;
    } else {
      const T = d.sensordata[this.touch];
      fl[dof] = this.ha + this.hb * (T >= 0.0 ? T : -T) + F;
    }
  }
}

// righting_servo.CurrentBasedPositionServo: the righting crank's XC330 in
// current-based position mode, torque through qfrc_applied, with its own
// gearbox friction. ctrl[swing] is the goal angle.
class CrankServo {
  constructor(c, model) {
    Object.assign(this, c);
    this.goalCurrentBase = c.goal_current;
    this.goalCurrent = c.goal_current;
    this.friction = new Gearbox(c.friction, model);
    this.ready = false;
  }

  reset(d) {
    this.cmd = new Array(this.nc).fill(d.ctrl[this.aid]);
    this.k = 0;
    this.torque = 0.0;
    d.qfrc_applied[this.dadr] = 0.0;
    this.friction.reset(d);
    this.tLast = d.time;
    this.ready = true;
  }

  torqueFor(iCmd, w) {
    const a = Math.abs(iCmd) - this.i0;
    if (a <= 0.0) return 0.0;
    const tau = Math.sign(iCmd) * this.k * a;
    const s = this.supply, x = w / this.w0;
    const u = (tau / this.ts + x) / s;
    if (u > 1.0 || u < -1.0) return this.ts * (s * (u > 0.0 ? 1.0 : -1.0) - x);
    return tau;
  }

  preStep(d) {
    if (!this.ready || d.time < this.tLast) this.reset(d);
    this.tLast = d.time;
    const k = this.k, k1 = (k + 1) % this.nc;
    this.cmd[k] = d.ctrl[this.aid];
    const goal = this.cmd[k1];
    this.k = k1;
    const q = d.qpos[this.qadr], w = d.qvel[this.dadr];
    const iGoal = this.goalCurrent * this.amps_per_count;
    const iCmd = clip(this.kp_a * (goal - q) - this.kd_a * w, -iGoal, iGoal);
    const tau = this.torqueFor(iCmd, w);
    this.torque = tau;
    this.friction.update(d, tau);
    d.qfrc_applied[this.dadr] = tau;
  }
}

// hw/rate_filter.RateFilter
class RateFilter {
  constructor(windowMs, taper, nominalDtMs) {
    const n = Math.max(1, roundHalfEven(windowMs / nominalDtMs));
    const w = [];
    for (let i = 0; i < n; i++) w.push(n > 1 ? 1.0 + (taper - 1.0) * i / (n - 1) : 1.0);
    const s = w.reduce((a, b) => a + b, 0);
    this.weights = w.map((x) => x / s);
    this.n = n;
    this.buf = [];
  }
  _value() {
    let num = 0, den = 0;
    for (let i = 0; i < this.buf.length; i++) {
      num += this.weights[i] * this.buf[i];
      den += this.weights[i];
    }
    return num / den;
  }
  update(rate) {
    this.buf.unshift(rate);
    if (this.buf.length > this.n) this.buf.pop();
    return this._value();
  }
  peek() { return this.buf.length ? this._value() : 0.0; }
}

// sim_odometry.SimOdometry (encoder "counts", fusion "front", no AHRS model)
// around hw/odometry.VelocityEstimator.
class Odometry {
  constructor(cfg) { Object.assign(this, cfg.odometry); this.reset(); }

  reset() {
    this.v_lon = 0.0; this.v_lat = 0.0;
    this.acc = 0.0;
    this.last = [0.0, 0.0];
    const f = () => new RateFilter(this.window_ms, this.taper, 1000.0 * this.odo_dt);
    this.filt = [f(), f()];
    this.prev = [null, null];
  }

  _sensor(d, name) {
    const [adr, dim] = this.sensors[name];
    const s = d.sensordata;
    return Array.from({ length: dim }, (_, i) => s[adr + i]);
  }

  _rates(d, dt) {
    const out = [0, 0];
    ["input_a_pos", "input_b_pos"].forEach((name, k) => {
      const counts = roundHalfEven(this._sensor(d, name)[0] / this.belt_ratio * this.counts_per_rad);
      const prev = this.prev[k];
      this.prev[k] = counts;
      if (prev === null) { out[k] = this.filt[k].peek(); return; }
      const W = this.pos_wrap, h = W / 2;
      const delta = ((((counts - prev + h) % W) + W) % W) - h;
      out[k] = this.filt[k].update((delta / this.counts_per_rad) / dt);
    });
    return out;
  }

  _advance(d, dt) {
    const [w, x, y, z] = this._sensor(d, "ahrs_quat");
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const pitch = Math.asin(clip(2 * (w * y - z * x), -1.0, 1.0));
    const [wa, wb] = this._rates(d, dt);
    const q = this.counts_per_rad * this.steer_gear_ratio;
    const steer = roundHalfEven(this._sensor(d, "steer_pos")[0] * q) / q;
    const yawRate = this._sensor(d, "ahrs_gyro")[2];
    const accel = this._sensor(d, "ahrs_accel");
    // VelocityEstimator.update
    let aLat = accel[1] - this.gravity * Math.sin(roll) * Math.cos(pitch);
    aLat -= this.v_lon * yawRate;
    const vLonW = (this.mix_hub_a * wa * this.belt_ratio
                   + this.mix_hub_b * wb * this.belt_ratio) * this.r_wheel;
    this.v_lon += (vLonW - this.v_lon) * this.lon_smooth;
    const theta = Math.atan2(Math.cos(this.rake) * Math.sin(steer), Math.cos(steer));
    const c = Math.cos(theta);
    let conf = clip((Math.abs(c) - this.cos_min) / (1.0 - this.cos_min), 0.0, 1.0) ** 2;
    let vLatC = 0.0;
    if (Math.abs(c) < 1e-6) conf = 0.0;
    else vLatC = this.v_lon * (Math.sin(theta) / c) - yawRate * this.wheelbase;
    const pred = this.v_lat + aLat * dt;
    this.v_lat = pred + conf * (vLatC - pred);
    return [this.v_lon, this.v_lat];
  }

  update(d, dt) {
    this.acc += dt;
    const n = Math.trunc(this.acc / this.odo_dt);
    if (n) {
      this.acc -= n * this.odo_dt;
      for (let i = 0; i < Math.min(n, 4); i++) this.last = this._advance(d, this.odo_dt);
    }
    return this.last;
  }
}

// control/policy.MinGRUPolicy
export class Policy {
  constructor(cfg, blob) {
    const f32 = new Float32Array(blob);
    const get = (k) => {
      const { offset, shape } = cfg.policy.manifest[k];
      const n = shape.reduce((a, b) => a * b, 1);
      return { a: Float64Array.from(f32.subarray(offset, offset + n)), shape };
    };
    this.mean = get("obs_mean").a;
    this.var = get("obs_var").a;
    this.enc = [get("enc_W"), get("enc_b").a];
    this.mem = [get("mem_W"), get("mem_b").a];
    this.layers = [];
    for (let i = 0; `W${i}` in cfg.policy.manifest; i++) {
      this.layers.push([get(`W${i}`), get(`b${i}`).a]);
    }
    this.clip = cfg.policy.obs_clip;
    this.bounds = cfg.policy.bounds.map(Math.fround);
    this.hidden = this.enc[0].shape[0];
    this.reset();
  }

  reset() { this.h = new Float64Array(this.hidden); }

  static matvec(W, x, b) {
    const [rows, cols] = W.shape, out = new Float64Array(rows);
    for (let r = 0; r < rows; r++) {
      let s = 0.0;
      const o = r * cols;
      for (let c = 0; c < cols; c++) s += W.a[o + c] * x[c];
      out[r] = s + b[r];
    }
    return out;
  }

  // The network's mean, advancing the memory one step.
  meanOf(obs) {
    const n = obs.length, xn = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      xn[i] = clip((obs[i] - this.mean[i]) / Math.sqrt(this.var[i] + 1e-8), -this.clip, this.clip);
    }
    const x = Policy.matvec(this.enc[0], xn, this.enc[1]);
    const m = Policy.matvec(this.mem[0], x, this.mem[1]);
    const H = this.hidden, y = new Float64Array(H);
    for (let i = 0; i < H; i++) {
      const c = m[i], z = sigmoid(m[H + i]);
      const ht = c >= 0 ? c + 0.5 : sigmoid(c);
      this.h[i] = (1.0 - z) * this.h[i] + z * ht;
      const s = sigmoid(m[2 * H + i]);
      y[i] = s * this.h[i] + (1.0 - s) * x[i];
    }
    let out = y;
    const L = this.layers;
    for (let k = 0; k < L.length; k++) {
      out = Policy.matvec(L[k][0], out, L[k][1]);
      if (k < L.length - 1) out = out.map(Math.tanh);
    }
    return out;
  }

  // The NORMALIZED action, as per_command.py hands it to the env:
  // scale_action(clip(mean)) / bounds.
  act(obs) {
    const m = this.meanOf(obs);
    return Array.from(m, (v, i) => (clip(v, -1.0, 1.0) * this.bounds[i]) / this.bounds[i]);
  }
}

// control/general_env.GeneralEnv: reset, step, _obs -- plus what teleop adds
// on top for the page: the righting crank held or swung, lie down / get up
// (sit.js), the shove and the ball.
//
// Two ways to run it. `step(action)` is one whole control step with a given
// action, exactly the env's; check.mjs drives it. `advance(n)` runs n PHYSICS
// steps, querying the policy itself at each control-step boundary; the page
// calls it every frame with however many substeps that frame's wall time
// covers, so what is drawn is the physics at that instant rather than a pose
// that only moves at 50 Hz (which, against a 60 Hz display, holds every fifth
// frame and reads as a judder at speed).
export class BikeSim {
  constructor(mj, model, cfg, policyBlob) {
    this.mj = mj;
    this.m = model;
    this.cfg = cfg;
    this.d = new mj.MjData(model);
    this.fresh = true;
    this.policy = new Policy(cfg, policyBlob);
    this.drive = new Drivetrain(cfg);
    this.gearbox = cfg.gearbox.map((g) => new Gearbox(g, model));
    this.crank = new CrankServo(cfg.crank, model);
    this.odo = new Odometry(cfg);
    this.sit = new SwingSit(cfg.crank.deploy, cfg.crank.recover_deg, cfg.crank.handoff_rate);
    this.sj = cfg.joints.steer_joint.qpos;
    this.sd = cfg.joints.steer_joint.dof;
    this.vCmdW = [0.0, 0.0];
    this.psiCmd = 0.0;
    this.onEngage = null;     // called when the policy takes the bike back
    this.pushDir = 1.0;
    this.peltSide = 1.0;
  }

  // GeneralEnv.reset with randomisation off. `phase` is the detent's; the
  // env draws it from its rng, so a caller replaying a trace passes the
  // trace's, and the page draws its own. `ball`: put the ball out in front
  // (the page) or park it off-scene, as the env does (a trace replay).
  reset(phase = Math.random() * (this.cfg.drivetrain.detent?.period ?? 0), { ball = true } = {}) {
    const { mj, m, cfg } = this;
    if (!this.fresh) mj.mj_resetData(m, this.d);
    this.fresh = false;
    const d = this.d, q = d.qpos, v = d.qvel;
    cfg.eq_qpos.forEach((x, i) => { q[i] = x; });
    v.fill(0.0);
    d.act.fill(0.0);
    d.qacc_warmstart.fill(0.0);
    const a = cfg.init_roll;
    q[3] = Math.cos(a / 2); q[4] = Math.sin(a / 2); q[5] = 0.0; q[6] = 0.0;
    mj.mj_forward(m, d);
    this.odo.reset();
    this.psi = chassisState(d).yaw;
    this.rawPrev = this.psi;
    this.parkBall();
    this.ballLive = ball;
    if (ball) this.placeBall();
    this.steer = q[this.sj];
    this.prevA = [0.0, 0.0, 0.0];
    this.vCmdW = [0.0, 0.0];
    this.psiCmd = this.psi;
    for (const g of this.gearbox) g.reset(d);
    this.crank.goalCurrent = this.crank.goalCurrentBase;
    this.crank.reset(d);
    this.drive.reset(d, phase);
    this.policy.reset();
    this.t = 0.0;
    this.k = 0;                 // substeps into the current control step
    this.fell = false;
    this.sit.reset();
    this.policyOff = false;
    this.heldSteer = null;
    this.crankGoal = 0.0;       // the operator's (Z/C); the sit owns it while active
    this.wingPos = 0;
    this.pushLeft = 0;
    this.obs = this.observe();
    return this.obs;
  }

  observe() {
    const d = this.d, s = chassisState(d);
    const [oLon, oLat] = this.odo.update(d, 0.0);
    const c = Math.cos(this.psi), sn = Math.sin(this.psi);
    const vCl = c * this.vCmdW[0] + sn * this.vCmdW[1];
    const vCt = -sn * this.vCmdW[0] + c * this.vCmdW[1];
    const psiErr = wrapPi(this.psiCmd - this.psi);
    const steer = d.qpos[this.sj];
    const pa = this.prevA;
    return [
      s.roll, s.roll_rate, s.yaw_rate,
      Math.sin(2 * steer), Math.cos(2 * steer), d.qvel[this.sd],
      oLon, oLat, vCl, vCt, Math.sin(psiErr), Math.cos(psiErr),
      pa[0], pa[1], pa[2], s.pitch, s.pitch_rate,
    ].map(Math.fround);
  }

  // The env's action -> controls, at the start of a control step.
  applyAction(action) {
    const { d, cfg } = this;
    const a = action.map(Math.fround);
    const b = cfg.bounds;
    const steerRate = clip(a[0], -1, 1) * b[0];
    const hub = clip(a[1], -1, 1) * b[1];
    const diff = clip(a[2], -1, 1) * b[2];
    const meas = d.qpos[this.sj], lead = cfg.steer_lead_max;
    this.steer = Math.min(Math.max(this.steer + steerRate * cfg.ctrl_dt, meas - lead), meas + lead);
    const common = hub / cfg.r_rear;
    const ctrl = d.ctrl;
    ctrl[cfg.actuators.drive_a] = common + diff / 2;
    ctrl[cfg.actuators.drive_b] = common - diff / 2;
    ctrl[cfg.actuators.steer] = this.steer;
    this.lastA = a;
    this.clearForce();
  }

  clearForce() {
    const xf = this.d.xfrc_applied, cb = this.cfg.chassis_body * 6;
    for (let i = 0; i < 6; i++) xf[cb + i] = 0.0;
  }

  // One physics step with every hook, in GeneralEnv's order.
  substep() {
    const { mj, m, d, cfg } = this;
    if (this.pushLeft > 0) {
      // The shove: xfrc_applied persists, so it is written every step and
      // cleared on the one the pulse ends.
      this.pushLeft -= 1;
      const f = this.pushLeft ? this.pushDir * this.pushForce : 0.0;
      const xf = d.xfrc_applied, cb = cfg.chassis_body * 6;
      xf[cb] = -Math.sin(this.psi) * f;
      xf[cb + 1] = Math.cos(this.psi) * f;
    }
    if (this.ballLive && cfg.ball.drag_tau > 0) {
      // Air drag on the ball alone, linear in its speed: a light ball that a
      // kick sends off and the air slows, rather than the floor alone.
      // MuJoCo's own fluid model is global (opt.density), which would drag
      // the bike too. Off while the ball is parked, so a trace replay is the
      // env's physics exactly.
      const b = cfg.ball, v = d.qvel, xf = d.xfrc_applied, o = 6 * b.body;
      const c = b.mass / b.drag_tau;
      for (let i = 0; i < 3; i++) xf[o + i] = -c * v[b.dadr + i];
    }
    this.drive.preStep(d);
    for (const g of this.gearbox) g.preStep(d);
    this.crank.preStep(d);
    mj.mj_step(m, d);
    this.odo.update(d, cfg.timestep);
  }

  // The end of a control step: heading, observation.
  finish() {
    const { d, cfg } = this;
    const s = chassisState(d);
    this.psi += Math.atan2(Math.sin(s.yaw - this.rawPrev), Math.cos(s.yaw - this.rawPrev));
    this.rawPrev = s.yaw;
    this.prevA = this.lastA;
    this.t += cfg.ctrl_dt;
    this.fell = Math.abs(s.roll) > cfg.fall_roll;
    this.obs = this.observe();
  }

  // One whole control step with a normalized action in [-1, 1]^3: the env's.
  step(action) {
    this.applyAction(action);
    for (let k = 0; k < this.cfg.substeps; k++) this.substep();
    this.finish();
    return this.obs;
  }

  // -- the page's loop ------------------------------------------------------

  advance(n) {
    for (let i = 0; i < n; i++) {
      if (this.k === 0) this.beginControl();
      this.substep();
      if (++this.k === this.cfg.substeps) { this.finish(); this.k = 0; }
    }
  }

  // A control-step boundary: lie down / get up first (teleop's sit_command),
  // then the policy, or the drives held while the sequence owns the bike.
  beginControl() {
    const { d, cfg, sit } = this;
    const s = chassisState(d);
    const crankQ = d.qpos[cfg.crank.qadr];
    let c = null;
    if (sit.phase === "riding" && sit.fallen(this.t, s.roll, s.roll_rate)) {
      sit.getUp(this.t, s.roll);
    }
    if (sit.active || sit.phase === "lying") {
      c = sit.step(this.t, s.roll, s.roll_rate, crankQ, Math.hypot(this.obs[6], this.obs[7]));
      this.crankGoal = c.crank;
      this.crank.goalCurrent = Math.round(this.crank.goalCurrentBase * c.current);
    } else if (this.crank.goalCurrent !== this.crank.goalCurrentBase) {
      this.crank.goalCurrent = this.crank.goalCurrentBase;  // just finished
      this.crankGoal = 0.0;
      this.wingPos = 0;
    }
    d.ctrl[cfg.crank.aid] = this.crankGoal;
    if (c === null || c.policy) {
      if (this.policyOff) this.engage();
      this.policyOff = false;
      this.heldSteer = null;
      this.applyAction(this.policy.act(this.obs));
      return;
    }
    // Policy off: drives at zero speed plus the sequence's differential, the
    // steer held where it was when the policy let go.
    this.policyOff = true;
    if (this.heldSteer === null) this.heldSteer = d.ctrl[cfg.actuators.steer];
    d.ctrl[cfg.actuators.drive_a] = c.diff / 2;
    d.ctrl[cfg.actuators.drive_b] = -c.diff / 2;
    d.ctrl[cfg.actuators.steer] = this.heldSteer;
    this.lastA = [0.0, 0.0, 0.0];
    this.clearForce();
  }

  // drive.engage_general: the policy starts from a clean slate -- empty
  // memory, no previous action, the steer integrator on the wheel, and a
  // command that holds where the bike points.
  engage() {
    this.policy.reset();
    this.prevA = [0.0, 0.0, 0.0];
    this.steer = this.d.qpos[this.sj];
    this.vCmdW = [0.0, 0.0];
    this.psiCmd = this.psi;
    this.obs = this.observe();
    if (this.onEngage) this.onEngage(this.psi);
  }

  // -- the operator's buttons ----------------------------------------------

  get busy() { return this.sit.active || this.sit.phase === "lying"; }

  // Teleop's `.`: a sideways pulse on the chassis, alternating sides --
  // across the BIKE, where teleop pushes along world y. `gentle` is the
  // nudge the policy catches; otherwise one it does not (export.py).
  shove(gentle = false) {
    const s = this.cfg.shove;
    this.pushDir = -this.pushDir;
    this.pushForce = gentle ? s.nudge : s.force;
    this.pushLeft = Math.round(s.seconds / this.cfg.timestep);
    this.pushTotal = this.pushLeft;
  }

  // The shove in progress, for drawing: the WORLD direction it pushes, its
  // force and how much of the pulse is left (1 -> 0); null when none is.
  get push() {
    if (this.pushLeft <= 0) return null;
    const f = this.pushDir;
    return { dir: [-Math.sin(this.psi) * f, Math.cos(this.psi) * f],
             force: this.pushForce, left: this.pushLeft / this.pushTotal };
  }

  // Step the blades left (-1) or right (+1) through left / centre / right,
  // to `crank.blade`, short of the floor. Ignored while lying down or
  // getting up, which own the crank.
  wing(dir) {
    if (this.busy) return;
    this.wingPos = Math.max(-1, Math.min(1, this.wingPos + dir));
    this.crankGoal = this.wingPos * this.cfg.crank.blade;
  }

  // Lie down when riding; get up when lying (or down after a fall).
  sitToggle() {
    const s = chassisState(this.d);
    if (this.sit.active) return;
    if (this.sit.phase === "riding" && Math.abs(s.roll) < 45 * Math.PI / 180) {
      this.vCmdW = [0.0, 0.0];
      this.sit.lieDown(this.t, s.roll);
      return "down";
    }
    this.sit.getUp(this.t, s.roll);
    return "up";
  }

  parkBall() {
    const b = this.cfg.ball, q = this.d.qpos, v = this.d.qvel;
    q[b.qadr] = b.parked[0]; q[b.qadr + 1] = b.parked[1]; q[b.qadr + 2] = b.parked_z;
    q[b.qadr + 3] = 1; q[b.qadr + 4] = 0; q[b.qadr + 5] = 0; q[b.qadr + 6] = 0;
    for (let i = 0; i < 6; i++) v[b.dadr + i] = 0.0;
  }

  // At its `start` (bike frame), at rest: there from the first frame, for
  // B to throw.
  placeBall() {
    const b = this.cfg.ball, q = this.d.qpos, [x, y] = b.start;
    const c = Math.cos(this.psi), s = Math.sin(this.psi);
    q[b.qadr] = q[0] + c * x - s * y;
    q[b.qadr + 1] = q[1] + s * x + c * y;
    q[b.qadr + 2] = b.radius;
  }

  // run_drive._pelt_ball: roll the ball into the bike's flank, alternating
  // sides, aimed somewhere along it.
  pelt() {
    const { d, cfg } = this, b = cfg.ball;
    this.peltSide = -this.peltSide;
    const cb = cfg.chassis_body, x = d.xpos, R = d.xmat;
    const o = [x[3 * cb], x[3 * cb + 1]];
    const flat = (u) => { const n = Math.hypot(u[0], u[1]) || 1e-9; return [u[0] / n, u[1] / n]; };
    const fwd = flat([R[9 * cb], R[9 * cb + 3]]);
    const lat = flat([this.peltSide * R[9 * cb + 1], this.peltSide * R[9 * cb + 4]]);
    const along = -0.02 + 0.2 * Math.random();
    const tx = o[0] + along * fwd[0], ty = o[1] + along * fwd[1];
    const q = d.qpos, v = d.qvel;
    q[b.qadr] = tx + b.pelt_standoff * lat[0];
    q[b.qadr + 1] = ty + b.pelt_standoff * lat[1];
    q[b.qadr + 2] = b.radius;
    q[b.qadr + 3] = 1; q[b.qadr + 4] = 0; q[b.qadr + 5] = 0; q[b.qadr + 6] = 0;
    v[b.dadr] = -b.pelt_speed * lat[0];
    v[b.dadr + 1] = -b.pelt_speed * lat[1];
    for (let i = 2; i < 6; i++) v[b.dadr + i] = 0.0;
  }
}
