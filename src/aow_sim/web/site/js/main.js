// Load MuJoCo, the bike and the policy; step the physics on wall time and
// draw every frame.

import { BikeSim, chassisState } from "./sim.js";
import { Teleop } from "./teleop.js";
import { Viewer } from "./viewer.js";

const $ = (id) => document.getElementById(id);
const status = (text) => { $("status").textContent = text; };
const params = new URLSearchParams(location.search);
if (params.get("hud") === "0") document.body.classList.add("no-hud");
// Telemetry is off for the demo; T or ?telemetry shows it.
const showTelemetry = (on) => { $("hud").hidden = !on; };

// At most this much simulated time per frame: a slow frame runs the bike in
// slow motion instead of queueing up physics it can never catch.
const MAX_FRAME_S = 0.06;

const DEG = 180 / Math.PI;

async function fetchBytes(url, label) {
  status(`Loading ${label}…`);
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.arrayBuffer();
}

async function boot() {
  const cfg = await (await fetch("assets/config.json")).json();
  // The .mjb loads only in the MuJoCo it was saved from: config.json names it.
  status("Loading physics engine…");
  const { default: loadMujoco } = await import(
    `https://cdn.jsdelivr.net/npm/@mujoco/mujoco@${cfg.mujoco_version}/mujoco.js`);
  const [mj, mjb, weights] = await Promise.all([
    loadMujoco(),
    fetchBytes("assets/bike.mjb", "bike"),
    fetchBytes("assets/policy.bin", "policy"),
  ]);
  const vfs = new mj.MjVFS();
  vfs.addBuffer("bike.mjb", new Uint8Array(mjb));
  const model = mj.MjModel.from_binary_path("bike.mjb", vfs);

  const sim = new BikeSim(mj, model, cfg, weights);
  const teleop = new Teleop(cfg.v_max);
  const viewer = new Viewer($("view"), model, cfg);
  new ResizeObserver(() => viewer.resize()).observe($("view"));
  // The policy takes the bike back after a get-up: command where it points.
  sim.onEngage = (psi) => teleop.reset(psi);
  if (params.has("debug")) Object.assign(window, { sim, teleop, viewer, cfg });

  const reset = () => {
    sim.reset();
    teleop.reset(sim.psi);
    viewer.recentre([sim.d.qpos[0], sim.d.qpos[1]]);
  };
  reset();

  // -- input ---------------------------------------------------------------
  const HOLD = {
    ArrowUp: "up", KeyW: "up", ArrowDown: "down", KeyS: "down",
    ArrowLeft: "left", KeyA: "left", ArrowRight: "right", KeyD: "right",
  };
  const ACTIONS = {
    snapLeft: () => teleop.snap(1, sim.psi),
    snapRight: () => teleop.snap(-1, sim.psi),
    shove: () => sim.shove(false),
    nudge: () => sim.shove(true),
    wingLeft: () => sim.wing(-1),
    wingRight: () => sim.wing(1),
    ball: () => sim.pelt(),
    sit: () => { if (sim.sitToggle() === "down") teleop.reset(sim.psi); },
    telemetry: () => showTelemetry($("hud").hidden),
    reset,
  };
  const TAP = {
    KeyQ: "snapLeft", KeyE: "snapRight", KeyZ: "wingLeft", KeyC: "wingRight",
    KeyB: "ball", KeyX: "sit", KeyR: "reset", KeyT: "telemetry",
  };
  addEventListener("keydown", (e) => {
    if (e.metaKey || e.ctrlKey || e.altKey) return;
    const h = HOLD[e.code];
    if (h) { teleop.press(h); e.preventDefault(); return; }
    if (e.repeat) return;
    if (e.code === "Space") { ACTIONS[e.shiftKey ? "nudge" : "shove"](); e.preventDefault(); return; }
    const t = TAP[e.code];
    if (t) { ACTIONS[t](); e.preventDefault(); }
  });
  addEventListener("keyup", (e) => { const h = HOLD[e.code]; if (h) teleop.release(h); });
  // A key released while the frame had no focus never sends keyup: let go
  // of everything rather than drive on.
  addEventListener("blur", () => { teleop.held.clear(); updateFocus(); });
  addEventListener("focus", updateFocus);
  function updateFocus() { document.body.classList.toggle("focused", document.hasFocus()); }
  updateFocus();

  for (const b of document.querySelectorAll("[data-hold]")) {
    const name = b.dataset.hold;
    b.addEventListener("pointerdown", (e) => {
      teleop.press(name);
      try { b.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
    });
    for (const ev of ["pointerup", "pointercancel"]) b.addEventListener(ev, () => teleop.release(name));
  }
  for (const b of document.querySelectorAll("[data-action]")) {
    b.addEventListener("pointerdown", (e) => { e.preventDefault(); ACTIONS[b.dataset.action](); });
  }

  // -- loop ----------------------------------------------------------------
  // Physics runs in whole substeps on wall time; the policy is queried by the
  // sim itself every `substeps` of them. Drawing the state after each frame's
  // substeps shows the bike where it is now, not where it was at the last
  // 50 Hz control tick.
  let last = performance.now(), acc = 0.0, slow = false;
  let physMs = 0, simS = 0, wallS = 0, frames = 0;     // since the last readout
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    teleop.update(dt, sim.psi);
    if (!sim.busy && !sim.policyOff) {
      sim.vCmdW = teleop.vCmdWorld();
      sim.psiCmd = teleop.psi;
    }
    acc += Math.min(dt, MAX_FRAME_S);
    slow = dt > MAX_FRAME_S;
    const n = Math.floor(acc / cfg.timestep);
    acc -= n * cfg.timestep;
    const t0 = performance.now();
    sim.advance(n);
    physMs += performance.now() - t0;
    simS += n * cfg.timestep;
    wallS += dt;
    frames++;
    const q = sim.d.qpos, cb = cfg.chassis_body, xi = sim.d.xipos;
    viewer.draw(sim.d, [q[0], q[1]], sim.psiCmd, dt, {
      busy: sim.busy, push: sim.push, com: [xi[3 * cb], xi[3 * cb + 1], xi[3 * cb + 2]] });
    hud(slow);
    requestAnimationFrame(frame);
  }

  let hudAt = 0;
  function hud(slowNow) {
    const now = performance.now();
    if ($("hud").hidden || now - hudAt < 250) return;
    hudAt = now;
    const s = chassisState(sim.d), o = sim.obs;
    const lead = Math.atan2(Math.sin(sim.psiCmd - sim.psi), Math.cos(sim.psiCmd - sim.psi));
    const set = (id, text) => { $(id).textContent = text; };
    set("hud-cmd", `${teleop.v.toFixed(2)} m/s`);
    set("hud-speed", `${s.v_lon.toFixed(2)} m/s`);
    set("hud-est", `${o[6].toFixed(2)} / ${o[7].toFixed(2)} m/s`);
    set("hud-lead", `${(lead * DEG).toFixed(0)}°${teleop.armed ? "" : " (snap)"}`);
    set("hud-roll", `${(s.roll * DEG).toFixed(1)}°`);
    set("hud-pitch", `${(s.pitch * DEG).toFixed(1)}°`);
    // The steer is multi-turn, and the wheel is the same either way round:
    // 180 deg on the servo is straight ahead. So the readout is mod 180, +-90.
    const st = sim.d.qpos[sim.sj];
    set("hud-steer", `${Math.round(Math.atan2(Math.sin(2 * st), Math.cos(2 * st)) / 2 * DEG) || 0}°`);
    set("hud-crank", `${(sim.d.qpos[cfg.crank.qadr] * DEG).toFixed(0)}°`);
    set("hud-phase", sim.sit.phase === "riding" ? "drive" : sim.sit.phase);
    set("hud-rate", `${(simS / Math.max(wallS, 1e-9)).toFixed(2)}× · ` +
        `${(physMs / Math.max(frames, 1)).toFixed(1)} ms/frame · ${(frames / Math.max(wallS, 1e-9)).toFixed(0)} fps`);
    physMs = simS = wallS = 0; frames = 0;
    $("hud-slow").hidden = !slowNow;
  }

  showTelemetry(params.has("telemetry"));
  status("");
  document.body.classList.add("ready");
  requestAnimationFrame(frame);
}

boot().catch((err) => {
  console.error(err);
  status(`Could not start the simulation: ${err.message}`);
  document.body.classList.add("failed");
});
