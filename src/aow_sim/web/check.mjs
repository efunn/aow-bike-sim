// Replay a Python trace through the page's own simulation code.
//
//   python -m aow_sim.web.export --trace traces/web_trace.json
//   npm install --prefix src/aow_sim/web          # once: @mujoco/mujoco
//   node src/aow_sim/web/check.mjs traces/web_trace.json
//
// Two comparisons. OPEN LOOP feeds the recorded observations to the JS
// policy, isolating the network port: it should agree to ~1e-12. CLOSED LOOP
// flies the JS plant under the trace's commands with its own actions; it
// starts at float noise and grows, because a balancing bike is chaotic and
// WASM's libm is not the native one. What it should NOT do is jump in the
// first steps, or fall where Python did not -- either means a ported line is
// wrong, not that the floats wandered.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import loadMujoco from "@mujoco/mujoco";
import { BikeSim, Policy } from "./site/js/sim.js";

const here = path.dirname(fileURLToPath(import.meta.url));
const assets = path.join(here, "site", "assets");
const tracePath = process.argv[2];
if (!tracePath) {
  console.error("usage: node check.mjs TRACE.json");
  process.exit(2);
}
const trace = JSON.parse(fs.readFileSync(tracePath, "utf8"));
const cfg = JSON.parse(fs.readFileSync(path.join(assets, "config.json"), "utf8"));
const blob = fs.readFileSync(path.join(assets, "policy.bin"));
const weights = blob.buffer.slice(blob.byteOffset, blob.byteOffset + blob.byteLength);

const pkg = JSON.parse(fs.readFileSync(
  path.join(here, "node_modules", "@mujoco", "mujoco", "package.json"), "utf8"));
if (pkg.version !== cfg.mujoco_version) {
  console.error(`@mujoco/mujoco ${pkg.version} installed, model exported from ` +
                `${cfg.mujoco_version}: an .mjb loads only in its own version`);
  process.exit(2);
}

const mj = await loadMujoco();
const vfs = new mj.MjVFS();
vfs.addBuffer("bike.mjb", new Uint8Array(fs.readFileSync(path.join(assets, "bike.mjb"))));
const model = mj.MjModel.from_binary_path("bike.mjb", vfs);

const maxAbs = (a, b) => a.reduce((m, x, i) => Math.max(m, Math.abs(x - b[i])), 0);
const fmt = (x) => x.toExponential(1).padStart(8);
const steps = trace.steps;

// -- open loop -----------------------------------------------------------
const pol = new Policy(cfg, weights);
let openWorst = 0;
for (const row of steps) openWorst = Math.max(openWorst, maxAbs(pol.act(row.obs), row.action));
console.log(`open loop  policy action vs Python, worst over ${steps.length} steps: ${fmt(openWorst)}`);

// -- closed loop ---------------------------------------------------------
const sim = new BikeSim(mj, model, cfg, weights);
sim.reset(trace.detent_phase, { ball: false });   // parked, as the env parks it
console.log(`reset      heading ${sim.psi.toFixed(6)} vs ${trace.psi0.toFixed(6)}`);
const marks = new Set([0, 1, 2, 5, 10, 25, 50, 100, 200, 350, 500, 699]);
// Chassis pose only (qpos[0:7]): the wheel and roller angles wind up without
// bound, so a whole-qpos difference is a spin count, not a disagreement.
const tols = [1e-9, 1e-6, 1e-3];
const first = tols.map(() => null);
let fellAt = null;
let jump = null;        // where the observations first part, and in what
const t0 = performance.now();
console.log("closed loop          obs      action   chassis  |  bike xy (js)      bike xy (py)");
steps.forEach((row, k) => {
  const dObs = maxAbs(sim.obs, row.obs);
  if (jump === null && dObs > 1e-3) {
    const i = sim.obs.findIndex((x, j) => Math.abs(x - row.obs[j]) > 1e-3);
    jump = `step ${k}, obs[${i}] ${cfg.obs_layout?.[i] ?? ""} ${sim.obs[i].toFixed(5)} vs ${row.obs[i].toFixed(5)}`;
  }
  const a = sim.policy.act(sim.obs);
  const dAct = maxAbs(a, row.action);
  sim.vCmdW = row.v_cmd_w;
  sim.psiCmd = row.psi_cmd;
  sim.step(a);
  const q = sim.d.qpos;
  const dq = maxAbs(q.slice(0, 7), row.qpos);
  tols.forEach((tol, i) => { if (first[i] === null && dq > tol) first[i] = k; });
  if (sim.fell && fellAt === null) fellAt = k;
  if (marks.has(k)) {
    console.log(`  step ${String(k).padStart(3)} t=${(k * cfg.ctrl_dt).toFixed(2).padStart(5)}` +
                `  ${fmt(dObs)}  ${fmt(dAct)}  ${fmt(dq)}  |  ` +
                `${q[0].toFixed(3).padStart(6)} ${q[1].toFixed(3).padStart(6)}    ` +
                `${row.qpos[0].toFixed(3).padStart(6)} ${row.qpos[1].toFixed(3).padStart(6)}`);
  }
});
const ms = performance.now() - t0;

// -- streaming -----------------------------------------------------------
// The page calls advance(n) with however many substeps a frame covers. That
// must be the SAME physics as whole control steps, bit for bit: run both
// from one reset under the trace's commands and compare.
const whole = new BikeSim(mj, model, cfg, weights);
const chunked = new BikeSim(mj, model, cfg, weights);
whole.reset(trace.detent_phase, { ball: false });
chunked.reset(trace.detent_phase, { ball: false });
let left = 0, streamWorst = 0;
const sub = cfg.substeps;
steps.forEach((row) => {
  whole.vCmdW = row.v_cmd_w; whole.psiCmd = row.psi_cmd;
  whole.step(whole.policy.act(whole.obs));
  chunked.vCmdW = row.v_cmd_w; chunked.psiCmd = row.psi_cmd;
  // Uneven chunks that straddle control boundaries, as frames do.
  let todo = sub;
  while (todo > 0) {
    if (left === 0) left = 1 + Math.floor(Math.random() * 90);
    const n = Math.min(left, todo);
    chunked.advance(n);
    left -= n; todo -= n;
  }
  streamWorst = Math.max(streamWorst, maxAbs(chunked.d.qpos, whole.d.qpos));
});
console.log("first step the chassis pose differs by " +
            tols.map((tol, i) => `>${tol.toExponential(0)}: ${first[i] ?? "never"}`).join(", "));
console.log(`first observation to differ by >1e-3: ${jump ?? "none"}`);
console.log(`fell: js ${fellAt === null ? "no" : `step ${fellAt}`}, python ` +
            `${steps.some((r) => r.fell) ? "yes" : "no"}`);
console.log(`streaming  advance() in random chunks vs whole steps, worst qpos: ${fmt(streamWorst)}`);
console.log(`${steps.length} control steps (${(steps.length * cfg.ctrl_dt).toFixed(1)} s simulated) ` +
            `in ${(ms / 1000).toFixed(2)} s wall: ${(steps.length * cfg.ctrl_dt * 1000 / ms).toFixed(1)}x real time`);
