"""The eval grid in GpuGeneralEnv against GeneralEnv (plan, phase 2, the
last check): does a policy do the same thing on each of the 20 commands in
both, not just on average?

GeneralEnv side: `eval_general._eval_episodes` itself, on the eval
config (`_eval_cfg`: randomization off, 5 s, no ball) -- one episode per
command, seeds 10_000 + k, exactly what training's eval and per_command.py
report. GPU side: every command x --seeds worlds at once, each holding its
command for the episode (what the `v_cmd` / `psi_cmd_rel` reset options do),
the metrics rebuilt from device state. The AHRS noise differs per seed, so
the GPU gives a spread per command and the question is whether GeneralEnv's
one episode sits inside it. Different engines (phase 1: contacts differ),
so agreement is behavioural, not numerical.

  python analysis/gpu/phase2/eval_grid.py --seeds 8           # on a GPU
  python analysis/gpu/phase2/eval_grid.py --seeds 1 --steps 20 --no-cpu   # Mac smoke

Metrics as `_eval_episodes` defines them (same names): fell, head_err_med,
vel_err_med (medians over the episode), head_err_tail / vel_err_tail (worst
over the last 2 s), t_head_s (first inside 10 deg, censored at the episode
length), drift_m (distance from the start), v_ach (mean truth v_lon over
the last 2 s). One difference: on a fall the GPU env resets inside the
step, so the falling step itself is not scored (GeneralEnv scores it).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import warp as wp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench_env import WarpMLP                                   # noqa: E402

from aow_sim.control.general_env import _load_rl_config         # noqa: E402
from aow_sim.gpu.env import GpuGeneralEnv                       # noqa: E402
from aow_sim.eval_general import (_HEAD_TOL_DEG, _TAIL_S, _eval_cfg,  # noqa: E402
                                  _eval_episodes, eval_cmds)

CONFIG = "config/rl/rl_general_v3.yaml"
METRICS = ("head_err_med", "vel_err_med", "head_err_tail", "vel_err_tail",
           "t_head_s", "drift_m", "v_ach")


def _wrap(x):
    return np.arctan2(np.sin(x), np.cos(x))


def _yaw(q):                                                    # wxyz, as extract_state
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


def gpu_grid(policy, seeds, steps, seed, device):
    base = _load_rl_config(CONFIG)
    cmds = eval_cmds(base["env"]["v_max"])
    ev = _eval_cfg(base)
    ctrl_dt = 1.0 / ev["env"]["control_rate_hz"]
    T = steps or int(ev["env"]["max_episode_s"] / ctrl_dt)

    def cfg(c):
        c = _eval_cfg(c)
        # half a step past the eval's length, so truncation (and the reset
        # inside the step that comes with it) lands AFTER the last scored step
        c["env"] = {**c["env"], "max_episode_s": (T + 1.5) * ctrl_dt}
        return c

    cw = np.repeat(np.array(cmds), seeds, axis=0)               # (n, 3): v_lon, v_lat, dpsi
    n = len(cw)
    g = GpuGeneralEnv(CONFIG, nworld=n, seed=seed, device=device, cfg_override=cfg)
    st = {}

    def hold_command():                                          # = the reset options
        psi_cmd = g.psi.numpy() + cw[:, 2]
        c, s = np.cos(psi_cmd), np.sin(psi_cmd)
        g.psi_cmd.assign(psi_cmd.astype(np.float32))
        g.v_cmd_w.assign(np.stack([c * cw[:, 0] - s * cw[:, 1],
                                   s * cw[:, 0] + c * cw[:, 1]], 1).astype(np.float32))
        g.next_resample.assign(np.full(n, 10 ** 9, np.int32))
        g.diff.fill_(1.0)
        st["psi_cmd"] = psi_cmd

    g.on_reset = hold_command
    g.reset()
    g.on_reset = None                    # later resets are past the scored episode
    p0 = g.d.qpos.numpy()[:, :2].copy()
    psi_cmd = st["psi_cmd"]
    lon_axis = np.stack([np.cos(psi_cmd), np.sin(psi_cmd)], 1)
    lat_axis = np.stack([-np.sin(psi_cmd), np.cos(psi_cmd)], 1)

    mlp = WarpMLP(policy, n, device)
    act = wp.zeros((n, 3), dtype=float, device=device)

    def one():
        mlp(g.obs, act)
        g.step(act)

    graph = None
    alive = np.ones(n, bool)
    fell = np.zeros(n, bool)
    n_steps = np.zeros(n, int)
    t_head = np.full(n, -1)
    heads, vels, vlon, xy_last = [], [], [], p0.copy()
    t0 = time.perf_counter()
    for k in range(T):
        if graph is not None:
            wp.capture_launch(graph)
        else:
            one()                        # the first step eager: compiles everything
            if wp.get_device(device).is_cuda:
                with wp.ScopedCapture(device=device) as cap:
                    one()
                graph = cap.graph
        qpos, qvel = g.d.qpos.numpy(), g.d.qvel.numpy()
        term = g.terminated.numpy().astype(bool)
        psi = g.psi.numpy()
        newly = alive & term
        fell |= newly
        alive &= ~term                   # a fallen world has already been reset
        yaw = _yaw(qpos[:, 3:7])
        cy, sy = np.cos(yaw), np.sin(yaw)
        v_lon = cy * qvel[:, 0] + sy * qvel[:, 1]
        v_lat = -sy * qvel[:, 0] + cy * qvel[:, 1]
        vc = g.v_cmd_w.numpy()
        cp, sp = np.cos(psi), np.sin(psi)
        v_cl = cp * vc[:, 0] + sp * vc[:, 1]
        v_ct = -sp * vc[:, 0] + cp * vc[:, 1]
        h = np.degrees(np.abs(_wrap(psi_cmd - psi)))
        v = np.hypot(v_cl - v_lon, v_ct - v_lat)
        mask = np.where(alive, 1.0, np.nan)
        heads.append(h * mask)
        vels.append(v * mask)
        vlon.append(v_lon * mask)
        xy_last[alive] = qpos[alive, :2]
        n_steps[alive] = k + 1
        hit = alive & (t_head < 0) & (h < _HEAD_TOL_DEG)
        t_head[hit] = k + 1
    wall = time.perf_counter() - t0

    H, V, VL = np.array(heads), np.array(vels), np.array(vlon)
    tail = max(1, int(_TAIL_S / ctrl_dt))
    rows = []
    for w in range(n):
        m = n_steps[w]
        hw, vw, lw = H[:m, w], V[:m, w], VL[:m, w]
        d = xy_last[w] - p0[w]
        rows.append({
            "cmd": (round(float(cw[w, 0]), 3), round(float(cw[w, 1]), 3),
                    int(round(np.degrees(cw[w, 2])))),
            "fell": bool(fell[w]), "steps": int(m),
            "head_err_med": float(np.median(hw)) if m else 180.0,
            "vel_err_med": float(np.median(vw)) if m else 9.9,
            "head_err_tail": float(np.max(hw[-tail:])) if m else 180.0,
            "vel_err_tail": float(np.max(vw[-tail:])) if m else 9.9,
            "t_head_s": float((t_head[w] if t_head[w] > 0 else m) * ctrl_dt),
            "drift_m": float(np.hypot(*d)),
            "drift_lon": float(d @ lon_axis[w]), "drift_lat": float(d @ lat_axis[w]),
            "v_ach": float(np.mean(lw[-tail:])) if m else 0.0,
        })
    return cmds, rows, wall, T


def cpu_grid(policy, steps):
    from aow_sim.control.general_env import GeneralEnv
    from aow_sim.control.policy import load_policy_npz
    pol = load_policy_npz(policy)
    cfg = _eval_cfg(_load_rl_config(CONFIG))
    if steps:                            # a short smoke run: the same length as the GPU's
        cfg["env"] = {**cfg["env"], "max_episode_s": steps / cfg["env"]["control_rate_hz"]}
    env = GeneralEnv(None, cfg, seed=0)
    t = time.perf_counter()
    _m, rows = _eval_episodes(env, lambda o: np.clip(pol.mean(o), -1, 1), eval_cmds(cfg["env"]["v_max"]))
    return rows, time.perf_counter() - t


def compare(cmds, grows, crows, seeds):
    """Per command: GeneralEnv's value, the GPU seeds' median [min, max], and
    whether the first sits inside the second."""
    out, inside, total = [], {k: 0 for k in METRICS}, 0
    for i, c in enumerate(cmds):
        g = grows[i * seeds:(i + 1) * seeds]
        row = {"cmd": g[0]["cmd"], "gpu_falls": f"{sum(r['fell'] for r in g)}/{seeds}"}
        if crows is not None:
            row["cpu_fell"] = bool(crows[i]["fell"])
        for k in METRICS:
            vals = np.array([r[k] for r in g])
            row[k] = {"gpu_med": round(float(np.median(vals)), 3),
                      "gpu_min": round(float(vals.min()), 3), "gpu_max": round(float(vals.max()), 3)}
            if crows is not None:
                cv = float(crows[i][k])
                row[k]["cpu"] = round(cv, 3)
                ok = vals.min() <= cv <= vals.max()
                row[k]["cpu_inside"] = bool(ok)
                inside[k] += ok
        total += 1
        out.append(row)
    return out, inside, total


def show(table, inside, total, has_cpu):
    hdr = f"{'command':>22} {'falls gpu':>9} {'cpu':>5}"
    for k in ("head_err_med", "vel_err_med", "t_head_s", "drift_m"):
        hdr += f" | {k:>24}"
    print(hdr)
    for r in table:
        line = f"{str(r['cmd']):>22} {r['gpu_falls']:>9} {str(r.get('cpu_fell', '-')):>5}"
        for k in ("head_err_med", "vel_err_med", "t_head_s", "drift_m"):
            m = r[k]
            cpu = f"{m['cpu']:6.2f}{'' if m.get('cpu_inside', True) else '*'}" if "cpu" in m else "     -"
            line += f" | {cpu} vs {m['gpu_med']:6.2f} [{m['gpu_min']:5.2f},{m['gpu_max']:5.2f}]"
        print(line)
    if has_cpu:
        print("GeneralEnv's value inside the GPU seeds' range (* = outside):")
        print("  " + ", ".join(f"{k} {inside[k]}/{total}" for k in METRICS))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="moves/archive/general_rl_v3_s0.npz")
    ap.add_argument("--seeds", type=int, default=8, help="GPU worlds per command")
    ap.add_argument("--steps", type=int, default=0, help="0 = the eval's 5 s")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-cpu", action="store_true", help="skip GeneralEnv")
    ap.add_argument("--out", default="results/eval_grid.json")
    a = ap.parse_args()
    wp.config.quiet = True
    wp.init()
    dev = "cuda:0" if wp.is_cuda_available() else "cpu"
    cmds, grows, gwall, T = gpu_grid(a.policy, a.seeds, a.steps, a.seed, dev)
    print(f"GPU ({wp.get_device(dev).name}): {len(grows)} worlds x {T} steps in {gwall:.1f} s; "
          f"falls {sum(r['fell'] for r in grows)}/{len(grows)}")
    crows = None
    if not a.no_cpu:
        crows, cwall = cpu_grid(a.policy, a.steps)
        print(f"GeneralEnv (_eval_episodes): {len(crows)} episodes in {cwall:.1f} s; "
              f"falls {sum(r['fell'] for r in crows)}/{len(crows)}")
    table, inside, total = compare(cmds, grows, crows, a.seeds)
    show(table, inside, total, crows is not None)
    try:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps({"policy": a.policy, "seeds": a.seeds, "steps": T,
                                           "gpu_wall_s": gwall, "table": table,
                                           "gpu_rows": grows, "cpu_rows": crows}, indent=1,
                                          default=float))
    except OSError:
        pass
