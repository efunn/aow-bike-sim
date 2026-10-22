"""What is a MinGRU policy's memory DOING? Its internals over the eval grid,
and the same grid with the memory cut.

    python analysis/policy/mingru_memory.py general_rl_v3_mingru_s0 general_puffer_fixes_52M

The layer (control/policy.py, MinGRUPolicy; recurrent_ppo.py has the why):

    x  = encoder(obs)                 128 units
    z  = sigmoid(g(x))                per unit: how much of the new candidate to take
    h  = (1 - z) h_prev + z h~(x)     an input-gated moving average
    out = s h + (1 - s) x,  s = sigmoid(p(x))     highway: x can bypass h

so a unit remembers only if z is small (time constant ~ dt / z) AND its
output reads h rather than x (s near 1). Reported per policy:

  - z and s over every step of the grid; the time constant -dt/ln(1 - z) of
    each unit at its mean z; how much of the head's input comes through h.
  - The grid three ways: as trained; NO GATE (z = 1, h = h~: the memory
    replaced by the current candidate at full weight -- the clean cut, the
    output keeps its scale); and NO CARRY (h_prev = 0 every step, so h = z h~
    -- also shrinks h by z, ~0.4, so it overstates what memory is worth). If
    NO GATE costs nothing, the policy is not using its memory.

Eval: train_general_rl's own grid and eval env (`_eval_cfg`: randomization
off, 5 s episodes) on config/rl/rl_general_v3.yaml's env -- the env both the
CPU MinGRU arm and the GPU port train on. One episode per command, so a
single-command difference is one episode.
"""
from __future__ import annotations

import argparse

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.flick_spec import scale_action
from aow_sim.control.policy import MinGRUPolicy, _sigmoid, load_policy_npz
from aow_sim.drivetrain_model import from_env_config
from aow_sim.control.general_spec import OBS_NAMES
from aow_sim.eval_general import FAMILIES, _eval_cfg, _eval_episodes, eval_cmds


class Probe:
    """Wraps a MinGRUPolicy: one `mean` step with an optional cut, recording
    z, s and the two halves of the head's input."""

    def __init__(self, pol: MinGRUPolicy, mode: str = "normal"):
        self.pol, self.mode, self.rec = pol, mode, []
        self.ctx = []                       # (episode, step, raw obs) per step
        self.ep, self.t = -1, 0

    def reset(self):
        self.pol.reset()
        self.ep, self.t = self.ep + 1, 0

    def mean(self, obs):
        p = self.pol
        x = p.enc[0] @ p._normalize(obs) + p.enc[1]
        c, g, q = np.split(p.mem[0] @ x + p.mem[1], 3)
        z = _sigmoid(g)
        h_tilde = np.where(c >= 0, c + 0.5, _sigmoid(c))
        if self.mode == "no_gate":
            z = np.ones_like(z)
        h_prev = np.zeros_like(p.h) if self.mode == "no_carry" else p.h
        p.h = (1.0 - z) * h_prev + z * h_tilde
        s = _sigmoid(q)
        self.rec.append((z, s, s * p.h, (1.0 - s) * x))
        self.ctx.append((self.ep, self.t, np.asarray(obs, float).copy()))
        self.t += 1
        return p._head(s * p.h + (1.0 - s) * x)


def run(name: str, mode: str, params, cfg):
    pol = load_policy_npz(f"moves/{name}.npz")
    if not isinstance(pol, MinGRUPolicy):
        raise SystemExit(f"{name} has no memory")
    probe = Probe(pol, mode)
    env = GeneralEnv(params, _eval_cfg(cfg))
    sc = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max, pol.bounds.diff_max])
    sc = np.where(sc == 0, 1, sc)

    def fn(obs):
        # MLPPolicy.action, on the probed mean: clip, scale to physical units;
        # then into the env's normalised action, as train_general_rl._eval
        a = scale_action(np.clip(probe.mean(obs), -1.0, 1.0), pol.bounds)
        return np.asarray(a, float)[:3] / sc

    fn.reset = probe.reset
    m, _rows = _eval_episodes(env, fn, eval_cmds(cfg["env"]["v_max"]))
    return m, probe, env.ctrl_dt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("moves", nargs="+")
    ap.add_argument("--config", default="config/rl/rl_general_v3.yaml")
    a = ap.parse_args()
    cfg = _load_rl_config(a.config)
    params = from_env_config(load_params(), cfg["env"])
    for name in a.moves:
        print(f"\n{name}")
        res = {mode: run(name, mode, params, cfg) for mode in ("normal", "no_carry", "no_gate")}
        _m, probe, dt = res["normal"]
        rec = probe.rec
        z = np.array([r[0] for r in rec])
        s = np.array([r[1] for r in rec])
        hm = np.array([np.linalg.norm(r[2]) for r in rec])
        xm = np.array([np.linalg.norm(r[3]) for r in rec])
        zbar = z.mean(0)
        tau = -dt / np.log(np.clip(1.0 - zbar, 1e-9, 1 - 1e-9))
        sbar = s.mean(0)
        z5 = np.quantile(z, 0.05, axis=0)
        tau5 = -dt / np.log(np.clip(1.0 - z5, 1e-9, 1 - 1e-9))
        print(f"  {len(rec)} steps. per-unit mean z: median {np.median(zbar):.2f}, "
              f"p10 {np.quantile(zbar, .1):.2f}, p90 {np.quantile(zbar, .9):.2f}")
        print(f"  time constant at mean z [s]: median {np.median(tau):.3f}, p90 {np.quantile(tau, .9):.3f}, "
              f"max {tau.max():.2f};  units > 0.1 s: {int((tau > 0.1).sum())}, > 0.5 s: {int((tau > 0.5).sum())}  (dt {dt} s)")
        print(f"  at each unit's SLOWEST 5 % of steps (p5 z): tau median {np.median(tau5):.3f} s, "
              f"max {tau5.max():.2f} s; units > 0.5 s: {int((tau5 > 0.5).sum())}")
        print(f"  per-unit mean s (output reads h): median {np.median(sbar):.2f}, p10 {np.quantile(sbar, .1):.2f}, "
              f"p90 {np.quantile(sbar, .9):.2f}")
        slow = tau > 0.1
        if slow.any():
            print(f"  the slow units (tau > 0.1 s): mean s {sbar[slow].mean():.2f}")
        print(f"  head input through h: {np.mean(hm / (hm + xm)):.0%} of |s h| + |(1-s) x| (mean over steps)")
        latch_states(probe, dt, eval_cmds(cfg["env"]["v_max"]))
        print(f"  {'variant':10s} surv  track  vel_err  head_deg  drift_m")
        for mode, (m, _r, _d) in res.items():
            print(f"  {mode:10s} {m['survive_rate']:.2f}  "
                  f"{m['track']:.3f}  {m['vel_err']:.3f}  {m['head_err_deg']:7.1f}  {m['drift_m']:.3f}")


LATCH_TAU_S = 0.5


def latch_states(probe, dt, cmds, top=3):
    """WHEN do the gates close? A unit is LATCHED on a step when its gate is
    small enough that its time constant there exceeds LATCH_TAU_S
    (z < 1 - exp(-dt / tau)). Over the units that ever latch (p5 z under that
    bar), report how many are latched by command family, time since the
    episode began, and a few observed quantities -- and, per unit, the
    observations its log z tracks most closely (rank correlation)."""
    z = np.array([r[0] for r in probe.rec])
    ep = np.array([c[0] for c in probe.ctx])
    t = np.array([c[1] for c in probe.ctx]) * dt
    obs = np.array([c[2] for c in probe.ctx])
    zbar = 1.0 - np.exp(-dt / LATCH_TAU_S)
    units = np.flatnonzero(np.quantile(z, 0.05, axis=0) < zbar)
    if not len(units):
        print("  no unit ever latches")
        return
    lat = z[:, units] < zbar                     # steps x latching units
    n = lat.sum(1)
    print(f"  LATCHING: {len(units)} units ever latch (tau > {LATCH_TAU_S} s); "
          f"per step, mean {n.mean():.1f} latched, max {n.max()}")
    fam_of = []
    for v_lon, v_lat, dpsi in cmds:
        c = (v_lon, v_lat, float(np.degrees(dpsi)))
        fam_of.append(next(f for f, pred in FAMILIES.items() if pred(c)))
    fam = np.array([fam_of[e] for e in ep])
    print("    by family:   " + "  ".join(
        f"{f} {n[fam == f].mean():.1f}" for f in FAMILIES if (fam == f).any()))
    edges = [0, 0.1, 0.5, 1.0, 2.0, 5.01]
    print("    by time since reset [s]:  " + "  ".join(
        f"{a:g}-{b:g}: {n[(t >= a) & (t < b)].mean():.1f}"
        for a, b in zip(edges[:-1], edges[1:]) if ((t >= a) & (t < b)).any()))
    names = list(OBS_NAMES[:obs.shape[1]])
    ix = {k: names.index(k) for k in ("roll", "roll_rate", "yaw_rate", "v_lon")}
    for k, e in (("|v_lon| m/s", [0, 0.05, 0.2, 0.5, 2]),
                 ("|roll| rad", [0, 0.01, 0.03, 0.1, 2]),
                 ("|yaw_rate| rad/s", [0, 0.05, 0.3, 1.0, 20])):
        col = np.abs(obs[:, ix[k.split("|")[1]]])
        print(f"    by {k}:  " + "  ".join(
            f"{a:g}-{b:g}: {n[(col >= a) & (col < b)].mean():.1f} ({int(((col >= a) & (col < b)).sum())})"
            for a, b in zip(e[:-1], e[1:]) if ((col >= a) & (col < b)).any()))

    def rank(a):
        return np.argsort(np.argsort(a, axis=0), axis=0).astype(float)
    ro, rz = rank(obs), rank(np.log(z[:, units]))
    ro -= ro.mean(0)
    rz -= rz.mean(0)
    cor = (rz.T @ ro) / np.outer(np.linalg.norm(rz, axis=0), np.linalg.norm(ro, axis=0) + 1e-12)
    tally = {}
    for j in range(len(units)):
        for i in np.argsort(-np.abs(cor[j]))[:1]:
            key = f"{names[i]} ({'+' if cor[j, i] > 0 else '-'})"
            tally[key] = tally.get(key, 0) + 1
    print("    each latching unit's gate tracks most closely (rank corr. of log z; "
          "- = gate closes as the quantity rises): " + ", ".join(
              f"{k} x{v}" for k, v in sorted(tally.items(), key=lambda kv: -kv[1])))
    print(f"    median |rank corr.| of that best match: {np.median(np.abs(cor).max(1)):.2f}")


if __name__ == "__main__":
    main()
