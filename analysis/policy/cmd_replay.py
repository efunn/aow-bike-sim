"""Fly a bike run's joystick commands through a policy in the sim.

    python analysis/policy/cmd_replay.py traces/bike/<stamp>_run
    python analysis/policy/cmd_replay.py <run> --policies general_puffer_5090_v6_s1_80M \
        general_puffer_5090_v5floor_s1_80M
    python analysis/policy/cmd_replay.py <run> --set bike.front_wheel.axle_friction_nm=0 \
        --drop-play rear_axle_lateral                  # one sim variant

The run's command (`bike_cmd_v_world`, `bike_cmd_psi`) is replayed OPEN
LOOP at the policy's 50 Hz, re-anchored so the record's first heading is
the sim's: the operator steered the real bike, not this one, so the two
paths part and nothing here says where the bike went. What it does say is
how a policy answers that SPEED and HEADING profile, which holds while the
commands change over seconds -- drive slowly, in held segments, when you
mean to replay a run. Built 2026-11-02, when v6 drove "too cautious" on the
bike and the eval grid (one held step per command) showed nothing: the
replay reproduced it, and the bike's own numbers sit beside each row.

Speed: along-command speed over |command| (1.0 tracks it), forward and
reverse split on the command's body-frame direction (|lon| > 0.7 |cmd|,
|cmd| > 0.15 m/s). Bike: the odometry's v_world; sim: the TRUE world
velocity. Turns: every commanded heading step of more than 60 deg (the
command moving that far within 1 s), timed to within 15 deg of it, or
"miss" if a newer step comes first or 6 s pass. Bike: the AHRS heading.

The sim is the eval grid's (per_command.py): randomization off, no ball,
`tm151_filter`, counts odometry. A fall ends that policy's replay and is
reported with its time -- a fall the bike did not have is often the open
loop (no operator reacting to the sim bike), not a verdict.

`--set PATH=VALUE` (as run_vs_sim.py) and `--drop-play GROUP` (from the
params AND the move's own `wheel_play`, which env_for would otherwise build
over them) change the sim's bike only, AFTER the policy trained: they ask how the policy copes
with a different plant, not what it would have learned on one.

Read-only: prints a table.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import per_command as pc                                   # noqa: E402
from rsa_policies import env_for, load_general             # noqa: E402
from run_vs_sim import _set                                # noqa: E402
from aow_sim.build_model import load_params                # noqa: E402
from aow_sim.control.steer import wrap_pi                  # noqa: E402

def _wrap(x):
    """wrap_pi for arrays (steer.wrap_pi takes a scalar)."""
    return np.arctan2(np.sin(x), np.cos(x))


TURN_DEG, TURN_WIN_S, TURN_TOL_DEG, TURN_MAX_S = 60.0, 1.0, 15.0, 6.0


def record(run_dir):
    z = np.load(Path(run_dir) / "capture.npz")
    t = z["t_host"] - z["t_host"][0]
    return t, z["bike_cmd_v_world"], z["bike_cmd_psi"], z["bike_v_world"], z["bike_psi"]


def speed_split(c, along, lon):
    """{fwd, rev}: (sum along / sum |cmd|, seconds-worth of samples)."""
    out = {}
    for k, sg in (("fwd", 1), ("rev", -1)):
        s = (c > 0.15) & (sg * lon > 0.7 * c)
        out[k] = (along[s].sum() / c[s].sum() if s.any() else np.nan, s.sum())
    return out


def turns(t, cpsi, psi):
    """Times to settle each big commanded heading step (np.nan = missed)."""
    dt = np.median(np.diff(t)); w = max(1, int(round(TURN_WIN_S / dt)))
    starts, i = [], w
    while i < len(t):
        if abs(_wrap(cpsi[i] - cpsi[i - w])) > np.radians(TURN_DEG):
            # the step's start: where the command began to move
            j = i - w + int(np.argmax(np.abs(np.diff(np.unwrap(cpsi[i - w:i + 1]))) > 1e-3))
            starts.append(j); i += w
        i += 1
    out = []
    for n, j in enumerate(starts):
        end = min(starts[n + 1] if n + 1 < len(starts) else len(t),
                  j + int(TURN_MAX_S / dt))
        err = np.abs(_wrap(psi[j:end] - cpsi[j:end]))
        hit = np.flatnonzero(err[int(TURN_WIN_S / dt):] < np.radians(TURN_TOL_DEG))
        out.append(t[j + int(TURN_WIN_S / dt) + hit[0]] - t[j] if hit.size else np.nan)
    return np.array(out)


def fmt_turns(ts):
    if ts.size == 0:
        return "no turns"
    ok = ts[np.isfinite(ts)]
    med = f"median {np.median(ok):.1f} s" if ok.size else "-"
    return f"{ok.size}/{ts.size} done, {med}"


def replay(name, t, cmd, cpsi, bpsi0, params, drop_play=()):
    pol = load_general(name)
    # A move carries the play it trained with, and env_for builds THAT over
    # the params' (general_spec.policy_env_overrides): dropping a group from
    # the params alone builds the same bike. Drop it from both.
    if pol.wheel_play:
        pol.wheel_play = {k: v for k, v in pol.wheel_play.items() if k not in drop_play}
    pol.odometry_encoder = "counts"
    pol.ahrs_level, pol.ahrs_tau_s, pol.ahrs_channels = "tm151_filter", 2.0, "both"
    cfg = pc._cfg_for()
    cfg = {**cfg, "env": {**cfg["env"], "max_episode_s": float(t[-1]) + 5.0}}
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]
    obs, _ = env.reset(seed=10_000, options={"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0,
                                             "difficulty": 1.0})
    pol.reset()
    off = env._psi - bpsi0
    R = np.array([[np.cos(off), -np.sin(off)], [np.sin(off), np.cos(off)]])
    ts = np.arange(0.0, t[-1], env.ctrl_dt)
    C, P, V, PSI, fell = [], [], [], [], None
    for tt in ts:
        k = min(int(np.searchsorted(t, tt)), len(t) - 1)
        env._v_cmd_w = R @ cmd[k]                  # held: reset made resampling infinite
        env._psi_cmd = float(wrap_pi(cpsi[k] + off))
        obs, _r, term, _trunc, _ = env.step((np.asarray(pol.action(obs), float) / scale)[:n_act])
        C.append(env._v_cmd_w.copy()); P.append(env._psi_cmd)
        V.append(env.data.qvel[:2].copy()); PSI.append(env._psi)
        if term:
            fell = tt
            break
    m = env.model                              # what was BUILT, printed per row
    built = (f"play {sum('play' in m.joint(i).name for i in range(m.njnt))} joints, "
             f"axle {m.dof_frictionloss[m.joint('front_spin').dofadr[0]]:.3f} N m")
    return (ts[:len(C)], np.array(C), np.array(P), np.array(V), np.array(PSI),
            fell, built)


def score(t, cmd, cpsi, v, psi):
    c = np.linalg.norm(cmd, axis=1)
    along = (v * (cmd / np.maximum(c, 1e-9)[:, None])).sum(1)
    lon = cmd[:, 0] * np.cos(cpsi) + cmd[:, 1] * np.sin(cpsi)
    return speed_split(c, along, lon), turns(t, cpsi, psi)


def row(label, sp, tn, dt, extra=""):
    cells = "  ".join(f"{k} {r:5.2f} ({n * dt:3.0f} s)" for k, (r, n) in sp.items())
    print(f"  {label:40s} {cells}   turns {fmt_turns(tn):22s} {extra}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run", type=Path, help="traces/bike/<stamp>_run")
    ap.add_argument("--policies", nargs="+", default=None,
                    help="moves to fly (default: the run's own policy)")
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="PATH=VALUE", help="override one sim bike parameter (repeatable)")
    ap.add_argument("--drop-play", action="append", default=[], metavar="GROUP",
                    help="remove one wheel_play group from the sim (repeatable)")
    a = ap.parse_args()

    import json
    meta = json.loads((a.run / "meta.json").read_text())
    names = a.policies or [meta["policy"]]
    t, cmd, cpsi, vb, bpsi = record(a.run)

    params = load_params()
    for kv in a.overrides:
        path, _, val = kv.partition("=")
        params = _set(params, path, yaml.safe_load(val))
    for g in a.drop_play:
        if g not in params.get("wheel_play", {}):
            raise SystemExit(f"--drop-play {g}: not in wheel_play ({list(params.get('wheel_play', {}))})")
        params = {**params, "wheel_play": {k: v for k, v in params["wheel_play"].items() if k != g}}
    variant = " ".join(a.overrides + [f"-{g}" for g in a.drop_play]) or "as shipped"

    print(f"{a.run.name}: {t[-1]:.0f} s, flew {meta['policy']}; sim: {variant}")
    sp, tn = score(t, cmd, cpsi, vb, bpsi)
    row(f"bike ({meta['policy'].replace('general_puffer_', '')})", sp, tn, np.median(np.diff(t)))
    for n in names:
        ts, C, P, V, PSI, fell, built = replay(n, t, cmd, cpsi, bpsi[0], params, a.drop_play)
        sp, tn = score(ts, C, P, V, PSI)
        row(f"sim  {n.replace('general_puffer_', '')}", sp, tn, ts[1] - ts[0],
            built + (f", FELL at {fell:.0f} s" if fell is not None else ""))


if __name__ == "__main__":
    main()
