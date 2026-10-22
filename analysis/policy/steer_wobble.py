"""The front-wheel wobble at constant speed: its frequency and amplitude,
per commanded speed, for one policy -- and how they move when the rake or
the wheelbase is changed under that same policy.

Each episode holds one command, (v, 0) on the current heading, for
`--seconds` (default 20 s, 3 seeds; `--seeds 5` for the figure); the first `--settle` s (5 s) are dropped and
the rest is the steady window. Per speed, over `--seeds` episodes:

  f_peak     the peak of the steer angle's spectrum (Welch, 0.3-25 Hz)
  amp        sqrt(2) x RMS of the steer angle band-passed at f_peak +-40%
             -- the amplitude of a sinusoid with that power [deg]
  rms        RMS of the whole detrended steer angle [deg] (wobble + drift)
  roll amp   the same band at the same frequency, on the chassis roll
  |steer|    mean steer angle [deg] (the turn's own steer, in a turn)
  sat %      share of steps the steer servo is at its stall-torque clip
  trk deg    mean |steer target - steer|: how far the wheel trails its command
  lag        the phase of roll relative to steer at f_peak [deg]; negative
             means roll lags the steer (forward: -55 to -100; reverse: +95
             to +110, the steer->roll coupling flips sign with speed)

`--turn-rate` holds a steady turn instead, the way teleop's held turn key
does: the heading command slews at that rate [rad/s] (1.2 in teleop), never
more than 35 deg ahead of the bike, and the speed command turns with it.
In a turn the 0.3 Hz floor of the peak search is the binding one: a slow
steer hunt of ~15 deg p-p at ~0.4 Hz is there at every rake.

All on truth (qpos), at the 50 Hz control rate, so nothing above 25 Hz is
seen. The policy flies on its own sensors (its move yaml's AHRS level and
odometry), randomization off, no ball.

GEOMETRY OVERRIDES are in memory only, the same edit as teleop's
`run_drive --wheelbase` / `--rake` (front end moves, chassis CoM held; the
steer axis tilts about the front axle, trail = r_front x tan(rake));
`bike_params.yaml` is not touched. They fly the SAME weights: the policy
was trained on the stock plant (wheelbase 0.20 m, rake 15 deg), so a changed plant is out of distribution
and the answer is "what this policy would do on that bike", not what a
policy trained on it would do.

    python analysis/policy/steer_wobble.py
    python analysis/policy/steer_wobble.py --seeds 5 --rake 0 5 20 22 25 30 \\
        --wheelbase 0.15 0.25 0.30 --plot   # analysis/policy/plots/steer_wobble_<move>.png
    python analysis/policy/steer_wobble.py --speeds -1.2 -0.8 -0.4 0 0.4 0.8 1.2 \\
        --turn-rate 0 0.6 1.2 --rake 20 22 25 30

Read-only apart from the optional figure.
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

SPEEDS = (-1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2)
CTRL_HZ = 50.0
LEAD_MAX = np.deg2rad(35.0)    # run_drive._LEAD_MAX


def _episode(job):
    move, v, rate, seed, seconds, rake, wheelbase = job
    from aow_sim.build_model import load_params
    from aow_sim.control.balance import extract_state
    from per_command import _cfg_for
    from rsa_policies import env_for, load_general

    params = load_params()
    if rake is not None:
        params["bike"]["rake_deg"] = float(rake)
    if wheelbase is not None:
        params["bike"]["wheelbase"] = float(wheelbase)
    cfg = _cfg_for()
    cfg = {**cfg, "env": {**cfg["env"], "max_episode_s": float(seconds)}}
    pol = load_general(move)
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]
    obs, _ = env.reset(seed=20_000 + seed,
                       options={"v_cmd": (v, 0.0), "psi_cmd_rel": 0.0, "difficulty": 1.0})
    pol.reset()
    aid = env.model.actuator("steer").id
    stall = float(env.model.actuator_forcerange[aid, 1])
    rows, done, term = [], False, False
    while not done:
        if rate:
            # teleop's held turn key: the heading command slews at `rate`,
            # never more than _LEAD_MAX ahead of the bike, and the speed
            # command turns with it (forward along the commanded heading)
            lead = np.clip(env._psi_cmd + rate * env.ctrl_dt - env._psi,
                           -LEAD_MAX, LEAD_MAX)
            env._psi_cmd = env._psi + lead
            env._v_cmd_w = v * np.array([np.cos(env._psi_cmd), np.sin(env._psi_cmd)])
        a = (np.asarray(pol.action(obs), float) / scale)[:n_act]
        obs, _r, term, trunc, _ = env.step(a)
        done = term or trunc
        s = extract_state(env.data, env._p0)
        steer = float(env.data.qpos[env._sj])
        rows.append((steer, s.roll, s.v_lon, float(env.data.qvel[5]), float(a[0]),
                     abs(env.data.actuator_force[aid]) >= 0.98 * stall,
                     env._steer - steer))
    return v, rate, seed, bool(term), np.array(rows)


def _band(x, f0, lo=0.6, hi=1.4):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / CTRL_HZ)
    X[(f < lo * f0) | (f > hi * f0)] = 0
    return np.fft.irfft(X, len(x))


def analyse(tr, settle):
    from scipy.signal import welch
    w = tr[int(settle * CTRL_HZ):]
    steer = np.unwrap(w[:, 0])
    steer = steer - np.polyval(np.polyfit(np.arange(len(steer)), steer, 1), np.arange(len(steer)))
    roll = w[:, 1] - w[:, 1].mean()
    f, P = welch(steer, CTRL_HZ, nperseg=min(len(steer), 256))
    m = (f >= 0.3) & (f <= 25)
    f0 = float(f[m][np.argmax(P[m])])
    bs, br = _band(steer, f0), _band(roll, f0)
    k = np.argmin(abs(np.fft.rfftfreq(len(steer), 1 / CTRL_HZ) - f0))
    lag = np.degrees(np.angle(np.fft.rfft(roll)[k] / np.fft.rfft(steer)[k]))
    return dict(f=f0, amp=np.degrees(np.sqrt(2) * bs.std()),
                sat=100 * w[:, 5].mean(), lag_t=np.degrees(abs(w[:, 6]).mean()),
                steer_mean=np.degrees(abs(np.unwrap(w[:, 0]).mean())),
                rms=np.degrees(steer.std()), roll=np.degrees(np.sqrt(2) * br.std()),
                lag=float(lag), v=float(w[:, 2].mean()), psd=(f, P))


def run(move, speeds, seeds, seconds, settle, rake=None, wheelbase=None, rates=(0.0,)):
    keys = [(v, r) for r in rates for v in speeds]
    jobs = [(move, v, r, s, seconds, rake, wheelbase) for v, r in keys for s in range(seeds)]
    out = {}
    with ProcessPoolExecutor(max_workers=os.cpu_count() or 4) as ex:
        for v, r, seed, fell, tr in ex.map(_episode, jobs):
            out.setdefault((v, r), []).append((fell, tr))
    res = {}
    for k in keys:
        ok = [analyse(tr, settle) for fell, tr in out[k]
              if not fell and len(tr) > (settle + 4) * CTRL_HZ]
        res[k] = (sum(f for f, _ in out[k]), ok)
    return res


def table(res, label):
    print(f"\n{label}")
    print(f"  {'v_cmd':>6} {'turn':>5} {'v_ach':>6} {'fell':>4} {'f_peak Hz':>14} {'amp deg':>9}"
          f" {'rms deg':>8} {'roll amp':>9} {'lag deg':>8} {'|steer|':>8} {'sat %':>6}"
          f" {'trk deg':>8}")
    for (v, rate), (nfell, ok) in res.items():
        if not ok:
            print(f"  {v:+6.1f} {rate:5.1f} {'':>6} {nfell:>4}   (no steady episode)")
            continue
        g = lambda k: np.array([r[k] for r in ok])
        fs = g("f")
        fcol = f"{np.median(fs):.1f} ({fs.min():.1f}-{fs.max():.1f})"
        print(f"  {v:+6.1f} {rate:5.1f} {g('v').mean():+6.2f} {nfell:>4} {fcol:>14}"
              f" {g('amp').mean():6.1f}+-{g('amp').std():<3.1f}"
              f" {g('rms').mean():7.1f} {g('roll').mean():8.2f} {np.median(g('lag')):8.0f}"
              f" {g('steer_mean').mean():8.1f} {g('sat').mean():6.1f} {g('lag_t').mean():8.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--move", default="general_puffer_fixes_72M")
    ap.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--rake", nargs="+", type=float, default=[],
                    help="also fly at these rakes [deg] (stock 15)")
    ap.add_argument("--wheelbase", nargs="+", type=float, default=[],
                    help="also fly at these wheelbases [m] (stock 0.20)")
    ap.add_argument("--turn-rate", nargs="+", type=float, default=[0.0],
                    help="heading-command slew [rad/s], as teleop's held turn "
                         "key (1.2 there); 0 = straight")
    ap.add_argument("--no-stock", action="store_true",
                    help="skip the stock-geometry arm")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    arms = ([] if a.no_stock else [("stock", None, None)]) + [(f"rake {r:g} deg", r, None) for r in a.rake] \
        + [(f"wheelbase {w:g} m", None, w) for w in a.wheelbase]
    results = {}
    for label, r, w in arms:
        results[label] = run(a.move, a.speeds, a.seeds, a.seconds, a.settle, r, w,
                             a.turn_rate)
        table(results[label], f"{a.move}, {label}")
    if a.plot:
        plot(results, a.move)


def plot(results, move):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for label, res in results.items():
        res = {v: x for (v, rate), x in res.items() if rate == 0.0}
        vs = [v for v, (_, ok) in res.items() if ok]
        med = lambda k: [np.median([r[k] for r in res[v][1]]) for v in vs]
        ax[0].plot(vs, med("f"), "o-", label=label)
        ax[1].plot(vs, med("amp"), "o-", label=label)
        ax[2].plot(vs, med("roll"), "o-", label=label)
    for a_, yl in zip(ax, ("steer wobble peak [Hz]", "steer amplitude at peak [deg]",
                           "roll amplitude at that frequency [deg]")):
        a_.set_xlabel("commanded speed [m/s]")
        a_.set_ylabel(yl)
        a_.grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    fig.suptitle(f"{move}: steer wobble at constant speed (truth, 50 Hz, 5 s settle dropped)")
    fig.tight_layout()
    out = Path(__file__).resolve().parent / "plots" / f"steer_wobble_{move}.png"
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    main()
