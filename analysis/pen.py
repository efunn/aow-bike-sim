"""The pen: what the bike actually draws on the floor, per command.

Teleop reports things a scalar metric hides -- "left crab prefers going
backward", "it draws a slow squiggly repeated S". Those are statements about
the PATH, and a mean velocity cannot represent them: a squiggle whose lobes
cancel and a straight line have the same mean. This is the top-down trace,
which is the same thing the operator sees, plus the numbers that go with it.

Each panel is one command held for the whole episode. The bike starts at the
origin facing +X. Ticks along the path are the heading every `--tick` seconds,
so a path that translates without turning (a true crab) shows parallel ticks,
while one that curves shows them fanning.

  python analysis/pen.py
  python analysis/pen.py --policies general_rl_glide_og --seconds 20

Writes analysis/pen_<policy>.png and prints the coupling table. Read-only
otherwise: loads moves/*.npz and changes nothing else.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.general_env import _load_rl_config
from rsa_policies import POLICIES, REPO, env_for, load_general

# (label, (v_lon, v_lat), heading step [deg]). The crab pair and the turn pair
# are the ones teleop reports asymmetries in, so both signs of each are here.
# A `held` entry is a HELD TURN KEY, not a step: teleop slews the heading
# command at _TURN_RATE with a _LEAD_MAX cap (run_drive.py:637-641, :1012),
# which is a sustained turn-RATE command. A step command says nothing about
# that regime -- the asymmetry the operator feels only appears here.
TURN_RATE = 1.2                  # rad/s, run_drive._TURN_RATE
LEAD_MAX = np.deg2rad(35.0)      # run_drive._LEAD_MAX

COMMANDS = [
    ("hold",      (0.0, 0.0),     0),
    ("fwd",       (0.8, 0.0),     0),
    ("rev",       (-0.5, 0.0),    0),
    ("crabL",     (0.0, 0.396),   0),
    ("crabR",     (0.0, -0.396),  0),
    ("turnL +90", (0.0, 0.0),    90),
    ("turnR -90", (0.0, 0.0),   -90),
    ("heldL",     (0.0, 0.0),  "held+"),
    ("heldR",     (0.0, 0.0),  "held-"),
]


def trace(pol, env, v_cmd, dpsi_deg, seconds):
    """World path + heading + body-frame velocity for one held command."""
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    held = 0
    if isinstance(dpsi_deg, str):            # "held+" / "held-"
        held, dpsi_deg = (1 if dpsi_deg.endswith("+") else -1), 0
    obs, _ = env.reset(seed=7, options={"v_cmd": v_cmd,
                                        "psi_cmd_rel": np.deg2rad(dpsi_deg),
                                        "difficulty": 1.0})
    xy, psi, v_lon, v_lat = [], [], [], []
    for _ in range(int(seconds / env.ctrl_dt)):
        if held:
            lead = float(np.arctan2(np.sin(env._psi_cmd - env._psi),
                                    np.cos(env._psi_cmd - env._psi)))
            if not ((held > 0 and lead >= LEAD_MAX) or
                    (held < 0 and lead <= -LEAD_MAX)):
                env._psi_cmd += held * TURN_RATE * env.ctrl_dt
        a = np.asarray(pol.action(obs), float) / scale
        obs, _r, term, trunc, _i = env.step(a[:env.action_space.shape[0]])
        s = extract_state(env.data, env._p0)
        xy.append(env.data.qpos[:2].copy())
        psi.append(s.yaw)
        v_lon.append(s.v_lon)
        v_lat.append(s.v_lat)
        if term or trunc:
            break
    return (np.array(xy), np.array(psi), np.array(v_lon), np.array(v_lat))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policies", nargs="*", default=None)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--tick", type=float, default=1.0,
                    help="seconds between heading ticks along the path")
    args = ap.parse_args()

    keys = args.policies or list(POLICIES)
    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}

    for key in keys:
        pol = load_general(key)
        env = env_for(pol, params, cfg)
        traces = {lab: trace(pol, env, v, d, args.seconds)
                  for lab, v, d in COMMANDS}

        # The coupling teleop reports, as numbers. Longitudinal drift is quoted
        # RELATIVE TO HOLD: this policy idles backwards at ~0.1 m/s, so an
        # absolute -0.05 under a turn is the bike moving FORWARD relative to
        # what it does when asked for nothing, which is what the operator
        # feels. Against zero it would read as "backward" and contradict them.
        hold_lon = traces["hold"][2].mean()
        print(f"\n{key}   (drift at hold {hold_lon:+.3f} m/s)")
        print(f"{'command':>12}{'v_lon':>9}{'vs hold':>9}{'v_lat':>9}"
              f"{'net m':>8}{'path m':>8}{'wander':>8}")
        for lab, *_ in COMMANDS:
            xy, psi, vl, vt = traces[lab]
            net = float(np.linalg.norm(xy[-1] - xy[0]))
            path = float(np.abs(np.diff(xy, axis=0)).sum())
            print(f"{lab:>12}{vl.mean():>+9.3f}{vl.mean() - hold_lon:>+9.3f}"
                  f"{vt.mean():>+9.3f}{net:>8.2f}{path:>8.2f}"
                  f"{path / max(net, 1e-6):>8.1f}")
        print("  wander = path length / net displacement. 1.0 is a straight "
              "line; large means the pen squiggled and cancelled.")

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\nno PNG: matplotlib is not installed in this interpreter")
            continue

        n = len(COMMANDS)
        fig, axes = plt.subplots(1, n, figsize=(3.0 * n, 3.4))
        step = max(1, int(args.tick / env.ctrl_dt))
        for ax, (lab, _v, _d) in zip(np.atleast_1d(axes), COMMANDS):
            xy, psi, _vl, _vt = traces[lab]
            ax.plot(xy[:, 0], xy[:, 1], lw=1.1, color="#1f77b4")
            ax.plot(xy[0, 0], xy[0, 1], "o", ms=4, color="0.2")
            # heading ticks: parallel = translating, fanning = turning
            for i in range(0, len(xy), step):
                c, s = np.cos(psi[i]), np.sin(psi[i])
                ax.plot([xy[i, 0], xy[i, 0] + 0.05 * c],
                        [xy[i, 1], xy[i, 1] + 0.05 * s],
                        lw=0.9, color="#d62728")
            ax.set_title(lab, fontsize=9)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25, lw=0.5)
            ax.tick_params(labelsize=6)
        axes[0].set_ylabel("world Y [m]  (+Y = left)", fontsize=8)
        fig.suptitle(f"{key} — ground track, {args.seconds:.0f} s per command "
                     f"(red ticks = heading every {args.tick:.0f} s)",
                     fontsize=10)
        fig.tight_layout()
        out = Path(__file__).parent / f"pen_{key}.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"  wrote {out}")


if __name__ == "__main__":
    main()
