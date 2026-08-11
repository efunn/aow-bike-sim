"""Which wheel leaves the ground, by how much, and is it pitch or a hop.

Supersedes the airborne-PERCENTAGE columns in hold_spectrum.py and chatter.py,
which are misleading here. Binary "is the wheel touching" counts the rear
omni's own envelope ripple -- ~0.6 mm across roll phase, expected geometry, see
section 1 of docs/measurements/omni-wheel-protocol.md -- identically to a
visible wheelie. Measured that way the rear reads 40-60% "airborne" while never
clearing 5 mm, and the front reads about the same while clearing 79 mm. The
percentage is the same; the behaviour is two orders of magnitude apart. So
report the GAP DISTRIBUTION, in mm, and the pitch angle that goes with it.

PITCH VS HOP. Gap alone cannot tell a nose-up from the whole bike leaving the
floor, so this reports three things side by side:

  measured pitch   chassis pitch from the freejoint quaternion [deg].
  implied pitch    atan((front gap - rear gap) / wheelbase) [deg] -- the pitch
                   the two gaps would need if the bike were pivoting about a
                   grounded contact.
  common lift      min(front, rear) gap [mm] -- what is left once pitch is
                   removed, i.e. the part that is a genuine hop.

Measured ~= implied and common lift ~= 0 means pure pitching. Measured near
zero with a large common lift means hopping. Both nonzero means both.

  python analysis/liftoff.py
  python analysis/liftoff.py --policies general_rl_glide_og --seconds 20

Writes analysis/liftoff_<policy>.png and prints the tables. Read-only
otherwise: loads moves/*.npz and changes nothing else.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.general_env import _load_rl_config
from rsa_policies import POLICIES, REPO, env_for, load_general
from wheel_slowmo import clearance_mm, wheel_vertices

COMMANDS = [
    ("hold",       (0.0, 0.0)),
    ("accelerate", (0.8, 0.0)),
    ("reverse",    (-0.5, 0.0)),
    ("crabL",      (0.0, 0.396)),
]


def pitch_deg(data) -> float:
    """Chassis pitch [deg], POSITIVE = nose up.

    Note the sign. `asin(-R[2,0])` is the usual ZYX pitch and it comes out
    NEGATIVE when the nose rises here, because R[2,0] is the world-z component
    of the body +X (forward) axis. Reporting that raw makes `max()` pick the
    nose-DOWN tail, which reads as "pitch never exceeds 0.4 deg" during a
    23 deg wheelie. Negated here so the sign matches the word.
    """
    M = np.zeros(9)
    mujoco.mju_quat2Mat(M, data.qpos[3:7])
    return float(np.degrees(np.arcsin(M.reshape(3, 3)[2, 0])))


def wheelbase(model) -> float:
    return float(abs(model.body("front_wheel").pos[0]
                     - model.body("aow_hub").pos[0])) or 0.2


def run(pol, env, v_cmd, seconds):
    """Per-step front/rear clearance [mm] and chassis pitch [deg]."""
    m = env.model
    names = [m.geom(i).name for i in range(m.ngeom)]
    rear = {i for i, n in enumerate(names) if n.startswith("roller_")}
    front = {names.index("front_tire")}
    vr, vf = wheel_vertices(m, rear), wheel_vertices(m, front)
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    obs, _ = env.reset(seed=7, options={"v_cmd": v_cmd, "psi_cmd_rel": 0.0,
                                        "difficulty": 1.0})
    R, F, P = [], [], []
    for _ in range(int(seconds / env.ctrl_dt)):
        a = np.asarray(pol.action(obs), float) / scale
        obs, _r, term, trunc, _i = env.step(a[:env.action_space.shape[0]])
        R.append(clearance_mm(env.data, vr))
        F.append(clearance_mm(env.data, vf))
        P.append(pitch_deg(env.data))
        if term or trunc:
            break
    return np.array(R), np.array(F), np.array(P)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policies", nargs="*", default=None)
    ap.add_argument("--seconds", type=float, default=15.0)
    args = ap.parse_args()

    keys = args.policies or list(POLICIES)
    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}

    all_traces = {}
    for key in keys:
        pol = load_general(key)
        env = env_for(pol, params, cfg)
        L = wheelbase(env.model)
        traces = {lab: run(pol, env, v, args.seconds) for lab, v in COMMANDS}
        all_traces[key] = (traces, L)

        print(f"\n{key}   (wheelbase {L:.3f} m)")
        print("  GAP [mm], positive = clear of the floor")
        print(f"{'command':>12}{'wheel':>7}{'p50':>8}{'p90':>8}{'p99':>8}"
              f"{'max':>9}{'>1mm':>7}{'>5mm':>7}")
        for lab, _v in COMMANDS:
            R, F, _P = traces[lab]
            for wl, g in (("rear", R), ("front", F)):
                print(f"{lab:>12}{wl:>7}"
                      + "".join(f"{np.percentile(g, q):>8.2f}"
                                for q in (50, 90, 99))
                      + f"{g.max():>9.2f}{np.mean(g > 1.0):>7.1%}"
                      f"{np.mean(g > 5.0):>7.1%}")

        print("  PITCH [deg], +ve = NOSE UP. `at peak` is measured at the "
              "same instant as")
        print("  the largest front gap, not independently -- comparing two "
              "maxima taken at")
        print("  different timesteps is what made this look self-contradictory.")
        print(f"{'command':>12}{'sd':>7}{'p99':>7}{'max':>7}{'at peak':>9}"
              f"{'implied':>9}{'corr':>8}{'common lift':>13}")
        for lab, _v in COMMANDS:
            R, F, P = traces[lab]
            implied = np.degrees(np.arctan2(F - R, L * 1000.0))
            k = int(np.argmax(F))
            corr = float(np.corrcoef(F, P)[0, 1]) if P.std() > 1e-9 else np.nan
            print(f"{lab:>12}{P.std():>7.2f}{np.percentile(P, 99):>7.2f}"
                  f"{P.max():>7.2f}{P[k]:>9.2f}{implied[k]:>9.2f}{corr:>8.3f}"
                  f"{np.percentile(np.minimum(F, R), 99):>10.2f}mm")
        print("  corr = corr(front gap, pitch) over the episode. Near +1 means "
              "the front")
        print("  gap IS the wheelie; a hop would show corr ~0 and a large "
              "common lift.")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nno PNG: matplotlib is not installed in this interpreter")
        return

    for key, (traces, L) in all_traces.items():
        n = len(COMMANDS)
        fig, axes = plt.subplots(2, n, figsize=(3.3 * n, 5.2), squeeze=False)
        for j, (lab, _v) in enumerate(COMMANDS):
            R, F, P = traces[lab]
            t = np.arange(len(R)) * 0.02
            ax = axes[0][j]
            ax.plot(t, F, lw=0.8, label="front", color="#d62728")
            ax.plot(t, R, lw=0.8, label="rear", color="#1f77b4")
            ax.axhline(0, color="0.3", lw=0.8)
            ax.axhline(0.6, color="0.6", lw=0.6, ls=":")   # facet ripple scale
            ax.set_title(lab, fontsize=9)
            ax.set_yscale("symlog", linthresh=1.0)
            ax.tick_params(labelsize=6)
            if j == 0:
                ax.set_ylabel("gap [mm]  (symlog, dotted = facet ripple)",
                              fontsize=7)
                ax.legend(fontsize=6, loc="upper right")
            ax = axes[1][j]
            ax.plot(t, P, lw=0.8, color="#2ca02c")
            ax.axhline(0, color="0.3", lw=0.8)
            ax.set_xlabel("s", fontsize=7)
            ax.tick_params(labelsize=6)
            if j == 0:
                ax.set_ylabel("chassis pitch [deg]", fontsize=7)
        fig.suptitle(f"{key} — wheel clearance and pitch", fontsize=10)
        fig.tight_layout()
        out = Path(__file__).parent / f"liftoff_{key}.png"
        fig.savefig(out, dpi=150, bbox_inches="tight")
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
