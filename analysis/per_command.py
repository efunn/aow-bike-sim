"""Per-command behaviour, policy by policy: which MOVES each one is bad at.

Every table in `docs/status.md` reports the eval grid as a single number per
policy, which answers "is it better" and never "where". This plots the 20
commands individually, one bar per policy, one figure per metric -- so a
policy that is excellent everywhere except reverse, or that only turns one
way, shows up as a shape rather than as a slightly worse average.

The x axis is grouped by COMMAND FAMILY, and mirrored pairs are adjacent
(+90 beside -90, +170 beside -170) so handedness reads directly off the
chart: a symmetric policy makes those pairs the same height. The grid's own
order is neither, so the mapping is spelled out in GROUPS below.

WHAT THE METRICS ARE. `head_med` / `vel_med` are medians over the whole
episode; `head_tail` / `vel_tail` are the worst excursion over the settled
last 2 s; `t_head_s` is time to first get inside 10 deg. NONE of them is a
whole-episode max, because both commands are steps applied at t=0 -- head_err
starts at |dpsi| and vel_err at |v_cmd| whatever the policy does. See
train_general_rl._eval_episodes.

A command the policy FELL on is drawn hatched and marked, because its bars are
not comparable with the rest: the episode ended early, so a "median heading
error" over 3 s of falling is not the same quantity as one over 15 s of
driving.

  python analysis/per_command.py                      # the 3 newest exports
  python analysis/per_command.py --policies general_rl_odo_ahrs \
      general_rl_odo_ahrs_rand2 --tag baseline_vs_rand2

Writes analysis/plots/per_command_<metric><tag>.png. Read-only otherwise.
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.build_model import load_params
from aow_sim.control.general_env import _load_rl_config
from aow_sim.train_general_rl import _eval_episodes, eval_cmds
from rsa_policies import REPO, env_for, load_general

PLOTS = Path(__file__).resolve().parent / "plots"

# The AHRS-trained set, oldest first: the baseline `control.general_move`
# currently points at, then the three runs that followed it. Chronological
# rather than ranked, so the left-to-right colour order is the order the
# questions were asked in. Not a standing set -- pass --policies for anything
# else, and pass --tag with it.
DEFAULT_POLICIES = ("general_rl_odo_ahrs",
                    "general_rl_odo_ahrs_rand",
                    "general_rl_odo_ahrs_tau019",
                    "general_rl_odo_ahrs_rand2")

# Colour-blind-safe trio, and deliberately not a rainbow: these are compared
# pairwise far more often than they are read as a sequence.
COLORS = ("#4477aa", "#ee6677", "#228833", "#ccbb44", "#66ccee")

# Command families, in plot order, with the grid index of each member and a
# short label. Mirrored pairs are adjacent ON PURPOSE -- see the docstring.
# The indices are into eval_cmds(v_max) and are checked against the actual
# command tuple at run time, so a change to the grid fails loudly instead of
# silently mislabelling every bar.
GROUPS = [
    ("hold + spin in place", [
        (0, "hold", (0.0, 0.0, 0)), (1, "spin +90", (0.0, 0.0, 90)),
        (14, "spin -90", (0.0, 0.0, -90)), (12, "spin +170", (0.0, 0.0, 170)),
        (13, "spin -170", (0.0, 0.0, -170)), (2, "spin 180", (0.0, 0.0, 180)),
    ]),
    ("forward, straight", [
        (3, "fwd 0.8", (0.804, 0.0, 0)), (6, "fwd 1.2", (1.2, 0.0, 0)),
    ]),
    ("forward + turn", [
        (10, "fwd .6 +45", (0.6, 0.0, 45)), (18, "fwd .6 -45", (0.6, 0.0, -45)),
        (4, "fwd .8 +90", (0.804, 0.0, 90)),
        (15, "fwd .8 -90", (0.804, 0.0, -90)),
        (5, "fwd .8 180", (0.804, 0.0, 180)),
    ]),
    ("reverse", [
        (7, "rev 0.5", (-0.504, 0.0, 0)),
        (8, "rev .5 +90", (-0.504, 0.0, 90)),
        (16, "rev .5 -90", (-0.504, 0.0, -90)),
    ]),
    ("crab", [
        (9, "crab L", (0.0, 0.396, 0)), (17, "crab R", (0.0, -0.396, 0)),
    ]),
    ("fwd + crab + 180", [
        (11, "fwd crabL 180", (0.6, 0.396, 180)),
        (19, "fwd crabR 180", (0.6, -0.396, 180)),
    ]),
]

# metric key -> (axis label, title, footnote). Lower is better for all of
# them, so that is said once in the title rather than stored per metric.
METRICS = {
    "t_head_s": ("seconds", "time to first get inside 10 deg of the commanded "
                            "heading",
                 "Commands with no heading change read 0 trivially -- only the "
                 "turning commands mean anything here. A bar at the episode "
                 "length (15 s) means it NEVER got inside 10 deg."),
    "head_err_med": ("degrees", "heading error, MEDIAN over the episode",
                     "How well it holds heading, typically. Not the last-step "
                     "value the moves/*.yaml metrics blocks carry."),
    "head_err_tail": ("degrees", "heading error, worst moment in the settled "
                                 "last 2 s",
                      "Deliberately not a whole-episode max: the command is a "
                      "STEP at t=0, so that would start at |dpsi| -- 180 deg "
                      "on the 180 commands -- however good the policy is."),
    "vel_err_med": ("m/s", "velocity-vector error, MEDIAN over the episode",
                    "|v_cmd - v_actual| over both axes, so it catches wrong "
                    "speed and wrong direction as one number."),
    "vel_err_tail": ("m/s", "velocity-vector error, worst moment in the "
                            "settled last 2 s",
                     "Same step-command argument as head_err_tail: a whole-"
                     "episode max would just report |v_cmd|."),
    "drift_m": ("m", "drift from the commanded position, at the END", ""),
    "drift_max": ("m", "drift from the commanded position, PEAK", ""),
    "drift_sd": ("m", "drift SD over the episode",
                 "Read WITH drift_max against drift_m: peak well above final "
                 "means it wandered out and came back, the two nearly equal "
                 "means it left and kept going."),
}


def order_and_labels(cmds):
    """(grid indices in plot order, bar labels, group spans) from GROUPS.

    Verifies every entry against the live grid: the labels are hand-written
    and a silently mislabelled bar is worse than no chart. Note the commands
    are `round(fraction * v_max, 3)`, so the nominal 0.8 m/s is really 0.804
    and 0.4 m/s of crab is 0.396 -- the LABELS round, the check does not.
    """
    idx, labels, spans = [], [], []
    for name, members in GROUPS:
        start = len(idx)
        for k, label, want in members:
            v_lon, v_lat, dpsi = cmds[k]
            got = (round(v_lon, 3), round(v_lat, 3),
                   int(round(np.degrees(dpsi))))
            if got != (round(want[0], 3), round(want[1], 3), want[2]):
                raise SystemExit(
                    f"eval grid changed: index {k} is {got}, GROUPS says "
                    f"{want} ({label!r}). Fix GROUPS in this script rather "
                    f"than the labels -- see the docstring.")
            idx.append(k)
            labels.append(label)
        spans.append((name, start, len(idx)))
    if len(idx) != len(cmds):
        raise SystemExit(f"GROUPS covers {len(idx)} of {len(cmds)} commands")
    return idx, labels, spans


def _cfg_for(ahrs, tau):
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    if ahrs != "none":
        cfg = {**cfg, "env": {**cfg["env"], "ahrs_level": ahrs,
                              "ahrs_tau_s": tau, "ahrs_channels": "both"}}
    return cfg


def _one_policy(job):
    """Whole 20-command grid for ONE policy. Runs in a worker process.

    Everything is built HERE rather than passed in: a MuJoCo model does not
    pickle, so the parent can only hand over strings and floats.

    Parallelised over POLICIES and deliberately not over commands. The grid's
    seed is `10_000 + k` where k is the index WITHIN the list handed to
    `_eval_episodes` -- so splitting the commands across workers would silently
    re-seed every episode and produce numbers that do not match any other table
    in the project. Four policies is four cores, which is the whole win here.
    """
    name, encoder, ahrs, tau = job
    params = load_params()
    cfg = _cfg_for(ahrs, tau)
    pol = load_general(name)
    if encoder:
        pol.odometry_encoder = encoder
    env = env_for(pol, params, cfg)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]

    def act(o):
        return (np.asarray(pol.action(o), float) / scale)[:n_act]

    _m, rows = _eval_episodes(env, act, eval_cmds(cfg["env"]["v_max"]))
    return name, rows


def run(names, encoder, ahrs, tau):
    """Eval every policy, in parallel, one process each.

    IMPORTING THIS: the pool uses spawn, so a caller must guard its own entry
    point with `if __name__ == "__main__":`. Without it each child re-imports
    the caller, re-enters `run`, and spawns again -- which surfaces as
    BrokenProcessPool rather than as anything resembling the actual mistake.
    """
    cmds = eval_cmds(_cfg_for(ahrs, tau)["env"]["v_max"])
    jobs = [(n, encoder, ahrs, tau) for n in names]
    out = {}
    with ProcessPoolExecutor(max_workers=min(len(jobs), os.cpu_count() or 1)) as ex:
        for name, rows in ex.map(_one_policy, jobs):
            out[name] = rows
            print(f"  {name}: {sum(r['fell'] for r in rows)}/{len(rows)} fell",
                  flush=True)
    # ex.map yields in submission order; restore the requested order anyway so
    # the colour assignment cannot depend on completion timing.
    return cmds, {n: out[n] for n in names}


def plot(metric, out, idx, labels, spans, args):
    unit, title, note = METRICS[metric]
    names = list(out)
    n = len(names)
    x = np.arange(len(idx))
    width = 0.8 / n

    # Width follows the number of COMMAND SLOTS, with a small allowance per
    # policy for the extra bars in each slot. Scaling by slots x policies
    # (the obvious formula) asks for a 50-inch canvas at four policies.
    fig, ax = plt.subplots(figsize=(max(12.0, 0.62 * len(idx) + 1.1 * n), 5.4))
    for j, name in enumerate(names):
        rows = out[name]
        vals = [rows[k][metric] for k in idx]
        fell = [rows[k]["fell"] for k in idx]
        pos = x + (j - (n - 1) / 2) * width
        ax.bar(pos, vals, width, label=name, color=COLORS[j % len(COLORS)],
               edgecolor="white", linewidth=0.4)
        # A fallen episode ended early, so its bar is a different quantity.
        # Hatch it rather than dropping it: the fall is information.
        for p, v, f in zip(pos, vals, fell):
            if f:
                ax.bar(p, v, width, color="none", edgecolor="black",
                       hatch="////", linewidth=0.6)
                ax.plot(p, v, marker="v", ms=5, color="black", clip_on=False)

    for _name, _a, b in spans[:-1]:
        ax.axvline(b - 0.5, color="0.75", lw=0.9, ls=":")
    # Group names ride just above the axes in AXES-FRACTION y, with the title
    # padded clear of them. Placing them at `ylim * 1.02` put them straight
    # through the title on every chart whose bars reach the top.
    for name, a, b in spans:
        ax.annotate(name, xy=((a + b - 1) / 2, 1.012),
                    xycoords=("data", "axes fraction"), ha="center",
                    va="bottom", fontsize=8.5, color="0.35")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel(f"{metric}  [{unit}]")
    ax.set_title(f"{title}   —   lower is better; hatched + ▼ = fell, "
                 f"episode ended early", fontsize=11, pad=26)
    ax.legend(fontsize=8.5, framealpha=0.9)
    ax.grid(axis="y", alpha=0.3)
    ax.set_axisbelow(True)
    ax.margins(x=0.01)
    # The sensor configuration goes ON the figure. Two runs of this script
    # differing only in --ahrs produce PNGs that are otherwise identical in
    # every label, and a chart you cannot identify from its own face is a
    # chart nobody can check.
    sensors = (f"sensors: --ahrs {args.ahrs}"
               + (f" tau {args.ahrs_tau:g} s" if args.ahrs != "none" else "")
               + f"   |   encoder {args.encoder or 'each policy\'s own'}"
               + ("   |   POLICIES READ MUJOCO TRUTH -- no sensor model in "
                  "the loop" if args.ahrs == "none" else ""))
    fig.text(0.5, -0.12, sensors, ha="center", va="top", fontsize=8.5,
             color="0.45")
    if note:
        fig.text(0.5, -0.17, note, ha="center", va="top", fontsize=8.5,
                 color="0.3", wrap=True)

    path = PLOTS / f"per_command_{metric}{args.tag}.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policies", nargs="+", metavar="NAME",
                    default=list(DEFAULT_POLICIES))
    ap.add_argument("--metrics", nargs="+", choices=list(METRICS),
                    default=list(METRICS))
    ap.add_argument("--encoder", default="counts",
                    choices=("counts", "ideal", "reported", ""),
                    help="force every policy onto ONE encoder model; \"\" "
                         "leaves each on its own declaration, which is not "
                         "comparable across a set that mixes them")
    ap.add_argument("--ahrs", default="tm151",
                    choices=("none", "tm151_static", "tm151", "tm171"))
    ap.add_argument("--ahrs-tau", type=float, default=0.19,
                    help="correlation time [s]; the measured value by default")
    ap.add_argument("--tag", default="",
                    help="suffix for the output filenames. REQUIRED in spirit "
                         "whenever --policies is not the default, or the new "
                         "figure silently overwrites the tracked one")
    args = ap.parse_args()
    args.tag = f"_{args.tag}" if args.tag and not args.tag.startswith("_") \
        else args.tag

    print(f"eval: {len(args.policies)} policies in parallel, one process "
          f"each, --encoder {args.encoder} --ahrs {args.ahrs} --ahrs-tau "
          f"{args.ahrs_tau:g}")
    cmds, out = run(args.policies, args.encoder, args.ahrs, args.ahrs_tau)
    idx, labels, spans = order_and_labels(cmds)
    PLOTS.mkdir(exist_ok=True)
    for metric in args.metrics:
        print("wrote", plot(metric, out, idx, labels, spans, args))


if __name__ == "__main__":
    main()
