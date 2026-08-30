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
from aow_sim.train_general_rl import FAMILIES, _eval_episodes, eval_cmds
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

# Subsections WITHIN a family, in plot order. The families themselves come
# from train_general_rl.FAMILIES -- imported, never restated, so a chart and a
# moves/*.yaml metrics block cannot disagree about what `cruise` means.
#
# A family is subdivided only where its members are two different manoeuvres:
# `cruise` splits forward from reverse, and `turn_big` splits the in-place
# reversals from the moving ones (measured 13.5 deg against 174.0 for
# general_rl_odo_ahrs -- the two halves are not the same task). Everything
# else is left whole. Subsections are display only; they do not change what
# the yaml reports.
SUBSECTIONS = {
    "cruise": [("forward", lambda c: c[0] > 0), ("reverse", lambda c: c[0] < 0)],
    "turn_big": [("in place", lambda c: c[0] == 0 and c[1] == 0),
                 ("moving", lambda c: c[0] != 0 or c[1] != 0)],
}

# Which commands a metric is DEFINED for. A bar is dropped only where the
# number is not a measurement of anything -- `t_head_s` on a command with no
# heading change fires on the first step, so those six bars would read 0 and
# say nothing. Everything else keeps its slot: drift on a moving command is
# not in the yaml but is still "how far it went", which is worth seeing.
VALID = {"t_head_s": lambda c: abs(c[2]) > 1}


def label_for(cmd):
    """A label GENERATED from the command, not hand-written.

    The previous version carried a hand-written index-to-label table, which
    needed a guard against the grid changing under it -- and the guard fired
    on the first run, because the commands are `round(fraction * v_max, 3)`
    and the nominal 0.8 m/s is really 0.804. Deriving the label removes the
    class of error rather than detecting it.
    """
    v_lon, v_lat, dpsi = cmd[0], cmd[1], cmd[2]
    head = f"{dpsi:+.0f}" if abs(dpsi) > 1 else ""
    if v_lon == 0 and v_lat == 0:
        return f"spin {head}" if head else "hold"
    # .rstrip("0") alone turns 1.00 into "1." -- strip the point too.
    num = lambda v: f"{abs(v):.2f}".rstrip("0").rstrip(".")
    parts = []
    if abs(v_lon) > 1e-9:
        parts.append(f"{'fwd' if v_lon > 0 else 'rev'} {num(v_lon)}")
    if abs(v_lat) > 1e-9:
        parts.append(f"crab{'L' if v_lat > 0 else 'R'} {num(v_lat)}")
    return " ".join(parts + ([head] if head else []))


def hand_of(cmd):
    """+1 left, -1 right, 0 neither -- for the tick colour, so a mirrored pair
    is visible as a pair without reading the numbers."""
    if abs(cmd[2]) > 1:
        return 1 if cmd[2] > 0 else -1
    if abs(cmd[1]) > 1e-9:
        return 1 if cmd[1] > 0 else -1
    return 0


HAND_COLOR = {1: "#227722", -1: "#bb5500", 0: "0.35"}


# metric key -> (axis label, title, footnote). Lower is better for all of
# them, so that is said once in the title rather than stored per metric.
METRICS = {
    "t_head_s": ("seconds", "time to first get inside 10 deg of the commanded "
                            "heading",
                 "A bar at the episode length (15 s) means it NEVER got "
                 "inside 10 deg. Commands with no heading change are omitted "
                 "-- they fire on the first step and would all read 0."),
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
    "drift_m": ("m", "drift from the commanded position, at the END",
                "Only `hold` reaches the moves/*.yaml block, but the number is "
                "defined everywhere and is still `how far it went`."),
    "drift_max": ("m", "drift from the commanded position, PEAK", ""),
    "drift_overshoot": ("m", "drift OVERSHOOT -- peak minus final",
                        "0 means it left and kept going; > 0 means it "
                        "wandered out and came back. Replaces drift_max, "
                        "which equalled drift_m in 3 of 4 policies."),
    "drift_sd": ("m", "drift SD over the episode",
                 "Read WITH drift_max against drift_m: peak well above final "
                 "means it wandered out and came back, the two nearly equal "
                 "means it left and kept going."),
    # WHICH WAY it went. `drift_m` is a hypot, so a bike backing out from
    # under itself and one sliding sideways read identically -- two different
    # failures with two different fixes. Resolved in the COMMANDED-heading
    # frame, fixed for the episode, NOT the live bike-yaw frame those e_lon /
    # e_lat come in: a decomposition that rotates with the thing it measures
    # cannot report the case where the heading is what went wrong.
    "drift_lon": ("m", "drift ALONG the commanded heading (+ = forward)",
                  "Read WITH drift_overshoot: a large |drift_lon| with a "
                  "large overshoot is out-and-back along the command axis; "
                  "with overshoot ~0 it left in that direction and stayed."),
    "drift_lat": ("m", "drift ACROSS the commanded heading (+ = left)",
                  "A handed policy shows one sign here across every command, "
                  "which no magnitude metric can express."),
}


def order_and_labels(cmds):
    """(grid indices in plot order, labels, spans) from FAMILIES.

    Mirrored pairs are adjacent inside every subsection -- sorted by turn
    magnitude then by sign, positive first -- so a symmetric policy makes each
    pair the same height and handedness reads off the chart without computing
    turn_asym.
    """
    idx, labels, spans = [], [], []
    for fam, pred in FAMILIES.items():
        members = [k for k, c in enumerate(cmds)
                   if pred((round(c[0], 3), round(c[1], 3),
                            int(round(np.degrees(c[2])))))]
        if not members:
            continue
        subs = SUBSECTIONS.get(fam, [("", lambda c: True)])
        for sub_name, sub_pred in subs:
            sel = [k for k in members
                   if sub_pred((cmds[k][0], cmds[k][1],
                                np.degrees(cmds[k][2])))]
            if not sel:
                continue
            sel.sort(key=lambda k: (abs(round(np.degrees(cmds[k][2]))),
                                    abs(cmds[k][1]), abs(cmds[k][0]),
                                    -np.degrees(cmds[k][2]), -cmds[k][1]))
            start = len(idx)
            idx += sel
            labels += [label_for((cmds[k][0], cmds[k][1],
                                  np.degrees(cmds[k][2]))) for k in sel]
            spans.append((f"{fam}\n{sub_name}" if sub_name else fam,
                          start, len(idx)))
    if len(idx) != len(cmds):
        raise SystemExit(
            f"FAMILIES covers {len(idx)} of {len(cmds)} commands -- one falls "
            f"through every predicate and would be invisible")
    return idx, labels, spans


def _cfg_for(ahrs, tau):
    """The eval env's config: randomization off, no ball.

    NO BALL, matching `train_general_rl._eval_cfg`. `ball_prob` is contact
    -robustness DR for TRAINING and it was leaking into every read-only
    measurement here -- a quarter of episodes had a ball parked within
    `ball_place_radius`, deterministic per command (the seed is `10_000 + k`)
    but arbitrary as to WHICH commands got one. It went unnoticed until one
    rolled through the `spin +170` panel of an eval video. Shared by
    `eval_video.py`, so the clips and the bars are the same episodes the
    metrics describe.
    """
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    cfg = {**cfg, "env": {**cfg["env"], "ball_prob": 0.0}}
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
    cmds = eval_cmds(_cfg_for(args.ahrs, args.ahrs_tau)["env"]["v_max"])
    deg = lambda k: (cmds[k][0], cmds[k][1], np.degrees(cmds[k][2]))

    # Drop only the bars this metric is not DEFINED for, and renumber the
    # spans around the gap. Every other command keeps its slot: drift on a
    # moving command never reaches the yaml but is still "how far it went".
    valid = VALID.get(metric)
    keep = [i for i, k in enumerate(idx) if valid is None or valid(deg(k))]
    if not keep:
        return None
    remap = {o: n for n, o in enumerate(keep)}
    idx_v = [idx[i] for i in keep]
    lab_v = [labels[i] for i in keep]
    spans_v = []
    for name, a_, b_ in spans:
        inside = [remap[i] for i in keep if a_ <= i < b_]
        if inside:
            spans_v.append((name, min(inside), max(inside) + 1))
    dropped = len(idx) - len(idx_v)

    names = list(out)
    n = len(names)
    x = np.arange(len(idx_v))
    width = 0.8 / n

    fig, ax = plt.subplots(figsize=(max(12.0, 0.66 * len(idx_v) + 1.1 * n), 5.6))
    for j, name in enumerate(names):
        rows = out[name]
        vals = [(rows[k]["drift_max"] - rows[k]["drift_m"])
                if metric == "drift_overshoot" else rows[k][metric]
                for k in idx_v]
        fell = [rows[k]["fell"] for k in idx_v]
        pos = x + (j - (n - 1) / 2) * width
        ax.bar(pos, vals, width, label=name, color=COLORS[j % len(COLORS)],
               edgecolor="white", linewidth=0.4)
        # A fallen episode ended early, so its bar is a different quantity.
        # Hatch it rather than dropping it: the fall is information.
        for p_, v, f in zip(pos, vals, fell):
            if f:
                ax.bar(p_, v, width, color="none", edgecolor="black",
                       hatch="////", linewidth=0.6)
                ax.plot(p_, v, marker="v", ms=5, color="black", clip_on=False)

    for _name, _a, b_ in spans_v[:-1]:
        ax.axvline(b_ - 0.5, color="0.75", lw=0.9, ls=":")
    for name, a_, b_ in spans_v:
        ax.annotate(name, xy=((a_ + b_ - 1) / 2, 1.012),
                    xycoords=("data", "axes fraction"), ha="center",
                    va="bottom", fontsize=8.5, color="0.35")

    ax.set_xticks(x)
    ax.set_xticklabels(lab_v, rotation=45, ha="right", fontsize=8)
    # Tick colour is HANDEDNESS: green left / +heading, orange right /
    # -heading, grey neither. Mirrored pairs sit adjacent, so a symmetric
    # policy makes each green/orange pair the same height.
    for t, k in zip(ax.get_xticklabels(), idx_v):
        t.set_color(HAND_COLOR[hand_of(deg(k))])

    ax.set_ylabel(f"{metric}  [{unit}]")
    ax.set_title(f"{title}   —   lower is better; hatched + ▼ = fell, "
                 f"episode ended early", fontsize=11, pad=32)
    # Pinned upper-left rather than "best": matplotlib put it centre-top,
    # on the group annotations. hold/spin are the lowest bars on every
    # metric, so the top-left corner is reliably free.
    ax.legend(fontsize=8.5, framealpha=0.9, loc="upper left")
    ax.grid(axis="y", alpha=0.3)
    ax.set_axisbelow(True)
    ax.margins(x=0.01)

    foot = ("tick colour: green = left / +heading, orange = right / -heading, "
            "grey = neither; mirrored pairs are adjacent")
    if dropped:
        foot += (f"\n{dropped} command(s) omitted — this metric is not "
                 f"defined for them")
    sensors = (f"sensors: --ahrs {args.ahrs}"
               + (f" tau {args.ahrs_tau:g} s" if args.ahrs != "none" else "")
               + f"   |   encoder {args.encoder or 'each policy\'s own'}"
               + ("   |   POLICIES READ MUJOCO TRUTH -- no sensor model in "
                  "the loop" if args.ahrs == "none" else ""))
    fig.text(0.5, -0.15, sensors, ha="center", va="top", fontsize=8.5,
             color="0.45")
    fig.text(0.5, -0.21, foot, ha="center", va="top", fontsize=8.5,
             color="0.45")
    if note:
        fig.text(0.5, -0.30, note, ha="center", va="top", fontsize=8.5,
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
