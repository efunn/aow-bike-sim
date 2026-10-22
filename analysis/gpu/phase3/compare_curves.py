"""PufferLib training curves of several runs on one figure: every dashboard
panel (one per epoch) of each train.log, against steps, against training
wall time, and against BIKE time per world -- steps / worlds x ctrl_dt, in
15 s episodes -- which lines up what each simulated bike has lived through
across world counts (the curriculum's lead and the panel's difficulty lag
are both paced in episodes, not steps).

    python analysis/gpu/phase3/compare_curves.py --tag style_4k_vs_16k \\
        "run 1, 4k x 64=analysis/gpu/phase3/results_5090_style/train.log" \\
        "run 2, 16k x 16=analysis/gpu/phase3/results_5090_mult/train.log"

Writes analysis/gpu/plots/gpu_curves_<tag>.png. The last panels of a finished
run ("To go 0ms") are PufferLib's end-of-run evaluation, not training, and
are dropped. A panel in which no episode ended has no episode metrics:
those points are left out, not drawn as zero. The world count is read from
the PufferLib ini saved beside the log (puffer_logs/aow_bike/*.ini,
total_agents).
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

METRICS = [("difficulty", "difficulty"), ("fall_rate", "fall rate"),
           ("track", "track"), ("episode_length", "episode length [steps]"),
           ("episode_return", "episode return (env units)"),
           ("entropy", "entropy"), (" kl", "kl"), ("clipfrac", "clipfrac")]
NUM = r"\s+(-?[\d.]+(?:e[+-]?\d+)?)([KM]?)"


def _num(panel, key):
    m = re.search(re.escape(key) + NUM, panel)
    if not m:
        return np.nan
    return float(m.group(1)) * {"": 1, "K": 1e3, "M": 1e6}[m.group(2)]


def _uptime(panel):
    m = re.search(r"Uptime\s+((?:\d+h\s*)?(?:\d+m\s*)?(?:\d+s\s*)?(?:\d+ms)?)", panel)
    if not m:
        return np.nan
    t = 0.0
    for v, u in re.findall(r"(\d+)(h|ms|m|s)", m.group(1)):
        t += float(v) * {"h": 3600, "m": 60, "s": 1, "ms": 1e-3}[u]
    return t


CTRL_DT = 0.02          # [s] one policy step (the graph json's ctrl_dt)
EPISODE_S = 15.0        # max_steps 750 x ctrl_dt


def worlds_for(log):
    inis = sorted(Path(log).parent.glob("puffer_logs/aow_bike/*.ini"))
    if not inis:
        raise SystemExit(f"{log}: no puffer_logs/aow_bike/*.ini beside it for total_agents")
    m = re.search(r"^total_agents\s*=\s*(\d+)", inis[-1].read_text(), re.M)
    return int(m.group(1))


def parse(path):
    panels = open(path, errors="replace").read().split("PufferLib 5.0")[1:]
    rows = []
    for p in panels:
        if re.search(r"To go\s+0ms", p):
            continue                       # the end-of-run evaluation
        r = {"steps": _num(p, "Steps"), "wall": _uptime(p)}
        for k, _ in METRICS:
            r[k] = _num(p, k)
        if not np.isnan(r["steps"]):
            rows.append(r)
    out = {k: np.array([r[k] for r in rows]) for k in rows[0]}
    out["bike"] = out["steps"] / worlds_for(path) * CTRL_DT / EPISODE_S
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("runs", nargs="+", help="LABEL=path/to/train.log")
    ap.add_argument("--tag", required=True)
    a = ap.parse_args()
    runs = [(s.split("=", 1)[0], parse(s.split("=", 1)[1])) for s in a.runs]
    fig, axes = plt.subplots(len(METRICS), 3, figsize=(17, 2.4 * len(METRICS)),
                             sharex="col")
    for i, (key, name) in enumerate(METRICS):
        for j, (xk, xs, xl) in enumerate((("steps", 1e6, "steps [M]"),
                                          ("wall", 60.0, "training wall time [min]"),
                                          ("bike", 1.0, "bike time per world [15 s episodes]"))):
            ax = axes[i, j]
            for label, d in runs:
                ok = ~np.isnan(d[key])
                ax.plot(d[xk][ok] / xs, d[key][ok], lw=1, label=label)
            ax.set_ylabel(name)
            ax.grid(alpha=0.3)
            if i == len(METRICS) - 1:
                ax.set_xlabel(xl)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(f"PufferLib training panels: {a.tag}")
    fig.tight_layout()
    out = Path(__file__).resolve().parents[3] / "analysis" / "gpu" / "plots" / f"gpu_curves_{a.tag}.png"
    fig.savefig(out, dpi=110)
    print(out)
    for label, d in runs:
        for lvl in (0.25, 0.5, 0.9):
            k = np.argmax(d["difficulty"] >= lvl) if (d["difficulty"] >= lvl).any() else None
            print(f"  {label}: difficulty >= {lvl}: "
                  + ("never" if k is None else f"{d['steps'][k] / 1e6:.1f}M steps, "
                                               f"{d['wall'][k] / 60:.1f} min, "
                                               f"{d['bike'][k]:.0f} episodes"))


if __name__ == "__main__":
    main()
