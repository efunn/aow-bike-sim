"""PufferLib's headless dashboard log -> one row per N epochs. No deps.

    python analysis/gpu/phase3/train_curve.py results/train.log [--every 10]

Each dashboard panel is one epoch. The LAST panels of a finished run ("To
go 0ms", difficulty 0.000) are PufferLib's end-of-run evaluation, not
training -- read the rows before them. What to watch: kl and clipfrac
(PPO is usually comfortable at kl ~0.01-0.05; the 2026-10-15 run sat at
0.7-2.5 before it went to NaN), entropy (collapse below ~0.2 is a warning),
episode_length / fall_rate, and difficulty (shared curriculum: it should
climb once episodes reach 750 steps). pg_grad and s/pg come from the
patched trainer's "AOW_GRAD" line (puffer_aow.patch): PPO's gradient on the
policy mean, and the temporal term's as a fraction of it ("-" before it).
"""
import argparse
import re

KEYS = [("Epoch", "epoch"), ("Steps", "steps"), ("SPS", "sps"),
        ("episode_length", "len"), ("episode_return", "ret"), ("fall_rate", "fall"),
        ("track", "track"), ("difficulty", "diff"), ("entropy", "ent"),
        (" kl", "kl"), ("clipfrac", "clip"), ("AOW_GRAD pg", "pg_grad"),
        ("ratio", "s/pg")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--every", type=int, default=10)
    a = ap.parse_args()
    text = open(a.log, errors="replace").read()
    panels = text.split("PufferLib 5.0")[1:]
    print(f"{len(panels)} panels")
    print("  ".join(f"{s:>8}" for _, s in KEYS))
    for i, p in enumerate(panels):
        if i % a.every and i != len(panels) - 1:
            continue
        row = []
        for k, _ in KEYS:
            m = re.search(re.escape(k) + r"\s+(-?[\d.]+(?:e[+-]?\d+)?[KM]?)", p)
            row.append(m.group(1) if m else "-")
        print("  ".join(f"{v:>8}" for v in row))


if __name__ == "__main__":
    main()
