"""Follow a LIVE PufferLib train.log: one row per checkpoint, alarms at once. No deps.

    python3 analysis/gpu/phase3/watch.py results/train.log --every 25 --until results/ALL_DONE

Runs on the pod (plain python3); stream it from here with
`ssh ... 'cd ~/aow && python3 analysis/gpu/phase3/watch.py ...'`. A row every
`--every` epochs (25 = aow_bike.ini's checkpoint_interval, ~6.5M steps), the
same columns as train_curve.py plus a verdict:

  NAN       losses nan, or kl and clipfrac exactly 0 with entropy at its
            start value -- the 2026-10-15 1 h run's dead signature. Printed
            on the first panel that shows it, not at the next checkpoint.
  KL HIGH   kl > 0.1 (PPO usually sits ~0.01-0.05; that run sat 0.7-2.5).
  ENT LOW   entropy < 0.2.

Exits when the --until file appears (after printing the last panel seen).
"""
import argparse
import os
import re
import sys
import time

from train_curve import KEYS


def field(panel, key):
    m = re.search(re.escape(key) + r"\s+(-?[\d.]+(?:e[+-]?\d+)?[KM]?|nan|-?inf)", panel)
    return m.group(1) if m else None


def verdict(p):
    loss = field(p, "Losses")
    kl, clip, ent = (field(p, k) for k in (" kl", "clipfrac", "entropy"))
    if loss in ("nan", "inf", "-inf") or "nan" in (kl, ent):
        return "NAN"
    try:
        kl_f, clip_f, ent_f = float(kl), float(clip), float(ent)
    except (TypeError, ValueError):
        return ""
    epoch = int(field(p, "Epoch") or 0)
    if epoch > 5 and kl_f == 0 and clip_f == 0 and ent_f > 4.2:
        return "NAN"
    flags = []
    if kl_f > 0.1:
        flags.append("KL HIGH")
    if ent_f < 0.2:
        flags.append("ENT LOW")
    return " ".join(flags) or "ok"


def row(p):
    return "  ".join(f"{field(p, k) or '-':>8}" for k, _ in KEYS) + "  " + verdict(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--every", type=int, default=25)
    ap.add_argument("--until", help="stop once this file exists")
    a = ap.parse_args()
    print("  ".join(f"{s:>8}" for _, s in KEYS) + "  verdict", flush=True)
    buf, pos, last_bucket, alarmed, last = "", 0, -1, False, None
    while True:
        done = a.until and os.path.exists(a.until)
        if os.path.exists(a.log):
            with open(a.log, errors="replace") as f:
                f.seek(pos)
                buf += f.read()
                pos = f.tell()
        parts = buf.split("PufferLib 5.0")
        buf = "PufferLib 5.0" + parts[-1] if len(parts) > 1 else buf
        for p in parts[1:] if done else parts[1:-1]:
            last = p
            epoch = int(field(p, "Epoch") or 0)
            v = verdict(p)
            if v == "NAN" and not alarmed:
                alarmed = True
                print(row(p) + "   <-- first dead panel", flush=True)
            elif epoch // a.every != last_bucket:
                last_bucket = epoch // a.every
                print(row(p), flush=True)
        if done:
            if last is not None:
                print("end: " + row(last), flush=True)
            return
        time.sleep(5)


if __name__ == "__main__":
    sys.exit(main())
