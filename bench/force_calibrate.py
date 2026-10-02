"""Calibrate the force sensors with known weights, one enter per reading.

Order: all unloaded, then each weight across the sensors, then all unloaded.
Enter records the last 0.5 s of all four channels; r redoes, q stops and fits.
Fit: each loaded reading minus that channel's unloaded readings either side
(cancels zero drift), through the origin. Readings go to bench/logs/.

    python bench/force_calibrate.py --weights 28.5,33,128,145,182
    python bench/force_calibrate.py --refit bench/logs/force_cal_20261002-0044.csv
"""
from __future__ import annotations

import argparse
import collections
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from force_sensor import CHANNELS, NOMINAL_N_PER_COUNT, find_port, lines  # noqa: E402

G = 9.80665
WINDOW_S = 0.5
UNSETTLED_SD = 3.0      # counts; noisier = the weight was still moving


class Stream:
    """The last ~2 s of samples, kept by a background thread."""

    def __init__(self, port: str):
        self.buf = collections.deque(maxlen=20000)
        self.lock = threading.Lock()
        self.error = None
        threading.Thread(target=self._run, args=(port,), daemon=True).start()

    def _run(self, port):
        try:
            for t, c in lines(port):
                with self.lock:
                    self.buf.append((t, c))
        except BaseException as e:                     # lines() exits via SystemExit
            self.error = e

    def window(self, seconds=WINDOW_S):
        with self.lock:
            data = list(self.buf)
        if self.error is not None or not data:
            sys.exit(f"sensor stream stopped: {self.error}")
        t_end = data[-1][0]
        c = np.array([c for t, c in data if t >= t_end - seconds], dtype=float)
        return c.mean(axis=0), c.std(axis=0), len(c)


def grams(spec: str) -> float:
    return sum(float(x) for x in spec.split("+"))


def marker(k) -> str:
    return "[" + "".join("X" if i == k else "O" for i in range(4)) + "]"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--weights", default="34,38,137",
                    help="grams, comma-separated; '+' for a stack, e.g. 34+137")
    ap.add_argument("--sensors", default="1,2,3,4", help="which, 1-4 left to right")
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=None, help="CSV (default bench/logs/force_cal_<time>.csv)")
    ap.add_argument("--refit", default=None, help="fit a saved CSV instead of measuring")
    args = ap.parse_args()
    if args.refit:
        with open(args.refit) as f:
            rows = [{k: (v if k in ("time",) else float(v)) for k, v in r.items()}
                    for r in csv.DictReader(f)]
        for r in rows:
            r["sensor"] = int(r["sensor"])
        fit(rows, [k - 1 for k in sorted({r["sensor"] for r in rows if r["sensor"]})])
        return
    weights = args.weights.split(",")
    sensors = [int(s) - 1 for s in args.sensors.split(",")]
    out = Path(args.out or Path(__file__).resolve().parent / "logs"
               / f"force_cal_{datetime.now():%Y%m%d-%H%M}.csv")

    stream = Stream(args.port or find_port())
    time.sleep(1.0)
    steps = [(None, "0")] + [(k, w) for w in weights for k in sensors] + [(None, "0")]
    rows, i = [], 0
    print(f"{len(steps)} steps.")
    print(f"keys:  enter = record the last {WINDOW_S} s   r + enter = redo the previous   "
          "q + enter = stop and fit\n")
    while i < len(steps):
        k, w = steps[i]
        label = (f"{'0':>8} g  {'all unloaded':13s}" if k is None
                 else f"{w:>8} g  sensor {k + 1} ({CHANNELS[k]:>2})")
        try:
            ans = input(f"{label}  {marker(k)}  enter> ").strip().lower()
        except EOFError:
            ans = "q"
        if ans == "q":
            break
        if ans == "r":
            if rows:
                rows.pop()
                i -= 1
                print("  dropped the previous reading")
            continue
        mean, sd, n = stream.window()
        rows.append(dict(sensor=0 if k is None else k + 1, grams=grams(w), step=i,
                         **{f"{ch}_counts": round(m, 2) for ch, m in zip(CHANNELS, mean)},
                         **{f"{ch}_sd": round(s, 2) for ch, s in zip(CHANNELS, sd)},
                         samples=n, time=datetime.now().isoformat(timespec="seconds")))
        shown = range(4) if k is None else [k]
        print("  " + "  ".join(f"{CHANNELS[j]} {mean[j]:.1f}" for j in shown)
              + f" counts (sd {max(sd[j] for j in shown):.1f}, {n} samples)")
        i += 1

    if not rows:
        return
    with open(out, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print(f"\nwrote {out}\n")

    fit(rows, sensors)


def fit(rows, sensors) -> None:
    """force [N] = scale * rise [counts] per sensor, through the origin."""
    print(f"{'sensor':>8} {'N/count':>9} {'vs nominal':>10} {'worst resid':>12} "
          f"{'zero return':>12} {'other ch moved':>15}")
    scales = {}
    blank = [x for x in rows if x["sensor"] == 0]
    for k in sensors:
        ch = CHANNELS[k]
        rise, force, flags = [], [], []
        for i, r in enumerate(rows):
            if r["sensor"] != k + 1:
                continue
            nb = [rows[j][f"{ch}_counts"] for j in (i - 1, i + 1)
                  if 0 <= j < len(rows) and rows[j]["sensor"] != k + 1]
            if not nb:
                continue
            rise.append(r[f"{ch}_counts"] - np.mean(nb))
            force.append(r["grams"] * G * 1e-3)
            if r[f"{ch}_sd"] > UNSETTLED_SD:
                flags.append(f"{r['grams']:g} g (sd {r[f'{ch}_sd']:.1f})")
        if not rise:
            continue
        rise, force = np.array(rise), np.array(force)
        scale = float(rise @ force / (rise @ rise))
        resid = force - scale * rise
        ret = ((blank[-1][f"{ch}_counts"] - blank[0][f"{ch}_counts"]) * scale
               if len(blank) >= 2 else float("nan"))
        loaded = [x for x in rows if x["sensor"] == k + 1]
        heavy = max(loaded, key=lambda x: x["grams"])
        cross = max(abs(heavy[f"{o}_counts"] - np.mean([x[f"{o}_counts"] for x in blank]))
                    * NOMINAL_N_PER_COUNT for j, o in enumerate(CHANNELS) if j != k) \
            if blank else float("nan")
        scales[k] = scale
        print(f"{k + 1:>4} ({ch:>2}) {scale:9.6f} {scale / NOMINAL_N_PER_COUNT:9.2f}x "
              f"{np.abs(resid).max():10.4f} N {ret:+10.4f} N {cross:13.4f} N")
        if flags:
            print(f"         still moving when recorded: {', '.join(flags)}")
    if len(scales) == 4:
        print("\n--scale " + ",".join(f"{scales[k]:.6f}" for k in range(4)))
    print(f"(nominal FS2050 at a 3.3 V supply: {NOMINAL_N_PER_COUNT:.6f} N/count)")


if __name__ == "__main__":
    main()
