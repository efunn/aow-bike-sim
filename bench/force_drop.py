"""Catch wheel drops on the force sensors; per impact: peak, contact, bounce.

Arms after 0.2 s with all four sensors unloaded; the next rise past 0.1 N on
any sensor is a drop. Zero: that 0.2 s. r + enter discards the last drop, q stops.
Heights are labels in order (--order cycle for the drop rig's cam).

  e_measured   first-impact restitution, no drop height needed: flight time
               gives v_out, impulse gives v_in. Needs the impact mass (--mass-g,
               else the resting force). Synthetic drops: +0.01 to +0.06 high.
  h_measured   the drop height that implies
  e_ratio      second flight / first: no height or mass, second impact
  e_flight, e_impulse   from the set height; only as good as that height
  clean        a real flight after the first impact, and e_measured within
               0.05 at a 0.2 N edge; otherwise the bounce numbers are blank

Reading clips ~19 N; sensor safe load ~37 N. Raw + summary CSVs to bench/logs/.

    python bench/force_drop.py --wheel front --mass-g 86 --heights 1,2 --repeats 5
    python bench/force_drop.py --wheel front --mass-g 86 --heights 0.5,1,1.5,2 --order cycle
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import threading  # noqa: E402

from force_calibrate import Stream, marker  # noqa: E402
from force_sensor import ADC_MAX, CALIBRATED_N_PER_COUNT, CHANNELS, find_port  # noqa: E402

G = 9.80665
ARM_S = 0.2               # arms after this long unloaded; also each drop's zero reference
PRE_S, POST_S = ARM_S, 1.4
HIT_N = 0.1               # contact edge; the mount rings +-0.05 N
MAX_CONTACT_MS = 15.0     # front: 7-9 ms measured
MIN_FLIGHT_MS = 3.0
SMALL_PEAK_N = 1.0        # considered a knock
CLIP_COUNTS = ADC_MAX - 8
WARN_N = 19.0             # 0.68 V zero + 1.98 V span runs past the 3.3 V input
SAFE_N = 36.8             # datasheet: 2.5x rated


def analyse(t, f, h0_mm, mass_g, thr=None):
    """Impacts, peak, contact time and the restitution estimates of one trace."""
    thr = HIT_N if thr is None else thr
    on = f > thr
    edges = np.flatnonzero(np.diff(on.astype(int)))
    starts = [i + 1 for i in edges if not on[i]]
    ends = [i + 1 for i in edges if on[i]]
    if on[0]:
        starts = [0] + starts
    hits = []
    for s in starts:
        e = next((x for x in ends if x > s), len(f))
        hits.append((s, e))
    # gaps under 1 ms are one contact
    merged = []
    for s, e in hits:
        if merged and t[s] - t[merged[-1][1] - 1] < 1e-3:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))
    if not merged:
        return None
    s1, e1 = merged[0]
    out = dict(peak_n=float(f[s1:e1].max()), contact_ms=(t[e1 - 1] - t[s1]) * 1e3,
               bounces=len(merged) - 1)
    rest = f[t > t[-1] - 0.2]
    resting = bool(rest.min() > thr)
    out["rest_n"] = float(rest.mean()) if resting else float("nan")
    h0 = h0_mm * 1e-3
    v_in = np.sqrt(2 * G * h0)
    if len(merged) > 1 and merged[1][0] < len(t):
        flight = t[merged[1][0]] - t[e1 - 1]
        out["flight_ms"] = flight * 1e3
        out["e_flight"] = float(np.sqrt(G * flight ** 2 / 8 / h0))
    else:
        out["flight_ms"] = out["e_flight"] = float("nan")
    if len(merged) > 2:
        flight2 = t[merged[2][0]] - t[merged[1][1] - 1]
        out["e_ratio"] = float(flight2 / (t[merged[1][0]] - t[e1 - 1]))
    else:
        out["e_ratio"] = float("nan")
    # --mass-g, else the resting force (same sensor, so a scale error cancels in J / m)
    m = mass_g * 1e-3 if mass_g else (out["rest_n"] / G if resting else float("nan"))
    out["mass_from"] = "given" if mass_g else ("rest" if resting else "none")
    out["mass_g"] = m * 1e3
    impulse = float(np.trapezoid(f[s1:e1], t[s1:e1]))
    out["impulse_mns"] = impulse * 1e3
    # J = m (v_in + v_out) + m g T
    t_c = t[e1 - 1] - t[s1]
    out["e_impulse"] = (impulse - m * G * t_c) / (m * v_in) - 1 if m == m else float("nan")
    if m == m and out["flight_ms"] == out["flight_ms"]:
        v_out = G * out["flight_ms"] * 1e-3 / 2
        v_in_m = (impulse - m * G * t_c) / m - v_out
        out["h_measured_mm"] = v_in_m ** 2 / (2 * G) * 1e3
        out["e_measured"] = float(v_out / v_in_m)
    else:
        out["h_measured_mm"] = out["e_measured"] = float("nan")
    # a wheel that rocks on the button never unloads: no flight, no bounce numbers
    out["clean"] = bool(out["contact_ms"] < MAX_CONTACT_MS
                        and out["flight_ms"] == out["flight_ms"]
                        and out["flight_ms"] >= MIN_FLIGHT_MS)
    if not out["clean"]:
        for key in ("e_flight", "e_impulse", "e_measured", "h_measured_mm", "e_ratio"):
            out[key] = float("nan")
    return out


def keys(cmds: list) -> None:
    """r / q typed at any time."""
    for line in sys.stdin:
        cmds.append(line.strip().lower())
    cmds.append("q")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--wheel", required=True, help="label, e.g. rear / front")
    ap.add_argument("--phase", default="", help="rear roller phase label, e.g. flat")
    ap.add_argument("--heights", default="1,2,3", help="mm, a label for grouping drops")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--order", choices=("block", "cycle"), default="block",
                    help="block: 1,1,1,2,2,2 (by hand); cycle: 1,2,1,2 (the drop rig's cam)")
    ap.add_argument("--mass-g", type=float, default=None,
                    help="impact mass; use it when anything supports the wheel at rest "
                         "(front wheel + fork halves weigh 86 g)")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    scale = np.array(CALIBRATED_N_PER_COUNT)
    heights = [float(h) for h in args.heights.split(",")]
    if args.order == "cycle":
        steps = [(h, r) for r in range(1, args.repeats + 1) for h in heights]
    else:
        steps = [(h, r) for h in heights for r in range(1, args.repeats + 1)]
    stem = Path(__file__).resolve().parent / "logs" / f"drops_{datetime.now():%Y%m%d-%H%M}"
    stream = Stream(args.port or find_port())
    time.sleep(1.0)

    with stream.lock:
        t_last = stream.buf[-1][0]
        zero = np.mean([c for t, c in stream.buf if t > t_last - 0.5], axis=0)
    print(f"watching all four sensors, {len(steps)} drops.")
    print("keys, any time:  r + enter = discard the last drop   q + enter = stop and save\n")

    cmds: list = []
    threading.Thread(target=keys, args=(cmds,), daemon=True).start()
    raw, summary = [], []
    seen = t_last
    quiet_since = t_last
    armed, prompted = True, False
    while len(summary) < len(steps):
        while cmds:
            c = cmds.pop(0)
            if c == "q":
                steps = steps[:len(summary)]
            elif c == "r" and summary:
                d = summary.pop()["drop"]
                raw = [x for x in raw if x["drop"] != d]
                prompted = False
                print("  discarded the last drop")
        if len(summary) >= len(steps):
            break
        h, r = steps[len(summary)]
        if armed and not prompted:
            print(f"{args.wheel:>6} {h:4g} mm  drop {r}/{args.repeats}  -- armed, drop when ready")
            prompted = True
        time.sleep(0.02)
        with stream.lock:
            new = [(t, c) for t, c in stream.buf if t > seen]
        if not new:
            continue
        seen = new[-1][0]
        hit = None
        for t, c in new:
            fn = float(((np.asarray(c) - zero) * scale).max())   # the most loaded sensor
            if fn <= HIT_N:
                if quiet_since is None:
                    quiet_since = t
                continue
            if armed and quiet_since is not None and t - quiet_since >= ARM_S:
                hit = t
                break
            quiet_since = None
        if quiet_since is not None and not armed and seen - quiet_since >= ARM_S:
            armed, prompted = True, False
        if hit is None:
            continue

        while True:
            with stream.lock:
                if stream.buf[-1][0] >= hit + POST_S:
                    data = [(t, c) for t, c in stream.buf if hit - PRE_S <= t <= hit + POST_S]
                    break
            time.sleep(0.02)
        seen = data[-1][0]
        armed, quiet_since = False, None
        t = np.array([x[0] for x in data]) - hit
        counts = np.array([x[1] for x in data], dtype=float)          # (n, 4)
        zero = counts[t < -0.002].mean(axis=0)
        force = (counts - zero) * scale
        k = int(force[t >= 0].max(axis=0).argmax())                    # the sensor it hit
        a = analyse(t, force[:, k], h, args.mass_g)
        if a is None:
            print("  impact lost, not recorded")
            continue
        a2 = analyse(t, force[:, k], h, args.mass_g, thr=2 * HIT_N)
        a["e_measured_2x_edge"] = a2["e_measured"]
        if a["clean"] and not abs(a2["e_measured"] - a["e_measured"]) <= 0.05:
            a["clean"] = False
            print(f"  marginal: e_measured {a['e_measured']:.2f} at a {HIT_N:g} N edge, "
                  f"{a2['e_measured']:.2f} at {2 * HIT_N:g} N (rocking or a slow second contact)")
        if a["peak_n"] < SMALL_PEAK_N:
            print(f"  peak only {a['peak_n']:.2f} N: probably a knock, not a drop (r + enter discards)")
        drop = len(summary) + 1
        for j in range(len(t)):
            raw.append(dict(drop=drop, t_s=round(t[j], 6),
                            **{f"{ch}_counts": int(counts[j, i]) for i, ch in enumerate(CHANNELS)},
                            **{f"{ch}_n": round(force[j, i], 4) for i, ch in enumerate(CHANNELS)}))
        others = max(force[t >= 0][:, i].max() for i in range(4) if i != k)
        summary.append(dict(drop=drop, wheel=args.wheel, phase=args.phase, height_mm=h,
                            sensor=k + 1, zero_counts=round(zero[k], 1),
                            scale_n_per_count=scale[k],
                            clipped=bool(counts[:, k].max() >= CLIP_COUNTS),
                            others_peak_n=round(others, 4),
                            **{x: (round(v, 4) if isinstance(v, float) else v)
                               for x, v in a.items()}))
        if not a["clean"]:
            print("  no clean flight after the first impact (it rocked or rolled on the"
                  " button): peak kept, bounce numbers blank")
        print(f"  sensor {k + 1} {marker(k)}: e_measured {a['e_measured']:.2f} "
              f"(height measured {a['h_measured_mm']:.2f} mm, set {h:g}), "
              f"peak {a['peak_n']:.2f} N, contact {a['contact_ms']:.1f} ms, "
              f"e_ratio {a['e_ratio']:.2f}, bounces {a['bounces']}, resting {a['rest_n']:.3f} N")
        if counts[:, k].max() >= CLIP_COUNTS:
            print("  !! ADC SATURATED: the peak is clipped; drop lower")
        elif a["peak_n"] > WARN_N * 0.8:
            print(f"  !! within 20% of the ~{WARN_N:g} N where the reading clips; "
                  f"sensor safe load ~{SAFE_N:.0f} N")
        print("  lift the wheel off to re-arm")

    if not summary:
        return
    for path, rows in ((f"{stem}.csv", raw), (f"{stem}_summary.csv", summary)):
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
