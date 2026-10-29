"""Is the detent's phase a fixed function of the drive encoders?

    python analysis/servo/detent_phase.py                     # every diff creep capture
    python analysis/servo/detent_phase.py 261028-* 261024-202227_creep

The claim under test (physical-bike-tests.md, item 6): the sawtooth flip sits
at a fixed ring-vs-hub angle mod 7.5 deg in the raw encoder frame,
BELT (sa P_A - sb P_B) / 2, whatever the power-up turn count (one servo turn
is 72 whole notches), the wheel's angle, or which way gravity points. If it
holds, a policy can be told the phase as one number from Present Position.

Per capture, from `drivetrain_trace.bench_detent` (where the diff creep's
jumps start, each way): the + and - start phases, the flip midway between,
and R, the circular concentration of each side's starts (1 = every jump
starts at the same phase, ~0 = spread evenly). `power-up` is the turn count
at the first frame: a capture taken within a few turns of power-up reads 0.
The flip spread across captures is the result; compare it with 7.5 deg and
with the + / - gap (the backlash between the two directions)."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "policy")); sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402

import drivetrain_fit as F  # noqa: E402
import drivetrain_trace as D  # noqa: E402

ROOT = HERE.parents[1]


def _is_diff_creep(d):
    import json
    try:
        m = json.loads((d / "meta.json").read_text())
    except (OSError, ValueError):
        return False
    return m.get("test") == "creep" and any(
        s["info"].get("kind") == "creep" and s["info"].get("mode") == "diff"
        for s in m["segments"])


def row(name):
    b = D.bench_detent(name)
    cap = F._load(name)
    A, B = (F._fill(cap.position_rad(i)) for i in b["ids"])
    turns = [int(np.floor(x[0] / (2 * np.pi))) for x in (A, B)]
    hub = np.degrees(F.BELT * (A[0] - B[0]) / 2) % 360
    Rp = D._circ(b["starts_plus"] % 7.5)[1]
    Rm = D._circ(b["starts_minus"] % 7.5)[1]
    return (name, cap.meta.get("note", ""), turns, hub, b["plus"], b["minus"],
            (b["minus"] - b["plus"]) % 7.5, b["flip"], Rp, Rm,
            len(b["starts_plus"]), len(b["starts_minus"]))


def main(argv):
    tr = ROOT / "traces" / "drivetrain"
    if argv:
        names = sorted({p.name for a in argv for p in tr.glob(a)})
    else:
        names = sorted(d.name for d in tr.iterdir() if _is_diff_creep(d))
    rows = []
    print(f"{'capture':28s} {'turns A,B':>12s} {'hub':>5s} {'+start':>7s} {'-start':>7s}"
          f" {'gap':>5s} {'FLIP':>6s} {'R+':>5s} {'R-':>5s} {'n+/n-':>7s}  note")
    for n in names:
        try:
            r = row(n)
        except Exception as e:                         # a capture with no diff creep
            print(f"{n:28s} skipped: {e}")
            continue
        rows.append(r)
        (n, note, t, hub, p, m, gap, flip, Rp, Rm, np_, nm) = r
        print(f"{n:28s} {t[0]:>5d},{t[1]:>5d} {hub:5.0f} {p:7.2f} {m:7.2f} {gap:5.2f}"
              f" {flip:6.2f} {Rp:5.2f} {Rm:5.2f} {np_:>3d}/{nm:<3d}  {note[:40]}")
    if len(rows) > 1:
        flips = np.array([r[7] for r in rows])
        mean, Rf = D._circ(flips)
        dev = (flips - mean + 3.75) % 7.5 - 3.75
        print(f"\nflip over {len(rows)} captures: {mean:.2f} deg mod 7.5, "
              f"max |dev| {abs(dev).max():.2f} deg ({abs(dev).max() / 7.5:.1%} of a notch), "
              f"R {Rf:.3f}")


if __name__ == "__main__":
    main(sys.argv[1:])
