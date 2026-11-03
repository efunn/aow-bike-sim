"""B2b: how fast the righting servo's overload charge drains between pushes.

docs/plans/righting-policy-integration.md, B2. The trip law is measured
(t = 0.645 A s / (I - 0.413 A)); how fast it RECOVERS was not -- every B2
trip was cleared by a reboot. Back-to-back get-ups, pumping and the
righting env's `cool_s` hang on it.

Per gap: stall at `--level` for `--charge` s (700 mA x 1.5 s is ~2/3 of a
trip by the law), then HOLD STILL for the gap -- the goal at the crank,
Goal Current capped at `--gap-current` (300 mA, under the 413 mA floor),
the operator relaxing -- then stall at `--level` again until it trips.
Every change is counted down `--warn` s ahead, the operator's hand first.
NOT limp in the gap: the first version went to 0 mA with a hand still
pushing, the crank sprang 16-25 deg, the next push snapped it back, and
every trial tripped the slip guard (run 261103-031753). With no recovery the second trip
comes after 0.75 s, with full recovery after the fresh 2.25; the charge
recovered in the gap is read off between. A reboot clears each trip (B2
showed it does: the next level's time fit the law) before the next gap.

    python analysis/servo/righting_recovery.py --port /dev/cu.usbserial-XXXX --side -
    ...  --gap-current 100      # a lower hold in the gap
    ...  --gaps 2 --warm 45     # one more, once the servo is at 45 C+

Writes traces/servo_stall/righting_b2b_<time>.csv, also on a crash.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.hw.dynamixel import DynamixelBus, describe_hardware_error  # noqa: E402
from righting_overload import save, write_gains  # noqa: E402
from righting_stall import POS_RAD, limp, read_block  # noqa: E402


FLOOR_A, CHARGE_AS = 0.413, 0.645       # B2's law
STOW_DEG, RIGHTING_SIGN = 180.0, 1       # config/onboard.yaml righting_stow_deg / _sign


def schedule(charge, gap, warn):
    """-> (push1, hold, push2) start times [s from Enter] and the operator's
    cues as [(time, text)]. EVERY change is announced `warn` s ahead with a
    countdown, so the hand moves before the servo does (user, 11-03: "the
    control input from ME needs to come before the control change from
    YOU"). A gap shorter than `warn` puts the second push's cue inside the
    first push. No terminal bell."""
    p1 = 1.0 + warn                      # the preload, then the cue
    h = p1 + charge
    p2 = h + gap
    cues = [(0.0, f"  preload {LEVEL[1]} mA -- you are holding it")]

    def countdown(t, what, after):
        """Cues at t - k s, never before the change before it (`after`)."""
        for k in range(int(math.ceil(warn)), 0, -1):
            if t - k > after + 1e-6:
                cues.append((t - k, f"    in {k} s: {what}"))
    countdown(p1, f"PUSH {LEVEL[0]} mA -- hold firm", 0.0)
    cues.append((p1, f"  PUSH {LEVEL[0]} mA for {charge:g} s -- hold firm"))
    if gap >= warn:
        countdown(h, "push ends, then RELAX (it holds itself)", p1)
        cues.append((h, f"  HOLD STILL {gap:g} s -- relax your grip"))
        countdown(p2, f"PUSH {LEVEL[0]} mA again -- hold firm", h)
    else:
        cues.append((p1, f"    after this push: only {gap:g} s, then PUSH "
                         "again -- KEEP holding through it"))
        cues.append((h, f"  gap {gap:g} s -- keep your grip"))
    cues.append((p2, f"  PUSH {LEVEL[0]} mA until it trips"))
    cues.sort(key=lambda c: c[0])
    return p1, h, p2, cues


LEVEL = [700, 300]   # --level, --preload; set in main, read by schedule's cues


def to_start(bus, i, args):
    """The crank to `--start-deg` before every trial, gently, counted down
    first (hands off), so each trial starts from the same place -- not from
    wherever the last push or slip left it (user, 11-03)."""
    want = math.radians(STOW_DEG + RIGHTING_SIGN * args.start_deg)
    if abs(read_block(bus, i)["pos"] - want) < math.radians(2.0):
        return
    for k in range(int(math.ceil(args.warn)), 0, -1):
        print(f"    in {k} s: crank to the start ({args.start_deg:+.0f} deg) "
              "-- hands off", flush=True)
        time.sleep(1.0)
    bus.write_raw(i, "Goal Current", int(args.start_current))
    bus.write_raw(i, "Goal Position", int(round(want / POS_RAD)))
    t0 = time.monotonic()
    while abs(read_block(bus, i)["pos"] - want) > math.radians(2.0):
        if time.monotonic() - t0 > 5.0:
            print("    did not reach the start in 5 s; carrying on from here")
            break
    time.sleep(0.3)
    here = read_block(bus, i)["pos"]
    limp(bus, i, here)
    print(f"  at the start: crank {math.degrees(here) - STOW_DEG:+.1f} deg", flush=True)


def trial(bus, i, sign, gap, args, gains, rows):
    if bus.read_raw(i, "Hardware Error Status"):
        bus.reboot(ids=(i,), settle_s=1.0)
    write_gains(bus, i, gains)
    temp = read_block(bus, i)["temp"]
    pos0 = read_block(bus, i)["pos"]
    limp(bus, i, pos0)
    bus.write_raw(i, "Torque Enable", 1)
    to_start(bus, i, args)
    p1, h, p2, cues = schedule(args.charge, gap, args.warn)
    print(f"\ngap {gap:g} s ({temp} C): preload, PUSH {args.level} mA "
          f"{args.charge:g} s, hold still {gap:g} s, PUSH until it trips. "
          f"Every change is counted down {args.warn:g} s ahead.")
    input("  hold it, then press Enter... ")
    t0 = time.monotonic()
    ref = read_block(bus, i)["pos"]
    bus.write_raw(i, "Goal Position",
                  int(round((ref + sign * math.radians(args.lead)) / POS_RAD)))
    bus.write_raw(i, "Goal Current", int(args.preload))   # take up the play
    phase, nxt, gap_ma, trip, why = "pre", 0, [], None, None
    while True:
        t = time.monotonic() - t0
        while nxt < len(cues) and cues[nxt][0] <= t:
            print(cues[nxt][1], flush=True)
            nxt += 1
        s = read_block(bus, i)
        if phase == "pre" and t >= p1:
            ref = s["pos"]                              # play taken up
            bus.write_raw(i, "Goal Position",
                          int(round((ref + sign * math.radians(args.lead)) / POS_RAD)))
            bus.write_raw(i, "Goal Current", int(args.level))
            phase = "push1"
        elif phase == "push1" and t >= h:
            bus.write_raw(i, "Goal Current", int(args.gap_current))
            bus.write_raw(i, "Goal Position", int(round(s["pos"] / POS_RAD)))
            phase = "gap"
        elif phase == "gap" and t >= p2:
            ref = s["pos"]
            bus.write_raw(i, "Goal Position",
                          int(round((ref + sign * math.radians(args.lead)) / POS_RAD)))
            bus.write_raw(i, "Goal Current", int(args.level))
            phase = "push2"
        s.update(t=t, level={"pre": args.preload, "push1": args.level, "gap": args.gap_current,
                             "push2": args.level}[phase],
                 sign={"pre": 0, "push1": 1, "gap": 2, "push2": 3}[phase], hold_pos=ref)
        rows.append(s)
        if phase == "gap":
            gap_ma.append(abs(s["ma"]))
        if s["err"]:
            why = describe_hardware_error(bus.read_raw(i, "Hardware Error Status"))
            if phase == "push2":
                trip = t - p2
            break
        if phase in ("push1", "push2") and abs(s["pos"] - ref) > math.radians(args.abort_deg):
            why = f"moved {math.degrees(s['pos'] - ref):+.1f} deg in {phase}"
            break
        if phase == "push2" and t - p2 > args.max_hold:
            why = "held to --max-hold"
            break
    try:
        limp(bus, i, read_block(bus, i)["pos"])
    except RuntimeError:
        pass
    gap_med = sorted(gap_ma)[len(gap_ma) // 2] if gap_ma else 0
    if trip is None:
        print(f"  no result: {why}; current off")
        return {"gap": gap, "t2": None, "recovered": None, "temp": temp,
                "gap_ma": gap_med}
    i_a = args.level / 1000.0
    charged = args.charge * (i_a - FLOOR_A) / CHARGE_AS
    left = 1.0 - trip * (i_a - FLOOR_A) / CHARGE_AS
    rec = charged - left
    print(f"  second trip after {trip:.2f} s ({why}): charge at the restart "
          f"{left:.2f} of a trip, {rec:+.2f} recovered of the {charged:.2f} put in "
          f"(gap current median {gap_med} mA)")
    return {"gap": gap, "t2": trip, "recovered": rec, "temp": temp,
            "gap_ma": gap_med}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=104)
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--side", choices="+-", default="-")
    ap.add_argument("--level", type=int, default=700)
    ap.add_argument("--charge", type=float, default=1.5, help="s at --level first")
    ap.add_argument("--gaps", type=float, nargs="+", default=[0.5, 1, 2, 4, 8])
    ap.add_argument("--gap-current", type=int, default=300,
                    help="Goal Current cap while holding still in the gap [mA]")
    ap.add_argument("--warm", type=float, default=None,
                    help="first wait until the servo is at least this warm [C]")
    ap.add_argument("--lead", type=float, default=40.0)
    ap.add_argument("--max-hold", type=float, default=10.0)
    ap.add_argument("--abort-deg", type=float, default=10.0)
    ap.add_argument("--preload", type=int, default=300,
                    help="mA before the first push; under B2's 413 mA floor, "
                         "so it charges nothing (100 was too light to hold "
                         "against, user 11-03)")
    ap.add_argument("--start-deg", type=float, default=0.0,
                    help="crank angle each trial starts from (0: centre)")
    ap.add_argument("--start-current", type=int, default=200,
                    help="mA for the move to the start")
    ap.add_argument("--warn", type=float, default=2.0,
                    help="every change is counted down this far ahead [s]")
    args = ap.parse_args()
    LEVEL[0], LEVEL[1] = args.level, args.preload
    if args.preload >= FLOOR_A * 1000:
        raise SystemExit(f"--preload {args.preload} mA is over the {FLOOR_A * 1000:.0f} mA "
                         "floor: it would charge the overload before the test")

    from aow_sim.build_model import load_params
    gains = load_params()["righting"]["module"]["gains"]
    sign = 1 if args.side == "+" else -1
    out = ROOT / "traces" / "servo_stall" / f"righting_b2b_{time.strftime('%y%m%d-%H%M%S')}.csv"
    rows, res = [], []
    with DynamixelBus(args.port, baud=args.baud, ids=(args.id,)) as bus:
        i = args.id
        if bus.read_raw(i, "Operating Mode") != 5:
            raise SystemExit("not in mode 5 (current-based position)")
        if args.warm and read_block(bus, i)["temp"] < args.warm:
            print(f"servo at {read_block(bus, i)['temp']} C, under --warm "
                  f"{args.warm:.0f}: warm it first (a few B2 trips), then rerun")
            return
        try:
            for gap in args.gaps:
                r = trial(bus, i, sign, gap, args, gains, rows)
                if r:
                    res.append(r)
        finally:
            save(out, rows)
            try:
                bus.write_raw(i, "Goal Current", 0)
                bus.write_raw(i, "Torque Enable", 0)
                if bus.read_raw(i, "Hardware Error Status"):
                    bus.reboot(ids=(i,), settle_s=1.0)
                    write_gains(bus, i, gains)
                print("torque off")
            except RuntimeError as e:
                print(f"CLEANUP FAILED ({e}): check the servo's torque by hand")
    if res:
        fresh = CHARGE_AS / (args.level / 1000 - FLOOR_A)
        print(f"\n level {args.level} mA, charge {args.charge} s, gap at "
              f"<= {args.gap_current} mA held; fresh trip {fresh:.2f} s, none recovered "
              f"{fresh - args.charge:.2f} s")
        print("  gap s   2nd trip s   recovered   gap mA   C")
        for r in res:
            t2 = f"{r['t2']:.2f}" if r["t2"] else "  --"
            rc = f"{r['recovered']:+.2f}" if r["recovered"] is not None else "  --"
            print(f"  {r['gap']:5g}   {t2:>9}   {rc:>9}   {r['gap_ma']:6.0f}   "
                  f"{r['temp']}")


if __name__ == "__main__":
    main()
