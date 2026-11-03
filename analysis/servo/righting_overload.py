"""B2: how long the righting servo holds a stall before its OVERLOAD trips.

docs/plans/righting-policy-integration.md, "Bench tests". The righting env's
overload estimate is a GUESS shaped on three bike trips (1.7 and 2.3 s at
PWM 885 and 670-750 mA; 5.8 s above 400 mA pumping). This measures the
limit directly: time to trip against Goal Current.

As B1 (righting_stall.py): the operator holds the crank -- between the
servo and the blade on the side it pushes -- a 100 mA preload takes up the
play, the goal goes `--lead` deg past the hold, then ONE level is held
until the servo trips (Hardware Error Status bit 5, overload) or
`--max-hold` s pass. After a trip: reboot (which restores RAM, so the
module's gains are written again before the next level), then wait with
torque off until the servo is back under `--cool-to` C.

B1 found PWM pins at 100 % from ~756 mA (11.8 V) while 600 mA runs at
~89 %. So the levels straddle the pin: if 785 trips and 600 holds for
`--max-hold`, the overload keys on PWM saturation, not on current.

Reported per level: time to trip (or none), time with PWM >= 99 %, median
current, PWM, volts, temperature at start and end.

    python analysis/servo/righting_overload.py --port /dev/cu.usbserial-XXXX --side -

Writes traces/servo_stall/righting_b2_<time>.csv (gitignored, synced).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from aow_sim.hw.dynamixel import DynamixelBus, describe_hardware_error  # noqa: E402
from righting_stall import POS_RAD, limp, read_block  # noqa: E402

OVERLOAD = 1 << 5


def write_gains(bus, i, gains):
    for name in ("Position P Gain", "Position I Gain", "Position D Gain"):
        if name in gains:
            bus.write_raw(i, name, int(gains[name]))


def wait_cool(bus, i, cool_to):
    t = read_block(bus, i)["temp"]
    if t <= cool_to:
        return t
    print(f"  cooling, torque off: {t} C -> {cool_to:.0f} C (you can let go)")
    while (t := read_block(bus, i)["temp"]) > cool_to:
        time.sleep(2.0)
        print(f"    {t} C", end="\r", flush=True)
    print(f"    {t} C        ")
    return t


def one_level(bus, i, sign, level, args, gains, rows):
    hw = bus.read_raw(i, "Hardware Error Status")
    if hw:
        bus.reboot(ids=(i,), settle_s=1.0)
    write_gains(bus, i, gains)
    t_start = wait_cool(bus, i, args.cool_to)
    pos0 = read_block(bus, i)["pos"]
    limp(bus, i, pos0)
    bus.write_raw(i, "Torque Enable", 1)
    input(f"\n{level} mA, pushing {'+' if sign > 0 else '-'}: hold it, "
          "then press Enter... ")
    pos0 = read_block(bus, i)["pos"]
    bus.write_raw(i, "Goal Position",
                  int(round((pos0 + sign * math.radians(args.lead)) / POS_RAD)))
    bus.write_raw(i, "Goal Current", int(args.preload))
    time.sleep(args.preload_s)
    s = read_block(bus, i)
    if abs(s["pos"] - pos0) > math.radians(args.preload_deg):
        limp(bus, i, s["pos"])
        print(f"  not held in the preload: moved "
              f"{math.degrees(s['pos'] - pos0):+.1f} deg; current off")
        return None
    pos0 = s["pos"]
    bus.write_raw(i, "Goal Position",
                  int(round((pos0 + sign * math.radians(args.lead)) / POS_RAD)))
    bus.write_raw(i, "Goal Current", int(level))
    t0 = time.monotonic()
    samples, pinned, tripped, why = [], 0.0, None, None
    t_prev = 0.0
    while (t := time.monotonic() - t0) < args.max_hold:
        s = read_block(bus, i)
        s.update(t=t, level=level, sign=sign, hold_pos=pos0)
        samples.append(s)
        rows.append(s)
        if abs(s["pwm"]) >= 0.99:
            pinned += t - t_prev
        t_prev = t
        if s["err"]:
            hw = bus.read_raw(i, "Hardware Error Status")
            tripped, why = t, describe_hardware_error(hw)
            break
        if abs(s["pos"] - pos0) > math.radians(args.abort_deg):
            why = f"moved {math.degrees(s['pos'] - pos0):+.1f} deg"
            break
        if s["temp"] > args.max_temp:
            why = f"temperature {s['temp']} C"
            break
    try:
        limp(bus, i, read_block(bus, i)["pos"])
    except RuntimeError:
        pass                                # a tripped servo may refuse writes
    body = samples[int(0.2 * len(samples)):] or samples
    med = lambda k: sorted(abs(x[k]) for x in body)[len(body) // 2]  # noqa: E731
    r = {"level": level, "sign": sign, "trip_s": tripped, "held_s": t_prev,
         "pinned_s": pinned, "ma": med("ma"), "pwm": med("pwm"),
         "volts": med("volts"), "t0": t_start, "t1": samples[-1]["temp"],
         "why": why or "held to --max-hold"}
    print(f"  {'TRIPPED at %.2f s (%s)' % (tripped, why) if tripped else 'no trip, ' + r['why']}"
          f"  PWM >= 99 % for {pinned:.2f} s, "
          f"{r['ma']:.0f} mA, PWM {100 * r['pwm']:.0f} %, {r['volts']:.1f} V, "
          f"{r['t0']}->{r['t1']} C")
    return r


def save(out_csv, rows):
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["sign", "level", "t", "pwm", "ma", "vel",
                                          "pos", "hold_pos", "volts", "temp", "err"])
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in w.fieldnames})
    print(f"wrote {out_csv.relative_to(ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=104)
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--side", choices="+-", default="-")
    ap.add_argument("--levels", type=int, nargs="+", default=[785, 700, 600])
    ap.add_argument("--lead", type=float, default=40.0)
    ap.add_argument("--max-hold", type=float, default=10.0, help="s per level")
    ap.add_argument("--cool-to", type=float, default=45.0)
    ap.add_argument("--max-temp", type=float, default=60.0)
    ap.add_argument("--abort-deg", type=float, default=10.0)
    ap.add_argument("--preload", type=int, default=100)
    ap.add_argument("--preload-s", type=float, default=1.0)
    ap.add_argument("--preload-deg", type=float, default=30.0)
    args = ap.parse_args()

    from aow_sim.build_model import load_params
    gains = load_params()["righting"]["module"]["gains"]
    sign = 1 if args.side == "+" else -1
    stamp = time.strftime("%y%m%d-%H%M%S")
    out_csv = ROOT / "traces" / "servo_stall" / f"righting_b2_{stamp}.csv"
    rows, results = [], []
    with DynamixelBus(args.port, baud=args.baud, ids=(args.id,)) as bus:
        i = args.id
        if bus.read_raw(i, "Operating Mode") != 5:
            raise SystemExit("not in mode 5 (current-based position)")
        sd = bus.read_raw(i, "Shutdown")
        print(f"Shutdown(63) = {sd}: torque-off on "
              f"{describe_hardware_error(sd) or 'nothing'}"
              + ("" if sd & OVERLOAD else "  -- OVERLOAD NOT ENABLED, no trip "
                 "will come"))
        try:
            for level in args.levels:
                r = one_level(bus, i, sign, level, args, gains, rows)
                if r:
                    results.append(r)
        finally:
            # The log first: a lost reply (run 261103-0300, rc -3001 mid-
            # preload) took the whole run's CSV with it when this came last.
            save(out_csv, rows)
            try:
                bus.write_raw(i, "Goal Current", 0)
                bus.write_raw(i, "Torque Enable", 0)
                if bus.read_raw(i, "Hardware Error Status"):
                    bus.reboot(ids=(i,), settle_s=1.0)
                    write_gains(bus, i, gains)
                    print("rebooted to clear the trip; gains written back")
                print("torque off")
            except RuntimeError as e:
                print(f"CLEANUP FAILED ({e}): check the servo's torque by hand")
    if results:
        print("\n goal mA   trip s   held s   PWM>=99% s   mA   PWM %   V     C")
        for r in results:
            trip = f"{r['trip_s']:.2f}" if r["trip_s"] else "  --"
            print(f"  {r['level']:5d}   {trip:>6}   {r['held_s']:6.2f}   "
                  f"{r['pinned_s']:9.2f}   {r['ma']:4.0f}  {100 * r['pwm']:5.0f}  "
                  f"{r['volts']:.1f}  {r['t0']}->{r['t1']}")


if __name__ == "__main__":
    main()
