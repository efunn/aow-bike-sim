"""B1: the righting servo at stall, Goal Current stepped, the crank held.

docs/plans/righting-policy-integration.md, "Bench tests". The model
(righting_servo.py) says a 785 mA stall runs the duty at ~0.8; the bike
pinned PWM 885 (100 %) at 670-750 mA (runs 261102-204438, -231204). Which
is right sets where the righting env's `servo_strength` 1.0 sits.

The operator HOLDS the crank (or a blade) still. Per side: the goal goes
`--lead` deg past where it is held, as the righting policy's 40 deg lead
holds it, and Goal Current steps through `--levels`, `--hold` s each, back
at the `--preload` (100 mA) for `--rest` s between, which first takes up any
play between the hold and the crank. Logged per sample: Present PWM, Current, Velocity,
Position, Input Voltage, Temperature (one 23-byte read, 124..146).
Reported per level, over the last 60 % of the hold: median PWM [% of
885], median current [mA], net creep [deg], volts, C, and the MODEL's
duty for that current at that voltage at stall, k (I - I0) / (ts V/12).

A hand hold is good enough because it is MEASURED: the creep column says
how still it was (servo_stall.py's argument). Safety: the current goes
to 0 and the goal back to the crank if it moves more than `--abort-deg`
from the hold, on any Hardware Error, or past `--max-temp`.

    python analysis/servo/righting_stall.py --port /dev/cu.usbserial-XXXX

Writes traces/servo_stall/righting_b1_<time>.csv (gitignored, synced).
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

from aow_sim.hw.dynamixel import DynamixelBus, signed  # noqa: E402

BLOCK_ADDR, BLOCK_LEN = 124, 23        # Present PWM .. Present Temperature
PWM_FULL = 885
POS_RAD = 2 * math.pi / 4096
VEL_RAD_S = 0.229 * 2 * math.pi / 60


def read_block(bus, i):
    data, rc, err = bus._packet.readTxRx(bus._port, i, BLOCK_ADDR, BLOCK_LEN)
    if rc != 0:
        raise RuntimeError(f"block read rc={rc} err={err}")
    u = lambda o, n: int.from_bytes(bytes(data[o:o + n]), "little")  # noqa: E731
    return {"pwm": signed(u(0, 2), 2) / PWM_FULL,
            "ma": signed(u(2, 2), 2),
            "vel": signed(u(4, 4), 4) * VEL_RAD_S,
            "pos": signed(u(8, 4), 4) * POS_RAD,
            "volts": u(20, 2) * 0.1, "temp": u(22, 1), "err": err}


def model_duty(params, ma, volts):
    srv = params["servos"]["xc330_t181"]
    k, i0, ts = (float(srv["current_torque_gain"]), float(srv["current_deadband"]),
                 float(srv["stall_torque"]))
    return k * max(0.0, ma / 1000.0 - i0) / (ts * volts / 12.0)


def limp(bus, i, pos):
    bus.write_raw(i, "Goal Current", 0)
    bus.write_raw(i, "Goal Position", int(round(pos / POS_RAD)))


def side(bus, i, sign, args, params, rows):
    hw = bus.read_raw(i, "Hardware Error Status")
    if hw:
        raise SystemExit(f"Hardware Error Status {hw}: reboot the servo first")
    pos0 = read_block(bus, i)["pos"]
    limp(bus, i, pos0)
    bus.write_raw(i, "Torque Enable", 1)
    word = "+ (right blade down)" if sign > 0 else "- (left blade down)"
    print(f"\nSIDE {word}: the crank will push {'+' if sign > 0 else '-'}.")
    print("  Hold it still; a 100 mA preload takes up any play first.")
    input("  Press Enter when holding... ")
    pos0 = read_block(bus, i)["pos"]
    goal = pos0 + sign * math.radians(args.lead)
    out = []
    try:
        # PRELOAD: push gently to take up the play between the hold and the
        # crank, then measure creep and the abort from where it settles. The
        # first run (261103-023651) aborted at 200 mA on both sides, the crank
        # accelerating freely to 5.7 rad/s over 15 deg in 48 ms.
        bus.write_raw(i, "Goal Position", int(round(goal / POS_RAD)))
        bus.write_raw(i, "Goal Current", int(args.preload))
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.preload_s:
            s = read_block(bus, i)
            if s["err"] or abs(s["pos"] - pos0) > math.radians(args.preload_deg):
                limp(bus, i, s["pos"])
                print(f"  ABORT in the preload ({args.preload} mA): moved "
                      f"{math.degrees(s['pos'] - pos0):+.1f} deg -- not held; "
                      "current off")
                return out
        s = read_block(bus, i)
        print(f"  preload {args.preload} mA: took up "
              f"{math.degrees(s['pos'] - pos0):+.1f} deg, holding at "
              f"{s['ma']:+d} mA")
        pos0 = s["pos"]
        goal = pos0 + sign * math.radians(args.lead)
        for level in args.levels:
            bus.write_raw(i, "Goal Position", int(round(goal / POS_RAD)))
            bus.write_raw(i, "Goal Current", int(level))
            t0 = time.monotonic()
            samples = []
            while (t := time.monotonic() - t0) < args.hold:
                s = read_block(bus, i)
                s.update(t=t, level=level, sign=sign, hold_pos=pos0)
                samples.append(s)
                rows.append(s)
                if s["err"] or abs(s["pos"] - pos0) > math.radians(args.abort_deg) \
                        or s["temp"] > args.max_temp:
                    limp(bus, i, s["pos"])
                    why = ("hardware error" if s["err"] else
                           f"temperature {s['temp']} C" if s["temp"] > args.max_temp
                           else f"moved {math.degrees(s['pos'] - pos0):+.1f} deg")
                    print(f"  ABORT at {level} mA: {why}; current off")
                    return out
            bus.write_raw(i, "Goal Current", int(args.preload))   # keep the play taken up
            tail = samples[int(0.4 * len(samples)):]
            med = lambda k: sorted(x[k] for x in tail)[len(tail) // 2]  # noqa: E731
            r = {"sign": sign, "level": level, "n": len(samples),
                 "pwm": med("pwm") * sign, "ma": med("ma") * sign,
                 "creep_deg": math.degrees(samples[-1]["pos"] - samples[0]["pos"]) * sign,
                 "volts": med("volts"), "temp": med("temp")}
            r["model"] = model_duty(params, r["ma"], r["volts"])
            out.append(r)
            print(f"  {level:4d} mA goal: PWM {100 * r['pwm']:5.1f} %  "
                  f"current {r['ma']:4.0f} mA  creep {r['creep_deg']:+5.1f} deg  "
                  f"{r['volts']:.1f} V {r['temp']:.0f} C  | model duty "
                  f"{100 * r['model']:5.1f} %  ({r['n']} samples)")
            time.sleep(args.rest)
    finally:
        limp(bus, i, read_block(bus, i)["pos"])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, default=104)
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--lead", type=float, default=40.0, help="goal past the hold [deg]")
    ap.add_argument("--levels", type=int, nargs="+",
                    default=[200, 300, 400, 500, 600, 700, 785])
    ap.add_argument("--hold", type=float, default=0.5, help="s per level")
    ap.add_argument("--rest", type=float, default=1.5, help="s limp between levels")
    ap.add_argument("--abort-deg", type=float, default=10.0,
                    help="creep from the preloaded hold that cuts the current")
    ap.add_argument("--preload", type=int, default=100, help="mA")
    ap.add_argument("--preload-s", type=float, default=1.0)
    ap.add_argument("--preload-deg", type=float, default=30.0,
                    help="play the preload may take up before it gives up")
    ap.add_argument("--max-temp", type=float, default=50.0)
    ap.add_argument("--sides", default="+-", help="'+', '-' or '+-'")
    args = ap.parse_args()

    from aow_sim.build_model import load_params
    params = load_params()
    from aow_sim.params import XC330_K5
    kp_a = params["righting"]["module"]["gains"]["Position P Gain"] * XC330_K5
    print(f"position loop at a {args.lead:.0f} deg lead asks "
          f"{1000 * kp_a * math.radians(args.lead):.0f} mA (model gains) -- above "
          "that level the cap binds, below it the loop does")

    stamp = time.strftime("%y%m%d-%H%M%S")
    out_csv = ROOT / "traces" / "servo_stall" / f"righting_b1_{stamp}.csv"
    rows, results = [], []
    with DynamixelBus(args.port, baud=args.baud, ids=(args.id,)) as bus:
        i = args.id
        mode = bus.read_raw(i, "Operating Mode")
        if mode != 5:
            raise SystemExit(f"id {i} is in mode {mode}, not 5 (current-based position)")
        t = time.monotonic()
        for _ in range(20):
            read_block(bus, i)
        per = (time.monotonic() - t) / 20 * 1000
        print(f"one block read: {per:.1f} ms"
              + ("  (slow: run `adjust-ftdi-latency` after a replug)" if per > 5 else ""))
        try:
            for ch in args.sides:
                results += side(bus, i, 1 if ch == "+" else -1, args, params, rows)
        finally:
            bus.write_raw(i, "Goal Current", 0)
            bus.write_raw(i, "Torque Enable", 0)
            print("\ntorque off")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["sign", "level", "t", "pwm", "ma", "vel",
                                          "pos", "hold_pos", "volts", "temp", "err"])
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in w.fieldnames})
    print(f"wrote {out_csv.relative_to(ROOT)}")
    if results:
        print("\nside  goal mA   PWM %   mA    model %   creep")
        for r in results:
            print(f"  {'+' if r['sign'] > 0 else '-'}   {r['level']:5d}  "
                  f"{100 * r['pwm']:6.1f}  {r['ma']:5.0f}  {100 * r['model']:7.1f}  "
                  f"{r['creep_deg']:+5.1f}")


if __name__ == "__main__":
    main()
