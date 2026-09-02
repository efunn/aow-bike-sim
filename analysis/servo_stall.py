"""Stall test: what does a Dynamixel current loop actually regulate?

Run it while HOLDING the output with a wrench. It prints a countdown, sweeps,
then tells you to let go. Nothing needs typing mid-hold.

    python analysis/servo_stall.py --port /dev/cu.usbserial-XXXX --id 150

THE QUESTION. At zero speed there is no back-EMF, so duty*vin is entirely
across the winding and the two candidate models separate cleanly:

    phase regulation :  duty = R*I/vin        -> duty     LINEAR in I
    bus   regulation :  duty = sqrt(R*I/vin)  -> duty     SQRT   in I

Rhoban/bam's `XL330CurrentActuator` asserts the second (commit c3cbf89) and
marks it UNTESTED; it was inferred from `|duty| ~ sqrt(|error|)` in POSITION
mode, never from a commanded current. This commands the current directly.

WHY A HAND HOLD IS GOOD ENOUGH, AND WHAT IT IS NOT. The hold does not need to
be perfect, it needs to be MEASURED. Three things make that work:

  * **Net drift, not a differenced median.** At 200 Hz one encoder count is
    0.31 rad/s, so any creep slower than that differences to zero on most
    samples and a median reads exactly 0.00 -- "slower than my quantisation
    floor" reported as "stopped". An earlier version of this test made exactly
    that mistake, called a hand hold stationary, and produced an exponent that
    creep alone could have manufactured. Net rotation over a 1 s hold resolves
    0.003 rad/s.
  * **Back-EMF correction per point**, from that drift, with the correction
    printed as a percentage so it can be judged rather than trusted.
  * **Sign alternation between levels** (`--direction alternate`, the default).
    A compliant hold creeps in the direction it is pushed, and creep grows with
    torque -- which bends duty(I) sublinear all by itself. Alternating means the
    hold returns roughly to where it started instead of walking away.

    A ONE-WAY CLAMP CANNOT DO THIS. `--direction cw` / `ccw` drives one way
    throughout, which is fine, but it removes a protection: creep then
    accumulates, and the drift measurement plus the back-EMF rejection are the
    ONLY things standing between a soft hold and a fabricated exponent. Watch
    the drift column, and prefer a rigid clamp for a single-direction run.

  * **Randomised order** (`--shuffle`, on by default). Winding resistance rises
    with temperature and a clamp loosens as it is worked, and both correlate
    with TIME. Sweeping current monotonically upward makes them correlate with
    CURRENT too, which is indistinguishable from the effect being measured. The
    order is printed and the seed is settable, so a run is still reproducible.

Points whose back-EMF correction exceeds `--max-bemf` are printed and then
EXCLUDED, and a sweep that cannot hold at all refuses rather than reporting.

WHAT THIS DOES TO THE SERVO -- read before clamping anything.

At stall there is no mechanical output, so ALL electrical power becomes winding
heat: P = I^2 * R, with R somewhere in 2.9-7.1 ohm on the candidate fits. At
R = 5:

    I         P instantaneous   P at this sweep's 33% duty
     50 mA        0.013 W            0.004 W
    260 mA        0.338 W            0.113 W
    450 mA        1.013 W            0.338 W   <- default top of the sweep
   1500 mA       11.250 W            3.750 W   <- datasheet stall, NOT used

The default sweep is 9 points, 1.0 s on and >= 2.0 s off, so a third duty and
0.34 W average at the worst point. A previous run went 22 C -> 23 C across the
whole sweep. Three protections on top:

  * temperature is read from the servo and checked BEFORE and after every
    point, aborting at `--max-temp` (default 45 C) -- 25 C below the servo's own
    Temperature Limit of 70, and its Shutdown register already has the overheat
    bit set, so the firmware is a second backstop that owes nothing to this code
  * off-time is `--cool` multiples of on-time, enforced between every point
  * current is RAMPED over `--ramp` rather than stepped, because a step to
    450 mA arrives as full torque in one control period and kicks whatever is
    holding the horn

`--dry-run` walks the whole sequence, printing every command and delay, without
ever enabling torque. Run that first.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "src")

from aow_sim.hw.dynamixel import DynamixelBus, IndirectMap   # noqa: E402

CPR = 4096.0


def net_drift(rows, dxl_id, lo=0.0, hi=1.0) -> float:
    """Net rotation over a fraction of the capture, in encoder counts."""
    pos = np.array([r["servos"][dxl_id]["Present Position"] for r in rows], float)
    c = pos * CPR / (2 * np.pi)
    d = ((np.diff(c) + CPR // 2) % CPR) - CPR // 2
    a, b = int(lo * len(d)), int(hi * len(d))
    return float(np.sum(d[a:b]))


def med(rows, dxl_id, key, lo=0.0) -> float:
    v = [abs(r["servos"][dxl_id][key]) for r in rows]
    return float(np.median(v[int(lo * len(v)):]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--currents", type=float, nargs="+",
                    default=[0.02, 0.03, 0.05, 0.08, 0.12, 0.18, 0.26, 0.36, 0.45],
                    help="amps")
    ap.add_argument("--direction", choices=("alternate", "cw", "ccw"),
                    default="alternate",
                    help="alternate flips sign level to level so a compliant "
                         "hold walks back; cw/ccw drive one way throughout, "
                         "which a one-way clamp needs but which lets creep "
                         "accumulate -- watch the drift column")
    ap.add_argument("--no-shuffle", action="store_true",
                    help="sweep in the order given instead of randomising")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hold", type=float, default=1.0, help="s measured per level")
    ap.add_argument("--settle", type=float, default=0.4,
                    help="s before measuring; the loop rises in 13-30 ms, this "
                         "is for the hand")
    ap.add_argument("--kt", type=float, default=0.43,
                    help="V/(rad/s) for the back-EMF correction. Crude; the "
                         "correction is printed so its weight can be judged")
    ap.add_argument("--max-bemf", type=float, default=0.05,
                    help="reject a point whose back-EMF exceeds this fraction "
                         "of applied volts")
    ap.add_argument("--max-temp", type=int, default=45,
                    help="abort above this; the servo's own limit is 70")
    ap.add_argument("--ramp", type=float, default=0.3,
                    help="s to ramp the current on, so it does not kick")
    ap.add_argument("--cool", type=float, default=2.0,
                    help="off-time as a multiple of on-time")
    ap.add_argument("--dry-run", action="store_true",
                    help="walk the sequence with torque OFF and print it")
    args = ap.parse_args()

    bus = DynamixelBus(args.port, baud=args.baud, ids=(args.id,)).open()
    orig = bus.read_raw(args.id, "Operating Mode")
    vin = bus.read(args.id, "Present Input Voltage")
    ct = bus.tables[args.id]
    out = []
    try:
        bus.prepare()
        bus.torque(False, [args.id]); bus.write_raw(args.id, "Operating Mode", 0)
        m = (IndirectMap({args.id: ct}).read("Realtime Tick")
             .read("Present Position").read("Present PWM")
             .read("Present Current").read("Present Temperature")
             .write("Goal Current", label="g"))
        bus.apply_map(m)

        order = list(args.currents)
        if not args.no_shuffle:
            np.random.default_rng(args.seed).shuffle(order)
        sign = {"cw": 1, "ccw": -1}.get(args.direction)
        off = args.cool * (args.hold + args.settle + args.ramp)
        total = len(order) * (args.hold + args.settle + args.ramp + off)
        rr = 5.0
        print(f"  worst point {1000*max(args.currents):.0f} mA -> "
              f"{max(args.currents)**2*rr:.2f} W in the winding at R~{rr:.0f}, "
              f"{max(args.currents)**2*rr/(1+args.cool):.2f} W average at this "
              f"duty cycle.\n  Aborting at {args.max_temp} C; the servo's own "
              f"limit is {bus.read_raw(args.id,'Temperature Limit')} C.")
        print(f"  direction {args.direction}"
              + ("" if sign is None else
                 "  -- ONE WAY, so creep accumulates and the drift column is "
                 "the only\n              protection left. Watch it.")
              + f"\n  order {'as given' if args.no_shuffle else f'shuffled, seed {args.seed}'}: "
              + " ".join(f"{1000*c:.0f}" for c in order) + " mA")
        if args.dry_run:
            print("  --dry-run: torque stays OFF, nothing is commanded.\n")
        print(f"{ct.name} id {args.id}, {vin:.1f} V, up to "
              f"{1000 * max(args.currents):.0f} mA. It will push against you for "
              f"about {total:.0f} s.")
        if not args.dry_run:
            for k in (3, 2, 1):
                print(f"  HOLD OR CLAMP THE HORN ... {k}", flush=True)
                time.sleep(1.0)
        print(f"\n  {'I/mA':>6s} {'I_rep':>6s} {'duty':>7s} {'drift':>7s} "
              f"{'w rad/s':>8s} {'bemf%':>6s} {'V_wind':>7s} {'T':>3s}", flush=True)

        if not args.dry_run:
            bus.torque(True, [args.id])
        for k, ic in enumerate(order):
            temp0 = bus.read_raw(args.id, "Present Temperature")
            if temp0 >= args.max_temp:
                print(f"  {temp0} C before the {1000*ic:.0f} mA point -- stopping")
                break
            signed = ic * (sign if sign is not None
                           else (1 if k % 2 == 0 else -1))
            if args.dry_run:
                print(f"  would ramp to {1000*signed:+.0f} mA over {args.ramp}s, "
                      f"hold {args.hold}s, then {off:.1f}s off  (T={temp0} C)")
                time.sleep(0.05)
                continue
            # RAMP, do not step: a step to the top current is full torque in one
            # control period and kicks whatever is holding the horn.
            bus.capture(seconds=args.ramp, rate_hz=200.0, warn_overrun=False,
                        command=lambda t, _r, s=signed: {args.id: s * min(1.0, t / args.ramp)})
            bus.write_frame({args.id: signed})
            time.sleep(args.settle)
            rows = bus.capture(seconds=args.hold, rate_hz=200.0,
                               warn_overrun=False)
            bus.write_frame({args.id: 0.0}); time.sleep(off)

            duty = med(rows, args.id, "Present PWM")
            cur = med(rows, args.id, "Present Current")
            temp = int(np.max([r["servos"][args.id]["Present Temperature"]
                               for r in rows]))
            counts = net_drift(rows, args.id)
            w = abs(counts) * (2 * np.pi / CPR) / args.hold
            applied = duty * vin
            bemf = args.kt * w
            frac = bemf / applied if applied > 0 else 1.0
            out.append((cur, duty, w, frac, applied - bemf))
            print(f"  {1000 * ic:6.0f} {1000 * cur:6.1f} {duty:7.4f} "
                  f"{counts:7.0f} {w:8.4f} {100 * frac:6.1f} "
                  f"{applied - bemf:7.3f} {temp:3d}"
                  + ("   REJECT" if frac > args.max_bemf else ""), flush=True)
            if temp >= args.max_temp:
                print(f"  {temp} C -- stopping"); break
        if not args.dry_run:
            bus.write_frame({args.id: 0.0}); time.sleep(0.2)
            bus.torque(False, [args.id])
            print("\n  LET GO.", flush=True)
    finally:
        try:
            bus.torque(False, [args.id])
            bus.write_raw(args.id, "Operating Mode", orig)
        except Exception as e:                              # noqa: BLE001
            print("restore failed:", e)
        bus.close()

    a = np.array(out) if out else np.zeros((0, 5))
    good = a[(a[:, 3] <= args.max_bemf) & (a[:, 1] < 0.99)] if len(a) else a
    print(f"\n{len(good)} of {len(a)} points survive the back-EMF cut "
          f"(<{100 * args.max_bemf:.0f}% of applied volts)")
    if len(good) < 6:
        print("Not enough to call it. A firmer hold, or fewer high-current "
              "points, and re-run.")
        return 1

    I, V = good[:, 0], good[:, 4]       # V is back-EMF corrected
    duty_c = V / vin

    def r2(pred):
        return 1 - np.sum((V - pred) ** 2) / np.sum((V - V.mean()) ** 2)

    R1 = float(np.linalg.lstsq(I[:, None], V, rcond=None)[0][0])
    V0, R3 = np.linalg.lstsq(np.column_stack([np.ones_like(I), I]), V,
                             rcond=None)[0]
    Rb = float(np.linalg.lstsq(I[:, None], (duty_c ** 2) * vin, rcond=None)[0][0])
    n, lna = np.polyfit(np.log(I), np.log(duty_c), 1)
    print(f"\n  MODEL                            fit                     R2")
    print(f"  duty*vin = I*R          (phase)  R={R1:6.2f} ohm         "
          f"{r2(I * R1):+.5f}")
    print(f"  duty*vin = V0 + I*R              R={R3:6.2f}, V0={V0:+.3f}V "
          f"{r2(V0 + I * R3):+.5f}")
    print(f"  duty = sqrt(R*I/vin)      (bus)  R={Rb:6.3f} ohm         "
          f"{r2(np.sqrt(np.clip(Rb * I / vin, 0, None)) * vin):+.5f}")
    print(f"  duty = a * I^n          (free)   n={n:.3f}"
          f"                  {r2(np.exp(lna) * I ** n * vin):+.5f}")
    print(f"\n  n = 1.0 is phase regulation, 0.5 is bus. Rhoban/bam fit "
          f"R = 2.5-3.5 ohm\n  for this part from pendulum trajectories, which "
          f"is an independent check\n  on whichever R the winning model gives.")
    print(f"\n  ONE SWEEP IS NOT A RESULT. Run it three times and compare n "
          f"before\n  believing any of it -- a compliant hold biases duty(I) "
          f"sublinear, and\n  that is the same direction as the effect being "
          f"looked for.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
