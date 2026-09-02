"""Does the Dynamixel current controller have dynamics of its own?

Step response of the CURRENT LOOP ALONE, at a held output. Mode 0 is the loop by
itself; mode 5 (current-based position) cascades a position PID into the same
loop, so whatever this measures is inherited there.

    python analysis/servo_step.py --port /dev/... --id 150

THREE THINGS THAT MAKE THIS A REAL MEASUREMENT, none of which our earlier
free-shaft attempt had:

1. HELD OUTPUT. On a free shaft the current rise is entangled with the shaft
   accelerating, and back-EMF eats the supply -- a 150 mA command saturated at
   duty 1.000 and only reached ~104 mA. At stall there is no back-EMF, so even
   450 mA needs only duty ~0.51 (bus model, R 2.98, 5.2 V) and NOTHING
   saturates. The full amplitude range becomes testable.

2. EQUIVALENT-TIME SAMPLING. 500 Hz is the FTDI latency ceiling, i.e. 2 ms per
   sample against a rise of order 15 ms -- 7 samples, which cannot distinguish a
   first-order lag from a slew limit from a transport delay. So each step is
   repeated with the command deliberately placed at a RANDOM offset within the
   sample period, and the repetitions pooled by time-since-command:

       1 rep  -> 2000 us resolution      40 reps ->  50 us
      10 reps ->  200 us                100 reps ->  20 us

   Nothing about the hardware changes; the information is recovered from the
   jitter instead of being averaged away by it.

3. AMPLITUDE SWEEP, which is what actually answers the question:

       t_63 INDEPENDENT of step size   -> linear; the loop has a time constant
       t_63 PROPORTIONAL to step size  -> slew limited; "as fast as it can"
       flat dead time before any rise  -> transport delay, model separately
       t_95 / t_63 == 3.0              -> genuinely FIRST order
       t_95 / t_63 >  3.0              -> higher order, an S-shaped rise

   That last ratio needs no fitting at all: for I(t) = I_inf*(1-exp(-t/tau)),
   t_63 = tau and t_95 = ln(20)*tau, so the ratio is 2.996 exactly.

Superposition is checked directly too (`--super`): a 100 -> 200 mA step must
rise identically to a 0 -> 100 mA step if the loop is linear, and the fall
100 -> 0 must mirror the rise.

THERMAL. Steps are ~250 ms with a long gap, so the duty cycle is under 10%; at
450 mA and R 2.98 that is 0.6 W instantaneous and under 60 mW average. Same
45 C abort and Goal-Current cap as the other bench scripts.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "src")

from aow_sim.hw.dynamixel import DynamixelBus, IndirectMap   # noqa: E402

CPR = 4096.0


def one_step(bus, dxl_id, i_from, i_to, rate, pre, post, rng):
    """One repetition. Returns (t_since_command, current, duty, drift_counts).

    The command is issued at a RANDOM time within a sample period, which is what
    makes the pooled ensemble finer than the sample rate.
    """
    bus.write_frame({dxl_id: i_from})
    time.sleep(0.25)
    fired = {}
    # SUB-SAMPLE PHASE. The command can only be issued from inside the frame
    # loop, so without this it always lands ON a sample boundary and every
    # offset is an exact multiple of the sample period -- N repetitions of the
    # same handful of points, no extra resolution at all. Sleeping a random
    # fraction of a period BEFORE the write puts the step between samples, so
    # pooling across reps fills in the gaps.
    jitter = float(rng.uniform(0.0, 1.0 / rate))

    def cmd(t, _row):
        if t >= pre and "t" not in fired:
            time.sleep(jitter)
            fired["t"] = t + jitter
            return {dxl_id: i_to}
        return {}

    rows = bus.capture(seconds=pre + post + jitter, rate_hz=rate, command=cmd,
                       warn_overrun=False)
    bus.write_frame({dxl_id: 0.0})
    if "t" not in fired:
        return None
    # ZERO IS UNCERTAIN BY ~ONE READ. `capture` stamps t_host, then reads the
    # frame (~2 ms), then calls this callback, then writes -- so the command
    # actually goes out ~one read after the stamped time. That offset is
    # CONSTANT across reps and amplitudes, so it shifts t=0 but not tau; the fit
    # below carries a free delay term to absorb it, which is why tau is read
    # from the fit rather than from a 63% crossing.
    t = np.array([r["t_host"] for r in rows]) - fired["t"]
    cur = np.array([abs(r["servos"][dxl_id]["Present Current"]) for r in rows])
    duty = np.array([abs(r["servos"][dxl_id]["Present PWM"]) for r in rows])
    pos = np.array([r["servos"][dxl_id]["Present Position"] for r in rows]) * CPR / (2*np.pi)
    d = ((np.diff(pos) + CPR // 2) % CPR) - CPR // 2
    return t, cur, duty, float(np.sum(np.abs(d)))


def fit_first_order(t, y, y_inf):
    """Fit y = y_inf*(1 - exp(-(t - td)/tau)) with td FREE.

    tau read from a fit rather than from a 63% crossing, because t=0 carries a
    constant unknown offset (see `one_step`) and a crossing time inherits it
    while a time constant does not. `td` absorbs that offset plus any real
    transport delay: its absolute value is not meaningful, but its variation
    ACROSS amplitudes is, because the offset is common to all of them.

    Returns (tau, td, r2, t63, t95) -- the last two descriptive, from the fit.
    """
    from scipy.optimize import curve_fit

    def model(x, tau, td):
        return y_inf * (1.0 - np.exp(-np.clip(x - td, 0, None) / tau))

    ok = np.isfinite(t) & np.isfinite(y)
    t, y = t[ok], y[ok]
    try:
        (tau, td), _ = curve_fit(model, t, y, p0=[0.015, 0.002],
                                 bounds=([1e-4, -0.02], [1.0, 0.05]),
                                 maxfev=40000)
    except Exception:                                          # noqa: BLE001
        return (float("nan"),) * 5
    r2 = 1 - np.sum((y - model(t, tau, td)) ** 2) / np.sum((y - y.mean()) ** 2)
    return tau, td, r2, td + tau, td + np.log(20.0) * tau


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--steps", type=float, nargs="+",
                    default=[0.02, 0.05, 0.10, 0.20, 0.30, 0.45],
                    help="amps; each is a 0 -> A step")
    ap.add_argument("--reps", type=int, default=40,
                    help="repetitions per step; sets the effective resolution "
                         "(2 ms / reps)")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--pre", type=float, default=0.05, help="s before the step")
    ap.add_argument("--post", type=float, default=0.25, help="s after")
    ap.add_argument("--gap", type=float, default=0.35, help="s off between reps")
    ap.add_argument("--current-limit", type=float, default=0.60)
    ap.add_argument("--max-temp", type=int, default=45)
    ap.add_argument("--max-drift", type=float, default=30,
                    help="counts; a rep that moved more than this is dropped")
    ap.add_argument("--superposition", action="store_true",
                    help="also run 100->200 mA and 100->0 mA")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--allow-negative", action="store_true",
                    help="permit negative current; only for a fixture that "
                         "holds in BOTH directions")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    # ONE DIRECTION ONLY. The stop holds against positive current and nothing
    # holds the other way, so a negative command drives the horn off the bar and
    # away. Refused rather than clamped, so a typo in --steps is an error and
    # not a silent runaway.
    if not args.allow_negative and any(a < 0 for a in args.steps):
        raise SystemExit(f"negative current in --steps {args.steps}; the stop "
                         f"holds one way only. Pass --allow-negative if the "
                         f"fixture really is bidirectional.")
    rng = np.random.default_rng(args.seed)
    bus = DynamixelBus(args.port, baud=args.baud, ids=(args.id,)).open()
    orig = bus.read_raw(args.id, "Operating Mode")
    vin = bus.read(args.id, "Present Input Voltage")
    try:
        bus.prepare()
        bus.torque(False, [args.id]); bus.write_raw(args.id, "Operating Mode", 0)
        m = (IndirectMap({args.id: bus.tables[args.id]}).read("Realtime Tick")
             .read("Present Position").read("Present PWM")
             .read("Present Current").read("Present Temperature")
             .write("Goal Current", label="g"))
        bus.apply_map(m)
        bus.write(args.id, "Current Limit", args.current_limit)

        pairs = [(0.0, a) for a in args.steps]
        if args.superposition:
            pairs += [(0.10, 0.20), (0.10, 0.0)]
        dur = len(pairs) * args.reps * (args.pre + args.post + args.gap + 0.25)
        print(f"{bus.tables[args.id].name} id {args.id}, {vin:.1f} V. "
              f"{len(pairs)} steps x {args.reps} reps ~ {dur/60:.1f} min.")
        print(f"effective resolution {1000/args.rate/args.reps*1000:.0f} us "
              f"from {1000/args.rate:.1f} ms sampling.")
        if args.dry_run:
            for a, b in pairs:
                print(f"  would step {1000*a:.0f} -> {1000*b:.0f} mA, "
                      f"{args.reps} reps")
            return 0

        bus.torque(True, [args.id])
        print(f"\n  {'step/mA':>10s} {'reps':>5s} {'I_inf':>7s} {'tau/ms':>7s} "
              f"{'delay/ms':>8s} {'duty_pk':>8s} {'fit R2':>8s} {'T':>3s}")
        results = []
        for i_from, i_to in pairs:
            T, Y, D, kept = [], [], [], 0
            for _ in range(args.reps):
                if bus.read_raw(args.id, "Present Temperature") >= args.max_temp:
                    print("  hot -- stopping"); break
                r = one_step(bus, args.id, i_from, i_to, args.rate,
                             args.pre, args.post, rng)
                time.sleep(args.gap)
                if r is None:
                    continue
                t, cur, duty, drift = r
                if drift > args.max_drift:
                    continue                      # the hold slipped, drop it
                T.append(t); Y.append(cur); D.append(duty); kept += 1
            if kept < 5:
                print(f"  {1000*i_to:10.0f} {kept:5d}   too few clean reps")
                continue
            t = np.concatenate(T); y = np.concatenate(Y); dy = np.concatenate(D)
            o = np.argsort(t); t, y, dy = t[o], y[o], dy[o]
            # pool onto a fine grid: the ensemble is what carries the resolution
            grid = np.arange(-args.pre, args.post, 1.0 / (args.rate * args.reps))
            idx = np.clip(np.searchsorted(grid, t) - 1, 0, len(grid) - 1)
            pooled = np.array([y[idx == k].mean() if (idx == k).any() else np.nan
                               for k in range(len(grid))])
            ok = ~np.isnan(pooled)
            gt, gy = grid[ok], pooled[ok]
            base = gy[gt < 0].mean() if (gt < 0).any() else 0.0
            inf = gy[gt > 0.8 * args.post].mean()
            # PEAK DUTY DURING THE RISE. This is the column that separates
            # "the drive railed, so error magnitude had nothing to act on" from
            # "the loop does not scale its output with error magnitude" -- two
            # explanations that produce IDENTICAL rise times and cannot be told
            # apart from the settled duty, which is what an earlier version of
            # this test tried to do. Railed during the rise (~1.0) means the
            # test says nothing about controller structure; well short of 1.0
            # means it genuinely did not drive harder for a bigger error.
            rise = (t >= 0) & (t <= 0.05)
            duty_pk = float(np.percentile(dy[rise], 95)) if rise.any() else float("nan")
            sel = gt >= 0
            tau, td, r2, t63, t95 = fit_first_order(
                gt[sel], gy[sel] - base, inf - base)
            temp = bus.read_raw(args.id, "Present Temperature")
            results.append((i_to - i_from, inf - base, tau, td, r2))
            print(f"  {1000*(i_to-i_from):10.0f} {kept:5d} {1000*(inf-base):7.1f} "
                  f"{1000*tau:7.2f} {1000*td:7.2f} {duty_pk:8.3f} {r2:8.5f} "
                  f"{temp:3d}" + ("   RAILED" if duty_pk > 0.97 else ""),
                  flush=True)
        bus.write_frame({args.id: 0.0}); time.sleep(0.2)
        bus.torque(False, [args.id])
    finally:
        try:
            bus.torque(False, [args.id])
            bus.write_raw(args.id, "Operating Mode", orig)
        except Exception as e:                                  # noqa: BLE001
            print("restore failed:", e)
        bus.close(); print("\nrestored, torque off")

    a = np.array([r for r in results if np.isfinite(r[2])])
    if len(a) < 3:
        return 0
    amp, tau, td, r2 = np.abs(a[:, 0]), a[:, 2], a[:, 3], a[:, 4]
    span, amp_span = tau.max() / tau.min(), amp.max() / amp.min()
    print(f"\n  tau {1000*tau.min():.2f}-{1000*tau.max():.2f} ms, varying "
          f"{span:.2f}x over a {amp_span:.0f}x amplitude range")
    print(f"    -> {'SLEW LIMITED (tau tracks amplitude)' if span > 0.5 * amp_span else 'LINEAR (tau independent of amplitude)'}")
    print(f"  delay spread {1000*(td.max()-td.min()):.2f} ms -- the ABSOLUTE "
          f"value is meaningless (t=0 is\n    uncertain by ~one read) but a "
          f"spread that grows with amplitude is not.")
    print(f"  first-order fit R2 {r2.min():.5f}-{r2.max():.5f}; a systematic "
          f"S-shape shows up here\n    as a poor fit, which is a better test "
          f"than t95/t63 when t=0 is uncertain.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
