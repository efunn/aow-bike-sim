"""What does each position mode actually COMMAND? Position(3) vs current-based
position(5), decomposed against a hard stop.

THE STRUCTURE, from ROBOTIS's own block diagrams:

  mode 3  Goal Pos -> Profile -> PID + feedforward -> Limiter -> INVERTER
          the PID output IS the PWM duty.

  mode 5  Goal Pos -> Profile -> PID + feedforward -> Limiter -> CURRENT
          CONTROLLER -> Inverter
          the PID output is a CURRENT SETPOINT. One extra loop, and Goal
          Current(102) becomes its limit rather than a command.

So the two modes differ in what the position error is converted INTO, and that
conversion is a single scale constant per mode. Against a hard stop the error is
held constant and known, the shaft cannot move, and the constant is readable:

  mode 3:  duty        = k3 * Kp * error       -> read Present PWM(124)
  mode 5:  I_setpoint  = k5 * Kp * error       -> read Present Current(126)

`analysis/servo_stall.py` already characterised the mode-0 current controller in
isolation (it regulates a blend, alpha*I_phase + (1-alpha)*I_bus, alpha ~ 0.16,
R ~ 3.6 ohm). If mode 5 really is "the position PID feeding THAT controller",
then the duty measured in mode 5 must be predicted by the blend from the current
measured in mode 5 -- with no new parameters. That consistency check is the
point of this script, and it is printed as `blend residual`.

WHY A HARD STOP AND NOT A CLAMP. The bar stops rotation past a fixed angle, so
commanding a goal BEYOND it holds a known, constant position error indefinitely
with the shaft genuinely stationary. Nothing is being held by hand, the error is
set in software, and net drift is measured per point to prove the stop held.

    python analysis/servo_cascade.py --port /dev/... --id 150 \\
        --stop-deg 70 --direction below

`--direction below` means the bar blocks motion toward 0, so goals BELOW
`--stop-deg` press into it (the 70 deg stop). `above` is the mirror case for a
stop at 290 deg.

THERMAL. Every point is a stall, so all electrical power is winding heat. The
current is bounded two ways: `Goal Current` is set to `--current-limit` (the
firmware's own cap in mode 5), and the requested (Kp, error) grid is checked
against a predicted current before anything is energised. Same 45 C abort and
enforced cool-down as servo_stall.py.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "src")

from aow_sim.hw.dynamixel import DynamixelBus, IndirectMap   # noqa: E402

CPR = 4096.0
MODE_POSITION, MODE_CUR_POS = 3, 5
# From analysis/servo_stall.py, XL330-M288-T at 5.2 V, two runs opposite
# directions: I_rep = (vin/R) * (alpha*duty + (1-alpha)*duty^2).
BLEND_ALPHA, BLEND_R = 0.158, 3.57


def blend_current(duty, vin, alpha=BLEND_ALPHA, R=BLEND_R):
    """Current the mode-0 controller would report at this duty, at stall."""
    return (vin / R) * (alpha * duty + (1.0 - alpha) * duty * duty)


def net_drift(rows, i):
    pos = np.array([r["servos"][i]["Present Position"] for r in rows], float)
    c = pos * CPR / (2 * np.pi)
    d = ((np.diff(c) + CPR // 2) % CPR) - CPR // 2
    return float(np.sum(d))


def med(rows, i, key):
    return float(np.median([abs(r["servos"][i][key]) for r in rows]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--stop-deg", type=float, required=True,
                    help="where the bar stops the horn")
    ap.add_argument("--direction", choices=("below", "above"), default="below",
                    help="below: goals under --stop-deg press into the bar")
    ap.add_argument("--errors", type=float, nargs="+",
                    default=[1, 2, 3, 5, 8, 12, 18],
                    help="degrees past the stop")
    ap.add_argument("--pgains", type=int, nargs="+", default=[100, 400])
    ap.add_argument("--park-deg", type=float, default=None,
                    help="angle to rest at between points. Defaults to 2 deg on "
                         "the FREE side of the stop -- see the note in the "
                         "source about why this must not be far away")
    ap.add_argument("--approach-step", type=float, default=2.0,
                    help="deg per step when first approaching the stop")
    ap.add_argument("--current-limit", type=float, default=0.35,
                    help="A; Goal Current(102), the firmware cap in mode 5")
    ap.add_argument("--hold", type=float, default=0.6)
    ap.add_argument("--cool", type=float, default=2.0)
    ap.add_argument("--max-temp", type=int, default=45)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    sgn = -1.0 if args.direction == "below" else 1.0
    # PARK JUST OFF THE STOP, NOT FAR FROM IT.
    #
    # The first version of this parked at 110 deg against a stop at 70 and
    # stepped straight to each target. `prepare()` sets Profile Velocity and
    # Acceleration to 0, which means NO trajectory shaping -- the goal jumps, so
    # the servo slews the full 40 deg at whatever authority the gain allows and
    # arrives at the bar with all of it. Fourteen points, fourteen impacts: the
    # bar walked from 70 deg to 65.5 across the sweep and came off entirely on
    # the last runs.
    #
    # The steady-state numbers survived that (drift was 0 while measuring, and
    # the fit uses the MEASURED position so a stop that moved BETWEEN points is
    # harmless) -- but the rig does not survive it, and a destroyed rig means no
    # second run. Parking 2 deg on the free side reaches the same zero-current
    # rest state with ~2 deg of travel instead of 40.
    if args.park_deg is None:
        args.park_deg = args.stop_deg - sgn * 2.0
    bus = DynamixelBus(args.port, baud=args.baud, ids=(args.id,)).open()
    orig = {r: bus.read_raw(args.id, r) for r in
            ("Operating Mode", "Position P Gain", "Position I Gain",
             "Position D Gain", "Goal Current")}
    vin = bus.read(args.id, "Present Input Voltage")
    out = []
    try:
        bus.prepare()
        print(f"{bus.tables[args.id].name} id {args.id}, {vin:.1f} V. Stop at "
              f"{args.stop_deg:.0f} deg, pressing {args.direction} it.")
        print(f"Goal Current cap {1000*args.current_limit:.0f} mA "
              f"({args.current_limit**2*BLEND_R:.2f} W at stall), abort {args.max_temp} C.")
        if args.dry_run:
            print("--dry-run: torque never enabled.\n")
        print(f"\n  {'mode':>4s} {'Kp':>4s} {'err':>5s} {'Kp*err':>7s} "
              f"{'duty':>7s} {'I/mA':>6s} {'drift':>6s} {'pos':>7s} {'T':>3s}  note")
        for mode in (MODE_POSITION, MODE_CUR_POS):
            for kp in args.pgains:
                # Gains AFTER the mode write -- writing Operating Mode resets
                # Position P/D to that mode's defaults. Read back or it is a
                # gain comparison wearing a mode comparison's clothes.
                bus.torque(False, [args.id])
                bus.write_raw(args.id, "Operating Mode", mode)
                bus.write_raw(args.id, "Position P Gain", kp)
                bus.write_raw(args.id, "Position I Gain", 0)   # else it winds up
                bus.write_raw(args.id, "Position D Gain", 0)   # zero at stall anyway
                got = tuple(bus.read_raw(args.id, r) for r in
                            ("Position P Gain", "Position I Gain", "Position D Gain"))
                assert got == (kp, 0, 0), f"gains did not stick: {got}"
                m = (IndirectMap({args.id: bus.tables[args.id]})
                     .read("Realtime Tick").read("Present Position")
                     .read("Present PWM").read("Present Current")
                     .read("Present Temperature")
                     .write("Goal Position", label="g"))
                bus.apply_map(m)
                bus.write_raw(args.id, "Position P Gain", kp)
                bus.write(args.id, "Goal Current", args.current_limit)   # amps, encoded
                if args.dry_run:
                    for e in args.errors:
                        print(f"  {mode:4d} {kp:4d} {e:5.1f} {kp*np.radians(e):7.2f}"
                              f"    would park at {args.park_deg:.0f} then command "
                              f"{args.stop_deg + sgn*e:.1f} deg")
                    continue
                # First approach of the block: walk in rather than jump, so the
                # horn does not arrive at the bar carrying the whole travel.
                here = np.degrees(bus.read(args.id, "Present Position"))
                bus.torque(True, [args.id])
                n_steps = max(1, int(abs(args.park_deg - here) / args.approach_step))
                for f in np.linspace(0, 1, n_steps + 1)[1:]:
                    bus.write_frame({args.id: np.radians(here + f * (args.park_deg - here))})
                    time.sleep(0.06)
                time.sleep(0.6)
                for e in args.errors:
                    t0 = bus.read_raw(args.id, "Present Temperature")
                    if t0 >= args.max_temp:
                        print(f"  {t0} C -- stopping"); break
                    # into the stop, hold, then back to park to cool
                    bus.write_frame({args.id: np.radians(args.stop_deg + sgn * e)})
                    time.sleep(0.5)
                    rows = bus.capture(seconds=args.hold, rate_hz=200.,
                                       warn_overrun=False)
                    bus.write_frame({args.id: np.radians(args.park_deg)})
                    time.sleep(args.cool * args.hold)
                    duty = med(rows, args.id, "Present PWM")
                    cur = med(rows, args.id, "Present Current")
                    pos = np.degrees(med(rows, args.id, "Present Position"))
                    drift = net_drift(rows, args.id)
                    temp = int(np.max([r["servos"][args.id]["Present Temperature"]
                                       for r in rows]))
                    # TRUE ERROR IS MEASURED, NOT ASSUMED. The bar is compliant:
                    # it yields progressively under torque, so `pos` walked from
                    # 70 deg down to 65 at Kp 400 and the servo pushed clean
                    # through it in mode 5. Taking the error as
                    # (goal - stop_deg) would have been wrong for most points.
                    # The servo reports where it actually is, so use that -- it
                    # makes the whole measurement independent of where the stop
                    # is and whether it moved BETWEEN points. Only movement
                    # DURING a hold matters, and `drift` is what proves that.
                    err_true = pos - (args.stop_deg + sgn * e)
                    note = ""
                    if abs(drift) > 20:
                        note = "MOVED DURING THE HOLD"
                    elif abs(err_true) < 0.2:
                        note = "reached the goal, no error to read"
                    elif cur >= 0.97 * args.current_limit:
                        note = "at Goal Current cap"
                    elif duty >= 0.97:
                        note = "PWM saturated"
                    out.append((mode, kp, err_true, kp * np.radians(err_true),
                                duty, cur, drift, pos, note))
                    print(f"  {mode:4d} {kp:4d} {err_true:5.2f} "
                          f"{kp*np.radians(err_true):7.2f} "
                          f"{duty:7.4f} {1000*cur:6.1f} {drift:6.0f} {pos:7.2f} "
                          f"{temp:3d}  {note}", flush=True)
                bus.write_frame({args.id: np.radians(args.park_deg)})
                time.sleep(0.5); bus.torque(False, [args.id])
    finally:
        try:
            bus.torque(False, [args.id])
            bus.write_raw(args.id, "Operating Mode", orig["Operating Mode"])
            for r, v in orig.items():
                if r != "Operating Mode":
                    bus.write_raw(args.id, r, v)
        except Exception as e:                                  # noqa: BLE001
            print("restore failed:", e)
        bus.close(); print("\nrestored, torque off")

    if not out:
        return 0
    print("\nWHAT EACH MODE CONVERTS THE POSITION ERROR INTO")
    for mode, lbl, col, unit in ((MODE_POSITION, "position(3)  duty", 4, "duty"),
                                 (MODE_CUR_POS, "cur-based(5) I_set", 5, "A")):
        pts = [r for r in out if r[0] == mode and not r[8] and r[3] > 0.005]
        if len(pts) < 3:
            print(f"  mode {mode}: only {len(pts)} clean points"); continue
        x = np.array([p[3] for p in pts]); y = np.array([p[col] for p in pts])
        k = float(np.linalg.lstsq(x[:, None], y, rcond=None)[0][0])
        r2 = 1 - np.sum((y - k*x)**2)/np.sum((y - y.mean())**2)
        print(f"  {lbl} = k * Kp * err   k = {k:.3e} {unit}/(Kp*rad)   "
              f"R2 {r2:+.5f}   n={len(pts)}")
    print("\n  BAM's constants, and the same derivation with 885 -> 1000:")
    print(f"    voltage (4096/2pi)/(256*885)  = "
          f"{(4096/(2*np.pi))/(256*885):.3e} duty/(Kp*rad)  [BAM, derived]")
    print(f"    current (4096/2pi)/(256*1000) = "
          f"{(4096/(2*np.pi))/(256*1000):.3e} A/(Kp*rad)     [the same formula]")
    print(f"    current 0.016/pi              = "
          f"{0.016/np.pi:.3e} A/(Kp*rad)     [BAM, undocumented literal]")
    print("\n  If k3*885 == k5*1000 then the position PID emits ONE number and")
    print("  the modes only differ in the unit downstream reads it in: PWM")
    print("  counts (885 full scale) or milliamps.")

    print("\nIS MODE 5's INNER LOOP THE SAME CONTROLLER AS MODE 0?")
    pts = [r for r in out if r[0] == MODE_CUR_POS and not r[8]]
    if len(pts) >= 3:
        d = np.array([p[4] for p in pts]); c = np.array([p[5] for p in pts])
        pred = blend_current(d, vin)
        print(f"  blend(alpha={BLEND_ALPHA}, R={BLEND_R}) predicts I from the "
              f"mode-5 duty, with NO new parameters:")
        print(f"  {'duty':>8s} {'I meas/mA':>10s} {'I pred/mA':>10s} {'resid/mA':>9s}")
        for dd, cc, pp in zip(d, c, pred):
            print(f"  {dd:8.4f} {1000*cc:10.1f} {1000*pp:10.1f} {1000*(cc-pp):9.1f}")
        print(f"  mean |residual| {1000*np.mean(np.abs(c-pred)):.1f} mA, "
              f"max {1000*np.max(np.abs(c-pred)):.1f} mA")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
