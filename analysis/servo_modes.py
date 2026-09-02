"""Where the servo lag comes from, and how the control modes differ.

Three studies behind `docs/measurements/servo-measurements.yaml`. Each was got
WRONG at least once before it was got right, and each wrong answer was
plausible, so the traps are encoded as guards here rather than left as prose:

  lag-origin  Is the ~22 ms drive lag the motor or the firmware? PWM mode
              (Operating Mode 16) removes the velocity loop entirely; a
              Velocity P Gain sweep then shows whether tau is a fixed property.
              -> s1_step_response.tau_origin
  modes       Plain position (3) vs current-based position (5), as a 2x2 with
              the derivative gain, because the modes have DIFFERENT DEFAULT
              GAINS and comparing them naively compares the gains.
              -> r5_control_mode_comparison
  no-load     Duty sweep on a bare shaft: speed confirms the datasheet no-load,
              and the reported current fails a consistency check that matters.
              -> xc330_no_load

    python analysis/servo_modes.py --port /dev/cu.usbserial-XXXX \
        --ids 101,102 lag-origin
    python analysis/servo_modes.py --port ... --ids 103,104 modes
    python analysis/servo_modes.py --port ... --ids 103,104 no-load

THE FOUR TRAPS, ALL OF WHICH PRODUCED A CONFIDENT WRONG NUMBER FIRST:

1. **Writing Operating Mode RESETS the position gains** to that mode's
   defaults (XC330-T181: mode 3 -> P900/D0, mode 4 -> P900/D0, mode 5 ->
   P700/D1400). Gains must be written AFTER the mode, and read back. The first
   version of the mode comparison set P=800 once up front and unknowingly ran
   mode 4 at P900/D0 against mode 5 at P700/D1400.
2. **Mode 3 is a 0..4095 COUNT RANGE.** A square wave centred on 0 has its
   whole negative half refused as out of range, which looks like a servo that
   does not move rather than an error. The centre is taken from the servo's own
   Min/Max Position Limit here.
3. **Extended position (4) unwinds** from its multi-turn rest at the start of a
   run, so the first condition of a block measures a long slew. Mode 3 is used
   instead, and the rig waits for the shaft to actually arrive before capturing.
4. **A velocity amplitude above Velocity Limit is silently refused** and the
   servo simply does not move. Clamped, from the register rather than a
   datasheet.

`DynamixelBus.discover` separately refuses an XC330 below firmware 53, which
accepts every indirect address write and then returns an all-zero data window.

SAFETY. Everything runs on a BARE SHAFT with no load. Every mode, gain and
limit touched is restored in a finally-block, torque is dropped on every exit
path including Ctrl-C, and no EEPROM is written beyond what `prepare()` sets
(Return Delay Time, the two Profile registers).

SCOPE. Bare shaft means no stiction anywhere. The small-amplitude regime on a
LOADED servo is a stiction limit cycle, whose character is set by loop gain
against a Coulomb band -- a different question. Re-run against the R1/R2 load
fixture before applying any of this to the self-righting mechanism.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "src")

from aow_sim.hw.dynamixel import DynamixelBus, IndirectMap   # noqa: E402

try:                                    # shared with the reversal study
    from servo_reversal import _unwrap
except ImportError:                     # when run from the repo root
    sys.path.insert(0, "analysis")
    from servo_reversal import _unwrap

TICK_WRAP = 32768
CPR = 4096.0
COUNT_DEG = 360.0 / CPR
MODE_VELOCITY, MODE_POSITION, MODE_CURRENT_POSITION, MODE_PWM = 1, 3, 5, 16

# Registers restored on exit, whatever a subcommand touched.
_RESTORE = ("Operating Mode", "Position P Gain", "Position I Gain",
            "Position D Gain", "Velocity P Gain", "Velocity I Gain")


# --------------------------------------------------------------------------
# shared machinery
# --------------------------------------------------------------------------

def timebase(rows: list, dxl_id: int) -> tuple:
    """(t [s], continuous position [rad]) from the SERVO's own clock."""
    sv = [r["servos"][dxl_id] for r in rows]
    tick = np.array([s["Realtime Tick"] for s in sv], float)
    t = np.cumsum(np.concatenate([[0.0], np.diff(tick) % TICK_WRAP])) * 1e-3
    counts = _unwrap(np.array([s["Present Position"] for s in sv]) * CPR / (2 * np.pi))
    return t, counts / CPR * 2 * np.pi


def swing(rows, dxl_id, freq, velocity=False) -> float:
    """Steady-state peak-to-peak, discarding the first half period."""
    t, rad = timebase(rows, dxl_id)
    x = rad
    if velocity:
        dt = np.diff(t)
        ok = dt > 0
        x = np.zeros_like(t)
        x[1:][ok] = np.diff(rad)[ok] / dt[ok]
    s = t > 0.5 / freq
    if s.sum() < 4:
        return float("nan")
    return float(np.percentile(x[s], 97) - np.percentile(x[s], 3))


def fit_tau(freqs, swings) -> tuple:
    """swing(f) = S_inf * tanh(1/(4 f tau)), both free.

    A square wave into a first-order lag settles to that peak-to-peak, so tau
    comes from the AMPLITUDE roll-off and needs no edge detection and no prior
    knowledge of the steady-state amplitude. Fitting S_inf as well is what makes
    it work in PWM mode, where the commanded quantity is duty and the achieved
    quantity is rad/s.
    """
    from scipy.optimize import curve_fit

    freqs = np.asarray(freqs, float)
    swings = np.asarray(swings, float)
    ok = np.isfinite(swings) & (swings > 0)
    if ok.sum() < 3:
        return float("nan"), float("nan")
    p, _ = curve_fit(lambda f, S, tau: S * np.tanh(1.0 / (4.0 * f * tau)),
                     freqs[ok], swings[ok],
                     p0=[min(max(swings[ok].max() * 1.2, 0.2), 900.0), 0.02],
                     bounds=([0.05, 1e-4], [1000.0, 1.0]), maxfev=20000)
    return float(p[0]), float(p[1])


def set_mode_and_gains(bus, dxl_id: int, mode: int, **gains) -> None:
    """Write the mode, THEN the gains, then read the gains back.

    Trap 1. Writing Operating Mode silently rewrites Position P/D Gain to that
    mode's defaults, so gains set beforehand are discarded. The read-back is not
    paranoia: it is the only thing standing between a mode comparison and a gain
    comparison, and it costs one round trip per register.
    """
    bus.torque(False, [dxl_id])
    bus.write_raw(dxl_id, "Operating Mode", mode)
    for name, value in gains.items():
        bus.write_raw(dxl_id, name.replace("_", " "), value)
    for name, value in gains.items():
        got = bus.read_raw(dxl_id, name.replace("_", " "))
        if got != value:
            raise RuntimeError(
                f"id {dxl_id}: {name} read back {got}, wrote {value}. Writing "
                f"Operating Mode resets the position gains; if this fires, "
                f"something else is resetting them too.")


def position_centre(bus, dxl_id: int) -> float:
    """Mid-range goal position [rad], from the servo's own limits.

    Trap 2. In a single-turn position mode the goal must lie inside
    [Min Position Limit, Max Position Limit]; a wave centred on 0 has its whole
    negative half refused, and a refused goal looks like a servo that will not
    move rather than an error.
    """
    lo = bus.read_raw(dxl_id, "Min Position Limit")
    hi = bus.read_raw(dxl_id, "Max Position Limit")
    return (lo + hi) / 2.0 / CPR * 2 * np.pi


def settle_at(bus, ids, targets: dict, tol_deg=1.0, timeout=3.0) -> bool:
    """Drive to a target and WAIT FOR ARRIVAL before capturing.

    Trap 3. A block's first condition otherwise captures the slew to the centre
    rather than the wave, which shows up as a ratio in the tens and is easy to
    read as instability.
    """
    bus.write_frame(targets)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        row = bus.read_frame()
        if all(abs(np.degrees(row[i]["Present Position"] - targets[i])) < tol_deg
               for i in ids):
            return True
        time.sleep(0.02)
    return False


def square(bus, ids, amp, freq, centre=0.0, rate=500.0, cycles=8):
    """One square-wave condition -> the captured rows."""
    def wave(t, _row):
        return {i: centre + amp * (1.0 if (t * freq) % 1.0 < 0.5 else -1.0)
                for i in ids}
    rows = bus.capture(seconds=max(cycles / freq, 0.6), rate_hz=rate,
                       command=wave, warn_overrun=False)
    bus.write_frame({i: centre for i in ids})
    time.sleep(0.2)
    return rows


def p95_current(rows, dxl_id) -> float:
    v = [abs(r["servos"][dxl_id].get("current", 0.0)) for r in rows]
    return float(np.percentile(v, 95)) if v else float("nan")


# --------------------------------------------------------------------------
# lag-origin
# --------------------------------------------------------------------------

def cmd_lag_origin(bus, args) -> None:
    """Is the drive lag the motor or the firmware velocity loop?"""
    ids = bus.ids
    freqs = args.freqs
    print(f"\nA. PWM MODE (Operating Mode {MODE_PWM}) -- no velocity loop at all.")
    print(f"   Duty straight to the bridge, so this is the motor's own")
    print(f"   electromechanical constant tau_m = J*R/(kt*ke).")
    for i in ids:
        set_mode_and_gains(bus, i, MODE_PWM)
    m = (IndirectMap({i: bus.tables[i] for i in ids})
         .read("Realtime Tick").read("Present Position")
         .write("Goal PWM", label="goal"))
    bus.apply_map(m)
    bus.torque(True, ids)
    sw = {i: [] for i in ids}
    for f in freqs:
        rows = square(bus, ids, args.duty, f, rate=args.rate)
        for i in ids:
            sw[i].append(swing(rows, i, f, velocity=True))
    bus.torque(False, ids)
    for i in ids:
        s_inf, tau = fit_tau(freqs, sw[i])
        print(f"   id {i}: swings {[round(x, 2) for x in sw[i]]} rad/s"
              f"  -> S_inf {s_inf:6.2f} rad/s, tau {1000 * tau:5.1f} ms")

    print(f"\nB. VELOCITY MODE, sweeping Velocity P Gain(78).")
    print(f"   A tau that MOVES with a firmware gain is not a motor constant.")
    v_limit = min(bus.read(i, "Velocity Limit") for i in ids)
    amp = min(args.amp, 0.95 * v_limit)      # trap 4
    print(f"   Velocity Limit {v_limit:.2f} rad/s -> amplitude {amp:.2f} rad/s")
    m = (IndirectMap({i: bus.tables[i] for i in ids})
         .read("Realtime Tick").read("Present Position")
         .write("Goal Velocity", label="goal"))
    for kvp in args.kvps:
        for i in ids:
            set_mode_and_gains(bus, i, MODE_VELOCITY, Velocity_P_Gain=kvp)
        bus.apply_map(m)
        bus.torque(True, ids)
        sw = {i: [] for i in ids}
        for f in freqs:
            rows = square(bus, ids, amp, f, rate=args.rate)
            for i in ids:
                sw[i].append(swing(rows, i, f, velocity=True))
        bus.torque(False, ids)
        for i in ids:
            s_inf, tau = fit_tau(freqs, sw[i])
            print(f"   KVP {kvp:5d} id {i}: swings {[round(x, 1) for x in sw[i]]}"
                  f" -> S_inf {s_inf:6.2f} rad/s, tau {1000 * tau:5.1f} ms")


# --------------------------------------------------------------------------
# modes
# --------------------------------------------------------------------------

def cmd_modes(bus, args) -> None:
    """Position vs current-based position, 2x2 against the derivative gain."""
    ids = bus.ids
    centre = {i: position_centre(bus, i) for i in ids}       # trap 2
    print(f"\nPOSITION SQUARE WAVE, both modes x D gain, P fixed at {args.pgain}")
    print(f"centre {[round(np.degrees(c), 1) for c in centre.values()]} deg, "
          f"from each servo's Min/Max Position Limit")
    print(f"\n{'mode':>6s} {'P':>5s} {'D':>5s} {'amp':>5s} {'f':>4s} {'id':>4s} "
          f"{'ratio':>6s} {'counts':>7s} {'I_p95':>6s}")
    rows_out = []
    for mode in (MODE_POSITION, MODE_CURRENT_POSITION):
        for dgain in args.dgains:
            for i in ids:
                set_mode_and_gains(bus, i, mode, Position_P_Gain=args.pgain,
                                   Position_D_Gain=dgain, Position_I_Gain=0)
            m = (IndirectMap({i: bus.tables[i] for i in ids})
                 .read("Realtime Tick").read("Present Position")
                 .read({i: ("Present Current" if "Present Current" in bus.tables[i]
                            else "Present Load") for i in ids}, label="current")
                 .write("Goal Position", label="goal"))
            bus.apply_map(m)
            for i in ids:                     # apply_map drops torque; confirm
                assert bus.read_raw(i, "Position P Gain") == args.pgain
                assert bus.read_raw(i, "Position D Gain") == dgain
            bus.torque(True, ids)
            if not settle_at(bus, ids, centre):               # trap 3
                print(f"   WARNING: mode {mode} D {dgain}: did not reach centre "
                      f"in time; the first condition may include the slew")
            for amp_deg in args.amps:
                for f in args.freqs:
                    a = np.radians(amp_deg)
                    rows = square(bus, ids, a, f, centre=centre[ids[0]],
                                  rate=args.rate)
                    for i in ids:
                        sw = np.degrees(swing(rows, i, f))
                        cur = p95_current(rows, i)
                        rows_out.append(dict(mode=mode, D=dgain, amp=amp_deg,
                                             hz=f, id=i, ratio=sw / (2 * amp_deg)))
                        print(f"{mode:>6d} {args.pgain:5d} {dgain:5d} "
                              f"{amp_deg:5.1f} {f:4d} {i:4d} "
                              f"{sw / (2 * amp_deg):6.2f} {sw / COUNT_DEG:7.1f} "
                              f"{cur:6.3f}")
            bus.torque(False, ids)

    print("\nHOW TO READ IT. A LINEAR controller gives a ratio that depends on")
    print("FREQUENCY ONLY -- equal at 1 deg and 5 deg. A ratio that IMPROVES as")
    print("the amplitude shrinks is a nonlinear map whose gain rises toward zero")
    print("error, which is what regulating BUS current (I_bus = duty*I_phase, so")
    print("duty ~ sqrt(I_cmd) at stall) predicts. A ratio far above 1 is a limit")
    print("cycle, not tracking -- check the current draw before believing it.")


# --------------------------------------------------------------------------
# no-load
# --------------------------------------------------------------------------

def cmd_no_load(bus, args) -> None:
    """Duty sweep on a bare shaft: speed, and a current consistency check."""
    ids = bus.ids
    for i in ids:
        set_mode_and_gains(bus, i, MODE_PWM)
    m = (IndirectMap({i: bus.tables[i] for i in ids})
         .read("Realtime Tick").read("Present Position")
         .read({i: ("Present Current" if "Present Current" in bus.tables[i]
                    else "Present Load") for i in ids}, label="current")
         .read("Present Input Voltage")
         .write("Goal PWM", label="goal"))
    bus.apply_map(m)
    bus.torque(True, ids)
    print(f"\n{'duty':>6s} {'id':>4s} {'w/rad/s':>9s} {'I or load':>10s} {'vin':>6s}")
    data = {i: [] for i in ids}
    for duty in args.duties:
        bus.write_frame({i: duty for i in ids})
        time.sleep(args.dwell)
        rows = bus.capture(seconds=0.4, rate_hz=args.rate, warn_overrun=False)
        for i in ids:
            t, rad = timebase(rows, i)
            w = (rad[-1] - rad[0]) / (t[-1] - t[0]) if t[-1] > t[0] else float("nan")
            cur = float(np.median([r["servos"][i]["current"] for r in rows]))
            vin = float(np.median([r["servos"][i]["Present Input Voltage"]
                                   for r in rows]))
            data[i].append((duty, w, cur, vin))
            print(f"{duty:6.2f} {i:4d} {w:9.3f} {cur:10.3f} {vin:6.1f}")
    bus.write_frame({i: 0.0 for i in ids})
    time.sleep(0.3)
    bus.torque(False, ids)

    print("\nCHECKS")
    for i in ids:
        a = np.array(data[i])
        big = a[a[:, 0] >= 0.15]
        slope = float(np.polyfit(big[:, 0], big[:, 1], 1)[0])
        print(f"  id {i}: speed is linear in duty above 0.15, extrapolating to "
              f"{slope:.2f} rad/s at full duty")
        # Is the reported current usable as PHASE current? At steady state
        # duty*vin = kt*w + I*R; fitting kt and R and checking the implied stall
        # current vin/R against the datasheet is the cheapest consistency test
        # there is, and it is the one that fails.
        A = np.column_stack([a[:, 1], a[:, 2]])
        (kt, R), *_ = np.linalg.lstsq(A, a[:, 0] * a[:, 3], rcond=None)
        mono = np.all(np.diff(a[:, 2]) >= -1e-9)
        print(f"         naive fit duty*vin = kt*w + I*R -> kt {kt:.3f}, R {R:.3f} ohm")
        print(f"         => implied stall current vin/R = {a[0, 3] / R:.1f} A")
        print(f"         current monotonic in duty: {mono}")
        if not mono or a[0, 3] / R > 5:
            print(f"         *** REPORTED CURRENT IS NOT PHASE CURRENT. A stall")
            print(f"             current this large, or a non-monotonic reading")
            print(f"             at monotonically rising speed, is what BUS-side")
            print(f"             sensing looks like: I_bus = duty * I_phase.")
            print(f"             Calibrate it at stall (reported current vs PWM:")
            print(f"             parabola = bus, straight line = phase) before")
            print(f"             using any current number quantitatively.")


# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=3_000_000)
    # COMMA-SEPARATED, not nargs="+": a variadic global option swallows the
    # subcommand name ("--ids 103 104 modes" reads `modes` as another id).
    ap.add_argument("--ids", required=True,
                    type=lambda s: tuple(int(x) for x in s.split(",")),
                    metavar="103,104")
    ap.add_argument("--rate", type=float, default=500.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    a = sub.add_parser("lag-origin", help="PWM mode vs a Velocity P Gain sweep")
    a.add_argument("--freqs", type=float, nargs="+", default=[5, 10, 15, 25, 40])
    a.add_argument("--duty", type=float, default=0.5)
    a.add_argument("--amp", type=float, default=10.5, help="rad/s, clamped")
    a.add_argument("--kvps", type=int, nargs="+", default=[25, 50, 100, 400, 1600])
    a.set_defaults(fn=cmd_lag_origin)

    b = sub.add_parser("modes", help="position(3) vs current-based position(5)")
    b.add_argument("--amps", type=float, nargs="+", default=[1.0, 5.0],
                   help="degrees")
    b.add_argument("--freqs", type=int, nargs="+", default=[10, 20])
    b.add_argument("--pgain", type=int, default=800)
    b.add_argument("--dgains", type=int, nargs="+", default=[0, 1400])
    b.set_defaults(fn=cmd_modes)

    c = sub.add_parser("no-load", help="duty sweep -> speed and a current check")
    c.add_argument("--duties", type=float, nargs="+",
                   default=[0.05, 0.15, 0.25, 0.40, 0.60, 0.80, 1.00])
    c.add_argument("--dwell", type=float, default=0.35, help="s, >> 5 tau")
    c.set_defaults(fn=cmd_no_load)

    args = ap.parse_args()

    bus = DynamixelBus(args.port, baud=args.baud, ids=args.ids).open()
    saved = {i: {r: bus.read_raw(i, r) for r in _RESTORE
                 if r in bus.tables[i]} for i in bus.ids}
    print("as found:")
    for i, regs in saved.items():
        print(f"  id {i} {bus.tables[i].name}: {regs}")
    try:
        bus.prepare()          # Return Delay Time 0, profiles 0
        args.fn(bus, args)
    finally:
        for i, regs in saved.items():
            try:
                bus.torque(False, [i])
                # Mode first: writing it resets the gains, so restoring gains
                # before the mode would undo itself. Same trap, in reverse.
                bus.write_raw(i, "Operating Mode", regs["Operating Mode"])
                for r, v in regs.items():
                    if r != "Operating Mode":
                        bus.write_raw(i, r, v)
            except Exception as e:                     # noqa: BLE001
                print(f"RESTORE FAILED id {i}: {e}")
        bus.close()
        print("\nrestored modes and gains, torque off, bus closed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
