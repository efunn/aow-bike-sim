"""Stage 0b: the drive servo reversal test (`servo-protocol.md` section 2).

THE QUESTION. Every `general_rl*` policy commands the drive faster than the
hardware may be able to execute. `general_rl_smooth_diff_og` reverses the hub
21.8 times a second under a stand-still command; the steer channel does the same
at 15-32 sign flips/s. Neither has ever been checked against a servo. This
measures what an XC430-W150 actually achieves when told to reverse, as a
function of amplitude and frequency.

(An earlier version of this paragraph said the sim's drive plant was "~31x too
stiff and ~31x too fast". That was true before commit b0a97a3, 2026-08-22, and
was read off a paragraph in `servo-protocol.md` that had not been updated since.
`drive_kv` has been 0.016016 for ten days and the model's settling constant is
18.7 ms, not 0.60. The reason to run this test is that the plant is still
UNMEASURED, not that it is known to be wrong.)

WHAT IT MEASURES. Square wave on Goal Velocity, amplitude A and frequency f
swept independently. Per condition:

  * `swing_ratio` -- achieved peak-to-peak shaft velocity over commanded (2A).
    1.0 means the servo tracked; the frequency where this falls through ~0.5 is
    the reversal envelope the reward has to respect.
  * `travel_ratio` -- shaft revolutions actually turned over what an ideal
    servo would turn. The chatter figure in `servo-protocol.md` is stated as rim
    metres, and this is the same quantity before the wheel radius.
  * `pwm_p95` and `load_p95` -- how much of the actuator is being spent to get
    that, so a pass at high amplitude is not read as free.

BOTH DRIVE SERVOS AT ONCE, deliberately. They are independent samples of the
same part under one bus and one command stream, which is (a) twice the data per
run, (b) the only way to see unit-to-unit spread, and (c) the loading the real
bus carries, so the frame rate here is the frame rate the bike gets. It is NOT
extra mechanical load -- the shafts are not coupled.

    python analysis/servo_reversal.py --port /dev/cu.usbserial-FTB8HNE3
    python analysis/servo_reversal.py --port ... --amps 0.25 --freqs 5 10 25
    python analysis/servo_reversal.py --port ... --dry-run     # no torque

THE CAVEAT THAT DECIDES HOW TO READ THE RESULT. With a bare shaft this measures
the servo's own rotor and gearbox inertia and nothing else. The bike's drive
sees the omni wheel through `belt_ratio` 3, and reflected wheel inertia is what
makes a reversal expensive -- so **a bare-motor pass is an upper bound, not an
answer.** Re-run it on the stage-1 drivetrain station (servo -> belt -> input
shaft -> hub) before concluding anything about the policies. The script prints
this alongside the table rather than trusting anyone to remember it.

SAFETY. Velocity mode is verified before torque is enabled, amplitude is clamped
to the servo's own Velocity Limit, and the finally-block ramps to zero and drops
torque on every exit path including Ctrl-C. Nothing is written to EEPROM except
Return Delay Time and the two Profile registers, which `DynamixelBus.prepare`
sets because a non-zero profile would make this measure the trajectory generator
instead of the velocity loop.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "src")

from aow_sim.build_model import load_params                     # noqa: E402
from aow_sim.hw.control_table import MODEL_NUMBERS              # noqa: E402
from aow_sim.hw.dynamixel import MODE_VELOCITY, DynamixelBus, IndirectMap  # noqa: E402

TICK_WRAP = 32768
POS_WRAP = 4096          # velocity mode reports position over one turn


def _unwrap(counts: np.ndarray) -> np.ndarray:
    """Single-turn position -> continuous, in counts.

    Velocity Control Mode reports Present Position over ONE rotation, so a
    continuously turning shaft rolls 4095 -> 0. Same shortest-path unwrap as
    `hw.dynamixel._pos_delta`, vectorised over a whole capture.
    """
    d = np.diff(counts)
    d = ((d + POS_WRAP // 2) % POS_WRAP) - POS_WRAP // 2
    return np.concatenate([[counts[0]], counts[0] + np.cumsum(d)])


def analyse(rows: list, dxl_id: int, amp: float, freq: float,
            counts_per_rev: float) -> dict:
    """One condition, one servo -> the metrics above."""
    sv = [r["servos"][dxl_id] for r in rows]
    tick = np.array([s["Realtime Tick"] for s in sv], float)
    pos_rad = np.array([s["Present Position"] for s in sv], float)
    vel_rep = np.array([s["Present Velocity"] for s in sv], float)
    pwm = np.array([s["Present PWM"] for s in sv], float)
    load = np.array([s["torque_sense"] for s in sv], float)

    t = np.cumsum(np.concatenate([[0.0], np.diff(tick) % TICK_WRAP])) * 1e-3
    counts = _unwrap(pos_rad * counts_per_rev / (2 * np.pi))
    rad = counts / counts_per_rev * 2 * np.pi

    dt = np.diff(t)
    good = dt > 0
    v = np.zeros_like(t)
    v[1:][good] = np.diff(rad)[good] / dt[good]

    # Discard the first quarter period: the step into the wave is a transient,
    # not the steady reversal being measured.
    settle = t > (0.25 / freq)
    v_s, pwm_s, load_s = v[settle], pwm[settle], load[settle]

    swing = np.percentile(v_s, 97) - np.percentile(v_s, 3)
    # One encoder count per sample is the smallest velocity a differenced
    # position can report. At 4096 counts/rev and 500 Hz that is 0.77 rad/s, and
    # a condition whose whole swing is a few counts is measuring quantisation
    # rather than the servo -- the amp 0.10 row read a BETTER swing_ratio at
    # 25 Hz than at 15 Hz for exactly this reason. Flagged, not dropped.
    quant = (2 * np.pi / counts_per_rev) / np.median(dt[good])
    travel = float(np.abs(np.diff(rad[settle])).sum())
    ideal = amp * (t[settle][-1] - t[settle][0])   # |v| = amp throughout
    return {
        "id": dxl_id,
        "swing_ach": float(swing),
        "swing_ratio": float(swing / (2 * amp)) if amp else float("nan"),
        "travel_rev": travel / (2 * np.pi),
        "travel_ratio": float(travel / ideal) if ideal else float("nan"),
        "vel_rep_swing": float(np.percentile(vel_rep[settle], 97)
                               - np.percentile(vel_rep[settle], 3)),
        "pwm_p95": float(np.percentile(np.abs(pwm_s), 95)),
        "load_p95": float(np.percentile(np.abs(load_s), 95)),
        "n": int(settle.sum()),
        "quant": float(quant),
        "quantised": bool(swing < 4 * quant),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[101, 102],
                    help="drive servos, both run simultaneously")
    ap.add_argument("--amps", type=float, nargs="+", default=[0.1, 0.25, 0.5, 1.0],
                    help="square-wave amplitude as a fraction of no-load speed")
    ap.add_argument("--freqs", type=float, nargs="+", default=[5, 10, 15, 25],
                    help="square-wave frequency [Hz]")
    ap.add_argument("--rate", type=float, default=500.0, help="sample rate [Hz]")
    ap.add_argument("--cycles", type=float, default=8.0,
                    help="whole cycles per condition (sets the dwell)")
    ap.add_argument("--save", metavar="NPZ",
                    help="write every captured frame here, so the sweep can be "
                         "re-analysed without re-running the hardware")
    ap.add_argument("--dry-run", action="store_true",
                    help="do everything except enable torque")
    args = ap.parse_args()

    params = load_params()
    no_load = params["servos"]["xc430_w150"]["no_load_rpm"] * 2 * np.pi / 60
    counts_per_rev = 4096.0
    print(f"XC430-W150 no-load {no_load:.2f} rad/s "
          f"({params['servos']['xc430_w150']['no_load_rpm']:.0f} rpm, 12 V datasheet)")

    bus = DynamixelBus(args.port, baud=args.baud, ids=tuple(args.ids)).open()
    try:
        for i in bus.ids:
            ct = bus.tables[i]
            if ct.model_number not in MODEL_NUMBERS:
                raise SystemExit(f"id {i}: unknown model {ct.model_number}")
            mode = bus.read_raw(i, "Operating Mode")
            if mode != MODE_VELOCITY:
                raise SystemExit(
                    f"id {i} is in Operating Mode {mode}, not "
                    f"{MODE_VELOCITY} (velocity). Refusing to enable torque: a "
                    f"velocity command in position mode is a position command.")
        bus.prepare()          # RDT 0, profiles 0 -- see the module docstring
        v_limit = min(bus.read(i, "Velocity Limit") if "Velocity Limit" in bus.tables[i]
                      else no_load for i in bus.ids)
        print(f"Velocity Limit {v_limit:.2f} rad/s; "
              f"servos {list(bus.ids)}; sampling {args.rate:g} Hz\n")

        imap = (IndirectMap(bus.tables)
                .read("Realtime Tick").read("Present Position")
                .read("Present Velocity").read("Present PWM")
                .read({i: ("Present Current" if "Present Current" in bus.tables[i]
                           else "Present Load") for i in bus.ids}, label="torque_sense")
                .write("Goal Velocity", label="goal"))
        bus.apply_map(imap, verify=True)

        if args.dry_run:
            print("--dry-run: torque stays off, commands are computed not sent")
        else:
            bus.torque(True)

        results, raw = [], {}
        hdr = (f"{'amp':>6s} {'f/Hz':>5s} {'id':>4s} {'swing_ach':>10s} "
               f"{'ratio':>6s} {'rep_sw':>7s} {'travel':>7s} {'t_rat':>6s} "
               f"{'pwm95':>6s} {'load95':>7s}")
        print(hdr + "\n" + "-" * len(hdr))
        for amp_frac in args.amps:
            amp = min(amp_frac * no_load, 0.98 * v_limit)
            for freq in args.freqs:
                def wave(t, _row, amp=amp, freq=freq):
                    v = amp * (1.0 if (t * freq) % 1.0 < 0.5 else -1.0)
                    return {} if args.dry_run else {i: v for i in bus.ids}

                rows = bus.capture(seconds=args.cycles / freq, rate_hz=args.rate,
                                   command=wave, warn_overrun=False)
                if not args.dry_run:
                    bus.write_frame({i: 0.0 for i in bus.ids})
                    time.sleep(0.25)          # let it stop before the next step
                if args.save:
                    for i in bus.ids:
                        raw.setdefault(f"{i}_{amp_frac}_{freq}", np.array(
                            [[r["servos"][i]["Realtime Tick"],
                              r["servos"][i]["Present Position"],
                              r["servos"][i]["Present Velocity"],
                              r["servos"][i]["Present PWM"],
                              r["servos"][i]["torque_sense"]] for r in rows]))
                for i in bus.ids:
                    m = analyse(rows, i, amp, freq, counts_per_rev)
                    m.update(amp_frac=amp_frac, amp=amp, freq=freq,
                             n_rows=len(rows))
                    results.append(m)
                    flag = " q" if m["quantised"] else ""
                    print(f"{amp_frac:6.2f} {freq:5.0f} {i:4d} "
                          f"{m['swing_ach']:10.2f} {m['swing_ratio']:6.2f} "
                          f"{m['vel_rep_swing']:7.2f} {m['travel_rev']:7.2f} "
                          f"{m['travel_ratio']:6.2f} {m['pwm_p95']:6.3f} "
                          f"{m['load_p95']:7.3f}{flag}")
        if args.save:
            np.savez_compressed(args.save, **raw)
            print(f"\nraw frames -> {args.save}")
    finally:
        try:
            bus.write_frame({i: 0.0 for i in bus.ids})
            time.sleep(0.3)
        except Exception:
            pass
        bus.torque(False)
        bus.close()
        print("\ntorque off, bus closed")

    if results:
        # FIRST-ORDER FIT. A square wave of amplitude A into a lag tau settles to
        # a peak-to-peak of 2A*tanh(T/(4*tau)), so swing_ratio = tanh(1/(4 f
        # tau)) and depends on FREQUENCY ONLY -- which is why the measured ratio
        # is the same at amp 0.25, 0.50 and 1.00. A slew-rate limit would make
        # the large amplitudes worse instead, so the amplitude-invariance is the
        # evidence for the model, not the fit residual.
        usable = [r for r in results if not r["quantised"] and 0.02 < r["swing_ratio"] < 0.99]
        if usable:
            taus = [1.0 / (4 * r["freq"] * np.arctanh(min(r["swing_ratio"], 0.999)))
                    for r in usable]
            tau = float(np.median(taus))
            print(f"\nfirst-order fit over {len(usable)} unquantised conditions:")
            print(f"  tau = {1000*tau:.1f} ms   (spread "
                  f"{1000*np.percentile(taus,10):.1f}-{1000*np.percentile(taus,90):.1f} ms)")
            print(f"  tau_m = J*R/(kt*ke) predicted 18.7 ms; the sim's own")
            print(f"  input_armature/drive_kv is also 18.7 ms (since b0a97a3,")
            print(f"  2026-08-22). Do NOT quote the old 0.60 ms -- that was the")
            print(f"  pre-fix drive_kv 0.5 and is stale wherever it appears.")
        if any(r["quantised"] for r in results):
            n = sum(r["quantised"] for r in results)
            print(f"\n{n} conditions were at the encoder quantisation floor "
                  f"(~{results[0]['quant']:.2f} rad/s at this sample rate) and "
                  f"are excluded from the fit; their swing_ratio is noise.")

        print("\nreversal envelope: highest frequency holding swing_ratio >= 0.5")
        for amp_frac in args.amps:
            for i in args.ids:
                ok = [r["freq"] for r in results
                      if r["amp_frac"] == amp_frac and r["id"] == i
                      and r["swing_ratio"] >= 0.5 and not r["quantised"]]
                print(f"  amp {amp_frac:4.2f} id {i}: "
                      f"{max(ok):.0f} Hz" if ok else
                      f"  amp {amp_frac:4.2f} id {i}: fails at every frequency tested")
        print("\nBARE SHAFT: rotor + gearbox inertia only. The bike's drive sees "
              "the omni wheel\nthrough belt_ratio 3, and reflected wheel inertia "
              "is what makes a reversal\nexpensive -- so this is an UPPER BOUND. "
              "Re-run on the stage-1 drivetrain\nstation before concluding "
              "anything about the trained policies.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
