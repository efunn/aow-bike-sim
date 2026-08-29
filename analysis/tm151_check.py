"""Hold a real TM151 recording up against the datasheet and against sim_ahrs.

`sim_ahrs.py` models this part from its datasheet, and every AHRS result this
month rests on three numbers. Two are quoted by the vendor. The third is not
quoted anywhere:

    GYRO_NOISE_PP_DPS       <= 0.5 deg/s peak-to-peak    datasheet
    ORIENT_RMS_DEG          0.5 static / 1.5 dynamic     datasheet
    TAU_ORIENT_S = 2.0      correlation time             *** A GUESS ***

This script measures all three from `tm151_record.py`'s output and says, per
row, whether the model is right, conservative, or optimistic. Optimistic is the
one that matters: it means the eval numbers -- `general_rl_odo` surviving 0.90
under a TM151, the truth-trained policies collapsing to 0.20 -- are too kind.

TAU IS THE POINT. It is measured by fitting the autocorrelation of the roll
error at rest -- see `autocorr_tau`, which documents two ways of doing this
that DO NOT WORK and were tried first. The fit does not assume the model is
right: `r2` reports whether the decay is exponential at all, so a bad shape
shows up as a bad fit rather than as a confidently wrong tau.

HOW LONG A CAPTURE, AND HOW MUCH TO BELIEVE IT. Estimating a correlation time
from a finite record is intrinsically noisy, and the estimator here is unbiased
but WIDE. Validated against synthetic processes of known tau, 12 seeds each --
the spread is what ONE capture gives you, not the mean:

    true tau   capture    mean     std     p10 - p90
     2.0 s      300 s     2.09    0.41    1.76 - 2.65
     2.0 s     1200 s     1.98    0.16    1.82 - 2.22
     2.0 s     3600 s     2.03    0.07    1.99 - 2.14
     3.7 s      300 s     3.71    0.91    2.92 - 4.57
     3.7 s     1200 s     3.63    0.44    3.14 - 4.15
     3.7 s     3600 s     3.64    0.19    3.42 - 3.84

The MEAN is right at every length; the SPREAD is the whole story. At 300 s a
tau of 3.7 s comes back somewhere in 2.9-4.6. **Five minutes gets the order of
magnitude, twenty minutes gets ~10%, an hour gets ~5%.** Since the question
being asked is "is the 2.0 s guess about right", five minutes answers it and
twenty settles it -- but the number is reported with an error bar, because a
bare figure from a short capture invites exactly the false precision this
script exists to avoid.

  python analysis/tm151_check.py --tag rest
  python analysis/tm151_check.py --tag rest --sim      # overlay sim_ahrs

AT REST, roll and pitch error are just (reading - mean), because a stationary
unit's true attitude is constant and unknown. That is exactly the quantity the
datasheet's STATIC accuracy row describes, so it is the honest comparison. It
says nothing about the DYNAMIC figure (1.5 deg), which needs a moving unit and
an independent attitude truth -- which a desk capture does not have. That
limit is stated in the output rather than papered over.

Writes analysis/plots/tm151_check_<tag>.png. Read-only otherwise.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from aow_sim.sim_ahrs import (GYRO_BIAS_STABILITY_DPH,  # noqa: E402
                              GYRO_NOISE_PP_DPS, ORIENT_RMS_DEG, PP_TO_SIGMA,
                              TAU_ORIENT_S)


def _load(tag: str) -> dict:
    p = Path(__file__).resolve().parent / "recordings" / f"tm151_{tag}.npz"
    if not p.exists():
        raise SystemExit(
            f"no recording at {p}\n"
            f"  make one: python analysis/tm151_record.py --tag {tag}")
    return dict(np.load(p, allow_pickle=True))


def pick(d: dict, field: str):
    """(values, seconds) for `field` from whichever packet kind carries it.

    THE UNIT MAY NOT STREAM `combo`. A TM151 on stock firmware streams `rpy`,
    `raw_gyro_acc_mag` and `status` as three separate streams, so roll and gyro
    arrive in DIFFERENT packets with their own clocks. Preference order puts
    `combo` first where it exists (one packet, one timestamp, no join needed)
    and falls back to the dedicated packet otherwise.

    Time is returned per field rather than once for the file, because those
    streams are independently sampled and a single shared `dt` is exactly the
    assumption that made an earlier version divide by zero: two packets share
    each tick, so `median(diff(t))` over the merged set is 0.
    """
    for kind in ("combo", "rpy", "raw_gyro_acc_mag", "q_s1_e", "status"):
        key = f"{kind}__{field}"
        if key in d and f"{kind}__t_us" in d:
            t = d[f"{kind}__t_us"] * 1e-6
            return d[key], t - t[0]
    return None, None


def rate_of(t: np.ndarray) -> float:
    """Sample rate from timestamps, ignoring same-tick duplicates."""
    dt = np.diff(t)
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if len(dt) else float("nan")


def autocorr_tau(x: np.ndarray, dt: float, lo: float = 0.35,
                 hi: float = 0.95):
    """(lags, autocorrelation, tau, r2) by fitting log(ACF) over SHORT lags.

    A first-order Gauss-Markov process has corr(lag) = exp(-lag/tau), so tau
    falls out of a straight-line fit to log(ACF). Two things about how that is
    done here were arrived at by getting them wrong first, against a synthetic
    capture whose true tau was 3.7 s:

    READING THE SINGLE 1/e CROSSING IS FAR TOO NOISY. It gave 2.40 s on one
    realisation and 4.22 s on another of the same process -- a spread of nearly
    2x -- because the ACF at lag ~tau is one number estimated from about
    N*dt/tau independent chunks, and at a few hundred chunks its standard error
    is around 0.1 on a value of 0.368. The fit uses the whole decay instead.

    AND THE FIT MUST STAY AT SHORT LAGS. Fitting out to where the ACF has
    decayed to noise (below ~0.15) let long-lag scatter dominate and returned
    21.9 s for the same 3.7 s process. Restricting to ACF in [0.35, 0.95] --
    i.e. lags below about 1.05*tau -- keeps every point in the regime where the
    estimate is precise. The BIASED estimator is deliberately used with it: its
    (N-k)/N taper is negligible when k << N, and it has lower variance than the
    unbiased form, which is inconsistent exactly where the unbiased one blows
    up.

    `r2` reports whether the decay is exponential AT ALL. A poor value means
    Gauss-Markov is the wrong shape for `sim_ahrs` and the model should be
    reconsidered rather than retuned.
    """
    x = np.asarray(x, float)
    x = x - x.mean()
    N = len(x)
    full = np.correlate(x, x, mode="full")[N - 1:]
    ac = full / full[0]                       # biased: low variance, k << N
    lags = np.arange(N) * dt

    band = (ac > lo) & (ac < hi)
    # A SHORT TAU IS RESOLVED BY FEW SAMPLES, and an absolute minimum point
    # count rejects it as "unfittable" when it is merely fast. The real TM151
    # at rest has tau ~0.19 s, which at a 50 Hz ODR is about 10 points in the
    # band -- a flat `< 20` guard called that a failed measurement. Five points
    # is enough for a two-parameter line; what matters more is whether the
    # decay is RESOLVED at all, reported via `dt_per_tau` below.
    if band.sum() < 5:
        return lags, ac, None, float("nan")
    L, A = lags[band], np.log(ac[band])
    slope, intercept = np.polyfit(L, A, 1)
    if slope >= 0:
        return lags, ac, None, float("nan")
    pred = slope * L + intercept
    r2 = 1.0 - np.sum((A - pred) ** 2) / np.sum((A - A.mean()) ** 2)
    return lags, ac, float(-1.0 / slope), float(r2)


def resolution_note(tau: float, dt: float) -> str:
    """Is the decay sampled finely enough to trust? A tau of a few samples is
    a real measurement of an UPPER BOUND, not of the value."""
    k = tau / dt
    if k < 3:
        return f"  <- BARELY RESOLVED: tau is only {k:.1f} samples; raise ODR"
    if k < 10:
        return f"  <- coarse: tau is {k:.1f} samples"
    return ""


def verdict(measured: float, spec: float, higher_is_worse: bool = True) -> str:
    if measured != measured:                      # NaN
        return "not measured"
    r = measured / spec if spec else float("inf")
    if r <= 0.8:
        return "model CONSERVATIVE (real part is better)"
    if r <= 1.2:
        return "model matches"
    return "*** MODEL OPTIMISTIC -- real part is worse ***" if higher_is_worse \
        else "model matches"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", default="rest")
    ap.add_argument("--sim", action="store_true",
                    help="overlay a sim_ahrs run of the same length")
    args = ap.parse_args()

    d = _load(args.tag)
    if not any("__" in k for k in d):
        raise SystemExit(
            f"'{args.tag}' has no payload columns -- it was written by the "
            f"pre-2026-08-27 recorder,\n  which silently dropped them. "
            f"Re-record: python analysis/tm151_record.py --tag {args.tag}")

    rpy, t_rpy = pick(d, "rpy_deg")
    gyro_rad, t_gyro = pick(d, "gyro")
    if rpy is None:
        raise SystemExit("no roll/pitch/yaw in this recording")
    gyro = np.degrees(gyro_rad) if gyro_rad is not None else None

    t = t_rpy
    hz = rate_of(t)
    dt = 1.0 / hz
    n = len(t)
    kinds = ", ".join(f"{k} x{c}" for k, c in d["_kinds"])
    print(f"  {args.tag}: {n} attitude samples, {t[-1]:.1f} s, {hz:.1f} Hz")
    print(f"  streams: {kinds}")
    if gyro is not None and t_gyro is not None:
        print(f"  gyro from a separate stream at {rate_of(t_gyro):.1f} Hz, "
              f"{len(gyro)} samples")
    print()

    roll_err = rpy[:, 0] - rpy[:, 0].mean()
    pitch_err = rpy[:, 1] - rpy[:, 1].mean()

    print(f"  {'quantity':32}{'measured':>12}{'sim_ahrs':>12}  verdict")
    print("  " + "-" * 92)

    g_spec = GYRO_NOISE_PP_DPS * PP_TO_SIGMA
    if gyro is None:
        print(f"  {'gyro':32}{'--':>12}{g_spec:>12.4f}  not in this recording")
    else:
        g_hz = int(round(rate_of(t_gyro)))
        g_sigma = float(np.std(gyro[:, 0]))
        print(f"  {'gyro x noise sigma [deg/s]':32}{g_sigma:>12.4f}"
              f"{g_spec:>12.4f}  {verdict(g_sigma, g_spec)}")
        wins = [np.ptp(gyro[i:i + g_hz, 0])
                for i in range(0, max(1, len(gyro) - g_hz), g_hz)]
        pp1 = float(np.mean(wins))
        print(f"  {'gyro x p-p, 1 s window [deg/s]':32}{pp1:>12.4f}"
              f"{GYRO_NOISE_PP_DPS:>12.4f}  "
              f"{'within spec' if pp1 <= GYRO_NOISE_PP_DPS else 'OVER SPEC'}")

    r_rms = float(np.sqrt(np.mean(roll_err ** 2)))
    p_rms = float(np.sqrt(np.mean(pitch_err ** 2)))
    spec_static = ORIENT_RMS_DEG["tm151_static"][0]
    print(f"  {'roll RMS at rest [deg]':32}{r_rms:>12.4f}{spec_static:>12.4f}"
          f"  {verdict(r_rms, spec_static)}")
    print(f"  {'pitch RMS at rest [deg]':32}{p_rms:>12.4f}{spec_static:>12.4f}"
          f"  {verdict(p_rms, spec_static)}")

    lags, ac, tau, r2 = autocorr_tau(roll_err, dt)
    # Empirical 1-sigma, fitted to the spread table in the docstring:
    # rel_std ~ 2.1 * sqrt(tau / T). Reproduces 23%/17% at 300 s and 6.7% at
    # 3600 s against measured 25%/20% and 5.2%. An error bar, not a result.
    tau_sd = 2.1 * np.sqrt(tau / t[-1]) * tau if tau else float("nan")
    tau_s = f"{tau:.2f}+-{tau_sd:.2f}" if tau is not None else "unfittable"
    if tau is None:
        note = "capture too short, or the decay is not exponential"
    else:
        note = f"MEASURED (r2 {r2:.3f})"
        res = resolution_note(tau, dt)
        if r2 <= 0.9:
            note += "  <- POOR FIT: is it Gauss-Markov at all?"
        elif res:
            note += res
        elif t[-1] < 100 * tau:
            note += (f"  <- CAPTURE TOO SHORT: want ~{100*tau:.0f} s "
                     f"for tau {tau:.1f}")
    print(f"  {'roll error tau [s]':32}{tau_s:>12}"
          f"{TAU_ORIENT_S:>12.2f}  {note}")
    if tau is not None and abs(tau - TAU_ORIENT_S) < 2 * tau_sd:
        print(f"  {'':32}{'':12}{'':12}  sim_ahrs.TAU_ORIENT_S "
              f"({TAU_ORIENT_S:g} s) is inside 2 sigma of this measurement")

    drift = float(np.polyfit(t, np.unwrap(np.radians(rpy[:, 2])) * 180 / np.pi,
                             1)[0]) * 60.0
    print(f"  {'yaw drift [deg/min]':32}{drift:>12.4f}"
          f"{3.0/25.0:>12.4f}  (datasheet is pure-inertial; a working "
          f"compass should beat it)")
    bias_spec = GYRO_BIAS_STABILITY_DPH / 3600.0
    if gyro is not None:
        print(f"  {'gyro x mean (bias) [deg/s]':32}{np.mean(gyro[:,0]):>12.5f}"
              f"{bias_spec:>12.5f}  (a standing offset is turn-on bias, not "
              f"instability)")

    print(f"\n  WHAT THIS CANNOT SAY: the DYNAMIC orientation figure "
          f"({ORIENT_RMS_DEG['tm151'][0]} deg), which is\n"
          f"  the row that actually drives the eval results. It needs a moving "
          f"unit and an\n  independent attitude truth; a desk capture has "
          f"neither.")

    fig, ax = plt.subplots(4, 1, figsize=(11, 12))
    ax[0].plot(t, roll_err, lw=0.6, label=f"roll, RMS {r_rms:.3f} deg")
    ax[0].plot(t, pitch_err, lw=0.6, label=f"pitch, RMS {p_rms:.3f} deg")
    for s in (spec_static, -spec_static):
        ax[0].axhline(s, color="k", ls="--", lw=0.8)
    ax[0].set_ylabel("deg"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[0].set_title("1. ORIENTATION ERROR at rest (dashed = datasheet static "
                    f"{spec_static} deg RMS)", fontsize=10)

    ax[1].plot(lags, ac, lw=1.2, label="measured roll-error autocorrelation")
    ax[1].plot(lags, np.exp(-lags / TAU_ORIENT_S), "r--", lw=1.0,
               label=f"sim_ahrs.TAU_ORIENT_S, tau={TAU_ORIENT_S} s")
    if tau:
        ax[1].plot(lags, np.exp(-lags / tau), "g-", lw=1.0,
                   label=f"measured tau={tau:.2f} s")
        ax[1].axvline(tau, color="g", lw=0.8, ls=":")
    ax[1].axhline(np.exp(-1), color="k", lw=0.8, ls="--")
    ax[1].set_xlabel("lag [s]"); ax[1].set_ylabel("autocorrelation")
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    ax[1].set_title("2. TAU — the one number sim_ahrs had no source for. "
                    "Exponential decay means Gauss-Markov was the right shape.",
                    fontsize=10)

    if gyro is not None:
        ax[2].plot(t_gyro, gyro[:, 0], lw=0.4,
                   label=f"wx, sigma {np.std(gyro[:,0]):.4f} deg/s")
    ax[2].axhline(g_spec * 3, color="k", ls="--", lw=0.8,
                  label=f"3 sigma of the datasheet figure")
    ax[2].axhline(-g_spec * 3, color="k", ls="--", lw=0.8)
    ax[2].set_ylabel("deg/s"); ax[2].legend(fontsize=8); ax[2].grid(alpha=0.3)
    ax[2].set_title("3. GYRO NOISE at rest", fontsize=10)

    f, P = _welch(roll_err, hz)
    ax[3].loglog(f, P, lw=0.9, label="roll error")
    ax[3].axvline(1 / (2 * np.pi * TAU_ORIENT_S), color="r", ls="--", lw=0.9,
                  label=f"corner of tau={TAU_ORIENT_S} s")
    if tau:
        ax[3].axvline(1 / (2 * np.pi * tau), color="g", ls="--", lw=0.9,
                      label=f"corner of the measured tau={tau:.2f} s")
    ax[3].set_xlabel("Hz"); ax[3].set_ylabel("PSD [deg^2/Hz]")
    ax[3].legend(fontsize=8); ax[3].grid(alpha=0.3, which="both")
    ax[3].set_title("4. WHERE THE ERROR LIVES IN FREQUENCY — a flat shelf "
                    "rolling off at the corner is what Gauss-Markov predicts",
                    fontsize=10)

    fig.suptitle(f"TM151 '{args.tag}' against sim_ahrs — {n} samples, "
                 f"{t[-1]:.0f} s, {hz:.0f} Hz", y=0.995)
    fig.tight_layout()
    out = Path(__file__).resolve().parent / "plots" / f"tm151_check_{args.tag}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"\n  wrote {out}")
    return 0


def _welch(x, fs, nperseg=4096):
    """Small Welch PSD, so this does not pull in scipy for one call."""
    nperseg = min(nperseg, len(x))
    step = nperseg // 2
    win = np.hanning(nperseg)
    segs = [x[i:i + nperseg] * win for i in range(0, len(x) - nperseg + 1, step)]
    if not segs:
        segs = [x[:nperseg] * win[:len(x)]]
    P = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    P /= (fs * (win ** 2).sum())
    return np.fft.rfftfreq(nperseg, 1 / fs)[1:], P[1:]


if __name__ == "__main__":
    raise SystemExit(main())
