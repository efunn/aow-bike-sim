"""What each policy is DOING while told to hold station: the three action
channels as time series and as spectra.

The hold command is (0,0) velocity with the heading already on target, so in
principle nothing needs to move. In practice every policy saws the bars
continuously -- |steer_rate| runs at 4.7-6.6 rad/s against an 8.0 bound -- and
carries a standing hub bias of about -0.26 m/s. This script asks what that
activity actually looks like: is it a clean limit cycle, at what frequency,
and is `hub` phase-locked to the steering (which would make the drift a
rectified oscillation rather than a static bias)?

Actions are plotted NORMALISED to their bounds, i.e. exactly what the network
emits before scale_action, so all three channels share a [-1, 1] scale.

  python analysis/hold_spectrum.py
  python analysis/hold_spectrum.py --seconds 20 --out /tmp/s.png

Read-only: loads moves/*.npz and writes one PNG.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz
from rsa_policies import POLICIES, REPO

CHANNELS = ("steer_rate", "hub", "diff")


def hold_trace(pol, env, steps):
    """Normalised actions + steer angle and roll, under a pure hold command."""
    obs, _ = env.reset(seed=7, options={"v_cmd": (0.0, 0.0),
                                        "psi_cmd_rel": 0.0, "difficulty": 1.0})
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    A, steer, roll = [], [], []
    for _ in range(steps):
        a = np.asarray(pol.action(obs), float)
        A.append(a / scale)
        na = (a / scale)[:env.action_space.shape[0]]
        obs, _r, term, trunc, _i = env.step(na)
        steer.append(np.degrees(float(env.data.qpos[env._sj])))
        roll.append(np.degrees(extract_state(env.data, env._p0).roll))
        if term or trunc:
            break
    return np.array(A), np.array(steer), np.array(roll)


def spectrum(x, dt):
    """One-sided amplitude spectrum of the MEAN-REMOVED signal. The mean is
    the standing bias and is reported separately -- leaving it in would put a
    spike at DC that dwarfs everything oscillatory."""
    x = np.asarray(x, float) - np.mean(x)
    n = len(x)
    w = np.hanning(n)
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(n, dt)
    amp = 2.0 * np.abs(X) / np.sum(w)
    return f, amp


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--show-seconds", type=float, default=3.0,
                    help="how much of the time series to draw")
    ap.add_argument("--out", type=Path,
                    default=Path(__file__).parent / "hold_spectrum.png")
    args = ap.parse_args()

    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = GeneralEnv(params, cfg)
    dt = env.ctrl_dt
    steps = int(args.seconds / dt)

    data = {}
    for key in POLICIES:
        pol = load_policy_npz(MOVES_DIR / f"{key}.npz")
        data[key] = hold_trace(pol, env, steps)

    print(f"hold, {args.seconds:.0f} s at {1/dt:.0f} Hz "
          f"(Nyquist {0.5/dt:.0f} Hz)\n")
    print(f"{'policy':16} {'channel':>11} {'mean':>8} {'rms(ac)':>8} "
          f"{'peak f':>8} {'peak amp':>9}")
    peaks = {}
    for key, (A, steer, roll) in data.items():
        for c, name in enumerate(CHANNELS):
            f, amp = spectrum(A[:, c], dt)
            k = int(np.argmax(amp[1:]) + 1)            # skip the DC bin
            peaks[(key, name)] = (f[k], amp[k])
            print(f"{key:16} {name:>11} {A[:, c].mean():>+8.3f} "
                  f"{np.std(A[:, c]):>8.3f} {f[k]:>7.2f}Hz {amp[k]:>9.3f}")
        print(f"{'':16} {'steer[deg]':>11} {steer.mean():>+8.1f} "
              f"{np.std(steer):>8.1f}")

    # Is hub phase-locked to the steering? If the drift were a rectified
    # oscillation, hub would ride at the steer frequency with a fixed phase.
    print("\nhub vs steer_rate at the steering peak:")
    for key, (A, _s, _r) in data.items():
        fs, _ = peaks[(key, "steer_rate")]
        n = len(A)
        f = np.fft.rfftfreq(n, dt)
        k = int(np.argmin(np.abs(f - fs)))
        S = np.fft.rfft(A[:, 0] - A[:, 0].mean())[k]
        H = np.fft.rfft(A[:, 1] - A[:, 1].mean())[k]
        coh = np.abs(H) / (np.abs(S) + 1e-12)
        print(f"  {key:16} f={fs:.2f}Hz  |hub|/|steer| = {coh:.3f}  "
              f"phase = {np.degrees(np.angle(H / S)):+.0f} deg")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nno PNG: matplotlib is not installed in this interpreter")
        return

    n_show = int(args.show_seconds / dt)
    fig, axes = plt.subplots(2, len(data), figsize=(5.0 * len(data), 6.4),
                             squeeze=False)
    t = np.arange(n_show) * dt
    for j, (key, (A, steer, roll)) in enumerate(data.items()):
        ax = axes[0][j]
        for c, name in enumerate(CHANNELS):
            ax.plot(t, A[:n_show, c], lw=1.0, label=name)
        ax.axhline(0, color="0.7", lw=0.6)
        ax.set_title(key)
        ax.set_xlabel("s")
        ax.set_ylim(-1.05, 1.05)
        if j == 0:
            ax.set_ylabel("action (fraction of bound)")
            ax.legend(fontsize=7, loc="upper right")

        ax = axes[1][j]
        for c, name in enumerate(CHANNELS):
            f, amp = spectrum(A[:, c], dt)
            ax.semilogy(f, np.maximum(amp, 1e-5), lw=1.0, label=name)
        ax.set_xlabel("Hz")
        ax.set_xlim(0, 0.5 / dt)
        if j == 0:
            ax.set_ylabel("amplitude (mean removed)")
    fig.suptitle("hold command: action channels, time series and spectra")
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
