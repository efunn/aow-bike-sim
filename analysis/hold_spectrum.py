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

GROUND TRUTH, printed alongside. The spectra say what the actuators are doing;
they cannot say whether any of it is useful, and a policy that buzzes
consistently looks no different from one that stands still. So the same
rollouts also report:

  drift / net displacement -- the zero command asks the bike not to move. This
    is the gate on reading `hold` as the origin of command space anywhere else
    (see the `centre="hold"` note in move_confusion.py): if the bike wanders,
    "the zero-command state" is a moving target.

  rear airborne % and touchdown force -- the chatter was severe enough in the
    0.005 policies to bounce the rear wheel off the floor 28-48% of the time,
    landing at ~1.1-1.4x body weight. A bike that is airborne half the time is
    not balancing on its contacts, it is hopping on them, and no action-space
    statistic shows that.

  wheel revolutions vs chassis travel -- the rear wheel net-rotates ~6 turns
    BACKWARD per 10 s while the chassis goes nowhere near what rolling
    predicts, and some policies end up travelling forward while the wheel
    turns backward. Rolling is being replaced by slip, which is the mechanism
    that makes the whole hold behaviour work at all.

  python analysis/hold_spectrum.py
  python analysis/hold_spectrum.py --seconds 20 --out /tmp/s.png

Read-only: loads moves/*.npz and writes one PNG.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.balance import extract_state
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz
from rsa_policies import POLICIES, REPO

CHANNELS = ("steer_rate", "hub", "diff")

# Everything the rear omni wheel can touch the floor with. The rollers carry
# the contact in normal running, but the hub and ring shells can reach it too
# and a "the wheel is airborne" claim must not miss those.
_REAR_PREFIX = ("roller_", "ring_body", "hub_body")


def _floor_normal_forces(model, data, floor, rear, front):
    """Normal force [N] currently carried by the rear wheel and by the front
    tire. Read off the contacts rather than the actuators: whether the bike is
    standing on the ground is not inferable from what it commanded."""
    fr = ft = 0.0
    buf = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if floor not in (c.geom1, c.geom2):
            continue
        other = c.geom2 if c.geom1 == floor else c.geom1
        mujoco.mj_contactForce(model, data, i, buf)
        if other in rear:
            fr += buf[0]
        elif other in front:
            ft += buf[0]
    return fr, ft


def hold_trace(pol, env, steps):
    """Normalised actions, steer angle, roll, and the ground truth: contact
    forces, chassis motion and rear-wheel rotation, under a pure hold."""
    model, data = env.model, env.data
    gname = [model.geom(i).name for i in range(model.ngeom)]
    floor = gname.index("floor")
    rear = {i for i, n in enumerate(gname) if n.startswith(_REAR_PREFIX)}
    front = {i for i, n in enumerate(gname) if n == "front_tire"}
    weight = float(model.body_subtreemass[model.body("chassis").id] * 9.81)
    hub_adr = model.joint("hub_spin").qposadr[0]

    obs, _ = env.reset(seed=7, options={"v_cmd": (0.0, 0.0),
                                        "psi_cmd_rel": 0.0, "difficulty": 1.0})
    b = pol.bounds
    scale = np.array([b.steer_rate_max, b.hub_max, b.diff_max])
    hub0 = float(data.qpos[hub_adr])
    A, steer, roll, speed, f_rear, f_front = [], [], [], [], [], []
    for _ in range(steps):
        a = np.asarray(pol.action(obs), float)
        A.append(a / scale)
        na = (a / scale)[:env.action_space.shape[0]]
        obs, _r, term, trunc, _i = env.step(na)
        s = extract_state(data, env._p0)
        steer.append(np.degrees(float(data.qpos[env._sj])))
        roll.append(np.degrees(s.roll))
        speed.append(float(np.hypot(s.v_lon, s.v_lat)))
        fr, ft = _floor_normal_forces(model, data, floor, rear, front)
        f_rear.append(fr)
        f_front.append(ft)
        if term or trunc:
            break
    A = np.array(A)
    f_rear, f_front = np.array(f_rear), np.array(f_front)
    s = extract_state(data, env._p0)
    down = f_rear > 1e-6
    hub_rev = (float(data.qpos[hub_adr]) - hub0) / (2 * np.pi)
    ground = {
        "drift_speed": float(np.mean(speed)),
        "net_disp": float(np.hypot(s.e_lon, s.e_lat)),
        "rear_air": float(np.mean(~down)),
        # Averaged over the steps it is actually DOWN: including the airborne
        # zeros would report a number that falls as the hopping gets worse.
        "rear_F": float(np.mean(f_rear[down]) / weight) if down.any() else np.nan,
        "front_air": float(np.mean(f_front <= 1e-6)),
        "hub_rev": hub_rev,
        "rolling_m": hub_rev * 2 * np.pi * env._r_rear,
    }
    return A, np.array(steer), np.array(roll), ground


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

    w = max(len(k) for k in data) + 2

    print(f"hold, {args.seconds:.0f} s at {1/dt:.0f} Hz "
          f"(Nyquist {0.5/dt:.0f} Hz)\n")
    print(f"{'policy':{w}} {'channel':>11} {'mean':>8} {'rms(ac)':>8} "
          f"{'peak f':>8} {'peak amp':>9}")
    peaks = {}
    for key, (A, steer, roll, _g) in data.items():
        for c, name in enumerate(CHANNELS):
            f, amp = spectrum(A[:, c], dt)
            k = int(np.argmax(amp[1:]) + 1)            # skip the DC bin
            peaks[(key, name)] = (f[k], amp[k])
            print(f"{key:{w}} {name:>11} {A[:, c].mean():>+8.3f} "
                  f"{np.std(A[:, c]):>8.3f} {f[k]:>7.2f}Hz {amp[k]:>9.3f}")
        print(f"{'':{w}} {'steer[deg]':>11} {steer.mean():>+8.1f} "
              f"{np.std(steer):>8.1f}")

    # Is hub phase-locked to the steering? If the drift were a rectified
    # oscillation, hub would ride at the steer frequency with a fixed phase.
    print("\nhub vs steer_rate at the steering peak:")
    for key, (A, _s, _r, _g) in data.items():
        fs, _ = peaks[(key, "steer_rate")]
        n = len(A)
        f = np.fft.rfftfreq(n, dt)
        k = int(np.argmin(np.abs(f - fs)))
        S = np.fft.rfft(A[:, 0] - A[:, 0].mean())[k]
        H = np.fft.rfft(A[:, 1] - A[:, 1].mean())[k]
        coh = np.abs(H) / (np.abs(S) + 1e-12)
        print(f"  {key:{w}} f={fs:.2f}Hz  |hub|/|steer| = {coh:.3f}  "
              f"phase = {np.degrees(np.angle(H / S)):+.0f} deg")

    # -- ground truth: is any of that activity holding the bike still, and is
    #    it standing on the floor while it does it?
    print("\nwhat the bike actually did (the zero command asks for none of it)")
    print(f"{'policy':{w}}{'drift m/s':>11}{'net m':>8}{'steer deg':>11}"
          f"{'steer sd':>10}{'roll sd':>9}{'rear air':>10}{'rear F/W':>10}"
          f"{'front air':>11}")
    for key, (_A, steer, roll, g) in data.items():
        print(f"{key:{w}}{g['drift_speed']:>11.3f}{g['net_disp']:>8.3f}"
              f"{steer.mean():>+11.1f}{np.std(steer):>10.1f}"
              f"{np.std(roll):>9.2f}{g['rear_air']:>10.0%}"
              f"{g['rear_F']:>10.2f}{g['front_air']:>11.0%}")

    print("\nrear wheel rotation vs chassis travel: if these disagree, the "
          "wheel is slipping, not rolling")
    print(f"{'policy':{w}}{'hub rev':>10}{'rolling m':>11}{'actual m':>10}")
    for key, (_A, _s, _r, g) in data.items():
        print(f"{key:{w}}{g['hub_rev']:>+10.2f}{g['rolling_m']:>+11.2f}"
              f"{g['net_disp']:>10.2f}")

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
    for j, (key, (A, steer, roll, _g)) in enumerate(data.items()):
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
