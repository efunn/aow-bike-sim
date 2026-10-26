"""The 10-25 held-bike bench tests flown in the floor twin.

The bike on the floor model, the policy's drivetrain overlay, the steer gearbox
hooks; the hand as a ROLL-ONLY spring-damper on the chassis (yaw, pitch and
translation free); the bench's recorded commands in, mapped to ctrl by
inverting BikeBus.write_commands (drive_a = 102, onboard signs, steer sign).

    python analysis/servo/held_twin.py diff 261025-200407_chirp --k 2 10
    python analysis/servo/held_twin.py steer traces/bench/steer_ground_261025-200807.json --k 2 5 10
    python analysis/servo/held_twin.py static traces/bench/steer_static_261025-203535.json --k 5
    python analysis/servo/held_twin.py pluck --k 1.8 5     # tap_test's spots, servos locked

`--k` is the hand's roll stiffness in N m/rad (damping at zeta 0.5 about the
floor line). Below ~1.6 (m g h) the bike falls over and the row is
meaningless. Both sides of every table use the same lock-ins: drivetrain_bench
._lockin for the diff steps, steer_response.fit for the steer tones. The
yaw / free-rolling ratio divides by 3 x 2.4 x 10.5 mm / 0.2005 m. Laptop
only: imports mujoco.
"""
import argparse, json, sys, warnings
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, "analysis/servo")
warnings.filterwarnings("ignore")
import drivetrain_bench as db                       # noqa: E402
import steer_response as sr                         # noqa: E402

ONB = yaml.safe_load(open("config/onboard.yaml"))
ID_A, ID_B = ONB["servo_ids"][:2]                   # 102, 101
SIGN = dict(zip((ID_A, ID_B), ONB["servo_sign"]))  # per servo id
STEER_SIGN = float(ONB["steer_sign"])
BELT = 3.0
FS = 500.0


def build(policy="general_puffer_fixes_72M"):
    import mujoco
    from aow_sim import drivetrain_model, gearbox_friction
    from aow_sim.build_model import build_model, load_params
    from aow_sim.control.flick import MOVES_DIR
    from aow_sim.control.linearize import settle_upright
    params = load_params()
    spec = yaml.safe_load((MOVES_DIR / f"{policy}.yaml").read_text())
    params = {**params, drivetrain_model.KEY: spec[drivetrain_model.KEY]}
    m = build_model(params)
    d = mujoco.MjData(m)
    d.qpos[:] = settle_upright(m).qpos
    mujoco.mj_forward(m, d)
    drive = drivetrain_model.DrivetrainSim.attach(m, params)
    drive.reset(d, np.random.default_rng(0))
    gear = gearbox_friction.attach_native(m, params)
    return mujoco, m, d, drive, gear


def fly(goal_fn, seconds, k, zeta=0.5, settle=1.0, steer_offset=0.0):
    """goal_fn(t) -> {servo id: velocity rad/s, 'steer': servo offset rad}.
    -> dict of FS-rate arrays: t, servo angles 101/102, steer joint, gyro (deg/s)."""
    mujoco, m, d, drive, gear = build()
    dt = m.opt.timestep
    aid = {n: m.actuator(n).id for n in ("drive_a", "drive_b", "steer")}
    ch = m.body("chassis").id
    sj = m.joint("steer_joint").qposadr[0]
    qa, qb = m.joint("input_a_spin").qposadr[0], m.joint("input_b_spin").qposadr[0]
    steer0 = float(d.ctrl[aid["steer"]]) + steer_offset / STEER_SIGN   # servo-frame offset
    d.qpos[sj] = steer0
    mujoco.mj_forward(m, d)
    # roll inertia about the floor line, for the damping
    mujoco.mj_forward(m, d)
    tot = m.body_subtreemass[ch]
    h = d.subtree_com[ch][2]
    I = tot * h * h + m.body_inertia[ch][0]
    c = 2 * zeta * np.sqrt(k * I) if k > 0 else 0.0
    every = int(round(1 / (FS * dt)))
    rows = []
    n_settle = int(settle / dt)
    for step in range(n_settle + int(seconds / dt)):
        t = (step - n_settle) * dt
        g = goal_fn(t) if t >= 0 else {}
        d.ctrl[aid["drive_a"]] = g.get(ID_A, 0.0) * BELT * SIGN[ID_A]
        d.ctrl[aid["drive_b"]] = g.get(ID_B, 0.0) * BELT * SIGN[ID_B]
        d.ctrl[aid["steer"]] = steer0 + g.get("steer", 0.0) / STEER_SIGN
        if k > 0:
            q = d.qpos[3:7]
            roll = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            xw = d.xmat[ch].reshape(3, 3)[:, 0]
            d.xfrc_applied[ch, 3:6] = -(k * roll + c * d.qvel[3]) * xw
        drive.pre_step(d)
        for gg in gear:
            gg.pre_step(d)
        mujoco.mj_step(m, d)
        if t >= 0 and step % every == 0:
            rows.append((t, d.qpos[qa] / BELT * SIGN[ID_A], d.qpos[qb] / BELT * SIGN[ID_B],
                         d.qpos[sj], *np.degrees(d.qvel[3:6]), d.qpos[2], *d.qpos[3:7]))
    a = np.array(rows)
    return {"t": a[:, 0], ID_A: a[:, 1], ID_B: a[:, 2], "steer": a[:, 3],
            "gyro": a[:, 4:7], "z": a[:, 7], "quat": a[:, 8:12], "I": I, "c": c}


K_ROLL = 3 * 2.4 * 0.0105 / 0.2005 * 180 / np.pi


def diff(name, ks):
    from aow_sim.hw.bench_log import Capture
    cap = Capture.load(f"traces/drivetrain/{name}")
    th = cap.arrays["t_host"]
    cmd = {i: cap.command(i) for i in cap.ids}
    sa, sb = cap.meta["signs"]
    A, B = cap.ids                                  # 101, 102 in bench order
    q_b = (sa * cap.position_rad(A) - sb * cap.position_rad(B)) / 2
    z = np.load(f"traces/drivetrain/{name}/ahrs.npz")

    def goal(t):
        j = min(np.searchsorted(th, t, side="right") - 1, len(th) - 1)
        j = max(j, 0)
        return {i: (0.0 if np.isnan(cmd[i][j]) else float(cmd[i][j])) for i in cap.ids}
    steps = [(k, s) for k, s in enumerate(cap.segments) if s["info"].get("kind") == "sine"]

    def table(t_s, q, ta, gyro):
        out = []
        for k, s in steps:
            sl = cap.segment_slice(k); f = s["info"]["freq"]; t0 = th[sl][0]
            m = (t_s >= t0 + max(0.4, 3 / f)) & (t_s <= th[sl][-1])
            dc = (sa * cmd[A] - sb * cmd[B]) / 2
            keepb = (th[sl] - t0) > max(0.4, 3 / f)
            C = db._lockin(th[sl][keepb] - t0, dc[sl][keepb], f)
            Q = db._lockin(t_s[m] - t0, q[m], f); V = 2j * np.pi * f * Q
            mg = (ta >= t0 + max(0.4, 3 / f)) & (ta <= th[sl][-1])
            Y = db._lockin(ta[mg] - t0, gyro[mg, 2], f)
            R = db._lockin(ta[mg] - t0, gyro[mg, 0], f)
            P = db._lockin(ta[mg] - t0, gyro[mg, 1], f)
            out.append((f, abs(V / C), np.degrees(np.angle(V / C)), abs(Y / (K_ROLL * V)),
                        abs(R / C), abs(P / C)))
        return np.array(out)
    bike = table(th, q_b, z["t"], np.degrees(z["gyro"]))
    sims = {}
    for k in ks:
        r = fly(goal, th[-1] + 0.01, k)
        qs = (sa * r[A] - sb * r[B]) / 2
        sims[k] = table(r["t"], qs, r["t"], r["gyro"])
        print(f"  [k {k:g} N m/rad: roll I {r['I']:.4f}, c {r['c']:.3f}; z drift {1e3*(r['z'].max()-r['z'].min()):.1f} mm]")
    print(f"\n{name} ({cap.meta['note']})")
    print("  Hz   | servo diff gain, phase | yaw / free rolling | roll per diff cmd | pitch per diff cmd")
    print("       | bike   " + "  ".join(f"k{k:g}" for k in ks))
    for i in range(len(bike)):
        def cells(col, fmt):
            return " ".join(fmt(x[i, col]) for x in [bike] + [sims[k] for k in ks])
        print(f"  {bike[i,0]:5.2f} | "
              + " ".join(f"{x[i,1]:.2f}{x[i,2]:+4.0f}" for x in [bike] + [sims[k] for k in ks])
              + " | " + cells(3, lambda v: f"{v:4.2f}") + " | " + cells(4, lambda v: f"{v:5.2f}")
              + " | " + cells(5, lambda v: f"{v:4.2f}"))


def steer(path, ks, offset_deg=None):
    meta = json.loads(Path(path).read_text())
    za = np.load(str(path).replace(".json", "_ahrs.npz"))
    tones = meta["tones"]
    fade, secs = 0.5, 3.0
    T = tones[-1]["t_tone"] + secs + 0.3

    def goal(t):
        for r in tones:
            if r["t_tone"] <= t < r["t_tone"] + secs:
                tt = t - r["t_tone"]
                return {"steer": np.radians(r["amp_deg"]) * sr.envelope(tt, secs, fade)
                        * np.sin(2 * np.pi * r["f"] * tt)}
        return {}

    def table(get_t, get_pos, get_goal, ta, gyro):
        out = []
        for r in tones:
            t, x, g = get_t(r), get_pos(r), get_goal(r)
            s = (t > fade) & (t < secs - fade)
            G = sr.fit(t[s], g[s], r["f"]); H = sr.fit(t[s], x[s], r["f"]) / G
            m = (ta >= r["t_tone"] + fade) & (ta <= r["t_tone"] + secs - fade)
            Gd = G * 180 / np.pi
            row = [r["amp_deg"], r["f"], abs(H), np.degrees(np.angle(H))]
            for ax in (0, 2, 1):
                row.append(abs(sr.fit(ta[m] - r["t_tone"], gyro[m, ax], r["f"]) / Gd))
            out.append(row)
        return np.array(out)
    bike = table(lambda r: np.asarray(r["t"]), lambda r: np.asarray(r["pos"]),
                 lambda r: np.asarray(r["goal"]), za["t"], np.degrees(za["gyro"]))
    if offset_deg is None:          # where the bike's fork sat: its first reading
        offset_deg = float(np.degrees(tones[0]["pos"][0]))
        offset_deg = (offset_deg + 90.0) % 180.0 - 90.0     # straight repeats every 180
    print(f"  steer offset from straight {offset_deg:+.2f} deg (servo frame), twin starts there")
    sims = {}
    for k in ks:
        sim = fly(goal, T, k, steer_offset=np.radians(offset_deg))
        ts = sim["t"]
        sj0 = sim["steer"][0]

        def seg(r, arr):
            m = (ts >= r["t_tone"]) & (ts < r["t_tone"] + secs)
            return ts[m] - r["t_tone"], arr[m]
        pos = lambda r: STEER_SIGN * (seg(r, sim["steer"])[1] - sj0)
        gl = lambda r: np.array([goal(r["t_tone"] + x).get("steer", 0.0) for x in seg(r, ts)[0]])
        sims[k] = table(lambda r: seg(r, ts)[0], pos, gl, ts, sim["gyro"])
        print(f"  [k {k:g}: z drift {1e3*(sim['z'].max()-sim['z'].min()):.1f} mm]")
    print(f"\n{path} ({meta['note']})")
    print("  amp  Hz | shaft gain, phase | roll per goal deg | yaw per goal deg | pitch per goal deg")
    print("          | bike   " + "  ".join(f"k{k:g}" for k in ks))
    for i in range(len(bike)):
        L = [bike] + [sims[k] for k in ks]
        print(f"  {bike[i,0]:3g} {bike[i,1]:4.1f} | " + " ".join(f"{x[i,2]:.2f}{x[i,3]:+4.0f}" for x in L)
              + " | " + " ".join(f"{x[i,4]:4.2f}" for x in L) + " | " + " ".join(f"{x[i,5]:4.2f}" for x in L)
              + " | " + " ".join(f"{x[i,6]:4.2f}" for x in L))


def _rpy(q):
    w, x, y, z = np.asarray(q, float).T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    up = -np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))    # nose up +
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    return roll, up, yaw


def static(path, ks):
    """Slow steer sweep (steer_response --amp-deg 30 40 --freqs 0.1 0.2): nose-up
    pitch = a + b steer + c steer^2 + d roll + e roll^2 (deg), bike and twin."""
    meta = json.loads(Path(path).read_text())
    za = np.load(str(path).replace(".json", "_ahrs.npz"))
    tones = meta["tones"]
    secs, fade = 25.0, 4.0                      # as the sweep was run
    ar = json.dumps(meta.get("argv", []))
    if "--seconds" in ar:
        av = meta["argv"]; secs = float(av[av.index("--seconds") + 1]); fade = float(av[av.index("--fade") + 1])

    def goal(t):
        for r in tones:
            if r["t_tone"] <= t < r["t_tone"] + secs:
                tt = t - r["t_tone"]
                return {"steer": np.radians(r["amp_deg"]) * sr.envelope(tt, secs, fade)
                        * np.sin(2 * np.pi * r["f"] * tt)}
        return {}

    def fit(st, rl, up):
        X = np.column_stack([np.ones_like(st), st, st ** 2, rl, rl ** 2])
        c, *_ = np.linalg.lstsq(X, up, rcond=None)
        return c, (up - X @ c).std()
    br, bu, by = _rpy(za["quat"])
    print("nose-up pitch [deg] = a + b steer + c steer^2 + d roll + e roll^2; 'even' = c x 35^2")
    for k in ks:
        sim = fly(goal, tones[-1]["t_tone"] + secs + 0.3, k)
        sr_, su, sy = _rpy(sim["quat"])
        sst = STEER_SIGN * np.degrees(sim["steer"] - sim["steer"][0])
        print(f"  [twin hand k {k:g}]")
        for r in tones:
            t = np.asarray(r["t"]); x = (np.degrees(np.asarray(r["pos"])) + 90) % 180 - 90
            s = (t > fade) & (t < secs - fade)
            tt = t[s] + r["t_tone"]
            cb, eb = fit(x[s], np.interp(tt, za["t"], br), np.interp(tt, za["t"], bu))
            ms = (sim["t"] >= r["t_tone"] + fade) & (sim["t"] < r["t_tone"] + secs - fade)
            ct, et = fit(sst[ms], sr_[ms], su[ms])
            for lab, c, e, rl, yw in (("bike", cb, eb, np.interp(tt, za["t"], br), np.interp(tt, za["t"], by)),
                                      ("twin", ct, et, sr_[ms], sy[ms])):
                print(f"  +-{r['amp_deg']:g} @ {r['f']:g} Hz {lab}: b {c[1] * 1e3:+6.2f}e-3  "
                      f"even {c[2] * 1225:+.3f}  d(roll) {c[3]:+.4f}  res {e:.3f}  "
                      f"| roll p-p {np.ptp(rl):.1f}, yaw p-p {np.ptp(yw):.1f}")


# tap_test's spots: (body, point offset from its frame [m], force direction).
# Push for 0.3 s ramped, 0.2 s held, then let go at once, three times.
PLUCKS = {
    "steerer": ("steer", (0, 0, 0.06), (0, 1, 0)),
    "front_axle": ("front_wheel", (0, 0, 0.05), (0, 1, 0)),
    "rear_axle": ("aow_hub", (0, 0, 0.05), (0, 1, 0)),
    "cage_side": ("chassis", (0, 0, 0.15), (0, 1, 0)),
    "cage_fore_aft": ("chassis", (0, 0, 0.15), (1, 0, 0)),
    "wing_left": ("swing_wing_left", (0, 0, 0), (0, 0, 1)),
}
PLUCK_N = 1.0


def pluck(ks):
    """The pluck test in the twin: servos locked (drives at 0, steer at its
    start), the hand as in `fly`, a push-and-release at each spot. Prints the
    ring-down spectrum's peaks per axis, as tap_test's ring-down read-out."""
    mujoco = __import__("mujoco")
    for k in ks:
        print(f"\ntwin, hand k {k:g}, pluck {PLUCK_N:g} N; peaks 2-30 Hz (power, of the largest)")
        for spot, (body, off, dirn) in PLUCKS.items():
            _, m, d, drive, gear = build()
            b = m.body(body).id
            ch = m.body("chassis").id
            aid = m.actuator("steer").id
            steer0 = float(d.ctrl[aid])
            mujoco.mj_forward(m, d)
            tot, h = m.body_subtreemass[ch], d.subtree_com[ch][2]
            c = 2 * 0.5 * np.sqrt(k * (tot * h * h + m.body_inertia[ch][0]))
            dt = m.opt.timestep
            T = 1.0 + 3 * 2.0
            gy, ts = [], []
            for step in range(int(T / dt)):
                t = step * dt
                d.ctrl[:] = 0.0
                d.ctrl[aid] = steer0
                q = d.qpos[3:7]
                roll = np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
                d.xfrc_applied[:] = 0.0
                d.xfrc_applied[ch, 3:6] = -(k * roll + c * d.qvel[3]) * d.xmat[ch].reshape(3, 3)[:, 0]
                tp = (t - 1.0) % 2.0 if t >= 1.0 else 9.0
                F = PLUCK_N * (tp / 0.3 if tp < 0.3 else 1.0 if tp < 0.5 else 0.0)
                if F:
                    R = d.xmat[b].reshape(3, 3)
                    pt = d.xpos[b] + R @ np.array(off, float)
                    f = R @ (F * np.array(dirn, float))
                    d.xfrc_applied[b, :3] += f
                    d.xfrc_applied[b, 3:] += np.cross(pt - d.xipos[b], f)
                drive.pre_step(d)
                for g in gear:
                    g.pre_step(d)
                mujoco.mj_step(m, d)
                if step % int(round(1 / (200 * dt))) == 0:
                    ts.append(t); gy.append(np.degrees(d.qvel[3:6]))
            ts, gy = np.array(ts), np.array(gy)
            n = 2048
            fr = np.fft.rfftfreq(n, 1 / 200)
            acc = np.zeros((3, len(fr)))
            for r in range(3):
                t_rel = 1.0 + 2.0 * r + 0.5 + 0.03
                w = (ts >= t_rel) & (ts < t_rel + 1.45)
                for ax in range(3):
                    x = gy[w, ax] - gy[w, ax].mean()
                    acc[ax] += np.abs(np.fft.rfft(x * np.hanning(len(x)), n)) ** 2

            def peaks(S, lo=2.0, hi=30):
                sel = np.flatnonzero((fr >= lo) & (fr <= hi))
                pk = [i for i in sel[1:-1] if S[i] > S[i - 1] and S[i] > S[i + 1]]
                pk = sorted(pk, key=lambda i: -S[i])[:4]
                return "  ".join(f"{fr[i]:4.1f}({S[i] / S[sel].max():.2f})" for i in sorted(pk))
            amp = np.abs(gy[ts > 1.0]).max(axis=0)
            print(f"  {spot:14s} roll: {peaks(acc[0])}   | yaw: {peaks(acc[2])}   "
                  f"| max deg/s r/p/y {amp[0]:.1f}/{amp[1]:.1f}/{amp[2]:.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("diff", "steer", "static", "pluck"))
    ap.add_argument("src", nargs="?")
    ap.add_argument("--k", type=float, nargs="+", default=[2.0])
    ap.add_argument("--steer-offset", type=float, default=None, metavar="DEG",
                    help="steer: fork offset from straight the twin starts at "
                         "(default: the bike's own first reading)")
    a = ap.parse_args()
    if a.what == "diff":
        diff(a.src, a.k)
    elif a.what == "static":
        static(a.src, a.k)
    elif a.what == "pluck":
        pluck(a.k)
    else:
        steer(a.src, a.k, a.steer_offset)
