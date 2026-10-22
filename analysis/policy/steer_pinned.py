"""Does the policy saw its steering when the bike CANNOT respond? The bench
run, in C MuJoCo.

On the bench (2026-10-17, `traces/bike/261017-174510_run_bench_policy`) the
servos were bare or clamped, the AHRS sat level and still (roll -0.12 deg,
sd 0.01), and the wheels spun in air. With roll pinned, the only loop that
closes is policy -> steer target -> steer servo -> steer angle/rate -> policy.
The steer built up from 1 to 8 deg rms over ~15 s and then held at 5.2 Hz,
the commanded target carrying more than the shaft. This asks whether the
policy does that against the SIM's steer servo too:

  * yes -> it is the policy's own behaviour in that loop, bike or no bike;
  * no  -> the real XC330 (P 900) answers differently from the modelled one,
           and that gap is what to measure.

The chassis freejoint is reset to a fixed pose, lifted clear of the floor,
before every physics substep (a pre_step hook in the env's own loop), so the
wheels and steer run free exactly as on the bench. The AHRS model is OFF
(`ahrs_level none`): a pinned body re-released every substep is in free fall
for that substep, so a simulated accelerometer would read ~0 g and the filter
would wander. Truth roll and rate are 0 here, which is what the level, still
TM151 reported. Command held at zero, as `--no-link` does.

    python analysis/policy/steer_pinned.py
    python analysis/policy/steer_pinned.py --steer0 -50 0 50 --seconds 60
    python analysis/policy/steer_pinned.py --sine --bare                # sim steer, as benched
    python analysis/policy/steer_pinned.py --sine --bare --armature 1e-3
    python analysis/policy/steer_pinned.py --steer0 50 --bare --armature 5e-4 --steer-kv 0.055
    python analysis/policy/steer_pinned.py --steer0 50 --pi-steer-rate

Bench comparisons (each the bench's own protocol, the steer stripped to the
servo + column as benched):

    python analysis/policy/steer_pinned.py --sine --bare --steer-kp 2.3 --armature 5.5e-4

and from Python, `ramp_response(...)` for the constant-speed ramps and
`two_regime=(0.2, 0.45)` on any of them for the PROTOTYPE slow regime
(`TwoRegime`; not in the plant -- see actuators.steer_kp in bike_params).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "servo"))  # sibling category

from aow_sim.build_model import load_params  # noqa: E402
from per_command import _cfg_for  # noqa: E402
from rsa_policies import env_for, load_general  # noqa: E402


class PiSteerRate:
    """The steer rate as the Pi forms it (hw/bike_bus.py, velocity_source
    "differenced"): position quantised to XC330 counts, backward-differenced
    once per 10 ms tick, through RateFilter(25 ms, 0.5) -- ~8 ms of lag the
    env's exact joint velocity does not have. Installed as a pre_step hook
    (sampling the state the previous substep left) plus an obs override."""

    IDX = 5                                      # general_spec.OBS_NAMES "steer_rate"

    def __init__(self, env, tick_s=0.01):
        from aow_sim.control.steer import XC330_COUNTS_PER_RAD
        from aow_sim.hw.rate_filter import RateFilter
        self.cpr, self.sj = XC330_COUNTS_PER_RAD, env._sj
        self.every = max(1, int(round(tick_s / env.model.opt.timestep)))
        self.tick_s = self.every * env.model.opt.timestep
        self.filt = RateFilter(25.0, 0.5, tick_s * 1000.0)
        self.k, self.prev, self.v = 0, None, 0.0
        orig = env._obs

        def obs():
            out = orig()
            out[0][self.IDX] = self.v
            return out
        env._obs = obs

    def reset(self, data):
        from aow_sim.hw.rate_filter import RateFilter
        self.filt = RateFilter(25.0, 0.5, self.tick_s * 1000.0)
        self.k, self.prev, self.v = 0, None, 0.0

    def pre_step(self, data):
        if self.k % self.every == 0:
            c = round(float(data.qpos[self.sj]) * self.cpr)
            if self.prev is not None:
                self.v = self.filt.update((c - self.prev) / self.cpr / self.tick_s)
            self.prev = c
        self.k += 1


class TwoRegime:
    """PROTOTYPE of the steer XC330's two regimes (bench ramps, 2026-10-17,
    traces/bench/steer_ramps_261017-192117.json): above ~0.45 rad/s the lag is
    the model's droop + running friction exactly; below ~0.2 it is transport
    alone -- no droop, no friction offset. Here both fade out below the band:
    the joint's damping (the droop, kv) and its friction limit (set by the
    gearbox hook just before this one) are scaled by a smoothstep in |w|.
    Must come AFTER the steer's GearboxFriction in env._gearbox."""

    def __init__(self, model, w0=0.2, w1=0.45):
        self.m, self.w0, self.w1 = model, float(w0), float(w1)
        self.dof = int(model.joint("steer_joint").dofadr[0])
        self.kv = float(model.dof_damping[self.dof])

    def g(self, w):
        u = min(max((abs(w) - self.w0) / (self.w1 - self.w0), 0.0), 1.0)
        return u * u * (3.0 - 2.0 * u)

    def reset(self, data):
        self.m.dof_damping[self.dof] = self.kv

    def pre_step(self, data):
        k = self.g(float(data.qvel[self.dof]))
        self.m.dof_damping[self.dof] = self.kv * k
        fl = self.m.dof_frictionloss
        fl[self.dof] = max(fl[self.dof] * k, 1e-6)       # keep the row


class Pin:
    """Hold the chassis freejoint at one pose, zero velocity, every substep."""

    def __init__(self, qpos7):
        self.q = np.array(qpos7, float)

    def reset(self, data):
        pass

    def pre_step(self, data):
        data.qpos[:7] = self.q
        data.qvel[:6] = 0.0


def wobble(x_deg, dt, win_s=0.25):
    """High-passed rms (minus a centred moving average) and its peak
    frequency -- the same measure applied to the bench records."""
    k = max(3, int(round(win_s / dt)))
    hp = x_deg - np.convolve(x_deg, np.ones(k) / k, mode="same")
    hp = hp[k:-k]
    f = np.fft.rfftfreq(len(hp), dt)
    p = np.abs(np.fft.rfft(hp - hp.mean()))
    return float(hp.std()), float(f[p[1:].argmax() + 1])


def make_env(name, seconds, steer=None, armature=None, bare=False,
             pi_rate=False, two_regime=None):
    """The eval env for `name`, AHRS off, with optional in-memory steer edits:
    `steer` overrides actuators.steer_*; `armature` sets the steer joint's
    rotor inertia [kg m2] (the model carries NONE); `bare` strips the fork and
    front wheel to ~nothing, as on the 10-17 bench (steering column and
    headset installed, hanging, no fork or wheel). The model's `steer` body is
    steerer AND fork in one, so the column goes too -- immaterial: fork +
    wheel are 4.1e-5 kg m2 about the axis, the fitted rotor 5e-4."""
    params = load_params()
    if steer:                       # in-memory only; bike_params.yaml untouched
        params["actuators"] = {**params["actuators"], **steer}
    pol = load_general(name)
    pol.ahrs_level = "none"
    cfg = _cfg_for()
    cfg = {**cfg, "env": {**cfg["env"], "max_episode_s": float(seconds) + 1.0}}
    env = env_for(pol, params, cfg)
    m = env.model
    j = m.joint("steer_joint")
    if armature is not None:
        m.dof_armature[j.dofadr[0]] = armature
    if bare:
        for b in range(m.nbody):
            if b == j.bodyid[0] or m.body_parentid[b] == j.bodyid[0]:
                m.body_mass[b] *= 1e-3
                m.body_inertia[b] *= 1e-3
        import mujoco
        mujoco.mj_setConst(m, env.data)
    if two_regime:
        env._gearbox.append(TwoRegime(m, *two_regime))
    if pi_rate:
        env._gearbox.append(PiSteerRate(env))
    return env, pol


def pin(env, lift_m, steer0_deg):
    import mujoco
    d = env.data
    q = env._eq[:7].copy()
    q[2] += lift_m
    d.qpos[:7] = q
    d.qpos[env._sj] = np.radians(steer0_deg)
    d.qvel[:] = 0.0
    env._steer = float(d.qpos[env._sj])
    mujoco.mj_forward(env.model, d)
    p = Pin(q)
    env._gearbox.append(p)
    return p


def run(name, steer0_deg, seconds, lift_m, seed, **edits):
    env, pol = make_env(name, seconds, **edits)
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    n_act = env.action_space.shape[0]
    env.reset(seed=seed, options={"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0,
                                  "difficulty": 1.0})
    d = env.data
    pin(env, lift_m, steer0_deg)
    obs = env._obs()[0]
    pol.reset()

    n = int(round(seconds / env.ctrl_dt))
    steer, target, hub = np.empty(n), np.empty(n), np.empty(n)
    for k in range(n):
        a = (np.asarray(pol.action(obs), float) / scale)[:n_act]
        obs, _r, term, trunc, _ = env.step(a)
        steer[k] = d.qpos[env._sj]
        target[k] = env._steer
        hub[k] = a[1]
        if term or trunc:
            steer, target, hub = steer[:k + 1], target[:k + 1], hub[:k + 1]
            print(f"  episode ended at {(k + 1) * env.ctrl_dt:.1f} s "
                  f"({'terminated' if term else 'truncated'})")
            break
    return env.ctrl_dt, np.degrees(steer), np.degrees(target), hub


def sine_response(name, lift_m, freqs, amp_deg=5.0, **edits):
    """The sim steer under analysis/servo/steer_response.py's exact protocol: 200 Hz
    goal frames, position read BEFORE each frame's goal, raised-cosine fades,
    fit over the middle 2 s of 3. -> [(f, gain, phase_deg)]."""
    import mujoco
    from steer_response import envelope, fit
    env, _pol = make_env(name, 5.0, **edits)
    m, d = env.model, env.data
    aid = env._aid["steer"]
    per = int(round(0.005 / m.opt.timestep))
    A, T, out = np.radians(amp_deg), 3.0, []
    for f in freqs:
        env.reset(seed=0)
        p = pin(env, lift_m, 0.0)
        p0 = float(d.qpos[env._sj])
        ts, gs, xs = [], [], []
        for j in range(int(T / 0.005)):
            t = j * 0.005
            u = A * envelope(t, T, 0.5) * np.sin(2 * np.pi * f * t)
            ts.append(t); xs.append(d.qpos[env._sj] - p0); gs.append(u)
            d.ctrl[aid] = p0 + u
            for _ in range(per):
                for g in env._gearbox:      # includes the pin
                    g.pre_step(d)
                mujoco.mj_step(m, d)
        t = np.array(ts)
        s = (t > 0.5) & (t < T - 0.5)
        H = fit(t[s], np.array(xs)[s], f) / fit(t[s], np.array(gs)[s], f)
        out.append((f, abs(H), float(np.degrees(np.angle(H)))))
        env._gearbox.remove(p)
    return out


def ramp_response(name, lift_m, speeds, travel_deg=40.0, accel=20.0, **edits):
    """analysis/servo/steer_response.py --ramps in the sim: median steady lag
    (goal - position, the position read before each 200 Hz goal) per speed."""
    import mujoco
    from steer_response import ramp_profile
    env, _pol = make_env(name, 5.0, **edits)
    m, d = env.model, env.data
    aid = env._aid["steer"]
    per = int(round(0.005 / m.opt.timestep))
    tr, out = np.radians(travel_deg), []
    for w in speeds:
        env.reset(seed=0)
        p = pin(env, lift_m, 0.0)
        p0 = float(d.qpos[env._sj])
        T = ramp_profile(0.0, w, tr, accel)[2]
        lag = {1: [], -1: []}
        for j in range(int((T + 0.3) / 0.005)):
            t = j * 0.005
            x = float(d.qpos[env._sj]) - p0
            g, v, _ = ramp_profile(t, w, tr, accel)
            if abs(abs(v) - w) < 1e-9:
                lag[1 if v > 0 else -1].append(g - x)
            d.ctrl[aid] = p0 + g
            for _ in range(per):
                for h in env._gearbox:
                    h.pre_step(d)
                mujoco.mj_step(m, d)
        mid = [np.median(np.array(lag[s])[len(lag[s]) // 4:]) for s in (1, -1)]
        out.append((w, 0.5 * (mid[0] - mid[1])))
        env._gearbox.remove(p)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--policy", default="general_puffer_fixes_72M")
    ap.add_argument("--steer0", type=float, nargs="+", default=[-50.0, 0.0, 50.0],
                    help="initial steer angle(s), deg")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--lift", type=float, default=0.10, help="m above the rest pose")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--steer-kp", type=float, help="override actuators.steer_kp")
    ap.add_argument("--steer-kv", type=float, help="override actuators.steer_kv")
    ap.add_argument("--steer-delay", type=float,
                    help="override actuators.steer_command_delay_s [s]")
    ap.add_argument("--armature", type=float,
                    help="steer joint rotor inertia [kg m2]; the model has none")
    ap.add_argument("--bare", action="store_true",
                    help="strip fork + front wheel, as on the 2026-10-17 bench "
                         "(column installed, no fork/wheel)")
    ap.add_argument("--sine", action="store_true",
                    help="instead of flying the policy, measure the sim steer's "
                         "goal->shaft response by steer_response.py's protocol")
    ap.add_argument("--pi-steer-rate", action="store_true",
                    help="observe the steer rate as the Pi forms it (counts, "
                         "10 ms difference, RateFilter), not the exact joint rate")
    a = ap.parse_args()
    steer = {k: v for k, v in (("steer_kp", a.steer_kp), ("steer_kv", a.steer_kv),
                               ("steer_command_delay_s", a.steer_delay)) if v is not None}

    edits = dict(steer=steer, armature=a.armature, bare=a.bare, pi_rate=a.pi_steer_rate)
    tag = ", ".join(f"{k} {v}" for k, v in (("steer", steer), ("armature", a.armature),
                                            ("bare", a.bare), ("pi steer rate", a.pi_steer_rate)) if v)
    if a.sine:
        print(f"sim steer response{', ' + tag if tag else ''}")
        for f, g, ph in sine_response(a.policy, a.lift, [1, 2, 3, 4, 5, 6, 8, 10], **edits):
            print(f"  {f:5.1f} Hz  gain {g:.2f}  phase {ph:5.0f} deg")
        return
    print(f"{a.policy}: chassis pinned {a.lift:.2f} m up, AHRS off, zero command, "
          f"{a.seconds:g} s" + (f", {tag}" if tag else ""))
    for s0 in a.steer0:
        dt, st, tg, hub = run(a.policy, s0, a.seconds, a.lift, a.seed, **edits)
        wrap = (st + 90) % 180 - 90
        print(f"steer0 {s0:+5.0f} deg")
        for t0 in range(0, int(len(st) * dt), 5):
            s = slice(int(t0 / dt), int(min(t0 + 5, len(st) * dt) / dt))
            if s.stop - s.start < 20:
                continue
            w, f = wobble(st[s], dt)
            wt, _ = wobble(tg[s], dt)
            print(f"  {t0:3d}-{t0 + 5:3d} s  steer(mod180) {wrap[s].mean():+6.1f}  "
                  f"wobble {w:5.2f} deg @ {f:4.1f} Hz  target {wt:5.2f}  "
                  f"|hub| {np.abs(hub[s]).mean():.2f}")


if __name__ == "__main__":
    main()
