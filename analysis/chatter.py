"""How hard each policy is sawing its actuators, PER CHANNEL, over the whole
eval grid -- and what that costs in behaviour.

WHY THIS IS SEPARATE FROM hold_spectrum.py. That script asks what the chatter
looks like (waveform, frequency, phase) under one command. This one asks how
much there is of it, across every command in the eval grid, in the exact
quantity the reward prices:

    w_smooth * |a_t - a_{t-1}|^2

so a row here can be multiplied by a candidate w_smooth and read directly as
"reward per step". The per-channel split is the point. Under a UNIFORM
w_smooth of 0.05 (moves/general_rl_smooth_og) steer and hub gave up ~70% of
their chatter and `diff` gave up 3% of its own, which is invisible in any
total and is the whole reason config/rl_general_smooth_diff.yaml exists.

WHY THE EVAL IS RE-RUN HERE rather than read from moves/*.yaml. Those metrics
blocks were written by training runs with different grids (n_eval 12 vs 20)
and different code, so they are not comparable across policies. Everything
below comes from one grid, identical seeds, randomization off.

SATURATION is reported alongside: |a| > 0.98 means the channel is pinned to a
bound. A policy can have low per-step change while sitting saturated (smooth
but maxed out), and high per-step change while never reaching a bound, so
neither number substitutes for the other.

SIGN FLIPS AND THE ACHIEVED JOINT RATE were added 2026-09-01, because
saturation alone reads the same for a channel held at +1 all episode and a
channel alternating +1/-1 every other step. Every general policy does the
LATTER on steer -- pinned to the bound 23-96% of the time while flipping sign
15-32 times a second -- and the joint meanwhile achieves only 1-20% of the
XC330's no-load speed. That is not a servo at its limit, it is a command
dithering near the control loop's Nyquist frequency, and telling the two apart
is what the reversal envelope in docs/measurements/servo-protocol.md section 2
is measured against.

MIXED WIDTHS. A policy trained with `act_wings` emits FOUR channels, not three,
and the set here contains both kinds. Each policy is normalized by its own
`ActionBounds.to_list()[:act_dim]`; the wing column prints "-" for the
three-channel policies. The consequence to keep in mind when reading down a
column: the per-channel cells are comparable across policies, the TOTALS are
not, because a four-channel policy sums one more term.

  python analysis/chatter.py
  python analysis/chatter.py --w-smooth 0.05   # price the table at a weight
  python analysis/chatter.py --policies general_rl_odo general_rl_nolat

`--policies` takes any move names, so a fresh export can be put next to the
standing set without editing rsa_policies.POLICIES. Each env is built by
`policy_env_overrides`, so a policy trained on the ONBOARD ESTIMATE
(`obs_odometry`) is evaluated on the estimate here too -- not on truth. The
same now holds for the ATTITUDE: a move that declares `ahrs_level` is run
against that error model with no flag, which it was not before 2026-09-11.
`--ahrs` still overrides, and overrides for every policy in the set, which is
how a truth-trained policy is asked the deployment question.

Read-only apart from stdout: loads moves/*.npz and writes nothing.
"""

from __future__ import annotations

import argparse
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz
from aow_sim.sim_ahrs import TAU_ORIENT_S
from aow_sim.sim_odometry import ENCODERS
from aow_sim.train_general_rl import _eval_episodes, _score, eval_cmds
from rsa_policies import POLICIES, REPO, env_for, load_general
from drivetrain_eval import VARIANTS, params_for

# The three channels every general policy has, plus the optional fourth. A
# policy's own width decides how many of these it fills -- see `act_scale`.
# Columns a policy does not have print as "-" rather than being dropped, so
# the table stays one shape across a mixed set.
CHANNELS = ("steer", "hub", "diff", "wing")

# Command families for the per-family breakdown. A policy could in principle
# be smooth at rest and violent while moving (or the reverse), which a single
# grid-wide average hides.
FAMILIES = {
    "hold": lambda c: c[0] == 0 and c[1] == 0 and c[2] == 0,
    "fwd": lambda c: c[0] > 0.1,
    "rev": lambda c: c[0] < -0.1,
    "crab": lambda c: abs(c[1]) > 0.1,
    "turn": lambda c: abs(c[2]) > 1,
}


def act_scale(pol) -> np.ndarray:
    """The bound of each channel THIS policy emits, in its own output order.

    Not a fixed 3-vector: a policy trained with `act_wings` emits four values
    and dividing it by three bounds raises a broadcast error that reads like a
    corrupt export. `ActionBounds.to_list()` is always four long and the
    policy's output width says how many of them are real.

    A zero bound would divide by zero, which is a silently poisoned table
    rather than a crash, so it is rejected: a policy that emits a channel it
    has no bound for is a broken export and should say so.
    """
    scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
    if not np.all(scale > 0):
        raise ValueError(
            f"policy emits {pol.act_dim} channels but its bounds are {scale} — "
            "a zero bound cannot normalize an action")
    return scale


# THE REAR WHEEL, AT PHYSICS RATE. Every table above is the ACTION, sampled at
# the 50 Hz control rate, and says nothing about whether the wheel follows it:
# the ideal drive and a P 100 servo smooth a dithering command, a stiffer loop
# may pass it through or ring on its own. The control-rate Nyquist is 25 Hz,
# and the bench put the XC430's resonance near 20 Hz, so the wheel is sampled
# on EVERY mj_step instead, through a stand-in for the `mujoco` module that
# general_env steps with.
WHEEL_SIGNALS = ("tread", "roller", "servo_a", "servo_b")
_SMOOTH_S = 0.1          # jitter: speed minus its centred 100 ms sliding-window mean
_REV_HYST = 0.002        # tread direction reversals counted past +-2 mm/s


class _StepTap:
    """`mujoco` as general_env sees it, with mj_step also sampling the wheel."""

    def __init__(self, module, sample):
        self._module, self._sample = module, sample

    def __getattr__(self, name):
        return getattr(self._module, name)

    def mj_step(self, model, data, *a):
        self._module.mj_step(model, data, *a)
        self._sample(data)


# Tread speed RMS per frequency band [Hz]. From the RAW speed's spectrum, not
# the jitter residual's: subtracting a 100 ms sliding-window mean leaves a
# residual whose gain peaks near 14 Hz whatever the input, so a residual
# spectrum puts its peak there for every policy.
WHEEL_BANDS = ((2, 8), (8, 16), (16, 32), (32, 64), (64, 1250))


def wheel_jitter(W, T, h):
    """Jitter per WHEEL_SIGNALS column, tread reversals per second, and the
    tread speed's RMS per WHEEL_BANDS band. Split into episodes where
    `data.time` rewinds, so nothing is smoothed or counted across a reset."""
    k = max(1, int(round(_SMOOTH_S / h)))
    win = np.ones(k) / k
    n_fft = 2 ** int(np.floor(np.log2(1.0 / h)))          # ~1 s segments
    taper = np.hanning(n_fft)
    t = np.arange(n_fft)
    sq, n, rev, dur = np.zeros(W.shape[1]), 0, 0, 0.0
    band_ms, n_win = np.zeros(len(WHEEL_BANDS)), 0
    f = np.fft.rfftfreq(n_fft, h)
    for seg in np.split(W, np.flatnonzero(np.diff(T) < 0) + 1):
        for i in range(0, len(seg) - n_fft + 1, n_fft):
            x = seg[i:i + n_fft, 0]
            x = x - np.polyval(np.polyfit(t, x, 1), t)     # linear detrend
            p = np.abs(np.fft.rfft(x * taper)) ** 2
            # One-sided Parseval, window-corrected: mean square in a band.
            band_ms += [2 * p[(f >= lo) & (f < hi)].sum() / (n_fft * (taper ** 2).sum())
                        for lo, hi in WHEEL_BANDS]
            n_win += 1
        if len(seg) <= 2 * k:
            continue
        smooth = np.stack([np.convolve(seg[:, j], win, mode="same")
                           for j in range(seg.shape[1])], 1)
        res = (seg - smooth)[k:-k]        # the edges the window cannot fill
        sq += (res ** 2).sum(0)
        n += len(res)
        s = np.sign(seg[:, 0]) * (np.abs(seg[:, 0]) > _REV_HYST)
        s = s[s != 0]
        rev += int(np.count_nonzero(np.diff(s)))
        dur += len(seg) * h
    return {"rms": np.sqrt(sq / max(n, 1)),
            "rev_per_s": rev / max(dur, 1e-9),
            "band_rms": np.sqrt(band_ms / max(n_win, 1))}


def rollout_grid(pol, env, cmds, params):
    """Run the eval grid, keeping every normalized action alongside the
    per-command metrics. Actions are recorded as a fraction of their bound,
    i.e. what the network emits before scale_action, so the channels share one
    scale and the numbers match what the reward sees.

    `A` is (steps, this policy's width) — ragged across a mixed policy set, so
    every consumer reads its width off the array rather than assuming three.

    THE ACHIEVED STEER RATE IS RECORDED TOO, because a saturated action does not
    mean a saturated joint. Every general policy pins the steer channel to its
    bound most of the time AND alternates sign, so the commanded RATE is
    bang-bang while the integrated command — and the joint — only creep. The
    saturation table below cannot tell those apart; `flips/s` and this can.
    Summarised into `m` rather than returned, so the tuple every caller unpacks
    is unchanged.
    """
    scale = act_scale(pol)
    acts, rates = [], []

    def act(obs):
        a = np.asarray(pol.action(obs), float) / scale
        acts.append(a)
        rates.append(float(env.data.qvel[env._sd]))
        return a[:env.action_space.shape[0]]

    # Surface speeds, so the columns share a unit: tread = hub x outer radius;
    # roller = k_roller x ring_spin (geared RELATIVE to the hub) x mean roller
    # radius; servo = input shaft / belt ratio, at the XC430's output.
    from aow_sim.control import general_env as ge
    ow, dt = params["omni_wheel"], params["drivetrain"]
    dofs = [int(env.model.joint(j).dofadr[0])
            for j in ("hub_spin", "ring_spin", "input_a_spin", "input_b_spin")]
    r_roller = (float(ow["roller"]["big_diameter"])
                + float(ow["roller"]["small_diameter"])) / 4
    gain = np.array([float(ow["outer_radius"]), float(dt["k_roller"]) * r_roller,
                     1 / float(dt["belt_ratio"]), 1 / float(dt["belt_ratio"])])
    wheel, wheel_t = [], []

    def sample(data):
        wheel_t.append(float(data.time))
        wheel.append(data.qvel[dofs] * gain)

    ge.mujoco = _StepTap(ge.mujoco, sample)
    try:
        m, rows = _eval_episodes(env, act, cmds)
    finally:
        ge.mujoco = ge.mujoco._module
    m["wheel"] = wheel_jitter(np.array(wheel), np.array(wheel_t),
                              float(env.model.opt.timestep))
    A = np.array(acts)
    # Steer joint rate against the XC330's own no-load speed. gear_ratio is
    # servo rotation per steer rotation, so the servo turns that much faster.
    no_load = (params["servos"]["xc330_t181"]["no_load_rpm"] * 2 * np.pi / 60
               / float(params["bike"]["steering"]["gear_ratio"]))
    r = np.abs(np.asarray(rates))
    m["ctrl_dt"] = float(env.ctrl_dt)
    m["steer_rate_med_frac"] = float(np.median(r) / no_load)
    m["steer_rate_p95_frac"] = float(np.percentile(r, 95) / no_load)
    # Per-episode slices, so a per-step difference never straddles a reset.
    i, per = 0, {}
    for r in rows:
        per[r["cmd"]] = A[i:i + r["steps"]]
        i += r["steps"]
    return m, rows, A, per


def _one_policy(job):
    """One policy's whole grid, in a worker process.

    Everything is built HERE: a MuJoCo model does not pickle, so the parent
    can only hand over strings and floats. Parallelised over POLICIES and
    deliberately not over commands -- `_eval_episodes` seeds each episode
    `10_000 + k` on the index WITHIN the list it is handed, so splitting the
    commands across workers would silently re-seed every episode and produce
    numbers matching no other table in the project.
    """
    name, encoder, force_odo, ahrs, tau, channels, plant = job
    # "own" is each policy's recorded drivetrain (policy_env_overrides);
    # anything else forces one drivetrain_eval variant on every policy.
    params = load_params() if plant == "own" else params_for(plant)
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    pol = load_general(name)
    if plant != "own":
        pol.drivetrain_model = None
    if encoder:
        pol.odometry_encoder = encoder
    if force_odo:
        pol.obs_odometry = True
    # ON THE POLICY, not on the cfg. `policy_env_overrides` overlays the
    # policy's own declaration on top of `cfg["env"]`, so an AHRS-trained
    # policy would override a cfg-level setting right back and `--ahrs` would
    # be a silent no-op for exactly the policies it matters most for. Setting
    # it here is the same route `--encoder` takes one line up. Leaving the
    # flag off now means "whatever the move declares", which is what makes a
    # bare `chatter.py` run an AHRS policy against its AHRS.
    if ahrs:
        pol.ahrs_level = ahrs
        pol.ahrs_tau_s = tau
        pol.ahrs_channels = channels
    return name, rollout_grid(pol, env_for(pol, params, cfg),
                              eval_cmds(cfg["env"]["v_max"]), params)


def cross_axis(rows):
    """Behavioural counterpart to the confusion matrix's `cross_axis_rms`:
    longitudinal and lateral are independent commands, so a pure command on
    one axis should produce no motion on the other."""
    def mean(sel, key):
        v = [r[key] for r in rows if sel(r["cmd"])]
        return float(np.mean(v)) if v else float("nan")

    pure_lon = lambda c: abs(c[1]) < 1e-9 and abs(c[2]) < 1
    return {
        "fwd_v_lat": mean(lambda c: c[0] > 0.1 and pure_lon(c), "v_lat_ach"),
        "rev_v_lat": mean(lambda c: c[0] < -0.1 and pure_lon(c), "v_lat_ach"),
        "crab_v_lon": mean(lambda c: abs(c[1]) > 0.1, "v_ach"),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--w-smooth", type=float, default=0.05,
                    help="price the per-channel table at this weight, as "
                         "reward per step")
    ap.add_argument("--policies", nargs="+", metavar="NAME",
                    help="move names to evaluate instead of the default set; "
                         "any moves/<NAME>.yaml will do")
    ap.add_argument("--encoder", choices=list(ENCODERS), default=None,
                    help="override the encoder model every odometry policy "
                         "runs on: 'ideal' instantaneous joint velocity, "
                         "'counts' quantised + our 25 ms RateFilter as "
                         "hw/dynamixel derives it, 'reported' the XC430's own "
                         "Present Velocity (a ~50 ms boxcar, 25 ms of lag)")
    ap.add_argument("--ahrs", choices=("none", "tm151_static", "tm151", "tm171"),
                    default=None,
                    help="TM151 orientation/gyro error model on roll, "
                         "roll_rate and yaw_rate: 'static' the datasheet "
                         "static accuracy (0.5 deg RMS), 'typical' its DYNAMIC "
                         "accuracy (1.5 deg), which is what a moving bike gets")
    ap.add_argument("--ahrs-tau", type=float, default=TAU_ORIENT_S,
                    help="correlation time of the orientation error [s]. "
                         "Defaults to the MEASURED sim_ahrs.TAU_ORIENT_S "
                         "(0.19); it was 2.0, a guess, until 2026-08-28, so "
                         "tables older than that were taken at 2.0. The effect "
                         "is small either way -- see analysis/ahrs_tau.py")
    ap.add_argument("--ahrs-channels", choices=("both", "orient", "gyro"),
                    default="both",
                    help="attribute the damage: 'orient' corrupts only the "
                         "attitude, 'gyro' only the rates")
    ap.add_argument("--force-odometry", action="store_true",
                    help="run EVERY policy on the onboard velocity estimate, "
                         "including ones trained on MuJoCo truth. This is the "
                         "deployment question -- what the Pi will hand them -- "
                         "and it is not what a truth-trained policy's own row "
                         "above measures.")
    ap.add_argument("--plant", default="own", choices=["own", *VARIANTS],
                    help="drivetrain every policy runs on: 'own' is each "
                         "policy's recorded one (ideal for most), else an "
                         "analysis/drivetrain_eval.py variant forced on all, "
                         "e.g. full (P 100) or full_p400")
    args = ap.parse_args()
    names = args.policies or list(POLICIES)

    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    # No AHRS patch here: the parent only needs v_max off this cfg, and the
    # sensor mode is applied per policy inside `_one_policy` -- see the note
    # there for why it cannot live in the config.
    cmds = eval_cmds(cfg["env"]["v_max"])

    # One env PER POLICY: a velocity-windowed policy needs a wider observation
    # and its own filter constant, so a single shared env cannot serve both.
    # That makes the policies independent, so they run in parallel -- one
    # process each. `--force-odometry` overrides a policy's own declaration
    # and is deliberately a flag: a truth-trained policy is OUT OF
    # DISTRIBUTION under it, so those rows answer "does it survive
    # deployment", not "how good is it".
    jobs = [(k, args.encoder, args.force_odometry, args.ahrs, args.ahrs_tau,
             args.ahrs_channels, args.plant) for k in names]
    done = {}
    with ProcessPoolExecutor(max_workers=min(len(jobs),
                                             os.cpu_count() or 1)) as ex:
        for name, res in ex.map(_one_policy, jobs):
            done[name] = res
    out = {k: done[k] for k in names}   # requested order, not completion order

    w = f"{max(len(k) for k in out) + 2}"

    print(f"eval grid: {len(cmds)} commands, identical seeds, "
          f"randomization off, plant: {args.plant}\n")
    print(f"{'policy':{w}}{'score':>8}{'surv':>7}{'track_geo':>11}"
          f"{'vel_err':>9}{'head_deg':>10}{'drift_m':>9}{'steer_rest':>11}")
    for k, (m, *_) in out.items():
        print(f"{k:{w}}{_score(m):>8.3f}{m['survive_rate']:>7.2f}"
              f"{m['track_geo']:>11.3f}{m['vel_err']:>9.3f}"
              f"{m['head_err_deg']:>10.1f}{m['drift_m']:>9.3f}"
              f"{m['steer_rest_deg']:>11.1f}")

    # PER FAMILY, because the whole-grid median is blind by construction: it
    # cannot see a failure affecting fewer than half the commands, and the
    # grid holds exactly 6 large turns. One table per family, policies as
    # rows -- the same shape as the by_family block in moves/*.yaml.
    print("\nBY COMMAND FAMILY. surv is a RATE within the family; read it "
          "WITH t_head,\nbecause high survival means competence OR refusal "
          "-- a policy that declines a\nturn cannot fall doing it. t_head is "
          "seconds to first get inside 10 deg, over\nthe family members that "
          "ask for a heading change (n_t of them).")
    fams = list(next(iter(out.values()))[0]["by_family"])
    for fam in fams:
        n = next(iter(out.values()))[0]["by_family"][fam]["n"]
        print(f"\n  {fam}  (n={n})")
        head = (f"  {'policy':{w}}{'surv':>7}{'t_head':>9}{'n_t':>5}"
                f"{'head_tail':>11}{'vel_med':>9}")
        if fam == "hold":
            head += f"{'drift_m':>9}{'overshoot':>11}"
        print(head)
        for k, (m, *_) in out.items():
            f = m["by_family"][fam]
            line = (f"  {k:{w}}{f['survive_rate']:>7.2f}"
                    + (f"{f['t_head_s']:>8.2f}s{f['t_head_n']:>5}"
                       if "t_head_s" in f else f"{'-':>9}{'-':>5}")
                    + f"{f['head_err_tail']:>10.1f}°{f['vel_err_med']:>9.3f}")
            if fam == "hold":
                line += f"{f['drift_m']:>9.3f}{f['drift_overshoot']:>11.3f}"
            print(line)
    print("\n  drift is hold-only and the grid has exactly one hold command, "
          "so it is ONE\n  episode. overshoot is peak minus final: 0 means it "
          "left and kept going,\n  > 0 means it wandered out and came back.")

    def cells(values, fmt):
        """One cell per CHANNEL, "-" where this policy has no such channel."""
        return "".join(f"{values[i]:>10{fmt}}" if i < len(values)
                       else f"{'-':>10}" for i in range(len(CHANNELS)))

    print("\nmean squared per-step action change, by channel "
          "(fraction of bound)")
    print(f"{'policy':{w}}" + "".join(f"{'d' + c + '^2':>10}" for c in CHANNELS)
          + f"{'total':>9}{'@' + str(args.w_smooth):>10}")
    for k, (_m, _r, A, _p) in out.items():
        d2 = np.diff(A, axis=0) ** 2      # grid-wide; per-family below is
        mu = d2.mean(0)                   #   the reset-safe version
        # The TOTAL is not comparable across widths -- a wings policy sums four
        # channels against three. Compare the per-channel cells.
        print(f"{k:{w}}" + cells(mu, ".3f")
              + f"{mu.sum():>9.3f}{args.w_smooth * mu.sum():>10.3f}")

    print("\nfraction of steps pinned to a bound (|a| > 0.98)")
    print(f"{'policy':{w}}" + "".join(f"{c:>10}" for c in CHANNELS)
          + f"{'any':>9}")
    for k, (_m, _r, A, _p) in out.items():
        sat = np.abs(A) > 0.98
        print(f"{k:{w}}" + cells(sat.mean(0), ".1%")
              + f"{sat.any(1).mean():>9.1%}")

    # SATURATION AND CHATTER ARE DIFFERENT FAILURES and the tables above cannot
    # separate them: a channel pinned to +1 for a whole episode and a channel
    # alternating +1/-1 every other step both read as ~100% saturated. The sign
    # flip rate is what tells them apart. At 50 Hz the ceiling is 50 flips/s
    # (one every step) and 25/s is one every other step, so a policy sitting in
    # the high twenties is dithering at a substantial fraction of the fastest
    # the control loop can express.
    _dt = next(iter(out.values()))[0]["ctrl_dt"]
    print("\naction sign flips per second, by channel "
          f"(ceiling is {1 / _dt:.0f}/s, a flip EVERY step; "
          f"{0.5 / _dt:.0f}/s is every other step)")
    print(f"{'policy':{w}}" + "".join(f"{c:>10}" for c in CHANNELS))
    for k, (m, _r, _A, per) in out.items():
        # Per EPISODE, so a flip is never counted across a reset.
        flips = np.zeros(next(iter(per.values())).shape[1])
        steps = 0
        for seg in per.values():
            if len(seg) > 1:
                flips += (np.diff(np.sign(seg), axis=0) != 0).sum(0)
                steps += len(seg) - 1
        print(f"{k:{w}}" + cells(flips / (steps * m["ctrl_dt"]), ".1f"))

    print("\nACHIEVED steer joint rate, as a fraction of the XC330's no-load "
          "speed")
    print(f"{'policy':{w}}{'median':>10}{'p95':>10}")
    for k, (m, *_) in out.items():
        print(f"{k:{w}}{m['steer_rate_med_frac']:>10.1%}"
              f"{m['steer_rate_p95_frac']:>10.1%}")
    print("  A policy can saturate the steer RATE command while the joint "
          "barely moves:\n  the command alternates faster than the joint can "
          "follow, so the integral of\n  the rate stays small. Read this "
          "against the saturation and flips tables.")

    print("\nsum-squared per-step action change, by command family")
    print(f"{'policy':{w}}" + "".join(f"{f:>9}" for f in FAMILIES))
    for k, (_m, _r, _A, per) in out.items():
        line = ""
        for pred in FAMILIES.values():
            v = [np.sum(np.diff(seg, axis=0) ** 2, 1).mean()
                 for c, seg in per.items() if pred(c) and len(seg) > 1]
            line += f"{np.mean(v):>9.3f}" if v else f"{'-':>9}"
        print(f"{k:{w}}{line}")

    print(f"\nREAR WHEEL JITTER, at physics rate: RMS of speed minus its "
          f"centred {_SMOOTH_S * 1e3:.0f} ms sliding-window mean")
    print(f"{'policy':{w}}{'tread':>9}{'roller':>9}{'servo_a':>9}{'servo_b':>9}"
          f"{'rev/s':>8}")
    for k, (m, *_) in out.items():
        wj = m["wheel"]
        rms = wj["rms"]
        print(f"{k:{w}}{rms[0] * 1e3:>9.1f}{rms[1] * 1e3:>9.1f}"
              f"{np.degrees(rms[2]):>9.1f}{np.degrees(rms[3]):>9.1f}"
              f"{wj['rev_per_s']:>8.2f}")
    print(f"  tread and roller in mm/s at the surface, servo in deg/s at the "
          f"XC430 output.\n  rev/s: tread direction reversals past "
          f"+-{_REV_HYST * 1e3:.0f} mm/s. The 100 ms residual weights "
          f"frequencies unevenly\n  (x0.36 at 5 Hz, x1.22 at 14 Hz): read the "
          f"band table below for WHERE the energy is.")
    print("\ntread speed RMS by frequency band [mm/s], raw speed, linear "
          "detrend per ~0.8 s window")
    print(f"{'policy':{w}}" + "".join(f"{f'{lo}-{hi}Hz':>10}"
                                      for lo, hi in WHEEL_BANDS))
    for k, (m, *_) in out.items():
        print(f"{k:{w}}" + "".join(f"{v * 1e3:>10.1f}"
                                   for v in m["wheel"]["band_rms"]))
    print("  The action is held for 20 ms, so dither every step or every "
          "other step lands at\n  25-50 Hz; the bench put the XC430 loop's "
          "resonance near 20 Hz.")

    print("\ncross-axis leakage in BEHAVIOUR [m/s]: motion on the axis that "
          "was not commanded")
    print(f"{'policy':{w}}{'fwd:v_lat':>11}{'rev:v_lat':>11}{'crab:v_lon':>12}")
    for k, (_m, rows, *_) in out.items():
        c = cross_axis(rows)
        print(f"{k:{w}}{c['fwd_v_lat']:>+11.3f}{c['rev_v_lat']:>+11.3f}"
              f"{c['crab_v_lon']:>+12.3f}")

    print("\nachieved / commanded speed, per direction and per crab side")
    print(f"{'policy':{w}}{'fwd':>8}{'rev':>8}{'crabL':>8}{'crabR':>8}"
          f"{'crab_hd':>9}{'turn_asym':>11}")
    for k, (m, *_) in out.items():
        print(f"{k:{w}}{m['speed_ratio_fwd']:>8.2f}{m['speed_ratio_rev']:>8.2f}"
              f"{m['crab_ratio_left']:>8.2f}{m['crab_ratio_right']:>8.2f}"
              f"{m['crab_head_err']:>9.1f}{m['turn_asym']:>11.3f}")


if __name__ == "__main__":
    main()
