"""Lateral estimators against truth on the eval grid, open loop.

control.general_move drives per_command.py's 20 commands (same cfg, seeds,
its own sensors and plant; the policy SEES the shipped estimator). Every
100 Hz estimator tick, alternatives are computed from the SAME sensor
readings and compared to truth: the chassis origin's velocity in the true
yaw frame, which is what the reward tracks. Open loop: the policy was not
flying any alternative, so this ranks accuracy, not closed-loop survival.
Recorded in docs/plans/odometry-sharp-steer.md ("Every lateral estimator").

  python analysis/sensing/odo_compare.py [policy]
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "policy"))  # sibling category
from per_command import _cfg_for, label_for                      # noqa: E402
from aow_sim.build_model import load_params                       # noqa: E402
from aow_sim.control.steer import wheel_heading                    # noqa: E402
from aow_sim.hw.odometry import GRAVITY                            # noqa: E402
from aow_sim.sim_odometry import _rpy                              # noqa: E402
from aow_sim.train_general_rl import _eval_episodes, eval_cmds     # noqa: E402
from rsa_policies import env_for, load_general                     # noqa: E402

V_REF = 0.25
LEAKS = (0.25, 1.0)          # s

name = sys.argv[1] if len(sys.argv) > 1 else load_params()["control"]["general_move"]
params = load_params()
cfg = _cfg_for()
pol = load_general(name)
env = env_for(pol, params, cfg)
odo = env._odo
assert odo is not None and odo.fusion == "front", "needs an odometry-trained policy"
est = odo.est
print(name, "| ahrs", pol.ahrs_level, "| encoder", pol.odometry_encoder, "| drivetrain",
      "detailed" if "drivetrain_model" in env.p else "ideal")

rows = []
ep = [-1]
state = {"leak": {t: 0.0 for t in LEAKS}}
last_rates = [None]
_wheel_rates = odo._wheel_rates
_advance = odo._advance


def wheel_rates(data, dt):
    r = _wheel_rates(data, dt)
    last_rates[0] = r
    return r


def advance(data, dt):
    v_lon, v_lat = _advance(data, dt)              # the shipped estimator
    wa, wb = last_rates[0]
    steer = float(odo._read(data, "steer_pos")[0])
    if odo.encoder != "ideal":
        q = 4096 / (2 * np.pi) * float(params["bike"]["steering"]["gear_ratio"])
        steer = round(steer * q) / q
    yaw_rate = float(odo._read(data, "ahrs_gyro")[2])
    roll, pitch, _ = _rpy(odo._read(data, "ahrs_quat"))
    accel = np.asarray(odo._read(data, "ahrs_accel"), float)
    front, conf = est.lateral_from_front(v_lon, steer, yaw_rate)
    roller = est.roller_lateral(wa, wb)
    w_speed = float(np.clip(abs(v_lon) / V_REF, 0, 1)) * conf
    a_lat = accel[1] - GRAVITY * np.sin(roll) * np.cos(pitch) - v_lon * yaw_rate
    leak = {}
    for tau, v in state["leak"].items():
        pred = v * (1 - dt / tau) + a_lat * dt
        state["leak"][tau] = leak[tau] = pred + conf * (front - pred)
    # truth: the chassis origin in the TRUE yaw frame (the reward's)
    _r, _p, yaw = _rpy(data.qpos[3:7])
    c, s = np.cos(yaw), np.sin(yaw)
    t_lon = c * data.qvel[0] + s * data.qvel[1]
    t_lat = -s * data.qvel[0] + c * data.qvel[1]
    th = abs(wheel_heading(steer, est.rake))
    fold = np.degrees(min(th, np.pi - th))
    rows.append((ep[0], fold, conf, t_lon, t_lat, v_lon, v_lat, front,
                 roller, conf * front + (1 - conf) * roller,
                 w_speed * front + (1 - w_speed) * roller,
                 conf * front, *[leak[t] for t in LEAKS]))
    return v_lon, v_lat


odo._wheel_rates = wheel_rates
odo._advance = advance
scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
n_act = env.action_space.shape[0]


def act(o):
    return (np.asarray(pol.action(o), float) / scale)[:n_act]


def reset():
    ep[0] += 1
    for t in LEAKS:
        state["leak"][t] = 0.0
    pol.reset()


act.reset = reset
cmds = eval_cmds(cfg["env"]["v_max"])
_m, res = _eval_episodes(env, act, cmds)
a = np.array(rows)
a = a[a[:, 0] >= 0]
NAMES = ["shipped (front + accel coast)", "front constraint only", "roller kinematics",
         "conf blend front/roller", "speed-aware blend", "front, conf-weighted to 0",
         *[f"front + accel, leak {t:g} s" for t in LEAKS]]
COLS = [6, 7, 8, 9, 10, 11, 12, 13]
t_lat, t_lon = a[:, 4], a[:, 3]


def rms(x):
    return 1e3 * float(np.sqrt(np.mean(x ** 2)))


print(f"\n{len(a)} ticks at 100 Hz, {len(cmds)} commands, "
      f"fell {sum(r['fell'] for r in res)}/{len(res)}")
print(f"v_lon (hub): RMS err {rms(a[:, 5] - t_lon):.1f} mm/s against truth RMS {rms(t_lon):.0f}")
bins = [(0, 15), (15, 30), (30, 45), (45, 90)]
hdr = "".join(f"{f'{lo}-{hi} deg':>12s}" for lo, hi in bins)
print(f"\nv_lat RMS error, mm/s (truth RMS {rms(t_lat):.0f}), by wheel heading:")
print(f"{'estimator':34s}{'all':>8s}{'corr':>7s}{hdr}")
counts = "".join(f"{f'n={int(((a[:, 1] >= lo) & (a[:, 1] < hi)).sum())}':>12s}" for lo, hi in bins)
print(f"{'':34s}{'':>8s}{'':>7s}{counts}")
for nm, col in zip(NAMES, COLS):
    e = a[:, col] - t_lat
    by = "".join(f"{rms(e[(a[:, 1] >= lo) & (a[:, 1] < hi)]) if ((a[:, 1] >= lo) & (a[:, 1] < hi)).any() else float('nan'):12.1f}"
                 for lo, hi in bins)
    print(f"{nm:34s}{rms(e):8.1f}{np.corrcoef(a[:, col], t_lat)[0, 1]:7.2f}{by}")

print("\nper command, v_lat RMS err mm/s: shipped / front only / roller / speed-aware   (truth RMS)")
for k, (cmd, r) in enumerate(zip(cmds, res)):
    m = a[:, 0] == k
    e = lambda col: rms(a[m, col] - t_lat[m])
    print(f"  {label_for(cmd):>22s} {'FELL' if r['fell'] else '    '} "
          f"{e(6):6.1f} {e(7):6.1f} {e(8):6.1f} {e(10):6.1f}   ({rms(t_lat[m]):.0f})")
