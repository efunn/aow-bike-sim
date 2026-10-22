"""How far does the policy steer on the eval grid? per_command.py's episodes
(same cfg, same seeds, its own sensors), with the steer joint logged every
control step. The wheel is front-back symmetric, so the angle that matters
to the estimator is the ground heading folded into [0, 90] deg: the same
|cos theta| the confidence uses."""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from per_command import _cfg_for, label_for                      # noqa: E402
from aow_sim.build_model import load_params                       # noqa: E402
from aow_sim.control.steer import wheel_heading                    # noqa: E402
from aow_sim.hw.odometry import COS_MIN                            # noqa: E402
from aow_sim.train_general_rl import _eval_episodes, eval_cmds     # noqa: E402
from rsa_policies import env_for, load_general                     # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else load_params()["control"]["general_move"]
params = load_params()
cfg = _cfg_for()
pol = load_general(name)
env = env_for(pol, params, cfg)
print(name, "| ahrs", pol.ahrs_level, "| odometry", pol.obs_odometry,
      pol.odometry_encoder, "| drivetrain",
      "detailed" if "drivetrain_model" in env.p else "ideal")
scale = np.asarray(pol.bounds.to_list(), float)[:pol.act_dim]
n_act = env.action_space.shape[0]
rake = np.deg2rad(float(params["bike"]["rake_deg"]))
sq = env.model.joint("steer_joint").qposadr[0]

log = []          # (episode index, folded heading deg)
ep = [-1]


def act(o):
    log.append((ep[0], float(env.data.qpos[sq])))
    return (np.asarray(pol.action(o), float) / scale)[:n_act]


def reset():
    ep[0] += 1
    pol.reset()


act.reset = reset
cmds = eval_cmds(cfg["env"]["v_max"])
_m, rows = _eval_episodes(env, act, cmds)
a = np.array(log)
th = np.array([abs(wheel_heading(d, rake)) for d in a[:, 1]])
fold = np.degrees(np.minimum(th, np.pi - th))
conf = np.clip((np.cos(np.deg2rad(fold)) - COS_MIN) / (1 - COS_MIN), 0, 1) ** 2


def pct(x):
    return f"{100 * np.mean(x):5.1f}%"


print(f"all {len(fold)} steps: past 45 {pct(fold > 45)}  past 60 {pct(fold > 60)}  "
      f"past 75 {pct(fold > 75)}  conf<0.1 {pct(conf < 0.1)}  "
      f"median {np.median(fold):.1f} deg  p95 {np.percentile(fold, 95):.1f}  max {fold.max():.1f}")
for k, (cmd, r) in enumerate(zip(cmds, rows)):
    m = a[:, 0] == k
    f = fold[m]
    print(f"  {label_for(cmd):>14s} {'FELL' if r['fell'] else '    '} "
          f"past45 {pct(f > 45)} past60 {pct(f > 60)} past75 {pct(f > 75)} "
          f"p95 {np.percentile(f, 95):5.1f} max {f.max():5.1f}")
