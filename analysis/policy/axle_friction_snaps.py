"""Standing 90 deg heading snaps against front-axle friction: where does a
policy start to fall?

    python analysis/policy/axle_friction_snaps.py                      # the pointer
    python analysis/policy/axle_friction_snaps.py general_puffer_5090_v5floor_s1_80M \
        --friction 0 0.01 0.03 0.1 --seeds 20

Why it exists (2026-10-31): after the 10-28 steer shim and front axle nut,
standing 90 deg snaps on the bike fell 4 times in 9 (v4 2/7, v5 s1 2/2)
against 3 in 102 for v4 before. The steer tracked the same in the snaps
that held, and the drive command sat at its limit in nearly every snap, so
a wheel that "rolls a bit tight" was as good a suspect as the shim.

Each episode: the move's own env (rsa_policies.env_for: obs blocks,
drivetrain, wheel play, floor), zero command for --hold s so the policy's
hidden state settles, then the heading command jumps by +-90 deg, as the
station's d-pad does, and the episode runs to --after s past the snap.
FELL is the env's own termination. `bike.front_wheel.axle_friction_nm` is the
front_spin joint's frictionloss (build_model.py), absent from bike_params.

Prints, per friction level: falls / snaps for each direction, the time from
snap to fall; of the snaps that stayed up, how many came within 10 deg of
the new heading, how long that took, and the heading error at the end; and
the peak yaw rate in the turn's direction. `--play-from` flies the policy on
another move's wheel play (or `none`), so two policies share one plant;
`--floor-mu` / `--floor-timeconst` override the floor it stands on.
"""
from __future__ import annotations

import argparse
import copy
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))


def _episode(job):
    name, mu, sign, seed, hold, after, config, play_from, floor = job
    from aow_sim.build_model import load_params
    from aow_sim.control.general_env import _load_rl_config
    from aow_sim.eval_general import _eval_cfg
    from rsa_policies import env_for, load_general

    cfg = _eval_cfg(_load_rl_config(config))
    cfg["env"] = {**cfg["env"], "max_episode_s": hold + after + 1.0}
    p = copy.deepcopy(load_params())
    p["bike"]["front_wheel"]["axle_friction_nm"] = mu
    pol = load_general(name)
    if play_from == "none":
        pol.wheel_play = None
    elif play_from:
        pol.wheel_play = load_general(play_from).wheel_play
    if floor:
        pol.floor = {**(pol.floor or {}), **floor}
    env = env_for(pol, p, cfg)
    sc = np.array([pol.bounds.steer_rate_max, pol.bounds.hub_max, pol.bounds.diff_max])
    sc = np.where(sc == 0, 1, sc)
    obs, _ = env.reset(seed=seed, options={"v_cmd": (0.0, 0.0), "psi_cmd_rel": 0.0,
                                           "difficulty": 1.0})
    pol.reset()
    n_hold, n_all = round(hold / env.ctrl_dt), round((hold + after) / env.ctrl_dt)
    yaw_pk, t_fall, fell_before, t_turned = 0.0, None, False, None
    for k in range(n_all):
        if k == n_hold:
            env._psi_cmd = env._psi + sign * np.pi / 2
        act = np.asarray(pol.action(obs), float)[:3] / sc
        obs, _r, term, trunc, _ = env.step(act)
        if k >= n_hold:
            yaw_pk = max(yaw_pk, sign * float(env.data.cvel[1][2]))  # world yaw rate of the chassis
            err = abs(np.angle(np.exp(1j * (env._psi_cmd - env._psi))))
            if t_turned is None and err < np.radians(10):
                t_turned = (k + 1 - n_hold) * env.ctrl_dt
        if term:
            if k < n_hold:
                fell_before = True
            else:
                t_fall = (k + 1 - n_hold) * env.ctrl_dt
            break
        if trunc:
            break
    return mu, sign, seed, fell_before, t_fall, np.degrees(yaw_pk), t_turned, np.degrees(err)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("move", nargs="?", default=None,
                    help="a moves/ name; default control.general_move")
    ap.add_argument("--friction", type=float, nargs="+",
                    default=[0.0, 0.003, 0.01, 0.03, 0.1, 0.3], metavar="NM",
                    help="front axle frictionloss levels [N m]")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--hold", type=float, default=1.5, help="s at zero command first")
    ap.add_argument("--after", type=float, default=3.0, help="s watched after the snap")
    ap.add_argument("--config", default="config/rl/rl_general_v3.yaml",
                    help="eval settings only, as standstill.py")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--play-from", default=None, metavar="MOVE|none",
                    help="fly on ANOTHER move's wheel play (or none), so two "
                         "policies can share one plant")
    ap.add_argument("--floor-mu", type=float, default=None,
                    help="the floor's sliding friction (sim.friction_sliding)")
    ap.add_argument("--floor-timeconst", type=float, default=None, metavar="S",
                    help="the floor's contact_solref time constant, dampratio 1")
    a = ap.parse_args()
    from aow_sim.build_model import load_params
    name = a.move or load_params()["control"]["general_move"]

    floor = {}
    if a.floor_mu is not None:
        floor["friction_sliding"] = a.floor_mu
    if a.floor_timeconst is not None:
        floor["contact_solref"] = [a.floor_timeconst, 1.0]
    jobs = [(name, mu, s, seed, a.hold, a.after, a.config, a.play_from, floor)
            for mu in a.friction for s in (1, -1) for seed in range(a.seeds)]
    with ProcessPoolExecutor(a.workers) as ex:
        res = list(ex.map(_episode, jobs))

    plant = {None: "its own", "none": "no"}.get(a.play_from) or f"{a.play_from}'s"
    print(f"{name} on {plant} wheel play: standing +-90 deg snaps after {a.hold} s "
          f"at rest, {a.seeds} seeds per direction, watched {a.after} s"
          + (f", floor {floor}" if floor else ""))
    print(f"{'axle N m':>9}  {'+90 fell':>8}  {'-90 fell':>8}  {'t to fall, s':>13}  "
          f"{'within 10 deg':>13}  {'t to 10 deg, med':>16}  {'final err, med':>14}  "
          f"{'peak yaw, med':>13}")
    for mu in a.friction:
        r = [x for x in res if x[0] == mu and not x[3]]
        f = {s: sum(x[4] is not None for x in r if x[1] == s) for s in (1, -1)}
        tf = [x[4] for x in r if x[4] is not None]
        tfs = f"{min(tf):.2f}-{max(tf):.2f}" if tf else "-"
        up = [x for x in r if x[4] is None]
        turned = [x[6] for x in up if x[6] is not None]
        med_t = f"{np.median(turned):.2f} s" if turned else "-"
        med_e = f"{np.median([x[7] for x in up]):.1f} deg" if up else "-"
        pre = sum(x[3] for x in res if x[0] == mu)
        print(f"{mu:9.3f}  {f[1]:3d}/{a.seeds:<4d}  {f[-1]:3d}/{a.seeds:<4d}  {tfs:>13}  "
              f"{len(turned):5d}/{len(up):<7d}  {med_t:>16}  {med_e:>14}  "
              f"{np.median([x[5] for x in r]):9.0f} d/s"
              + (f"  ({pre} fell BEFORE the snap)" if pre else ""))

if __name__ == "__main__":
    main()
