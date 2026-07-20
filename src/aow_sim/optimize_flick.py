"""Offline trajectory optimization for the two-arc 180-degree flick.

Direct shooting: differential_evolution over the 7 flick parameters, each
candidate scored by a full MuJoCo rollout with roll/lateral crawl-balance
underneath (see control/flick.py). Writes the winner to moves/flick.yaml.

  python -m aow_sim.optimize_flick [--maxiter N] [--popsize K] [--seed S]

Run it yourself; it never modifies config/bike_params.yaml. Rerunning only
rewrites moves/flick.yaml.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from scipy.optimize import differential_evolution, minimize

from .build_model import build_model, load_params
from .control import flick as F
from .control.drive import DriveController
from .control.linearize import settle_upright

# Module globals for the worker processes (differential_evolution with
# workers=-1 pickles the objective; keep the heavy handles process-global).
_G = {}


def _init_globals():
    if _G:
        return
    params = load_params()
    model = build_model(params, variant="full")
    eq = settle_upright(model).qpos.copy()
    K0 = DriveController(params, model)._K0
    _G.update(params=params, model=model, eq=eq, K0=K0)


def _objective(x) -> float:
    _init_globals()
    fl = F.FlickTrajectory.from_params(x)
    m = F.rollout(_G["model"], _G["params"], _G["eq"], _G["K0"], fl, settle=1.5)
    return F.cost(m)


# Hand-designed warm-start (a plausible reverse->forward two-arc flick).
WARM_START = np.array([2.6, np.pi * 0.30, np.pi * 0.55, np.pi * 0.80,
                       -0.40, 0.0, 0.40])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--maxiter", type=int, default=40)
    ap.add_argument("--popsize", type=int, default=15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=-1)
    ap.add_argument("--name", default="flick", help="output move name")
    ap.add_argument("--reverse-first", action="store_true",
                    help="constrain the drive to go backward then forward "
                         "(first hub knot < 0, last > 0)")
    args = ap.parse_args()

    _init_globals()
    bounds = list(F.PARAM_BOUNDS)
    warm = WARM_START.copy()
    if args.reverse_first:
        bounds[4] = (-0.6, -0.05)            # first hub knot: reverse
        bounds[6] = (0.05, 0.6)              # last hub knot: forward
        warm[4], warm[6] = -0.4, 0.4
    rng = np.random.default_rng(args.seed)
    lb = np.array([b[0] for b in bounds])
    ub = np.array([b[1] for b in bounds])
    n_pop = args.popsize * F.N_PARAM
    init = rng.uniform(lb, ub, size=(n_pop, F.N_PARAM))
    init[0] = warm                           # seed one member with the guess

    print(f"optimizing flick: {F.N_PARAM} params, popsize {n_pop}, "
          f"maxiter {args.maxiter}, workers {args.workers}")
    t0 = time.time()
    res = differential_evolution(
        _objective, bounds, init=init, maxiter=args.maxiter,
        tol=1e-3, mutation=(0.5, 1.0), recombination=0.7, seed=args.seed,
        polish=False, workers=args.workers, updating="deferred")
    # Nelder-Mead polish (serial, cheap) on the winner.
    pol = minimize(_objective, res.x, method="Nelder-Mead",
                   options={"maxiter": 200, "xatol": 1e-3, "fatol": 1e-2})
    best = pol.x if pol.fun < res.fun else res.x
    print(f"done in {time.time()-t0:.0f}s; cost {min(pol.fun, res.fun):.3f}")

    fl = F.FlickTrajectory.from_params(best)
    metrics = F.rollout(_G["model"], _G["params"], _G["eq"], _G["K0"], fl,
                        settle=2.0)
    L = _G["params"]["bike"]["wheelbase"]
    print(f"  duration      {fl.T:.2f} s")
    print(f"  final yaw     {np.degrees(metrics['yaw_final']):+.1f} deg")
    print(f"  max roll      {np.degrees(metrics['max_roll']):.1f} deg")
    print(f"  lateral env   {metrics['max_lat']:.3f} m ({metrics['max_lat']/L:.2f} L)")
    print(f"  x shift       {metrics['x_shift']:+.2f} m ({metrics['x_shift']/L:+.2f} L)")
    print(f"  survived      {not metrics['fell']}")

    path = F.save_move(args.name, fl, metrics, F.COST_WEIGHTS)
    # backfill the L-normalized envelope
    import yaml
    doc = yaml.safe_load(open(path))
    doc["optimized"]["metrics"]["lateral_envelope_L"] = round(float(metrics["max_lat"] / L), 3)
    yaml.safe_dump(doc, open(path, "w"), sort_keys=False)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
