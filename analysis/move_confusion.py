"""Move-by-move confusability of each general_rl policy, as a heatmap.

For every command condition ("move") the policy is rolled out closed-loop and
the mean penultimate-layer activation is taken as that move's pattern. Each
cell is the correlation between two move patterns: high = the policy
represents those two commands alike (confusable), low or negative = it holds
them apart.

The diagonal is NOT a trivial 1.0. Each trace is split in half and the
diagonal reports the split-half reliability of that move's own pattern -- the
noise ceiling. An off-diagonal cell can only be read against it: two moves are
"as similar as a move is to itself" when the cell approaches the diagonal.
Off-diagonals are cross-validated the same way (half A of one move against
half B of the other), so nothing on the plot is inflated by within-half noise.

The scores at the bottom are therefore reported twice: corrected for that
ceiling and raw. Two of the three have a target of ZERO, and noise shrinks
off-diagonals toward zero, so an unreliable policy would otherwise score
better than a clean one for being noisier -- see disattenuate().

WHY ONLY FIVE MOVES. The default set is the teleop primitives whose command is
SUSTAINED -- hold, fwd, rev, crabL, crabR. Those are stationary: the command is
still being asked for at the end of the trace, so one mean pattern fairly
represents the condition, and split-half reliability lands at 0.93-0.99.

Turns are not stationary and must be left out. A +-90 deg heading command
COMPLETES: |psi_err| falls 90 -> 24 -> 8 -> ~1 deg within about 1.5 s, after
which the state is indistinguishable from `hold` (zero velocity command, zero
heading error). Split a turn trace in half and half A is "turning" while half
B is "arrived and holding" -- two different states wearing one label. The
damage is threefold: reliability drops (turnL 0.64, turnR 0.73 against fwd
0.98); after the across-move centring below the two halves' residuals point
opposite ways, so the diagonal goes NEGATIVE (-0.25, -0.44); and every
off-diagonal in a turn row compares a mixture whose ratio is set by the
arbitrary trace length, so longer traces make turns look ever more like hold.

Windowing does not rescue it -- restricted to the first 40 steps, turnL 0.67
and turnR 0.39, no better. A turn is a TRAJECTORY, not a state, and it is
non-stationary at every timescale while it happens. Comparing turns needs a
time-resolved analysis (matched time offsets), not a static pattern.

  python analysis/move_confusion.py                  # the five sustained ones
  python analysis/move_confusion.py --set with_turns # the counterexample above
  python analysis/move_confusion.py --set crab --steps 200

Writes analysis/move_confusion_<set>.png and prints the matrices.
Loads moves/*.npz; changes nothing else.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aow_sim.build_model import load_params
from aow_sim.control.flick import MOVES_DIR
from aow_sim.control.general_env import GeneralEnv, _load_rl_config
from aow_sim.control.policy import load_policy_npz
from rsa_policies import (POLICIES, REPO, env_for, load_general, condition_sets,
                          hidden, trace)


def hold_quality(pol, env, steps):
    """Ground truth for the zero command: does the bike actually stand still?

    The confusion matrix cannot answer this. Its diagonal is split-half
    RELIABILITY -- how repeatable the hold pattern is -- and a policy that
    drifts consistently scores just as well as one that does not move. This
    measures the drift itself, in metres, and doubles as the gate on whether
    `hold` may be used as the origin (see `centre="hold"`): if the bike is
    wandering, "the zero-command state" is a moving target and every
    correlation involving it is contaminated.
    """
    from aow_sim.control.balance import extract_state
    obs, _ = env.reset(seed=7, options={"v_cmd": (0.0, 0.0),
                                        "psi_cmd_rel": 0.0, "difficulty": 1.0})
    V, A, ST = [], [], []
    for _ in range(steps):
        a = pol.action(obs)
        A.append(a)
        na = np.array([a[0] / pol.bounds.steer_rate_max,
                       a[1] / pol.bounds.hub_max,
                       a[2] / pol.bounds.diff_max])[:env.action_space.shape[0]]
        obs, *_ = env.step(na)
        s = extract_state(env.data, env._p0)
        V.append(np.hypot(s.v_lon, s.v_lat))
        ST.append(abs(float(env.data.qpos[env._sj])))
    s = extract_state(env.data, env._p0)
    A = np.array(A)
    return {"drift_speed": float(np.mean(V)),
            "net_displacement": float(np.hypot(s.e_lon, s.e_lat)),
            "steer_rate_abs": float(np.abs(A[:, 0]).mean()),
            "hub_mean": float(A[:, 1].mean()),
            "diff_mean": float(A[:, 2].mean()),
            "steer_deg": float(np.degrees(np.mean(ST)))}


def confusion(pol, env, conds, steps, centre="grand"):
    """Cross-validated move x move correlation matrix.

    `centre` picks the origin the patterns are measured from:
      "grand" -- subtract the mean across all moves. Arbitrary, and it makes
                 the origin depend on which moves happen to be in the set.
      "hold"  -- subtract the `hold` pattern, so every other move is read as a
                 deviation from the zero command, which is what the command
                 space actually says the origin is. `hold` then drops out of
                 the matrix (it is the reference). Only trustworthy when
                 hold_quality() shows the bike really is close to still.
    """
    A, B, labels = [], [], []
    for cond in conds:
        H = hidden(pol, trace(pol, env, cond, steps))
        half = len(H) // 2
        A.append(H[:half].mean(0))
        B.append(H[half:].mean(0))
        labels.append(cond[0])
    A, B = np.array(A), np.array(B)
    if centre == "hold" and "hold" in labels:
        h = labels.index("hold")
        A, B = A - A[h], B - B[h]
        keep = [i for i in range(len(labels)) if i != h]
        A, B = A[keep], B[keep]
        labels = [labels[i] for i in keep]
    else:
        # Centre across moves: removes the large common mode every command
        # shares (the "stay upright" component, ~60-70% of the variance).
        A -= A.mean(0)
        B -= B.mean(0)
    n = len(labels)
    C = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            a, b = A[i], B[j]
            c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
            C[i, j] = c
    return labels, (C + C.T) / 2          # symmetrize the cross-validation


def ideal_matrix(labels):
    """The target a well-formed general controller should produce.

    The five sustained primitives are five points in the 2-D velocity command
    space: hold at the origin, fwd/rev at +-x, crabL/crabR at +-y. Only some
    cells of the resulting matrix are pinned down by the plant; the rest are
    left NaN and excluded from scoring, because a target invented for them
    would measure the author's taste rather than the policy.

    PINNED, and why:

    1. DIAGONAL = 1. A sustained command is executed consistently, so its
       pattern is stable and split-half reliability is perfect. Anything less
       is execution inconsistency, not code structure.

    2. hold/fwd/rev vs crabL/crabR = 0, all six cells. This is DERIVED, not
       preferred. Under the sagittal mirror those three commands map to
       themselves while crabL <-> crabR. A mirror-equivariant policy induces
       an orthogonal map on hidden space, and orthogonal maps preserve inner
       products, so for any mirror-invariant X

           corr(X, crabL) = corr(M X, M crabL) = corr(X, crabR).

       The two must be EQUAL -- which is testable on its own, without any
       linearity assumption, and is what `symmetry_gap` measures. Combined
       with crabL = -crabR, equal-and-opposite forces both to zero.

    3. crabL vs crabR = -1. Symmetry gives the reflection relation; the -1
       additionally assumes the code is linear in the command. A nonlinear
       code could separate the two perfectly well without being antipodal, so
       read this as "is the lateral command one signed axis", not as a verdict.

    LEFT UNCONSTRAINED (NaN, shown grey, never scored):

      hold-fwd, hold-rev -- both commands are mirror-invariant, so symmetry is
        silent. Zero would follow only from hold's centred pattern being
        exactly the zero vector; that holds under an affine code but is
        numerically degenerate, since the correlation then divides by a
        vanishing norm and measures noise.

      fwd-rev -- the bike has a FRONT, and reversing inverts the caster that
        stabilises it, so forward and reverse are genuinely different
        dynamical regimes. An earlier version of this matrix demanded -1 here;
        that was the weakest claim in it and is now dropped rather than scored.
    """
    idx = {l: i for i, l in enumerate(labels)}
    n = len(labels)
    M = np.full((n, n), np.nan)          # NaN = the physics makes no claim
    np.fill_diagonal(M, 1.0)

    def put(a, b, v):
        if a in idx and b in idx:
            M[idx[a], idx[b]] = M[idx[b], idx[a]] = v

    # FORCED by mirror equivariance + the antipodal crab axis. Under the
    # sagittal mirror, hold/fwd/rev map to themselves while crabL <-> crabR,
    # and the induced hidden-space map is orthogonal, so
    #     corr(X, crabL) = corr(M X, M crabL) = corr(X, crabR)
    # for every mirror-invariant X. With crabL = -crabR that forces both to 0.
    for x in ("hold", "fwd", "rev"):
        put(x, "crabL", 0.0)
        put(x, "crabR", 0.0)
    # The lateral command as one signed axis. Symmetry gives the reflection
    # relation; -1 additionally assumes the code is linear in the command.
    put("crabL", "crabR", -1.0)

    # LEFT UNCONSTRAINED (NaN), because nothing licenses a target:
    #   hold-fwd, hold-rev -- both mirror-invariant, so symmetry is silent.
    #     Zero would follow only from hold's centred pattern being exactly the
    #     zero vector, which is true under an affine code but numerically
    #     degenerate: the correlation divides by a vanishing norm, so whatever
    #     is measured there is noise.
    #   fwd-rev -- the bike has a FRONT and reversing inverts the caster, so
    #     forward and reverse are different dynamical regimes. Demanding an
    #     exact sign flip was the weakest claim in the first version of this
    #     matrix, and it is dropped rather than scored.
    return M


def disattenuate(C):
    """Divide every off-diagonal by sqrt(rel_i * rel_j) -- the classical
    correction for attenuation by measurement noise.

    WHY IT MATTERS HERE, and not as a cosmetic adjustment. Two of the three
    scores below have a target of ZERO, and an unreliable pattern shrinks
    every off-diagonal toward zero. So a policy whose code is simply noisy
    scores a BETTER symmetry gap and a better cross-axis leakage than one
    whose code is clean, purely for being noisier. The reliabilities here
    range from 0.40 to 0.99 across policies, which is more than enough spread
    for that to reverse a ranking.

    Applied, it settled the question it was written for: the smoothed policy's
    symmetry gap (0.52 raw) is not an artefact of its unusually high
    reliability -- correcting moves it to 0.54 while general_rl_1k stays at
    0.17. The entanglement is real, not a de-attenuation effect. The raw
    numbers are still printed beside these, because disattenuation divides by
    a noisy estimate and can inflate wildly when reliability is near zero.

    The diagonal is left alone: it IS the reliability, and dividing it by
    itself would just print a column of ones.
    """
    r = np.clip(np.diag(C), 1e-3, None)
    D = C / np.sqrt(np.outer(r, r))
    np.fill_diagonal(D, np.diag(C))
    return D


def score_against_ideal(C, labels):
    """Three statistics, each tied to something the plant actually forces,
    instead of one correlation against a partly-invented target."""
    i = {l: n for n, l in enumerate(labels)}
    have = lambda *ls: all(l in i for l in ls)

    out = {}
    if have("hold", "fwd", "rev", "crabL", "crabR"):
        # 1. Symmetry violation: the purely symmetry-derived test, no
        #    linearity assumed. Target 0.
        gaps = [abs(C[i[x], i["crabL"]] - C[i[x], i["crabR"]])
                for x in ("hold", "fwd", "rev")]
        out["symmetry_gap"] = float(np.mean(gaps))
        # 2. Cross-axis leakage: longitudinal and lateral are independent
        #    degrees of freedom. Target 0.
        cross = [C[i[a], i[b]] for a in ("fwd", "rev")
                 for b in ("crabL", "crabR")]
        out["cross_axis_rms"] = float(np.sqrt(np.mean(np.square(cross))))
        # 3. The crab axis itself. Target -1.
        out["crab_axis"] = float(C[i["crabL"], i["crabR"]])
    out["min_reliability"] = float(np.diag(C).min())
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--set", default="sustained",
                    help="condition set (see rsa_policies.condition_sets); "
                         "'with_turns' reproduces the non-stationarity "
                         "counterexample described above")
    ap.add_argument("--steps", type=int, default=150)
    ap.add_argument("--out", type=Path, default=None,
                    help="default: analysis/move_confusion_<set>.png")
    args = ap.parse_args()
    if args.out is None:
        args.out = Path(__file__).parent / f"move_confusion_{args.set}.png"

    params = load_params()
    cfg = _load_rl_config(REPO / "config" / "rl_general.yaml")
    cfg = {**cfg, "randomization": {**cfg["randomization"], "enabled": False}}
    env = GeneralEnv(params, cfg)
    conds = condition_sets(cfg["env"]["v_max"])[args.set]

    mats = {"IDEAL": None}
    for key, short in POLICIES.items():
        pol = load_general(key)          # one env per policy width
        labels, C = confusion(pol, env_for(pol, params, cfg), conds,
                              args.steps)
        mats[short] = C
        if mats["IDEAL"] is None:
            mats["IDEAL"] = ideal_matrix(labels)
        print(f"\n{short}  ({args.set})")
        print("       " + "".join(f"{l:>8}" for l in labels))
        for i, l in enumerate(labels):
            print(f"{l:>6} " + "".join(f"{C[i, j]:>8.2f}" for j in range(len(labels))))

    w = max(len(s) for s in mats) + 2
    print("\nagainst the ideal  (only cells the plant actually constrains)")
    print("  each score twice: corrected for the noise ceiling, then (raw). "
          "A noisy code\n  scores an artificially GOOD symmetry gap and "
          "cross-axis -- see disattenuate().")
    print(f"  {'policy':{w}} {'symmetry gap':>17} {'cross-axis':>17} "
          f"{'crab axis':>17} {'min reliab':>11}")
    print(f"  {'target':{w}} {0.0:>17.2f} {0.0:>17.2f} {-1.0:>17.2f} "
          f"{1.0:>11.2f}")
    for short, C in mats.items():
        if short == "IDEAL":
            continue
        raw = score_against_ideal(C, labels)
        adj = score_against_ideal(disattenuate(C), labels)
        nan = float("nan")

        def pair(key):
            return (f"{adj.get(key, nan):>9.2f} "
                    f"{'(' + format(raw.get(key, nan), '.2f') + ')':>7}")

        print(f"  {short:{w}} {pair('symmetry_gap')} {pair('cross_axis_rms')} "
              f"{pair('crab_axis')} {raw['min_reliability']:>11.2f}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nno PNG: matplotlib is not installed in this interpreter")
        return

    n = len(mats)
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n, 4.9))
    for ax, (short, C) in zip(np.atleast_1d(axes), mats.items()):
        cmap = matplotlib.cm.get_cmap("RdBu_r").copy()
        cmap.set_bad("0.85")            # NaN = no claim made
        im = ax.imshow(C, cmap=cmap, vmin=-1, vmax=1)
        ax.set_xticks(range(len(labels)), labels, rotation=45, ha="right")
        ax.set_yticks(range(len(labels)), labels)
        ax.set_title(short)
        for i in range(len(labels)):
            for j in range(len(labels)):
                if np.isnan(C[i, j]):
                    ax.text(j, i, "—", ha="center", va="center",
                            fontsize=9, color="0.45")
                    continue
                ax.text(j, i, f"{C[i, j]:.2f}", ha="center", va="center",
                        fontsize=7,
                        color="white" if abs(C[i, j]) > 0.6 else "black")
    fig.colorbar(im, ax=list(np.atleast_1d(axes)), shrink=0.8,
                 label="pattern correlation (cross-validated)")
    fig.suptitle(f"general_rl move confusability — {args.set} conditions "
                 f"(diagonal = split-half reliability)")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
