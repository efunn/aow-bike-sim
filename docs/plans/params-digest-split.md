# Splitting `params_digest` — one hash, two questions

Started 2026-08-25. Status: **proposed, not implemented.** Written because the
problem was found while auditing something else, and the fix is small but the
reasoning behind it is not obvious from the code.

---

## The symptom that started it

`control.general_move` names which trained policy teleop drives with. It has
pointed at `general_rl`, which cannot reverse, for four status snapshots, and
`CLAUDE.md` called changing it "a one-line fix" every time.

It is not a one-line fix. `params_digest` hashes the whole parameter file, so
editing a **policy pointer** moves the digest:

    aa232834f462a229  ->  6d7c7f61025721a1     (measured 2026-08-25)

which re-breaks the deploy bundle and knocks every digest-matching policy out
of match, for a change with no physical content whatsoever.

## What the digest actually covers

`params_digest(params)` is `sha256(json.dumps(whole file, sort_keys))[:16]` --
**176 leaf values under 11 top-level keys**, i.e. everything `load_params`
returns after `{value, source}` unwrapping and `derive_righting`.

Two things enforce it, at different severities:

| artifact | stamped by | checked by | on mismatch |
|---|---|---|---|
| `deploy/bundle.npz` | `export_deploy.py:107` | `hw/state.py:123` | **raises** |
| `moves/*.yaml` | the four `train_*_rl.py` | `control/flick.py:228` | warns |

## The mismatch, measured

`control` is 44 of the 176 leaves. What actually reads them:

- **The deploy bundle reads 12** -- `control.rate_hz`, the ten `control.lqr.*`
  weights, and `control.drive.speed_grid` (`linearize.py:145,271,284`).
- **Trained policies read ZERO.** Nothing under `train_*.py` or `*_env.py`
  touches `params["control"]` at all; the env takes `control_rate_hz` from the
  RL config's `cfg["env"]`, not from `bike_params.yaml`.

So **32 control leaves are read by nothing that carries a digest**: the six PD
gains, fourteen analytic-drive knobs, six flip and four pivot params,
`moves_dir`, and `general_move`. Perturbing exactly those 32 and nothing else
moves the digest `aa232834f462a229 -> 53a164baab19bb86`, invalidating the
bundle and all 39 policy exports.

`general_move` is not one awkward field. It is a third of the `control` block.

And policies come off worse than the bundle: they are judged against a hash
covering 44 fields they cannot read.

## This has already been worked around twice

Neither is filed as a digest problem, but both are:

- **`run_drive.py:965`** -- ball-throw speed kept OUT of `bike_params.yaml`
  because "adding it to the params would move `params_digest` -- and so
  invalidate the deploy bundle and every trained policy -- for a diagnostic."
- **`export_deploy.py:57`** -- the AHRS mount quaternion, "deliberately NOT
  part of `bike_params.yaml`, and so not part of `params_digest`... making it
  invalidate both on every re-seat of the sensor would be a pure cost."

Both solved it by **exiling data from the file**. That means file membership is
being decided by the hash rather than by what the data is, which is backwards.

## The bundle is not one artifact

`load_bundle` returns two things, and they do not share a dependency set:

| half | contents | depends on |
|---|---|---|
| `LQRDesign` | `K`, `Ks`, `speeds`, `fit_r2` | plant **+** the 12 control fields |
| `DeployModel` | `nq/nv/nu`, actuator ids and ranges, joint addresses, `qpos_eq` | the plant only |

`DeployModel` exists because the Pi has no MuJoCo. **`run_bike.py:143` loads the
bundle unconditionally**, before any controller is chosen, because the RL path
needs `DeployModel` even though it never touches `K`. So today a stale LQR gain
schedule can refuse to let an RL policy run.

That matters now that RL is the intended primary controller and LQR is a
reference baseline to be tried on hardware later.

## Proposal

Two digests, one per question:

- **`plant_digest`** -- every top-level key EXCEPT `control`. The question "was
  this trained/derived against the bike I am now running?" Stamped into
  `moves/*.yaml` and into the bundle for the `DeployModel` half.
- **`design_digest`** -- `plant_digest` plus the 12 fields the LQR design reads
  (`control.rate_hz`, `control.lqr.*`, `control.drive.speed_grid`). The
  question "were these gains designed against the weights I am now running?"

Severity should follow the question:

- `plant_digest` mismatch on the bundle -> **raise**, as today. Wrong actuator
  ids or joint addresses is a fall, not a warning.
- `design_digest` mismatch -> **warn**, or raise only when LQR is actually
  engaged. It cannot hurt an RL run, so it must not be able to stop one.
- `plant_digest` mismatch on a policy -> warn, as today.

### Consequences worth having

- `general_move` becomes genuinely a one-line change.
- Ball-throw speed and the AHRS mount can come home into the params file,
  under a group covered by neither digest.
- PD gains stop invalidating trained policies.

### Migration

Cheap, because nothing needs backfilling. The user's call, 2026-08-25: the 39
existing exports **should keep warning** -- only four or five are current, and
more will be trained. So:

- new exports stamp `plant_digest` and `design_digest`;
- old exports carry only the legacy `params_digest` field and warn forever;
- nothing tries to recompute a legacy digest under the new scheme.

The failure mode to avoid is a migration that silently blesses artifacts which
should warn -- that is the one thing this mechanism exists to prevent. Not
backfilling avoids it entirely.

### The actual prize: diagnosability

Today a mismatch prints two hex strings and the reader has no idea what moved.
With subtrees it can say **which group** changed -- "plant unchanged,
`control.lqr` moved" tells you your policy is fine and only the gains are
stale. Worth reporting the differing subtree names, not just a boolean.

## Open questions

1. Where do `control.flip.*` and `control.pivot.*` belong? They parameterise
   open-loop moves. Nothing carrying a digest reads them today, but a replayed
   move arguably depends on them.
2. Should `design_digest` include `plant_digest`, or be independent? Included
   is simpler to reason about; independent makes the report sharper.
3. Does anything else read `params["control"]` that this survey missed? The
   survey was a grep over `train_*.py`, `*_env.py`, `export_deploy.py` and
   `linearize.py`. Re-run it before implementing.

## What this does NOT fix

`bike_params_cad.yaml` near-duplicating `bike_params.yaml` -- 163 shared leaves,
of which 9 had drifted as of 2026-08-25. That is a separate problem with a
separate fix; see the reconciliation notes in `docs/status.md`.
