# Retired plans

Design records whose work is finished, superseded, or was never started. They
are kept because the *reasoning* in them is still worth reading — the control
mapping, the linearization argument, the first-principles architecture — not
because anything here is pending.

Each file opens with a status banner saying which of the three it is and what
closed it. Nothing in this folder is a queue.

| doc | why it is here |
|---|---|
| `HANDOFF-2026-08-28.md` | superseded — `docs/status.md` is the handoff now |
| `stationary-balance-controller.md` | built 2026-07-18 (`control/balance.py`, `linearize.py`, `run_balance.py`) |
| `pivot-controller.md` | built 2026-07-18 (`control/pivot.py`, `run_pivot.py`, later re-authored as RL) |
| `prelim-architecture.md` | superseded by `mujoco-modeling-decisions.md` |
| `sharper-turns-stage-1.md` | superseded — `drive.py` is the LQR baseline; RL drives |
| `agility-turn-180-move.md` | concept only, never implemented, nothing blocked on it |

Retired 2026-09-08 in a docs triage. If one of these becomes live again, move it
back up a level and replace the banner rather than editing around it.
