"""A timed command sequence for the general policy: a "play".

docs/plans/wishlist.md, "ball shots": a shot is the general policy driven
through a preset heading/velocity trajectory, not a policy of its own. This
is that trajectory, and nothing ball-specific is in it.

A play is keyframes of the SAME command teleop sends from the keyboard
(`DriveController.set_command_polar`):

    t       [s] from the start of the play
    v       [m/s] along the commanded heading (+ forward)
    v_lat   [m/s] across it (+ left: a crab)
    psi     [deg] commanded heading, RELATIVE to the bike's heading when the
            play started (+ left)

held from one keyframe to the next (`interp: step`, the default -- what a key
press does) or ramped (`interp: linear`). After the last keyframe its
command stands; a play that should stop says so in its last row.

Two optional extras:

    crank   a FIFTH column [deg]: the righting crank's goal (+ puts the
            RIGHT blade down, 116.9 is the full stroke). A play without it
            leaves the crank to the operator; with it, its last row stands
            too, so fold the blade in the last row.
    ball    `ball: [x, y]` [m], where to park the --hockey ball when the
            play starts, in the bike's frame (x forward, y left). Keep it
            CLOSE: the bike reaches 1.2 m/s in 0.3 s and 0.25 m, and a
            standing bike's heading wanders a few degrees -- 10-15 cm at a
            ball 3 m out (teleop, 2026-10-16), ~2 cm at 0.6 m.

    # config/plays/ram.yaml
    interp: step
    ball: [0.6, 0.0]
    keys:
      - [0.0, 1.2, 0.0, 0]     # straight at it
      - [0.8, 0.0, 0.0, 0]     # stop

Teleop: `--play config/plays/<name>.yaml` (repeatable), End starts the next
one, any driving key aborts. `analysis/ball_play.py` flies one headless.
Pure numpy + yaml, so it can run anywhere the policy does.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Play:
    name: str
    t: np.ndarray          # (n,) s, strictly increasing, t[0] == 0
    cmd: np.ndarray        # (n, 3): v, v_lat [m/s], psi [rad] relative
    interp: str = "step"
    crank: np.ndarray | None = None    # (n,) rad, or None: not driven
    ball: tuple | None = None          # (x, y) m, bike frame at the start

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    def at(self, t: float) -> tuple[float, float, float]:
        """(v, v_lat, psi_rel [rad]) at `t` s into the play."""
        if self.interp == "linear":
            return tuple(float(np.interp(t, self.t, self.cmd[:, k])) for k in range(3))
        i = int(np.searchsorted(self.t, t, side="right")) - 1
        return tuple(float(x) for x in self.cmd[max(i, 0)])

    def crank_at(self, t: float) -> float | None:
        """Crank goal [rad] at `t`, or None if the play does not drive it."""
        if self.crank is None:
            return None
        if self.interp == "linear":
            return float(np.interp(t, self.t, self.crank))
        i = int(np.searchsorted(self.t, t, side="right")) - 1
        return float(self.crank[max(i, 0)])

    @classmethod
    def from_keys(cls, keys, name: str = "play", interp: str = "step",
                  ball=None) -> "Play":
        k = np.asarray(keys, dtype=float)
        if k.ndim != 2 or k.shape[1] not in (4, 5):
            raise ValueError(f"{name}: each key is [t, v, v_lat, psi_deg] or "
                             f"[t, v, v_lat, psi_deg, crank_deg], got shape {k.shape}")
        if k[0, 0] != 0.0 or np.any(np.diff(k[:, 0]) <= 0):
            raise ValueError(f"{name}: times must start at 0 and increase")
        if interp not in ("step", "linear"):
            raise ValueError(f"{name}: interp is 'step' or 'linear', got {interp!r}")
        cmd = k[:, 1:4].copy()
        cmd[:, 2] = np.deg2rad(cmd[:, 2])
        crank = np.deg2rad(k[:, 4]) if k.shape[1] == 5 else None
        if ball is not None:
            ball = tuple(float(x) for x in ball)
            if len(ball) != 2:
                raise ValueError(f"{name}: ball is [x, y]")
        return cls(name, k[:, 0].copy(), cmd, interp, crank, ball)

    def ball_world(self, pos, heading: float):
        """World (x, y) of the ball for a bike at `pos` [m] facing `heading`."""
        c, s = np.cos(heading), np.sin(heading)
        bx, by = self.ball
        return (float(pos[0] + c * bx - s * by), float(pos[1] + s * bx + c * by))


def load_play(path: str | Path) -> Play:
    import yaml
    path = Path(path)
    d = yaml.safe_load(path.read_text())
    return Play.from_keys(d["keys"], name=path.stem, interp=d.get("interp", "step"),
                          ball=d.get("ball"))
