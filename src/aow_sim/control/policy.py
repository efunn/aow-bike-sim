"""Dependency-free numpy replay of a trained RL policy.

Training (stable-baselines3 PPO, in train_flick_rl.py) exports the deterministic
policy — the MLP that maps normalized observation to the action-distribution
mean — as a plain `.npz`:
  n_layers, W0,b0,...  : Linear layers of the policy net + final action mean layer
  act                  : activation name for the hidden layers ("tanh")
  obs_mean, obs_var    : VecNormalize observation statistics (identity if unused)
  bounds               : [steer_rate_max, hub_max, diff_max]
A policy with memory (recurrent_ppo.py) also carries
  arch = "mingru", enc_W, enc_b, mem_W, mem_b : the encoder and MinGRU layer
in front of the same MLP, and replays as `MinGRUPolicy`.

Inference is a numpy forward pass — no torch, no gymnasium — so the sim/replay
machine needs nothing beyond the base install. The action is the distribution
mean, clipped to [-1, 1], then scaled by `flick_spec.scale_action`.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .flick_spec import ACT_DIM, OBS_DIM, ActionBounds, scale_action

_ACTIVATIONS = {
    "tanh": np.tanh,
    "relu": lambda x: np.maximum(x, 0.0),
}


class MLPPolicy:
    """A trained policy replayed with numpy. `kind == 'policy'` so DriveController
    can distinguish it from a `FlickTrajectory` (`kind == 'trajectory'`)."""

    kind = "policy"

    def __init__(self, layers, activation, obs_mean, obs_var, bounds: ActionBounds,
                 obs_clip: float = 10.0):
        self.layers = layers                       # list of (W, b), last is the mean
        self.activation = _ACTIVATIONS[activation]
        self.obs_mean = np.asarray(obs_mean, np.float64)
        self.obs_var = np.asarray(obs_var, np.float64)
        self.bounds = bounds
        self.obs_clip = float(obs_clip)            # VecNormalize clip_obs
        # 3 = full, 2 = feedforward, 4 = full + the general policy's optional
        # wing channel (see general_spec; ACT_DIM stays 3 for the moves).
        self.act_dim = self.layers[-1][0].shape[0]
        self.obs_dim = self._input_width()           # flick=10, ball=14, etc.
        # Consistency check against the normalization stats, not a fixed OBS_DIM,
        # so the same replay serves any move's observation length.
        assert self.obs_dim == self.obs_mean.shape[0], "policy input != obs stats"
        assert self.act_dim in (2, ACT_DIM, ACT_DIM + 1), (
            f"policy output dim {self.act_dim} is not 2 (feedforward), "
            f"{ACT_DIM} (full) or {ACT_DIM + 1} (full + wing channel)")

    def _input_width(self) -> int:
        return self.layers[0][0].shape[1]

    def reset(self) -> None:
        """Episode start. Nothing to forget: `action` is a pure function of
        the observation. `MinGRUPolicy` overrides this, so a caller that
        replays either kind calls it at every episode start / engage."""

    def peek(self, obs):
        """`action(obs)` WITHOUT advancing any memory: a what-if query (the
        same step with a doctored observation, say) beside the real one. For
        this stateless MLP it is just `action`."""
        return self.action(obs)

    def action(self, obs):
        """obs (OBS_DIM,) -> (steer_rate, hub, diff[, wing_rate]). The return
        arity follows the policy's own output width, so a move policy always
        unpacks three. Mirrors VecNormalize +
        the SB3 policy: normalize+clip obs, MLP forward, deterministic mean,
        clip to the action box, then scale to physical units."""
        return scale_action(np.clip(self.mean(obs), -1.0, 1.0), self.bounds)

    def mean(self, obs) -> np.ndarray:
        """The network's deterministic mean BEFORE the clip to [-1, 1], in
        units of each channel's bound. `action` is exactly this, clipped and
        scaled. Kept separate because SB3's Gaussian is unsquashed, so the
        mean can sit well past the bound and the clip hides by how much --
        `analysis/policy/chatter.py` reports it."""
        return self._head(self._normalize(obs))

    def _normalize(self, obs) -> np.ndarray:
        obs = np.asarray(obs, np.float64)
        if obs.shape[0] != self.obs_dim:
            raise ValueError(
                f"obs length {obs.shape[0]} != policy obs_dim {self.obs_dim}: "
                "the observation spec changed since this move was trained — "
                "retrain/re-export it")
        x = (obs - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        return np.clip(x, -self.obs_clip, self.obs_clip)

    def _head(self, x) -> np.ndarray:
        for W, b in self.layers[:-1]:
            x = self.activation(W @ x + b)
        W, b = self.layers[-1]
        return W @ x + b


def _sigmoid(x):
    return 0.5 * (1.0 + np.tanh(0.5 * x))     # no overflow for large |x|


class MinGRUPolicy(MLPPolicy):
    """A policy with MEMORY: obs -> encoder -> MinGRU -> the same MLP head.
    recurrent_ppo.py has the layer's equations; this is them in numpy.

    STATEFUL. Each `action(obs)` or `mean(obs)` call is ONE control step and
    advances the memory, so call exactly one of them per step, at the rate
    the policy trained at, and `reset()` at every episode start or engage.
    A missed reset carries the last episode's memory into the next one; a
    second call on the same step advances the memory twice. Neither raises."""

    def __init__(self, enc, mem, layers, activation, obs_mean, obs_var,
                 bounds: ActionBounds, obs_clip: float = 10.0):
        self.enc = (np.asarray(enc[0], np.float64), np.asarray(enc[1], np.float64))
        self.mem = (np.asarray(mem[0], np.float64), np.asarray(mem[1], np.float64))
        super().__init__(layers, activation, obs_mean, obs_var, bounds, obs_clip)
        self.hidden = self.enc[0].shape[0]
        self.h = np.zeros(self.hidden)

    def _input_width(self) -> int:
        return self.enc[0].shape[1]

    def reset(self) -> None:
        self.h = np.zeros(self.hidden)

    def peek(self, obs):
        h = self.h
        try:
            return self.action(obs)
        finally:
            self.h = h

    def mean(self, obs) -> np.ndarray:
        x = self.enc[0] @ self._normalize(obs) + self.enc[1]
        c, g, p = np.split(self.mem[0] @ x + self.mem[1], 3)
        z = _sigmoid(g)
        h_tilde = np.where(c >= 0, c + 0.5, _sigmoid(c))
        self.h = (1.0 - z) * self.h + z * h_tilde
        s = _sigmoid(p)
        return self._head(s * self.h + (1.0 - s) * x)


def load_policy_npz(path: Path | str) -> MLPPolicy:
    d = np.load(path)
    n = int(d["n_layers"])
    layers = [(d[f"W{i}"], d[f"b{i}"]) for i in range(n)]
    activation = str(d["act"]) if "act" in d else "tanh"
    obs_mean = d["obs_mean"] if "obs_mean" in d else np.zeros(OBS_DIM)
    obs_var = d["obs_var"] if "obs_var" in d else np.ones(OBS_DIM)
    obs_clip = float(d["obs_clip"]) if "obs_clip" in d else 10.0
    bounds = ActionBounds.from_list(d["bounds"])
    if "arch" in d and str(d["arch"]) == "mingru":
        return MinGRUPolicy((d["enc_W"], d["enc_b"]), (d["mem_W"], d["mem_b"]),
                            layers, activation, obs_mean, obs_var, bounds, obs_clip)
    return MLPPolicy(layers, activation, obs_mean, obs_var, bounds, obs_clip)


def save_policy_npz(path: Path | str, layers, activation, obs_mean, obs_var,
                    bounds: ActionBounds, obs_clip: float = 10.0,
                    memory=None) -> None:
    """Used by the training exporter (kept here so save/load stay in sync).
    `memory` = ((enc_W, enc_b), (mem_W, mem_b)) writes a MinGRUPolicy."""
    arrs = {"n_layers": np.array(len(layers)), "act": np.array(activation),
            "obs_mean": np.asarray(obs_mean, np.float32),
            "obs_var": np.asarray(obs_var, np.float32),
            "obs_clip": np.array(obs_clip, np.float32),
            "bounds": np.asarray(bounds.to_list(), np.float32)}
    for i, (W, b) in enumerate(layers):
        arrs[f"W{i}"] = np.asarray(W, np.float32)
        arrs[f"b{i}"] = np.asarray(b, np.float32)
    if memory is not None:
        (eW, eb), (mW, mb) = memory
        arrs.update(arch=np.array("mingru"),
                    enc_W=np.asarray(eW, np.float32), enc_b=np.asarray(eb, np.float32),
                    mem_W=np.asarray(mW, np.float32), mem_b=np.asarray(mb, np.float32))
    np.savez(path, **arrs)
