"""Dependency-free numpy replay of a trained RL policy.

Training (stable-baselines3 PPO, in train_flick_rl.py) exports the deterministic
policy — the MLP that maps normalized observation to the action-distribution
mean — as a plain `.npz`:
  n_layers, W0,b0,...  : Linear layers of the policy net + final action mean layer
  act                  : activation name for the hidden layers ("tanh")
  obs_mean, obs_var    : VecNormalize observation statistics (identity if unused)
  bounds               : [steer_rate_max, hub_max, diff_max]

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
        self.act_dim = self.layers[-1][0].shape[0]   # 3 = full, 2 = feedforward
        self.obs_dim = self.layers[0][0].shape[1]    # flick=10, ball=14, etc.
        # Consistency check against the normalization stats, not a fixed OBS_DIM,
        # so the same replay serves any move's observation length.
        assert self.obs_dim == self.obs_mean.shape[0], "policy input != obs stats"
        assert self.act_dim in (2, ACT_DIM), "policy output dim must be 2 or 3"

    def action(self, obs) -> tuple[float, float, float]:
        """obs (OBS_DIM,) -> (steer_rate, hub, diff). Mirrors VecNormalize +
        the SB3 policy: normalize+clip obs, MLP forward, deterministic mean,
        clip to the action box, then scale to physical units."""
        x = (np.asarray(obs, np.float64) - self.obs_mean) / np.sqrt(self.obs_var + 1e-8)
        x = np.clip(x, -self.obs_clip, self.obs_clip)
        for W, b in self.layers[:-1]:
            x = self.activation(W @ x + b)
        W, b = self.layers[-1]
        mean = np.clip(W @ x + b, -1.0, 1.0)
        return scale_action(mean, self.bounds)


def load_policy_npz(path: Path | str) -> MLPPolicy:
    d = np.load(path)
    n = int(d["n_layers"])
    layers = [(d[f"W{i}"], d[f"b{i}"]) for i in range(n)]
    activation = str(d["act"]) if "act" in d else "tanh"
    obs_mean = d["obs_mean"] if "obs_mean" in d else np.zeros(OBS_DIM)
    obs_var = d["obs_var"] if "obs_var" in d else np.ones(OBS_DIM)
    obs_clip = float(d["obs_clip"]) if "obs_clip" in d else 10.0
    bounds = ActionBounds.from_list(d["bounds"])
    return MLPPolicy(layers, activation, obs_mean, obs_var, bounds, obs_clip)


def save_policy_npz(path: Path | str, layers, activation, obs_mean, obs_var,
                    bounds: ActionBounds, obs_clip: float = 10.0) -> None:
    """Used by the training exporter (kept here so save/load stay in sync)."""
    arrs = {"n_layers": np.array(len(layers)), "act": np.array(activation),
            "obs_mean": np.asarray(obs_mean, np.float32),
            "obs_var": np.asarray(obs_var, np.float32),
            "obs_clip": np.array(obs_clip, np.float32),
            "bounds": np.asarray(bounds.to_list(), np.float32)}
    for i, (W, b) in enumerate(layers):
        arrs[f"W{i}"] = np.asarray(W, np.float32)
        arrs[f"b{i}"] = np.asarray(b, np.float32)
    np.savez(path, **arrs)
