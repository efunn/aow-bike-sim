"""PPO with smoothness terms on the POLICY'S MEAN, added to the training loss.

docs/plans/policy-smoothness-losses.md has the why: the deployed action is the
network's deterministic mean, and a reward term (`w_smooth`) only reaches it
through noisy returns on SAMPLED actions. These terms act on the mean directly.

Three terms, each per sample and summed over action channels, μ in units of
each channel's bound (what the network emits before the clip to [-1, 1]):

    bound       relu(|μ| - 1)^2               the mean kept inside the box
    temporal    (μ_{t+1} - μ_t)^2             CAPS's temporal term
    second      (μ_{t+1} - 2 μ_t + μ_{t-1})^2  Grad-CAPS: a zig-zag costs, a
                                              steady ramp is free

configured as `algo.smooth: {bound: λ, temporal: λ, second: λ}`, absent keys
meaning 0. Only the policy's mean is regularised; the value net never sees
these terms (SB3's MlpPolicy keeps separate actor and critic networks here).

WRITTEN AGAINST stable-baselines3 2.9.0. SB3 has no hook for an extra loss
term -- the whole loss is one line of PPO.train() -- so `train` below is a
copy of upstream's with three additions, each marked `# SMOOTH`. The guard is
tests/test_smooth_ppo.py: with every λ at 0, one update is bit-identical to
stock PPO. An SB3 upgrade that changes the original fails that test.

TEMPORAL WINDOWS come from the rollout buffer BEFORE it is shuffled:
`rollout_buffer.get()` flattens it in place and destroys which step followed
which. At that point `observations` is (n_steps, n_envs, obs_dim) and already
VecNormalize-normalised -- exactly what the policy saw. A window is valid only
inside one episode (`episode_starts`).
"""

from __future__ import annotations

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.utils import explained_variance
from torch.nn import functional as F

TERMS = ("bound", "temporal", "second")


def parse_smooth(cfg) -> dict:
    """`algo.smooth` -> {term: λ} with every term present. A misspelt key is
    an error, not a silent zero: a typo would otherwise train an arm with the
    term it was meant to test switched off."""
    cfg = dict(cfg or {})
    bad = set(cfg) - set(TERMS)
    if bad:
        raise ValueError(f"algo.smooth: unknown term(s) {sorted(bad)}; "
                         f"known: {list(TERMS)}")
    out = {k: float(cfg.get(k, 0.0)) for k in TERMS}
    if any(v < 0 for v in out.values()):
        raise ValueError(f"algo.smooth: weights must be >= 0, got {out}")
    return out


def bound_term(mu: th.Tensor) -> th.Tensor:
    """Per sample: sum over channels of relu(|μ| - 1)^2."""
    return th.relu(mu.abs() - 1.0).pow(2).sum(-1)


def temporal_term(mu0: th.Tensor, mu1: th.Tensor) -> th.Tensor:
    """Per window: sum over channels of (μ_{t+1} - μ_t)^2."""
    return (mu1 - mu0).pow(2).sum(-1)


def second_term(mu0: th.Tensor, mu1: th.Tensor, mu2: th.Tensor) -> th.Tensor:
    """Per window: sum over channels of (μ_{t+1} - 2 μ_t + μ_{t-1})^2.
    Zero on any straight line in time; 16 per channel on a ±1 zig-zag, against
    the temporal term's 4."""
    return (mu2 - 2.0 * mu1 + mu0).pow(2).sum(-1)


def windows(episode_starts: np.ndarray, k: int) -> tuple[np.ndarray, np.ndarray]:
    """(t, env) index arrays of every length-`k` run of consecutive steps that
    stays inside one episode. `episode_starts[t, e]` is 1 where step t is the
    first of a new episode, so steps t..t+k-1 belong together iff none of
    t+1..t+k-1 starts one."""
    T = episode_starts.shape[0]
    if T < k:
        return np.zeros(0, int), np.zeros(0, int)
    ok = np.ones((T - k + 1, episode_starts.shape[1]), bool)
    for j in range(1, k):
        ok &= episode_starts[j:T - k + 1 + j] == 0
    return np.nonzero(ok)


class SmoothPPO(PPO):
    """PPO whose loss also carries `smooth` terms on the policy mean."""

    def __init__(self, *args, smooth=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.smooth = parse_smooth(smooth)

    def _mu(self, obs: th.Tensor) -> th.Tensor:
        """Pre-clip deterministic mean: what MLPPolicy.mean replays."""
        return self.policy.get_distribution(obs).distribution.mean

    def _window_obs(self):
        """Per active temporal term, the buffer's valid windows as a tensor
        (n_windows, k, obs_dim). Built once per train(), before get()."""
        rb = self.rollout_buffer
        out = {}
        for name, k in (("temporal", 2), ("second", 3)):
            if not self.smooth[name]:
                continue
            t, e = windows(rb.episode_starts, k)
            obs = np.stack([rb.observations[t + j, e] for j in range(k)], 1)
            out[name] = th.as_tensor(obs, device=self.device)
        return out

    def _smooth_loss(self, obs: th.Tensor, win: dict, n: int):
        """(weighted loss, {term: unweighted mean}) for one minibatch. Temporal
        windows are drawn at random, `n` of them, so each term sees as many
        samples as the PPO minibatch."""
        s, loss, parts = self.smooth, 0.0, {}
        if s["bound"]:
            v = bound_term(self._mu(obs)).mean()
            loss, parts["bound"] = loss + s["bound"] * v, v.item()
        for name in ("temporal", "second"):
            if name not in win or len(win[name]) == 0:
                continue
            w = win[name][th.randint(len(win[name]), (n,), device=self.device)]
            mus = [self._mu(w[:, j]) for j in range(w.shape[1])]
            v = (temporal_term(*mus) if name == "temporal"
                 else second_term(*mus)).mean()
            loss, parts[name] = loss + s[name] * v, v.item()
        return loss, parts

    def train(self) -> None:
        """Upstream PPO.train (SB3 2.9.0) plus the `# SMOOTH` lines."""
        # Switch to train mode (this affects batch norm / dropout)
        self.policy.set_training_mode(True)
        # Update optimizer learning rate
        self._update_learning_rate(self.policy.optimizer)
        # Compute current clip range
        clip_range = self.clip_range(self._current_progress_remaining)  # type: ignore[operator]
        # Optional: clip range for the value function
        if self.clip_range_vf is not None:
            clip_range_vf = self.clip_range_vf(self._current_progress_remaining)  # type: ignore[operator]

        entropy_losses = []
        pg_losses, value_losses = [], []
        clip_fractions = []
        smooth_on = any(self.smooth.values())                       # SMOOTH
        win = self._window_obs() if smooth_on else {}               # SMOOTH
        smooth_parts = {k: [] for k in TERMS}                       # SMOOTH

        continue_training = True
        # train for n_epochs epochs
        for epoch in range(self.n_epochs):
            approx_kl_divs = []
            # Do a complete pass on the rollout buffer
            for rollout_data in self.rollout_buffer.get(self.batch_size):
                actions = rollout_data.actions
                if isinstance(self.action_space, spaces.Discrete):
                    # Convert discrete action from float to long
                    actions = rollout_data.actions.long().flatten()

                values, log_prob, entropy = self.policy.evaluate_actions(rollout_data.observations, actions)
                values = values.flatten()
                # Normalize advantage
                advantages = rollout_data.advantages
                # Normalization does not make sense if mini batchsize == 1, see GH issue #325
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                # ratio between old and new policy, should be one at the first iteration
                ratio = th.exp(log_prob - rollout_data.old_log_prob)

                # clipped surrogate loss
                policy_loss_1 = advantages * ratio
                policy_loss_2 = advantages * th.clamp(ratio, 1 - clip_range, 1 + clip_range)
                policy_loss = -th.min(policy_loss_1, policy_loss_2).mean()

                # Logging
                pg_losses.append(policy_loss.item())
                clip_fraction = th.mean((th.abs(ratio - 1) > clip_range).float()).item()
                clip_fractions.append(clip_fraction)

                if self.clip_range_vf is None:
                    # No clipping
                    values_pred = values
                else:
                    # Clip the difference between old and new value
                    # NOTE: this depends on the reward scaling
                    values_pred = rollout_data.old_values + th.clamp(
                        values - rollout_data.old_values, -clip_range_vf, clip_range_vf
                    )
                # Value loss using the TD(gae_lambda) target
                value_loss = F.mse_loss(rollout_data.returns, values_pred)
                value_losses.append(value_loss.item())

                # Entropy loss favor exploration
                if entropy is None:
                    # Approximate entropy when no analytical form
                    entropy_loss = -th.mean(-log_prob)
                else:
                    entropy_loss = -th.mean(entropy)

                entropy_losses.append(entropy_loss.item())

                loss = policy_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss

                if smooth_on:                                       # SMOOTH
                    s_loss, parts = self._smooth_loss(
                        rollout_data.observations, win, len(advantages))
                    loss = loss + s_loss
                    for k, v in parts.items():
                        smooth_parts[k].append(v)

                # Calculate approximate form of reverse KL Divergence for early stopping
                # see issue #417: https://github.com/DLR-RM/stable-baselines3/issues/417
                # and discussion in PR #419: https://github.com/DLR-RM/stable-baselines3/pull/419
                # and Schulman blog: http://joschu.net/blog/kl-approx.html
                with th.no_grad():
                    log_ratio = log_prob - rollout_data.old_log_prob
                    approx_kl_div = th.mean((th.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                    approx_kl_divs.append(approx_kl_div)

                if self.target_kl is not None and approx_kl_div > 1.5 * self.target_kl:
                    continue_training = False
                    if self.verbose >= 1:
                        print(f"Early stopping at step {epoch} due to reaching max kl: {approx_kl_div:.2f}")
                    break

                # Optimization step
                self.policy.optimizer.zero_grad()
                loss.backward()
                # Clip grad norm
                th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()

            self._n_updates += 1
            if not continue_training:
                break

        explained_var = explained_variance(self.rollout_buffer.values.flatten(), self.rollout_buffer.returns.flatten())

        # Logs
        self.logger.record("train/entropy_loss", np.mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", np.mean(pg_losses))
        self.logger.record("train/value_loss", np.mean(value_losses))
        self.logger.record("train/approx_kl", np.mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", np.mean(clip_fractions))
        self.logger.record("train/loss", loss.item())
        self.logger.record("train/explained_variance", explained_var)
        if hasattr(self.policy, "log_std"):
            self.logger.record("train/std", th.exp(self.policy.log_std).mean().item())

        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/clip_range", clip_range)
        if self.clip_range_vf is not None:
            self.logger.record("train/clip_range_vf", clip_range_vf)
        for k, v in smooth_parts.items():                           # SMOOTH
            if v:  # unweighted term values: comparable across arms and λ
                self.logger.record(f"train/smooth_{k}", np.mean(v))
