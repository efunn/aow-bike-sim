"""SmoothPPO with MEMORY in the actor: one MinGRU layer in front of its MLP.

The CPU test of the network PufferLib 5.0 defaults to (docs/plans/
gpu-training.md, phase 4), before any GPU work: does memory help this bike
at all? Everything else is today's recipe -- the same env, VecNormalize, the
same PPO loss and the same smoothness terms (smooth_ppo.py).

    actor   obs -> Linear(obs, H) -> MinGRU(H) -> MLP net_arch (tanh) -> mean
    critic  obs -> MLP net_arch (tanh) -> value          (exactly as MlpPolicy)

The critic stays stateless on purpose: one change at a time. It is also what
keeps the truncation bootstrap and the rollout's last value exactly SB3's.

THE LAYER, as PufferLib's (`src/algo.cu` @ ea7f989), per unit:

    [c, g, p] = W x + b                   (H -> 3H, the input ONLY)
    z  = sigmoid(g)                       how much of the new candidate to take
    h~ = c + 0.5 if c >= 0 else sigmoid(c)
    h  = (1 - z) h_prev + z h~            an input-gated moving average
    out = sigmoid(p) h + (1 - sigmoid(p)) x    highway: the input passes too

`h` (H numbers per env) is the whole state. It is zeroed at every episode
start: in a rollout, in training, and on replay (`MinGRUPolicy.reset`).

TRAINING ON STRETCHES. A stateless policy can be trained on any shuffled set
of steps; a recurrent one has to be re-run over consecutive steps from the
state it actually had. So each env's `n_steps` rollout is cut into
`bptt`-step chunks, the state at each chunk's start is taken from the
rollout (stored per step), and a minibatch is `batch_size / bptt` whole
chunks drawn at random. Gradients flow back at most `bptt` steps; the state
itself carries further. Steps inside a chunk are correlated, so a minibatch
of 32 chunks x 32 steps is noisier than 1024 independent steps -- the price
of recurrence, and part of what the experiment measures.

THE SMOOTHNESS TERMS come out of the same forward pass here: a chunk already
holds the mean at consecutive steps, so `temporal` and `second` are taken
over neighbouring steps inside a chunk (never across an episode start),
rather than over separately sampled windows as SmoothPPO does.

Stored state goes stale after the first epoch's updates (the stored-state
approximation every recurrent PPO makes, sb3-contrib's and PufferLib's
included). At epoch 0, before any update, the recomputed log-probs equal
the rollout's: tests/test_recurrent_ppo.py checks exactly that.
"""

from __future__ import annotations

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import MlpExtractor
from stable_baselines3.common.utils import explained_variance, obs_as_tensor
from torch import nn
from torch.nn import functional as F

from .smooth_ppo import TERMS, SmoothPPO, bound_term, second_term, temporal_term


def parse_memory(cfg) -> dict | None:
    """`algo.memory` -> {hidden, bptt}, or None for the stateless MLP. A
    misspelt key is an error, as in parse_smooth."""
    if not cfg:
        return None
    cfg = dict(cfg)
    known = {"hidden", "bptt"}
    bad = set(cfg) - known
    if bad:
        raise ValueError(f"algo.memory: unknown key(s) {sorted(bad)}; "
                         f"known: {sorted(known)}")
    return {"hidden": int(cfg.get("hidden", 128)), "bptt": int(cfg.get("bptt", 32))}


class MinGRU(nn.Module):
    """One MinGRU layer with highway, H -> H. See the module docstring."""

    def __init__(self, hidden: int):
        super().__init__()
        self.hidden = hidden
        self.lin = nn.Linear(hidden, 3 * hidden)

    def forward(self, x: th.Tensor, h0: th.Tensor, starts: th.Tensor):
        """x (T, N, H), h0 (N, H), starts (T, N): 1 where step t begins an
        episode, which zeroes the state before that step. Returns the outputs
        (T, N, H), the states after each step (T, N, H), and the mean gate z
        (a diagnostic: ~1/z steps is a unit's memory)."""
        c, g, p = self.lin(x).chunk(3, dim=-1)
        z = th.sigmoid(g)
        h_tilde = th.where(c >= 0, c + 0.5, th.sigmoid(c))
        a, b = 1.0 - z, z * h_tilde
        keep = (1.0 - starts).unsqueeze(-1)
        # Only this linear recurrence is sequential; z and h~ above came out
        # of one batched matmul over every step.
        hs, h = [], h0
        for t in range(x.shape[0]):
            h = a[t] * (h * keep[t]) + b[t]
            hs.append(h)
        hs = th.stack(hs)
        s = th.sigmoid(p)
        return s * hs + (1.0 - s) * x, hs, z.mean()


class _SplitExtractor(nn.Module):
    """MlpExtractor's two halves on different inputs: the actor's MLP reads
    the MinGRU output (H wide), the critic's reads the observation."""

    def __init__(self, hidden, obs_dim, net_arch, activation_fn, device):
        super().__init__()
        self.policy_net = MlpExtractor(hidden, net_arch, activation_fn, device).policy_net
        self.value_net = MlpExtractor(obs_dim, net_arch, activation_fn, device).value_net
        self.latent_dim_pi = self.latent_dim_vf = int(net_arch[-1]) if net_arch else 0

    def forward_actor(self, x):
        return self.policy_net(x)

    def forward_critic(self, x):
        return self.value_net(x)


class MemoryPolicy(ActorCriticPolicy):
    """ActorCriticPolicy with a MinGRU between the observation and the actor
    MLP. Every stateless entry point SB3 would call raises: a call that
    forgot the state would silently act with an empty memory."""

    def __init__(self, *args, memory_hidden: int = 128, **kwargs):
        self.memory_hidden = int(memory_hidden)
        super().__init__(*args, **kwargs)

    def _build_mlp_extractor(self) -> None:
        H = self.memory_hidden
        # Built here so they exist before _build creates the optimizer. Torch
        # default init (not SB3's orthogonal one, which covers mlp_extractor).
        self.encoder = nn.Linear(self.features_dim, H)
        self.memory = MinGRU(H)
        arch = self.net_arch
        if isinstance(arch, dict):
            if list(arch.get("pi", [])) != list(arch.get("vf", [])):
                raise ValueError("MemoryPolicy takes one net_arch list for both")
            arch = list(arch["pi"])
        self.mlp_extractor = _SplitExtractor(H, self.features_dim, list(arch),
                                             self.activation_fn, self.device)

    def _get_constructor_parameters(self):
        return {**super()._get_constructor_parameters(),
                "memory_hidden": self.memory_hidden}

    # ---- the two halves -------------------------------------------------
    def actor_seq(self, obs: th.Tensor, h0: th.Tensor, starts: th.Tensor):
        """obs (T, N, obs_dim) -> (latent (T, N, L), states (T, N, H), gate)."""
        T, N = obs.shape[:2]
        f = self.extract_features(obs.reshape(T * N, -1)).reshape(T, N, -1)
        out, hs, z = self.memory(self.encoder(f), h0, starts)
        return self.mlp_extractor.forward_actor(out), hs, z

    def predict_values(self, obs: th.Tensor) -> th.Tensor:
        """Stateless critic, the same call SB3 makes for a bootstrap."""
        return self.value_net(self.mlp_extractor.forward_critic(self.extract_features(obs)))

    def step(self, obs, h_prev, starts, deterministic=False):
        """One control step for N envs: (actions, values, log_probs, h)."""
        lat, hs, _ = self.actor_seq(obs[None], h_prev, starts[None])
        dist = self._get_action_dist_from_latent(lat[0])
        actions = dist.get_actions(deterministic=deterministic)
        return actions, self.predict_values(obs), dist.log_prob(actions), hs[0]

    def evaluate_seq(self, obs, actions, h0, starts):
        """Over (T, N) chunks: values, log_prob, entropy (each (T, N)), the
        deterministic mean (T, N, A) and the mean gate."""
        T, N = obs.shape[:2]
        lat, _, z = self.actor_seq(obs, h0, starts)
        dist = self._get_action_dist_from_latent(lat.reshape(T * N, -1))
        log_prob = dist.log_prob(actions.reshape(T * N, -1)).reshape(T, N)
        entropy = dist.entropy().reshape(T, N)
        values = self.predict_values(obs.reshape(T * N, -1)).reshape(T, N)
        mu = dist.distribution.mean.reshape(T, N, -1)
        return values, log_prob, entropy, mu, z

    # ---- SB3's stateless API, closed ------------------------------------
    def forward(self, obs, deterministic=False):
        raise NotImplementedError("MemoryPolicy is recurrent: use step()")

    def evaluate_actions(self, obs, actions):
        raise NotImplementedError("MemoryPolicy is recurrent: use evaluate_seq()")

    def get_distribution(self, obs):
        raise NotImplementedError("MemoryPolicy is recurrent: use actor_seq()")

    def predict(self, observation, state=None, episode_start=None, deterministic=False):
        """sb3-contrib's recurrent signature: returns (actions, state). Pass
        the returned state back in, and `episode_start` True on an episode's
        first step. `state` None means an empty memory."""
        self.set_training_mode(False)
        obs, vectorized = self.obs_to_tensor(observation)
        n = obs.shape[0]
        h = (th.zeros(n, self.memory_hidden, device=self.device) if state is None
             else th.as_tensor(np.asarray(state), dtype=th.float32,
                               device=self.device).reshape(n, -1))
        st = (th.zeros(n, device=self.device) if episode_start is None
              else th.as_tensor(np.asarray(episode_start, np.float32),
                                device=self.device).reshape(n))
        with th.no_grad():
            lat, hs, _ = self.actor_seq(obs[None], h, st[None])
            actions = self._get_action_dist_from_latent(lat[0]).get_actions(
                deterministic=deterministic)
        actions = actions.cpu().numpy().reshape((-1, *self.action_space.shape))
        if isinstance(self.action_space, spaces.Box):
            actions = np.clip(actions, self.action_space.low, self.action_space.high)
        h = hs[0].cpu().numpy()
        if not vectorized:
            actions, h = actions.squeeze(axis=0), h.squeeze(axis=0)
        return actions, h


class RecurrentSmoothPPO(SmoothPPO):
    """SmoothPPO over a MemoryPolicy: stores the actor state per step and
    trains on `bptt`-step chunks. See the module docstring."""

    def __init__(self, *args, bptt: int = 32, **kwargs):
        self.bptt = int(bptt)
        super().__init__(*args, **kwargs)

    def _setup_model(self) -> None:
        super()._setup_model()
        if not isinstance(self.policy, MemoryPolicy):
            raise ValueError("RecurrentSmoothPPO needs a MemoryPolicy")
        T, H = self.bptt, self.policy.memory_hidden
        if self.n_steps % T or self.batch_size % T:
            raise ValueError(f"algo.memory.bptt {T} must divide n_steps "
                             f"{self.n_steps} and batch_size {self.batch_size}")
        self._last_h = th.zeros(self.n_envs, H, device=self.device)
        # The actor state BEFORE each step (before the episode-start reset).
        self._h_buf = np.zeros((self.n_steps, self.n_envs, H), np.float32)

    def _excluded_save_params(self):
        return super()._excluded_save_params() + ["_last_h", "_h_buf"]

    def collect_rollouts(self, env, callback, rollout_buffer, n_rollout_steps) -> bool:
        """Upstream OnPolicyAlgorithm.collect_rollouts (SB3 2.9.0) plus the
        `# MEMORY` lines; gSDE and discrete actions dropped (unused here)."""
        assert self._last_obs is not None, "No previous observation was provided"
        self.policy.set_training_mode(False)
        n_steps = 0
        rollout_buffer.reset()
        callback.on_rollout_start()
        h = self._last_h                                            # MEMORY
        while n_steps < n_rollout_steps:
            with th.no_grad():
                obs_tensor = obs_as_tensor(self._last_obs, self.device)
                starts = th.as_tensor(self._last_episode_starts,        # MEMORY
                                      dtype=th.float32, device=self.device)
                self._h_buf[n_steps] = h.cpu().numpy()                  # MEMORY
                actions, values, log_probs, h = self.policy.step(       # MEMORY
                    obs_tensor, h, starts)
            actions = actions.cpu().numpy()
            clipped_actions = np.clip(actions, self.action_space.low, self.action_space.high)
            new_obs, rewards, dones, infos = env.step(clipped_actions)
            self.num_timesteps += env.num_envs
            callback.update_locals(locals())
            if not callback.on_step():
                return False
            self._update_info_buffer(infos, dones)
            n_steps += 1
            # Handle timeout by bootstrapping with value function
            # see GitHub issue #633 (the critic is stateless: unchanged)
            for idx, done in enumerate(dones):
                if (done and infos[idx].get("terminal_observation") is not None
                        and infos[idx].get("TimeLimit.truncated", False)):
                    terminal_obs = self.policy.obs_to_tensor(infos[idx]["terminal_observation"])[0]
                    with th.no_grad():
                        terminal_value = self.policy.predict_values(terminal_obs)[0]
                    rewards[idx] += self.gamma * terminal_value
            rollout_buffer.add(self._last_obs, actions, rewards,
                               self._last_episode_starts, values, log_probs)
            self._last_obs = new_obs
            self._last_episode_starts = dones
        self._last_h = h                                            # MEMORY
        with th.no_grad():
            values = self.policy.predict_values(obs_as_tensor(new_obs, self.device))
        rollout_buffer.compute_returns_and_advantage(last_values=values, dones=dones)
        callback.update_locals(locals())
        callback.on_rollout_end()
        return True

    def _chunks(self):
        """The rollout as (n_chunks, T, ...) tensors plus each chunk's start
        state. Read from the buffer's (n_steps, n_envs, ...) arrays directly,
        never through get(), which flattens them in place."""
        rb, T = self.rollout_buffer, self.bptt
        n_c = self.n_steps // T

        def cut(x):
            x = np.asarray(x)
            x = x.reshape(n_c, T, self.n_envs, *x.shape[2:]).swapaxes(1, 2)
            return th.as_tensor(x.reshape(n_c * self.n_envs, T, *x.shape[3:]),
                                device=self.device)

        h0 = self._h_buf[::T].reshape(n_c * self.n_envs, -1)
        return {"obs": cut(rb.observations), "actions": cut(rb.actions),
                "old_values": cut(rb.values), "old_log_prob": cut(rb.log_probs),
                "advantages": cut(rb.advantages), "returns": cut(rb.returns),
                "starts": cut(rb.episode_starts.astype(np.float32)),
                "h0": th.as_tensor(h0, device=self.device)}

    def _smooth_seq(self, mu, starts):
        """(weighted loss, {term: unweighted mean}) over one minibatch of
        chunks; mu (T, B, A), starts (T, B). A window may not contain an
        episode start after its first step."""
        s, loss, parts = self.smooth, 0.0, {}
        if s["bound"]:
            v = bound_term(mu).mean()
            loss, parts["bound"] = loss + s["bound"] * v, v.item()
        if s["temporal"] and mu.shape[0] > 1:
            ok = starts[1:] == 0
            v = temporal_term(mu[:-1], mu[1:])[ok].mean()
            loss, parts["temporal"] = loss + s["temporal"] * v, v.item()
        if s["second"] and mu.shape[0] > 2:
            ok = (starts[1:-1] == 0) & (starts[2:] == 0)
            v = second_term(mu[:-2], mu[1:-1], mu[2:])[ok].mean()
            loss, parts["second"] = loss + s["second"] * v, v.item()
        return loss, parts

    def train(self) -> None:
        """SmoothPPO.train's loss, minibatched over chunks (`# MEMORY`)."""
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        clip_range = self.clip_range(self._current_progress_remaining)
        if self.clip_range_vf is not None:
            clip_range_vf = self.clip_range_vf(self._current_progress_remaining)

        entropy_losses, pg_losses, value_losses, clip_fractions = [], [], [], []
        smooth_parts = {k: [] for k in TERMS}
        gates = []
        data = self._chunks()                                       # MEMORY
        n_chunks = data["obs"].shape[0]
        per_mb = self.batch_size // self.bptt

        continue_training = True
        for epoch in range(self.n_epochs):
            approx_kl_divs = []
            perm = th.randperm(n_chunks)
            for i in range(0, n_chunks, per_mb):
                idx = perm[i:i + per_mb]
                # (B, T, ...) -> (T, B, ...): time first, as MinGRU takes it
                d = {k: v[idx].transpose(0, 1) for k, v in data.items() if k != "h0"}
                values, log_prob, entropy, mu, z = self.policy.evaluate_seq(
                    d["obs"], d["actions"], data["h0"][idx], d["starts"])
                gates.append(z.item())
                values, log_prob, entropy = values.flatten(), log_prob.flatten(), entropy.flatten()
                old_values = d["old_values"].flatten()
                old_log_prob = d["old_log_prob"].flatten()
                returns = d["returns"].flatten()
                advantages = d["advantages"].flatten()
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                ratio = th.exp(log_prob - old_log_prob)
                policy_loss_1 = advantages * ratio
                policy_loss_2 = advantages * th.clamp(ratio, 1 - clip_range, 1 + clip_range)
                policy_loss = -th.min(policy_loss_1, policy_loss_2).mean()
                pg_losses.append(policy_loss.item())
                clip_fractions.append(th.mean((th.abs(ratio - 1) > clip_range).float()).item())

                if self.clip_range_vf is None:
                    values_pred = values
                else:
                    values_pred = old_values + th.clamp(values - old_values,
                                                        -clip_range_vf, clip_range_vf)
                value_loss = F.mse_loss(returns, values_pred)
                value_losses.append(value_loss.item())
                entropy_loss = -th.mean(entropy)
                entropy_losses.append(entropy_loss.item())
                loss = policy_loss + self.ent_coef * entropy_loss + self.vf_coef * value_loss

                if any(self.smooth.values()):
                    s_loss, parts = self._smooth_seq(mu, d["starts"])
                    loss = loss + s_loss
                    for k, v in parts.items():
                        smooth_parts[k].append(v)

                with th.no_grad():
                    log_ratio = log_prob - old_log_prob
                    approx_kl_div = th.mean((th.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                    approx_kl_divs.append(approx_kl_div)
                if self.target_kl is not None and approx_kl_div > 1.5 * self.target_kl:
                    continue_training = False
                    if self.verbose >= 1:
                        print(f"Early stopping at step {epoch} due to reaching max kl: {approx_kl_div:.2f}")
                    break

                self.policy.optimizer.zero_grad()
                loss.backward()
                th.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.policy.optimizer.step()

            self._n_updates += 1
            if not continue_training:
                break

        explained_var = explained_variance(self.rollout_buffer.values.flatten(),
                                           self.rollout_buffer.returns.flatten())
        self.logger.record("train/entropy_loss", np.mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", np.mean(pg_losses))
        self.logger.record("train/value_loss", np.mean(value_losses))
        self.logger.record("train/approx_kl", np.mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", np.mean(clip_fractions))
        self.logger.record("train/loss", loss.item())
        self.logger.record("train/explained_variance", explained_var)
        self.logger.record("train/std", th.exp(self.policy.log_std).mean().item())
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/clip_range", clip_range)
        if self.clip_range_vf is not None:
            self.logger.record("train/clip_range_vf", clip_range_vf)
        for k, v in smooth_parts.items():
            if v:
                self.logger.record(f"train/smooth_{k}", np.mean(v))
        # Mean update gate: ~1/z steps is the typical unit's memory. Near 1
        # means the layer has learned to be a feedforward one.
        self.logger.record("train/memory_gate", np.mean(gates))


def sb3_actor(model, vecnorm):
    """`act(obs)` for a deterministic SB3 model, stateful when it is
    recurrent, with `act.reset()` to call at every episode start (a no-op for
    the MLP). `vecnorm` None feeds observations unnormalised."""
    recurrent = isinstance(model.policy, MemoryPolicy)
    st = {"h": None, "start": True}

    def act(obs):
        o = vecnorm.normalize_obs(obs) if vecnorm is not None else obs
        if not recurrent:
            return model.predict(o, deterministic=True)[0]
        a, st["h"] = model.policy.predict(o, st["h"], np.array([st["start"]]),
                                          deterministic=True)
        st["start"] = False
        return a

    def reset():
        st["h"], st["start"] = None, True

    act.reset = reset
    return act
