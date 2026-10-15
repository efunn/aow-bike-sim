"""A PufferLib checkpoint (`.bin`) -> a move (`moves/<name>.npz` + `.yaml`)
that replays as `control.policy.MinGRUPolicy`: the eval grid, per_command.py,
teleop, DriveController and the Pi all take it unchanged. No warp import.

    python -m aow_sim.gpu.export_policy CHECKPOINT.bin --graph-meta G.json \\
        --ini aow_bike.ini --name general_puffer_<run> [--steps N] [--no-eval]

WHAT THE .bin IS (PufferLib ea7f989, read from the source, not documented):
`puf_save_weights` dumps the fp32 master weights, one flat array of
`total_elems` floats, no header. The master copy mirrors the parameter
allocation byte for byte in ELEMENTS, and that allocation (`alloc_create`)
starts every tensor on a 16-byte boundary -- 8 elements for a bf16 build,
4 for `--float`. The tensors, in registration order (`arch reg_params`):

    encoder   W (H, obs)             no bias, no activation
    decoder   W (A + 1, H)           rows 0..A-1 the action mean, row A the
                                     value; no bias
    logstd    (1, A)                 continuous only (sampling; not replayed)
    mingru    W (3H, H) per layer    rows [c | g | p], no bias

`total_elems` counts the tensors WITHOUT the padding, while the dump (and
the optimiser's view) covers the first `total_elems` elements of the padded
block. So padding before a tensor shifts it, and the LAST tensor loses as
many trailing weights as there were padding elements: never trained, never
saved. With obs 17, A 3, H 128 in bf16 that is 5 of the MinGRU's 49152
(after logstd's 3). They stay at their random init on the GPU; here they
are written as 0 and counted in the move yaml.

THE FORWARD (algo.cu): out = dec @ mingru(enc @ obs), the layer exactly as
recurrent_ppo.py / MinGRUPolicy (highway included), the action the
UNSQUASHED mean, which replay clips to [-1, 1] as the env does. No
observation normalization: identity stats, and no clip (obs_clip 1e6).
Trained in bf16, replayed in float64 -- small differences, not checked
against the trainer's own forward.
"""
from __future__ import annotations

import argparse
import configparser
import json
from pathlib import Path

import numpy as np


def _ini(path) -> configparser.ConfigParser:
    c = configparser.ConfigParser(inline_comment_prefixes=(";", "#"))
    c.read(path)
    return c


def layout(obs: int, act: int, hidden: int, layers: int, align: int):
    """[(name, shape, offset)] in the padded allocation, and total_elems."""
    shapes = [("enc", (hidden, obs)), ("dec", (act + 1, hidden)), ("logstd", (1, act))]
    shapes += [(f"mingru{i}", (3 * hidden, hidden)) for i in range(layers)]
    out, off, total = [], 0, 0
    for name, shp in shapes:
        off = (off + align - 1) // align * align
        n = int(np.prod(shp))
        out.append((name, shp, off))
        off += n
        total += n
    return out, total


def unpack(flat: np.ndarray, obs: int, act: int, hidden: int, layers: int, align: int):
    """The tensors out of a checkpoint; also how many trailing weights the
    dump does not hold (see the module docstring)."""
    lay, total = layout(obs, act, hidden, layers, align)
    if flat.size != total:
        raise SystemExit(f"checkpoint has {flat.size} floats; obs {obs}, act {act}, "
                         f"hidden {hidden}, layers {layers} need {total} -- wrong "
                         "network size, or not this PufferLib version")
    padded = np.zeros(lay[-1][2] + int(np.prod(lay[-1][1])), np.float32)
    padded[:total] = flat
    missing = padded.size - total
    return {n: padded[o:o + int(np.prod(s))].reshape(s) for n, s, o in lay}, missing


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("checkpoint")
    ap.add_argument("--graph-meta", required=True, help="the exported graph's .json")
    ap.add_argument("--ini", required=True, help="the aow_bike.ini the run used")
    ap.add_argument("--name", required=True)
    ap.add_argument("--steps", type=int, default=0, help="training steps, for the record")
    ap.add_argument("--float", action="store_true", help="the trainer was built --float")
    ap.add_argument("--no-eval", action="store_true", help="skip the deterministic eval")
    ap.add_argument("--moves-dir", default=None, help="default: moves/")
    a = ap.parse_args()

    import yaml
    from ..build_model import load_params
    from ..control.general_env import _load_rl_config
    from ..control.general_spec import ActionBounds
    from ..control.policy import save_policy_npz
    from ..drivetrain_model import from_env_config
    from .. import train_general_rl as tg
    from ..control import flick

    meta = json.loads(Path(a.graph_meta).read_text())
    ini = _ini(a.ini)
    hidden = ini.getint("policy", "hidden_size")
    nl = ini.getint("policy", "num_layers")
    if nl != 1:
        raise SystemExit("MinGRUPolicy replays exactly one MinGRU layer")
    obs, act = int(meta["obs_dim"]), int(meta["act_dim"])
    flat = np.fromfile(a.checkpoint, dtype=np.float32)
    t, missing = unpack(flat, obs, act, hidden, nl, 4 if a.float else 8)

    cfg = _load_rl_config(meta["config"])
    params = from_env_config(load_params(), cfg["env"])
    out = Path(a.moves_dir) if a.moves_dir else tg.MOVES_DIR
    name = flick.reserve_move_name(a.name, out)       # never overwrites a move
    npz = out / f"{name}.npz"
    z = lambda n: np.zeros(n, np.float32)
    save_policy_npz(npz, [(t["dec"][:act], z(act))], "tanh", z(obs), np.ones(obs),
                    ActionBounds(**tg.export_bounds(cfg)), obs_clip=1e6,
                    memory=((t["enc"], z(hidden)), (t["mingru0"], z(3 * hidden))))
    metrics = None if a.no_eval else tg._eval(params, cfg, npz)
    trained = {"algo": "PufferLib 5.0 PPO (ea7f989), Muon",
               "timesteps": int(a.steps),
               "arch": f"Linear({obs},{hidden}) -> MinGRU({hidden}) -> Linear({hidden},{act}), no biases",
               "precision": "fp32" if a.float else "bf16 (fp32 master weights)",
               "logstd": [round(float(v), 4) for v in t["logstd"][0]],
               "unsaved_weights": int(missing),
               "graph": {k: meta[k] for k in ("nworld", "config", "config_sha256", "seed")
                         if k in meta} | {"curriculum": meta.get("curriculum")},
               "exported_from": str(a.checkpoint)}
    if metrics is not None:
        trained["metrics"] = metrics
    with open(out / f"{name}.yaml", "w") as f:
        yaml.safe_dump(tg.move_doc(name, params, cfg, trained), f, sort_keys=False)
    print(json.dumps({"wrote": str(npz), "unsaved_weights": missing,
                      "logstd": trained["logstd"], "metrics": metrics}, default=str))


if __name__ == "__main__":
    main()
