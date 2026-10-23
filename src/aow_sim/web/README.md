# Browser demo

The general policy driving the bike, live in MuJoCo's WebAssembly build. A static
site: no server code, no build step, about 4 MB on the wire.

| file | what |
|---|---|
| `export.py` | writes `site/assets/` from a move: the compiled model, the weights, the constants |
| `site/` | the page. Deploy this folder |
| `site/js/sim.js` | `GeneralEnv.step` ported to JS: firmware drive loop, detent, gearbox friction, odometry, MinGRU, the crank servo; plus teleop's shove and ball |
| `site/js/sit.js` | `swing_sit.SwingSit`: lie down, get up, auto-up after a fall |
| `site/js/teleop.js` | teleop's command ramps, same constants |
| `site/js/viewer.js` | three.js, teleop's bare look |
| `check.mjs` | replays a Python trace through `sim.js` in node |

## Export, check, look

```sh
python -m aow_sim.web.export --trace traces/web_trace.json
```

```sh
npm install --prefix src/aow_sim/web
```

```sh
node src/aow_sim/web/check.mjs traces/web_trace.json
```

```sh
python -m http.server 8766 --directory src/aow_sim/web/site
```

`--policy NAME` exports another move (default `control.general_move`); the
exporter refuses a policy that needs a path `sim.js` does not implement (an
AHRS model, counts steer rate, wings, the style bit, an MLP).

**What the check should say** (2026-10-23, `general_puffer_fixes_72M`): open
loop, the JS policy matches Python to ~1e-15. Closed loop the chassis agrees
to 1e-9 for ~0.5 s and 1e-6 for ~1.4 s, then parts (the first observation to
move is `v_lon`, an encoder count) and ends ~4 cm apart after 14 s, neither
side falling. The exact step moves with any change to the model, the parked
ball included; the shape should not. Streaming (`advance()` in random chunks, as frames call it)
matches whole control steps exactly, 0.0. A jump in the first few steps, or a
fall Python does not have, is a porting bug, not float noise.

Not covered by the trace: the lie-down / get-up sequence, the shove and the
ball, which GeneralEnv does not have. Those were checked by behaviour: 20
knockdowns in a row, some doubled, all riding again in 3.8-7.8 s, 2-3 of them
on a retried lift (the retry now lives in `swing_sit`); lie down and get up through every phase; blades at 91
deg with the bike still balancing (peak lean 2-4 deg).

## Keys

| key | does |
|---|---|
| arrows / WASD | speed and turn: teleop's ramps and lead clamp, without its 10 deg jump on a fresh turn press |
| Q / E | heading snap 90 deg left / right; it leads past the clamp until the bike catches up |
| Space | knock it over (3 N, from the side, alternating; a red arrow shows it for exactly as long as the force acts); it gets up by itself, and tries again 1 s after a lift that fails |
| Shift+Space | a nudge the policy catches (1.5 N) |
| Z / C | step the blades left / centre / right, to 91 deg of crank (70 % of the stroke), not the floor |
| X | lie down; again to get up. The shell pulses while it does either |
| B | throw the ball into its flank at 2.5 m/s (it starts in front of the bike); or just drive into it |
| T | telemetry, hidden by default |
| R | reset |

The rear wheel leaves teleop's red trail (2 s, then fading). Phones get the
same keys as buttons. `?telemetry` starts with the readout shown;
`?hud=0` hides the key hints; `?debug` puts `sim`, `teleop`, `viewer` and
`cfg` on `window`.

## The plant

The policy's own eval env (`per_command.py`'s), on the current `bike_params`,
with the AHRS error model off, so attitude and heading are MuJoCo truth. Felt
in teleop before this was built: no visible difference. Everything else it
trained with stays: the XC430 firmware loop (the one that matters by hand),
detent, roller slop, steer gearbox friction, and velocity from quantised
encoder counts through the onboard estimator.

Two additions make it teleop's bike: the righting module JOINTED, its crank
on the current-based position servo model (the policy trained on it welded,
and holds it at stow either way), and a ball, without the ball-shot move's
stick panels: a 160 mm, 10 g play ball that bounces (0.48 of its impact
speed back, with a contact stiff enough not to jitter against the wheel), with air drag the page applies and rolling friction (at the
sim's condim 4 rolling friction is ignored and the study's ball rolled 53 m).
Its contact settings are its own: MuJoCo mixes each contact from both geoms
(friction the larger, solref/solimp weighted by solmix), and the ball's
solmix is set to win, so nothing else's contact changed. Driving into it or
past it with a blade out, 0 falls in 24; the 67 mm ball fell when the blade
rode up onto it and propped the bike over, which is why it is big.
`export.py` has the sweeps; `docs/plans/web-demo-divergences.md` collects
what differs from teleop.

The model ships as `bike.mjb`, the compiled binary, because an XML round trip
was not faithful (float precision, re-processed meshes, lost `eq_active0`).
An `.mjb` loads only in the MuJoCo version that wrote it, so `config.json`
records the version and the page loads exactly that build from jsDelivr;
`package.json` pins the same one for the check. **Re-export after upgrading
the Python `mujoco`.**

## Deploy and embed

`site/assets/` is generated. Commit it if the host builds from git (Cloudflare
Pages with a repo, output directory `src/aow_sim/web/site`, no build command),
or upload the folder directly:

```sh
npx wrangler pages deploy src/aow_sim/web/site --project-name aow-bike
```

The largest file is `bike.mjb` at 4.3 MB (0.9 MB gzipped), under Pages' 25 MiB
per-file limit. MuJoCo's wasm (10 MB, 2.7 MB gzipped) and three.js come from
jsDelivr, cached across sites.

```html
<iframe src="https://<your-site>/" title="AOW bike simulation"
        style="width:100%; aspect-ratio:16/10; border:0"></iframe>
```

Keys reach the frame once it is clicked; the hint says so until then, and the
host page does not scroll from them.
