# Browser demo

The general policy driving the bike, live in MuJoCo's WebAssembly build. A static
site: no server code, no build step, about 4 MB on the wire.

| file | what |
|---|---|
| `export.py` | writes `site/assets/` from a move: the compiled model, the weights, the constants |
| `site/` | the page. Deploy this folder |
| `site/js/sim.js` | `GeneralEnv.step` ported to JS: firmware drive loop, detent, gearbox friction, odometry, steer rate, MinGRU, the crank servo; plus teleop's shove and ball, and `run_bike`'s steer hold while lying down or getting up |
| `site/js/sit.js` | `swing_sit.SwingSit`, every option: lie down, get up, auto-up after a fall |
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
AHRS model, wings, the style bit, an MLP).

**What the check should say** (2026-10-26, `general_puffer_a40_v4_80M`):
open loop, the JS policy matches Python to ~1e-15. Closed loop the chassis
agrees to 1e-9 for ~0.5 s and 1e-6 for ~1 s, then parts and ends ~1.5 cm
apart after 14 s, neither side falling. That is the plant's own sensitivity:
Python against itself with 1e-14 added to one velocity parts on the same
schedule (1e-6 at step 44 vs the JS's 48, 1e-3 at 71 vs 77, 2.2 cm at the
end); the sawtooth detent's stick-slip parts runs sooner than the old plant
did (1e-6 at ~1.4 s). The exact step moves with any change to the model, the parked
ball included; the shape should not. Streaming (`advance()` in random chunks, as frames call it)
matches whole control steps exactly, 0.0. A jump in the first few steps, or a
fall Python does not have, is a porting bug, not float noise.

Not covered by the trace: the lie-down / get-up sequence, the shove and the
ball, which GeneralEnv does not have. `sit.js` against `swing_sit.py` on
scripted inputs (run_bike's settings, a toy plant, every phase including
`pump`): 32k steps, commands equal to 2e-16. Flown on the page's sim
(10-26): lie down and get up, 6 / 6 both sides, up in 3.1 s; 20 knockdowns
in a row, some doubled, all riding again in 4.3-5.3 s, no lift failing;
a lift made to fail (5 % Goal Current) lowers, lies and lifts again 1 s
later. Blades at 91 deg with the bike still balancing (peak lean 2-4 deg,
10-23).

## Keys

| key | does |
|---|---|
| arrows / WASD | speed and turn: teleop's ramps and lead clamp, without its 10 deg jump on a fresh turn press |
| Q / E | heading snap 90 deg left / right; it leads past the clamp until the bike catches up |
| Space | knock it over (2 N, from the side, alternating; a red arrow shows it for exactly as long as the force acts); it gets up by itself, and tries again 1 s after a lift that fails |
| Shift+Space | a nudge the policy catches (0.75 N; the v4 bike tips from ~1 N) |
| Z / C | step the blades left / centre / right, to 91 deg of crank (70 % of the stroke), not the floor |
| X | lie down; again to get up. The bike's sequence: stop, blade out, lean onto it, lower along the bike's measured roll map to the cage; up, the blade lifts it to a prop and the policy takes it from there. The steer eases straight meanwhile. The shell pulses throughout |
| B | throw the ball into its flank at 2.5 m/s (it starts in front of the bike); or just drive into it |
| T | telemetry, hidden by default |
| R | reset |

The sim pauses while less than half of it is on screen or the browser is in
the background (it sits below the clips on the site's `/aow/`, and kept
running while scrolled away), with a "Paused" banner at the top, and resumes
by itself. `/aow/` hands it the keyboard whenever it comes into view, so no
click is needed there. Dragging orbits the camera; the wheel scrolls the
page, no zoom.
The rear wheel leaves teleop's red trail (2 s, then fading). Phones get the
same keys as buttons. `?telemetry` starts with the readout shown;
`?hud=0` hides the key hints; `?debug` puts `sim`, `teleop`, `viewer` and
`cfg` on `window`.

## The plant

The policy's own eval env (`per_command.py`'s), on the current `bike_params`,
with the AHRS error model off, so attitude and heading are MuJoCo truth. Felt
in teleop before this was built: no visible difference. Everything else it
trained with stays, from its move yaml: the XC430 firmware loop (the one that
matters by hand), the detent with its sawtooth and kinetic friction, roller
slop, wheel play (front axle, headset, rear axle), steer gearbox friction,
velocity from quantised encoder counts through the onboard estimator, and the
steer rate differenced from the servo's counts.

Lie down and get up are `run_bike`'s, read off it at export (`sit_config`),
with two page choices where the bike waits for the operator. The bike stops
a get-up propped on a blade until Enter; the sim does not stay propped (its
lift reaches the prop still rolling and coasts over the top), so the page
hands it to the policy at once. And the bike never retries a failed lift (a
stalled crank, lifted again and again, overloaded); the page does, after
1 s.

Two additions make it teleop's bike: the righting module JOINTED, its crank
on the current-based position servo model (the policy trained on it welded,
and holds it at stow either way), and a ball, without the ball-shot move's
stick panels: a pickleball-sized (74 mm), 10 g play ball that bounces (0.48 of its impact
speed back, with a contact stiff enough not to jitter against the wheel), with air drag the page applies and rolling friction (at the
sim's condim 4 rolling friction is ignored and the study's ball rolled 53 m).
Its contact settings are its own: MuJoCo mixes each contact from both geoms
(friction the larger, solref/solimp weighted by solmix), and the ball's
solmix is set to win, so nothing else's contact changed. Driving into it or
past it with a blade out, 0 falls in 24 at 160 mm; the 67 mm ball fell when
the blade rode up onto it and propped the bike over. It was 160 mm for that
until 10-27, when it went to a real pickleball's size anyway (not re-swept).
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
