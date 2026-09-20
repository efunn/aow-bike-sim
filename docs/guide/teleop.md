# Teleop reference

Every key, and what it does. Confirm in `src/aow_sim/run_drive.py` because I like changing the controls pretty often.

```sh
mjpython -m aow_sim.run_drive --teleop
```

**`mjpython`, not `python`.** MuJoCo's interactive viewer requires it on macOS.

Bindings are mostly digits, arrows, and punctuation to avoid collisions with the default MuJoCo keybinds (i.g. A-Z keys).

---

## Controllers

**A trained policy drives by default** whenever a `moves/general_rl*` export
exists and matches the current spec. You give it a velocity vector and a
heading, continuously, and it balances.

`,` can swap it for the **analytic LQR**, which is a reference baseline and maintained in a 'best effort' manner. **Not all the keys work there**: it has no lateral command at all, and the digits address a library of older canned maneuvers instead of steering the live command. The tables below say which layer each key belongs to.

## Key controls (global)

| key | what it does |
|---|---|
| `↑` / `↓` | throttle / brake into reverse. The opposite key brakes hard *through* zero; releasing both coasts the target back to zero |
| `←` / `→` | turn. Hold to slew continuously, tap for a 10° nudge |
| `5` | **stop now** — longitudinal *and* lateral, not a coast-down |
| `/` | re-zero the command |
| `2` | toggle the ground dial overlay |
| `,` | policy menu |
| `\` | camera: free → follow → overhead → rear wheel |
| `;` / `'` | trail: pen-up / 2 s / 4 s / 10 s / infinite |
| `-` / `=` | slow motion: halve / double, clamped 1×–64× |
| `ENTER` | [spawn dial](#spawn-dial-tilt-the-floor-and-reposition-the-spawn-orientation) — pause, reposition, tilt the floor |
| `F5` | fullscreen (`Backspace` resets) |

Velocity is game-style: a fresh press steps the command by 0.25 m/s, holding
builds it continuously, releasing everything coasts the target to zero.
Heading is a **setpoint** — it stays where you leave it.

A *held* turn is clamped to **35° of lead** over the bike's actual heading, so it cannot wind up. Heading snaps deliberately bypass that clamp.

## Driving keys (RL policy)

| key | what it does |
|---|---|
| `1` / `3` | **crab left / right** — lateral velocity, heading held |
| `6` / `7` | snap the heading 90° left / right |
| `8` | snap the heading 180° |

Crab is policy-layer only: the analytic controller has no lateral command. It behaves like the throttle: tap to step, hold to build, release to coast, and is clamped to the lateral envelope the policy actually trained on (`v_lat_frac` in its move file; at 0.4 × `v_max` that is about 0.48 m/s). **The clamp follows whichever policy is loaded**, because each move file carries its own value.

## Driving keys (LQR)

**Legacy.** These maneuvers predate the general policy, which does all of it from a live command instead. They are documented because the keys are still live and still do something, not because they are good.

| key | what it does |
|---|---|
| `6` / `7` | circle left / right |
| `8` / `9` | flick 180° — trajopt, reverse-first / forward-first |
| `3` | flick 180° — RL policy |
| `4` | crawl front-pivot 180°, in place |
| `.` | pivot 180° holding the front wheel's line — RL |
| `1` | ball shot (RL), re-parking the ball first |

Each zeroes the live command, runs to completion, and hands control back to the LQR. A missing move file prints what is absent rather than raising.

## Self-righting mechanism

`--swing-linkage` adds the co-rotating self-righting mechanism (several other self-righting mechanisms were tested and can be accessed by other args like `--swing` or `--linkage`). They are all operated with the same keys.

| key | what it does |
|---|---|
| `9` / `4` | **step** through left → centre → right. Double-tapping either one crosses the whole range, so "swap sides" is one gesture |
| `.` | shove the bike over — a repeatable 8 N pulse, alternating sides |

Nothing deploys or hands off by itself (yet). That is deliberate: push the bike over and watch what it actually does before deciding what should trigger.

By hand instead of `.`: double-click the bike, then Ctrl + right-drag.

## `--hockey`: a ball scene

| key | what it does |
|---|---|
| `Space` | throw the ball at the bike, alternating sides |
| `[` / `]` | throw speed down / up, 0.5 m/s a press, up to 8 m/s |
| `0` | re-park the ball at its start pose — works in either controller layer |

At 0 m/s the ball simply drops where it is. The topple threshold was once measured around 4.5 m/s.

## Policy menu

`,` opens a menu in the top right: every `kind: general` move in `moves/`, newest first, with the analytic LQR as the first entry. `↑`/`↓` move the cursor, `ENTER` loads, `,` closes. `>` marks the cursor, `*` marks what is currently driving.

**Loading is live.** The new policy inherits the heading and velocity you were already commanding, so a mid-drive swap does not jolt the bike. The command *is* zeroed on any switch between controller **kinds**, so nothing inherits a stale setpoint.

The cursor opens on the controller you are *not* using, so `,` `ENTER` is a two-keystroke analytic/policy toggle. The list is rebuilt every time the menu opens, so a policy exported from another terminal shows up without restarting.

While the menu is open it **swallows every key**, not just the ones it uses: picking a controller and driving are different activities, and a stray throttle tap landing between opening the menu and choosing would be applied to whatever loads next.

It is a menu of labels rather than a real dropdown because MuJoCo's passive viewer exposes no widget API and no 2D overlay — scene geometry is the only thing a script can draw. `TAB` is unavailable for this; it is the viewer's own left-panel toggle.

## Spawn dial: tilt the floor and reposition the spawn orientation

`ENTER` **pauses** and opens a dial for placing the bike. It borrows the keyboard while open, and `ENTER` again resumes with whatever heading you left it on.

| key | what it does |
|---|---|
| `←` / `→` | swing the spawn heading |
| `6` / `7` / `8` | snap it 90° left / 90° right / 180° |
| `5` | back to zero |
| `-` / `=` | **step through the floors** — a preview of the tilt |
| `BACKSPACE` | respawn the bike there, staying open so you can try another |
| `ENTER` | resume |

`-`/`=` are slow motion while driving and floor selection in here. The spare floors are tilted, and the dial picks the **magnitude** of the tilt; the *direction* comes from `--slope-bearing` at launch, or from respawning on a different heading.

`--slope-bearing` defaults to 0, which puts downhill **across** the fixed camera. That is deliberate and worth knowing: a few degrees of tilt falling *away* from the viewer reads as flat, so the default is the one bearing where a small slope is visible at all. 90 is downhill to the bike's left.

## Ground dial

`2` toggles it. A dial drawn on the ground under the bike showing:

- **heading** as a tick on the rim: green commanded, cyan actual
- **velocity** as an arrow in the inner gauge, full scale `v_max`: orange commanded, yellow actual

## Camera and trail

`\` cycles four modes: **free** (the viewer's own, mouse-driven), **follow** (chase, azimuth tracking the bike's heading), **overhead** (plan view) and **rear wheel** (broadside, with roller stripes and a 25 mm ground grid).

Switching back to free re-frames once to the opening 3/4 view centred on the bike, rather than inheriting overhead's straight-down elevation. After that handoff the mouse owns the camera again.

A 0.5 m **floor grid** is drawn in every mode. The tracked cameras hold the bike still in frame, so without a world-fixed reference a moving bike looks parked.

A **red trail** marks where the bike has actually been — the last 2 s solid, then fading to clear over 0.5 s. `;` and `'` step that history through pen-up / 2 s / 4 s / 10 s / infinite. **Pen-up keeps what is already drawn and stops adding**, so the bike can be repositioned invisibly and a disconnected shape drawn: that is how the `t` drawing script puts a stem under a crossbar without retracing it.

## Hold-to-drive needs an extra

The viewer reports a key going *down* and nothing else (no release, no auto-repeat) so holding a key can only work if the OS supplies real key state. On macOS we use pyobjc:

```sh
pip install -e '.[dev,teleop]'
```

Without it teleop still works (kinda), but every key is tap-only.

## The virtual gamepad

Teleop and the drawing scripts are two front-ends onto one mapping, `control/gamepad.py`: axes in, a velocity vector plus a heading rate out. Anything a script can draw is therefore reachable by hand.

| stick | key | command |
|---|---|---|
| LEFT Y | `↑` / `↓` | longitudinal velocity (± `v_max`) |
| LEFT X | `1` / `3` | lateral velocity (± `v_lat_frac · v_max`) |
| RIGHT X | `←` / `→` | heading **rate** — integrates, so releasing holds the heading |
| A / B / X / Y | `5` `/` `6` `7` | stop · re-zero · snap 90° L / R |
| LB / RB | `'` / `;` | trail longer / shorter |

**The reason this indirection exists** is that a gamepad is not the only thing that can push those axes. `general_rl` takes a velocity vector and a heading and does the balancing; `gamepad.py` is the layer that turns *any* stream of axis values into that command. So a higher-level policy (one doing navigation, or stringing together moves) can drive the bike through the same interface a human does, on top of the trained low-level controller, without either side knowing about the other.

`aow_sim.record` burns a gamepad overlay into every video frame (stick gates, pen state, snap button) so a recording shows what was *commanded*, not just what happened. The live viewer does not have it, deliberately: the keyboard path has no continuous axes to display, so the gates would read as a square wave. It becomes worth porting if a real game controller feeds `gamepad.py`.

## Driving the real bike

`run_drive --mirror HOST` drives the real bike and mirrors it in a viewer of its own. **Nothing is simulated**: there is no physics step at all, and every pose you see is telemetry coming back from the bike over UDP. What carries across is the *key map*: the same command surface as `--teleop`, on the same protocol `hw/ground.py` speaks.

| key | what it does |
|---|---|
| `P` | **pin the bike at the origin**, or let it go. Unpinned it walks by its own dead-reckoned odometry, so the drift *is* the odometry error, made visible. Pinned, the drift is hidden and you get a fixed reference |
| `-` / `=` | zoom. The viewer owns the scroll wheel |

The camera is fixed-azimuth tracking, closer than teleop's. There is a telemetry readout on screen as well. **It is still in flux**, so it is not documented here (read it off the window).

`hw/ground.py` is a fallback station for a machine with no display or no MuJoCo: arrows or `WASD`, space to re-zero. See [physical-build.md](physical-build.md) for the bike side.
