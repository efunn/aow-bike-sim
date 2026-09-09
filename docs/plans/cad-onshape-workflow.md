# Text to CAD: the Onshape / FeatureScript workflow

> **Status: ACTIVE reference for the round trip; the RECORD is frozen at
> 2026-08-19.** The workflow, the API-quota budget and the two traps still hold
> and are the thing to read before touching Onshape. Three weeks of CAD have
> happened since (servo mounts, the swing linkage, a Feature Studio tab); that
> work is recorded in `docs/status.md`, not here.

How `aow_sim.cad_layout` gets the simulator's geometry into a drawn bike, what
the platform will and will not let us do, and the things that cost a day to
find out. Started 2026-08-18; this section of the record covers up to
2026-08-19.

The export itself is `docs/cad/cad_layout.fs`, generated. Never edit
it — the regeneration command is in its header.

---

## The shape of it

One **Feature Studio** (a code tab) holding **several features**, inserted into
a Part Studio as separate nodes:

| feature | what it does |
|---|---|
| `AOW layout variables` | `setVariable` for every coordinate, so a sketch can reference `#aow_servo_steer_z`. Draws nothing, cannot fail. Insert first, once. |
| `AOW drivetrain` / `steering` / `servos` / `mount` / `electronics` / `righting` | envelopes, origin points and axis planes for **one group** |
| `AOW planes` | one print plane (the fork's) and four belt-clearance planes, two per side. No tickboxes — everything in it is a plane already |
| `AOW belts` | the four straight belt runs as solids, opaque near-black |
| `AOW belts mirror` | each run copied to the OTHER side, translucent magenta. Suppress this node to drop the symmetry aid without touching the real belts |
| `AOW four-bar sketch` | the righting linkage as construction geometry |
| `AOW bike layout` | the superseded all-in-one node, kept so documents that already have it keep their geometry. **Now carries a "Draw <group>" tickbox per group** — see below |

The per-group split is not cosmetic. A tree node can be **renamed**,
**suppressed**, and **reordered**; a checkbox inside one feature can do none of
those. It also shrinks the blast radius: a runtime error aborts the whole
feature it is in, and the first version of this lost the published variables to
a bug in the envelope code because both lived in one function.

Groups are derived from the data — `groups = dict.fromkeys(it["group"] ...)` —
so adding a group to `build()` adds a feature. There is no per-group code.

### The legacy node got per-group tickboxes too

The argument above is still right: a tree node can be renamed, reordered,
suppressed and folded, and a checkbox can do none of those. But a document
already built on the single `AOW bike layout` node cannot have any of it
without inserting eleven features and losing every entity id it already
references — which is a real cost for the one thing people actually want,
"hide all the electronics". So the same group list also generates a
`Draw <group>` boolean on the legacy node, and the draw loop skips a component
whose group is switched off.

They live in a **named parameter section** (`"Group Name" : "Which groups to
draw"`), which is not cosmetic. Appended bare they landed underneath the
`Plane size` numeric field at the bottom of a fifteen-field dialog, and the
first person to go looking for one did not find it. A generated control nobody
can locate is not a feature.

**The test is `== false`, deliberately, not `!= true`.** These parameters are
new on a feature that is already inserted in live documents. If Onshape does
not backfill an annotation default into an existing instance, every one of them
reads `undefined` — and under `!= true` the whole model would vanish on the
next regeneration. Under `== false` an unset parameter draws, so the worst case
is a checkbox that does nothing until the feature is edited once.
Wrong-but-visible beats wrong-and-empty.

## What can be named, and what cannot

- **Bodies: yes**, via `setProperty(..., PropertyType.NAME, ...)`. Every
  envelope and origin point carries its component name.
- **Planes and mate connectors: no, and there is no workaround.** They carry no
  metadata. The UI derives their names from the feature that made them, and
  that derivation is hardcoded to the feature type literally called `cPlane` —
  rename that feature and even Onshape's own naming stops. Both the filtered
  query (silently names nothing) and the unfiltered one (throws, taking the
  plane with it) were dead ends for the same reason.
- A property **the user has set by hand can never afterwards be overwritten**
  from FeatureScript. Rename a generated part in the UI and later
  regenerations silently stop renaming it. Reset under part → properties.

So a plane is always "Plane N" under whatever feature drew it, and the two
things that work are drawing fewer of them per feature and renaming the
feature.

## Query variables

`setQueryVariable(context, name, query)` — release 1.203 and later — publishes
a **named selection** that downstream features consume in a selection or plane
field. This is the mechanism that actually solves the naming problem: you
cannot name the plane, but you can name the *reference* to it, and that
reference survives the geometry being regenerated underneath it.

What the export publishes:

| name | what |
|---|---|
| `aow_q_<component>` | the envelope body |
| `aow_q_<component>_point` | the origin point |
| `aow_q_<component>_plane` | the axis or print plane |
| `aow_q_<component>_case_holes_<face>` | a 4-point case hole pattern |
| `aow_q_fourbar` | the four-bar sketch edges |

The `plane_*` items therefore come out as `#aow_q_plane_fork_print_plane` and
so on — the doubled word is the item's own `plane_` prefix meeting the `_plane`
suffix the helper appends. Ugly, stable, left alone.

**They do not appear in the Variable table.** That table lists variables owned
by Variable features; `setVariable` from a custom feature lands there, and
`setQueryVariable` does not. The queries surface only where they are
consumable. There is no browser, so the generated `.fs` is the index —
`grep setQueryVariable docs/cad/cad_layout.fs`.

`--no-query-vars` regenerates without them. Worth knowing why the flag exists:
an unknown function in FeatureScript is a **compile** error, which takes down
the entire Feature Studio rather than one feature, so a signature we could not
verify from the docs needed an escape hatch. It compiled first time.

## Reference stability across a regeneration

Onshape does not do SolidWorks-style topological name matching. **Entity ids
are a deterministic function of the id of the operation that created them.**
Consequences, in the order they matter:

- **Order of operations inside the script is irrelevant**, because sub-ids are
  keyed by component *name* — `id + ("solid_" ~ name)` — not by a loop counter.
  This is the single most important thing to preserve. With `id + i`, inserting
  a component renumbers everything after it and silently rebinds downstream
  references to different geometry.
- **Changing a number is free.** New position, new radius: same ids, geometry
  moves, downstream features follow. This is why pasting a regenerated studio
  over the old one has been safe every time.
- **Changing a key breaks loudly.** Rename a component, drop one, or flip its
  `shape`, and the ids it produced cease to exist. Downstream features go red
  with a missing reference. That is the good failure.
- Same for variables: `#aow_payload_pi_z` breaks visibly on a rename.
- The one exception to "keyed by name" is a hole *pattern*, whose four corners
  are indexed. Safe, because the four corners of a rectangle cannot be
  reordered without the pattern itself changing.

The genuinely fragile part is that there is **no version pinning**: the Feature
Studio lives in the same workspace as the Part Studio, so every paste
propagates instantly with no rollback but undo. Putting the generated studio in
its own document and importing it as a versioned linked document would fix
that, at the cost of a version per regeneration. Not done; noted.

## Getting the script into Onshape

Copy-paste, or `--push`. Wired 2026-08-21 as `aow_sim.onshape`, two calls wide:

```
POST /api/featurestudios/d/{did}/w/{wid}/e/{eid}   {"contents": "<the .fs text>"}
GET  /api/partstudios/d/{did}/w/{wid}/e/{eid}/shadedviews?pixelSize=0&...
```

Copy-paste is still the default and still the fallback: browser calls are
exempt from the quota, so exhausting the API changes nothing about pasting.

`config/onshape.yaml` holds the document, workspace and three tab ids under
names, so `--push` and `--shot` take no arguments. It is checked in and carries
no secret — an id grants nobody anything. Naming the tabs is not tidiness: the
Feature Studio and the two Part Studios are three 24-hex strings with no
visible difference, and passing the wrong one 404s with nothing to read.

- **`feature_studio`** — generated, overwritten wholesale on every push
- **`layout`** (the Part Studio named `aow-bike-import`) — where the generated
  features are inserted, and what `--shot` renders by default
- **`bike`** — the real drawing, with imported wheel geometry. A `--shot`
  target only; nothing generated is ever written there

HTTP Basic with `accessKey:secretKey` from an API key (Onshape calls that
"local testing only" and prefers HMAC-SHA256; for a script on one machine it is
five lines instead of forty). The key needs **read + write documents and
nothing else** — replacing a studio's contents is a write, not a delete, and
`delete` / `share` / `purchases` are the three scopes that turn a bad script
from recoverable into not.

**You cannot test a key for free.** Measured 2026-08-21: Onshape answers 404 to
a nonexistent document id whether or not you are authenticated, and 403 to a
real one identically for bogus credentials and for no `Authorization` header at
all. So no probe of an absent document distinguishes a good key from a bad one,
and `python -m aow_sim.onshape <url>` reads a document you own instead — one
call on success, zero on failure, because 4xx is not billable. It proves read
scope only; the first push is the only test of write. `sourceMicroversion` + `rejectMicroversionSkew`
are optional concurrency guards — for a generated file that is never
hand-edited, overwriting is what you want.

**The quota is the reason not to get clever.** Free and Standard plans get
**2500 API calls per year** (Professional 5000); failed calls and anything done
in the browser do not count. A `--push` that POSTs once per regeneration is
fine. A watcher that syncs on save, or anything that walks the feature tree
through `getPartStudioFeatures`, is how you burn a year's allowance in a week.
Onshape shows a total and never a breakdown, so every call is appended to
`~/.local/state/aow/onshape_calls.jsonl` with what it did, billable or not
(`python -m aow_sim.onshape --log`). It lives outside the repo because it is
per-machine state, and it is advisory — check it against the usage page rather
than trusting it.

**The billing cycle is not the calendar year, and it is not the date the usage
page calls "Tracking start date" either.** That field read 19 Feb 2026 while
the same page said 312/365 days elapsed, which puts the real anchor at
13 Oct 2025. The elapsed-day count is the field to trust. Unused calls do not
roll over, so the number that matters is calls-per-day remaining, which is what
`budget_line` prints.

**`pixelSize=0` is what makes the render usable** — it fits the model to the
frame. With any other value the view matrix sets direction and pan only, and
the zoom is yours to get wrong. `shadedviews` reads the Part Studio, not the
Feature Studio, so it needs the other tab's element id; a studio that failed to
compile renders the error rather than the previous geometry.

There is no official local↔Onshape sync, no LSP, and no first-party editor
integration; the community VS Code extension is syntax highlighting only.

## The belts, and what "symmetric" costs

The belts were four clearance PLANES and are now also eight SOLIDS. A plane has
no thickness and no ends, so it forbids a whole sheet the belt does not occupy
and permits the two regions past the pulleys that it does; a chainstay can only
be checked against a solid.

Each run is a prism from tangent point to tangent point, 104.66 x 9 x 3.6 mm,
with its INNER face on the tangent line — `p1`/`p2` sit on the flange
envelopes, so the line is the belt's inner surface, not its centre. Only the
straight run is drawn; the wrap is the pulley envelope, already there.

**Each side also carries the OTHER side's belt, mirrored** (`belts_mirror`,
translucent magenta, its own feature node so it can be suppressed in one
click). The servos straddle 45 deg rather than sharing it — left at 37.372,
right at 52.628 — so the two sides' keep-outs differ by the 15.256 deg of
straddle. A chainstay that is the same part on both sides has to clear both,
and no single side's geometry shows that. Mirroring draws the union instead of
asserting it.

### The corridor, in the frame that matters

Two coordinate systems get mixed up here, and they differ by 90 deg.

- **Run angles**: left 24.522 / 50.222, right 39.778 / 65.478. These OVERLAP —
  right-lower sits below left-upper — so the union is contiguous, 24.522 to
  65.478, and there is no threading between the two belts. A guess that the gap
  lies "between the lower drive's upper belt and the upper drive's lower belt"
  is inverted: that window is negative by 10.444 deg.
- **Angular station about the rear axle**, which is the frame a chainstay
  leaving the axle actually lives in. Each tangent line touches the input
  pulley at run_angle -/+ 90, so the hull occupies 205.700 deg per side and the
  symmetric free window is **155.478 to 294.522 deg** — 139.044 wide, centred
  on exactly 225 = `drive_servo_angle_deg` + 180. Ray-sampled against the true
  two-circle hull to confirm.

Recorded rather than drawn. A drawn sector would wrongly exclude a chainstay
that ducks under both belts entirely, which is a legitimate route.

## Two traps in the generator

**The framed-box ordering.** `render` and `render_featurescript` feed `fCuboid`
the extents as `(box[1], box[0], box[2])` — the model-to-CAD X/Y swap — but
pass the three frame axes through `to_cad_dir` **without reordering the tuple**.
So `box[0]` lands along `frame[1]`, `box[1]` along `frame[0]`, and only
`box[2]`/`frame[2]` pair up the way they read. The servos are correct because
`box_size[0]` is the lateral width and `frame[1]` is the lateral axis, so they
happen to agree — which is why this never surfaced. Authoring the belts the
obvious way produced a run 9 mm long and 104.66 mm wide. Matched, not fixed:
the servos depend on it. Normalising it means reordering the frame tuple
alongside the extents and re-verifying the servos and the payload pack.

**Colour is a one-way door, and this document has already walked through it.**
`setProperty` with `PropertyType.APPEARANCE` and `color(r, g, b, a)` on
`qCreatedBy(subId, EntityType.BODY)` works; the 4th alpha argument is real and
alpha is honoured on a part with no `allowFaces`. But the rule that governs
names governs appearance: a colour the USER has set by hand can never
afterwards be overwritten from FeatureScript.

**Observed 2026-08-21, not merely feared.** A per-group palette was pushed with
correct `rgba` on all 49 bodies — verified in the generated `.fs`, amber servos,
green electronics, violet linkage — and in the viewport ONLY the eight belts
changed colour. The belts were brand-new bodies; every other body in this
document already carried a hand-set appearance (which is why the model was
uniformly pale translucent blue before any of this), and each one is
permanently immune.

Nothing in the generator can recover them. **What actually worked was a fresh
Part Studio tab**: new bodies have no hand-set appearance, so they take the
palette immediately. The old tab is kept as `rip` in `config/onshape.yaml`
rather than deleted, because the lock-out is worth being able to poke at.
Resetting appearance per part is the in-place alternative, if you can find the
control.

Until one or the other happens the palette is real in the export and invisible
in the document — worth knowing before concluding the generator is broken. It
took a check of the pushed `.fs`, where the correct `rgba` was sitting on all
49 bodies, to tell the two apart.

Two live consequences: **alpha is scene-wide.** A first attempt at 0.45 across
every group washed all hues toward the background AND made `shadedviews` return
a BLACK background instead of white. 0.85 restored both. And the see-through
should be spent only where it earns its keep — the mirrored belts, and the
frame's inertia primitives, which are not parts at all.

## Imports

`import(path : "<element id>", version : "<version>")`, optionally namespaced.
Same workspace → always current. Another document → pinned to a version, with a
link icon that offers the update when you want it. You can import Feature
Studios (their exported symbols), Part Studios (a `build` function that
instantiates their geometry — this is the Derived mechanism), and data blobs
(JSON/CSV via `BLOB_DATA`).

That last one is the interesting one for us: the layout *data* could be a JSON
blob tab consumed by a hand-written, stable Feature Studio, so the pushed
artifact is data and the code stops churning. Not done.

For several Part Studios — one per real part — the rule is **don't reference
geometry across them if you can avoid it**. Every studio imports the same
constants and positions itself from the same numbers the sim uses. Nothing to
break.

## Folders

Onshape has feature-tree folders: select features, group, name, nest, suppress
as a unit. But they organise the tree that *contains* a feature, and the planes
a custom feature draws are inside the node rather than siblings of it — so no
folder can be wrapped around them. FeatureScript cannot create folders or put
anything into one, and the same goes for the Parts list's folders.

Where it does pay off: the per-group features can be selected and folded into
one collapsed line. Which is an argument for inserting them.

## When a plane is worth generating

**Only when it does not coincide with geometry the model already has.** A print
or sketch plane picked off an existing face costs nothing to select and cannot
go stale; a generated one is a thing to keep in sync. So the servo mounting
plane never needed exporting — it is the plate's own face — while the fork's
plane did, because it holds the axle direction and the raked steering axis at
once and no face in the model is parallel to it.

### The two prospective build planes

Both added 2026-08-21, both alongside the datum they derive from rather than
replacing it — sketching on one must not silently redefine the other.

- **`plane_fork_print_offset`** — `plane_fork_print` moved 8 mm along its own
  normal (forward, tilted up by the 15 deg of rake). The ORIENTATION is the
  derived half and stays authoritative; the offset is `bike.fork_print_offset`,
  signed, and is meant to be edited once the fork has a real thickness.
- **`plane_drive_mount_print`** — the rear motor mount and dropout as one part.
  Normal is the mount's TANGENTIAL axis, 135 deg, so the build direction is up
  and rearward and the first layer is the bottom-front face. It lies in the
  LOWER (left) servo's long 34 x 46.5 outer face, at 28.500 mm tangential off
  the 45 deg centre line — a face that already exists in the assembly, rather
  than a datum nobody can point at. That 28.500 is `C sin(dtheta)` = 14.250
  plus half a case = 14.250, equal only because `drive_servo_gap` is 0 and the
  cases touch; open that gap and the plane moves with it. Per
  `drive_mount_open_wall`, a tangential build axis makes a SIDE wall the
  ceiling.

  It is a build plane for the MOUNT end, not a promise the whole part lies
  flat: the dropout still crosses the belt plane between here and the axle.

Two traps found by using them:

- **The linkage's axis planes are not datums.** Each is normal to its own link,
  so they sit at 61.7 deg, 57.2 deg and so on to lateral — they look like flat
  references in the viewport and measuring against one produced a 47.076 deg
  that had no meaning. Only `AOW planes` and the `front_wheel` / `fork` axis
  planes are intended as datums.
- **The two sides are not mirror images.** The servos straddle 45 deg rather
  than sharing it, so each belt spans its own pair of tangents — left 24.52 to
  50.22, right 39.78 to 65.48. The bands OVERLAP, so their union is contiguous
  and there is no corridor between them: anything crossing both belt planes at
  one angular station has to pass below 24.52 or above 65.48.
- **A plane parallel to a belt run is clearance, not a print orientation.** A
  rear dropout leaves the axle outboard of its own belt and arrives at the
  servo mount inboard of it, so it crosses the belt plane, and it can only do
  that outside the belt-and-pulley hull — the way a chainstay threads past the
  chain. The crossing dictates the shape, and no single flat build plane then
  aligns with both the sleeve and the arm.

## What the export carries

Shapes: `box`, `cylinder`, `capsule`, `point`, `holes` (a pattern of positions
travelling as one entry, so eight of them do not bury the layout) and `plane`
(position plus normal). Frame conversion is done once, in code — model frame is
+X forward / +Y left / +Z up in metres, CAD is +X right / +Y forward / +Z up in
millimetres, and hand-converting per component is exactly the sign error that
survives review.

## What the ROBOTIS drawings taught us

Three separate times, the servo's real envelope exceeded what `box_size`
carried, and each time the mechanism was the same: **the depth ROBOTIS quotes
is not consistently the same thing.**

- **XC330**: quoted 26 mm = 23 case + 3 horn, dimensioned as two numbers on its
  own drawing. The horn was already inside the box.
- **XC430**: quoted 34 mm is the case *alone*, measured from the horn-side case
  face. The Ø20.5 horn stands 2 mm proud of it and a Ø7.9 boss another 1.9
  beyond that. Real overall depth 36, not 34.
- Neither is documented as such anywhere except the three-view.

The fix was to stop treating a datasheet number as a face. `box_size` D is now
the **case alone** for both, the horn is its own primitive, and the shaft point
is the **mounting datum** — the outer face of the horn, the surface a pulley or
bracket actually bolts to — which makes `shaft_from_horn_face` negative and
equal to `-horn_thickness`.

Also worth having written down:

- The **P.C.D 16 pattern on the horn side is on the horn** (bolt circle 16 sits
  inside the Ø20.5 disc), so it rotates and is not a mount. There is no idler
  on the back by default. The only static pattern common to both faces is the
  **22 × 40** corner set, which is where the case's own M2.5 FHS assembly
  screws live — longer replacements capture a plate.
- The `8 | 8` chain beside the horn is the P.C.D 16 written as two radii. The
  same circle twice, not a third pattern.
- Reading a dimension off the wrong end of a symmetric drawing is easy: the
  XC430's horn side reads `3.9 | 2` and its idler side `2 | 3`.

## Open

- No version pinning between the Feature Studio and the Part Studio.
- `--push` not written; the quota says one call per regeneration is affordable.
- The layout data could travel as a JSON blob rather than as generated
  FeatureScript source.
- The drive-servo mount is a proposal, not a decision — see
  `analysis/servo_mount.py`, four tags: `cage`, `spine`, `sleeve`, `plate`.

---

## The CAD record to 2026-09-03 — moved here from docs/status.md

Moved verbatim 2026-09-08: the layout export, the servo mounts, the case
sides, and where `bike_params_cad.yaml` stands. The workflow above is how to
drive Onshape; this is what has been drawn with it.

### CAD — the bike stops being parametric

Started 2026-08-18. Drawn in Onshape; `python -m aow_sim.cad_layout` exports the
component layout from the parameters, as YAML for reading and as a FeatureScript
Feature Studio for Onshape. Both are **exports** — generated, never edited, with
the regeneration command in the header.

`config/bike_params_cad.yaml` is a scratch copy of `bike_params.yaml` that the
CAD work edits freely. It eventually becomes the authoritative one. **Never pass
it to `export_deploy`**: `params_digest` hashes the whole tree, so a bundle built
from a diverged file carries a digest no bike matches, and refusing that is the
entire point of the check. Keeping the work here is also why none of it has
moved the digest — `deploy/bundle.npz` and all 23 `moves/*.npz` are still valid.

**What CAD has already sent back into the model.** This is the value of the
workstream and it arrived immediately:

- **`input_pulley_offset` was made up.** Its own comment said so — "placeholder
  until the mount/pulley design is done" — and nothing derived from it. Pinned
  from real belt geometry (9 mm HTD5M, 45T/15T on the 370 mm belts bought,
  centre distance 107.35 mm from the belt equation) it becomes 7.5 mm, the
  minimum a 9 mm belt allows over a 33 mm wheel. **Rear width 99 → 80 mm.**
  It reached 75 briefly, on the wheel clearance alone. The belt plane is now
  **derived as the larger of two clearances** — the pulley missing the wheel
  (24.0) and the drive-servo mount plate missing the pulley (26.5) — and the
  mount binds. Every millimetre of plate or of plate-to-pulley gap is two on
  the bike, and the servos themselves do not move for any of it. The 3 mm that bought is what makes the two servo cases
  symmetric about the centreline (±17.0), which is what lets a single flat plate
  on one side of the bike bolt to both of them.
- **The drive-servo mount is drawn, and it set the width.** A plate on EACH
  side takes both servos — they face opposite ways, so one plane per side meets
  one horn-side face and one back face — on the 22 x 40 case pattern with M2.5
  machine screws, sixteen in all. The P.C.D 16 on the horn side is on the
  ROTATING horn and there is no idler by default, so 22 x 40 is the only static
  pattern common to both faces. The second plate cost NOTHING in width: the
  belt plane had already been pushed out to clear the first. A four-walled
  sleeve joins them and carries the torque in bearing, so the sixteen screws
  only retain; one of its six faces is the ceiling in the print and gets
  deleted in CAD, which face depending on the build axis. `AOW mount` and
  `AOW planes` are their own features in the Feature Studio, hence their own
  tickboxes.
- **The servo gap is now zero.** The 2 mm was clearance for a packing solve in
  which nothing located the cases; the sleeve does, so a gap between them was
  slop. Separation re-solved 16.35 -> 15.2563 deg, and it has a closed form now
  that the cases are parallel. The fit clearance moved to the sleeve cavity,
  where FDM shrinkage actually lives.
- **The Onshape workflow is written down** in
  `docs/plans/cad-onshape-workflow.md`: what the platform will and will not
  name, how query variables replace naming, why a regenerated Feature Studio
  can be pasted over the old one safely, the API quota, and the three separate
  ways a ROBOTIS datasheet depth turned out not to be a face.
- **Seven planes are exported**, being where CAD actually starts. One is the
  fork's datum: it holds the axle direction and the raked steering axis at
  once, so it is the front view tilted back by 15 deg. Four are the belt runs,
  two per side — NOT mirror images, because the servos straddle 45 deg rather
  than sharing it. Two are prospective BUILD planes added 2026-08-21: the
  fork's datum offset 8 mm along its own normal, and the rear motor+dropout's,
  which lies in the lower servo's long outer face with its normal on the
  mount's tangential axis (135 deg), so the part builds up-and-rearward off a
  face that already exists rather than off a datum nobody can point at. All
  derived, none eyeballed.
- **The belts are solids now, and each side carries both of them.** Eight
  prisms: the four real runs (104.66 x 9 x 3.6 mm, inner face on the tangent
  line, opaque near-black) plus each run mirrored onto the other side
  (translucent magenta, its own suppressible feature node). A plane has no
  thickness and no ends, so it could never be checked against; the mirror
  exists because the 15.256 deg of straddle makes the two sides' keep-outs
  differ, and a chainstay that is the same part on both sides must clear both.

  The corridor arithmetic, in the frame that matters: measured as an angular
  station about the rear axle, each belt hull occupies 205.700 deg and the
  symmetric free window is **155.478 to 294.522 deg**, 139.044 wide, centred on
  exactly 225 = `drive_servo_angle_deg` + 180 (ray-sampled to confirm). In
  RUN-ANGLE — a different frame, 90 deg away — the bands are left 24.522 to
  50.222 and right 39.778 to 65.478, and they OVERLAP by 10.444 deg, so there
  is no threading between the two belts. Recorded, not drawn: a drawn sector
  would wrongly exclude a chainstay that ducks under both belts.
- **The steer servo was 10.17 mm off the steering axis**, which direct drive at
  `gear_ratio: 1.0` does not permit. Its position is now solved, not chosen.
- **The TM151 is 40 × 34 × 12.6 mm and 19 g**, against the 30 × 30 × 12 mm / 12 g
  placeholders the sim still carries.
- **The drive servos' separation is a solved 2D packing problem** — two
  rectangles free to rotate about their own shafts, separating-axis tested —
  not a guess. Now 15.2563° with the gap closed, and reducible to a closed form
  because the two cases ended up parallel; the alternatives are tabled in the
  config.
- **The self-righting linkage moved 75 → 130 mm** to clear the drive belts and
  then the servo cases. **`analysis/linkage_through_belt.py` (2026-08-21) says
  most of that is recoverable, and cheaply.** The belt is genuinely what binds
  today — belt-limited minimum station 123.5 mm, so ~6.5 mm of the 130 is
  margin — but delete the belts and the floor is 109.0, set by the mount
  sleeve. Sweeping tooth counts and belt lengths with the servo cases carried
  radially along with the centre distance:

  | change | station | won | ratio | top speed |
  |---|---|---|---|---|
  | as built, 45T/15T on 370 mm | 123.5 | — | 3.00 | 1.06 |
  | **same pulleys, 340 mm belt** | 111.0 | 12.5 | 3.00 | **unchanged** |
  | **36T/12T on a 310 mm belt** | 105.0 | 18.5 | 3.00 | **unchanged** |
  | 32T/12T on a 290 mm belt | 98.0 | 25.5 | 2.67 | 0.94 |
  | 28T/12T on a 280 mm belt | 96.0 | 27.5 | 2.33 | 0.82 |

  It saturates at 96 mm — below ratio 2.33 nothing more is won, because that is
  where the servo cases meet the rear wheel rather than where the belt runs
  out. **Not adopted, and nothing is changed by it.** Smaller pulleys at the
  same ratio, closer together, is the free lunch and the thing to check first;
  the cases are translated rather than re-solved, so any shortlist entry wants
  re-deriving through `cad_layout` before it is believed.

- **The Feature Studio pushes over the API now.** `--push` and `--shot` on
  `aow_sim.cad_layout`, one billable call each, with the document and tab ids
  in `config/onshape.yaml` and API keys in the macOS Keychain — never in this
  checkout, which is Dropbox-synced, where gitignored is not un-synced. Every
  call is logged with what it did (`python -m aow_sim.onshape --log`). The
  annual quota is 2500 and the cycle is anchored to **13 Oct**, not January and
  not the "Tracking start date" the usage page shows — that field disagreed
  with the same page's own elapsed-day count. Copy-paste still works and is
  quota-exempt, so exhausting the API strands nothing.

  **Usage at 2026-08-24: 75 / 2500, with 50 days left in the cycle** — roughly
  48 calls a day available, so the budget is not a live concern. It becomes one
  only if something polls: `getPartStudioFeatures` reads the document's actual
  tickbox and suppression state in one call, which is genuinely useful and is
  also the endpoint that would burn a year in a week if put in a loop. The
  ground rules are in `CLAUDE.md`, "Onshape — the CAD round trip".

- **Generated FeatureScript is checked before it is pushed, as of 2026-08-24.**
  `--check` on `aow_sim.cad_layout` compiles AND RUNS the export against a
  throwaway copy of a Part Studio's context and refuses to push if it does not
  build. One billable call. This closes a real hole: a push *cannot* fail on
  bad FeatureScript, because the contents endpoint takes any text at all, so a
  broken export used to land in the document and surface as an EMPTY render
  with no error anywhere — two calls spent to learn nothing. Verified both ways
  on 2026-08-24: the current export runs clean, and a planted `fCuboid` ->
  `opCuboid` typo is caught as `Function opCuboid with 3 argument(s) not found`
  and blocks the push.

  The per-feature body counts it prints are also a free consistency check
  between the two arms of the generator: the eight geometry groups sum to 49,
  which is exactly what the monolithic `AOW bike layout` feature builds alone.

  The target is a **new empty Part Studio, `Eval Harness`**, created via the
  API on 2026-08-24 and recorded as the `check` tab in `config/onshape.yaml`.
  **Keep it empty** — its emptiness is the feature, since a body modelled there
  lands in any query not scoped to `qCreatedBy(id, ...)`. Nothing is ever
  written to it: Onshape derives the context, runs the script and discards it,
  confirmed by building a body, counting it, and counting 0 again on the next
  call.

  The awkward part is `_eval_wrapper` in `cad_layout.py`, which has to bring
  every top-level declaration inside one function expression — `export const`
  passes through, `export function` becomes a `const f = function(...)`, and
  the one `export predicate` is dropped as precondition-only — and must
  **synthesise the 15 `definition.*` parameters**, because the bodies test them
  as bare `if (definition.drawEnvelopes)` and an absent key is `undefined`,
  which throws rather than reading false. So it checks that the code RUNS, not
  that every branch matches a particular tick-box state. **Accepted.**

**Two traps worth not re-learning**, both of which produced confident wrong
answers before the user caught them from the CAD:

- **A 2D projection is not an interference.** The battery reads as overlapping
  the drive pulley by 13 mm in side view and clears it by 1.00 mm in 3D — the
  pack is 35 mm wide and the pulleys start at 18.5 mm, so they never share
  lateral space.
- **The roof is a cylinder, so its constraint is radial.** "Stay below the roof
  axis" is a *sufficient* condition, not the real one, and using it understated
  the battery's headroom by 36 mm.

**Outstanding.** `bike_params_cad.yaml` still has the drive servos at their old
`[45, 30, 75]` — `cad_layout` derives the real position every run and prints it
but does not write it back, so a MuJoCo model built from that file is not yet
the layout the CAD shows. Electronics packing is deferred until the tethered
version's wire routing is understood.

---

### Servo mounts — two custom features, screwless (2026-08-25)

`aow_sim.cad_servo_mount` generates two Onshape custom features into their own
Feature Studio (`horn_features` in `config/onshape.yaml`), separate from the
`cad_layout` studio because that one is overwritten wholesale on every push.

| feature | what it makes |
|---|---|
| `X330 horn pin` | 4 pins on the Phi 12 bolt circle, root reliefs, the horn well, and a standalone collar when no target part is picked |
| `X330 case shell` | both halves of the nesting case, 2 pins per face, from one dialog |

**Screwless, and the two pins are not the same pin.** The horn pin is Phi 1.4 in
the Phi 1.6 tapping hole (0.2 diametral); the case pin is Phi 1.9 in the Phi 2
*relief* bore of the drawing's Detail A/B (0.1). Different holes doing different
jobs — `config/bike_params_cad.yaml` says not to unify them.

**Only one of the two case hole rows is usable**, and it shapes the whole part.
The rows sit +/-15 from the face centre, putting one 22.5 mm from the shaft axis
and the other 7.5 — and 7.5 is inside the Phi 16 horn. So the shell wraps the far
end and leaves the shaft end open, and `caseWrapLength` is bounded at 16.5 where
the cap would foul the horn.

**The numbers are measured, not proposed.** The horn interface is off
`docs/robotis/XC-330.pdf`; the case interface was read back through the API off
the working `top-case` / `bottom-case` in `dynamixel-link`. The reconstructed
frame then agreed with the drawing on four independent dimensions — 23.00 /
34.00 / 20.00 against 23 / 34 / 20, shaft axis 9.50 against 9.5.

#### How it is checked, and the hole in that

Every revolved profile is a polygon emitted from Python, and the SAME polygon
feeds `revolve_volume` to predict what the result must measure. So `--check`
tests that Onshape's revolve, pattern and boolean did what the polygon says. It
is **not** an independent check of the shape; that still needs eyes on the part.

It caught three real bugs a render would not have: `cs.yAxis` does not exist on
a `CoordSystem`; `opBoolean` UNION takes `tools` only, so written the
SUBTRACTION way it unioned four pins with each other and never touched the
target (five bodies, no error, found by volume being short by exactly four
pins); and the well's undercut chamfer closed IN onto the floor, putting it at
Phi 15.7 against a Phi 16 horn.

**And one it structurally cannot catch.** `--check` throws away everything below
`SPLIT_MARK` — enums, `precondition`, `defineFeature` — which is what lets it run
without a human picking a mate connector. A doubled brace in the case dialog
therefore reached the document intact and took every feature in the studio red
at once, with the check still green. Two guards now: `lint_fs` rejects a literal
brace pair locally, and `verify_studio` hits
`GET /featurestudios/.../featurespecs` after every push, compiling the whole
studio server-side. A push cannot fail on bad FeatureScript — the contents
endpoint accepts any text — so without that a broken studio lands silently.

**An edge filter in front of the Onshape API rejects a command in backticks**
(shell command substitution) with a bare nginx 403 carrying no JSON, so the call
never reaches Onshape. Narrowed by probing the push endpoint, which takes any
text: backticks round a harmless word passed, the bare command passed, the two
together did not. The generated header writes its regeneration command unquoted
for that reason — do not tidy it into backticks.

**Outstanding.** The case shell has not been eyeballed against the as-built
parts; it agrees with box arithmetic and with Onshape's own measurement of it
and nothing more. `case_pin_root_chamfer` is a GUESS carried from the horn pins
— the geometry survey looked for cylinders and planes, and a chamfer is a cone.
`case_wrap_length` 10 mm is a cable-clearance judgement, not a measurement.

Cost: 111 API calls of the 2500/year, cycle anchored 13 October.

---

### The case sides, and where `bike_params_cad` now stands

The bike gained fixed side panels — two 4 mm ABS plates per side in the stowed
wing's own plane, which with the wing make one continuous wall. Skirt below,
upper panel continuing the wing's silhouette rearward, both translucent so the
rear wheel stays visible in the `wheel` camera. They also replace the hockey
stick: `_add_hockey` builds the stick only when `case_*` is absent, so
`bike_params.yaml` keeps `moves/ball_rl.npz` and `tests/test_ball_rl.py`
working untouched while the CAD file gets the real part.

**`case_gap` 5 mm is not a manufacturing allowance.** The wing's inner-bottom
corner swings DOWN before clearing the panel band sideways — 26.06 mm from the
pivot at 38.5° from vertical, leaving the band at z 67.65, i.e. **3.95 mm below
the stowed underside**. `analysis/wing_linkage.py --stick` reports 0.0 mm and
is wrong for this: its 2D wing is a LINE at the outer face, so it has no inner
corner to dip. Trust it for the mechanism, not for panel clearance.

The bumper is retired (commented out, not deleted) — the pads sat at |y| 40–52
mm, outboard of the 75 mm envelope the case now sets.

**`bike_params_cad.yaml` is now as close to the CAD as it gets without an
Onshape read-back**, and is still NOT authoritative. Diffed key by key: physics
is in sync (the actuators block was ported verbatim, comments included);
**7 keys conflict and every one is the CAD file being better sourced** (AHRS
mass 12 g `GUESS` → 19 g `datasheet`, pulley offset and steer-servo station
as-drawn, wing pivot at the belt-clearance station); **nothing is lost** — the
9 keys unique to the authoritative file are the retired bumper (6) and
`payload.electronics`, which the CAD file decomposes into pi/u2d2/power_board.

Net effect on what the controllers feel: mass 1.0162 → 1.0232 kg, CoM +5.8 mm
up and 9.2 mm back. The individual moves largely cancel; it is not a different
bike.

**The wing-pivot warning was wrong and is retracted.** It said the 75 → 130 mm
station left the mechanism unverified pending a self-righting re-run. The pivot's
fore/aft station is not where the wing acts on the ground: the wings are long,
so the contact point is set by the panel's extent, and moving the pivot in x
slides the mechanism without moving the footprint that lifts. Re-run for a
PANEL or pivot-HEIGHT change; not for a fore/aft one.

Switching authority is one line — `DEFAULT_PARAMS` in `params.py` — plus a
digest move, re-baselining the 115 tests keyed to `bike_params`, and lifting the
case panels out of the linkage builder so `--hockey` has a striker without
`--linkage`.

---
