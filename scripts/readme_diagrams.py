#!/usr/bin/env python3
"""The README's two diagrams, as SVG: the control scheme and the curriculum.

    python scripts/readme_diagrams.py                 # both, for the pointer
    python scripts/readme_diagrams.py --policy NAME   # another export

Writes docs/media/control_scheme.svg and docs/media/curriculum.svg.

Everything drawn is READ, not typed: the observation layout and the network
from the policy's move yaml (`obs_layout`, `trained.arch`), the action bounds
and the command sampler's constants from the training config the move names
(`trained.graph.config`), whose sha256 is checked against the one the move
recorded -- a config edited since the run would otherwise be drawn as if it
had trained it. The family mix is computed with the sampler's own formula
(`gpu/commands.py sample_family`, `GeneralEnv._family_weights`).

The bike icons are the drivetrain-traces page's (analysis/policy/templates/
drivetrain_traces.html: BACK, TOP, the arc arrow), redrawn here at the same
40-unit scale. The wings are left out of every drawing on purpose: the
policy neither sees nor drives them.

Colours follow the sim's ground dial: commanded heading green, commanded
velocity orange. Both files carry a prefers-color-scheme block, so they read
on GitHub's light and dark themes.
"""
from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "docs" / "media"

FONT = "-apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif"
STYLE = f"""
<style>
  svg {{ --ink:#1f2328; --muted:#59636e; --rule:#d1d9e0; --card:#f6f8fa;
         --head:#1a7f37; --vel:#bc4c00; --meas:#0969da; --est:#8250df;
         --out:#cf222e; --box:#24292f; --boxink:#f6f8fa; }}
  @media (prefers-color-scheme: dark) {{
    svg {{ --ink:#e6edf3; --muted:#9198a1; --rule:#3d444d; --card:#151b23;
           --head:#3fb950; --vel:#f0883e; --meas:#4493f8; --est:#ab7df8;
           --out:#f85149; --box:#e6edf3; --boxink:#0d1117; }} }}
  text {{ font-family:{FONT}; fill:var(--ink); }}
  .t {{ font-size:15px; font-weight:600; }}
  .s {{ font-size:13px; fill:var(--muted); }}
  .h {{ font-size:12px; font-weight:600; letter-spacing:.08em; fill:var(--muted); }}
  .card {{ fill:var(--card); stroke:var(--rule); }}
  .ic {{ fill:none; stroke:var(--ink); stroke-width:1.6; stroke-linecap:round;
         stroke-linejoin:round; }}
  .ic .faint {{ stroke:var(--muted); opacity:.55; }}
  .wire {{ fill:none; stroke:var(--muted); stroke-width:1.4; }}
</style>
"""


# ---- the icons, on the drivetrain-traces page's 40-unit grid ---------------
def arrow(cx, cy, rx, ry, a0, a1, cls="", width=None):
    """An elliptical arc a0 -> a1 (deg, y-down) with a head at a1: the
    template's `arrow`."""
    R = math.pi / 180
    pt = lambda a: (cx + rx * math.cos(a * R), cy + ry * math.sin(a * R))
    (x0, y0), (x1, y1) = pt(a0), pt(a1)
    s = 1 if a1 > a0 else -1
    b, (bx, by) = a1, (x1, y1)
    while math.hypot(bx - x1, by - y1) < 3.2 and abs(b - a0) > 1:
        b -= s
        bx, by = pt(b)
    n = math.hypot(x1 - bx, y1 - by) or 1
    tx, ty = (x1 - bx) / n, (y1 - by) / n
    h = lambda k: f"{x1 - 3.2 * tx + k * 2.4 * ty:.1f} {y1 - 3.2 * ty - k * 2.4 * tx:.1f}"
    big = 1 if abs(a1 - a0) > 180 else 0
    st = f' style="stroke:{cls}"' if cls else ""
    return (f'<path{st} d="M{x0:.1f} {y0:.1f} A{rx} {ry} 0 {big} {1 if s > 0 else 0} '
            f'{x1:.1f} {y1:.1f}"/><path{st} d="M{h(1)} L{x1:.1f} {y1:.1f} L{h(-1)}"/>')


def line_arrow(x0, y0, x1, y1, col):
    """A straight arrow, head at (x1, y1)."""
    n = math.hypot(x1 - x0, y1 - y0) or 1
    tx, ty = (x1 - x0) / n, (y1 - y0) / n
    h = lambda k: f"{x1 - 3.4 * tx + k * 2.6 * ty:.1f} {y1 - 3.4 * ty - k * 2.6 * tx:.1f}"
    return (f'<path style="stroke:{col}" d="M{x0:.1f} {y0:.1f} L{x1:.1f} {y1:.1f}"/>'
            f'<path style="stroke:{col}" d="M{h(1)} L{x1:.1f} {y1:.1f} L{h(-1)}"/>')


BACK = ('<rect x="17.5" y="15" width="5" height="21" rx="1.5"/>'
        '<path d="M12.2 24.8 A11 11 0 1 1 27.8 24.8"/>')


def TOP(steer=0.0, faint_body=True):
    body = '<rect class="faint" x="13" y="3" width="14" height="34" rx="7"/>' if faint_body else ""
    return (body + '<rect x="18" y="25" width="4" height="10" rx="1"/>'
            f'<rect x="18" y="5" width="4" height="10" rx="1" transform="rotate({steer} 20 10)"/>')


HEAD, VEL, MEAS, EST, OUT = "var(--head)", "var(--vel)", "var(--meas)", "var(--est)", "var(--out)"

ICONS = {
    # the command: the bike, a ghost of it at the commanded heading, and the
    # velocity along that heading
    "command": (f'<g class="faint" transform="rotate(40 20 20)">{TOP(0)}</g>'
                f'<g transform="translate(0 0)">{TOP(0, faint_body=False)}</g>'
                # out past the end of the ghost's pill (17 from its centre)
                + line_arrow(20, 20, 20 + 23 * math.sin(math.radians(40)),
                             20 - 23 * math.cos(math.radians(40)), VEL)
                # the heading drawn last, inside the velocity's tip; it ends
                # 14 deg short of the velocity's line, ~2 stroke widths clear
                + arrow(20, 20, 22.4, 22.4, -100, -64, HEAD)),
    # the AHRS: the bike from behind, leaning, with its roll arrow
    "ahrs": ('<path class="faint" d="M6 37.5 H34"/>'
             f'<g transform="rotate(-14 20 37)">{BACK}</g>'
             + arrow(15.5, 18, 15.5, 15.5, -35, -125, MEAS)),
    # the steering encoder: from above, the front wheel turned
    "steer": f'<g transform="translate(0 2)">{TOP(-28)}</g>' + arrow(20, 12, 10.5, 10.5, -35, -145, MEAS),
    # odometry: from above, the velocity it estimates, forward and sideways
    "odometry": (TOP(0) + line_arrow(20, 20, 20, -6, EST) + line_arrow(20, 20, 36, 20, EST)),
    # the previous action: a loop
    "prev": arrow(20, 20, 12, 12, -60, 230, "var(--muted)"),
    # steer rate out
    "steer_out": f'<g transform="translate(0 2)">{TOP(-28)}</g>' + arrow(20, 12, 10.5, 10.5, -35, -145, OUT),
    # common mode: the rear wheel from the side, rolling
    "hub": ('<circle cx="20" cy="20" r="4"/><circle class="faint" cx="20" cy="20" r="10"/>'
            + "".join(f'<rect x="-4" y="-2.2" width="8" height="4.4" rx="2" transform="translate('
                      f'{20 + 13 * math.cos(k * math.pi / 4):.1f} {20 + 13 * math.sin(k * math.pi / 4):.1f}) '
                      f'rotate({k * 45 + 90})"/>' for k in range(8))
            + '<path class="faint" d="M2 37.5 H38"/>'
            + arrow(20, 20, 19, 19, 200, 330, OUT)),
    # differential: the bike from behind, the contact patch driven sideways
    "diff": (f'<g class="faint">{BACK}</g><rect x="17.5" y="15" width="5" height="21" rx="1.5"/>'
             '<path class="faint" d="M4 37.5 H36"/>'
             + line_arrow(23, 42.5, 35, 42.5, OUT) + line_arrow(17, 42.5, 5, 42.5, OUT)),
}


def icon(key, x, y, size=64):
    return (f'<g class="ic" transform="translate({x} {y}) scale({size / 40})">'
            f'{ICONS[key]}</g>')


# ---- inputs: the move and its config ----------------------------------------
def load(policy):
    move_p = ROOT / "moves" / f"{policy}.yaml"
    move = yaml.safe_load(move_p.read_text())
    tr = move["trained"]
    cfg_p = ROOT / tr["graph"]["config"]
    raw = cfg_p.read_bytes()
    want = tr["graph"].get("config_sha256")
    got = hashlib.sha256(raw).hexdigest()
    if want and want != got:
        sys.exit(f"{cfg_p.relative_to(ROOT)} has changed since {policy} trained "
                 f"(sha256 {got[:12]} != {want[:12]}); check out the version it used")
    return move, yaml.safe_load(raw)


def groups(move):
    """The observation, split by where each entry comes from. Every name in
    obs_layout must land in exactly one group, or this refuses."""
    spec = [
        ("command", "Command", "from the operator", HEAD,
         {"v_cmd_lon": "velocity, forward", "v_cmd_lat": "velocity, sideways",
          "sin_psi_err": "heading error (sin)", "cos_psi_err": "heading error (cos)"}),
        ("ahrs", "Measured", "AHRS", MEAS,
         {"roll": "roll", "roll_rate": "roll rate", "pitch": "pitch",
          "pitch_rate": "pitch rate", "yaw_rate": "yaw rate"}),
        ("steer", "Measured", "steering servo encoder", MEAS,
         {"sin2steer": "steer angle (sin 2δ)", "cos2steer": "steer angle (cos 2δ)",
          "steer_rate": "steer rate"}),
        ("odometry", "Estimated", "odometry, from the encoders", EST,
         {"v_lon": "velocity, forward", "v_lat": "velocity, sideways"}),
        ("prev", "Feedback", "previous action", "var(--muted)",
         {"prev_steer_rate": "steer rate", "prev_hub": "common drive",
          "prev_diff": "differential drive"}),
    ]
    names = list(move["obs_layout"])
    out, seen = [], set()
    for key, title, sub, col, m in spec:
        here = [n for n in names if n in m]
        seen.update(here)
        if here:
            out.append((key, title, sub, col, [m[n] for n in here]))
    left = [n for n in names if n not in seen]
    if left:
        sys.exit(f"observation entries with no group: {left}")
    return out, len(names)


# ---- diagram 1: the control scheme ------------------------------------------
def control_scheme(move, cfg):
    gs, n_obs = groups(move)
    ab = cfg["env"]["action_bounds"]
    arch = move["trained"]["arch"]
    hz = move["control_rate_hz"]
    W, H = 1000, 620
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" '
         f'role="img" aria-label="The policy\'s observations, network and actions">', STYLE]
    o.append(f'<text class="h" x="20" y="24">OBSERVED · {n_obs} VALUES</text>')
    o.append('<text class="h" x="508" y="24">CONTROL POLICY</text>')
    o.append('<text class="h" x="692" y="24">ACTIONS · 3 VALUES</text>')

    # left: the observation groups
    card_h, gap, y0 = 100, 10, 40
    ins = []
    for i, (key, title, sub, col, items) in enumerate(gs):
        y = y0 + i * (card_h + gap)
        o.append(f'<rect class="card" x="10" y="{y}" width="420" height="{card_h}" rx="10"/>')
        o.append(f'<rect x="10" y="{y}" width="5" height="{card_h}" rx="2" style="fill:{col}"/>')
        o.append(icon(key, 28, y + 18))
        o.append(f'<text class="t" x="110" y="{y + 28}">{title} '
                 f'<tspan class="s">· {sub} · {len(items)}</tspan></text>')
        half = (len(items) + 1) // 2
        for j, cols_ in enumerate((items[:half], items[half:])):
            for k, it in enumerate(cols_):
                o.append(f'<text class="s" x="{110 + j * 160}" y="{y + 52 + k * 18}">{it}</text>')
        ins.append(y + card_h / 2)

    # centre: the network
    bx, by, bw, bh = 500, 170, 150, 270
    o.append(f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="12" style="fill:var(--box)"/>')
    # "Linear(17,128) -> MinGRU(128) -> Linear(128,3), no biases"
    parts = [re.sub(r"\)\s*,.*$", ")", p.strip()) for p in arch.split("->")]
    rows = [(p, "memory" if "GRU" in p else "") for p in parts]
    for k, (txt, note) in enumerate(rows):
        yy = by + 70 + k * 70
        o.append(f'<text x="{bx + bw / 2}" y="{yy}" text-anchor="middle" '
                 f'style="fill:var(--boxink);font-size:14px;font-weight:600">{txt}</text>')
        if note:
            o.append(f'<text x="{bx + bw / 2}" y="{yy + 18}" text-anchor="middle" '
                     f'style="fill:var(--boxink);font-size:12px;opacity:.7">{note}</text>')
        if k < len(rows) - 1:
            o.append(f'<path d="M{bx + bw / 2} {yy + (26 if note else 10)} V{yy + 48}" '
                     f'style="stroke:var(--boxink);stroke-width:1.4;opacity:.5"/>')
    o.append(f'<text x="{bx + bw / 2}" y="{by + 34}" text-anchor="middle" '
             f'style="fill:var(--boxink);font-size:12px;opacity:.7">every {1000 // hz} ms ({hz} Hz)</text>')
    for k, y in enumerate(ins):
        ty = by + 40 + k * (bh - 80) / max(1, len(ins) - 1)
        o.append(f'<path class="wire" d="M430 {y} C470 {y} 460 {ty} {bx} {ty}"/>')

    # right: the actions
    acts = [
        ("steer_out", "Steer rate", f"±{ab['steer_rate_max']:g} rad/s",
         ["integrated into the steering", "servo's position target"]),
        ("hub", "Common mode", f"±{ab['hub_max']:g} m/s",
         ["both drive servos together:", "the wheel rolls"]),
        ("diff", "Differential", f"±{ab['diff_max']:g} rad/s",
         ["the drive servos against each", "other: the rollers spin, and",
          "the wheel drives sideways"]),
    ]
    outs = []
    for i, (key, title, rng, lines) in enumerate(acts):
        y = 110 + i * 120
        o.append(f'<rect class="card" x="690" y="{y}" width="300" height="108" rx="10"/>')
        o.append(f'<rect x="985" y="{y}" width="5" height="108" rx="2" style="fill:{OUT}"/>')
        o.append(icon(key, 704, y + 22))
        o.append(f'<text class="t" x="786" y="{y + 28}">{title}</text>')
        o.append(f'<text class="s" x="786" y="{y + 46}" style="fill:var(--out)">{rng}</text>')
        for k, ln in enumerate(lines):
            o.append(f'<text class="s" x="786" y="{y + 66 + k * 16}">{ln}</text>')
        outs.append(y + 54)
    for k, y in enumerate(outs):
        sy = by + 70 + k * 65
        o.append(f'<path class="wire" d="M{bx + bw} {sy} C{bx + bw + 20} {sy} 665 {y} 690 {y}"/>')

    # the loop back into the observation
    py = y0 + 4 * (card_h + gap) + card_h
    o.append(f'<path class="wire" style="stroke-dasharray:4 4" '
             f'd="M850 {110 + 2 * 120 + 108} V{py + 22} H220 V{py}"/>')
    o.append(f'<path class="wire" d="M215 {py + 7} L220 {py} L225 {py + 7}"/>')
    o.append(f'<text class="s" x="540" y="{py + 16}" text-anchor="middle">'
             f'fed back at the next step</text>')
    o.append("</svg>")
    return "\n".join(o)


# ---- diagram 2: the curriculum ----------------------------------------------
def family_mix(fam, d):
    """(hold, turn_in_place, straight, moving_turn) at difficulty d -- the
    sampler's own formula."""
    ramp = lambda on, full: 1.0 if d >= full else 0.0 if d <= on else (d - on) / (full - on)
    p_hold = fam["hold_min"] + (fam["hold_max"] - fam["hold_min"]) * (1 - d) ** fam["hold_decay"]
    r = {k: fam[k]["weight"] * ramp(fam[k]["onset"], fam[k]["full"])
         for k in ("turn_in_place", "straight", "moving_turn")}
    tot = sum(r.values())
    if tot <= 0:
        return 1.0, 0.0, 0.0, 0.0
    sh = (1 - p_hold) / tot
    return p_hold, r["turn_in_place"] * sh, r["straight"] * sh, r["moving_turn"] * sh


def sector(cx, cy, r0, r1, a0, a1):
    """An annular sector between radii r0..r1 and compass angles a0..a1 (deg,
    0 = up = straight ahead, + = left)."""
    pt = lambda r, a: (cx - r * math.sin(math.radians(a)), cy - r * math.cos(math.radians(a)))
    big = 1 if abs(a1 - a0) > 180 else 0
    (x0, y0), (x1, y1) = pt(r1, a0), pt(r1, a1)
    (x2, y2), (x3, y3) = pt(r0, a1), pt(r0, a0)
    return (f"M{x0:.1f} {y0:.1f} A{r1} {r1} 0 {big} 0 {x1:.1f} {y1:.1f} "
            f"L{x2:.1f} {y2:.1f} A{r0} {r0} 0 {big} 1 {x3:.1f} {y3:.1f} Z")


def velocity_directions(s_lo, s_hi):
    """Compass directions (deg, [0, 360)) a moving turn's velocity can point:
    forward along a heading step of +-[s_lo, s_hi], or reverse along it
    (+180). Returned as merged intervals."""
    raw = []
    for base in (0.0, 180.0):
        for a0, a1 in ((base + s_lo, base + s_hi), (base - s_hi, base - s_lo)):
            a0, a1 = a0 % 360, a1 % 360
            raw += [(a0, a1)] if a0 <= a1 else [(a0, 360.0), (0.0, a1)]
    raw.sort()
    out = []
    for a0, a1 in raw:
        if out and a0 <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], a1))
        else:
            out.append((a0, a1))
    # a sector over 359.9 deg would draw as nothing; split a full circle
    return [p for a0, a1 in out
            for p in (((a0, (a0 + a1) / 2), ((a0 + a1) / 2, a1)) if a1 - a0 > 300 else ((a0, a1),))]


def arc(cx, cy, r, a0, a1):
    pt = lambda a: (cx - r * math.sin(math.radians(a)), cy - r * math.cos(math.radians(a)))
    (x0, y0), (x1, y1) = pt(a0), pt(a1)
    return f"M{x0:.1f} {y0:.1f} A{r} {r} 0 {1 if abs(a1 - a0) > 180 else 0} 0 {x1:.1f} {y1:.1f}"


def curriculum(move, cfg):
    env = cfg["env"]
    fam = env["cmd_families"]
    v_max = float(env["v_max"])
    v_lo = float(env.get("cmd_v_lo", 0.1))
    step_lo = float(env.get("cmd_step_lo_deg", 10.0))
    levels = [0.0, 0.25, 0.5, 0.75, 1.0]
    W, H, pw = 1000, 405, 200
    R = 74                                   # disc radius, px, = v_max
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" '
         f'role="img" aria-label="The training curriculum: which commands occur at each difficulty">',
         STYLE]
    o.append('<text class="h" x="10" y="24">COMMANDS DRAWN AT EACH DIFFICULTY</text>')
    labels = ("hold", "turn in place", "straight", "moving turn")
    colours = ("var(--muted)", HEAD, VEL, VEL)
    opac = (0.5, 1.0, 1.0, 0.45)
    for i, d in enumerate(levels):
        cx, cy = i * pw + pw / 2, 140
        v_hi = v_max * (0.25 + 0.75 * d)
        s_hi = step_lo + (180.0 - step_lo) * d
        r_lo, r_hi = R * v_lo / v_max, R * v_hi / v_max
        mix = family_mix(fam, d)
        o.append(f'<circle cx="{cx}" cy="{cy}" r="{R}" style="fill:var(--card);stroke:var(--rule)"/>')
        o.append(f'<circle cx="{cx}" cy="{cy}" r="{R / 2}" style="fill:none;stroke:var(--rule);'
                 f'stroke-dasharray:2 3"/>')
        # moving turns: velocity along the new heading, forward or reverse,
        # drawn as the UNION of the directions covered so no overlap doubles
        if mix[3] > 0:
            for lo_, hi_ in velocity_directions(step_lo, min(s_hi, 180)):
                o.append(f'<path d="{sector(cx, cy, r_lo, r_hi, lo_, hi_)}" '
                         f'style="fill:var(--vel);opacity:.28"/>')
        # straight: along the bike's own axis
        if mix[2] > 0:
            for sgn in (1, -1):
                o.append(f'<path d="M{cx} {cy - sgn * r_lo:.1f} V{cy - sgn * r_hi:.1f}" '
                         f'style="stroke:var(--vel);stroke-width:5;stroke-linecap:round"/>')
        # turns in place: a heading change at zero speed
        if mix[1] > 0:
            for sgn in (1, -1):
                a0, a1 = sorted((sgn * step_lo, sgn * min(s_hi, 180)))
                o.append(f'<path d="{arc(cx, cy, 20, a0, a1)}" style="fill:none;'
                         f'stroke:var(--head);stroke-width:4;stroke-linecap:round"/>')
        o.append(f'<g class="ic" transform="translate({cx - 12} {cy - 12}) scale(0.6)">{TOP(0)}</g>')
        if i == 0:
            for r_, lab in ((R / 2, f"{v_max / 2:g}"), (R, f"{v_max:g} m/s")):
                o.append(f'<text class="s" x="{cx + r_ * 0.72 + 3:.1f}" y="{cy + r_ * 0.72 + 12:.1f}" '
                         f'style="font-size:11px">{lab}</text>')
        o.append(f'<text class="t" x="{cx}" y="{cy - R - 12}" text-anchor="middle">'
                 f'difficulty {d:g}</text>')
        o.append(f'<text class="s" x="{cx}" y="{cy + R + 22}" text-anchor="middle">'
                 f'speed {v_lo:g}–{v_hi:.2f} m/s</text>')
        o.append(f'<text class="s" x="{cx}" y="{cy + R + 40}" text-anchor="middle">'
                 + (f'turns {step_lo:g}–{s_hi:.0f}°' if mix[1] + mix[3] > 0 else 'no turns')
                 + '</text>')
        # the mix
        bx, bw, by = cx - 80, 160, cy + R + 56
        x = bx
        for k, p in enumerate(mix):
            if p <= 0:
                continue
            o.append(f'<rect x="{x:.1f}" y="{by}" width="{bw * p:.1f}" height="12" '
                     f'style="fill:{colours[k]};opacity:{opac[k]}"/>')
            x += bw * p
        o.append(f'<rect x="{bx}" y="{by}" width="{bw}" height="12" rx="2" '
                 f'style="fill:none;stroke:var(--rule)"/>')
        for k, p in enumerate(mix):
            o.append(f'<text class="s" x="{bx}" y="{by + 30 + k * 16}">{labels[k]}</text>')
            o.append(f'<text class="s" x="{bx + bw}" y="{by + 30 + k * 16}" text-anchor="end">'
                     f'{100 * p:.0f}%</text>')
    # legend
    ly = H - 18
    o.append(f'<path d="M14 {ly - 4} H34" style="stroke:var(--vel);stroke-width:5;stroke-linecap:round"/>')
    o.append(f'<text class="s" x="42" y="{ly}">straight: velocity along the bike\'s axis, forward or back</text>')
    o.append(f'<rect x="420" y="{ly - 11}" width="20" height="12" style="fill:var(--vel);opacity:.28"/>')
    o.append(f'<text class="s" x="448" y="{ly}">moving turn: velocity along the new heading</text>')
    o.append(f'<path d="{arc(770, ly + 14, 20, -40, 40)}" style="fill:none;stroke:var(--head);'
             f'stroke-width:4;stroke-linecap:round"/>')
    o.append(f'<text class="s" x="798" y="{ly}">turn in place: heading only</text>')
    o.append("</svg>")
    return "\n".join(o)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", default=None,
                    help="moves/NAME. Default: control.general_move")
    a = ap.parse_args()
    sys.path.insert(0, str(ROOT / "src"))
    from aow_sim.params import load_params
    policy = a.policy or load_params()["control"]["general_move"]
    move, cfg = load(policy)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, svg in (("control_scheme", control_scheme(move, cfg)),
                      ("curriculum", curriculum(move, cfg))):
        p = OUT_DIR / f"{name}.svg"
        p.write_text(svg + "\n")
        print(f"{p.relative_to(ROOT)}  {p.stat().st_size // 1024} KB  ({policy})")


if __name__ == "__main__":
    main()
