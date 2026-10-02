"""Snail cam for the drop rig (docs/plans/drop-release-rig.md): SVG + DXF, 1:1 mm.

One turn is split into one segment per drop. Each segment is a bottom dwell
at --r-dwell, a ramp up to that drop's top radius, a top flat (--top-deg, the
servo's run-up to speed), and a step straight back down. Turned CLOCKWISE as drawn (looking at the horn), the follower rides
up each ramp and falls off each step: --drops 0.5,1,1.5,2 gives four drops
per turn, in that order.

DROP HEIGHTS ARE SET RELATIVE TO WHERE THE FOLLOWER RESTS. With the wheel on the
sensor the follower sits --gap above the dwell radius, so a segment's top radius
is r_dwell + gap + drop. The cam fixes the DIFFERENCES between the drops
exactly; the servo's mounting height (shims) shifts them all together: a
servo 0.3 mm high gives 0.8/1.3/1.8/2.3 instead of 0.5/1/1.5/2, and anything
from 0.5 mm low to --gap high still leaves four real drops and a clear gap.
force_drop.py measures each real height anyway.

SWAPPABLE: the bore is a D (--bore across the round, --flat across the
flat) that keys onto a hub on the horn, held by one screw into the hub. The
start position is set by hand every run, so cams need no precise indexing.

Each step face leans back --undercut-deg so the falling follower does not drag
down it. Follower: a small flat with a crisp downstream corner, not a roller.

    python bench/drop_cam.py                               # 0.5/1/1.5/2 mm, four per turn
    python bench/drop_cam.py --drops 2.5 --out bench/drop_cam_single
    python bench/drop_cam.py --drops 1,1,1,1 --out bench/drop_cam_1mm_x4
    python bench/drop_cam.py --explain docs/plans/drop-cam-explained.svg   # the figure
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path


def profile(r_dwell, gap, drops, dwell_deg, undercut_deg, top_deg=0.0, n=120):
    """Closed outline [(x, y) mm]. Segment i spans [i, i+1) * 360/N degrees:
    bottom dwell, ramp up to r_dwell + gap + drops[i], top flat, step. Under the
    follower the cam runs through INCREASING angle: clockwise as drawn."""
    seg = 2 * math.pi / len(drops)
    dw, uc, tp = math.radians(dwell_deg), math.radians(undercut_deg), math.radians(top_deg)
    pts = []
    for i, h in enumerate(drops):
        a0 = i * seg
        top = r_dwell + gap + h
        start = a0 - uc                             # under the previous step's lean-back
        for k in range(13):                         # bottom dwell
            a = start + (a0 + dw - start) * k / 12
            pts.append((r_dwell * math.cos(a), r_dwell * math.sin(a)))
        a_ramp = a0 + seg - tp
        for k in range(1, n + 1):                   # ramp
            a = a0 + dw + (a_ramp - a0 - dw) * k / n
            r = r_dwell + (top - r_dwell) * k / n
            pts.append((r * math.cos(a), r * math.sin(a)))
        for k in range(1, 13 if tp else 1):         # top flat
            a = a_ramp + tp * k / 12
            pts.append((top * math.cos(a), top * math.sin(a)))
    return pts                                      # closing edge = the last step face


def d_bore(bore, flat, n=48):
    """D-shaped bore: a circle of diameter `bore` cut flat at `flat` across,
    the flat on the +x side."""
    r = bore / 2
    xf = flat - r
    a_f = math.acos(max(-1.0, min(1.0, xf / r)))
    pts = [(xf, -r * math.sin(a_f))]
    for k in range(n + 1):
        a = a_f + (2 * math.pi - 2 * a_f) * k / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def write_svg(path, outlines, size):
    s = size / 2 + 3
    body = "".join(
        f'  <polygon points="{" ".join(f"{x:.3f},{-y:.3f}" for x, y in o)}" '
        f'fill="none" stroke="black" stroke-width="0.1"/>\n' for o in outlines)
    path.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{2 * s}mm" '
                    f'height="{2 * s}mm" viewBox="{-s} {-s} {2 * s} {2 * s}">\n{body}</svg>\n')


def write_dxf(path, outlines):
    out = ["0", "SECTION", "2", "ENTITIES"]
    for o in outlines:
        for (x0, y0), (x1, y1) in zip(o, o[1:] + o[:1]):
            out += ["0", "LINE", "8", "0", "10", f"{x0:.4f}", "20", f"{y0:.4f}",
                    "11", f"{x1:.4f}", "21", f"{y1:.4f}"]
    out += ["0", "ENDSEC", "0", "EOF"]
    path.write_text("\n".join(out) + "\n")


def explain(path, r_dwell, gap, drops, dwell_deg, undercut_deg, top_deg):
    """The explainer figure for docs/plans: the cam the instant before its
    tallest drop, and the follower's height through one turn."""
    seg = 360.0 / len(drops)
    rest = r_dwell + gap
    S, cx, cy = 9.0, 300, 270
    k_top = max(range(len(drops)), key=lambda i: drops[i])
    rot = math.radians(90 - (k_top + 1) * seg)          # that step's top under the follower

    def scr(x, y):
        xr = x * math.cos(rot) - y * math.sin(rot)
        yr = x * math.sin(rot) + y * math.cos(rot)
        return cx + xr * S, cy - yr * S

    def polar(r, a_deg):
        a = math.radians(a_deg)
        return scr(r * math.cos(a), r * math.sin(a))

    def surface(a):                                     # cam radius under cam angle a
        i, loc = int(a // seg) % len(drops), a % seg
        if loc < dwell_deg:
            return r_dwell
        ramp = seg - dwell_deg - top_deg
        return r_dwell + (gap + drops[i]) * min(1.0, (loc - dwell_deg) / ramp)

    o = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 500" '
         'font-family="Helvetica, Arial, sans-serif" font-size="13">',
         '<rect width="1000" height="500" fill="white"/>',
         '<defs><marker id="ah" markerWidth="10" markerHeight="10" refX="6" refY="3" '
         'orient="auto"><path d="M0,0 L6,3 L0,6 z" fill="#333"/></marker></defs>',
         f'<text x="20" y="30" font-size="17" font-weight="bold">Four-step snail cam: '
         f'{" / ".join(f"{h:g}" for h in drops)} mm drops, one per quarter turn</text>']
    pts = profile(r_dwell, gap, drops, dwell_deg, undercut_deg, top_deg)
    o.append('<polygon points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in (scr(*p) for p in pts))
             + '" fill="#dbe8f5" stroke="#1f4e79" stroke-width="2"/>')
    for r, dash in ((r_dwell, "4 4"), (rest, "2 5")):
        o.append(f'<circle cx="{cx}" cy="{cy}" r="{r * S}" fill="none" stroke="#888" '
                 f'stroke-dasharray="{dash}"/>')
    o.append(f'<circle cx="{cx}" cy="{cy}" r="4" fill="#1f4e79"/>')
    o.append(f'<text x="{cx + 8}" y="{cy + 18}">horn axis</text>')
    for i, h in enumerate(drops):                       # label each step outside the cam
        x, y = polar(r_dwell + gap + h + 4.5, (i + 1) * seg - 4)
        o.append(f'<text x="{x - 14:.0f}" y="{y + 4:.0f}" fill="#b03030" font-weight="bold">'
                 f'{h:g}</text>')
    fx, fy = cx, cy - (rest + drops[k_top]) * S
    o += [f'<rect x="{fx - 60}" y="{fy - 34}" width="120" height="16" fill="#e9d8b8" stroke="#7a5a20"/>',
          f'<text x="{fx + 66}" y="{fy - 22}">arm (wheel end)</text>',
          f'<line x1="{fx}" y1="{fy - 18}" x2="{fx}" y2="{fy}" stroke="#7a5a20" stroke-width="5"/>',
          f'<text x="{fx + 10}" y="{fy - 3}">follower</text>']
    R = (r_dwell + gap + max(drops) + 7) * S
    a0, a1 = math.radians(210), math.radians(150)
    o.append(f'<path d="M {cx + R * math.cos(a0):.1f} {cy - R * math.sin(a0):.1f} A {R} {R} 0 0 1 '
             f'{cx + R * math.cos(a1):.1f} {cy - R * math.sin(a1):.1f}" fill="none" stroke="#333" '
             f'stroke-width="2" marker-end="url(#ah)"/>')
    o.append(f'<text x="{cx - R - 10:.0f}" y="{cy + 20:.0f}" text-anchor="end">turns</text>'
             f'<text x="{cx - R - 10:.0f}" y="{cy + 36:.0f}" text-anchor="end">clockwise</text>')
    o += [f'<text x="40" y="{cy + R + 30:.0f}" fill="#555">Red: each step\'s drop in mm. Dashed: '
          f'the {r_dwell:g} mm dwell, and where the follower rests ({gap:g} mm above it).</text>',
          f'<text x="40" y="{cy + R + 48:.0f}" fill="#555">Drawn the instant before the '
          f'{drops[k_top]:g} mm drop. After each step the follower hangs clear of the cam.</text>']
    gx, gy, gw, gh = 600, 90, 360, 250                  # right: follower height through a turn
    lo, hi = -gap - 0.5, max(drops) + 0.5

    def G(t, h):
        return gx + t / 360 * gw, gy + gh - (h - lo) / (hi - lo) * gh

    o += [f'<text x="{gx}" y="{gy - 25}" font-weight="bold">Follower height through one turn</text>',
          f'<line x1="{gx}" y1="{gy + gh}" x2="{gx + gw}" y2="{gy + gh}" stroke="#333"/>',
          f'<line x1="{gx}" y1="{gy}" x2="{gx}" y2="{gy + gh}" stroke="#333"/>']
    park = (k_top + 1) * seg + dwell_deg / 2          # start just past the tallest step
    ts = [t / 2 for t in range(721)]
    cam = [(t, surface(park + t) - rest) for t in ts]
    fol = [(t, max(h, 0.0)) for t, h in cam]
    for line, style in ((cam, 'stroke="#1f4e79" stroke-width="2" stroke-dasharray="5 3"'),
                        (fol, 'stroke="#b03030" stroke-width="3"')):
        o.append('<polyline points="' + " ".join(f"{G(t, h)[0]:.1f},{G(t, h)[1]:.1f}" for t, h in line)
                 + f'" fill="none" {style}/>')
    for i in range(len(drops)):
        t_step = (k_top + 1 + i + 1) * seg - park
        h = drops[(k_top + 1 + i) % len(drops)]
        x, y = G(t_step % 360 or 360, h)
        o.append(f'<text x="{x - 6:.0f}" y="{y - 8:.0f}" fill="#b03030" text-anchor="end">'
                 f'drop {h:g}</text>')
    x, y = G(0, 0)
    o.append(f'<text x="{x + 4}" y="{y + 16}" fill="#b03030">wheel resting</text>')
    x, y = G(4, -gap)
    o.append(f'<text x="{x}" y="{y + 18}" fill="#1f4e79">cam surface under the follower (dashed)</text>')
    for t in (0, 90, 180, 270, 360):
        x, y = G(t, lo)
        o.append(f'<text x="{x - 8}" y="{y + 18}" fill="#555">{t}</text>')
    o += [f'<text x="{gx + gw / 2 - 70}" y="{gy + gh + 40}" fill="#555">cam turn [deg]</text>',
          f'<text x="{gx}" y="{gy + gh + 72}" fill="#555">The steps fix the drops\' differences; '
          f'the servo\'s</text>',
          f'<text x="{gx}" y="{gy + gh + 90}" fill="#555">height (shims) shifts all four '
          f'together.</text>', '</svg>']
    Path(path).write_text("\n".join(o) + "\n")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--drops", default="0.5,1,1.5,2", help="mm, in rotation order")
    ap.add_argument("--r-dwell", type=float, default=12.0)
    ap.add_argument("--gap", type=float, default=1.0, help="follower above the dwell, wheel resting")
    ap.add_argument("--dwell-deg", type=float, default=20.0, help="bottom flat, per segment")
    ap.add_argument("--top-deg", type=float, default=30.0, help="top flat (run-up), per segment")
    ap.add_argument("--undercut-deg", type=float, default=3.0)
    ap.add_argument("--bore", type=float, default=8.0, help="D-bore diameter, mm (0: none)")
    ap.add_argument("--flat", type=float, default=7.0, help="D-bore across the flat, mm")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent / "drop_cam"))
    ap.add_argument("--explain", default=None,
                    help="also write the explainer figure here (docs/plans/drop-cam-explained.svg)")
    args = ap.parse_args()
    drops = [float(x) for x in args.drops.split(",")]
    seg = 360.0 / len(drops)
    ramp_deg = seg - args.dwell_deg - args.top_deg
    if args.dwell_deg <= args.undercut_deg or ramp_deg < 10:
        raise SystemExit("--dwell-deg must exceed --undercut-deg, and leave >= 10 deg of ramp")
    outlines = [profile(args.r_dwell, args.gap, drops, args.dwell_deg, args.undercut_deg,
                        args.top_deg)]
    if args.bore:
        outlines.append(d_bore(args.bore, args.flat))
    r_max = args.r_dwell + args.gap + max(drops)
    out = Path(args.out)
    write_svg(out.with_suffix(".svg"), outlines, 2 * r_max)
    write_dxf(out.with_suffix(".dxf"), outlines)
    if args.explain:
        explain(args.explain, args.r_dwell, args.gap, drops, args.dwell_deg, args.undercut_deg,
                args.top_deg)
    print(f"wrote {out}.svg / .dxf: {len(drops)} segments of {seg:g} deg: dwell "
          f"{args.dwell_deg:g}, ramp {ramp_deg:g}, top flat {args.top_deg:g} deg; "
          f"follower resting {args.gap:g} mm above the {args.r_dwell:g} mm dwell")
    for i, h in enumerate(drops):
        rise = args.gap + h
        slope = rise / (math.radians(ramp_deg) * (args.r_dwell + rise / 2))
        print(f"  step {i + 1}: drop {h:g} mm, top r {args.r_dwell + rise:g} mm, step at "
              f"{(i + 1) * seg:g} deg, ramp slope {slope:.3f} "
              f"({math.degrees(math.atan(slope)):.1f} deg pressure angle)")


if __name__ == "__main__":
    main()
