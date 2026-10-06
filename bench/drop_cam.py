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

WAVE CAM (--tones, or --lobes N): no steps, for shaking the arm to test the
AHRS (drop-release-rig.md, "AHRS mode"). The follower's nose rides a smooth
sum of tones -- N lobes per turn, pp peak to peak each, never lower than
--lift-min-mm above where
it rests with the wheel on the sensor -- so the wheel never touches and
nothing drops. Same follower height and the same envelope as the drop cam
(the drop cam's tops reach r_dwell + gap + 2 = 15 mm). The follower is a
ROUNDED NOSE of radius --nose-r-mm, centred over the cam, its lowest point
--nose-below-mm under the old pad's face (the jam-on tip's); the profile is the
nose centre's path offset inward by the nose radius. Either direction works.

    python bench/drop_cam.py --lobes 8 --pp-mm 0.8 --out bench/wave_cam_8x08
    python bench/drop_cam.py --tones 3:1.0,11:0.2:90 --out bench/wave_cam_two_tone
    python bench/drop_cam.py --tones none --out bench/wave_cam_blank   # the control
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


def parse_tones(text):
    """"8:0.8" or "3:1.0,11:0.2:90" -> [(lobes, pp_mm, phase_deg)]; "none" -> [] (a
    plain circle: the servo turns, nothing moves)."""
    if text.strip().lower() in ("", "none", "blank"):
        return []
    out = []
    for part in text.split(","):
        f = part.split(":")
        out.append((int(f[0]), float(f[1]), float(f[2]) if len(f) > 2 else 0.0))
    return out


def _lift_terms(tones, a):
    """The tones' sum s, s', s'' at cam angle a (rad), mm per rad^k."""
    s = d1 = d2 = 0.0
    for n, pp, ph in tones:
        x = n * a + math.radians(ph)
        s += pp / 2 * (1 - math.cos(x))
        d1 += pp / 2 * n * math.sin(x)
        d2 += pp / 2 * n * n * math.cos(x)
    return s, d1, d2


def _grid(tones, n=None):
    n = n or max(1440, 120 * max((t[0] for t in tones), default=1))
    return [2 * math.pi * k / n for k in range(n)]


def wave_pitch(rest, nose_r, lift_min, tones):
    """The nose CENTRE's distance from the cam axis, and its first two
    derivatives, as functions of cam angle: resting nose bottom `rest`, plus
    the nose radius, plus lift_min, plus the tones shifted so their lowest
    point is 0."""
    s0 = min(_lift_terms(tones, a)[0] for a in _grid(tones)) if tones else 0.0
    base = rest + nose_r + lift_min - s0
    R = lambda a: base + _lift_terms(tones, a)[0]          # noqa: E731
    R1 = lambda a: _lift_terms(tones, a)[1]                # noqa: E731
    R2 = lambda a: _lift_terms(tones, a)[2]                # noqa: E731
    return R, R1, R2


def valley_angle(tones):
    """The cam angle of the lowest lift (where the follower is drawn)."""
    if not tones:
        return 0.0
    return min(_grid(tones), key=lambda a: _lift_terms(tones, a)[0])


def wave_profile(rest, nose_r, lift_min, tones, n=None):
    """Closed outline [(x, y) mm] of a wave cam for a round-nosed follower: the
    pitch curve (nose centre) offset inward by the nose radius, along its
    normal. A single tone with phase 0 has a valley at angle 0."""
    R, R1, _ = wave_pitch(rest, nose_r, lift_min, tones)
    pts = []
    for a in _grid(tones, n):
        r, dr = R(a), R1(a)
        tx, ty = dr * math.cos(a) - r * math.sin(a), dr * math.sin(a) + r * math.cos(a)
        t = math.hypot(tx, ty)
        nx, ny = ty / t, -tx / t                    # outward normal of a CCW curve
        pts.append((r * math.cos(a) - nose_r * nx, r * math.sin(a) - nose_r * ny))
    return pts


def wave_report(rest, nose_r, lift_min, tones, fall_ms2, servo_dps, r_follower, r_contact):
    """Geometry checks and what the cam does to the arm. Returns (lines, ok)."""
    R, R1, R2 = wave_pitch(rest, nose_r, lift_min, tones)
    out, ok = [], True
    rho_min, phi_max, d2_neg, d2_abs, d1_abs = 1e9, 0.0, 0.0, 0.0, 0.0
    for a in _grid(tones, 7200):
        r, r1, r2 = R(a), R1(a), R2(a)
        den = r * r + 2 * r1 * r1 - r * r2
        if den > 0:                                  # convex: the profile is the pitch less the nose
            rho_min = min(rho_min, (r * r + r1 * r1) ** 1.5 / den)
        phi_max = max(phi_max, math.degrees(math.atan2(abs(r1), r)))
        d2_neg, d2_abs, d1_abs = max(d2_neg, -r2), max(d2_abs, abs(r2)), max(d1_abs, abs(r1))
    prof = wave_profile(rest, nose_r, lift_min, tones)
    rmin = min(math.hypot(*q) for q in prof)
    rmax = max(math.hypot(*q) for q in prof)
    span = rmax - rmin
    out.append(f"nose r {nose_r:g} mm; nose {lift_min:g}-{lift_min + span:.2f} mm over rest "
               f"({r_contact / r_follower * lift_min:.2f}-{r_contact / r_follower * (lift_min + span):.2f}"
               f" at the contact); cam r {rmin:.2f}-{rmax:.2f} mm")
    if not tones:
        out.append("  a plain circle: the servo and its gears run, the arm does not move")
        return out, True
    if rho_min - nose_r < 1.0:
        ok = False
        out.append(f"  FAIL: the cam curves at {rho_min - nose_r:.2f} mm under the nose (>= 1 "
                   f"wanted): fewer lobes, less pp, or a smaller nose")
    else:
        out.append(f"  tightest convex curve {rho_min - nose_r:.2f} mm at the cam")
    out.append(f"  steepest pressure angle {phi_max:.1f} deg")
    if phi_max > 30:
        ok = False
        out.append("  FAIL: pressure angle over 30 deg; the follower may jam or chatter")
    # radial follower: y = R(Omega t), so its acceleration is R'' Omega^2; it
    # leaves the cam where that pulls DOWN harder than the arm falls
    rev_sep = math.sqrt(fall_ms2 / (d2_neg * 1e-3)) / (2 * math.pi) if d2_neg > 0 else 1e9
    rev_max = servo_dps / 360
    out.append(f"  follower leaves the cam above {rev_sep:.2f} rev/s (falls at {fall_ms2:g} m/s^2); "
               f"the servo reaches {rev_max:.2f} rev/s at {servo_dps:g} deg/s")
    tone_hdr = " ".join(f"{n}x" for n, _, _ in tones)
    out.append(f"  {'rev/s':>6} {'Hz (' + tone_hdr + ')':>18} {'acc pk follower':>16} "
               f"{'at contact':>11} {'arm rate pk':>12}")
    for rev in (0.1, 0.25, 0.5, 1.0, 1.5, min(rev_max, 0.97 * rev_sep)):
        if rev > min(rev_max, 0.97 * rev_sep) + 1e-9:
            continue
        w = 2 * math.pi * rev
        acc = d2_abs * 1e-3 * w * w / 9.81 * 1e3
        rate = math.degrees(d1_abs / r_follower * w)
        hz = "/".join(f"{n * rev:.1f}" for n, _, _ in tones)
        out.append(f"  {rev:6.2f} {hz:>18} {acc:13.1f} mg {acc * r_contact / r_follower:8.1f} mg "
                   f"{rate:9.1f} deg/s")
    return out, ok


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
    w = ap.add_argument_group("wave cam (--tones / --lobes): no steps, shakes the arm for the AHRS")
    w.add_argument("--tones", default=None,
                   help='lobes:pp_mm[:phase_deg], comma-separated, e.g. "3:1.0,11:0.2:90"; "none" = a circle')
    w.add_argument("--lobes", type=int, default=0, help="one tone, shorthand for --tones N:pp")
    w.add_argument("--pp-mm", type=float, default=0.8, help="with --lobes: peak to peak at the follower")
    w.add_argument("--lift-min-mm", type=float, default=0.7,
                   help="lowest nose position over its resting height (wheel clear by this x lever)")
    w.add_argument("--nose-r-mm", type=float, default=0.45, help="the follower tip's nose radius")
    w.add_argument("--nose-below-mm", type=float, default=2.3,
                   help="the nose's lowest point below the old pad's face (the tip: 0.8 plate + 1.5)")
    w.add_argument("--fall-ms2", type=float, default=5.0,
                   help="the follower's free fall: the contact's 8.30 m/s^2 (drop_rig_bench.yaml "
                        "m_eff fit, 10-03) x 124/205")
    w.add_argument("--servo-dps", type=float, default=618.0, help="XL330-M288 no-load, deg/s")
    w.add_argument("--r-follower-mm", type=float, default=124.0, help="pivot to follower")
    w.add_argument("--r-contact-mm", type=float, default=205.0, help="pivot to wheel contact")
    args = ap.parse_args()
    tones = (parse_tones(args.tones) if args.tones is not None
             else [(args.lobes, args.pp_mm, 0.0)] if args.lobes else None)
    if tones is not None:
        rest = args.r_dwell + args.gap - args.nose_below_mm
        lines, ok = wave_report(rest, args.nose_r_mm, args.lift_min_mm, tones, args.fall_ms2,
                                args.servo_dps, args.r_follower_mm, args.r_contact_mm)
        print("wave cam: " + (", ".join(f"{n} lobes {pp:g} mm pp" + (f" @{ph:g} deg" if ph else "")
                                        for n, pp, ph in tones) or "plain circle"))
        print("\n".join(lines))
        if not ok:
            raise SystemExit("not written")
        outlines = [wave_profile(rest, args.nose_r_mm, args.lift_min_mm, tones)]
        if args.bore:
            outlines.append(d_bore(args.bore, args.flat))
        r_max = max(math.hypot(*q) for q in outlines[0])
        out = Path(args.out)
        write_svg(out.with_suffix(".svg"), outlines, 2 * r_max)
        write_dxf(out.with_suffix(".dxf"), outlines)
        print(f"wrote {out}.svg / .dxf")
        return
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
