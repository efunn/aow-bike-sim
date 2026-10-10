FeatureScript 3044;
import(path : "onshape/std/geometry.fs", version : "3044.0");

/* GENERATED, do not hand-edit: the next push overwrites the whole studio.
 *   python -m aow_sim.cad_wing_linkage_metal --push wing_metal_features
 *
 * Every primitive of every part is computed in aow_sim.cad_wing_linkage_metal
 * and carried here as WLM_PARTS; this file only builds them. Millimetres.
 * MODULE FRAME: origin on the wing rod at the mid-plane between the couplers,
 * +Y forward, +X right, +Z up. Poses: WLM_POSES, a rotation about +Y through
 * `p` by `a` degrees, then a shift `d`, per group.
 */

export const WLM_PARTS = [{ "slug" : "crankR", "name" : "half crank R", "group" : "crank", "color" : [0.72, 0.74, 0.78], "note" : "Turned journal + milled web, pressed on the crankpin. Oldham slot across X in the journal end (the front twin's is unused).", "add" : [{ "k" : "slot", "pts" : [[-7.9284179177, 26.1591861528], [-5.9463134225, 40.8763659665], [5.9463134558, 40.8763659539], [7.92841792, 26.159186136]], "mids" : [[0.0000000222, 46.0755186068], [-0.0000000085, 17.0913896732]], "a" : [0, 25.0913896732], "b" : [0.0000000158, 40.0755186068], "ra" : 8, "rb" : 6, "y0" : -10.75, "y1" : -5.75 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -18.75, "y1" : -10.75, "r" : 6 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -27.75, "y1" : -18.75, "r" : 4 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -11.75, "y1" : -4.75, "r" : 2.5 }, { "k" : "box", "lo" : [-5, -28.75, 23.5913896732], "hi" : [5, -25.75, 26.5913896732] }] }, { "slug" : "couplerR", "name" : "coupler R", "group" : "cR", "color" : [0.72, 0.74, 0.78], "note" : "38.11 mm centres. Delrin bushings both eyes.", "add" : [{ "k" : "slot", "pts" : [[5.2961149143, 44.6527630816], [30.2053187649, 15.8314916049], [21.3383590423, 7.7520200329], [-5.0486714287, 35.2267129142]], "mids" : [[29.706919179, 7.4731320084], [-4.7146486991, 45.2496936613]], "a" : [0.0000000158, 40.0755186068], "b" : [25.665791709, 11.908139198], "ra" : 7, "rb" : 6, "y0" : -5.25, "y1" : -0.25 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -6.25, "y1" : 0.75, "r" : 4.5 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -6.25, "y1" : 0.75, "r" : 3.5 }] }, { "slug" : "bigR", "name" : "big end bushing R", "group" : "cR", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -5.25, "y1" : -0.25, "r" : 4.5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -6.25, "y1" : 0.75, "r" : 2.5 }] }, { "slug" : "smallR", "name" : "small end bushing R", "group" : "cR", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -5.25, "y1" : -0.25, "r" : 3.5 }], "cut" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -6.25, "y1" : 0.75, "r" : 2 }] }, { "slug" : "washJR", "name" : "washer rocker R", "group" : "cR", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -5.75, "y1" : -5.25, "r" : 5 }], "cut" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -6.75, "y1" : -4.25, "r" : 2 }] }, { "slug" : "rockerR", "name" : "rocker R", "group" : "wR", "color" : [0.72, 0.74, 0.78], "note" : "Hub on the rod (Delrin bushed), arm to the coupler at 28.29 mm, ear to the panel pad. The pad runs back past the bulkhead.", "add" : [{ "k" : "slot", "pts" : [[-2.9775374827, 7.4252454868], [23.2465425044, 17.941151156], [28.7102252105, 6.1651935552], [3.7469950787, -7.068240791]], "mids" : [[31.5620624122, 14.6438277393], [-7.2569485577, -3.3670012816]], "a" : [0, 0], "b" : [25.665791709, 11.908139198], "ra" : 8, "rb" : 6.5, "y0" : -10.75, "y1" : -5.75 }, { "k" : "slot", "pts" : [[-4.4903545599, 6.6209301406], [32.4201846862, 31.653879393], [39.5866724236, 22.0438613256], [5.0649624233, -6.1924272826]], "mids" : [[40.5977887074, 30.2750268413], [-6.4131174684, -4.7824600717]], "a" : [0, 0], "b" : [35.7879506061, 26.6881817876], "ra" : 8, "rb" : 6, "y0" : -18.75, "y1" : -10.75 }, { "k" : "prism", "pts" : [[39.0269997488, 13.4307853369], [45.1226115245, 36.6437885265], [40.2865691933, 37.9137076464], [34.1909574177, 14.7007044568]], "y0" : -39.75, "y1" : -10.75 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -19.75, "y1" : -4.75, "r" : 5 }, { "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -11.75, "y1" : -4.75, "r" : 2 }] }, { "slug" : "rodbushR", "name" : "rocker bushing R", "group" : "wR", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -18.75, "y1" : -5.75, "r" : 5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -19.25, "y1" : -18.75, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -20.25, "y1" : -4.75, "r" : 3 }] }, { "slug" : "rpinR", "name" : "rocker pin R", "group" : "wR", "color" : [0.3, 0.3, 0.32], "note" : "Pressed in the rocker arm, runs in the coupler's small end.", "add" : [{ "k" : "cyl", "x" : 25.665791709, "z" : 11.908139198, "y0" : -10.75, "y1" : -0.25, "r" : 2 }], "cut" : [] }, { "slug" : "knuckleR", "name" : "knuckle R", "group" : "wR", "color" : [0.72, 0.74, 0.78], "note" : "The wing's second support, ahead of the far bulkhead: carries panel loads to the rod, drives nothing.", "add" : [{ "k" : "slot", "pts" : [[-4.4903545599, 6.6209301406], [32.4201846862, 31.653879393], [39.5866724236, 22.0438613256], [5.0649624233, -6.1924272826]], "mids" : [[40.5977887074, 30.2750268413], [-6.4131174684, -4.7824600717]], "a" : [0, 0], "b" : [35.7879506061, 26.6881817876], "ra" : 8, "rb" : 6, "y0" : 27.75, "y1" : 39.75 }, { "k" : "prism", "pts" : [[39.0269997488, 13.4307853369], [45.1226115245, 36.6437885265], [40.2865691933, 37.9137076464], [34.1909574177, 14.7007044568]], "y0" : 27.75, "y1" : 39.75 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 26.75, "y1" : 40.75, "r" : 5 }] }, { "slug" : "knbushR", "name" : "knuckle bushing R", "group" : "wR", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 27.75, "y1" : 39.75, "r" : 5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : 27.25, "y1" : 27.75, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 26.25, "y1" : 40.75, "r" : 3 }] }, { "slug" : "panelR", "name" : "wing panel R (mock)", "group" : "wR", "color" : [0.45, 0.7, 0.5], "note" : "Placeholder panel on the linkage config's line; bolts to the two pads.", "add" : [{ "k" : "prism", "pts" : [[38.0110644529, 9.561951472], [63.4094468514, 106.2827980953], [69.2126976488, 104.7588951514], [43.8143152503, 8.038048528]], "y0" : -39.75, "y1" : 39.75 }], "cut" : [] }, { "slug" : "bulkheadR", "name" : "bulkhead R", "group" : "fixed", "color" : [0.72, 0.74, 0.78], "note" : "Rod clamped, crank bushing pressed. Top face screws to the bridge.", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -27.25, "y1" : -19.25, "r" : 13 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -27.25, "y1" : -19.25, "r" : 12 }, { "k" : "box", "lo" : [-10, -27.25, 0], "hi" : [10, -19.25, 53] }, { "k" : "box", "lo" : [-18, -27.25, 35], "hi" : [18, -19.25, 53] }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -28.25, "y1" : -18.25, "r" : 3 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -28.25, "y1" : -18.25, "r" : 5.5 }] }, { "slug" : "jbushR", "name" : "journal bushing R", "group" : "fixed", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -27.25, "y1" : -19.25, "r" : 5.5 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -19.25, "y1" : -18.75, "r" : 7.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -28.25, "y1" : -17.75, "r" : 4 }] }, { "slug" : "crankL", "name" : "half crank L", "group" : "crank", "color" : [0.72, 0.74, 0.78], "note" : "Turned journal + milled web, pressed on the crankpin. Oldham slot across X in the journal end (the front twin's is unused).", "add" : [{ "k" : "slot", "pts" : [[7.9284179177, 26.1591861528], [5.9463134225, 40.8763659665], [-5.9463134558, 40.8763659539], [-7.92841792, 26.159186136]], "mids" : [[-0.0000000222, 46.0755186068], [0.0000000085, 17.0913896732]], "a" : [0, 25.0913896732], "b" : [-0.0000000158, 40.0755186068], "ra" : 8, "rb" : 6, "y0" : 5.75, "y1" : 10.75 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 10.75, "y1" : 18.75, "r" : 6 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 18.75, "y1" : 27.75, "r" : 4 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 4.75, "y1" : 11.75, "r" : 2.5 }, { "k" : "box", "lo" : [-5, 25.75, 23.5913896732], "hi" : [5, 28.75, 26.5913896732] }] }, { "slug" : "couplerL", "name" : "coupler L", "group" : "cL", "color" : [0.72, 0.74, 0.78], "note" : "38.11 mm centres. Delrin bushings both eyes.", "add" : [{ "k" : "slot", "pts" : [[-5.2961149143, 44.6527630816], [-30.2053187649, 15.8314916049], [-21.3383590423, 7.7520200329], [5.0486714287, 35.2267129142]], "mids" : [[-29.706919179, 7.4731320084], [4.7146486991, 45.2496936613]], "a" : [-0.0000000158, 40.0755186068], "b" : [-25.665791709, 11.908139198], "ra" : 7, "rb" : 6, "y0" : 0.25, "y1" : 5.25 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : -0.75, "y1" : 6.25, "r" : 4.5 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.75, "y1" : 6.25, "r" : 3.5 }] }, { "slug" : "bigL", "name" : "big end bushing L", "group" : "cL", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : 0.25, "y1" : 5.25, "r" : 4.5 }], "cut" : [{ "k" : "cyl", "x" : -0.0000000158, "z" : 40.0755186068, "y0" : -0.75, "y1" : 6.25, "r" : 2.5 }] }, { "slug" : "smallL", "name" : "small end bushing L", "group" : "cL", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 0.25, "y1" : 5.25, "r" : 3.5 }], "cut" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : -0.75, "y1" : 6.25, "r" : 2 }] }, { "slug" : "washJL", "name" : "washer rocker L", "group" : "cL", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 5.25, "y1" : 5.75, "r" : 5 }], "cut" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 4.25, "y1" : 6.75, "r" : 2 }] }, { "slug" : "rockerL", "name" : "rocker L", "group" : "wL", "color" : [0.72, 0.74, 0.78], "note" : "Hub on the rod (Delrin bushed), arm to the coupler at 28.29 mm, ear to the panel pad. The pad runs back past the bulkhead.", "add" : [{ "k" : "slot", "pts" : [[2.9775374827, 7.4252454868], [-23.2465425044, 17.941151156], [-28.7102252105, 6.1651935552], [-3.7469950787, -7.068240791]], "mids" : [[-31.5620624122, 14.6438277393], [7.2569485577, -3.3670012816]], "a" : [0, 0], "b" : [-25.665791709, 11.908139198], "ra" : 8, "rb" : 6.5, "y0" : 5.75, "y1" : 10.75 }, { "k" : "slot", "pts" : [[4.4903545599, 6.6209301406], [-32.4201846862, 31.653879393], [-39.5866724236, 22.0438613256], [-5.0649624233, -6.1924272826]], "mids" : [[-40.5977887074, 30.2750268413], [6.4131174684, -4.7824600717]], "a" : [0, 0], "b" : [-35.7879506061, 26.6881817876], "ra" : 8, "rb" : 6, "y0" : 10.75, "y1" : 18.75 }, { "k" : "prism", "pts" : [[-34.1909574177, 14.7007044568], [-40.2865691933, 37.9137076464], [-45.1226115245, 36.6437885265], [-39.0269997488, 13.4307853369]], "y0" : 10.75, "y1" : 39.75 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 4.75, "y1" : 19.75, "r" : 5 }, { "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 4.75, "y1" : 11.75, "r" : 2 }] }, { "slug" : "rodbushL", "name" : "rocker bushing L", "group" : "wL", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 5.75, "y1" : 18.75, "r" : 5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : 18.75, "y1" : 19.25, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 4.75, "y1" : 20.25, "r" : 3 }] }, { "slug" : "rpinL", "name" : "rocker pin L", "group" : "wL", "color" : [0.3, 0.3, 0.32], "note" : "Pressed in the rocker arm, runs in the coupler's small end.", "add" : [{ "k" : "cyl", "x" : -25.665791709, "z" : 11.908139198, "y0" : 0.25, "y1" : 10.75, "r" : 2 }], "cut" : [] }, { "slug" : "knuckleL", "name" : "knuckle L", "group" : "wL", "color" : [0.72, 0.74, 0.78], "note" : "The wing's second support, ahead of the far bulkhead: carries panel loads to the rod, drives nothing.", "add" : [{ "k" : "slot", "pts" : [[4.4903545599, 6.6209301406], [-32.4201846862, 31.653879393], [-39.5866724236, 22.0438613256], [-5.0649624233, -6.1924272826]], "mids" : [[-40.5977887074, 30.2750268413], [6.4131174684, -4.7824600717]], "a" : [0, 0], "b" : [-35.7879506061, 26.6881817876], "ra" : 8, "rb" : 6, "y0" : -39.75, "y1" : -27.75 }, { "k" : "prism", "pts" : [[-34.1909574177, 14.7007044568], [-40.2865691933, 37.9137076464], [-45.1226115245, 36.6437885265], [-39.0269997488, 13.4307853369]], "y0" : -39.75, "y1" : -27.75 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -40.75, "y1" : -26.75, "r" : 5 }] }, { "slug" : "knbushL", "name" : "knuckle bushing L", "group" : "wL", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -39.75, "y1" : -27.75, "r" : 5 }, { "k" : "cyl", "x" : 0, "z" : 0, "y0" : -27.75, "y1" : -27.25, "r" : 7 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -40.75, "y1" : -26.25, "r" : 3 }] }, { "slug" : "panelL", "name" : "wing panel L (mock)", "group" : "wL", "color" : [0.45, 0.7, 0.5], "note" : "Placeholder panel on the linkage config's line; bolts to the two pads.", "add" : [{ "k" : "prism", "pts" : [[-43.8143152503, 8.038048528], [-69.2126976488, 104.7588951514], [-63.4094468514, 106.2827980953], [-38.0110644529, 9.561951472]], "y0" : -39.75, "y1" : 39.75 }], "cut" : [] }, { "slug" : "bulkheadL", "name" : "bulkhead L", "group" : "fixed", "color" : [0.72, 0.74, 0.78], "note" : "Rod clamped, crank bushing pressed. Top face screws to the bridge.", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 19.25, "y1" : 27.25, "r" : 13 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 19.25, "y1" : 27.25, "r" : 12 }, { "k" : "box", "lo" : [-10, 19.25, 0], "hi" : [10, 27.25, 53] }, { "k" : "box", "lo" : [-18, 19.25, 35], "hi" : [18, 27.25, 53] }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : 18.25, "y1" : 28.25, "r" : 3 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 18.25, "y1" : 28.25, "r" : 5.5 }] }, { "slug" : "jbushL", "name" : "journal bushing L", "group" : "fixed", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 19.25, "y1" : 27.25, "r" : 5.5 }, { "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 18.75, "y1" : 19.25, "r" : 7.5 }], "cut" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : 17.75, "y1" : 28.25, "r" : 4 }] }, { "slug" : "washP0", "name" : "washer crankpin 1", "group" : "crank", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -5.75, "y1" : -5.25, "r" : 5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -6.75, "y1" : -4.25, "r" : 2.5 }] }, { "slug" : "washP1", "name" : "washer crankpin 2", "group" : "crank", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -0.25, "y1" : 0.25, "r" : 5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -1.25, "y1" : 1.25, "r" : 2.5 }] }, { "slug" : "washP2", "name" : "washer crankpin 3", "group" : "crank", "color" : [0.95, 0.94, 0.88], "note" : "delrin", "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : 5.25, "y1" : 5.75, "r" : 5 }], "cut" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : 4.25, "y1" : 6.75, "r" : 2.5 }] }, { "slug" : "crankpin", "name" : "crankpin", "group" : "crank", "color" : [0.3, 0.3, 0.32], "note" : "Steel dowel, pressed in both webs.", "add" : [{ "k" : "cyl", "x" : 0.0000000158, "z" : 40.0755186068, "y0" : -10.75, "y1" : 10.75, "r" : 2.5 }], "cut" : [] }, { "slug" : "rod", "name" : "wing rod", "group" : "fixed", "color" : [0.3, 0.3, 0.32], "note" : "Steel, clamped in both bulkheads; clips at the ends.", "add" : [{ "k" : "cyl", "x" : 0, "z" : 0, "y0" : -41.75, "y1" : 41.75, "r" : 3 }], "cut" : [] }, { "slug" : "oldham", "name" : "oldham disc", "group" : "crank", "color" : [0.95, 0.94, 0.88], "note" : "Tongue across X forward (journal), across Z back (horn hub): torque only.", "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -33.25, "y1" : -28.25, "r" : 7 }, { "k" : "box", "lo" : [-3.5, -28.25, 23.5913896732], "hi" : [3.5, -26.75, 26.5913896732] }, { "k" : "box", "lo" : [-1.5, -34.75, 21.5913896732], "hi" : [1.5, -33.25, 28.5913896732] }], "cut" : [] }, { "slug" : "hornhub", "name" : "horn hub", "group" : "crank", "color" : [0.72, 0.74, 0.78], "note" : "4 x M2 into the horn (PCD 12), slot across Z for the Oldham disc.", "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -37.25, "y1" : -33.75, "r" : 8 }], "cut" : [{ "k" : "box", "lo" : [-1.5, -35.75, 16.0913896732], "hi" : [1.5, -32.75, 34.0913896732] }, { "k" : "cyl", "x" : 4.2426406871, "z" : 29.3340303603, "y0" : -38.25, "y1" : -32.75, "r" : 1.1 }, { "k" : "cyl", "x" : -4.2426406871, "z" : 29.3340303603, "y0" : -38.25, "y1" : -32.75, "r" : 1.1 }, { "k" : "cyl", "x" : -4.2426406871, "z" : 20.848748986, "y0" : -38.25, "y1" : -32.75, "r" : 1.1 }, { "k" : "cyl", "x" : 4.2426406871, "z" : 20.848748986, "y0" : -38.25, "y1" : -32.75, "r" : 1.1 }] }, { "slug" : "horn", "name" : "XC330 horn (mock)", "group" : "crank", "color" : [0.16, 0.16, 0.18], "note" : "mock", "add" : [{ "k" : "cyl", "x" : 0, "z" : 25.0913896732, "y0" : -40.25, "y1" : -37.25, "r" : 8 }], "cut" : [] }, { "slug" : "servo", "name" : "XC330 (mock)", "group" : "fixed", "color" : [0.16, 0.16, 0.18], "note" : "mock", "add" : [{ "k" : "box", "lo" : [-10, -63.25, 15.5913896732], "hi" : [10, -40.25, 49.5913896732] }], "cut" : [] }, { "slug" : "cradle", "name" : "servo cradle", "group" : "fixed", "color" : [0.93, 0.56, 0.2], "note" : "Holds the XC330 by its back face; the horn face is free. Hangs from the bridge.", "add" : [{ "k" : "box", "lo" : [10, -63.25, 29], "hi" : [13, -27.25, 53] }, { "k" : "box", "lo" : [-13, -63.25, 29], "hi" : [-10, -27.25, 53] }, { "k" : "box", "lo" : [-13, -66.25, 12.6], "hi" : [13, -63.25, 53] }], "cut" : [{ "k" : "cyl", "x" : -8, "z" : 17.5913896732, "y0" : -67.25, "y1" : -62.25, "r" : 1.1 }, { "k" : "cyl", "x" : -8, "z" : 47.5913896732, "y0" : -67.25, "y1" : -62.25, "r" : 1.1 }, { "k" : "cyl", "x" : 8, "z" : 17.5913896732, "y0" : -67.25, "y1" : -62.25, "r" : 1.1 }, { "k" : "cyl", "x" : 8, "z" : 47.5913896732, "y0" : -67.25, "y1" : -62.25, "r" : 1.1 }] }, { "slug" : "bridge", "name" : "bridge", "group" : "fixed", "color" : [0.72, 0.74, 0.78], "note" : "Ties the bulkheads and the cradle; its top face is the mount to the bike.", "add" : [{ "k" : "box", "lo" : [-18, -66.25, 53], "hi" : [18, 27.25, 58] }], "cut" : [] }, { "slug" : "floor", "name" : "floor (mock)", "group" : "fixed", "color" : [0.85, 0.85, 0.85], "note" : "floor", "add" : [{ "k" : "box", "lo" : [-130, -66.25, -43.2], "hi" : [130, 41.75, -41.2] }], "cut" : [] }];

export const WLM_POSES = { "-1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 230.7, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 2.6798048387, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 284.0766645315, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -40.2642141254, "d" : [-11.5953215024, -24.4747895809] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -32.9793165092, "d" : [-11.5953215024, -24.4747895809] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "-0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : 263.025, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -11.7730025422, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 306.6068553466, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -38.0264383263, "d" : [-14.8732348998, -16.803745373] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -27.1397088644, "d" : [-14.8732348998, -16.803745373] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "-0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : -72.408, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -14.207694032, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 322.7192991998, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -29.6586423238, "d" : [-14.2833643483, -10.4553737796] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -21.8297524981, "d" : [-14.2833643483, -10.4553737796] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "-0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : -64.65, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -14.0469614626, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 327.5318901749, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -26.5495191007, "d" : [-13.5412961599, -8.5687241421] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -19.9655521756, "d" : [-13.5412961599, -8.5687241421] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "-0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : -32.325, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : -9.6533976711, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : -14.3105771293, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : -12.9302418337, "d" : [-8.0123301116, -2.3221114666] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : -11.0337329566, "d" : [-8.0123301116, -2.3221114666] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+0.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 0, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 0, "d" : [0, 0] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+0.25" : { "crank" : { "p" : [0, 25.0913896732], "a" : 32.325, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 14.3105771293, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 9.6533976711, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 11.0337329426, "d" : [8.0123301067, -2.3221114835] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 12.9302418423, "d" : [8.0123301067, -2.3221114835] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+0.50" : { "crank" : { "p" : [0, 25.0913896732], "a" : 64.65, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 32.4681098251, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 14.0469614626, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 19.9655521429, "d" : [13.5412961418, -8.5687241707] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 26.549519113, "d" : [13.5412961418, -8.5687241707] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+0.56" : { "crank" : { "p" : [0, 25.0913896732], "a" : 72.408, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 37.2807008002, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 14.207694032, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 21.8297524605, "d" : [14.2833643262, -10.4553738098] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 29.6586423362, "d" : [14.2833643262, -10.4553738098] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+0.75" : { "crank" : { "p" : [0, 25.0913896732], "a" : 96.975, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 53.3931446534, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : 11.7730025422, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 27.1397088113, "d" : [14.8732348643, -16.8037454044] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 38.0264383368, "d" : [14.8732348643, -16.8037454044] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } }, "+1.00" : { "crank" : { "p" : [0, 25.0913896732], "a" : 129.3, "d" : [0, 0] }, "wR" : { "p" : [0, 0], "a" : 75.9233354685, "d" : [0, 0] }, "wL" : { "p" : [0, 0], "a" : -2.6798048387, "d" : [0, 0] }, "cR" : { "p" : [0.0000000158, 40.0755186068], "a" : 32.9793164383, "d" : [11.5953214507, -24.4747896054] }, "cL" : { "p" : [0.0000000158, 40.0755186068], "a" : 40.2642141229, "d" : [11.5953214507, -24.4747896054] }, "fixed" : { "p" : [0, 0], "a" : 0, "d" : [0, 0] } } };


/** One primitive: a cylinder along Y, a prism from an (X, Z) outline, or a box. */
export function wlmPrim(context is Context, id is Id, q is map) returns Query
{
    const mm = millimeter;
    if (q.k == "cyl")
    {
        fCylinder(context, id, { "bottomCenter" : vector(q.x, q.y0, q.z) * mm,
                                 "topCenter"    : vector(q.x, q.y1, q.z) * mm,
                                 "radius"       : q.r * mm });
    }
    else if (q.k == "box")
    {
        fCuboid(context, id, { "corner1" : vector(q.lo[0], q.lo[1], q.lo[2]) * mm,
                               "corner2" : vector(q.hi[0], q.hi[1], q.hi[2]) * mm });
    }
    else if (q.k == "slot")
    {
        var sk = newSketchOnPlane(context, id + "sk", {
                "sketchPlane" : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0)) });
        const P = function(v) { return vector(v[0], -v[1]) * mm; };
        skLineSegment(sk, "l1", { "start" : P(q.pts[0]), "end" : P(q.pts[1]) });
        skArc(sk, "ab", { "start" : P(q.pts[1]), "mid" : P(q.mids[0]), "end" : P(q.pts[2]) });
        skLineSegment(sk, "l2", { "start" : P(q.pts[2]), "end" : P(q.pts[3]) });
        skArc(sk, "aa", { "start" : P(q.pts[3]), "mid" : P(q.mids[1]), "end" : P(q.pts[0]) });
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    else
    {
        // sketch plane normal +Y, x axis +X, so the sketch's second axis is -Z
        var sk = newSketchOnPlane(context, id + "sk", {
                "sketchPlane" : plane(vector(0, q.y0, 0) * mm, vector(0, 1, 0), vector(1, 0, 0)) });
        var pts = [];
        for (var p in q.pts)
            pts = append(pts, vector(p[0], -p[1]) * mm);
        // std has no polygon: one segment per edge, closed
        for (var i = 0; i < size(pts); i += 1)
            skLineSegment(sk, "s" ~ i, { "start" : pts[i], "end" : pts[(i + 1) % size(pts)] });
        skSolve(sk);
        opExtrude(context, id + "ext", {
                "entities"  : qSketchRegion(id + "sk"),
                "direction" : vector(0, 1, 0),
                "endBound"  : BoundingType.BLIND,
                "endDepth"  : (q.y1 - q.y0) * mm });
        opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(id + "sk", EntityType.BODY) });
    }
    return qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
}

/** One part: its primitives united, its cuts subtracted, named and coloured. */
export function wlmPart(context is Context, id is Id, p is map) returns Query
{
    var adds = [];
    for (var i = 0; i < size(p.add); i += 1)
        adds = append(adds, wlmPrim(context, id + ("a" ~ i), p.add[i]));
    if (size(adds) > 1)
        opBoolean(context, id + "uni", { "tools" : qUnion(adds),
                                         "operationType" : BooleanOperationType.UNION });
    // EVALUATED before the cutters exist: they are created under this same id,
    // so a lazy query would make every cutter its own target too.
    const body = qUnion(evaluateQuery(context, qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID)));
    if (size(p.cut) > 0)
    {
        var cuts = [];
        for (var i = 0; i < size(p.cut); i += 1)
            cuts = append(cuts, wlmPrim(context, id + ("c" ~ i), p.cut[i]));
        opBoolean(context, id + "sub", { "tools" : qUnion(cuts), "targets" : body,
                                         "operationType" : BooleanOperationType.SUBTRACTION });
    }
    const q = qBodyType(qCreatedBy(id, EntityType.BODY), BodyType.SOLID);
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : p.name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE,
                           "value" : color(p.color[0], p.color[1], p.color[2]) });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.DESCRIPTION, "value" : p.note });
    return q;
}

/** The transform that puts one group at one pose. */
export function wlmXf(g is map) returns Transform
{
    const mm = millimeter;
    return transform(vector(g.d[0], 0, g.d[1]) * mm)
         * rotationAround(line(vector(g.p[0], 0, g.p[1]) * mm, vector(0, 1, 0)), g.a * degree);
}

/** Build every part at rest; returns the bodies by group. */
export function wlmBuild(context is Context, id is Id, mocks is boolean, trace is boolean) returns map
{
    var groups = {};
    for (var p in WLM_PARTS)
    {
        if (!mocks && isIn(p.slug, ["servo", "horn", "floor", "panelR", "panelL"]))
            continue;
        if (trace)
            println("BUILD|" ~ p.slug);
        const q = wlmPart(context, id + p.slug, p);
        groups[p.group] = append(groups[p.group] == undefined ? [] : groups[p.group], q);
    }
    return groups;
}

/** Move every group to pose `key` (from rest), or back with `back`. */
export function wlmPose(context is Context, id is Id, groups is map, key is string, back is boolean)
{
    const ps = WLM_POSES[key];
    for (var g in ["crank", "cR", "cL", "wR", "wL"])
    {
        if (groups[g] == undefined)
            continue;
        const xf = wlmXf(ps[g]);
        opTransform(context, id + g, { "bodies" : qUnion(groups[g]),
                                       "transform" : back ? inverse(xf) : xf });
    }
}

// ==== UI LAYER BELOW -- dropped by --check ====

export enum WingPose
{
    annotation { "Name" : "Rest" } REST,
    annotation { "Name" : "Right wing 25%" } R25,
    annotation { "Name" : "Right wing 50%" } R50,
    annotation { "Name" : "Right wing 75%" } R75,
    annotation { "Name" : "Right wing down (100%)" } R100,
    annotation { "Name" : "Left wing 25%" } L25,
    annotation { "Name" : "Left wing 50%" } L50,
    annotation { "Name" : "Left wing 75%" } L75,
    annotation { "Name" : "Left wing down (100%)" } L100
}

annotation { "Feature Type Name" : "AOW wing linkage metal",
             "Feature Type Description" : "Diamond swing linkage as a supported crankshaft, every pivot bushed, Oldham to the XC330; Y forward, origin on the wing rod" }
export const aowWingLinkageMetal = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Pose" }
        definition.pose is WingPose;
        annotation { "Name" : "Show mocks (servo, panels, floor)", "Default" : true }
        definition.mocks is boolean;
    }
    {
        const groups = wlmBuild(context, id + "build", definition.mocks, false);
        const keys = { WingPose.REST : "+0.00", WingPose.R25 : "+0.25", WingPose.R50 : "+0.50", WingPose.R75 : "+0.75", WingPose.R100 : "+1.00", WingPose.L25 : "-0.25", WingPose.L50 : "-0.50", WingPose.L75 : "-0.75", WingPose.L100 : "-1.00" };
        const key = keys[definition.pose];
        if (key != "+0.00")
            wlmPose(context, id + "pose", groups, key, false);
        reportFeatureInfo(context, id, "Diamond x1.25: crank 14.98, coupler 38.11, rocker 28.29; crank axis 25.09 above the rod; stroke +-129.3 deg");
    });
