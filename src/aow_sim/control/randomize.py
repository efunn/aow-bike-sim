"""Domain randomization shared by every RL env.

One implementation instead of four byte-identical copies of
`_apply_randomization` (general/flick/pivot/ball). The envs differ in reward,
episode structure, and command sampling — never in how the model is perturbed
— so this is the single place that has to change when a new randomization axis
is added.

Axes:
  mass_frac      — per-body mass scale.
  friction_frac  — global sliding-friction scale (tangential coefficient only;
                   torsional is left alone, it is already a GUESS).
  actuator_frac  — ACTUATOR STRENGTH, i.e. supply voltage. Added for the
                   untethered bike: the sim's servo numbers are the 12 V
                   datasheet column, but a 3S pack runs 12.6 V down to 9.9 V,
                   so the real bike is up to ~18% weaker than the model at any
                   given moment. See docs/plans/untethered-setup.md.
  solref_frac    — CONTACT STIFFNESS, scaling `sim.contact_solref[0]`.
  dampratio_range— CONTACT RESTITUTION, drawn absolutely (not as a fraction)
                   from [lo, hi], because the plausible band is wide and not
                   centred on the nominal.

Why contact was worth adding (2026-08): `contact_solref` is the least known
parameter in the model -- the bench measurements that would pin it are only
now specified, in docs/measurements/contact-protocol.md -- and it is the one
the policies are most brittle to. Holding timeconst at 0.005 and sweeping
dampratio over the eval grid, survival is flat at 1.00 from 1.0 down to 0.5
and then falls off a cliff: 0.9 at 0.3, and 0.7 for two of three policies at
0.2. A fixed dampratio trains on one point of that curve; the physical drop
test puts the true value right at the edge of it. Randomizing costs nothing
and covers what the measurement cannot yet resolve.

Why actuator_frac scales ctrlrange AND forcerange from ONE draw: for a DC
servo both no-load speed and stall torque are proportional to supply voltage,
so they are not independent — a pack at 10 V is simultaneously slower and
weaker. Drawing them separately would train against motors that cannot exist.
The internal PI gains (drive_kv, steer_kp) are NOT scaled: those are firmware
constants and do not move with voltage.

MuJoCo enforces both clamps itself (ctrl is clipped to ctrlrange when
ctrllimited, actuator force to forcerange), so perturbing the model arrays is
enough — no env-side action rescaling.
"""

from __future__ import annotations

import numpy as np


class DomainRandomizer:
    """Holds the nominal model arrays and re-draws them per episode.

    Baselines are captured at construction, so build the model fully (hockey
    extras, payload, variant) before handing it over.
    """

    def __init__(self, model, cfg: dict):
        self.model = model
        self.cfg = cfg
        self._mass0 = model.body_mass.copy()
        self._friction0 = model.geom_friction.copy()
        self._forcerange0 = model.actuator_forcerange.copy()
        self._ctrlrange0 = model.actuator_ctrlrange.copy()
        self._solref0 = model.geom_solref.copy()

    def reset_nominal(self) -> None:
        """Restore the unperturbed model (also the `enabled: false` path)."""
        self.model.body_mass[:] = self._mass0
        self.model.geom_friction[:] = self._friction0
        self.model.actuator_forcerange[:] = self._forcerange0
        self.model.actuator_ctrlrange[:] = self._ctrlrange0
        self.model.geom_solref[:] = self._solref0

    def apply(self, rng) -> None:
        """Draw one perturbed model. Call once per episode, before reset."""
        r = self.cfg
        self.reset_nominal()
        if not r["enabled"]:
            return
        self.model.body_mass[:] = self._mass0 * (
            1 + rng.uniform(-r["mass_frac"], r["mass_frac"], self._mass0.shape))
        self.model.geom_friction[:, 0] *= (
            1 + rng.uniform(-r["friction_frac"], r["friction_frac"]))
        # Absent from the older move configs -> nominal strength, so existing
        # runs reproduce exactly.
        af = float(r.get("actuator_frac", 0.0))
        if af > 0.0:
            v = 1 + rng.uniform(-af, af)      # one draw = one supply voltage
            self.model.actuator_forcerange[:] = self._forcerange0 * v
            self.model.actuator_ctrlrange[:] = self._ctrlrange0 * v
        # Contact stiffness and restitution. Also absent from older configs ->
        # nominal contact, so existing runs reproduce exactly.
        sf = float(r.get("solref_frac", 0.0))
        dr = r.get("dampratio_range")
        if sf > 0.0:
            self.model.geom_solref[:, 0] = (self._solref0[:, 0]
                                            * (1 + rng.uniform(-sf, sf)))
        if dr:
            self.model.geom_solref[:, 1] = rng.uniform(float(dr[0]),
                                                       float(dr[1]))
