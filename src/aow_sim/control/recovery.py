"""When is the bike ready to be handed back to the policy?

ONE definition, because there are two callers and they must not drift:

  * `control/righting.py` -- the sim-side fall-to-balancing sequencer. Its
    `lift` phase ends when this says yes, and the general policy takes over.
  * `hw/run_bike.py` -- the onboard fall guard. Its `cut` state ends when this
    says yes (plus a dwell and the operator's go-ahead; see there).

They are the SAME EVENT seen from two places, so the predicate lives here
rather than in either of them. This module deliberately imports nothing but
numpy: `righting.py` pulls in mujoco and `build_model`, which nothing under
`hw/` may touch, and that is the only reason this is a third file instead of a
function in the one that came first.

THE NUMBERS ARE MEASURED, NOT CHOSEN. `analysis/no_return.py` bisects the
largest lean `general_rl` still walks away from, from the settled state, in the
two signals the AHRS actually reports. At standstill:

    right   16.3 deg at 0 rad/s, 12.6 at 1, 8.8 at 2, 3.0 at 3
    left    11.8 deg at 0 rad/s,  4.7 at 1, 2.6 at 2, 0.4 at 3

so the WEAKER (left) side sets the angle, and rate matters more than angle --
which is why this is a pair and not a threshold. The policy's left/right
asymmetry is therefore a constraint on the mechanism; see
`docs/plans/self-righting.md` section 1 and the `turn_asym` risk in
`docs/status.md`.

RAISING EITHER CONSTANT IS A QUESTION, NOT A FIX. They describe what the policy
can do, so a hand-off that keeps failing means the mechanism is not finishing
its job, or the policy has changed and `no_return.py` needs re-running.
"""

from __future__ import annotations

import numpy as np

#: Hand-back angle [deg]. Inside the cold recoverable set on BOTH sides at
#: standstill, with margin.
RECOVER_DEG = 12.0

#: Hand-back rate [rad/s]. A roll inside the window but still moving fast is
#: not a hand-off, it is a bike on its way past.
HANDOFF_RATE = 3.0


def ready_for_policy(roll_rad: float, roll_rate: float,
                     recover_deg: float = RECOVER_DEG,
                     handoff_rate: float = HANDOFF_RATE) -> bool:
    """True when roll [RADIANS] and roll_rate [rad/s] are inside the window.

    Both are BODY-frame and both are what the AHRS reports directly --
    `qpos[3:7]` through a quaternion, and `qvel[3]` -- so the same call works
    against mjData and against a TM151.

    THE ARGUMENT IS RADIANS AND THE THRESHOLD IS DEGREES, which is a trap worth
    naming because both callers nearly fell into it: `righting.roll_pitch`
    returns DEGREES, so the sequencer converts at its call site. Radians wins
    because that is what every other state signal in the repo and on the bike
    carries; degrees survive in the threshold because that is the unit
    `no_return.py` measures and publishes it in. Passing a degree-valued roll
    here is silently always-true for any plausible lean.
    """
    return abs(roll_rad) < np.deg2rad(recover_deg) and abs(roll_rate) < handoff_rate
