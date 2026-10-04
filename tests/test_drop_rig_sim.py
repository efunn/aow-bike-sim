"""analysis/drop_rig_sim.py: the drop rig in MuJoCo is the bench's rig.

What the contact sees is the static force and the effective mass at the
wheel (MuJoCo's soft contact scales with the latter), so those are pinned
here; plus that a drop from the cam's contact height lands, and that a
bouncy contact bounces.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import drop_rig_sim as rig  # noqa: E402

pytestmark = pytest.mark.analysis


def test_the_arm_carries_the_measured_rest_force_and_effective_mass():
    m, info = rig.build("front", rest_n=0.738, m_eff_g=89)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, M)
    hinge = m.jnt_dofadr[m.joint("arm_hinge").id]
    # free wheel: its spin stays out, so the hinge's inertia is m_eff r^2 alone
    spin = m.jnt_dofadr[m.joint("front_spin").id]
    i_hinge = M[hinge, hinge] - M[hinge, spin] ** 2 / M[spin, spin]
    assert i_hinge / info["r"] ** 2 == pytest.approx(0.089, rel=1e-3)
    runs, info = rig.drops([], rest_n=0.738, m_eff_g=89)
    assert info["rest_force_n"] == pytest.approx(0.738, rel=0.01)
    assert abs(info["rest_angle_deg"]) < 0.5            # the bar about level


def test_a_drop_lands_and_a_bouncy_contact_bounces():
    (t, f), = rig.drops([3.09], solref=[-30000, -25])[0]
    tt, a = rig.analyse(t, f, 3.09, 89)
    assert a is not None and a["bounces"] >= 1 and a["flight_ms"] > 10
    # released from rest at 3.09 mm, falling at F_rest / m_eff
    a_fall = 0.738 / 0.089
    assert t[np.flatnonzero(f > 0.1)[0]] == pytest.approx(np.sqrt(2 * 3.09e-3 / a_fall), abs=2e-3)


def test_solimp_survives_the_fit_encoding():
    solref, solimp = [-60659.4, -60.86], [0.1718, 0.4114, 0.001261, 0.5118, 4.268]
    back = rig._decode(rig._encode(solref, solimp), rig.SOLIMP, solimp)
    assert np.allclose(back[0], solref) and np.allclose(back[1], solimp)


def test_the_fit_recovers_a_known_contact_from_its_own_drops():
    """A 'bench' made by the rig itself at a known solref: the fit, started
    elsewhere, finds it again."""
    h_c, truth = [0.61, 1.44, 2.26, 3.09], [-40000.0, -50.0]
    runs, _ = rig.drops(h_c, solref=truth, record_s=0.2)
    tg = np.arange(-0.005, 0.1, 1e-4)
    bench = {}
    for h, (t, f) in zip(h_c, runs):
        bench[h] = (tg, np.interp(tg, t - rig.first_touch(t, f), f, left=0.0, right=0.0))
    solimp = list(rig.load_params()["sim"]["contact_solimp"])
    solref, _, rms = rig.fit(bench, h_c, h_c, [-63500.0, -48.0], solimp, fit_imp=(),
                             maxfev=150)
    assert rms < 0.05
    assert solref == pytest.approx(truth, rel=0.05)


def test_held_solimp_terms_stay_put():
    held = [0.9, 0.95, 0.001, 0.5, 2.0]
    z = rig._encode([-1e4, -10.0], [0.4, 0.95, 0.002, 0.5, 2.0], ("dmin", "width"))
    solref, imp = rig._decode(z, ("dmin", "width"), held)
    assert imp[0] == pytest.approx(0.4) and imp[2] == pytest.approx(0.002)
    assert imp[1] == 0.95 and imp[3:] == [0.5, 2.0]


@pytest.mark.parametrize("solimp", [[0.95, 0.95, 0.001, 0.5, 2], [0.9, 0.95, 0.001, 0.5, 2],
                                    [0.177, 0.95, 0.00126, 0.5, 2]])
def test_negative_solref_is_the_positive_one_spelled_differently(solimp):
    """(-k, -b) == (timeconst, dampratio) = (2/b, b / (2 sqrt k)), solimp and
    all: the same drop to the step. config/bike_params.yaml leans on this."""
    k, b = 60659.0, 60.9
    trace = []
    for sr in ([-k, -b], [2 / b, b / (2 * np.sqrt(k))]):
        imp = " ".join(map(str, solimp))
        m = mujoco.MjModel.from_xml_string(f"""<mujoco><option timestep="4e-4"/><worldbody>
            <geom type="plane" size="1 1 .1" solref="{sr[0]} {sr[1]}" solimp="{imp}"/>
            <body pos="0 0 0.03"><freejoint/><geom type="sphere" size="0.02" mass="0.089"
             solref="{sr[0]} {sr[1]}" solimp="{imp}"/></body></worldbody></mujoco>""")
        d = mujoco.MjData(m)
        z = []
        for _ in range(1000):
            mujoco.mj_step(m, d)
            z.append(d.qpos[2])
        trace.append(np.array(z))
    assert np.abs(trace[0] - trace[1]).max() < 1e-9
