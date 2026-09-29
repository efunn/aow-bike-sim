"""Session-wide plumbing: the accepted-red registry.

The suite has a standing red set (see `expected_failures.txt`), and the useful
question at the end of a run is never "did anything fail" -- something always
does -- but "did the red set MOVE". That is what this reports:

    red set unchanged (7 accepted failures)

or, when it moved, the three ways it can move, named individually:

    NEWLY RED            a failure nobody signed off on
    UNEXPECTEDLY GREEN   a listed test now passes; the registry is stale
    STALE ENTRY          a listed nodeid was not collected at all (renamed,
                         deleted, or deselected by -k / -m)

Deliberately NOT xfail. An xfail hides the failure, and a non-strict one
reports a regression as a green run -- the exact objection docs/status.md
raises against xfailing `test_lqr_model_fit_and_steering`. These tests still
run, still fail, and still make the session exit non-zero. The registry adds a
verdict on top; it never suppresses anything.

The section prints just above pytest's own "short test summary info" --
the terminal reporter is a hookwrapper that emits that list after every
plugin hook has run, so no hook ordering puts this below it.

The exit status is left alone on purpose. A shrinking red set is good news and
should not be reported as breakage, and a growing one already exits non-zero
because the test genuinely failed.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REGISTRY = Path(__file__).parent / "expected_failures.txt"


def _accepted() -> dict[str, str]:
    """nodeid -> reason, from the registry. Empty if the file is missing."""
    if not REGISTRY.exists():
        return {}
    out = {}
    for line in REGISTRY.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        nodeid, _, reason = line.partition("#")
        out[nodeid.strip()] = reason.strip()
    return out


def _same(a, b) -> bool:
    import numpy as np
    if isinstance(a, (tuple, list)):
        return type(a) is type(b) and len(a) == len(b) and all(map(_same, a, b))
    return bool(np.array_equal(a, b))


@pytest.fixture(scope="module")
def lqr_design_once():
    """Hand out each distinct LQR design once per module, as a copy.

    A DriveController re-identifies the plant and designs its gain schedule on
    EVERY construction -- deliberately, so the product can never fly a stale
    design (control/balance.py says why it is not cached). A test file that
    starts one per test against the same params and model pays that each time:
    measured 2026-09-29, 37 of test_teleop's 50 s, 23 of test_drive's 32, 33 of
    test_odometry_in_the_loop's 46, all of test_hw_replay's 11.

    Keyed on the params digest and the model OBJECT (kept alive, so its id
    stays unique), plus the call's other arguments. Teardown re-designs every
    entry from scratch and asserts it is identical to what was handed out, so a
    model mutated in place under the memo fails loudly instead of flying a
    stale design. Opt in per file: `pytest.mark.usefixtures("lqr_design_once")`.
    Files that test the design itself should not.
    """
    import copy

    from aow_sim.control import linearize as lz
    from aow_sim.params import params_digest

    memo = {}

    def once(fn):
        def wrapped(params, model, *a, **k):
            key = (fn.__name__, params_digest(params), id(model), a,
                   tuple(sorted(k.items())))
            if key not in memo:
                memo[key] = (fn, copy.deepcopy(params), model, a, k,
                             fn(params, model, *a, **k))
            return copy.deepcopy(memo[key][-1])
        return wrapped

    originals = {n: getattr(lz, n) for n in
                 ("design_lqr", "design_gain_schedule", "design_crawl_fallback")}
    with pytest.MonkeyPatch.context() as mp:
        for name, fn in originals.items():
            mp.setattr(lz, name, once(fn))
        yield
    for key, (fn, params, model, a, k, got) in memo.items():
        assert _same(fn(params, model, *a, **k), got), (
            f"{key[0]} memoised in this module no longer matches a fresh "
            "design: something changed the model in place. Drop "
            "lqr_design_once from this file.")


def pytest_collection_modifyitems(config, items):
    """`prospective` tests are targets no policy clears yet, so the default run
    skips them. Naming them runs them: `-m` mentioning `prospective`, or a
    path argument into their file. A skip rather than a deselect keeps them
    seen, so their registry entry is not reported STALE."""
    if "prospective" in (config.option.markexpr or ""):
        return
    named = [str(a).split("::")[0] for a in config.option.file_or_dir or []]
    skip = pytest.mark.skip(reason="prospective: run with `pytest -m prospective`")
    for item in items:
        if "prospective" in item.keywords and not any(
                str(item.path).endswith(n) for n in named if n.endswith(".py")):
            item.add_marker(skip)


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    accepted = _accepted()
    if not accepted or config.option.collectonly:
        return
    tr = terminalreporter

    # Errors count as red too: a test that cannot even set up is not passing,
    # and collapsing the two here keeps the registry from having to care which
    # phase broke.
    red = {r.nodeid for k in ("failed", "error") for r in tr.stats.get(k, [])}
    green = {r.nodeid for r in tr.stats.get("passed", [])}
    # Anything that ran at all. A nodeid outside this set was not collected,
    # which is a different thing from passing -- `-k` and `-m` both produce it,
    # so it is reported separately rather than as an unexpected pass.
    seen = red | green | {r.nodeid for r in tr.stats.get("skipped", [])}

    new_red = sorted(red - set(accepted))
    now_green = sorted(green & set(accepted))
    # Under any restriction most of the registry is deselected rather than
    # gone, so a STALE list would be almost entirely noise and would bury the
    # two verdicts that DO survive a filter. Only an unqualified run can tell
    # stale from deselected.
    #
    # `file_or_dir` is the positional args, so `pytest tests/test_steer.py`
    # counts as filtered too -- it deselects exactly as thoroughly as -k does.
    # This errs toward silence on purpose: a false STALE is noise that erodes
    # trust in the whole report, while a missed one just surfaces on the next
    # full run. NEWLY RED is never suppressed either way.
    filtered = bool(config.option.keyword or config.option.markexpr
                    or config.option.file_or_dir)
    stale = [] if filtered else sorted(set(accepted) - seen)

    hit = red & set(accepted)          # accepted failures that actually ran

    # A filtered run that touched none of this has nothing to say. Staying
    # quiet keeps the marker workflow (`pytest -m pure`, 0.2 s) clean.
    if filtered and not (new_red or now_green or hit):
        return

    tr.write_sep("=", "accepted-red registry", bold=True)
    if not (new_red or now_green or stale):
        n = len(hit)
        # Say "of this selection" rather than implying the whole registry was
        # checked -- a -k run that happens to hit all seven still only proves
        # it about the seven it ran.
        scope = " in this selection" if filtered else ""
        tr.write_line(f"red set unchanged{scope} ({n} accepted failure"
                      f"{'' if n == 1 else 's'}) -- tests/expected_failures.txt",
                      green=True)
        return

    for nodeid in new_red:
        tr.write_line(f"NEWLY RED           {nodeid}", red=True, bold=True)
    for nodeid in now_green:
        tr.write_line(f"UNEXPECTEDLY GREEN  {nodeid}", yellow=True)
    for nodeid in stale:
        tr.write_line(f"STALE ENTRY         {nodeid}", yellow=True)
    if filtered:
        tr.write_line("(partial run -- this is only the selected tests)")

    if new_red:
        tr.write_line("")
        tr.write_line("A newly red test is a regression until shown otherwise. "
                      "If it is an accepted cost, add it to "
                      "tests/expected_failures.txt WITH THE REASON -- an "
                      "accepted cost with no note becomes an unnoticed one.")
    if now_green or stale:
        tr.write_line("")
        tr.write_line("Drop those lines from tests/expected_failures.txt, and "
                      "say in docs/status.md what fixed them.")
