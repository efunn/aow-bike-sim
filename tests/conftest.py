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
