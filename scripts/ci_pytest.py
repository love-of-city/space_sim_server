"""Opt-in basic CI selection; default pytest discovery remains unchanged."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
EXCLUSIONS = json.loads((Path(__file__).with_name("ci-exclusions.json")).read_text(encoding="utf-8"))
_skipped = []


def pytest_addoption(parser):
    parser.addoption("--ci-basic", action="store_true", help="Exclude documented specialized tests; fail on skips")


def pytest_configure(config):
    _skipped.clear()


def pytest_ignore_collect(collection_path, config):
    if not config.getoption("--ci-basic"):
        return None
    try:
        relative = collection_path.relative_to(ROOT).as_posix()
    except ValueError:
        return None
    if relative in EXCLUSIONS["files"]:
        return True
    return None


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--ci-basic"):
        return
    # Refuse stale file exclusions; renames must update the reviewed manifest.
    missing = [name for name in EXCLUSIONS["files"] if not (ROOT / name).is_file()]
    if missing:
        raise pytest.UsageError(f"Stale CI exclusions: {missing}")
    kept, excluded = [], []
    for item in items:
        relative = item.path.relative_to(ROOT).as_posix()
        key = relative + "::" + getattr(item, "originalname", item.name)
        (excluded if key in EXCLUSIONS["nodes"] else kept).append(item)
    items[:] = kept
    if excluded:
        config.hook.pytest_deselected(items=excluded)
    report = ROOT / "reports" / "coverage-scope.json"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps({
        "profile": "basic", "selected": len(kept),
        "excluded_files": EXCLUSIONS["files"],
        "excluded_nodes": EXCLUSIONS["nodes"],
        "not_validated": ["UE C++/GPU/WebRTC", "real Basilisk dynamics", "full end-to-end capture"],
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def pytest_collectreport(report):
    if report.skipped:
        _skipped.append(report.nodeid)


def pytest_runtest_logreport(report):
    if report.skipped:
        _skipped.append(report.nodeid)


def pytest_sessionfinish(session, exitstatus):
    if session.config.getoption("--ci-basic") and _skipped:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter):
    if terminalreporter.config.getoption("--ci-basic"):
        terminalreporter.write_sep("=", "Basic CI scope: reports/coverage-scope.json (not full system acceptance)")
        if _skipped:
            terminalreporter.write_line("Unexpected skips/xfails are failures: " + ", ".join(_skipped), red=True)

