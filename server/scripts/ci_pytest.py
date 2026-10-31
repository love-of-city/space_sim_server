"""Opt-in basic CI selection; default pytest discovery remains unchanged."""
from __future__ import annotations

import ast
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
EXCLUSIONS = json.loads((Path(__file__).with_name("ci-exclusions.json")).read_text(encoding="utf-8"))
NATIVE = json.loads((Path(__file__).with_name("native-test-profile.json")).read_text(encoding="utf-8"))
_skipped = []


def _node_exists(node_id: str) -> bool:
    relative, separator, test_name = node_id.partition("::")
    if not separator or not test_name:
        return False
    path = ROOT / relative
    if not path.is_file() or path.suffix != ".py":
        return False
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except (OSError, SyntaxError, UnicodeDecodeError):
        return False
    return any(
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == test_name
        for node in ast.walk(tree)
    )


def pytest_addoption(parser):
    parser.addoption("--ci-native", action="store_true", help="Run the explicit native profile; missing dependencies and skips fail")
    parser.addoption("--ci-basic", action="store_true", help="Exclude documented specialized tests; fail on skips")
    parser.addoption("--ci-skip-group", action="append", default=[], metavar="NAME",
                     help="With --ci-basic, also exclude a documented group from ci-exclusions.json")


def _skipped_group_files(config):
    groups = EXCLUSIONS.get("groups", {})
    return {name for group in config.getoption("--ci-skip-group") for name in groups[group]["files"]}


def _skipped_group_nodes(config):
    groups = EXCLUSIONS.get("groups", {})
    return {name for group in config.getoption("--ci-skip-group") for name in groups[group].get("nodes", [])}


def pytest_configure(config):
    _skipped.clear()
    if config.getoption("--ci-native") and config.getoption("--ci-basic"):
        raise pytest.UsageError("Choose exactly one CI profile: basic or native")
    if config.getoption("--ci-native"):
        missing = [name for name in NATIVE["files"] if not (ROOT / name).is_file()]
        if missing:
            raise pytest.UsageError(f"Stale native profile: {missing}")
    unknown = sorted(set(config.getoption("--ci-skip-group")) - set(EXCLUSIONS.get("groups", {})))
    if unknown:
        raise pytest.UsageError(f"Unknown CI groups: {unknown}")


def pytest_ignore_collect(collection_path, config):
    if config.getoption("--ci-native") and collection_path.is_file():
        return collection_path.relative_to(ROOT).as_posix() not in NATIVE["files"]
    if not config.getoption("--ci-basic"):
        return None
    try:
        relative = collection_path.relative_to(ROOT).as_posix()
    except ValueError:
        return None
    if relative in EXCLUSIONS["files"] or relative in _skipped_group_files(config):
        return True
    return None


def pytest_collection_modifyitems(config, items):
    if config.getoption("--ci-native"):
        selected = [item for item in items if item.path.relative_to(ROOT).as_posix() in NATIVE["files"]]
        excluded = [item for item in items if item not in selected]
        items[:] = selected
        if excluded:
            config.hook.pytest_deselected(items=excluded)
        return
    if not config.getoption("--ci-basic"):
        return
    # Refuse stale file exclusions; renames must update the reviewed manifest.
    group_files = [name for group in EXCLUSIONS.get("groups", {}).values() for name in group["files"]]
    missing = [name for name in [*EXCLUSIONS["files"], *group_files] if not (ROOT / name).is_file()]
    if missing:
        raise pytest.UsageError(f"Stale CI exclusions: {missing}")
    # Refuse stale node exclusions; renamed tests must update the reviewed manifest.
    group_nodes = [node for group in EXCLUSIONS.get("groups", {}).values() for node in group.get("nodes", [])]
    missing_nodes = sorted(node for node in [*EXCLUSIONS["nodes"], *group_nodes] if not _node_exists(node))
    if missing_nodes:
        raise pytest.UsageError(f"Stale CI node exclusions: {missing_nodes}")
    kept, excluded = [], []
    for item in items:
        relative = item.path.relative_to(ROOT).as_posix()
        key = relative + "::" + getattr(item, "originalname", item.name)
        (excluded if key in EXCLUSIONS["nodes"] or relative in EXCLUSIONS["files"]
         or relative in _skipped_group_files(config) or key in _skipped_group_nodes(config) else kept).append(item)
    items[:] = kept
    if excluded:
        config.hook.pytest_deselected(items=excluded)
    # Under xdist every worker collects; only one may write the shared report.
    if getattr(config, "workerinput", {}).get("workerid", "gw0") != "gw0":
        return
    report = ROOT / "reports" / "coverage-scope.json"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps({
        "profile": "basic", "selected": len(kept),
        "excluded_files": EXCLUSIONS["files"],
        "excluded_nodes": EXCLUSIONS["nodes"],
        "skipped_groups": {name: EXCLUSIONS["groups"][name] for name in config.getoption("--ci-skip-group")},
        "not_validated": ["UE C++/GPU/WebRTC", "real Basilisk dynamics", "full end-to-end capture"],
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def pytest_collectreport(report):
    if report.skipped:
        _skipped.append(report.nodeid)


def pytest_runtest_logreport(report):
    if report.skipped:
        _skipped.append(report.nodeid)


def pytest_sessionfinish(session, exitstatus):
    if (session.config.getoption("--ci-basic") or session.config.getoption("--ci-native")) and _skipped:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter):
    if terminalreporter.config.getoption("--ci-native"):
        terminalreporter.write_sep("=", "Native CI scope: scripts/native-test-profile.json (not UE/GPU acceptance)")
        if _skipped:
            terminalreporter.write_line("Missing native dependencies / unexpected skips fail: " + ", ".join(_skipped), red=True)
    if terminalreporter.config.getoption("--ci-basic"):
        terminalreporter.write_sep("=", "Basic CI scope: reports/coverage-scope.json (not full system acceptance)")
        if _skipped:
            terminalreporter.write_line("Unexpected skips/xfails are failures: " + ", ".join(_skipped), red=True)

