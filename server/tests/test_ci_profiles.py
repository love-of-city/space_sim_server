"""CI profiles run their declared tests and cannot hide missing dependencies."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize("profile", ["basic", "native"])
@pytest.mark.parametrize("skip", [False, True])
def test_profile_selects_declared_layer_and_rejects_skip(tmp_path, profile, skip):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    tests = tmp_path / "tests"
    tests.mkdir()
    root = Path(__file__).resolve().parents[1]
    (scripts / "ci_pytest.py").write_bytes((root / "scripts/ci_pytest.py").read_bytes())
    (scripts / "ci-exclusions.json").write_text(json.dumps({
        "files": {"tests/test_native.py": "native-only"}, "nodes": {}, "groups": {}}))
    (scripts / "native-test-profile.json").write_text(json.dumps({"files": ["tests/test_native.py"]}))
    selected = tests / ("test_native.py" if profile == "native" else "test_basic.py")
    other = tests / ("test_basic.py" if profile == "native" else "test_native.py")
    selected.write_text("import pytest\ndef test_run(): " + ("pytest.skip('missing native dependency')" if skip else "assert True"))
    other.write_text("raise RuntimeError('wrong profile imported')\n")
    env = dict(os.environ, PYTHONPATH=str(scripts), PYTEST_ADDOPTS="", PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    result = subprocess.run([sys.executable, "-m", "pytest", "-p", "ci_pytest", "--ci-" + profile,
                             "-q", "tests"], cwd=tmp_path, env=env, capture_output=True, text=True)
    assert result.returncode == (1 if skip else 0), result.stdout + result.stderr
    assert "wrong profile imported" not in result.stdout
