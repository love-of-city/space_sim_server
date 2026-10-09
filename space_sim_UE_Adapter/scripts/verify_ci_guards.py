"""Exercise actual pytest exit codes and version mismatch behavior in throwaway files."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
cache = root / ".pytest_cache"
cache.mkdir(exist_ok=True)
environment = dict(os.environ, PYTHONPATH=str(root / "scripts"), PYTEST_ADDOPTS="")
cases = {
    "pass": ("def test_probe(): assert True\n", 0),
    "failure": ("def test_probe(): assert False\n", 1),
    "skip": ("import pytest\ndef test_probe(): pytest.skip('probe')\n", 1),
    "collection_skip": ("import pytest\npytest.skip('probe', allow_module_level=True)\n", 1),
    "xfail": ("import pytest\n@pytest.mark.xfail\ndef test_probe(): assert False\n", 1),
}
with tempfile.TemporaryDirectory(dir=cache) as directory:
    probe = Path(directory) / "test_probe.py"
    for name, (source, expected) in cases.items():
        probe.write_text(source, encoding="utf-8")
        result = subprocess.run(
            [sys.executable, "-m", "pytest", "-p", "ci_pytest", "--ci-basic", str(probe), "-q"],
            cwd=root, env=environment, capture_output=True, text=True,
        )
        if result.returncode != expected:
            raise SystemExit(f"{name}: expected {expected}, got {result.returncode}\n{result.stdout}\n{result.stderr}")
        print(f"{name}: exit {result.returncode}, correct")
    # Optional adapter guard: verify mismatch is rejected without modifying the checkout.
    if (root / "scripts/check_versions.py").exists():
        sandbox = Path(directory) / "version-probe"
        for name in [
            "scripts/check_versions.py", "VERSION", "pyproject.toml",
            "Unreal/BskUnrealRenderer/python/pyproject.toml",
            "Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/BskUnrealRuntime.uplugin",
        ]:
            destination = sandbox / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root / name, destination)
        (sandbox / "VERSION").write_text("99.0.0\n")
        result = subprocess.run([sys.executable, str(sandbox / "scripts/check_versions.py")], capture_output=True, text=True)
        if result.returncode != 1 or "Version mismatch" not in result.stderr:
            raise SystemExit("Version mismatch guard failed")
        print("version mismatch: rejected")

