"""Fail if Python and UE adapter release version strings diverge."""
import json
from pathlib import Path
import tomllib

root = Path(__file__).resolve().parents[1]
versions = {
    "VERSION": (root / "VERSION").read_text().strip(),
    "project": tomllib.loads((root / "pyproject.toml").read_text())["project"]["version"],
    "compatibility_package": tomllib.loads((root / "Unreal/BskUnrealRenderer/python/pyproject.toml").read_text())["project"]["version"],
    "UE VersionName": json.loads((root / "Unreal/BskUnrealRenderer/Plugins/BskUnrealRuntime/BskUnrealRuntime.uplugin").read_text())["VersionName"],
}
if len(set(versions.values())) != 1:
    raise SystemExit(f"Version mismatch: {versions}")
print(versions)

