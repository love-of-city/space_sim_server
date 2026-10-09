"""Check required Basilisk imports/APIs without pinning its source or loading Python mujoco."""
from __future__ import annotations

import ast
import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = ["simulation", "model/SARM/platform/scenarios/scenario_sarm_grasp.py"]


def requirements():
    required = {("Basilisk.simulation.mujoco", "MJScene")}
    for entry in SOURCE_ROOTS:
        location = ROOT / entry
        for path in ([location] if location.is_file() else location.rglob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8-sig"))
            aliases = {}
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom) and node.module and node.module.startswith("Basilisk"):
                    for name in node.names:
                        required.add((node.module, name.name))
                        aliases[name.asname or name.name] = node.module + "." + name.name
            for node in ast.walk(tree):
                if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id in aliases:
                    required.add((aliases[node.value.id], node.attr))
    return sorted(required)


def main():
    failures = []
    for module_name, attribute in requirements():
        try:
            module = importlib.import_module(module_name)
            if not hasattr(module, attribute):
                # Python's 'from package import child' also imports submodules.
                importlib.import_module(module_name + "." + attribute)
        except Exception as error:
            failures.append(f"{module_name}.{attribute}: {type(error).__name__}: {error}")
    if failures:
        print("Missing/unloadable Basilisk capabilities (check package features and Python ABI):")
        print("\n".join(failures))
        return 1
    print("Required Basilisk APIs available; real simulation/UE acceptance must run separately.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
