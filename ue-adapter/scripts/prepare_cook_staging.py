"""Stage only declared UE inputs; local Generated assets are never discovered implicitly."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess

ADAPTER = Path(__file__).resolve().parents[1]
REPO = ADAPTER.parent


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def contained(root: Path, value: str) -> Path:
    relative = PurePosixPath(value)
    if relative.is_absolute() or ".." in relative.parts or "\\" in value or not relative.parts:
        raise ValueError(f"Expected a package-relative path: {value}")
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"Path escapes input root: {value}")
    return path


def load_assets(manifests: list[Path], project: Path) -> tuple[dict[str, str], list[str], list[str]]:
    assets: dict[str, str] = {}
    cook_roots, runtime_assets = set(), set()
    for path in manifests:
        manifest = json.loads(path.read_text(encoding="utf-8"))
        if manifest.get("schema") != "ue-cook-inputs/1":
            raise ValueError(f"Unsupported Cook manifest: {path}")
        for rel, sha in manifest["files"].items():
            if not rel.startswith("Content/") or Path(rel).suffix not in (".uasset", ".umap", ".ubulk", ".uexp"):
                raise ValueError(f"Cook asset is not a project Content package: {rel}")
            source = contained(project, rel)
            if not source.is_file():
                raise FileNotFoundError(source)
            with source.open("rb") as stream:
                if stream.read(100).startswith(b"version https://git-lfs.github.com/spec/v1"):
                    raise ValueError(f"Unresolved Git LFS pointer: {source}")
            if digest(source) != sha:
                raise ValueError(f"Cook input hash mismatch: {source}")
            if rel in assets and assets[rel] != sha:
                raise ValueError(f"Conflicting Cook input: {rel}")
            assets[rel] = sha
        for root in manifest.get("cook_directories", []):
            if not root.startswith(("/Game/", "/CelestialVault/")) or ".." in root or any(c in root for c in '\n\r"'):
                raise ValueError(f"Invalid UE package root: {root}")
            cook_roots.add(root)
        runtime_assets.update(manifest.get("runtime_assets", []))
    for rel in assets:
        if rel.endswith(".uasset"):
            package = "/Game/" + str(PurePosixPath(rel).relative_to("Content").with_suffix(""))
            runtime_assets.add(package + "." + PurePosixPath(package).name)
    return dict(sorted(assets.items())), sorted(cook_roots), sorted(runtime_assets)


def tracked_project_inputs(project: Path) -> list[Path]:
    result = subprocess.run(["git", "-C", str(project), "ls-files", "--cached", "--others", "--exclude-standard", "-z", "--", "."],
                            check=True, capture_output=True)
    deleted = set(subprocess.run(["git", "-C", str(project), "ls-files", "--deleted", "-z", "--", "."],
                                 check=True, capture_output=True).stdout.decode().split("\0"))
    inputs = []
    for name in sorted(set(result.stdout.decode().split("\0"))):
        if not name or name in deleted:
            continue
        rel = PurePosixPath(name)
        if rel.parts[0] in ("Source", "Config", "Plugins") or rel.suffix == ".uproject":
            source = contained(project, name)
            if not source.is_file():
                raise FileNotFoundError(source)
            inputs.append(source)
    if not any(path.suffix == ".uproject" for path in inputs):
        raise ValueError("No tracked UE project found")
    return inputs


def prepare(project: Path, output: Path, manifests: list[Path]) -> dict:
    project, output = project.resolve(), output.resolve()
    if output.is_relative_to(project) or project.is_relative_to(output):
        raise ValueError("Cook staging must be separate from the input project")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Cook staging must be empty; refusing to mix build inputs")
    assets, cook_roots, runtime_assets = load_assets(manifests, project)
    sources = tracked_project_inputs(project)
    output.mkdir(parents=True, exist_ok=True)
    for source in sources + [project / rel for rel in assets]:
        destination = output / source.relative_to(project)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    # Replace all inherited broad project cook roots with precisely selected roots.
    game_ini = output / "Config/DefaultGame.ini"
    text = game_ini.read_text(encoding="utf-8-sig")
    lines = [line for line in text.splitlines() if "DirectoriesToAlwaysCook=" not in line]
    section = "[/Script/UnrealEd.ProjectPackagingSettings]"
    index = lines.index(section) + 1 if section in lines else len(lines)
    if section not in lines:
        lines.append(section)
        index = len(lines)
    lines[index:index] = ["!DirectoriesToAlwaysCook=ClearArray"] + [f'+DirectoriesToAlwaysCook=(Path="{root}")' for root in cook_roots]
    game_ini.write_text("\n".join(lines) + "\n", encoding="utf-8")
    source_hashes = {str(path.relative_to(output)): digest(path) for path in sorted(output.rglob("*")) if path.is_file()}
    result = {"schema": "ue-cook-staging/1", "files": source_hashes, "asset_files": assets,
              "input_manifests": [{"name": path.name, "sha256": digest(path)} for path in manifests],
              "cook_directories": cook_roots,
              "validation": {"packaged_runtime": "not-run", "gpu": "not-run"}}
    (output / "cook-input-manifest.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "cook-runtime-assets.json").write_text(json.dumps({"schema": "ue-cook-runtime-assets/1",
        "assets": [{"object_path": path} for path in runtime_assets]}, indent=2) + "\n")
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=ADAPTER / "Unreal/BskUnrealRenderer")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, action="append")
    args = parser.parse_args(argv)
    manifests = args.manifest or [REPO / "manifests/assets/ue-cook-base-assets.json"]
    result = prepare(args.project, args.output, manifests)
    print(json.dumps({"staging": str(args.output.resolve()), "assets": len(result["asset_files"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
