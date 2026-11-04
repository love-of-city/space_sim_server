"""Check the separately distributed Earth package before starting the renderer.

The fast startup check reads metadata, file sizes and short headers. Use the repository's
external_environment_assets.py verify --full command for SHA256 validation.
This script and Config/ExternalAssets are self-contained in deployed UE copies.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path, PurePosixPath


def check_external_assets(project: Path) -> None:
    project = Path(project).resolve()
    manifest_path = project / "Config/ExternalAssets/incredible-earth.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"External Earth manifest missing: {manifest_path}; sync Config and scripts from the same code version")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "space-sim-external-assets/1":
        raise RuntimeError(f"Unsupported external Earth manifest: {manifest_path}")
    files = manifest.get("files")
    if (not isinstance(files, list) or not files
            or manifest.get("file_count") != len(files)
            or manifest.get("content_root") != "ue-adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth"
            or any(not isinstance(item.get("size_bytes"), int) or item["size_bytes"] <= 0 for item in files)
            or manifest.get("total_size_bytes") != sum(item["size_bytes"] for item in files)
            or len({item["path"].casefold() for item in files}) != len(files)):
        raise RuntimeError(f"Invalid or incomplete external Earth manifest: {manifest_path}")
    root = project / "Content/IncredibleEarth"
    problems = []
    for item in files:
        relative = PurePosixPath(item["path"])
        if relative.is_absolute() or ".." in relative.parts or any(c in item["path"] for c in ("\\", ":")):
            raise RuntimeError(f"Invalid external asset path: {item['path']}")
        path = root / relative
        if not path.is_file():
            problems.append(f"missing {relative}")
        elif path.stat().st_size != item["size_bytes"]:
            problems.append(f"wrong size (or LFS pointer) {relative}")
        else:
            with path.open("rb") as stream:
                if stream.read(100).startswith(b"version https://git-lfs.github.com/spec/v1"):
                    problems.append(f"LFS pointer {relative}")
    if problems:
        sample = "; ".join(problems[:5])
        raise RuntimeError(
            f"External Earth assets are not ready ({len(problems)} files): {sample}. "
            f"Restore package {manifest['package_id']} {manifest['package_version']} into {root}. "
            "git lfs pull does not download this package. "
            "See docs/guides/external-environment-assets.md in the source repository."
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        check_external_assets(args.project)
    except (RuntimeError, ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")
    print("External Earth assets present; full checksum verification is a separate step.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
