"""Explicit, non-deleting source-to-runtime UE project synchronization.

Stop the renderer before applying. Dry-run is read-only. A previous sync
manifest permits safe updates; edits made in the runtime copy are conflicts.
Authored environment levels require their own explicit overwrite flag.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile

PROJECT_RELATIVE = Path("Unreal/BskUnrealRenderer")
PROJECT_FILE = "BskUnrealRenderer.uproject"
LEVEL = "Content/BSK/Maps/BskEnvironment.umap"
ENVIRONMENT_ASSETS = "Content/BSK/Environment/"
MANIFEST = ".bsk-source-sync.json"
SOURCE_DIRECTORIES = ("Config", "Content", "ContentSource", "Plugins", "Source", "scripts")
GENERATED_DIRECTORIES = {
    "binaries", "intermediate", "saved", "deriveddatacache", "dist", "__pycache__",
    ".git", ".vs", ".idea", ".vscode", "node_modules", "developers", "collections",
}


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def linked(path: Path) -> bool:
    """Also reject Windows junctions, which are not reported as symlinks."""
    if path.is_symlink():
        return True
    try:
        return bool(getattr(path.lstat(), "st_file_attributes", 0)
                    & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0))
    except FileNotFoundError:
        return False


def machine_file(path: Path) -> bool:
    name = path.name.lower()
    return (".local." in name or name.endswith(".local") or name.startswith(".env")
            or name.startswith(".space-sim-") or name.endswith((".pyc", ".pyo", ".log"))
            or (path.parts[0].lower() == "config" and name.startswith("user")))


def source_files(project: Path):
    yield Path(PROJECT_FILE)
    for directory in SOURCE_DIRECTORIES:
        root = project / directory
        if not root.exists():
            continue
        if linked(root):
            raise ValueError(f"Source directory is a symlink: {root}")
        for current, directories, names in os.walk(root, followlinks=False):
            directories[:] = sorted(name for name in directories
                                    if name.lower() not in GENERATED_DIRECTORIES)
            for name in directories:
                if linked(Path(current) / name):
                    raise ValueError(f"Source directory is a symlink: {Path(current) / name}")
            for name in sorted(names):
                path = Path(current) / name
                relative = path.relative_to(project)
                if machine_file(relative):
                    continue
                if linked(path):
                    raise ValueError(f"Source file is a symlink: {path}")
                yield relative


def safe_destination(project: Path, relative: Path) -> Path:
    path = project
    for part in relative.parts:
        path = path / part
        if linked(path):
            raise ValueError(f"Destination symlink is not allowed: {path}")
    if not path.resolve().is_relative_to(project.resolve()):
        raise ValueError(f"Destination escapes the project: {path}")
    return path


def plan_sync(source_adapter: Path, destination_adapter: Path, *, overwrite=False, overwrite_level=False):
    unresolved_destination = destination_adapter.expanduser().absolute()
    for path in (unresolved_destination, *unresolved_destination.parents):
        if linked(path):
            raise ValueError(f"Destination directory is a symlink or junction: {path}")
    source_adapter = source_adapter.expanduser().resolve()
    destination_adapter = destination_adapter.expanduser().resolve()
    if (source_adapter.is_relative_to(destination_adapter)
            or destination_adapter.is_relative_to(source_adapter)):
        raise ValueError("Source and destination must be distinct, non-overlapping adapter directories")
    source = source_adapter / PROJECT_RELATIVE
    destination = destination_adapter / PROJECT_RELATIVE
    if not (source / PROJECT_FILE).is_file():
        raise FileNotFoundError(f"Source UE project missing: {source / PROJECT_FILE}")
    # Refuse a symlink at any existing destination level, including its root.
    for path in (destination_adapter, *destination_adapter.parents, destination.parent, destination):
        if linked(path):
            raise ValueError(f"Destination directory is a symlink: {path}")
    manifest_path = safe_destination(destination, Path(MANIFEST))
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    previous = manifest.get("files", {})
    actions = []
    for relative in source_files(source):
        path = source / relative
        with path.open("rb") as stream:
            if stream.read(200).startswith(b"version https://git-lfs.github.com/spec/v1"):
                raise RuntimeError(f"Source is a Git LFS pointer: {path}; run git lfs pull before synchronizing")
        key = relative.as_posix()
        source_hash = digest(path)
        target = safe_destination(destination, relative)
        if not target.exists():
            action = "create"
        elif not target.is_file():
            raise ValueError(f"Destination is not a regular file: {target}")
        else:
            target_hash = digest(target)
            if target_hash == source_hash:
                action = "unchanged"
            elif (key == LEVEL or key.startswith(ENVIRONMENT_ASSETS)) and not overwrite_level:
                action = "conflict"
            elif (key == LEVEL or key.startswith(ENVIRONMENT_ASSETS)) and overwrite_level:
                action = "update"
            elif overwrite or previous.get(key) == target_hash:
                action = "update"
            else:
                action = "conflict"
        actions.append({"path": key, "action": action, "sha256": source_hash})
    return source, destination, actions


def synchronize(source_adapter: Path, destination_adapter: Path, *, dry_run=False,
                overwrite=False, overwrite_level=False):
    source, destination, actions = plan_sync(source_adapter, destination_adapter,
                                             overwrite=overwrite, overwrite_level=overwrite_level)
    conflicts = [item["path"] for item in actions if item["action"] == "conflict"]
    report = {"source": str(source), "destination": str(destination), "dry_run": dry_run,
              "counts": {kind: sum(item["action"] == kind for item in actions)
                         for kind in ("create", "update", "unchanged", "conflict")},
              "conflicts": conflicts,
              "changes": [{"path": item["path"], "action": item["action"]}
                          for item in actions if item["action"] != "unchanged"]}
    if dry_run:
        return report
    if conflicts:
        raise RuntimeError("Sync aborted before copying; preserve/reconcile runtime edits or explicitly approve overwrite: "
                           + ", ".join(conflicts[:20]))
    for item in actions:
        if item["action"] == "unchanged":
            continue
        relative = Path(item["path"])
        target = safe_destination(destination, relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        # Source may have been saved in the editor after planning. Never deploy
        # an unreviewed mix of the planned file and its newer replacement.
        if digest(source / relative) != item["sha256"]:
            raise RuntimeError(f"Source changed during sync; rerun: {source / relative}")
        fd, temporary = tempfile.mkstemp(prefix=".bsk-sync-", dir=target.parent)
        os.close(fd)
        try:
            shutil.copy2(source / relative, temporary)
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    destination.mkdir(parents=True, exist_ok=True)
    manifest = {"version": 1, "source": str(source),
                "files": {item["path"]: item["sha256"] for item in actions}}
    manifest_path = safe_destination(destination, Path(MANIFEST))
    fd, temporary = tempfile.mkstemp(prefix=".bsk-sync-", dir=destination)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2)
            stream.write("\n")
        os.replace(temporary, manifest_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-adapter", type=Path, required=True)
    parser.add_argument("--destination-adapter", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--overwrite-conflicts", action="store_true",
                        help="Explicitly replace conflicting runtime source files; environment level stays protected")
    parser.add_argument("--overwrite-environment-level", action="store_true",
                        help="Explicitly replace saved environment level and Content/BSK/Environment assets from source")
    args = parser.parse_args()
    report = synchronize(args.source_adapter, args.destination_adapter, dry_run=args.dry_run,
                         overwrite=args.overwrite_conflicts, overwrite_level=args.overwrite_environment_level)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if report["conflicts"]:
        parser.exit(2, "Conflicts found; nothing was copied.\n")


if __name__ == "__main__":
    main()
