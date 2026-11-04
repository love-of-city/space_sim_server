"""Verify and exchange the versioned external UE environment asset package.

This is an explicit release tool, not an automatic download or runtime hook.
Only the Python standard library is required. Archive paths always use the
current repository layout; --content-root can read an older local checkout.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tempfile
import zipfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = Path("manifests/external-assets/incredible-earth.json")
CONTENT_ROOT = "ue-adapter/Unreal/BskUnrealRenderer/Content/IncredibleEarth"
SCHEMA = "space-sim-external-assets/1"
CHUNK_SIZE = 4 * 1024 * 1024
LFS_HEADER = b"version https://git-lfs.github.com/spec/v1"


def relative_path(value: str) -> PurePosixPath:
    """Accept portable relative file names, without Windows path aliases."""
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise ValueError(f"Unsafe relative path: {value!r}")
    parts = value.split("/")
    reserved = {"CON", "PRN", "AUX", "NUL", *(f"COM{i}" for i in range(1, 10)),
                *(f"LPT{i}" for i in range(1, 10))}
    if any(part in ("", ".", "..") or part.endswith((".", " "))
           or part.split(".")[0].upper() in reserved
           or any(ord(c) < 32 or c in '<>"|?*' for c in part) for part in parts):
        raise ValueError(f"Unsafe relative path: {value!r}")
    return PurePosixPath(value)


def load_manifest(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != SCHEMA or data.get("content_root") != CONTENT_ROOT:
        raise ValueError("Unsupported external asset manifest schema or content root")
    if not data.get("package_id") or not data.get("package_version"):
        raise ValueError("Manifest must identify a package and version")
    files = data.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("Manifest contains no files")
    seen = set()
    for item in files:
        path_key = str(relative_path(item["path"])).casefold()
        if path_key in seen:
            raise ValueError(f"Duplicate manifest path: {item['path']}")
        seen.add(path_key)
        if type(item.get("size_bytes")) is not int or item["size_bytes"] <= 0:
            raise ValueError(f"Invalid size for {item['path']}")
        if not re.fullmatch(r"[0-9a-f]{64}", str(item.get("sha256", ""))):
            raise ValueError(f"Invalid SHA256 for {item['path']}")
    if data.get("file_count") != len(files) or data.get("total_size_bytes") != sum(
            item["size_bytes"] for item in files):
        raise ValueError("Manifest totals do not match the file list")
    return data


def manifest_bytes(manifest: dict) -> bytes:
    return (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def asset_path(content_root: Path, relative: str) -> Path:
    """Reject symlinks/junctions at or within the explicitly selected root."""
    parts = relative_path(relative).parts
    root = content_root.absolute()
    current = root
    for part in (None, *parts):
        if part is not None:
            current = current / part
        try:
            info = current.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode) or getattr(info, "st_file_attributes", 0) & 0x400:
            raise ValueError(f"Linked/reparse asset paths are not allowed: {current}")
    if not current.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"Asset path escapes content root: {relative}")
    return current


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(CHUNK_SIZE), b""):
            digest.update(block)
    return digest.hexdigest()


def check_file(path: Path, item: dict, full: bool) -> None:
    if not path.is_file():
        raise ValueError(f"Missing external asset: {path}")
    with path.open("rb") as source:
        if source.read(len(LFS_HEADER)).startswith(LFS_HEADER):
            raise ValueError(f"Asset is a Git LFS pointer, not its payload: {path}")
    if path.stat().st_size != item["size_bytes"]:
        raise ValueError(f"Size mismatch: {path}")
    if full and file_sha256(path) != item["sha256"]:
        raise ValueError(f"SHA256 mismatch: {path}")


def verify(manifest: dict, content_root: Path, full: bool = False) -> dict:
    errors = []
    for item in manifest["files"]:
        try:
            check_file(asset_path(content_root, item["path"]), item, full)
        except (OSError, ValueError) as exc:
            errors.append(str(exc))
    if errors:
        detail = "\n".join(errors[:8])
        if len(errors) > 8:
            detail += f"\n... and {len(errors) - 8} more errors"
        raise ValueError(f"External asset verification failed ({len(errors)} files):\n{detail}")
    return {"package": manifest["package_id"], "version": manifest["package_version"],
            "files": manifest["file_count"], "bytes": manifest["total_size_bytes"],
            "verification": "sha256" if full else "presence-and-size"}


def publish_new(source: Path, destination: Path) -> None:
    """Reserve a fresh name, then atomically replace only our own empty file."""
    with destination.open("xb"):
        pass
    try:
        os.replace(source, destination)
    except BaseException:
        destination.unlink()
        raise


def stream_asset(source, destination, item: dict) -> None:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: source.read(CHUNK_SIZE), b""):
        size += len(block)
        if size > item["size_bytes"]:
            raise ValueError(f"Size mismatch: {item['path']}")
        digest.update(block)
        if destination is not None:
            destination.write(block)
    if size != item["size_bytes"] or digest.hexdigest() != item["sha256"]:
        raise ValueError(f"SHA256/size mismatch: {item['path']}")


def pack(manifest: dict, content_root: Path, output: Path, repository_root: Path) -> dict:
    output = output.absolute()
    if output.suffix.lower() != ".zip":
        raise ValueError("The output must have a .zip extension")
    if output.resolve().is_relative_to(repository_root.resolve()) or output.resolve().is_relative_to(
            content_root.resolve()):
        raise ValueError("Write asset packages outside the repository and source content directory")
    checksum = output.with_name(output.name + ".sha256")
    if output.exists() or checksum.exists():
        raise ValueError(f"Refusing to overwrite an existing package/checksum: {output}")
    if not output.parent.is_dir():
        raise ValueError(f"Output directory does not exist: {output.parent}")
    result = verify(manifest, content_root)
    # Stored ZIP avoids expensive recompression of UE payloads; force_zip64
    # supports individual textures over 4 GiB and packages over 4 GiB.
    with tempfile.TemporaryDirectory(prefix=".external-assets-pack-", dir=output.parent) as staging:
        temporary = Path(staging) / output.name
        with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as archive:
            archive.writestr(MANIFEST_PATH.as_posix(), manifest_bytes(manifest))
            for item in manifest["files"]:
                path = asset_path(content_root, item["path"])
                archive_path = f"{manifest['content_root']}/{item['path']}"
                with path.open("rb") as source, archive.open(archive_path, "w", force_zip64=True) as target:
                    stream_asset(source, target, item)
        digest = file_sha256(temporary)
        temporary_checksum = Path(staging) / checksum.name
        temporary_checksum.write_text(f"{digest}  {output.name}\n", encoding="utf-8")
        publish_new(temporary_checksum, checksum)
        try:
            publish_new(temporary, output)
        except BaseException:
            checksum.unlink()
            raise
    return {**result, "verification": "sha256", "archive": str(output), "archive_sha256": digest}


def inspect_archive(archive: zipfile.ZipFile, manifest: dict) -> dict:
    expected = {f"{manifest['content_root']}/{item['path']}": item for item in manifest["files"]}
    expected_names = set(expected) | {MANIFEST_PATH.as_posix()}
    names = set()
    for info in archive.infolist():
        relative_path(info.filename)
        if info.filename in names:
            raise ValueError(f"Duplicate archive entry: {info.filename}")
        names.add(info.filename)
        mode = info.external_attr >> 16
        if stat.S_IFMT(mode) not in (0, stat.S_IFREG) or info.is_dir():
            raise ValueError(f"Archive entry is not a regular file: {info.filename}")
        if info.filename not in expected_names:
            raise ValueError(f"Unexpected archive path: {info.filename}")
        if info.filename in expected and info.file_size != expected[info.filename]["size_bytes"]:
            raise ValueError(f"Archive size mismatch: {info.filename}")
        if info.filename == MANIFEST_PATH.as_posix() and info.file_size > 1024 * 1024:
            raise ValueError("Archive manifest is too large")
    if names != expected_names:
        raise ValueError("Archive does not contain every manifest asset")
    if json.loads(archive.read(MANIFEST_PATH.as_posix())) != manifest:
        raise ValueError("Archive manifest does not match the checked-in package version")
    return expected


def restore(manifest: dict, archive_path: Path, content_root: Path) -> dict:
    """Validate all payloads before installation; never replace different files."""
    with zipfile.ZipFile(archive_path, "r") as archive:
        expected = inspect_archive(archive, manifest)
        missing = set()
        # Preflight every existing destination before creating any asset files.
        for item in manifest["files"]:
            destination = asset_path(content_root, item["path"])
            if destination.exists():
                check_file(destination, item, full=True)
            else:
                missing.add(item["path"])
        content_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".external-assets-restore-", dir=content_root) as staging:
            for name, item in expected.items():
                staged = Path(staging).joinpath(*relative_path(item["path"]).parts)
                with archive.open(name) as source:
                    if item["path"] in missing:
                        staged.parent.mkdir(parents=True, exist_ok=True)
                        with staged.open("xb") as target:
                            stream_asset(source, target, item)
                    else:
                        stream_asset(source, None, item)
            for item in manifest["files"]:
                if item["path"] not in missing:
                    continue
                destination = asset_path(content_root, item["path"])
                destination.parent.mkdir(parents=True, exist_ok=True)
                staged = Path(staging).joinpath(*relative_path(item["path"]).parts)
                publish_new(staged, destination)
    return {"package": manifest["package_id"], "version": manifest["package_version"],
            "files": manifest["file_count"], "installed": len(missing),
            "already_present": manifest["file_count"] - len(missing), "verification": "sha256"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("verify", "pack", "restore"):
        sub = commands.add_parser(command)
        sub.add_argument("--repository-root", type=Path, default=REPOSITORY_ROOT,
                         help="Repository to verify/install (default: this tool's repository)")
        sub.add_argument("--manifest", type=Path, default=REPOSITORY_ROOT / MANIFEST_PATH,
                         help="Trusted package manifest, normally checked in with this tool")
        sub.add_argument("--content-root", type=Path,
                         help="Explicit Content/IncredibleEarth directory, including an older checkout")
        if command == "verify":
            sub.add_argument("--full", action="store_true", help="Hash every asset (about 18 GB of reads)")
        elif command == "pack":
            sub.add_argument("--output", required=True, type=Path, help="New ZIP64 file outside the repository")
        else:
            sub.add_argument("--archive", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        content_root = args.content_root or args.repository_root / manifest["content_root"]
        if args.command == "verify":
            result = verify(manifest, content_root, args.full)
        elif args.command == "pack":
            result = pack(manifest, content_root, args.output, args.repository_root)
        else:
            result = restore(manifest, args.archive, content_root)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        print("See docs/guides/external-environment-assets.md for package installation.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
