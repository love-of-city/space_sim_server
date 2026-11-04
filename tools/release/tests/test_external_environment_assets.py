"""Exercise external asset exchange without loading UE or real Earth textures."""
import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import sys
import zipfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import external_environment_assets as assets


@pytest.fixture
def package(tmp_path):
    repository = tmp_path / "repository"
    content = repository / assets.CONTENT_ROOT
    payloads = {"Textures/T_Earth.uasset": b"earth texture\x00\xff", "Meshes/SM_Earth.uasset": b"earth mesh"}
    files = []
    for name, payload in payloads.items():
        path = content / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        files.append({"path": name, "size_bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()})
    manifest = {"schema": assets.SCHEMA, "package_id": "earth-fixture", "package_version": "test-1",
                "content_root": assets.CONTENT_ROOT, "files": files, "file_count": len(files),
                "total_size_bytes": sum(item["size_bytes"] for item in files)}
    path = repository / assets.MANIFEST_PATH
    path.parent.mkdir(parents=True)
    path.write_bytes(assets.manifest_bytes(manifest))
    return repository, content, assets.load_manifest(path), payloads


def test_round_trip_is_zip64_and_preserves_payloads(package, tmp_path):
    repository, content, manifest, payloads = package
    (content / "not-in-manifest.txt").write_text("do not distribute")
    output = tmp_path / "earth.zip"
    result = assets.pack(manifest, content, output, repository)
    assert result["archive_sha256"] == hashlib.sha256(output.read_bytes()).hexdigest()
    assert output.with_suffix(".zip.sha256").read_text().startswith(result["archive_sha256"])
    with zipfile.ZipFile(output) as archive:
        assert set(archive.namelist()) == {assets.MANIFEST_PATH.as_posix(), *(f"{assets.CONTENT_ROOT}/{p}" for p in payloads)}
        assert all(archive.getinfo(f"{assets.CONTENT_ROOT}/{p}").extract_version >= 45 for p in payloads)
    restored = tmp_path / "receiver" / assets.CONTENT_ROOT
    assert assets.restore(manifest, output, restored)["installed"] == 2
    for name, payload in payloads.items():
        assert (restored / name).read_bytes() == payload
    assert assets.verify(manifest, restored, full=True)["verification"] == "sha256"
    assert assets.restore(manifest, output, restored)["installed"] == 0


def test_full_verify_detects_same_size_edit(package):
    _, content, manifest, _ = package
    item = manifest["files"][0]
    (content / item["path"]).write_bytes(b"x" * item["size_bytes"])
    assert assets.verify(manifest, content)["verification"] == "presence-and-size"
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        assets.verify(manifest, content, full=True)


def test_missing_and_lfs_pointer_are_reported(package):
    _, content, manifest, _ = package
    (content / manifest["files"][0]["path"]).write_bytes(assets.LFS_HEADER + b"\noid sha256:fake\n")
    (content / manifest["files"][1]["path"]).unlink()
    with pytest.raises(ValueError, match="Git LFS pointer") as error:
        assets.verify(manifest, content)
    assert "Missing external asset" in str(error.value)


@pytest.mark.parametrize("name", ["../escape", "/absolute", "C:/escape", "nested\\escape", "nested/./file", "nested//file", "name.", "NUL.txt"])
def test_manifest_rejects_nonportable_or_escaping_paths(package, name):
    repository, _, manifest, _ = package
    manifest["files"][0]["path"] = name
    path = repository / assets.MANIFEST_PATH
    path.write_bytes(assets.manifest_bytes(manifest))
    with pytest.raises(ValueError, match="Unsafe relative path"):
        assets.load_manifest(path)


def test_manifest_rejects_case_aliases(package):
    repository, _, manifest, _ = package
    manifest["files"][1]["path"] = manifest["files"][0]["path"].upper()
    path = repository / assets.MANIFEST_PATH
    path.write_bytes(assets.manifest_bytes(manifest))
    with pytest.raises(ValueError, match="Duplicate manifest path"):
        assets.load_manifest(path)


def test_failed_pack_publishes_nothing(package, tmp_path):
    repository, content, manifest, _ = package
    item = manifest["files"][1]
    (content / item["path"]).write_bytes(b"x" * item["size_bytes"])
    output = tmp_path / "corrupt.zip"
    with pytest.raises(ValueError, match="SHA256/size mismatch"):
        assets.pack(manifest, content, output, repository)
    assert not output.exists()
    assert not output.with_suffix(".zip.sha256").exists()
    assert not list(tmp_path.glob(".external-assets-pack-*"))


def test_pack_never_overwrites_and_keeps_packages_outside_repository(package, tmp_path):
    repository, content, manifest, _ = package
    output = tmp_path / "existing.zip"
    output.write_bytes(b"previous release")
    with pytest.raises(ValueError, match="Refusing to overwrite"):
        assets.pack(manifest, content, output, repository)
    assert output.read_bytes() == b"previous release"
    with pytest.raises(ValueError, match="outside the repository"):
        assets.pack(manifest, content, repository / "earth.zip", repository)


def test_restore_checks_all_existing_files_before_installing(package, tmp_path):
    repository, content, manifest, _ = package
    output = tmp_path / "earth.zip"
    assets.pack(manifest, content, output, repository)
    restored = tmp_path / "receiver"
    conflict = restored / manifest["files"][1]["path"]
    conflict.parent.mkdir(parents=True)
    conflict.write_bytes(b"user modifications")
    with pytest.raises(ValueError, match="mismatch"):
        assets.restore(manifest, output, restored)
    assert conflict.read_bytes() == b"user modifications"
    assert not (restored / manifest["files"][0]["path"]).exists()


@pytest.mark.parametrize("attack", ["traversal", "symlink", "corrupt", "duplicate", "version"])
def test_restore_rejects_invalid_archive_before_installing(package, tmp_path, attack):
    _, content, manifest, _ = package
    output = tmp_path / "bad.zip"
    with zipfile.ZipFile(output, "w") as archive:
        archive_manifest = dict(manifest)
        if attack == "version":
            archive_manifest["package_version"] = "unexpected"
        archive.writestr(assets.MANIFEST_PATH.as_posix(), assets.manifest_bytes(archive_manifest))
        for index, item in enumerate(manifest["files"]):
            name = f"{assets.CONTENT_ROOT}/{item['path']}"
            payload = (content / item["path"]).read_bytes()
            if attack == "corrupt" and index == 1:
                payload = b"x" * len(payload)
            archive.writestr(name, payload)
        if attack == "traversal":
            archive.writestr("../outside.txt", b"escape")
        elif attack == "symlink":
            link = zipfile.ZipInfo("link")
            link.create_system = 3
            link.external_attr = (stat.S_IFLNK | 0o777) << 16
            archive.writestr(link, "../outside.txt")
        elif attack == "duplicate":
            with pytest.warns(UserWarning, match="Duplicate name"):
                archive.writestr(assets.MANIFEST_PATH.as_posix(), assets.manifest_bytes(manifest))
    restored = tmp_path / "receiver"
    with pytest.raises(ValueError):
        assets.restore(manifest, output, restored)
    assert not list(restored.rglob("*.uasset"))
    assert not (tmp_path / "outside.txt").exists()


def test_restore_rejects_destination_symlink(package, tmp_path):
    repository, content, manifest, _ = package
    output = tmp_path / "earth.zip"
    assets.pack(manifest, content, output, repository)
    restored = tmp_path / "receiver"
    restored.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    try:
        (restored / "Textures").symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("Creating symlinks is unavailable for this Windows account")
    with pytest.raises(ValueError, match="Linked/reparse"):
        assets.restore(manifest, output, restored)
    assert not list(outside.iterdir())


def test_cli_can_verify_original_checkout_content_root(package):
    repository, content, _, _ = package
    assert assets.main(["verify", "--manifest", str(repository / assets.MANIFEST_PATH),
                        "--content-root", str(content), "--full"]) == 0


@pytest.fixture
def deployed_package(package, tmp_path):
    _, _, manifest, payloads = package
    repository = Path(__file__).resolve().parents[3]
    script = repository / "ue-adapter/Unreal/BskUnrealRenderer/scripts/check_external_environment_assets.py"
    spec = importlib.util.spec_from_file_location("external_asset_startup_check", script)
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    project = tmp_path / "deployed" / "BskUnrealRenderer"
    manifest_path = project / "Config/ExternalAssets/incredible-earth.json"
    manifest_path.parent.mkdir(parents=True)
    manifest_path.write_bytes(assets.manifest_bytes(manifest))
    for name, payload in payloads.items():
        path = project / "Content/IncredibleEarth" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    return checker, project, manifest


def test_deployment_manifest_matches_canonical_manifest():
    repository = Path(__file__).resolve().parents[3]
    deployed = repository / "ue-adapter/Unreal/BskUnrealRenderer/Config/ExternalAssets/incredible-earth.json"
    assert json.loads(deployed.read_text(encoding="utf-8")) == assets.load_manifest(repository / assets.MANIFEST_PATH)


def test_startup_checker_accepts_complete_deployed_package(deployed_package):
    checker, project, _ = deployed_package
    checker.check_external_assets(project)


def test_startup_checker_reports_missing_package_file(deployed_package):
    checker, project, manifest = deployed_package
    (project / "Content/IncredibleEarth" / manifest["files"][0]["path"]).unlink()
    with pytest.raises(RuntimeError, match="missing") as error:
        checker.check_external_assets(project)
    assert "git lfs pull does not download this package" in str(error.value)


@pytest.mark.parametrize("same_size", [False, True])
def test_startup_checker_rejects_lfs_pointer(deployed_package, same_size):
    checker, project, manifest = deployed_package
    pointer = assets.LFS_HEADER + b"\noid sha256:" + b"0" * 64 + b"\nsize 999999\n"
    (project / "Content/IncredibleEarth" / manifest["files"][0]["path"]).write_bytes(pointer)
    if same_size:
        manifest["files"][0]["size_bytes"] = len(pointer)
        manifest["total_size_bytes"] = sum(item["size_bytes"] for item in manifest["files"])
        (project / "Config/ExternalAssets/incredible-earth.json").write_bytes(assets.manifest_bytes(manifest))
    with pytest.raises(RuntimeError, match="LFS pointer"):
        checker.check_external_assets(project)


def test_startup_checker_rejects_empty_manifest(deployed_package):
    checker, project, manifest = deployed_package
    manifest["files"] = []
    manifest["file_count"] = 0
    manifest["total_size_bytes"] = 0
    (project / "Config/ExternalAssets/incredible-earth.json").write_bytes(assets.manifest_bytes(manifest))
    with pytest.raises(RuntimeError):
        checker.check_external_assets(project)
