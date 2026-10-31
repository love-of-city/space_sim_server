"""Release version metadata comes from executable inputs and ELF requirements."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import runtime_metadata as metadata


def test_python_version_is_read_from_actual_interpreter():
    result = metadata.python_metadata(Path(sys.executable))
    assert result["version"] == ".".join(map(str, sys.version_info[:3]))
    assert len(result["executable_sha256"]) == 64
    assert result["abi"]


def test_required_glibc_uses_needs_not_provided_definitions(tmp_path, monkeypatch):
    elf = tmp_path / "library.so"
    elf.write_bytes(b"\x7fELFfake")
    monkeypatch.setattr(metadata.shutil, "which", lambda name: "/tools/readelf")
    monkeypatch.setattr(metadata.subprocess, "run", lambda *a, **kw: SimpleNamespace(stdout=
        "Version definition section\nName: GLIBC_2.99\nVersion needs section\nName: GLIBC_2.17\nName: GLIBC_2.34\n"))
    result = metadata.required_glibc([tmp_path])
    assert result["minimum"] == "2.34"
    assert result["elf_files_scanned"] == 1


def test_no_elf_cannot_certify_a_glibc_minimum(tmp_path, monkeypatch):
    monkeypatch.setattr(metadata.shutil, "which", lambda name: "/tools/readelf")
    with pytest.raises(ValueError, match="No ELF"):
        metadata.required_glibc([tmp_path])


def test_runtime_versions_are_not_fixed_strings(tmp_path, monkeypatch):
    version = tmp_path / "Engine/Build/Build.version"
    version.parent.mkdir(parents=True)
    version.write_text(json.dumps({"MajorVersion": 5, "MinorVersion": 6, "PatchVersion": 7}))
    for rel in ("Experimental/CelestialVault/CelestialVault.uplugin", "Media/PixelStreaming2/PixelStreaming2.uplugin"):
        plugin = tmp_path / "Engine/Plugins" / rel
        plugin.parent.mkdir(parents=True)
        plugin.write_text('{"VersionName":"custom"}')
    monkeypatch.setattr(metadata, "python_metadata", lambda _: {"version": "3.13.42"})
    result = metadata.collect(Path(sys.executable), tmp_path)
    assert result["python"]["version"] == "3.13.42"
    assert result["engine"]["PatchVersion"] == 7
    assert result["certification"]["clean_install"] == "not-run"
