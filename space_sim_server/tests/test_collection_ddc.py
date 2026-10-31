"""Linux collection uses writable persistent DDC without Zen retries."""
import os
from pathlib import Path
from types import SimpleNamespace
import pytest

from space_arm_platform.collection_runtime import renderer_command, renderer_environment


def test_collection_ddc_matches_platform_and_reuses_project_cache(tmp_path, monkeypatch):
    monkeypatch.delenv("UE-LocalDataCachePath", raising=False)
    monkeypatch.setattr("space_arm_platform.collection_runtime.unreal_editor", lambda root: root / "UE")
    args = SimpleNamespace(adapter=tmp_path, unreal=tmp_path, offline=True,
                           render_port=21002, capture_port=21001)
    graph = "InstalledDerivedDataBackendGraph" if os.name == "nt" else "InstalledNoZenLocalFallback"
    command = renderer_command(args, tmp_path)
    assert f"-DDC={graph}" in command
    env = renderer_environment(args)
    cache = Path(env["UE-LocalDataCachePath"])
    if os.name == "nt":
        assert cache.parent.name == "BskDDC" and len(str(cache)) <= 119
        other = renderer_environment(SimpleNamespace(adapter=tmp_path / "another"))
        assert env["UE-LocalDataCachePath"] != other["UE-LocalDataCachePath"]
    else:
        assert cache == tmp_path / "Unreal/BskUnrealRenderer/Saved/DerivedDataCache"
    assert "UE-LocalDataCachePath" not in os.environ


def test_collection_preserves_explicit_cache_and_rejects_windows_overflow(monkeypatch, tmp_path):
    cache = str(Path(os.environ.get("LOCALAPPDATA", "/tmp")) / "BskDDC/shared")
    monkeypatch.setenv("UE-LocalDataCachePath", cache)
    assert renderer_environment(SimpleNamespace(adapter=tmp_path))["UE-LocalDataCachePath"] == cache
    if os.name == "nt":
        monkeypatch.setenv("UE-LocalDataCachePath", cache + "/long"*40)
        with pytest.raises(ValueError, match="119 characters"):
            renderer_environment(SimpleNamespace(adapter=tmp_path))
