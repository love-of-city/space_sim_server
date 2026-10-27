"""Linux collection uses writable persistent DDC without Zen retries."""
import os
from types import SimpleNamespace

from space_arm_platform.collection_runtime import renderer_command, renderer_environment


def test_collection_ddc_matches_platform_and_reuses_project_cache(tmp_path, monkeypatch):
    monkeypatch.setattr("space_arm_platform.collection_runtime.unreal_editor", lambda root: root / "UE")
    args = SimpleNamespace(adapter=tmp_path, unreal=tmp_path, offline=True,
                           render_port=21002, capture_port=21001)
    graph = "InstalledDerivedDataBackendGraph" if os.name == "nt" else "InstalledNoZenLocalFallback"
    command = renderer_command(args, tmp_path)
    assert f"-DDC={graph}" in command
    env = renderer_environment(args)
    assert env["UE-LocalDataCachePath"] == str(tmp_path / "Unreal/BskUnrealRenderer/Saved/DerivedDataCache")
