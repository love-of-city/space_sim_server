"""Fail-closed optional build selection; tests do not create a physics scene."""
import json
from pathlib import Path

import pytest

from simulation import native_acceleration as loader
from simulation.mjscene_threadpool import MJSceneThreadPoolProbe


def test_missing_native_build_has_actionable_error(tmp_path,monkeypatch):
    monkeypatch.setattr(loader,"ROOT",tmp_path)
    loader.load_native_library.cache_clear()
    try:
        with pytest.raises(FileNotFoundError,match="build_native_acceleration"):
            loader.load_native_library("posture")
    finally:
        loader.load_native_library.cache_clear()


def test_source_change_rejects_stale_dll_before_loading(tmp_path,monkeypatch):
    (tmp_path/"native").mkdir()
    (tmp_path/"native/posture.cpp").write_text("// new source",encoding="utf-8")
    out=tmp_path/"run/native_acceleration";out.mkdir(parents=True)
    (out/"posture.json").write_text(json.dumps({"abi":1,"source_sha256":"old"}),encoding="utf-8")
    monkeypatch.setattr(loader,"ROOT",tmp_path)
    loader.load_native_library.cache_clear()
    try:
        with pytest.raises(RuntimeError,match="stale"):
            loader.load_native_library("posture")
    finally:
        loader.load_native_library.cache_clear()


def test_unknown_component_never_loads_arbitrary_library():
    with pytest.raises(ValueError,match="unknown"):
        loader.load_native_library("../outside")


@pytest.mark.parametrize("workers",[-1,9,True,1.5,"2"])
def test_thread_count_validation_precedes_any_native_pointer_access(workers):
    with pytest.raises(ValueError,match="workers"):
        MJSceneThreadPoolProbe(None,workers)


def test_probe_refuses_publish_only_scene_of_local_dynamics_backend():
    from types import SimpleNamespace
    with pytest.raises(RuntimeError,match="local dynamics backend"):
        MJSceneThreadPoolProbe(SimpleNamespace(isDynamicsSynced=True),0)


def test_closed_probe_cannot_dereference_released_scene():
    probe=object.__new__(MJSceneThreadPoolProbe)
    probe._closed=True
    with pytest.raises(RuntimeError,match="after close"):
        probe.snapshot()
    probe.close()  # Idempotent; no pointers or libraries need to exist.


def test_thread_pool_not_enabled_by_live_teleop_entrypoint():
    root=Path(__file__).resolve().parents[1]
    for name in ("simulation/teleop_grasp_unreal.py","scripts/run_simulation.ps1"):
        source=(root/name).read_text(encoding="utf-8")
        assert "MJSceneThreadPoolProbe" not in source
    source=(root/"tools/profile_simulation_runtime.py").read_text(encoding="utf-8")
    assert 'probe.close()' in source
