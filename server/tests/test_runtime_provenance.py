"""Runtime changes must never be silently mixed into a frozen collection batch."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from collection import runtime_provenance as provenance


@pytest.fixture
def frozen_batch(tmp_path):
    for name in ("config.snapshot.json", "plan.json", "pool.snapshot.json", "source-scene.json", "inputs.json"):
        (tmp_path / name).write_text("{}")
    (tmp_path / "progress.json").write_text('{"jobs": {}}')
    recording = tmp_path / "recording.json"
    recording.write_text("{}")
    return tmp_path, SimpleNamespace(recording_config=str(recording), seed=123, adapter=None, unreal=None)


def test_resume_accepts_same_runtime_and_rejects_source_or_native_changes(frozen_batch):
    output, config = frozen_batch
    identity = {"source": "first", "native": "built", "ue_build_id": "shader1"}
    original = provenance.freeze_or_verify(output, config, identity=identity)
    assert provenance.freeze_or_verify(output, config, identity=identity) == original
    for key in identity:
        with pytest.raises(RuntimeError, match="new batch"):
            provenance.freeze_or_verify(output, config, identity={**identity, key: "changed"})
    assert json.loads((output / "launch-provenance.json").read_text()) == original


def test_frozen_input_change_is_rejected(frozen_batch):
    output, config = frozen_batch
    provenance.freeze_or_verify(output, config, identity={})
    (output / "plan.json").write_text('{"changed":true}')
    with pytest.raises(RuntimeError, match="frozen inputs changed"):
        provenance.freeze_or_verify(output, config, identity={})


def test_legacy_batch_is_not_retroactively_certified(frozen_batch):
    output, config = frozen_batch
    (output / "launch-provenance.json").write_text('{"commit":"legacy"}')
    with pytest.raises(RuntimeError, match="Legacy batch"):
        provenance.freeze_or_verify(output, config, identity={})
    (output / "launch-provenance.json").unlink()
    (output / "progress.json").write_text('{"jobs":{"existing":{"status":"pending_audit"}}}')
    with pytest.raises(RuntimeError, match="retroactively"):
        provenance.freeze_or_verify(output, config, identity={})


def test_source_scope_covers_untracked_code_but_excludes_docs(tmp_path):
    server, adapter = tmp_path / "server", tmp_path / "ue-adapter"
    (server / "simulation").mkdir(parents=True)
    (adapter / "Adapters").mkdir(parents=True)
    code = server / "simulation/worker.py"
    code.write_text("VERSION = 1\n")
    original = provenance.source_identity(server, adapter)
    (server / "README.md").write_text("Documentation only")
    assert provenance.source_identity(server, adapter) == original
    (server / "simulation/untracked_runtime.py").write_text("VALUE = 1\n")
    assert provenance.source_identity(server, adapter) != original


@pytest.mark.parametrize("relative", ["server/model/SARM/platform/scenarios/scenario_sarm_grasp.py",
    "ue-adapter/Unreal/BskUnrealRenderer/examples/scenario_spacecraft_arm_grasp_unreal.py",
    "server/simulation/local_mujoco_bindings.py",
    "ue-adapter/Adapters/bsk_render_adapter/scene_description.py",
    "ue-adapter/Adapters/bsk_render_adapter/device_registration.py",
    "ue-adapter/Adapters/bsk_render_adapter/command_dispatch.py"])
def test_runtime_entrypoints_and_extracted_modules_participate_in_identity(tmp_path, relative):
    server, adapter = tmp_path / "server", tmp_path / "ue-adapter"
    code = tmp_path / relative
    code.parent.mkdir(parents=True)
    code.write_text("VALUE = 1\n")
    original = provenance.source_identity(server, adapter)
    code.write_text("VALUE = 2\n")
    assert provenance.source_identity(server, adapter) != original


def test_native_library_hash_and_path_are_checked(tmp_path):
    directory = tmp_path / "run/native_acceleration"
    directory.mkdir(parents=True)
    library = directory / "helper.so"
    library.write_bytes(b"native")
    manifest = directory / "helper.json"
    document = {"library": "run/native_acceleration/helper.so", "library_sha256": provenance.digest(library)}
    manifest.write_text(json.dumps(document))
    assert provenance.native_identity(tmp_path)["helper"]["library_sha256"] == provenance.digest(library)
    library.write_bytes(b"replaced")
    with pytest.raises(RuntimeError, match="fingerprint"):
        provenance.native_identity(tmp_path)
    document["library"] = "../outside.so"
    manifest.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="escapes"):
        provenance.native_identity(tmp_path)
