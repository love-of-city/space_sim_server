"""Single-episode capture must never label a default model as a saved layout."""
import asyncio
import json
from types import SimpleNamespace

import pytest

from collection import record_scripted_task as recorder


def scene_file(tmp_path, saved_layout):
    path = tmp_path / "scene.json"
    path.write_text(json.dumps({
        "template_id": "sarm-task-box-20261012",
        "instance_id": "saved-scene",
        **saved_layout,
    }), encoding="utf-8")
    return path


@pytest.mark.parametrize("saved_layout", [
    {"collection_layout_randomization": True},
    {"task_box_layout": {"schema": "task-box-layout/3", "model_sha256": "saved-model"}},
    {"collection_config": {"layout_randomization": {"enabled": True, "model_sha256": "saved-model"}}},
])
def test_saved_layout_requires_model_before_any_runtime_or_output(tmp_path, monkeypatch, saved_layout):
    output = tmp_path / "recording"
    args = SimpleNamespace(
        warmup_sim_seconds=0, recording_config=None,
        scene=scene_file(tmp_path, saved_layout), task="drawer_cycle", model=None,
        output=output,
    )
    monkeypatch.setattr(recorder, "recording_settings", lambda _: SimpleNamespace())

    def unexpected_runtime(*_):
        pytest.fail("Saved layout must be rejected before resolving or starting UE")

    monkeypatch.setattr(recorder, "unreal_editor", unexpected_runtime)
    with pytest.raises(ValueError, match="--model is required"):
        asyncio.run(recorder.run(args))
    assert not output.exists()


def test_explicit_model_keeps_capture_layout_validation(tmp_path, monkeypatch):
    adapter = tmp_path / "adapter"
    project = adapter / "Unreal/BskUnrealRenderer"
    project.mkdir(parents=True)
    (project / "BskUnrealRenderer.uproject").touch()
    editor, model, catalog = (tmp_path / name for name in ("editor", "model.xml", "catalog.json"))
    for path in (editor, model, catalog):
        path.touch()
    args = SimpleNamespace(
        warmup_sim_seconds=0, recording_config=None,
        scene=scene_file(tmp_path, {"task_box_layout": {"model_sha256": "saved-model"}}),
        task="drawer_cycle", model=model, catalog=catalog,
        adapter=adapter, unreal=tmp_path / "engine", output=tmp_path / "recording",
        control_port=21000, capture_port=21001, render_port=21002, signalling_port=21003,
    )
    monkeypatch.setattr(recorder, "recording_settings", lambda _: SimpleNamespace())
    monkeypatch.setattr(recorder, "unreal_editor", lambda _: editor)
    monkeypatch.setattr(recorder, "check_port_available", lambda _: None)

    class LayoutValidated(Exception):
        pass

    def validate(selected_model, selected_catalog):
        assert selected_model == model.resolve()
        assert selected_catalog == catalog.resolve()
        raise LayoutValidated

    monkeypatch.setattr("simulation.task_box_layout.validated_capture_layout", validate)
    with pytest.raises(LayoutValidated):
        asyncio.run(recorder.run(args))


@pytest.mark.parametrize("applied", [
    {},
    {"applied_schema": "socket-colors/1", "model_digest": "different-assignment"},
    {"applied_schema": "socket-colors/1", "model_digest": "assignment", "appearance_digest": "old-assignment"},
])
def test_colour_instruction_rejects_unrealized_or_stale_appearance(applied):
    scene = {"collection_config": {
        "color_randomization": {
            "prompt_instruction": "Insert the red plug into the blue hole.",
            "assignment_sha256": "assignment",
            "appearance_sha256": applied.get("appearance_digest", "assignment"),
            "applied_schema": applied.get("applied_schema"),
        },
        "model_visual_randomization": {"assignment_sha256": applied.get("model_digest")},
    }}
    with pytest.raises(ValueError, match="applied model appearance"):
        recorder.validate_color_instruction(scene)


def test_colour_instruction_accepts_applied_appearance_and_plain_tasks():
    recorder.validate_color_instruction({})
    scene = {"collection_config": {
        "color_randomization": {
            "prompt_instruction": "Insert the red plug into the blue hole.",
            "assignment_sha256": "assignment",
            "appearance_sha256": "assignment",
            "applied_schema": "socket-colors/1",
        },
        "model_visual_randomization": {"assignment_sha256": "assignment"},
    }}
    recorder.validate_color_instruction(scene)
    instruction = recorder.task_instruction(scene, "aviation_plug_insert", "aviation_insert_40p1")
    metadata = recorder.task_metadata(scene, "aviation_plug_insert", "aviation_insert_40p1", instruction)
    assert metadata["instruction"] == "Insert the red plug into the blue hole."
    assert metadata["color_randomization"]["appearance_sha256"] == "assignment"
