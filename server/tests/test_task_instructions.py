"""Instruction diversity reaches recorded metadata without changing physical identities."""
from copy import deepcopy
import json

import pytest

from simulation.scripted_tasks.instructions import BANK_PATH, STREAM, TASKS, load_bank
from simulation.scripted_tasks.randomization import generator, prepare_episode
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from collection.record_scripted_task import task_instruction, task_metadata, task_target_site


@pytest.mark.parametrize("task", TASKS)
def test_all_twenty_variants_reach_recording_with_actual_colours(task):
    seeds = {}
    for seed in range(1000):
        seeds.setdefault(int(generator(seed, STREAM).integers(20)), seed)
    assert len(seeds) == 20
    bank = load_bank()["tasks"][task]
    assert len(bank) == len(set(bank)) == 20
    lengths = [len(t.split()) for t in bank]
    assert max(lengths) >= min(lengths) + 10
    source = dict(aviation_plug_insert="aviation_plug", guide_plug_insert="guide_plug",
                  cylindrical_plug_insert="cylindrical_plug", aviation_bolt_insert="bolt").get(task)
    target = task_target_site(task)
    for index, seed in seeds.items():
        scene = prepare_episode(dict(seed=seed, scripted_task_name=task, target_site=target))
        cfg = scene["collection_config"]
        selected = cfg["instruction_randomization"]
        prompt = task_instruction(scene, task, target)
        metadata = task_metadata(scene, task, target, prompt)
        assert selected["template_index"] == index
        assert selected["template_id"] == f"{task}/{index+1:02d}"
        assert selected["template"] == bank[index]
        assert metadata["instruction_randomization"] == selected
        assert cfg["task_instruction"] == scene["scripted_task_instruction"] == prompt
        assert prompt.isascii() and "{" not in prompt and "}" not in prompt
        if source:
            assignment = cfg["episode_color_assignment"]
            object_name = "aviation bolt" if source == "bolt" else source.replace("_", " ")
            assert f"{assignment['source_colors'][source]} {object_name}" in prompt
            noun = "hole" if source == "bolt" else "socket"
            assert f"{assignment['hole_colors'][target]} {noun}" in prompt
            assert cfg["color_randomization"]["prompt_instruction"] == prompt
        elif task == "drawer_cycle":
            assert "8 cm" in prompt and "drawer" in prompt
        else:
            assert "fastener" in prompt and "clip" in prompt and "rod" in prompt
        # Serialized replay preserves the exact chosen wording and ID.
        replay = prepare_episode(json.loads(json.dumps(scene)))
        assert replay["collection_config"]["instruction_randomization"] == selected
        assert task_instruction(replay, task, target) == prompt


def test_new_batch_attempt_resamples_but_keeps_other_random_streams_independent():
    base = dict(randomization=dict(arm_joint_position_rad=[0.] * 8))
    prompts = set()
    for seed in range(30):
        first = randomized_scene(base, seed, task_name="aviation_plug_insert")
        selected = first["collection_config"]["instruction_randomization"]
        prompts.add(selected["template_id"])
        second = randomized_scene(first, seed + 100, task_name="aviation_plug_insert")
        assert second["collection_config"]["instruction_randomization"]["seed"] == seed + 100
        before = deepcopy(first)
        # Re-selecting a sentence never resamples the episode appearance or joints.
        first["collection_config"].pop("instruction_randomization")
        prepare_episode(first)
        assert first == before
    assert len(prompts) > 10


def test_replay_uses_saved_template_even_when_bank_changes(monkeypatch):
    import simulation.scripted_tasks.instructions as instructions
    scene = prepare_episode(dict(seed=42, scripted_task_name="drawer_cycle"))
    selected = deepcopy(scene["collection_config"]["instruction_randomization"])
    monkeypatch.setattr(instructions, "load_bank", lambda: pytest.fail("Replay must not resample the bank"))
    prepare_episode(scene)
    assert scene["collection_config"]["instruction_randomization"] == selected


def test_template_corruption_rejected_and_derived_text_rebuilt():
    scene = prepare_episode(dict(seed=42, scripted_task_name="aviation_plug_insert"))
    selected = scene["collection_config"]["instruction_randomization"]
    expected = selected["instruction"]
    selected["instruction"] = "stale text"
    prepare_episode(scene)
    assert scene["collection_config"]["instruction_randomization"]["instruction"] == expected
    scene["collection_config"]["instruction_randomization"]["template"] += " Changed."
    with pytest.raises(ValueError, match="checksum"):
        prepare_episode(scene)


def test_plug_alias_and_alternate_target_use_same_template_but_correct_hole():
    scene = prepare_episode(dict(seed=3, scripted_task_name="plug_insert", target_site="cylindrical_insert_40p1"))
    cfg = scene["collection_config"]
    assert cfg["instruction_randomization"]["task"] == "cylindrical_plug_insert"
    hole = cfg["episode_color_assignment"]["hole_colors"]["cylindrical_insert_40p1"]
    assert f"{hole} socket" in task_instruction(scene, "plug_insert", "cylindrical_insert_40p1")
