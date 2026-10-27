"""Noise must affect the intended layer and stay reproducible and auditable."""

from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from pydantic import ValidationError
from space_arm_platform.collection_randomization import (
    CollectionRandomization,
    randomization_catalog,
)
from space_arm_platform.collection_jobs import CollectionCreate
from space_arm_platform.models import WrenchObservation
from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
from simulation.scripted_tasks.disturbances import EpisodeDisturbances
from simulation.scripted_tasks.drawer_initialization import randomized_scene


def scene(**options):
    return prepare_episode(
        dict(
            seed=42,
            collection_randomization=options,
            randomization={"arm_joint_position_rad": [0.0] * 8},
        )
    )


def wrench(stamp=20_000_000):
    return dict(
        sensor_name="wrist_ft",
        site_name="wrist_ft_site",
        parent_body="ee_base",
        child_body="tool",
        frame="sensor_site",
        sign_convention="mujoco_force_torque",
        measurement="raw_interface_wrench",
        sample_phase="last_substep_start",
        valid=True,
        sample_time_ns=str(stamp - 2_500_000),
        interval_end_time_ns=str(stamp),
        force_n=[1.0, 2.0, 3.0],
        torque_nm=[0.1, 0.2, 0.3],
    )


@pytest.mark.parametrize(
    "options",
    [
        {"unknown": {}},
        {"action_noise": {"std_deg": -1}},
        {"joint_bias": {"max_abs_deg": float("nan")}},
        {"observation_delay": {"delay_ms": 7}},
        {"lighting": {"variation_pct": 100}},
        {"external_disturbance": {"body": "world"}},
        {"joint_observation_noise": {"enabled": "false"}},
    ],
)
def test_bad_options_are_rejected_before_start(options):
    with pytest.raises(ValidationError):
        CollectionCreate(randomization=options)


def test_defaults_are_neutral_and_do_not_mutate_inputs():
    state = scene()
    d = EpisodeDisturbances(state)
    q = np.arange(8, dtype=float) / 10
    v = q / 2
    mq, mv, stamp = d.observe(q, v, 123)
    aq, av = d.action(q, v, 0, np.full(6, -3), np.full(6, 3))
    np.testing.assert_array_equal(mq, q)
    np.testing.assert_array_equal(mv, v)
    np.testing.assert_array_equal(aq, q)
    np.testing.assert_array_equal(av, v)
    assert stamp == 123 and d.wrench(wrench()) == wrench()
    assert not any(CollectionRandomization().flags().values())
    np.testing.assert_array_equal(d.external_wrench(1), np.zeros(6))


def test_option_streams_are_independent_and_seeded():
    source = scene(joint_bias={"enabled": True})
    other = scene(
        joint_bias={"enabled": True},
        lighting={"enabled": True},
        dynamics={"enabled": True},
    )
    assert (
        source["collection_config"]["realized_randomization"]["joint_bias_deg"]
        == other["collection_config"]["realized_randomization"]["joint_bias_deg"]
    )
    before = deepcopy(other)
    prepare_episode(other)
    assert other == before
    original = deepcopy(source)
    a = randomized_scene(source, 100)
    b = randomized_scene(source, 101)
    assert source == original
    assert a["randomization"] != b["randomization"]
    assert (
        a["collection_config"]["realized_randomization"]
        != b["collection_config"]["realized_randomization"]
    )
    assert randomized_scene(source, 100) == a


def test_delay_uses_historical_samples_and_never_delays_control():
    d = EpisodeDisturbances(scene(observation_delay={"enabled": True, "delay_ms": 20}))
    for i in range(10):
        d.ingest(np.full(8, i, dtype=float), np.zeros(8), i * 5_000_000)
    q, v, stamp = d.observe(np.full(8, 9.0), np.zeros(8), 45_000_000)
    assert stamp == 25_000_000 and q.tolist() == [5.0] * 8
    aq, av = d.action(np.ones(8), np.zeros(8), 0.045, [-3] * 6, [3] * 6)
    np.testing.assert_array_equal(aq, np.ones(8))


def test_sensor_noise_never_changes_physics_and_wrench_is_shared_across_rates():
    d = EpisodeDisturbances(
        scene(
            joint_observation_noise={"enabled": True},
            wrist_wrench_noise={"enabled": True},
        )
    )
    q = np.zeros(8)
    v = np.zeros(8)
    mq, mv, _ = d.observe(q, v, 20_000_000)
    assert np.any(mq[:6]) and np.any(mv[:6])
    assert np.all(q == 0) and np.all(v == 0)
    assert np.all(mq[6:] == 0)
    raw = wrench()
    first = d.wrench(raw)
    second = d.wrench(raw)
    assert first == second and first["force_n"] != raw["force_n"]
    assert first["ground_truth_force_n"] == raw["force_n"] and raw == wrench()
    WrenchObservation.model_validate(first)
    invalid = {
        **raw,
        "valid": False,
        "sample_time_ns": "0",
        "interval_end_time_ns": "0",
        "force_n": [0.0] * 3,
        "torque_nm": [0.0] * 3,
    }
    assert d.wrench(invalid) == invalid


def test_actions_change_actual_reference_and_respect_joint_limits():
    config = scene(action_noise={"enabled": True})
    a, b = EpisodeDisturbances(config), EpisodeDisturbances(config)
    for tick in range(100):
        q, v = a.action(
            np.zeros(8), np.zeros(8), tick * 0.005, [-0.001] * 6, [0.001] * 6
        )
        q2, v2 = b.action(
            np.zeros(8), np.zeros(8), tick * 0.005, [-0.001] * 6, [0.001] * 6
        )
        np.testing.assert_array_equal(q, q2)
        assert np.max(np.abs(q[:6])) <= 0.001
        assert np.all(q[6:] == 0) and np.all(v[6:] == 0)
    assert np.any(q[:6])


def test_external_force_is_smooth_bounded_and_can_be_zeroed():
    d = EpisodeDisturbances(scene(external_disturbance={"enabled": True}))
    assert np.linalg.norm(d.external_wrench(1)) > 0
    np.testing.assert_allclose(
        d.external_wrench(1 - 1e-6), d.external_wrench(1 + 1e-6), atol=1e-10
    )
    np.testing.assert_array_equal(d.external_wrench(1, False), np.zeros(6))
    assert np.max(np.abs(d.external_wrench(1)[:3])) <= 0.3


def test_model_variant_preserves_geometry_scales_mass_inertia_and_pair_friction(
    tmp_path,
):
    source = tmp_path / "original.xml"
    original = '<mujoco><compiler meshdir="assets"/><asset><mesh name="m" file="shape.stl"/></asset><worldbody><body name="b"><inertial pos="0 0 0" mass="2" diaginertia="1 2 3"/><geom name="g" type="sphere" size=".1"/></body></worldbody><contact><pair geom1="g" geom2="g" friction="1 2 3 4 5"/></contact></mujoco>'
    source.write_text(original)
    state = scene(dynamics={"enabled": True})
    variant = write_model_variant(source, tmp_path / "run/model.xml", state)
    mass = state["collection_config"]["realized_randomization"]["mass_scale"]
    friction = state["collection_config"]["realized_randomization"]["friction_scale"]
    xml = ET.parse(variant).getroot()
    assert float(xml.find(".//inertial").get("mass")) == pytest.approx(2 * mass)
    np.testing.assert_allclose(
        np.fromstring(xml.find(".//inertial").get("diaginertia"), sep=" "),
        np.array([1, 2, 3]) * mass,
    )
    np.testing.assert_allclose(
        np.fromstring(xml.find(".//pair").get("friction"), sep=" "),
        np.arange(1, 6) * friction,
    )
    assert xml.find(".//worldbody//geom").get("size") == ".1"
    assert Path(xml.find(".//mesh").get("file")).is_absolute()
    assert source.read_text() == original
    assert write_model_variant(source, tmp_path / "none.xml", scene()) == source
    assert not (tmp_path / "none.xml").exists()


def test_catalog_and_request_have_same_defaults_and_constraints():
    catalog = randomization_catalog()
    defaults = CollectionRandomization().model_dump()
    assert {item["key"]: item["default"] for item in catalog} == defaults
    assert (
        next(x for x in catalog if x["key"] == "observation_delay")["fields"][0][
            "multipleOf"
        ]
        == 5
    )


def test_initialization_switch_also_applies_to_cli_source_scene():
    source = scene()
    source["initialization_enabled"] = False
    result = randomized_scene(source, 123, 10)
    assert result["randomization"]["arm_joint_position_rad"] == [0.0] * 8
    assert result["collection_config"]["initialization"]["half_range_deg"] == 0


def test_color_prompt_is_concise_english_and_rebuilt_from_assignment():
    value = prepare_episode(
        {
            "seed": 42,
            "scripted_task_name": "aviation_plug_insert",
            "target_site": "aviation_insert_40p5",
            "collection_config": {
                "task": "aviation_plug_insert",
                "target_site": "aviation_insert_40p5",
            },
            "randomization": {"arm_joint_position_rad": [0.0] * 8},
        }
    )
    colour = value["collection_config"]["color_randomization"]
    source_key = colour["source_colors"]["aviation_plug"]
    target_key = colour["hole_colors"]["aviation_insert_40p5"]
    expected = (
        f"Pick up the {colour['palette'][source_key]['color_name']} aviation plug, "
        f"align it with the {colour['palette'][target_key]['color_name']} socket, "
        "and insert it fully."
    )
    assert colour["prompt_instruction"] == expected
    assert all(ord(char) < 128 for char in expected)
    assert "aviation_insert_40p5" not in expected
    assert "40.5" not in expected

    # A stale prompt in a replayed assignment must not bring back the old
    # Chinese/verbose wording after the template change.
    value["collection_config"]["episode_color_assignment"]["prompt_instruction"] = "旧提示"
    prepare_episode(value)
    assert value["collection_config"]["color_randomization"]["prompt_instruction"] == expected


def test_disabling_lighting_restores_original_base_intensity():
    value=scene(lighting={"enabled":True})
    assert value["environment"]["lighting"]["sunlight_intensity_scale"]!=1
    value["collection_config"]["randomization_options"]["lighting"]["enabled"]=False
    prepare_episode(value)
    assert value["environment"]["lighting"]["sunlight_intensity_scale"]==1


def test_noisy_force_stream_retains_truth_without_changing_sensor_identity(tmp_path):
    from space_arm_platform.models import SimulationObservation
    from space_arm_platform.wrench_recording import WrenchStreamWriter
    from test_sarm_observation import observation
    writer=WrenchStreamWriter(tmp_path)
    d=EpisodeDisturbances(scene(wrist_wrench_noise={"enabled":True}))
    for stamp in (20_000_000,40_000_000):
        payload=observation();payload["sim_time_ns"]=str(stamp)
        payload["wrist_wrench_rate_hz"]=200
        rows=[]
        for end in range(stamp-15_000_000,stamp+1,5_000_000):
            raw=wrench(end);raw["force_n"][0]=end/1e9
            rows.append(d.wrench(raw))
        payload["wrist_wrench"]=rows[-1];payload["wrist_wrench_samples"]=rows
        writer.append(SimulationObservation.model_validate(payload))
    records=[json.loads(line) for line in (tmp_path/writer.relative_path).read_text().splitlines()]
    assert len(records)==5
    assert records[-1]["ground_truth_force_n"][0]==.04
    assert records[-1]["force_n"] != records[-1]["ground_truth_force_n"]


@pytest.mark.parametrize("task,source,target,object_name,ending", [
    ("aviation_plug_insert", "aviation_plug", "aviation_insert_40p5", "aviation plug", "insert it fully."),
    ("guide_plug_insert", "guide_plug", "guide_insert_40p5", "guide plug", "insert it fully."),
    ("cylindrical_plug_insert", "cylindrical_plug", "cylindrical_insert_40p5", "cylindrical plug", "insert it fully."),
    ("plug_insert", "cylindrical_plug", "cylindrical_insert_40p5", "cylindrical plug", "insert it fully."),
    ("aviation_plug_insert", "aviation_plug", "aviation_insert_40p1", "aviation plug", "insert it fully."),
    ("aviation_bolt_insert", "bolt", "bolt_assembly_hole", "aviation bolt", "screw it in fully."),
])
def test_each_colour_instruction_uses_its_actual_source_and_target(task, source, target, object_name, ending):
    value = prepare_episode({"seed": 4, "scripted_task_name": task, "target_site": target})
    assignment = value["collection_config"]["episode_color_assignment"]
    colour = value["collection_config"]["color_randomization"]
    source_colour = assignment["source_colors"][source]
    target_colour = assignment["hole_colors"][target]
    if task == "aviation_bolt_insert":
        expected = (
            f"Unscrew the {source_colour} aviation bolt from its holder, "
            f"then screw it fully into the {target_colour} hole."
        )
    else:
        expected = (
            f"Pick up the {source_colour} {object_name}, align it with the {target_colour} socket, and {ending}"
        )
    assert colour["prompt_instruction"] == expected
    assert colour["prompt_instruction"].isascii()
    assert colour["assignment_sha256"] == assignment["assignment_sha256"]


def test_colour_instruction_rejects_another_plug_type_target():
    with pytest.raises(ValueError, match="not this plug's assembly hole"):
        prepare_episode({"seed": 4, "scripted_task_name": "aviation_plug_insert",
                         "target_site": "guide_insert_40p5"})


def test_bolt_colour_instruction_rejects_a_storage_or_plug_target():
    for target in ("bolt_storage_hole", "aviation_insert_40p5"):
        with pytest.raises(ValueError, match="not this bolt's assembly hole"):
            prepare_episode({"seed": 4, "scripted_task_name": "aviation_bolt_insert", "target_site": target})


@pytest.mark.parametrize("task", ["aviation_plug_insert", "guide_plug_insert",
                                  "cylindrical_plug_insert", "plug_insert", "aviation_bolt_insert"])
def test_colour_instruction_rejects_model_without_independent_hole_surfaces(tmp_path, task):
    source = tmp_path / "monolithic.xml"
    original = '<mujoco><asset/><worldbody><body name="asm_board1"><geom name="asm_board1_geom" type="box" size=".2 .2 .01"/></body></worldbody></mujoco>'
    source.write_text(original)
    value = prepare_episode({"seed": 4, "scripted_task_name": task})
    output = tmp_path / "run" / "model.xml"
    with pytest.raises(ValueError, match="require an independent task-box layout"):
        write_model_variant(source, output, value)
    assert not output.exists()
    assert source.read_text() == original
    assert "model_visual_randomization" not in value["collection_config"]


def test_uncoloured_default_drawer_and_diagnostic_models_remain_supported(tmp_path):
    source = tmp_path / "monolithic.xml"
    source.write_text('<mujoco><asset/><worldbody/></mujoco>')
    for task in (None, "drawer_lift", "drawer_cycle"):
        value = prepare_episode({"seed": 4, "scripted_task_name": task})
        colour = value["collection_config"].get("color_randomization", {})
        assert "prompt_instruction" not in colour
        assert write_model_variant(source, tmp_path / "variant.xml", value) == source.resolve()


def test_colour_materialization_records_the_same_assignment_as_the_prompt(tmp_path):
    from test_socket_colors import seven_hole_model, physical_xml

    source = tmp_path / "independent.xml"
    root = seven_hole_model()
    ET.ElementTree(root).write(source)
    original = source.read_bytes()
    value = prepare_episode({"seed": 4, "scripted_task_name": "aviation_plug_insert"})
    expected = deepcopy(value["collection_config"]["episode_color_assignment"])
    output = write_model_variant(source, tmp_path / "run/model.xml", value)
    result = ET.parse(output).getroot()
    assert source.read_bytes() == original
    assert physical_xml(result) == physical_xml(root)
    config = value["collection_config"]
    assert config["episode_color_assignment"] == expected
    assert config["model_visual_randomization"]["assignment_sha256"] == expected["assignment_sha256"]
    assert config["color_randomization"]["appearance_sha256"] == expected["assignment_sha256"]
    assert config["color_randomization"]["applied_schema"] == "socket-color-identity/3"
    for hole, colour in expected["hole_colors"].items():
        assert result.find(f'.//body[@name="layout_{hole}"]/geom[@group="2"]').get("material") == "socket_identity_color_" + colour
    for source_id, colour in expected["source_colors"].items():
        assert result.find(f'.//body[@name="{source_id}"]/geom[@group="2"]').get("material") == "socket_identity_color_" + colour


def test_episode_colour_streams_and_replay_do_not_change_when_other_options_change():
    plain = prepare_episode({"seed": 77, "scripted_task_name": "guide_plug_insert"})
    noisy = prepare_episode({"seed": 77, "scripted_task_name": "guide_plug_insert",
                             "collection_randomization": {"lighting": {"enabled": True},
                                                          "dynamics": {"enabled": True}}})
    expected = deepcopy(plain["collection_config"]["episode_color_assignment"])
    assert noisy["collection_config"]["episode_color_assignment"] == expected
    plain["seed"] = 999
    prepare_episode(plain)
    assert plain["collection_config"]["episode_color_assignment"] == expected
    assert plain["collection_config"]["realized_randomization"]["episode_color_assignment"] == expected
