"""Satellite mesh extraction, physical provenance and collection integration."""
from copy import deepcopy
from dataclasses import replace
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from test_floating_capture import MODEL, source_scene
from collection.record_scripted_task import task_instruction
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.floating_assets import FLOATING_TASKS
from simulation.scripted_tasks.floating_objects import validate_floating_object_recording
from simulation.scripted_tasks.randomization import write_model_variant
from simulation.scripted_tasks.satellite_components import SATELLITE_COMPONENTS, build_satellite_component


TASKS = [name for name, asset in FLOATING_TASKS.items() if asset.source_component]


@pytest.mark.parametrize("task", TASKS)
def test_extraction_matches_assembly_inventory_and_source_coordinates(task):
    root = ET.parse(MODEL).getroot()
    before = ET.tostring(root)
    asset = FLOATING_TASKS[task]
    spec = SATELLITE_COMPONENTS[asset.source_component]
    inventory = json.loads((MODEL.parents[2]/"ground_validation_satellite/articulation_review/assembly_inventory.json").read_text())
    assembly = next(x for x in inventory if x["id"] == spec.assembly_id)
    assert list(spec.geoms) == assembly["geoms"]
    np.testing.assert_allclose(spec.center_m, assembly["center_m"], atol=1e-12)
    np.testing.assert_allclose(spec.size_m, assembly["size_m"], atol=1e-12)
    body = build_satellite_component(root, asset.source_component, asset.body)
    assert ET.tostring(root) == before
    assert len(body.findall("freejoint")) == 1 and body.find("body") is None
    inertial = body.find("inertial")
    assert float(inertial.get("mass")) == spec.mass_kg > 0
    inertia = np.fromstring(inertial.get("diaginertia"), sep=" ")
    assert np.all(inertia > 0) and 2*max(inertia) < sum(inertia)
    for name in spec.geoms:
        original = root.find(f'.//body[@name="capture_target"]/geom[@name="target_{name}"]')
        visual = body.find(f'geom[@name="{asset.body}_{name}"]')
        assert visual.get("mesh") == original.get("mesh")
        assert visual.get("quat") == original.get("quat")
        assert visual.get("rgba") == original.get("rgba")
        assert (visual.get("mass"), visual.get("contype"), visual.get("conaffinity")) == ("0", "0", "0")
        np.testing.assert_allclose(np.fromstring(visual.get("pos"), sep=" ")+spec.center_m,
                                   np.fromstring(original.get("pos"), sep=" "), atol=1e-12)
    colliders = body.findall('geom[@group="3"]')
    assert len(colliders) == len(spec.collision_geoms) + len(spec.collision_primitives)
    assert all(g.get("contype") == g.get("conaffinity") == "1" and g.get("class") == "object_geom" for g in colliders)
    assert {g.get("mesh") for g in colliders if g.get("type") == "mesh"} == {
        root.find(f'.//geom[@name="target_{n}"]').get("mesh") for n in spec.collision_geoms}
    root.find("worldbody").append(body)
    with pytest.raises(ValueError, match="unique"):
        build_satellite_component(root, asset.source_component, asset.body)


@pytest.mark.parametrize("task", TASKS)
@pytest.mark.parametrize("mode", ["single", "same-type-colors", "new-distractor", "new-target"])
def test_components_replay_as_target_and_distractor(tmp_path, task, mode):
    source = source_scene()
    source["floating_capture"] = dict(min_distance_m=.35, max_distance_m=.55, max_tilt_deg=0)
    source["floating_target"] = dict(color_choices=["blue"])
    selected = "floating_plug_capture" if mode == "new-distractor" else task
    if mode != "single":
        source["floating_scene"] = dict(object_count=2, composition=("same-type-colors" if mode == "same-type-colors" else "different-types"))
        if mode != "same-type-colors":
            source["floating_scene"]["distractor_types"] = [task if mode == "new-distractor" else "floating_bolt_capture"]
    scene = randomized_scene(source, 75, 0, selected)
    model = write_model_variant(MODEL, tmp_path/"model.xml", scene)
    reloaded = json.loads(json.dumps(scene))
    replay = write_model_variant(MODEL, tmp_path/"replay.xml", reloaded)
    assert replay.read_bytes() == model.read_bytes()
    instruction = task_instruction(scene, selected, None)
    validate_floating_object_recording(scene["collection_config"], [], instruction)
    root = ET.parse(model).getroot()
    original = ET.parse(MODEL).getroot()
    assert ET.tostring(root.find('.//body[@name="capture_target"]')) == ET.tostring(original.find('.//body[@name="capture_target"]'))
    for obj in scene["collection_config"]["floating_objects"]["instances"]:
        body = root.find(f'./worldbody/body[@name="{obj["instance_id"]}"]')
        assert body is not None
        assert len(body.findall("freejoint")) + len(body.findall('joint[@type="free"]')) == 1
        component_id = FLOATING_TASKS[obj["type_task"]].source_component
        if component_id:
            spec = SATELLITE_COMPONENTS[component_id]
            assert len(body.findall('geom[@group="2"]')) == len(spec.geoms)
            assert obj["initial_state"]["asset_snapshot"]["component_definition"] == spec.snapshot()
            assert len(obj["variation"]["colored_geoms"]) == len(spec.geoms)
    for tag in ("body", "geom", "freejoint"):
        names = [e.get("name") for e in root.iter(tag) if e.get("name")]
        assert len(names) == len(set(names))


@pytest.mark.parametrize("mode", ["target", "distractor"])
def test_changed_component_definition_rejects_saved_episode(tmp_path, monkeypatch, mode):
    source = source_scene()
    task = "floating_sun_sensor_capture" if mode == "target" else "floating_plug_capture"
    if mode == "distractor":
        source["floating_capture"] = dict(min_distance_m=.35, max_distance_m=.55)
        source["floating_scene"] = dict(object_count=2, distractor_types=["floating_sun_sensor_capture"])
    scene = randomized_scene(source, 75, 0, task)
    write_model_variant(MODEL, tmp_path/"model.xml", scene)
    monkeypatch.setitem(SATELLITE_COMPONENTS, "sun_sensor", replace(SATELLITE_COMPONENTS["sun_sensor"], mass_kg=.1))
    with pytest.raises(ValueError, match="snapshot|Saved floating objects"):
        write_model_variant(MODEL, tmp_path/"changed.xml", deepcopy(scene))


def test_missing_component_member_fails_before_recording():
    root = ET.parse(MODEL).getroot()
    source = root.find('.//body[@name="capture_target"]')
    source.remove(source.find('geom[@name="target_instance_0077_part_034_color_00"]'))
    with pytest.raises(ValueError, match="Missing satellite component mesh"):
        build_satellite_component(root, "sun_sensor", "missing")


def test_data_antenna_preserves_air_under_plate_and_grasps_solid_thickness():
    root = ET.parse(MODEL).getroot()
    body = build_satellite_component(root, "data_antenna", "test_antenna")
    colliders = body.findall('geom[@group="3"]')
    assert not any(g.get("type") == "mesh" for g in colliders)

    def containing(point):
        result = []
        for g in colliders:
            local = np.asarray(point)-np.fromstring(g.get("pos"), sep=" ")
            size = np.fromstring(g.get("size"), sep=" ")
            inside = (np.all(np.abs(local) <= size) if g.get("type") == "box" else
                      np.linalg.norm(local[:2]) <= size[0] and abs(local[2]) <= size[1])
            if inside:
                result.append(g)
        return result

    assert not containing((-.038, .011, 0.))  # Previously filled by the convex hull.
    assert not containing((.006, 0., -.006))  # Do not inflate the rear shaft.
    assert containing((0., 0., -.006))  # Real rear connector is retained.
    asset = FLOATING_TASKS["floating_data_antenna_capture"]
    assert [g.get("type") for g in containing(asset.grip_point)] == ["box"]
    plate = body.find('geom[@type="box"]')
    p, size = (np.fromstring(plate.get(k), sep=" ") for k in ("pos", "size"))
    assert p[2]-size[2] == pytest.approx(.00211367221513)
    assert p[2]+size[2] == pytest.approx(.00612967221513)
    assert asset.grip_point[2] == pytest.approx(p[2])
