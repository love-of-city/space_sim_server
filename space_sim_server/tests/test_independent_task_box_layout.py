"""Packing and recording contracts for independently installed mechanisms."""
import copy
import hashlib
import json
import random
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.independent_task_box_layout import (
    MODULES, PICKUPS, POSE_SCHEMA, SCHEMA, TARGETS, normalize_geometry_order,
    planar_transform, rotated_footprints, sample_placements, sample_pose_placements,
    validate_pose_placements, validate_placements,
)
from simulation.task_box_layout import validated_capture_layout


@pytest.fixture
def deck():
    # Conservative sizes of the delivered split plates and swept mechanisms.
    sizes = [(110, 106), (120, 129), (110, 106), (67, 82), (67, 82), (67, 82),
             (67, 82), (70, 98), (67, 82), (93, 93), (93, 82), (242, 83), (208, 63)]
    footprints = {name:[[0., 0.], [w/1000, h/1000]] for name, (w, h) in zip(MODULES, sizes, strict=True)}
    return footprints, [[[.06, .2987], [.16, .55]]]


def test_all_units_pack_reproducibly_with_clearance_and_without_global_rng_changes(deck):
    footprints, obstacles = deck
    before = random.getstate()
    first, _ = sample_placements(footprints, 20261021, obstacles)
    assert first == sample_placements(footprints, 20261021, obstacles)[0]
    assert first != sample_placements(footprints, 20261022, obstacles)[0]
    assert len({tuple(v) for v in first.values()}) == 13
    assert random.getstate() == before
    for seed in range(20):
        offsets, _ = sample_placements(footprints, seed, obstacles)
        validate_placements(footprints, offsets, obstacles)
        boxes = {k:np.asarray(v)+offsets[k] for k,v in footprints.items()}
        for i, name in enumerate(MODULES):
            assert np.all(boxes[name][0] >= .005-1e-10)
            assert np.all(boxes[name][1] <= np.array([.445, .545])+1e-10)
            for other in MODULES[:i]:
                # Independent interval distance, including negative distances
                # when intervals overlap, must leave one separated axis.
                gap = np.maximum(boxes[name][0]-boxes[other][1], boxes[other][0]-boxes[name][1])
                assert max(gap) >= .008-1e-10


@pytest.mark.parametrize("failure", ["missing", "overlap", "edge", "obstacle", "nan", "negative_gap"])
def test_invalid_placements_are_rejected(deck, failure):
    footprints, obstacles = deck
    offsets, _ = sample_placements(footprints, 12, obstacles)
    kwargs = {}
    if failure == "missing":
        offsets.pop(MODULES[0])
    elif failure == "overlap":
        offsets[MODULES[1]] = offsets[MODULES[0]]
    elif failure == "edge":
        offsets[MODULES[0]] = [-.1, .1]
    elif failure == "obstacle":
        offsets[MODULES[0]] = [.06, .30]
    elif failure == "nan":
        offsets[MODULES[0]] = [float("nan"), .1]
    else:
        kwargs["gap"] = -.01
    with pytest.raises(ValueError):
        validate_placements(footprints, offsets, obstacles, **kwargs)


def test_oversized_or_impossible_problem_has_bounded_failure(deck):
    footprints, obstacles = deck
    footprints[MODULES[0]] = [[0, 0], [.6, .1]]
    with pytest.raises(ValueError, match="larger"):
        sample_placements(footprints, 1, obstacles)
    footprints[MODULES[0]] = [[0, 0], [.44, .54]]
    with pytest.raises(ValueError, match="No disjoint"):
        sample_placements(footprints, 1, obstacles, max_restarts=2)


def test_yaw_rotates_about_installation_anchor_and_keeps_deck_height():
    anchor = [.2, .3, .005]
    matrix = planar_transform(anchor, [.05, -.1], np.pi/2)
    points = np.array([[.2, .3, .005, 1.], [.24, .31, -.015, 1.]])
    actual = (matrix @ points.T).T[:, :3]
    np.testing.assert_allclose(actual, [[.25, .2, .005], [.24, .24, -.015]], atol=1e-15)


def test_xy_yaw_sampling_is_reproducible_and_checks_rotated_swept_boxes():
    footprints = {name: [[.1, .2], [.14, .25]] for name in MODULES}
    footprints['drawer'] = [[.1, .2], [.342, .282]]
    footprints['locking_mechanism'] = [[.1, .2], [.308, .263]]
    anchors = {name: [*.5*(np.asarray(box)[0]+np.asarray(box)[1]), .005] for name, box in footprints.items()}
    obstacles = [[[.06, .2987], [.16, .55]]]
    before = random.getstate()
    first = sample_pose_placements(footprints, anchors, 12, obstacles)
    assert first == sample_pose_placements(footprints, anchors, 12, obstacles)
    assert random.getstate() == before
    positions, angles = [], []
    for seed in range(20):
        offsets, yaws, _ = sample_pose_placements(footprints, anchors, seed, obstacles)
        boxes = validate_pose_placements(footprints, offsets, anchors, yaws, obstacles)
        positions.append(tuple(offsets['drawer']))
        angles.append(yaws['drawer'])
        for name in MODULES:
            a = np.asarray(anchors[name])[:2]
            c, s = np.cos(yaws[name]), np.sin(yaws[name])
            r = np.array([[c, -s], [s, c]])
            box = np.asarray(footprints[name])
            corners = np.array([[x, y] for x in box[:, 0] for y in box[:, 1]])
            transformed = (corners-a) @ r.T + a + offsets[name]
            np.testing.assert_allclose(boxes[name], [transformed.min(0), transformed.max(0)], atol=1e-14)
    assert len(set(positions)) == len(set(angles)) == 20
    with pytest.raises(ValueError, match='yaws'):
        rotated_footprints(footprints, anchors, {})
    for angle in [float('nan'), float('inf'), 2*np.pi, True]:
        with pytest.raises(ValueError):
            planar_transform(anchors[MODULES[0]], [0., 0.], angle)
    with pytest.raises(ValueError, match='half-range'):
        sample_pose_placements(footprints, anchors, 0, yaw_range_deg=181)
    impossible = {name: [[0., 0.], [.44, .54]] for name in MODULES}
    with pytest.raises(ValueError, match='sampling budget'):
        sample_pose_placements(impossible, anchors, 0, max_restarts=1, max_attempts=2)


def test_appended_parent_geoms_cannot_shift_renderer_mesh_binding():
    root = ET.fromstring('''<mujoco><worldbody><body name="base" pos=".1 .2 0">
      <joint name="base_free" type="free"/><geom name="original" mesh="original_mesh"/>
      <body name="drawer"><joint name="slide" type="slide"/><geom name="drawer_visual" mesh="drawer_mesh"/></body>
      <geom name="new_base" mesh="new_base_mesh"/>
      <body name="socket"><geom name="socket_visual" mesh="socket_mesh"/></body>
      <geom name="new_collision" mesh="collision_mesh"/>
    </body></worldbody></mujoco>''')
    joints = [dict(node.attrib) for node in root.iter("joint")]
    bodies = [dict(node.attrib) for node in root.iter("body")]
    nodes = {id(node):dict(node.attrib) for node in root.iter()}
    normalize_geometry_order(root)
    assert [node.get("name") for node in root.iter("geom")] == [
        "original", "new_base", "new_collision", "drawer_visual", "socket_visual"]
    assert [dict(node.attrib) for node in root.iter("joint")] == joints
    assert [dict(node.attrib) for node in root.iter("body")] == bodies
    assert {id(node):dict(node.attrib) for node in root.iter()} == nodes
    once = ET.tostring(root)
    normalize_geometry_order(root)
    assert ET.tostring(root) == once


def test_recording_requires_every_independent_visual_and_original_state_contract(tmp_path, deck):
    footprints, obstacles = deck
    offsets, _ = sample_placements(footprints, 1, obstacles)
    model = tmp_path/"layout.xml"
    model.write_text("<mujoco/>")
    visuals, entries = [], {}
    for name in (*PICKUPS, *TARGETS, "task_box_base"):
        path = tmp_path/(name+".stl")
        path.write_bytes(name.encode())
        sha = hashlib.sha256(path.read_bytes()).hexdigest()
        visuals.append(dict(name=name, file=str(path), sha256=sha))
        entries[str(path)] = dict(source_sha256=sha)
    record = dict(schema=SCHEMA, model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                  initial_states_randomized=False, orientations_randomized=False,
                  adapted_base=dict(visual=visuals[-1]["file"]), generated_visuals=visuals,
                  offsets_body_xy_m=offsets, original_footprints_body_xy_m=footprints,
                  fixed_obstacles_body_xy_m=obstacles, deck_edge_margin_m=.005, minimum_module_gap_m=.008)
    manifest, catalog = model.with_suffix(".layout.json"), tmp_path/"catalog.json"
    manifest.write_text(json.dumps(record))
    catalog_data = dict(source_mjcf=str(model), assets=entries)
    catalog.write_text(json.dumps(catalog_data))
    assert validated_capture_layout(model, catalog) == record
    changed = copy.deepcopy(record)
    changed["orientations_randomized"] = True
    manifest.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match="state/placement"):
        validated_capture_layout(model, catalog)
    rotated = copy.deepcopy(record)
    rotated.update(schema=POSE_SCHEMA, orientations_randomized=True,
                   rotation_axis='task_box_base/+Z', installation_yaw_rad=dict.fromkeys(MODULES, 0.),
                   modules={name: dict(original_anchor_body_m=[0., 0., .005]) for name in MODULES})
    manifest.write_text(json.dumps(rotated))
    assert validated_capture_layout(model, catalog) == rotated
    rotated['installation_yaw_rad'].pop(MODULES[0])
    manifest.write_text(json.dumps(rotated))
    with pytest.raises(ValueError, match='yaws'):
        validated_capture_layout(model, catalog)
    manifest.write_text(json.dumps(record))
    entries.pop(visuals[0]["file"])
    catalog.write_text(json.dumps(catalog_data))
    with pytest.raises(ValueError, match="independent visual"):
        validated_capture_layout(model, catalog)
