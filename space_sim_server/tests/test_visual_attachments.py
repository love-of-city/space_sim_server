"""Body-local visual attachments must never enter authoritative physics."""
import json
from pathlib import Path

import pytest


def test_attachment_keeps_body_local_pose_and_existing_geometry(tmp_path):
    from Basilisk.architecture import messaging
    from bsk_render_adapter import BasiliskRenderBridge, GeometryVisual
    from simulation.visual_attachments import attach_visual_geometries

    bridge = BasiliskRenderBridge(port=0)
    state = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    original = GeometryVisual('target/hull', 'box', (.2, .1, .4))
    bridge.add_object('teleop/capture_target', state, geometries=[original])
    path = tmp_path / 'marker.json'
    path.write_text(json.dumps([dict(geometry_id='marker/0', shape='box',
        dimensions_m=[.05, .00005, .05], position_body_m=[.1, .0578, .15],
        color_rgba=[0, 0, 0, 1], render_role='visual')]))
    added = attach_visual_geometries(bridge._objects,
        [dict(parent_id='teleop/capture_target', geometry_file='marker.json')], tmp_path)
    assert added == 1
    assert len(bridge._objects) == 1
    assert bridge._objects[0].geometries[0] is original
    payload = bridge._objects[0].geometries[1].to_payload()
    assert payload['position_body_m'] == [.1, .0578, .15]
    assert payload['dimensions_m'] == [.05, .00005, .05]
    assert payload['render_role'] == 'visual'
    assert not ({'mass', 'inertia', 'contype', 'conaffinity'} & payload.keys())


def test_missing_parent_is_rejected_instead_of_showing_detached_markers(tmp_path):
    from simulation.visual_attachments import attach_visual_geometries
    (tmp_path / 'marker.json').write_text('[]')
    with pytest.raises(ValueError, match='parent'):
        attach_visual_geometries([], [dict(parent_id='missing', geometry_file='marker.json')], tmp_path)


def test_imported_handoff_has_four_correctly_scaled_marker_boards():
    from types import SimpleNamespace
    from simulation.visual_attachments import attach_visual_geometries
    root = Path(__file__).resolve().parents[1]
    target = SimpleNamespace(object_id='teleop/capture_target', geometries=[])
    attach_visual_geometries([target], [dict(parent_id=target.object_id,
        geometry_file='config/visual_attachments/target_aruco_5cm/geometry_metres.json')], root)
    assert len(target.geometries) == 260
    boards = [g for g in target.geometries if g.geometry_id.endswith('/mount')]
    assert len(boards) == 4
    assert sorted(tuple(g.position_body_m) for g in boards) == [
        (.1, .056249999999999994, -.13), (.1, .056249999999999994, .15),
        (.25, .056249999999999994, -.13), (.25, .056249999999999994, .15)]
    for board in boards:
        assert board.dimensions_m == pytest.approx([.06666666666666667, .003, .06666666666666667])
    assert all(g.render_role == 'visual' and g.material_emission == 0 for g in target.geometries)
