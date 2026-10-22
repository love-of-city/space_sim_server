import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_candidate_has_symmetric_zero_thickness_planes_and_original_bits():
    spec=importlib.util.spec_from_file_location('flush_generator',ROOT/'tools/generate_flush_capture26_markers.py')
    generator=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    original=json.loads((ROOT/'config/visual_attachments/target_aruco_5cm/aruco_rig.json').read_text())
    rig,rows=generator.generate(original)
    assert len(rows)==256 and all(r['shape']=='plane' and r['dimensions_m'][2]==0 for r in rows)
    assert all(r['render_role']=='visual' and r['material_emission']==0 for r in rows)
    assert [m['bits'] for m in rig['markers']]==[m['bits'] for m in original['markers']]
    assert all(m['size_m']==.02 for m in rig['markers'])
    assert abs(sum(m['center_m'][0] for m in rig['markers'])/4-.1755)<1e-10
    assert abs(sum(m['center_m'][2] for m in rig['markers'])/4-.0420787001299893)<1e-10
    assert original['markers'][0]['size_m']==.05


def test_authorized_film_preserves_5cm_and_uses_no_board_or_physics_geometry():
    rig=json.loads((ROOT/'config/visual_attachments/target_aruco_flush_5cm_v2/aruco_rig.json').read_text())
    rows=json.loads((ROOT/'config/visual_attachments/target_aruco_flush_5cm_v2/geometry_metres.json').read_text())
    assert rig['hole_bridging_authorized'] is True
    assert all(m['size_m']==.05 for m in rig['markers'])
    assert rig['physical_thickness_m']==0 and len(rows)==256
    assert rig['mounting']['mounting_plate'] is False
    assert rig['mounting']['plate_thickness_m']==0
    assert all(r['shape']=='plane' and r['dimensions_m'][2]==0 for r in rows)
    assert all(abs(r['position_body_m'][1]-.05476)<1e-12 for r in rows)
