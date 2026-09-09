"""Read-only review invariants; integration checks skip without generated artifacts."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
np = pytest.importorskip('numpy')
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('component_review', ROOT/'tools/review_satellite_components.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)
FOLDER = ROOT/'model/ground_validation_satellite'
OUTPUT = FOLDER/'articulation_review'


def test_decode_step_names():
    assert review.decode_step_name('_X2_89E395015668_X0_01') == '解锁器01'
    assert review.decode_step_name('WXJL180-1_0001_1') == 'WXJL180-1_0001_1'


def test_axis_sign_and_axial_origin_are_equivalent():
    p, d = review.canonical_axis([1, 2, 90], [0, 0, -2])
    q, e = review.canonical_axis([1, 2, -20], [0, 0, 1])
    np.testing.assert_allclose(p, q)
    np.testing.assert_allclose(d, e)
    np.testing.assert_allclose(p, [1, 2, 0])


@pytest.mark.parametrize('point,direction', [([0, 0, 0], [0, 0, 0]), ([np.nan, 0, 0], [0, 0, 1]), ([0, 0, 0], [0, np.inf, 0])])
def test_invalid_axis_rejected(point, direction):
    with pytest.raises(ValueError):
        review.canonical_axis(point, direction)


def test_cad_millimetres_to_model_metres_with_rotation():
    c = dict(face_index=1, radius_mm=2, axis_point_local_mm=[1000, 0, 0],
             axis_direction_local=[1, 0, 0], area_mm2=12)
    t = np.array([[0, -1, 0, .1], [1, 0, 0, .2], [0, 0, 1, .3], [0, 0, 0, 1]])
    result = review.transform_cylinder(c, t, np.array([.01, .02, .03]))
    np.testing.assert_allclose(result['direction_model'], [0, 1, 0])
    np.testing.assert_allclose(result['line_point_model_m'], [.09, 0, .27])
    assert result['radius_m'] == .002


def test_component_selection_keeps_normal_indices_and_owner():
    g = dict(vertices=np.arange(18).reshape(6, 3), normals=np.arange(18, 36).reshape(6, 3),
             faces=np.array([[0, 1, 2], [3, 4, 5]]), normal_faces=np.array([[5, 4, 3], [2, 1, 0]]),
             colors=np.array([[1., 0, 0], [0, 1., 0]]), owners=np.array([6, 199]))
    out = review.select_geometry(g, ['instance_0199'])
    np.testing.assert_array_equal(out['vertices'][out['faces']], g['vertices'][g['faces'][1:]])
    np.testing.assert_array_equal(out['normals'][out['normal_faces']], g['normals'][g['normal_faces'][1:]])
    assert out['owners'].tolist() == [199]
    with pytest.raises(ValueError):
        review.select_geometry(g, ['instance_9999'])


def test_projection_places_annotation_at_expected_pixel():
    c = dict(eye=[0, 0, 0], basis=np.eye(3), width=1000, height=800, focal=100)
    np.testing.assert_allclose(review.project([[1, 2, 2]], c), [[550, 300, .5]])
    with pytest.raises(ValueError):
        review.project([[0, 0, -1]], c)


@pytest.mark.skipif(not (OUTPUT/'candidates.json').exists(), reason='Generated CAD review not present')
def test_real_candidates_are_unconfirmed_and_traceable():
    manifest = json.loads((FOLDER/'conversion_manifest.json').read_text(encoding='utf-8'))
    evidence = json.loads((OUTPUT/'cad_surface_evidence.json').read_text(encoding='utf-8'))
    nodes = review.build_inventory(manifest)
    groups, findings = review.make_findings(manifest, nodes, evidence)
    assert [c['id'] for c in findings['candidates']] == ['A', 'B', 'C', 'D']
    assert all(not c['confirmed'] and c['range_deg'] is None for c in findings['candidates'])
    assert nodes['instance_0111']['decoded_name'] == '自拍相机'
    assert len(nodes['instance_0001']['leaf_instances']) == 151
    assert len(nodes['instance_0198']['leaf_instances']) == 11
    assert not (set(groups['A']) & set(nodes['instance_0131']['leaf_instances']))
    assert set(groups['A']).isdisjoint(groups['B'])
    axis = findings['candidates'][0]['preliminary_axis']
    assert axis['max_line_offset_m'] < 1e-9
    np.testing.assert_allclose(axis['direction_model'], [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(axis['point_model_m'][:2], [-.0145, -.07702], atol=1e-9)
    assert len(axis['upper']['matched_cylindrical_faces']) == 10
    assert len(axis['lower']['matched_cylindrical_faces']) == 10
    assert [len(f['matched_cylindrical_faces']) for f in findings['candidates'][1]['preliminary_cylindrical_features']] == [17, 20, 19]


@pytest.mark.skipif(not (OUTPUT/'overview.svg').exists(), reason='Generated SVG review not present')
def test_svg_images_embedded_and_labels_inside_viewport():
    ns = {'s': 'http://www.w3.org/2000/svg'}
    for name in ('overview', 'rear_annotated', 'front_annotated', 'hinge_pair_annotated', 'selfie_camera_annotated'):
        root = ET.parse(OUTPUT/(name+'.svg')).getroot()
        assert root.findall('.//s:image', ns)
        assert all(im.attrib['href'].startswith('data:image/png;base64,') for im in root.findall('.//s:image', ns))
        if name != 'overview':
            for rect in root.findall('.//s:rect', ns):
                assert float(rect.attrib['x']) + float(rect.attrib['width']) <= 1000
                assert float(rect.attrib['y']) + float(rect.attrib['height']) <= 800


@pytest.mark.skipif(not (OUTPUT/'review.html').exists(), reason='Generated HTML review not present')
def test_html_feedback_defaults_and_portable_readme():
    text = (OUTPUT/'review.html').read_text(encoding='utf-8')
    assert text.count('<select data-id=') == 4
    assert text.count('<option value="unconfirmed">') == 4
    assert '<script src=' not in text
    assert '.overview > svg{' in text
    assert len((OUTPUT/'README.md').read_text(encoding='utf-8').splitlines()) > 20
