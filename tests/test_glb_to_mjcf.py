"""Regression tests for the deliberately limited Draco GLB visual converter."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest

np = pytest.importorskip('numpy')
DracoPy = pytest.importorskip('DracoPy')
pytest.importorskip('mujoco')
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('glb_mjcf_converter', ROOT/'tools/convert_glb_to_mjcf.py')
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


def synthetic_doc(*, planar=False, degenerate=False):
    vertices = np.array([[0., 0, 0], [1., 0, 0], [0., 1, 0], [0., 0, 1]])
    faces = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]], dtype=np.uint32)
    if planar:
        vertices[3] = [1, 1, 0]
        faces = np.array([[0, 1, 2], [1, 3, 2]], dtype=np.uint32)
    if degenerate:
        faces = np.concatenate([faces, [[0, 0, 1]]]).astype(np.uint32)
    normals = np.tile([0., 0., 1.], (4, 1))
    # Non-sequential ID tests the extension's unique-ID mapping explicitly.
    binary = DracoPy.encode(vertices, faces, preserve_order=True, generic_attributes={7: normals})
    decoded = DracoPy.decode(binary)
    position_id = next(attr['unique_id'] for attr in decoded.attributes if attr['attribute_type'] == 0)
    doc = dict(asset={'version': '2.0'}, buffers=[dict(byteLength=len(binary))],
               extensionsUsed=[converter.DRACO_EXTENSION], extensionsRequired=[converter.DRACO_EXTENSION],
               bufferViews=[dict(buffer=0, byteOffset=0, byteLength=len(binary))],
               accessors=[dict(componentType=5126, type='VEC3', count=len(decoded.points)),
                          dict(componentType=5126, type='VEC3', count=len(decoded.points)),
                          dict(componentType=5125, type='SCALAR', count=decoded.faces.size)],
               materials=[dict(name='Gold', pbrMetallicRoughness=dict(baseColorFactor=[0.8, 0.4, 0.1, 1.]))],
               meshes=[dict(primitives=[dict(attributes={'POSITION': 0, 'NORMAL': 1}, indices=2, material=0,
                                            extensions={converter.DRACO_EXTENSION: dict(
                                                bufferView=0, attributes={'POSITION': position_id, 'NORMAL': 7})})])],
               nodes=[dict(name='parent', translation=[2, 3, 4], children=[1]),
                      dict(name='surface', mesh=0, translation=[-1, 2, 1], scale=[2, 3, 4])],
               scenes=[dict(nodes=[0])], scene=0)
    return doc, binary


def write_glb(path, doc, binary):
    payload = json.dumps(doc).encode()
    payload += b' ' * (-len(payload) % 4)
    binary += b'\0' * (-len(binary) % 4)
    raw = (struct.pack('<4sII', b'glTF', 2, 28 + len(payload) + len(binary))
           + struct.pack('<II', len(payload), 0x4E4F534A) + payload
           + struct.pack('<II', len(binary), 0x004E4942) + binary)
    path.write_bytes(raw)


@pytest.mark.parametrize('planar', [False, True])
def test_convert_hierarchy_positions_colors_normals_and_static_physics(tmp_path, planar):
    doc, binary = synthetic_doc(planar=planar)
    source = tmp_path/'scene.glb'
    write_glb(source, doc, binary)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    report = converter.convert(source, tmp_path/'converted', name='test_visual', scale=0.1)
    assert report['status'] == 'complete'
    assert report['source']['sha256'] == hashlib.sha256(source.read_bytes()).hexdigest() == digest
    assert report['validation']['compiled']
    assert report['validation']['joints'] == report['validation']['dofs'] == 0
    assert report['validation']['all_body_masses_and_inertias_zero']
    assert report['validation']['max_triangle_vertex_error_m'] < 1e-6
    assert report['validation']['max_normal_vector_error'] < 1e-6
    expected_dimensions = np.array([2., 0. if planar else 4., 3.]) * 0.1
    np.testing.assert_allclose(report['dimensions_m'], expected_dimensions, atol=1e-6)
    xml = ET.parse(tmp_path/'converted/test_visual.xml').getroot()
    assert xml.find('.//joint') is None and xml.find('.//inertial') is None
    assert xml.find('.//body[@name="node_000_parent"]/body[@name="node_001_surface"]') is not None
    assert xml.find('default/geom').get('contype') == '0'
    assert xml.find('compiler').get('inertiafromgeom') == 'false'
    assert report['coordinates']['physical_scale_verified'] is False


def test_zero_area_only_cleanup_and_attribute_unique_ids():
    doc, binary = synthetic_doc(degenerate=True)
    mesh = converter.decode_primitive(doc, binary, doc['meshes'][0]['primitives'][0])
    assert mesh['decoded_triangles'] == 5
    assert mesh['dropped_zero_area_triangles'] == 1
    assert len(mesh['faces']) == 4
    np.testing.assert_allclose(mesh['normals'], np.tile([0., 0., 1.], (4, 1)))


def test_reflection_flips_winding_and_nonuniform_scale_uses_normal_inverse_transpose():
    v = np.array([[0., 0, 0], [1, 0, 0], [0, 1, 1]])
    n = np.tile(np.array([0., -1, 1])/np.sqrt(2), (3, 1))
    f = np.array([[0, 1, 2]])
    transform = np.diag([-2., 3., 4., 1.])
    v2, n2, f2 = converter.transform_geometry(v, n, f, transform)
    cross = np.cross(v2[f2[0, 1]]-v2[f2[0, 0]], v2[f2[0, 2]]-v2[f2[0, 0]])
    cross /= np.linalg.norm(cross)
    np.testing.assert_allclose(n2, np.tile(cross, (3, 1)), atol=1e-12)
    assert f2.tolist() == [[0, 2, 1]]


def test_matrix_column_major_and_trs_composition_agree():
    node = dict(translation=[1, 2, 3], rotation=[0, 0, np.sin(0.3), np.cos(0.3)], scale=[2, 3, 4])
    transform = converter.node_matrix(node)
    np.testing.assert_allclose(converter.node_matrix(dict(matrix=transform.flatten(order='F').tolist())), transform)
    np.testing.assert_allclose(transform[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(np.linalg.norm(transform[:3, :3], axis=0), [2, 3, 4])


@pytest.mark.parametrize('feature', ['textures', 'animations', 'skins', 'images'])
def test_unsupported_features_rejected_without_writing_output(tmp_path, feature):
    doc, binary = synthetic_doc()
    doc[feature] = [{}]
    source = tmp_path/'bad.glb'
    write_glb(source, doc, binary)
    with pytest.raises(ValueError, match=feature):
        converter.convert(source, tmp_path/'should_not_exist')
    assert not (tmp_path/'should_not_exist').exists()


def test_reject_corrupt_chunk_and_refuse_overwrite(tmp_path):
    source = tmp_path/'bad.glb'
    source.write_bytes(b'glTF')
    with pytest.raises(ValueError, match='Truncated'):
        converter.read_glb(source)
    output = tmp_path/'existing'
    output.mkdir()
    (output/'user.txt').write_text('keep')
    with pytest.raises(FileExistsError):
        converter.convert(source, output)
    assert (output/'user.txt').read_text() == 'keep'


@pytest.mark.parametrize('scale', [0, -1, float('inf'), float('nan')])
def test_reject_invalid_scale(scale):
    doc, binary = synthetic_doc()
    with pytest.raises(ValueError, match='Scale'):
        converter.prepare(doc, binary, scale)


def test_reject_cycles():
    doc, _ = synthetic_doc()
    doc['nodes'][1]['children'] = [0]
    with pytest.raises(ValueError, match='cyclic'):
        converter.scene_nodes(doc)


def test_nasa_lro_real_source_integration(tmp_path):
    source = ROOT/'run/nasa-lro-a/Lunar Reconnaissance Orbiter (A).glb'
    if not source.exists():
        pytest.skip('Downloaded NASA GLB is optional and not checked into git')
    report = converter.convert(source, tmp_path/'nasa_lro_a')
    assert report['source']['sha256'] == 'b9547bc7c8c37c44675a80c3c3e27b647a9ff1293cf40030ef36f806b9c616b1'
    assert report['counts']['source_mesh_groups'] == 5
    assert report['counts']['source_materials'] == 34
    assert report['counts']['visual_primitives'] == 55
    assert report['counts']['decoded_triangles'] == 76349
    assert report['counts']['dropped_zero_area_triangles'] == 2250
    assert report['counts']['exported_triangles'] == 74099
    assert report['validation']['bodies_including_world'] == 13
    assert not any(report['validation']['warning_counts'])
    np.testing.assert_allclose(report['dimensions_m'], [33.277178, 32.258003, 26.293749], atol=1e-5)


def test_delivered_model_assets_and_manifest():
    folder = ROOT/'model/nasa_lro_a'
    if not (folder/'conversion_manifest.json').exists():
        pytest.skip('Delivery has not yet been generated')
    report = json.loads((folder/'conversion_manifest.json').read_text('utf-8'))
    assert hashlib.sha256((folder/report['model_file']).read_bytes()).hexdigest() == report['model_sha256']
    for asset in report['assets']:
        assert hashlib.sha256((folder/asset['file']).read_bytes()).hexdigest() == asset['sha256']
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(folder/report['model_file']))
    assert (model.nmesh, model.ngeom, model.njnt, model.nv, model.nu) == (55, 55, 0, 0, 0)
    assert np.all(model.body_mass == 0) and np.all(model.geom_contype == 0)
