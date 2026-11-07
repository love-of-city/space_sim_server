"""Complete-model preservation and adversarial provenance checks; no dynamics."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

from simulation.benchmark_capture_model import (
    MODEL_FILENAME, MODEL_PATH, PLUGIN_SOURCES, ROOT_BODIES, SETTINGS_PATH,
    prepare_capture_model, verify_capture_model,
)

SERVER = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture(scope='module')
def prepared(tmp_path_factory):
    folder = tmp_path_factory.mktemp('capture-preparation')/'model'
    source = SERVER/MODEL_PATH
    before = (digest(source), source.stat().st_mtime_ns)
    model, files, manifest = prepare_capture_model(SERVER, folder)
    assert (digest(source), source.stat().st_mtime_ns) == before
    return model, files, manifest


def test_complete_team_model_keeps_every_authored_physical_element(prepared):
    model, files, manifest = prepared
    original = ET.parse(SERVER/MODEL_PATH).getroot()
    generated = ET.parse(model).getroot()
    # Restore only the actual changed filenames, then independently compare
    # every element/attribute and ordering against the original XML tree.
    original_meshes = original.findall('./asset/mesh[@file]')
    generated_meshes = generated.findall('./asset/mesh[@file]')
    assert len(original_meshes) == len(generated_meshes) == 1587
    for before, after in zip(original_meshes, generated_meshes, strict=True):
        path = ((SERVER/MODEL_PATH).parent/before.get('file')).resolve()
        assert after.get('file') == path.as_posix() and str(path) in files
        assert manifest['bound_files_sha256'][path.relative_to(SERVER).as_posix()] == digest(path)
        after.set('file', before.get('file'))
    assert [(node.tag, node.attrib) for node in original.iter()] == [
        (node.tag, node.attrib) for node in generated.iter()]
    assert manifest['topology']['root_bodies'] == list(ROOT_BODIES)
    assert manifest['topology']['equality_constraints'] == 4
    assert len(manifest['capture_joints']) == 12
    assert [row['position_unit'] for row in manifest['capture_joints']] == ['rad']*6+['m', 'rad']+['m']*4
    assert [row['effort_limit'] for row in manifest['capture_joints']] == [80.]*6+[15., 1.]+[5.]*4
    assert 'bodies' not in manifest  # No fictional constant articulated COM.
    assert b'\r\n' not in model.read_bytes()
    assert (model.parent/'attitude_control.json').read_bytes() == (SERVER/SETTINGS_PATH).read_bytes()
    assert all(name in manifest['bound_files_sha256'] for name in PLUGIN_SOURCES)


def test_model_verification_is_readonly_and_binds_all_original_resources(prepared):
    model, _, manifest = prepared
    paths = [model, model.parent/'model_manifest.json', model.parent/'attitude_control.json']
    paths.extend(SERVER/key for key in manifest['bound_files_sha256'])
    before = {str(path): (digest(path), path.stat().st_mtime_ns) for path in paths}
    assert verify_capture_model(SERVER, model.parent) == manifest
    assert {str(path): (digest(path), path.stat().st_mtime_ns) for path in paths} == before
    with pytest.raises(FileExistsError):
        prepare_capture_model(SERVER, model.parent)
    assert {str(path): (digest(path), path.stat().st_mtime_ns) for path in paths} == before


@pytest.mark.parametrize('change', ['joint', 'inertia', 'collision', 'equality', 'resource', 'extra_actuator'])
def test_rehashed_model_edit_cannot_turn_into_the_original_model(tmp_path, prepared, change):
    model, _, manifest = prepared
    destination = tmp_path/'tampered'
    shutil.copytree(model.parent, destination)
    document = ET.parse(destination/MODEL_FILENAME)
    root = document.getroot()
    if change == 'joint':
        body = root.find('.//body[@name="jaw_0"]')
        body.remove(body.find('joint'))
    elif change == 'inertia':
        root.find('.//inertial').set('mass', '1')
    elif change == 'collision':
        root.find('.//geom[@name="capture_x_piece_0"]').set('contype', '0')
    elif change == 'equality':
        root.remove(root.find('equality'))
    elif change == 'resource':
        meshes = root.findall('./asset/mesh[@file]')
        meshes[0].set('file', meshes[1].get('file'))
    else:
        ET.SubElement(root.find('actuator'), 'motor', name='undeclared_motor', joint='bottom_joint1')
    document.write(destination/MODEL_FILENAME, encoding='utf-8', xml_declaration=True)
    changed = deepcopy(manifest)
    changed['generated_sha256'] = digest(destination/MODEL_FILENAME)
    (destination/'model_manifest.json').write_text(json.dumps(changed), encoding='utf-8')
    with pytest.raises(ValueError, match='physical tree'):
        verify_capture_model(SERVER, destination)


@pytest.mark.parametrize('change', ['provenance', 'settings'])
def test_runtime_input_edits_are_rejected(tmp_path, prepared, change):
    model, _, _ = prepared
    destination = tmp_path/'changed'
    shutil.copytree(model.parent, destination)
    path = destination/('model_manifest.json' if change == 'provenance' else 'attitude_control.json')
    record = json.loads(path.read_text('utf-8'))
    record['unrecorded_change'] = True
    path.write_text(json.dumps(record), encoding='utf-8')
    with pytest.raises(ValueError, match='provenance|hardware settings'):
        verify_capture_model(SERVER, destination)


@pytest.mark.parametrize('change', ['stowed_joint', 'missing_root', 'include', 'compiler',
                                   'path_escape', 'lfs', 'empty', 'unknown_resource'])
def test_invalid_source_fails_before_creating_output(tmp_path, change):
    repository = tmp_path/'source'
    source = repository/MODEL_PATH
    source.parent.mkdir(parents=True)
    root = ET.parse(SERVER/MODEL_PATH).getroot()
    first_mesh = root.find('./asset/mesh[@file]')
    if change == 'stowed_joint':
        body = root.find('.//body[@name="bottom_link1"]')
        body.remove(body.find('joint'))
    elif change == 'missing_root':
        world = root.find('worldbody')
        world.remove(world.find('body[@name="aviation_plug"]'))
    elif change == 'include':
        ET.SubElement(root, 'include', file='other.xml')
    elif change == 'compiler':
        root.find('compiler').set('angle', 'degree')
    elif change == 'path_escape':
        first_mesh.set('file', '../../../../outside.stl')
    elif change in ('lfs', 'empty'):
        first_mesh.set('file', 'unavailable.stl')
        (source.parent/'unavailable.stl').write_bytes(
            b'version https://git-lfs.github.com/spec/v1\noid sha256:missing\n' if change == 'lfs' else b'')
    else:
        ET.SubElement(root.find('asset'), 'texture', name='unbound', file='private.png')
    ET.ElementTree(root).write(source, encoding='utf-8', xml_declaration=True)
    destination = tmp_path/'episode/model'
    with pytest.raises(ValueError):
        prepare_capture_model(repository, destination)
    assert not destination.exists()
