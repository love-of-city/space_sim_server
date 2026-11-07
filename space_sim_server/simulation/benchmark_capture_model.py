"""Prepare the complete capture26 MJCF without changing its physical model.

Only resource filenames become absolute, for native/UE loading from an episode
directory. There is no stowing, welded target, added actuator or COM estimate.
The caller still owns initialization, orbital forces, servos and task grading.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.capture26_mission import MODEL_PATH

SCHEMA = 'spacesim-autonomy-capture-model/1'
MODEL_FILENAME = 'team_flight.xml'  # Shared renderer mirror convention, not a stowed model.
SETTINGS_PATH = MODEL_PATH.with_name('attitude_control.json')
PLUGIN_SOURCES = ('native/sarm_thread_sdf.cpp', 'model/SARM_mjcf_20261012/thread_sdf.cpp')
ROOT_BODIES = ('cubesat_bus', 'aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt', 'capture_target')


def _content(path):
    content = path.read_bytes()
    if not content or content.startswith(b'version https://git-lfs.github.com/spec/v1'):
        raise ValueError(f'capture model resource is empty or an unhydrated LFS pointer: {path}')
    return content


def _sha(content):
    return hashlib.sha256(content).hexdigest()


def _semantic(element):
    """Whitespace-independent tree identity, retaining order and every value."""
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            tuple(_semantic(child) for child in element), (element.tail or '').strip())


def _capture_joints(root):
    channels = []
    for index, (body_name, joint_name, initial) in enumerate(INITIAL_JOINTS):
        body = root.find(f'.//body[@name="{body_name}"]')
        joint = None if body is None else body.find(f'joint[@name="{joint_name}"]')
        actuator_name = (f'bottom_joint{index+1}_act' if index < 6 else
                         ('axial_act' if index == 6 else 'cam_act' if index == 7 else f'jaw_act_{index-8}'))
        actuator = root.find(f'./actuator/position[@name="{actuator_name}"]')
        if joint is None or actuator is None or actuator.get('joint') != joint_name:
            raise ValueError(f'original capture26 joint/servo is missing: {joint_name}')
        kind = joint.get('type', 'hinge')
        bounds = [float(value) for value in joint.get('range', '').split()]
        force = [float(value) for value in actuator.get('forcerange', '').split()]
        kp, kd = float(actuator.get('kp', 'nan')), float(actuator.get('kv', 'nan'))
        if (kind not in ('hinge', 'slide') or len(bounds) != 2 or len(force) != 2
                or not all(math.isfinite(value) for value in [*bounds, *force, initial, kp, kd])
                or not bounds[0] <= initial <= bounds[1] or not bounds[0] < bounds[1]
                or not force[0] < 0 < force[1] or force[0] != -force[1] or kp <= 0 or kd < 0):
            raise ValueError(f'invalid explicit capture26 joint/servo calibration: {joint_name}')
        channels.append(dict(body=body_name, joint=joint_name, actuator=actuator_name, type=kind,
            position_unit='rad' if kind == 'hinge' else 'm', position_range=bounds,
            initial_reference=initial, kp=kp, kd=kd, effort_limit=force[1],
            effort_unit='N m' if kind == 'hinge' else 'N'))
    return channels


def _prepared_document(repository):
    repository = Path(repository).resolve()
    source = repository/MODEL_PATH
    original = _content(source)
    root = ET.fromstring(original)
    if root.tag != 'mujoco' or root.find('.//include') is not None:
        raise ValueError('capture26 preparation requires the original expanded MJCF')
    compiler = root.find('compiler')
    if (compiler is None or compiler.get('angle') != 'radian' or compiler.get('meshdir') != '.'
            or compiler.get('fusestatic') != 'false'):
        raise ValueError('capture26 compiler convention changed')
    roots = tuple(body.get('name') for body in root.findall('./worldbody/body'))
    if roots != ROOT_BODIES:
        raise ValueError('capture26 requires all original service, free-part and target roots')
    if root.find('./extension/plugin[@plugin="sarm.sdf.two_start_thread"]') is None:
        raise ValueError('capture26 requires the original thread collision plugin')
    channels = _capture_joints(root)
    meshes = root.findall('./asset/mesh[@file]')
    if not meshes or len(root.findall('.//*[@file]')) != len(meshes):
        raise ValueError('capture26 has unexpected external resources; extend the binding explicitly')
    bindings = {MODEL_PATH.as_posix(): _sha(original)}
    files = []
    for mesh in meshes:
        relative = mesh.get('file')
        if Path(relative).is_absolute():
            raise ValueError('authored capture26 meshes must use repository-relative paths')
        path = (source.parent/relative).resolve()
        if not path.is_relative_to(repository/'model'):
            raise ValueError('capture26 mesh escaped the versioned model tree')
        key = path.relative_to(repository).as_posix()
        if key not in bindings:
            bindings[key] = _sha(_content(path))
            files.append(str(path))
        mesh.set('file', path.as_posix())
    settings = _content(repository/SETTINGS_PATH)
    # Copy this input exactly; task control cadence is a runtime declaration.
    json.loads(settings)
    bindings[SETTINGS_PATH.as_posix()] = _sha(settings)
    for name in PLUGIN_SOURCES:
        bindings[name] = _sha(_content(repository/name))
    physical_joints = root.findall('./worldbody//joint')
    topology = dict(root_bodies=list(roots), bodies=len(root.findall('.//body')),
        scalar_joints=sum(joint.get('type') != 'free' for joint in physical_joints),
        free_joints=len(root.findall('./worldbody//freejoint'))
            +sum(joint.get('type') == 'free' for joint in physical_joints),
        geometries=len(root.findall('.//geom')), mesh_declarations=len(meshes),
        actuator_names=[node.get('name') for node in root.find('actuator')],
        equality_constraints=len(root.findall('./equality/*')), keyframes=len(root.findall('./keyframe/key')))
    ET.indent(root)
    generated = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    manifest = dict(schema_version=SCHEMA, source_model=MODEL_PATH.as_posix(),
        source_sha256=_sha(original), generated_sha256=_sha(generated),
        bound_files_sha256=dict(sorted(bindings.items())), topology=topology, capture_joints=channels,
        derivation='absolute mesh filenames only; every authored physical element is retained',
        mass_properties='compute from the compiled articulated configuration; no fixed assembly COM supplied',
        plugin_status='source bound; the runtime must separately verify its loaded native plugin/build',
        scope='full-model preparation component; not a runnable task or docking/grasp acceptance')
    return root, generated, settings, files, manifest


def prepare_capture_model(repository: Path, destination: Path):
    """Create a fresh per-episode complete model, settings and provenance.

    Resolve and validate all inputs before creating the directory. A preexisting
    directory is rejected, including a previous partial attempt; no evidence is
    silently replaced. A failed write leaves that partial attempt inspectable.
    """
    _, model_bytes, settings, files, manifest = _prepared_document(repository)
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    for name, content in ((MODEL_FILENAME, model_bytes), ('attitude_control.json', settings),
            ('model_manifest.json', (json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode())):
        with (destination/name).open('xb') as stream:
            stream.write(content)
    return destination/MODEL_FILENAME, files, deepcopy(manifest)


def verify_capture_model(repository: Path, destination: Path):
    """Purely read and compare source/assets/manifest and the entire prepared tree.

    This binds static inputs only. The episode must additionally bind the model
    hash actually loaded by LocalMujocoStepper, its build, and runtime evidence.
    """
    expected_tree, expected_bytes, settings, _, expected = _prepared_document(repository)
    destination = Path(destination).resolve()
    actual = _content(destination/MODEL_FILENAME)
    manifest = json.loads((destination/'model_manifest.json').read_text('utf-8'))
    if _semantic(ET.fromstring(actual)) != _semantic(expected_tree):
        raise ValueError('prepared capture26 physical tree or resource paths changed')
    if actual != expected_bytes or manifest != expected:
        raise ValueError('prepared capture26 byte identity or provenance changed')
    if _content(destination/'attitude_control.json') != settings:
        raise ValueError('prepared capture26 attitude hardware settings changed')
    return deepcopy(expected)
