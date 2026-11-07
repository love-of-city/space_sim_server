"""Capture model preparation: preserve physics and restore original CAD visuals.

The force/torque site is at the physical capture gripper/last-link interface.
The operating-arm wrist sensor remains present and is never relabeled.
"""
from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

REFERENCE_MODEL = Path(__file__).resolve().parents[2] / 'model/SARM/platform/sarm_dual_capture26.xml'


def restore_capture_sun_sensor_mount(root):
    """Restore the omitted CAD mount in the saved, scaled operating scene.

    Keep the source scene untouched. Two surviving members establish and check
    the assembly's scale/translation before adding the original visual geom.
    Its original zero mass and disabled collision flags are preserved.
    """
    target = root.find('.//body[@name="capture_target"]')
    mount_name = 'target_instance_0077_part_034_color_00'
    if target is None or target.find(f'geom[@name="{mount_name}"]') is not None:
        return
    anchor_name = 'target_instance_0078_part_035_color_00'
    witness_name = 'target_instance_0079_part_036_color_00'
    anchor = target.find(f'geom[@name="{anchor_name}"]')
    if anchor is None:
        return
    reference = ET.parse(REFERENCE_MODEL).getroot()
    original = reference.find('.//body[@name="capture_target"]')
    original_anchor = original.find(f'geom[@name="{anchor_name}"]')
    old_mesh = reference.find(f'./asset/mesh[@name="{original_anchor.get("mesh")}"]')
    new_mesh = root.find(f'./asset/mesh[@name="{anchor.get("mesh")}"]')
    if new_mesh is None:
        raise ValueError('Sun sensor anchor mesh missing')
    vector = lambda element, key, default: np.fromstring(element.get(key, default), sep=' ')
    ratio = vector(new_mesh, 'scale', '1 1 1') / vector(old_mesh, 'scale', '1 1 1')
    if not np.isfinite(ratio).all() or ratio[0] <= 0 or not np.allclose(ratio, ratio[0]):
        raise ValueError('Sun sensor restoration requires a uniform assembly scale')
    scale = float(ratio[0])
    anchor_position = vector(anchor, 'pos', '0 0 0')
    original_position = vector(original_anchor, 'pos', '0 0 0')
    for name in (anchor_name, witness_name):
        before = original.find(f'geom[@name="{name}"]')
        after = target.find(f'geom[@name="{name}"]')
        if (after is None or
                not np.allclose(vector(after, 'pos', '0 0 0'), anchor_position +
                                scale * (vector(before, 'pos', '0 0 0') - original_position),
                                rtol=0, atol=1e-10) or
                not np.allclose(vector(after, 'quat', '1 0 0 0'),
                                vector(before, 'quat', '1 0 0 0'), rtol=0, atol=1e-10)):
            raise ValueError('Sun sensor assembly transform differs from the original CAD')
    mount = deepcopy(original.find(f'geom[@name="{mount_name}"]'))
    mesh = root.find(f'./asset/mesh[@name="{mount.get("mesh")}"]')
    if mesh is None:
        raise ValueError('Original sun sensor mount mesh missing')
    scaled_name = 'capture_target_scale_' + mesh.get('name')
    scaled_mesh = deepcopy(mesh)
    scaled_mesh.set('name', scaled_name)
    scaled_mesh.set('scale', ' '.join(format(v * scale, '.17g')
                                    for v in vector(mesh, 'scale', '1 1 1')))
    existing = root.find(f'./asset/mesh[@name="{scaled_name}"]')
    if existing is None:
        root.find('asset').append(scaled_mesh)
    elif existing.attrib != scaled_mesh.attrib:
        raise ValueError('Sun sensor mount mesh conflicts with restored scale')
    mount.set('mesh', scaled_name)
    mount.set('pos', ' '.join(format(v, '.17g') for v in anchor_position + scale *
                             (vector(mount, 'pos', '0 0 0') - original_position)))
    target.append(mount)


def prepare_capture_model(source, output):
    source, output = Path(source).resolve(), Path(output)
    root = ET.parse(source).getroot()
    restore_capture_sun_sensor_mount(root)
    # A saved preheld scene is the handoff reference. Capture starts separated,
    # and must acquire/transport the payload with real contacts throughout.
    lock = root.find('./equality/weld[@name="preheld_capture_lock"]')
    if lock is not None: lock.set('active','false')
    compiler = root.find('compiler')
    for key in ('meshdir','texturedir','assetdir'):
        if compiler is not None and compiler.get(key):
            compiler.set(key,str((source.parent/compiler.get(key)).resolve()))
    # Files without a compiler directory must also remain relocatable.
    dirs = compiler.attrib if compiler is not None else {}
    for element in root.iter():
        if element.get('file'):
            directory = dirs.get('meshdir' if element.tag == 'mesh' else 'texturedir' if element.tag == 'texture' else 'assetdir','')
            element.set('file',str((source.parent/directory/element.get('file')).resolve()))
    gripper = root.find('.//body[@name="gripper"]')
    if gripper is None: raise ValueError('capture gripper missing')
    if gripper.find('site[@name="capture_wrist_ft_site"]') is None:
        ET.SubElement(gripper,'site',name='capture_wrist_ft_site',pos='0 0 0',size='.001',rgba='0 0 0 0')
        sensors = root.find('sensor')
        if sensors is None: sensors = ET.SubElement(root,'sensor')
        ET.SubElement(sensors,'force',name='capture_wrist_ft_force',site='capture_wrist_ft_site')
        ET.SubElement(sensors,'torque',name='capture_wrist_ft_torque',site='capture_wrist_ft_site')
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)
    companion = source.with_name('attitude_control.json')
    if companion.exists() and companion.resolve() != output.with_name(companion.name).resolve():
        shutil.copy2(companion,output.with_name(companion.name))
    return output
