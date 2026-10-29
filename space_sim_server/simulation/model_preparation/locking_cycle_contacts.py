"""Restore the snap-clip interface and remove a proven coarse-hull false wall.

The clip contact compliance is an uncalibrated elastic snap-fit approximation.
Precision base hulls partition the original CAD material, retaining geometry,
collision masks, mass and inertia; they do not carve out a robot clearance.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
PRECISION_DIRECTORY = ROOT/'model/SARM/platform/mechanism_collision'
CLIP_SOLREF = '.03 1'
CLIP_SOLIMP = '.9 .95 .001'


def apply_locking_cycle_contacts(root):
    contact = root.find('contact')
    rod = root.find('.//body[@name="locking_rod"]')
    clip = root.find('.//body[@name="limit_clip"]')
    if any(node is None for node in (contact, rod, clip)):
        raise ValueError('Expected original locking rod and snap clip')
    pairs = 0
    for first in rod.findall('geom'):
        if first.get('contype') != '1':
            continue
        for second in clip.findall('geom'):
            if second.get('contype') != '1':
                continue
            name = f'locking_clip_contact_{pairs:04d}'
            attributes = dict(name=name, geom1=first.get('name'), geom2=second.get('name'),
                              condim='3', friction='.3 .3 .005 .0001 .0001',
                              solref=CLIP_SOLREF, solimp=CLIP_SOLIMP)
            existing = contact.find(f'pair[@name="{name}"]')
            if existing is None:
                ET.SubElement(contact, 'pair', **attributes)
            else:
                existing.attrib.update(attributes)
            pairs += 1
    original = root.find('.//geom[@name="task_box_base_geom_collision_016"]')
    prior_precision = [geom for geom in root.findall('.//geom')
                       if geom.get('name', '').startswith('locking_precision_base_')]
    if original is None and prior_precision:
        original = prior_precision[0]
    replacement_count = 0
    if original is not None:
        manifest = json.loads((PRECISION_DIRECTORY/'locking_sweep_base.json').read_text())
        parents = {child: parent for parent in root.iter() for child in parent}
        parent = parents[original]
        index = list(parent).index(original)
        assets = root.find('asset')
        for i, filename in enumerate(manifest['files']):
            path = PRECISION_DIRECTORY/filename
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh_name = f'locking_precision_base_{i:03d}_mesh'
            asset = assets.find(f'mesh[@name="{mesh_name}"]')
            if asset is None:
                ET.SubElement(assets, 'mesh', name=mesh_name, file=path.as_posix())
            else:
                asset.set('file', path.as_posix())
            geom = copy.deepcopy(original)
            geom.set('name', f'locking_precision_base_{i:03d}')
            geom.set('mesh', mesh_name)
            parent.insert(index+i, geom)
            replacement_count += 1
        for previous in dict.fromkeys([original, *prior_precision]):
            parents[previous].remove(previous)
    return {'clip_explicit_contact_pairs': pairs,
            'clip_solref': CLIP_SOLREF, 'clip_solimp': CLIP_SOLIMP,
            'clip_compliance_calibration': 'nominal elastic snap fit; not measured material stiffness',
            'reason_for_explicit_pairs': 'original rod/base exclusion also suppresses welded clip body',
            'base_false_hull_replacement_count': replacement_count,
            'collision_masks_disabled': False, 'visual_geometry_changed': False}
