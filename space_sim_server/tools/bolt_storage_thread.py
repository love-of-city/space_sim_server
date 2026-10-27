"""Add a physical female thread to the delivered bolt storage bore.

The original target SDF and male SDF remain unchanged. The storage liner uses
the same right-handed, two-start 16-mm-lead thread. Its installation phase is
derived from the initial male pose rather than moving the freely jointed bolt.
"""
import copy
import math

import numpy as np

from tools.captive_fastener import body_frames, quaternion_product, rotation

LEAD_M = .016
PITCH_M = .008
MALE_LENGTH_M = .060
FEMALE_LENGTH_M = .020


def attach_bolt_thread_mounts(root, modules):
    """Keep both SDF liners on the independent bores moved by the builder."""
    parents = {child: parent for parent in root.iter() for child in parent}
    for geom_name, module_name in (('board_thread_sdf', 'bolt_assembly_hole'),
                                   ('bolt_storage_thread_sdf', 'bolt_storage_hole')):
        thread = root.find(f'.//geom[@name="{geom_name}"]')
        if thread is not None and parents[thread] is not modules[module_name]:
            parents[thread].remove(thread)
            modules[module_name].append(thread)
    return ensure_bolt_storage_thread(root)


def ensure_bolt_storage_thread(root):
    """Give every full-platform task the same physical storage thread.

    Reduced models without the delivered male SDF are outside this migration.
    An existing liner must belong to its bore's installation unit, so a stale
    randomized layout cannot silently leave the thread at the original hole.
    """
    male = root.find('.//body[@name="bolt"]/geom[@name="bolt_thread_sdf"]')
    if male is None:
        return None
    storage = root.find('.//geom[@name="bolt_storage_thread_sdf"]')
    if storage is None:
        if root.find('./contact/pair[@name="physical_bolt_storage_thread"]') is not None:
            raise ValueError('Bolt storage contact pair references a missing female thread')
        return dict(status='installed', **apply_bolt_storage_thread(root))
    site = root.find('.//site[@name="bolt_storage_hole"]')
    parents = {child: parent for parent in root.iter() for child in parent}
    if site is None or parents[storage] is not parents[site]:
        raise ValueError('Bolt storage thread must move with its installation bore; regenerate the layout')
    pairs = root.findall('./contact/pair[@name="physical_bolt_storage_thread"]')
    if (len(pairs) != 1 or pairs[0].get('geom1') != 'bolt_thread_sdf'
            or pairs[0].get('geom2') != 'bolt_storage_thread_sdf'):
        raise ValueError('Bolt storage thread is missing its physical contact pair')
    target = root.find('.//geom[@name="board_thread_sdf"]')
    if (target is None or storage.get('type') != 'sdf'
            or storage.get('mesh') != target.get('mesh')
            or storage.find('plugin') is None or target.find('plugin') is None
            or storage.find('plugin').attrib != target.find('plugin').attrib):
        raise ValueError('Bolt storage thread does not use the delivered female SDF')
    return dict(status='present', model='sdf_contact', male_geom=male.get('name'),
                female_geom=storage.get('name'), site=site.get('name'),
                contact_pair=pairs[0].get('name'), runtime_state_writes=False)


def apply_bolt_storage_thread(root):
    """Install one SDF contact pair; do not add joints or change initial state."""
    if root.find('.//geom[@name="bolt_storage_thread_sdf"]') is not None:
        raise ValueError('Bolt storage thread is already installed')
    site = root.find('.//site[@name="bolt_storage_hole"]')
    target = root.find('.//geom[@name="board_thread_sdf"]')
    male = root.find('.//body[@name="bolt"]/geom[@name="bolt_thread_sdf"]')
    if any(node is None for node in (site, target, male)):
        raise ValueError('Expected delivered bolt and both installation bores')
    parents = {child: parent for parent in root.iter() for child in parent}
    board = parents[site]
    if board.tag != 'body':
        raise ValueError('Storage bore site must belong to a body')
    frames = body_frames(root)
    bp, bq = frames[board.get('name')]
    mp, mq = frames['bolt']
    br, mr = rotation(bq), rotation(mq)
    site_position = np.fromstring(site.get('pos', '0 0 0'), sep=' ')
    fp = bp + br @ site_position
    site_quat = np.fromstring(site.get('quat', '1 0 0 0'), sep=' ')
    fr = br @ rotation(site_quat)
    mp = mp + mr @ np.fromstring(male.get('pos', '0 0 0'), sep=' ')
    mr = mr @ rotation(np.fromstring(male.get('quat', '1 0 0 0'), sep=' '))
    relative = fr.T @ mr
    displacement = fr.T @ (mp-fp)
    if (np.linalg.norm(displacement[:2]) > 1e-9
            or not np.allclose(relative[:, 2], [0, 0, 1], atol=1e-9)):
        raise ValueError('Storage bolt and bore must initially be coaxial')
    male_yaw = math.atan2(relative[1, 0], relative[0, 0])
    # Both delivered SDF modes have +4-mm phase. For u=z-h*theta+phase,
    # female installation yaw = male yaw - axial origin offset/h (mod pi).
    yaw = male_yaw - displacement[2] * (2*math.pi/LEAD_M)
    yaw = (yaw + math.pi/2) % math.pi - math.pi/2
    quat = quaternion_product(site_quat, [math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
    storage = copy.deepcopy(target)
    storage.set('name', 'bolt_storage_thread_sdf')
    storage.set('pos', site.get('pos'))
    storage.set('quat', ' '.join(format(float(v), '.17g') for v in quat))
    # Geoms precede child bodies to preserve XML/compiled renderer indexing.
    index = next((i for i, node in enumerate(board) if node.tag == 'body'), len(board))
    board.insert(index, storage)
    target_pair = root.find('./contact/pair[@name="physical_two_start_thread"]')
    if target_pair is None:
        raise ValueError('Missing physical target thread contact parameters')
    pair = copy.deepcopy(target_pair)
    pair.set('name', 'physical_bolt_storage_thread')
    pair.set('geom2', storage.get('name'))
    root.find('contact').append(pair)
    male_bottom = float(displacement[2]-MALE_LENGTH_M)
    overlap = max(0., min(float(displacement[2]), 0.) - max(male_bottom, -FEMALE_LENGTH_M))
    travel = max(0., -male_bottom)
    return {'model': 'sdf_contact', 'male_geom': male.get('name'),
            'female_geom': storage.get('name'), 'site': site.get('name'),
            'contact_pair': pair.get('name'), 'lead_m_per_turn': LEAD_M,
            'pitch_m': PITCH_M, 'starts': 2, 'handedness': 'right',
            'female_installation_yaw_rad': yaw,
            'initial_male_origin_above_bore_m': float(displacement[2]),
            'initial_overlap_m': overlap, 'axial_travel_to_disengage_m': travel,
            'turns_to_disengage': travel/LEAD_M,
            'added_joints': 0, 'runtime_state_writes': False,
            'calibration': 'source thread geometry/contact parameters reused; not measured hardware friction'}
