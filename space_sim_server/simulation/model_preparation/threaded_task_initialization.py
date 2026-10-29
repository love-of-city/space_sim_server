"""Legal episode-only MJCF initial state for the hinged locking-rod cycle.

Joint reference coordinates and matching body rest transforms express the
closed state in qpos0. No simulation state, actuator or equality is overwritten.
Other tasks retain the delivered loose state and original hinge limits.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import math
import xml.etree.ElementTree as ET

import numpy as np

from simulation.model_preparation.captive_fastener import LEAD, quaternion_product, rotation

LOCKED_TRAVEL_M = .00479054  # ~1-um first seating contact, without a large preload impulse
LOCKING_ROD_HINGE_RANGE_RAD = (-1.75, .15)
MARKER = 'scripted_locking_cycle_initial_state'


def apply_locking_cycle_initial_state(root):
    travel = LOCKED_TRAVEL_M
    spin = travel*(2*math.pi/LEAD)
    existing = root.find(f'./custom/numeric[@name="{MARKER}"]')
    if existing is not None:
        nut = root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
        slide = nut.find('joint[@name="fastener_slide"]') if nut is not None else None
        hinge = nut.find('joint[@name="fastener_spin"]') if nut is not None else None
        rod = root.find('.//joint[@name="locking_rod_hinge"]')
        if slide is None or hinge is None or rod is None:
            raise ValueError('Initial-state marker has no matching captive mechanism')
        previous = np.fromstring(existing.get('data', ''), sep=' ')
        previous_travel, previous_spin = float(slide.get('ref', '0')), float(hinge.get('ref', '0'))
        if (len(previous) != 4
                or not np.allclose(previous[:2], [previous_travel, previous_spin], atol=1e-12, rtol=0)
                or not math.isclose(previous_spin, previous_travel*2*math.pi/LEAD, abs_tol=1e-12)):
            raise ValueError('Initial-state marker disagrees with captive joint references')
        migrated = previous_travel != travel or previous_spin != spin
        if migrated:
            # Migrate an older task variant by another legal screw rest transform.
            # Runtime qpos is never changed, and zero still restores the same loose pose.
            position = np.fromstring(nut.get('pos', '0 0 0'), sep=' ')
            quat = np.fromstring(nut.get('quat', '1 0 0 0'), sep=' ')
            position += rotation(quat)[:, 2]*(travel-previous_travel)
            delta = spin-previous_spin
            quat = quaternion_product(quat, [math.cos(delta/2), 0, 0, math.sin(delta/2)])
            nut.set('pos', ' '.join(format(float(v), '.17g') for v in position))
            nut.set('quat', ' '.join(format(float(v), '.17g') for v in quat))
            slide.set('ref', format(travel, '.17g'))
            hinge.set('ref', format(spin, '.17g'))
        rod.set('range', ' '.join(format(v, '.17g') for v in LOCKING_ROD_HINGE_RANGE_RAD))
        existing.set('data', ' '.join(format(v, '.17g')
                                    for v in (travel, spin, *LOCKING_ROD_HINGE_RANGE_RAD)))
        from simulation.model_preparation.locking_cycle_contacts import apply_locking_cycle_contacts
        contacts = apply_locking_cycle_contacts(root)
        return {'model': 'captive', 'slide_m': travel, 'spin_rad': spin,
                'hinge_range_rad': list(LOCKING_ROD_HINGE_RANGE_RAD),
                'source': 'migrated_episode_initial_state' if migrated else 'existing_episode_initial_state',
                'runtime_state_writes': False,
                'physical_contacts': contacts}
    nut = root.find('.//body[@name="locking_rod"]/body[@name="fastener"]')
    if nut is None:
        raise ValueError('locking_rod_cycle requires the captive fastener model')
    slide = nut.find('joint[@name="fastener_slide"]')
    hinge = nut.find('joint[@name="fastener_spin"]')
    rod = root.find('.//joint[@name="locking_rod_hinge"]')
    if slide is None or hinge is None or rod is None:
        raise ValueError('Missing captive fastener or rod coordinates')
    if any(abs(float(joint.get('ref', '0'))) > 1e-12 for joint in (slide, hinge)):
        raise ValueError('Unrecognized preexisting fastener joint references')
    if not 0 < travel < float(slide.get('range').split()[1]):
        raise ValueError('Initial tightening travel exceeds captive thread stroke')
    original_position = np.fromstring(nut.get('pos', '0 0 0'), sep=' ')
    original_quat = np.fromstring(nut.get('quat', '1 0 0 0'), sep=' ')
    axis = np.fromstring(slide.get('axis'), sep=' ')
    spin_axis = np.fromstring(hinge.get('axis'), sep=' ')
    if not np.allclose(axis, [0, 0, 1]) or not np.allclose(spin_axis, axis):
        raise ValueError('Expected coaxial +Z captive slide and spin')
    position = original_position + rotation(original_quat) @ (axis*travel)
    quat = quaternion_product(original_quat, [math.cos(spin/2), 0, 0, math.sin(spin/2)])
    nut.set('pos', ' '.join(format(float(v), '.17g') for v in position))
    nut.set('quat', ' '.join(format(float(v), '.17g') for v in quat))
    slide.set('ref', format(travel, '.17g'))
    hinge.set('ref', format(spin, '.17g'))
    rod.set('range', ' '.join(format(v, '.17g') for v in LOCKING_ROD_HINGE_RANGE_RAD))
    custom = root.find('custom')
    if custom is None:
        custom = ET.SubElement(root, 'custom')
    ET.SubElement(custom, 'numeric', name=MARKER,
                  data=' '.join(format(v, '.17g') for v in (travel, spin, *LOCKING_ROD_HINGE_RANGE_RAD)))
    from simulation.model_preparation.locking_cycle_contacts import apply_locking_cycle_contacts
    contacts = apply_locking_cycle_contacts(root)
    return {'model': 'captive', 'slide_m': travel, 'spin_rad': spin,
            'hinge_range_rad': list(LOCKING_ROD_HINGE_RANGE_RAD),
            'source': 'MJCF_joint_ref_and_matching_rest_transform',
            'runtime_state_writes': False,
            'physical_contacts': contacts,
            'calibration': 'nominal seated travel; final tightening remains contact-driven'}
